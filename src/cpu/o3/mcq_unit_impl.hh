#ifndef __CPU_O3_MCQ_UNIT_IMPL_HH__
#define __CPU_O3_MCQ_UNIT_IMPL_HH__

#include "arch/generic/debugfaults.hh"
#include "arch/locked_mem.hh"
#include "arch/arm/tlb.hh"
#include "base/str.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/o3/mcq.hh"
#include "cpu/o3/mcq_unit.hh"
#include "debug/Activity.hh"
#include "debug/IEW.hh"
#include "debug/MCQUnit.hh"
#include "debug/MCQCheck.hh"
#include "debug/O3PipeView.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

#define BLOCK224 64
#define bits(b, n, m) ((b << (63-n)) >> (63-(n-m)))
#define bit(b, n) ((b << (63-n)) >> 63)

template <class Impl>
bool
MCQUnit<Impl>::recvTimingResp(PacketPtr pkt)
{
    auto senderState = dynamic_cast<MCQSenderState*>(pkt->senderState);
    MCQRequest* req = senderState->request();
    assert(req != nullptr);
    bool ret = true;
    /* Check that the request is still alive before any further action. */
    if (senderState->alive()) {
        ret = req->recvTimingResp(pkt);
    } else {
        senderState->outstanding--;
    }
    return ret;

}

template<class Impl>
void
MCQUnit<Impl>::writebackStores()
{
    using namespace TheISA;
    // Make sure that a store exists.

    Fault fault = NoFault;

    auto iter = mchkQueue.begin();
    while (storesToWB > 0 &&
            iter->valid() &&
            mcq->cachePortAvailable(false)) {

        assert(iter->inst);

        if (!iter->instruction()->isBndInst()) {
            iter++;
            continue;
        }

        if (!iter->canWB() || iter->state != ST3)
            break;

        DynInstPtr inst = iter->inst;

        unsigned int size = 8;
        Addr addr = iter->bndAddr + (iter->subIndex * 8);
        Request::Flags flags = TLB::MustBeOne|2|TLB::AllowUnaligned;

        DPRINTF(MCQUnit,"ExecuteStore(%s) bndAddr: %lu lbndData: %lu ubndData: %lu PAC: %lu "
                            "index: %lu count: %lu\n",
                            (iter->instruction()->isBndstr() ? "bndstr" : "bndclr"),
                            addr, iter->lbndData, iter->ubndData,
                            iter->PAC, iter->index, iter->count);

        assert(!iter->completed());
        assert(!iter->inst->isSquashed());
        assert(!iter->request());

        fault = mcq->pushRequest(
                    inst,
                    /* ld */ false, (uint8_t *)iter->data(), size, addr, flags, nullptr, nullptr);
        assert(fault == NoFault && "Faulted store create found\n");

        MCQRequest* req = iter->request();

        assert(iter->hasRequest());
        assert(!iter->committed());

        assert(req->_size == 8);

        if (!inst->memData)
            inst->memData = new uint8_t[req->_size];

        memcpy(inst->memData, iter->data(), req->_size);

        if (req->senderState() == nullptr) {
            MQSenderState *state = new MQSenderState(iter);
            state->isLoad = false;
            state->needWB = false;
            state->inst = inst;
            req->senderState(state);
        }

        req->buildPackets();
        req->_packets.back()->isBndStore = true; //

        DPRINTF(MCQUnit, "D-Cache: Writing back store idx:%i PC:%s "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                iter.idx(), inst->pcState(),
                req->request()->getPaddr(), (int)*(inst->memData),
                inst->seqNum);

        /* Send to cache */
        req->sendPacketToCache();
        if (!req->isSent()) {
            iter->setRequest(nullptr);
            req->discard();
            DPRINTF(MCQUnit, "Store is not sent!\n");
            break;
        } else {
            DPRINTF(MCQUnit, "Store is sent!\n");
            --storesToWB;
            iter->completed() = true;
            iter->state = ST5;

            // Check load-store reordering violation
            auto load_it = iter;
            while (load_it != mchkQueue.end()) {
                load_it++;

                if (load_it == mchkQueue.end())
                    break;

                if (load_it->PAC == iter->PAC &&
                    (load_it->state == ST1 ||
                      load_it->state == ST2 ||
                      load_it->state == ST3 ||
                      load_it->state == ST4)) {

                    if (load_it->instruction()->isBndInst())
                        mcqReplayedBoundStore++;
                    else
                        mcqReplayedBoundCheck++;

                    DPRINTF(MCQUnit, "Need replay for this inst [sn:%lli]\n",
                            load_it->instruction()->seqNum);
                    load_it->needReplay = true;

                    if (load_it->state == ST3) {
                        if (load_it->canWB()) {
                            load_it->canWB() = false;
                            storesToWB--;
                        }

                        load_it->state = ST4;
                        readyInsts.push_back(load_it->instruction());
                    } else if (load_it->state == ST4 &&
                        load_it->count >= btNumWays) {
                        readyInsts.push_back(load_it->instruction());
                    }

                    load_it->count = 0;
                }
            }
        }
        iter++;
    }

    for (auto& x : mchkQueue) {
        assert(x.valid());

        if (x.completed()) {
            mcqCommittedInsts++;

            if (x.instruction()->isBndstr()) {
                mcqCommittedBndstr++;
            } else if (x.instruction()->isBndclr()) {
                mcqCommittedBndclr++;
            } else if (x.isSigned) {
                if (x.instruction()->isLoad()) {
                    mcqCommittedSignedLoad++;
                } else {
                    mcqCommittedSignedStore++;
                }
            } else {
                if (x.instruction()->isLoad()) {
                    mcqCommittedUnsignedLoad++;
                } else {
                    mcqCommittedUnsignedStore++;
                }
            }

            DPRINTF(MCQUnit, "Committing head instruction in mchkQueue PC %s"
                    "[sn:%lli]\n",
                    x.instruction()->pcState(),
                    x.instruction()->seqNum);

            mchkQueue.front().clear(); // free?
            mchkQueue.pop_front();
            insts--;
        } else {
            break;
        }
    }

}

template<class Impl>
void
MCQUnit<Impl>::commitInsts(InstSeqNum &youngest_inst)
{
    /* Forward iterate the mchk queue (age order). */
    for (auto& x : mchkQueue) {
        assert(x.valid());
        // Mark any stores that are now committed and have not yet
        // been marked as able to write back.

        if (x.instruction()->seqNum > youngest_inst)
            break;

        auto iter = mchkQueue.getIterator(x.instruction()->mcqIdx);

        if (x.count >= btNumWays && iter == mchkQueue.begin()) {
            if (x.instruction()->isBndstr() ) {
                printf("### Occupancy check failed for bndstr: %lu [sn:%lu]", x.PAC, x.inst_org->seqNum);
                assert(false && "Bounds table resizing should have occurred.");
            } else if (x.instruction()->isBndclr()) {
                printf("### Occupancy check failed for bndclr: PAC: %lu addr: %lu [sn:%lu]", x.PAC, x.effAddr, x.inst_org->seqNum);
                x.instruction()->dump();
                x.state = ST5;
								numBndClrFailure++;
            } else {
                assert(!x.inst_org->isSquashed());
                printf("### Bounds check failed: PAC: %lu [sn:%lu] ", x.PAC, x.inst_org->seqNum);
                x.instruction()->dump();
                x.inst_org->setValidated(true);
                x.state = ST5;
								numBndChkFailure++;
            }
        }

        if (x.state == ST5) {
            // Pop
            if (iter == mchkQueue.begin()) {
                mcqCommittedInsts++;

                for (uint8_t i=0; i<iter->count; i++)
                    mcqNumCounterInc++;

                if (iter->instruction()->isBndstr()) {
                    mcqCommittedBndstr++;
                } else if (iter->instruction()->isBndclr()) {
                    mcqCommittedBndclr++;
                } else if (iter->isSigned) {
                    if (iter->instruction()->isLoad())
                        mcqCommittedSignedLoad++;
                    else
                        mcqCommittedSignedStore++;
                } else {
                    if (iter->instruction()->isLoad())
                        mcqCommittedUnsignedLoad++;
                    else
                        mcqCommittedUnsignedStore++;
                }

                if (iter->needUpdate && (iter->instruction()->isBndInst() ||
                    iter->isSigned)) {

                    DPRINTF(MCQUnit, "Update BWB, PAC: %lu Hash: %lu Index: %lu [sn:%lli]\n",
                            iter->PAC, iter->hash, (iter->index + iter->count) % btNumWays,
                            iter->instruction()->seqNum);

                    // Update BWB
                    bwb.update(iter->PAC, iter->hash, (iter->index + iter->count) % btNumWays, iter->instruction()->isBndclr());
                }

                DPRINTF(MCQUnit, "Committing head instruction in mchkQueue PC %s"
                        "[sn:%lli]\n",
                        iter->instruction()->pcState(),
                        iter->instruction()->seqNum);

                mchkQueue.front().clear(); // free?
                mchkQueue.pop_front();
                insts--;
            } else {
                DPRINTF(MCQUnit, "Tried to commit, but not head instruction in mchkQueue PC %s"
                        "[sn:%lli]\n",
                        iter->instruction()->pcState(), iter->instruction()->seqNum);
            }
        } else if (x.state == ST3 && !x.canWB()) {
             DPRINTF(MCQUnit, "This bound store inst can now write back PC %s [sn:%lli]\n",
                    iter->instruction()->pcState(), iter->instruction()->seqNum);
            x.canWB() = true;
            ++storesToWB;
        }

        if (mchkQueue.empty())
            break;
    }
}

template<class Impl>
void
MCQUnit<Impl>::completeDataAccess(PacketPtr pkt)
{
    MCQSenderState *state = dynamic_cast<MCQSenderState *>(pkt->senderState);
    DynInstPtr inst = state->inst;

    /* Notify the sender state that the access is complete (for ownership
     * tracking). */
    state->complete();

    if (!state->isLoad)
        return;

    auto iter = mchkQueue.getIterator(inst->mcqIdx);

    assert(!cpu->switchedOut());
    if (!inst->isSquashed()) {
        Addr addr = iter->effAddr;
        uint8_t effSize = iter->effSize;
        bool succeed = false;

        for (int i=0; i<4; i++) {
            Addr bndData = *(pkt->getPtr<uint64_t>() + i);
            Addr lbndData = (bndData & 0xFFFFFFFF);
            Addr ubndData = lbndData + (bndData >> 32);
						bool isLoad = inst->isLoad();

						//inst->dump();
            DPRINTF(MCQUnit, "CompleteAccess check, addr: %lu effSize: %u lbndData: %lu "
                   "ubndData: %lu for inst [sn:%lu]\n",
                    addr, effSize, lbndData, ubndData, inst->seqNum);

            switch (iter->state) {
              case ST1:
                if (inst->isNeonLoad() && neonBoundCheck(addr, effSize, lbndData, ubndData)) {
                    Addr neon_lbndData = lbndData - (lbndData % 64);
                    Addr neon_ubndData = ubndData - (ubndData % 64) + 64;

                    DPRINTF(MCQCheck, "Passed neonBoundCheck! bndAddr: %lu addr: %lu effSize: %u "
                            "neon_lbndData: %lu neon_ubndData: %lu index: %lu count: %u PAC: %lu Hash: %lu [sn:%lli]\n",
                            iter->bndAddr, addr, effSize, neon_lbndData, neon_ubndData, iter->index, iter->count,
                            iter->PAC, iter->hash, iter->inst_org->seqNum);
                    iter->inst_org->setValidated(true); // signal to ROB
                    iter->state = ST5;
                    iter->completed() = true;
                    succeed = true;
                } else if (!inst->isNeonLoad() && boundsCheck(addr, effSize, lbndData, ubndData, isLoad)) {
                    DPRINTF(MCQCheck, "Passed boundsCheck! bndAddr: %lu addr: %lu effSize: %u "
                            "lbndData: %lu ubndData: %lu index: %lu count: %u PAC: %lu Hash: %lu [sn:%lli]\n",
                            iter->bndAddr, addr, effSize, lbndData, ubndData, iter->index, iter->count,
                            iter->PAC, iter->hash, iter->inst_org->seqNum);
                    iter->inst_org->setValidated(true); // signal to ROB
                    iter->state = ST5;
                    iter->completed() = true;
                    succeed = true;
                }

                break;

              case ST2:
                if (!iter->needReplay && occupancyCheck(addr, lbndData, ubndData, inst->isBndstr())) {
                    if (iter->instruction()->isBndstr()) {
                        DPRINTF(MCQCheck, "Passed bndstr! bndAddr: %lu PAC: %lu addr: 0x%lx size: %lu bndData: 0x%lx subIndex: %lu [sn:%lli]\n",
																iter->bndAddr, iter->PAC, iter->effAddr, iter->ubndData - iter->lbndData, iter->bndData, i, iter->inst_org->seqNum);
										} else {
                        DPRINTF(MCQCheck, "Passed bndclr! bndAddr: %lu PAC: %lu addr: 0x%lx lbndData: 0x%lx ubndData: 0x%lx subIndex: %lu [sn:%lli]\n",
																iter->bndAddr, iter->PAC, iter->effAddr, lbndData, ubndData, i, iter->inst_org->seqNum);
										}
                    iter->inst_org->setCanCommit();
                    iter->state = ST3;
                    iter->subIndex = i;
                    succeed = true;
                }

                break;

              default:
                assert(false && "Wrong state in completeDataAccess\n");
            }

            if (succeed)
                break;
        } // for

        if (!succeed) {
            Addr bndData = *(pkt->getPtr<uint64_t>());
            Addr lbndData = (bndData & 0xFFFFFFFF);
            Addr ubndData = lbndData + (bndData >> 32);

            switch (iter->state) {
              case ST1:
                if (iter->needReplay) {
                    iter->state = ST4;
                    readyInsts.push_back(inst);
                } else {
                    DPRINTF(MCQCheck, "Failed boundsCheck! bndAddr: %lu addr: %lu effSize: %u "
                            "lbndData: %lu ubndData: %lu index: %lu count: %u PAC: %lu Hash: %lu [sn:%lli]\n",
                            iter->bndAddr, addr, effSize, lbndData, ubndData, iter->index, iter->count,
                            iter->PAC, iter->hash, iter->inst_org->seqNum);
                    iter->state = ST4;
                    readyInsts.push_back(inst);
                }

                break;
              case ST2:
                if (iter->needReplay) {
                    iter->state = ST4;
                    readyInsts.push_back(inst);
                } else {
                    DPRINTF(MCQCheck, "Failed occupancyCheck! (%s) bndAddr: %lu lbndData:"
                            " 0x%lx ubndData: 0x%lx index: %lu count: %u PAC: %lu Hash: %lu [sn:%lli]\n",
                            (iter->instruction()->isBndstr() ? "bndstr" : "bndclr"),
                            iter->bndAddr, lbndData, ubndData, iter->index, iter->count,
                            iter->PAC, iter->hash, iter->inst_org->seqNum);

                    iter->state = ST4;
                    readyInsts.push_back(inst);
                }

                break;
              default:
                assert(false && "Wrong state in completeDataAccess\n");
            }
        }
    }

    iter->request()->discard();
    iter->setRequest(nullptr);
}

template <class Impl>
MCQUnit<Impl>::MCQUnit(uint32_t mcqEntries, uint32_t bwbEntries)
    : mcqID(-1), mchkQueue(mcqEntries+1), insts(0), storesToWB(0),
      cacheBlockMask(0), stalled(false), isStoreBlocked(false),
      storeInFlight(false), hasPendingRequest(false),
      pendingRequest(nullptr)
{
  bwb.mcqUnit = this;
  bwb.maxBWBEntries = bwbEntries;
}

template<class Impl>
void
MCQUnit<Impl>::init(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params,
        MCQ *mcq_ptr, unsigned id)
{
    mcqID = id;
    cpu = cpu_ptr;
    iewStage = iew_ptr;
    mcq = mcq_ptr;
    btBaseAddr = mcq->btBaseAddr;
    btNumWays = mcq->btNumWays;

    DPRINTF(MCQUnit, "Creating MCQUnit%i object.\n", mcqID);

    resetState();
}


template<class Impl>
void
MCQUnit<Impl>::resetState()
{
    insts = storesToWB = 0;
    retryPkt = NULL;
    stalled = false;
    cacheBlockMask = ~(cpu->cacheLineSize() - 1);
}

template<class Impl>
std::string
MCQUnit<Impl>::name() const
{
    if (Impl::MaxThreads == 1) {
        return iewStage->name() + ".mcq";
    } else {
        return iewStage->name() + ".mcq.thread" + std::to_string(mcqID);
    }
}

template<class Impl>
void
MCQUnit<Impl>::regStats()
{
    mcqSquashedInsts
        .name(name() + ".squashedInsts")
        .desc("Number of insts squashed");

    mcqCacheBlocked
        .name(name() + ".cacheBlocked")
        .desc("Number of times an access to memory failed due to the cache being blocked");

    mcqCommittedInsts
        .name(name() + ".committedInsts")
        .desc("Number of insts committed");

    mcqCommittedBndstr
        .name(name() + ".committedBndstr")
        .desc("Number of bndstr insts committed");

    mcqCommittedBndclr
        .name(name() + ".committedBndclr")
        .desc("Number of bndclr insts committed");

    mcqCommittedSignedLoad
        .name(name() + ".committedSignedLoad")
        .desc("Number of signed load insts committed");

    mcqCommittedSignedStore
        .name(name() + ".committedSignedStore")
        .desc("Number of signed store insts committed");

    mcqCommittedUnsignedLoad
        .name(name() + ".committedUnsignedLoad")
        .desc("Number of unsigned load insts committed");

    mcqCommittedUnsignedStore
        .name(name() + ".committedUnsignedStore")
        .desc("Number of unsigned store insts committed");

    mcqBoundCheck
        .name(name() + ".numBoundCheck")
        .desc("Number of bound check requests");

    mcqDataFwdBoundCheck
        .name(name() + ".numDataForwardedBoundCheck")
        .desc("Number of data forwarded bound check requests");

    mcqOccupancyCheck
        .name(name() + ".numOccupancyCheck")
        .desc("Number of occupancy check requests");

    mcqDataFwdOccupancyCheck
        .name(name() + ".numDataForwardedOccupancyCheck")
        .desc("Number of data forwarded occupancy check requests");

    mcqReplayedBoundStore
        .name(name() + ".numReplayedBoundStore")
        .desc("Number of replayed bound store insts");

    mcqReplayedBoundCheck
        .name(name() + ".numReplayedBoundCheck")
        .desc("Number of replayed bound check insts");

    mcqNumCounterInc
        .name(name() + ".numCounterInc")
        .desc("Number of times of counter increase");

    mcqAvgCounterInc
        .name(name() + ".avgCounterInc")
        .desc("Average of times of counter increase");

    mcqAvgCounterInc = mcqNumCounterInc / mcqCommittedInsts;

    bwbNumHits
        .name(name() + ".bwbNumHits")
        .desc("Number of BWB hits");

    bwbNumMisses
        .name(name() + ".bwbNumMisses")
        .desc("Number of BWB misses");

    bwbNumEvictions
        .name(name() + ".bwbNumEvictions")
        .desc("Number of BWB evictions");

    mcqSmallBin
        .name(name() + ".mcqSmallBin")
        .desc("Number of small bin accesses");

    mcqMediumBin
        .name(name() + ".mcqMediumBin")
        .desc("Number of medium bin accesses");

    mcqLargeBin
        .name(name() + ".mcqLargeBin")
        .desc("Number of large bin accesses");

		numBndStrFailure
				.name(name() + ".numBndStrFailure")
				.desc("Number of bounds store failures");

		numBndClrFailure
				.name(name() + ".numBndClrFailure")
				.desc("Number of bounds clear failures");

		numBndChkFailure
				.name(name() + ".numBndChkFailure")
				.desc("Number of bounds check failures");
}


template<class Impl>
void
MCQUnit<Impl>::setDcachePort(MasterPort *dcache_port)
{
    dcachePort = dcache_port;
}

template<class Impl>
void
MCQUnit<Impl>::drainSanityCheck() const
{
    for (int i = 0; i < mchkQueue.capacity(); ++i)
        assert(!mchkQueue[i].valid());

    assert(storesToWB == 0);
}

template<class Impl>
void
MCQUnit<Impl>::takeOverFrom()
{
    resetState();
}

template <class Impl>
void
MCQUnit<Impl>::cacheUnblocked()
{
    readyInsts.splice(readyInsts.end(), blockedInsts);
    blockedInsts.clear();
}

template <class Impl>
void
MCQUnit<Impl>::insertInst(const DynInstPtr &inst)
{
    assert(!mchkQueue.full());
    assert(insts < mchkQueue.capacity());

    DPRINTF(MCQUnit, "Inserting inst PC %s, idx:%i [sn:%lli]\n",
            inst->pcState(), mchkQueue.tail(), inst->seqNum);

    DynInstPtr mchkInst = new DynInst(inst->staticInst, inst->macroop,
                    inst->pcState(), inst->predPC, inst->seqNum,
                    this->cpu);

    mchkInst->thread = inst->thread;

    /* Grow the queue. */
    mchkQueue.advance_tail();
    mchkQueue.back().set(mchkInst, inst);
    mchkInst->mcqIdx = mchkQueue.tail();
    mchkInst->mcqIt = mchkQueue.getIterator(mchkInst->mcqIdx);
    ++insts;

    notReadyInsts.push_back(mchkInst);
}

template <class Impl>
unsigned
MCQUnit<Impl>::numFreeEntries()
{
        //LQ has an extra dummy entry to differentiate
        //empty/full conditions. Subtract 1 from the free entries.
        DPRINTF(MCQUnit, "MCQ size: %d, #insts occupied: %d\n",
                1 + mchkQueue.capacity(), insts);
        return mchkQueue.capacity() - insts;
}

template <class Impl>
void
MCQUnit<Impl>::checkSnoop(PacketPtr pkt)
{
    assert(false && "checkSnoop entered\n");
}

template <class Impl>
bool
MCQUnit<Impl>::isFull()
{
    return mchkQueue.full();
}

template <class Impl>
void
MCQUnit<Impl>::squash(const InstSeqNum &squashed_num)
{
    DPRINTF(MCQUnit, "Squashing until [sn:%lli]!"
            "(insts:%i)\n", squashed_num, insts);

    while (insts != 0 &&
            mchkQueue.back().instruction()->seqNum > squashed_num) {

        DPRINTF(MCQUnit, "MCQ Instruction PC %s squashed, "
                "[sn:%lli]\n",
                mchkQueue.back().instruction()->pcState(),
                mchkQueue.back().instruction()->seqNum);

        mchkQueue.back().instruction()->setSquashed();
        mchkQueue.back().clear();
        --insts;
        mchkQueue.pop_back();
        ++mcqSquashedInsts;

        assert(!(insts != 0 && mchkQueue.empty()));
    }

    auto it = readyInsts.begin();
    while (it != readyInsts.end()) {
        if ((*it)->seqNum > squashed_num) {
            it = readyInsts.erase(it);
        }
        else
            it++;
    }

    auto ite = executeQueue.begin();
    while (ite != executeQueue.end()) {
        if ((*ite)->seqNum > squashed_num) {
            ite = executeQueue.erase(ite);
        }
        else
            ite++;
    }

    auto itb = blockedInsts.begin();
    while (itb != blockedInsts.end()) {
        if ((*itb)->seqNum > squashed_num) {
            itb = blockedInsts.erase(itb);
        }
        else
            itb++;
    } 

    auto itn = notReadyInsts.begin();
    while (itn != notReadyInsts.end()) {
        if ((*itn)->seqNum > squashed_num) {
            itn = notReadyInsts.erase(itn);
        }
        else
            itn++;
    }
}

template <class Impl>
bool
MCQUnit<Impl>::trySendPacket(bool isLoad, PacketPtr data_pkt)
{
    bool ret = true;
    bool cache_got_blocked = false;

    auto state = dynamic_cast<MCQSenderState*>(data_pkt->senderState);

    assert(data_pkt->senderState); //yh+

    if (!mcq->cacheBlocked() &&
        mcq->cachePortAvailable(isLoad)) {
        if (!dcachePort->sendTimingReq(data_pkt)) {
            ret = false;
            cache_got_blocked = true;
        }
    } else {
        ret = false;
    }

    if (ret) {
        if (!isLoad) {
            isStoreBlocked = false;
        }
        mcq->cachePortBusy(isLoad);
        state->outstanding++;
        state->request()->packetSent();
    } else {
        if (cache_got_blocked) {
            iewStage->ldstQueue.cacheBlocked(true); //yh+
            mcq->cacheBlocked(true);
            ++mcqCacheBlocked;
        }

        if (!isLoad) {
            isStoreBlocked = true;
        }
        state->request()->packetNotSent();
    }
    return ret;
}

template <class Impl>
void
MCQUnit<Impl>::recvRetry()
{
    if (isStoreBlocked) {
        DPRINTF(MCQUnit, "Receiving retry: blocked store\n");
        assert(false && "Need to check\n");
    }
}

template <class Impl>
void
MCQUnit<Impl>::dumpInsts() const
{
    cprintf("Mem check queue: Dumping instructions.\n");
    cprintf("Mem check queue size: %i\n", insts);
    cprintf("Mem check queue: ");

    for (const auto& e: mchkQueue) {
        const DynInstPtr &inst(e.instruction());
        cprintf("%s.[sn:%i] ", inst->pcState(), inst->seqNum);
    }
    cprintf("\n");
}

template <class Impl>
unsigned int
MCQUnit<Impl>::cacheLineSize()
{
    return cpu->cacheLineSize();
}

/* ==============================================
 *
 * ============================================== */
template <class Impl>
bool
MCQUnit<Impl>::isPACed(Addr effAddr)
{
    return ((effAddr & 0x600000000000) > 0) ? true : false;
}

template <class Impl>
uint64_t
MCQUnit<Impl>::getPAC(Addr effAddr)
{
    /* V: Valid, R: Reserved
      |63-56|55|54-47|46|...|
      |upper| R|lower| V|...|
    */
    Addr upper = (effAddr >> 56);
    Addr lower = (effAddr << 9) >> 56;

    uint64_t PAC = (uint64_t) ((upper << 8) + lower);

    return PAC;
}

template <class Impl>
Addr
MCQUnit<Impl>::getBndAddr(uint64_t PAC, uint64_t index, uint64_t count)
{
    int N = log2(btNumWays);
    Addr bndAddr = (Addr) (btBaseAddr + 
                      (PAC << (N + 3 + 2)) +
                      (((index + count) % (1 << N)) << (3 + 2)));

    DPRINTF(MCQUnit, "PAC: %lu index: %lu count: %lu btBaseAddr: %lu bndAddr: %lu\n",
            PAC, index, count, btBaseAddr, bndAddr);

    return bndAddr;
}

template <class Impl>
bool
MCQUnit<Impl>::isSrcRegsReady(DynInstPtr& inst)
{
    if (inst->isBndstr())
        return (inst->readyRegs == 2);
    else if (inst->isBndclr())
        return (inst->readyRegs == 1);
    else
        return inst->effAddrValid();
}

template <class Impl>
void
MCQUnit<Impl>::setMCQEntry(typename MChkQueue::iterator& iter)
{
    auto inst = iter->inst_org;
    Addr effAddr = 0;
    Addr size = 0;

    if (inst->isBndstr()) {
        effAddr = inst->readIntRegOperand(nullptr, 0);
        size = inst->readIntRegOperand(nullptr, 1);
    } else if (inst->isBndclr()) {
        effAddr = inst->readIntRegOperand(nullptr, 0);
    } else {
        effAddr = inst->effAddr_org;
    }

    iter->inst->effAddr_org = iter->inst_org->effAddr_org;
    iter->inst->effSize = iter->inst_org->effSize;
    iter->effSize = iter->inst_org->effSize;

    uint64_t PAC = getPAC(effAddr);
    uint64_t hash = getHash(effAddr, PAC);
    uint64_t index = bwb.getIndex(PAC, hash, inst->isBndstr());

    iter->PAC = PAC;
    iter->index = index;
    iter->bndAddr = getBndAddr(PAC, index, 0);
    iter->hash = hash;

    effAddr = effAddr & 0x00801FFFFFFFFFFF;
    iter->effAddr = effAddr;

    if (inst->isBndstr()) {
        iter->bndData = bits(effAddr, 31, 0);
        iter->bndData = iter->bndData + (size << 32);

        iter->lbndData = effAddr;
        iter->ubndData = effAddr + size;
        DPRINTF(MCQUnit, "PAC:%lu bndstr effAddr: %lu lbndData: %lu ubndData: %lu size: %lu\n",
            PAC, effAddr, iter->lbndData, iter->ubndData, size);

        memcpy(iter->data(), &iter->bndData, 8);
        assert(*((uint64_t*) (iter->data())) == iter->bndData);
    } else if (inst->isBndclr()) {
        DPRINTF(MCQUnit, "PAC:%lu bndclr effAddr: %lu\n", PAC, effAddr);

        iter->bndData = 0;
        memset(iter->data(), 0, 8);
    }
}

template <class Impl>
bool
MCQUnit<Impl>::neonBoundCheck(Addr addr, uint8_t effSize, Addr lbndData, Addr ubndData)
{
    // Special care for mem_neon_ld
    Addr neon_lbndData = lbndData - (lbndData % 64);
    Addr neon_ubndData = ubndData - (ubndData % 64) + 64;

    uint64_t C = 0;
    if ((bit(addr, 31) == 0) && (bit(neon_lbndData, 31) == 1))
        C = 1;

    addr = (C << 32) | bits(addr, 31, 0);

    return (addr >= neon_lbndData && addr <= neon_ubndData);
}

template <class Impl>
bool
MCQUnit<Impl>::boundsCheck(Addr addr, uint8_t effSize, Addr lbndData, Addr ubndData, bool isLoad)
{
    uint64_t C = 0;
    if ((bit(addr, 31) == 0) && (bit(lbndData, 31) == 1))
        C = 1;

    addr = (C << 32) | bits(addr, 31, 0);

    DPRINTF(MCQUnit, "boundsCheck! addr: %lu effSize: %u lbndData: %lu ubndData: %lu\n",
            addr, effSize, lbndData, ubndData);

		if (isLoad) 
	    return (addr >= lbndData && addr <= ubndData);
		else
	    return (addr >= lbndData && (addr + effSize) <= ubndData);
}

template <class Impl>
bool
MCQUnit<Impl>::occupancyCheck(Addr addr, Addr lbndData, Addr ubndData, bool empty)
{
    uint64_t C = 0;
    if ((bit(addr, 31) == 0) && (bit(lbndData, 31) == 1))
        C = 1;

    addr = (C << 32) | bits(addr, 31, 0);

    DPRINTF(MCQCheck, "occupancyCheck! addr: %lu lbndData: %lu ubndData: %lu\n",
            addr, lbndData, ubndData);

    if (empty)
        return (lbndData == 0); // bndstr
    else
        return (addr == lbndData || addr == ubndData); // bndclr
}

template <class Impl>
Fault
MCQUnit<Impl>::executeLoad(typename MChkQueue::iterator& iter)
{
    using namespace TheISA;
    // Execute a specific load.
    Fault fault = NoFault;

    DynInstPtr inst = iter->inst;
    unsigned int size = 32;
    Addr addr = iter->bndAddr;
    Request::Flags flags = TLB::MustBeOne|2|TLB::AllowUnaligned;

    DPRINTF(MCQUnit, "ExecuteLoad, addr:%lu\n", addr);

    assert(!iter->inst->isSquashed());

    fault = mcq->pushRequest(
                inst,
                /* ld */ true, nullptr, size, addr, flags, nullptr, nullptr);

    assert(fault == NoFault && "Faulted load create found\n");

    return fault;
}

template <class Impl>
void
MCQUnit<Impl>::addReadyInsts()
{
    executeQueue.splice(executeQueue.begin(), readyInsts);
    readyInsts.clear();

    auto it = notReadyInsts.begin();
    while (it != notReadyInsts.end()) {
        auto inst = mchkQueue[(*it)->mcqIdx].inst_org;

        if (isSrcRegsReady(inst)) {
            executeQueue.push_back(*it);
            it = notReadyInsts.erase(it);
        } else {
            it++;
        }
    }
}

template <class Impl>
void
MCQUnit<Impl>::execute()
{
    int inst_num = 0;
    int insts_to_execute = 8;

    addReadyInsts();

    if (executeQueue.size() < insts_to_execute)
        insts_to_execute = executeQueue.size();

    for (; inst_num < insts_to_execute; ++inst_num) {
        DynInstPtr inst = executeQueue.front();
        executeQueue.pop_front();

        //printf("In MCQ, "); inst->dump();
        auto iter = mchkQueue.getIterator(inst->mcqIdx);
        assert(iter->valid());

        switch (iter->state) {
          case ST0:
            if (inst->isBndInst()) {
                DPRINTF(MCQUnit, "Bounds inst encountered PC %s, idx:%i [sn:%lli]\n",
                        inst->pcState(), inst->mcqIdx, inst->seqNum);

                iter->inst_org->setExecuted();
                setMCQEntry(iter);
                iter->state = ST2; // go to OCCU_CHK
                readyInsts.push_back(inst);
            } else if (isPACed(iter->inst_org->effAddr_org)) {
                DPRINTF(MCQUnit, "Signed load/store inst encountered PC %s, idx:%i [sn:%lli]\n",
                        inst->pcState(), inst->mcqIdx, inst->seqNum);

                setMCQEntry(iter);
                iter->isSigned = true;
                iter->state = ST1; // go to BND_CHK
                readyInsts.push_back(inst);
            } else {
                DPRINTF(MCQUnit, "Unsigned load/store inst encountered PC %s, idx:%i [sn:%lli]\n",
                        inst->pcState(), inst->mcqIdx, inst->seqNum);
                iter->isSigned = false;
                iter->inst_org->setValidated(true); // signal to ROB
                iter->state = ST5; // go to DONE
            }

            break;

          case ST1: // BND_CHK
          case ST2: // OCCU_CHK
            if (iter->needReplay) {
                iter->state = ST4;
                readyInsts.push_back(inst);
                break;
            }

            executeLoad(iter);
            break;

          case ST4:
            if (iter->needReplay) {
                iter->needReplay = false;
                iter->count = 0;
                iter->bndAddr = getBndAddr(iter->PAC, iter->index, iter->count);
                iter->state = inst->isBndInst() ? ST2 : ST1;
                readyInsts.push_back(inst);
            } else {
                iter->count++;

                if (iter->count >= btNumWays) {
                    if (inst->isBndstr()) {
                        printf("### Occupancy check failed for bounds store inst: PAC: %lu addr: %lu [sn:%lu]",
																iter->PAC, iter->effAddr, iter->inst_org->seqNum);

                        iter->instruction()->dump();
                        iter->needReplay = true;
                        reallocBoundTable();
                        readyInsts.push_back(inst);
												numBndStrFailure++;
                    } else if (inst->isBndclr()) {
                        iter->inst_org->setCanCommit();
                        iter->state = ST3;
                    } else {
                        iter->inst_org->setValidated(true);
                    }
                } else {
                    iter->bndAddr = getBndAddr(iter->PAC, iter->index, iter->count);
                    iter->state = inst->isBndInst() ? ST2 : ST1;
                    readyInsts.push_back(inst);
                }
            }

            break;

          case ST3: // BND_STORE
          case ST5: // DONE
          default:
            printf("State: %i\n", iter->state);
            assert(false && "Wrong state found in MCQ!\n");
            break;
        }
    }
}

template <class Impl>
uint64_t
MCQUnit<Impl>::getHash(uint64_t effAddr, uint64_t PAC)
{
    uint64_t hash = 0;
    uint64_t AHC = (effAddr & 0x600000000000) >> 45;

    DPRINTF(MCQUnit, "AHC: %d\n", AHC);

    if (AHC == 1) {
        effAddr = effAddr >> 7;
        hash = (PAC << 16) + (((effAddr & 0x3FFF) << 2) + AHC);
        mcqSmallBin++;
    } else if (AHC == 2) {
        effAddr = effAddr >> 10;
        hash = (PAC << 16) + (((effAddr & 0x3FFF) << 2) + AHC);
        mcqMediumBin++;
    } else if (AHC == 3) {
        effAddr = effAddr >> 12;
        hash = (PAC << 16) + (((effAddr & 0x3FFF) << 2) + AHC);
        mcqLargeBin++;
    }

    return hash;
}

template <class Impl>
void
MCQUnit<Impl>::BWB::eviction()
{
    auto entry = m_list.back();

    m_list.pop_back();
    delete entry;
}

template <class Impl>
void
MCQUnit<Impl>::BWB::insert(uint64_t PAC, uint64_t hash, uint64_t index)
{
    BWBEntry *entry = new BWBEntry();
    entry->setPAC(PAC);
    entry->hash = hash;
    entry->index = index;

    m_list.push_front(entry);
}

template <class Impl>
void
MCQUnit<Impl>::BWB::update(uint64_t PAC, uint64_t hash, uint64_t index, bool isBndclr)
{
    auto it = m_list.begin();

    for (; it != m_list.end(); it++) {
        if ((*it)->PAC == PAC && (*it)->hash == hash) {
            (*it)->index = index;
            m_list.erase(it);
            m_list.push_front(*it);

            return;
        }
    }

    if (isFull()) {
        mcqUnit->bwbNumEvictions++;
        eviction();
    }

    insert(PAC, hash, index);
}

template <class Impl>
uint64_t
MCQUnit<Impl>::BWB::getIndex(uint64_t PAC, uint64_t hash, bool isBndstr)
{
    auto it = m_list.begin();

    // bndstr starts from index 0
    if (isBndstr) {
        return 0;
    } else {
        for (; it != m_list.end(); ++it) {
            if ((*it)->PAC == PAC && (*it)->hash == hash) {
                mcqUnit->bwbNumHits++;

                return (*it)->index;
            }
        }
    }

    mcqUnit->bwbNumMisses++;

    return 0;
}

template <class Impl>
void
MCQUnit<Impl>::reallocBoundTable()
{
    uint64_t btBaseAddr_new = mcq->btBaseAddr;
    uint64_t btSize_new = (uint64_t) (mcq->btSize << 1); // x2
    int btNumWays_new = (mcq->btNumWays << 1); // x2
    
    ThreadID tid = mcqID;

    cpu->thread[tid]->getProcessPtr()->allocateMem(mcq->btBaseAddr + mcq->btSize, mcq->btSize);

    int N = log2(mcq->btNumWays);
    int N_new = log2(btNumWays_new);

    uint8_t *buf_p = new uint8_t[32 * mcq->btNumWays];
    uint8_t *buf_zero = new uint8_t[32 * mcq->btNumWays];
    memset(buf_zero, 0, 32 * mcq->btNumWays);
 
    for (int i=65535; i>=0; i--) {
        cpu->thread[tid]->getMemProxy().readBlob(btBaseAddr + (i << (N+5)),
                                                  buf_p, 32 * mcq->btNumWays);

        cpu->thread[tid]->getMemProxy().writeBlob(btBaseAddr_new + (i << (N_new+5)),
                                                  buf_p, 32 * mcq->btNumWays);

        cpu->thread[tid]->getMemProxy().writeBlob(btBaseAddr_new + (i << (N_new+5)) + 32 * mcq->btNumWays,
                                                  buf_zero, 32 * mcq->btNumWays);
    }

    mcq->btBaseAddr = btBaseAddr_new;
    mcq->btSize = btSize_new;
    mcq->btNumWays = btNumWays_new;

    this->btBaseAddr = btBaseAddr_new;
    this->btNumWays = btNumWays_new;

    printf("[AOS] Reallocate bounds table, btBaseAddr: %lu btSize: %lu "
            "btNumWays: %d\n", btBaseAddr_new, btSize_new, btNumWays_new);

    delete[] buf_p;
}

#endif//__CPU_O3_MCQ_UNIT_IMPL_HH__

