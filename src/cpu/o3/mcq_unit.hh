#ifndef __CPU_O3_MCQ_UNIT_HH__
#define __CPU_O3_MCQ_UNIT_HH__

#include <algorithm>
#include <cstring>
#include <map>
#include <queue>
#include <list>

#include "arch/generic/debugfaults.hh"
#include "arch/isa_traits.hh"
#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "config/the_isa.hh"
#include "cpu/inst_seq.hh"
#include "cpu/timebuf.hh"
#include "debug/MCQUnit.hh"
#include "debug/MCQCheck.hh"
#include "mem/packet.hh"
#include "mem/port.hh"

struct DerivO3CPUParams;
#include "base/circular_queue.hh"

/**
 * Class that implements the actual LQ and SQ for each specific
 * thread.  Both are circular queues; load entries are freed upon
 * committing, while store entries are freed once they writeback. The
 * MCQUnit tracks if there are memory ordering violations, and also
 * detects partial load to store forwarding cases (a store only has
 * part of a load's data) that requires the load to wait until the
 * store writes back. In the former case it holds onto the instruction
 * until the dependence unit looks at it, and in the latter it stalls
 * the MCQ until the store writes back. At that point the load is
 * replayed.
 */
template <class Impl>
class MCQUnit
{
  public:
    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::CPUPol::IEW IEW;
    typedef typename Impl::CPUPol::MCQ MCQ;
    typedef typename Impl::CPUPol::IssueStruct IssueStruct;

    using MCQSenderState = typename MCQ::MCQSenderState;
    using MCQRequest = typename Impl::CPUPol::MCQ::MCQRequest;

    enum state{ST0, ST1, ST2, ST3, ST4, ST5};

  private:
    class MCQEntry
    {
      //private:
      public:
        /** The instruction. */
        DynInstPtr inst;
        ///** The instruction. */
        DynInstPtr inst_org;
        /** The request. */
        MCQRequest* req;
        /** The size of the operation. */
        uint8_t _size;
        /** Valid entry. */
        bool _valid;

        Addr effAddr;
        Addr bndAddr;

        Addr bndData;
        Addr lbndData;
        Addr ubndData;
        uint8_t effSize;

        uint64_t PAC;
        uint64_t index;
        uint64_t count;
        uint64_t hash;
        uint64_t subIndex;

        bool needReplay;
        bool needUpdate;

        bool isSigned;

        enum state state;

        uint64_t AHC; // for debugging

      private:
        /** The store data. */
        char _data[64];  // TODO: 64 should become a parameter
        /** Whether or not the store can writeback. */
        bool _canWB;
        /** Whether or not the store is committed. */
        bool _committed;
        /** Whether or not the store is completed. */
        bool _completed;
        /** Does this request write all zeros and thus doesn't
         * have any data attached to it. Used for cache block zero
         * style instructs (ARM DC ZVA; ALPHA WH64)
         */
        bool _isAllZeros;

      public:
        /** Constructs an empty store queue entry. */
        MCQEntry()
            : inst(nullptr), req(nullptr), _size(0), _valid(false),
              _canWB(false), _committed(false), _completed(false),
              _isAllZeros(false)
        {
            bndAddr = 0;
            bndData = 0;
            lbndData = 0;
            ubndData = 0;
            effSize = 0;
            std::memset(_data, 0, 8);
            PAC = 0;
            index = 0;
            subIndex = 0;
            count = 0;
            hash = 0;
            needReplay = false;
            needUpdate = true;
            isSigned = false;
            state = ST0;
            AHC = 0;
        }

        ~MCQEntry()
        {
            inst = nullptr;
            inst_org = nullptr;
            if (req != nullptr) {
                req->freeMCQEntry();
                req = nullptr;
            }
        }

        void
        clear()
        {
            inst = nullptr;
            inst_org = nullptr;
            if (req != nullptr) {
                req->freeMCQEntry();
            }
            req = nullptr;
            _valid = false;
            _size = 0;
            effAddr = 0;
            bndAddr = 0;
            bndData = 0;
            lbndData = 0;
            ubndData = 0;
            effSize = 0;
            PAC = 0;
            index = 0;
            subIndex = 0;
            count = 0;
            hash = 0;
            _canWB = _completed = _committed = _isAllZeros = false;
            needReplay = false;
            needUpdate = true;
            isSigned = false;
            state = ST0;
            AHC = 0;
        }

        void
        set(const DynInstPtr& inst)
        {
            assert(!_valid);
            this->inst = inst;
            _valid = true;
            _size = 0;
            count = 0;
            state = ST0;
        } 

        void
        set(const DynInstPtr& inst, const DynInstPtr& inst_org)
        {
            assert(!_valid);
            this->inst = inst;
            this->inst_org = inst_org;
            _valid = true;
            _size = 0;
            count = 0;
            state = ST0;
        } 

        MCQRequest* request() { return req; }
        void setRequest(MCQRequest* r) { req = r; }
        bool hasRequest() { return req != nullptr; }
        /** Member accessors. */
        /** @{ */
        bool valid() const { return _valid; }
        uint8_t& size() { return _size; }
        const uint8_t& size() const { return _size; }
        const DynInstPtr& instruction() const { return inst; }
        /** @} */

        /** Member accessors. */
        /** @{ */
        bool& canWB() { return _canWB; }
        const bool& canWB() const { return _canWB; }
        bool& completed() { return _completed; }
        const bool& completed() const { return _completed; }
        bool& committed() { return _committed; }
        const bool& committed() const { return _committed; }
        bool& isAllZeros() { return _isAllZeros; }
        const bool& isAllZeros() const { return _isAllZeros; }
        char* data() { return _data; }
        const char* data() const { return _data; }
        /** @} */
    };

  public:
    using MChkQueue = CircularQueue<MCQEntry>;

    /** Constructs an MCQ unit. init() must be called prior to use. */
    MCQUnit(uint32_t mcqEntries, uint32_t bwbEntries);

    /** We cannot copy MCQUnit because it has stats for which copy
     * contructor is deleted explicitly. However, STL vector requires
     * a valid copy constructor for the base type at compile time.
     */
    MCQUnit(const MCQUnit &l) { panic("MCQUnit is not copy-able"); }

    /** Initializes the MCQ unit with the specified number of entries. */
    void init(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params,
            MCQ *mcq_ptr, unsigned id);

    /** Returns the name of the MCQ unit. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Sets the pointer to the dcache port. */
    void setDcachePort(MasterPort *dcache_port);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Inserts a load instruction. */
    void insertInst(const DynInstPtr &inst);

    /** Check if an incoming invalidate hits in the lsq on a load
     * that might have issued out of order wrt another load beacuse
     * of the intermediate invalidate.
     */
    void checkSnoop(PacketPtr pkt);

    void addReadyInsts();

    void execute();

    Fault executeLoad(typename MChkQueue::iterator& iter);

    void writebackStores();

    /**
    * Commits stores up until the given sequence number for a specific thread
    */
    void commitInsts(InstSeqNum &youngest_inst);

    /** Completes the data access that has been returned from the
     * memory system. */
    void completeDataAccess(PacketPtr pkt);

    /** Squashes all instructions younger than a specific sequence number. */
    void squash(const InstSeqNum &squashed_num);

    /** Returns the number of free MCQ entries. */
    unsigned numFreeEntries();

    /** Returns the number of loads in the MCQ. */
    int numInsts() { return insts; }

    bool isFull();

    bool isEmpty() const { return insts == 0; }

    /** Returns the number of instructions in the MCQ. */
    unsigned getCount() { return insts; }

    /** Returns if there are any stores to writeback. */
    bool hasStoresToWB() { return storesToWB; }

    /** Returns the number of stores to writeback. */
    int numStoresToWB() { return storesToWB; }

    /** Handles doing the retry. */
    void recvRetry();

    unsigned int cacheLineSize();

    bool isPACed(Addr);

    uint64_t getPAC(Addr effAddr);

    Addr getBndAddr(uint64_t PAC, uint64_t index, uint64_t count);

    bool isSrcRegsReady(DynInstPtr& inst);

    void setMCQEntry(typename MChkQueue::iterator& iter);

    bool boundsCheck(Addr addr, uint8_t effSize, Addr lbndData, Addr ubndData, bool isLoad);

    bool neonBoundCheck(Addr addr, uint8_t effSize, Addr lbndData, Addr ubndData);

    bool occupancyCheck(Addr addr, Addr lbndData, Addr ubndData, bool empty);

    bool checkForwardAndStall(typename MChkQueue::iterator& iter);

    uint64_t getHash(uint64_t effAddr, uint64_t PAC);

    Fault pushRequest(const DynInstPtr& inst, bool isLoad, uint8_t *data,
                       unsigned int size, Addr addr, Request::Flags flags,
                       uint64_t *res, typename MChkQueue::iterator& iter);

    void reallocBoundTable();

  private:
    /** Reset the MCQ state */
    void resetState();

  public:
    /** Attempts to send a packet to the cache.
     * Check if there are ports available. Return true if
     * there are, false if there are not.
     */
    bool trySendPacket(bool isLoad, PacketPtr data_pkt);

    /** Debugging function to dump instructions in the MCQ. */
    void dumpInsts() const;

    /** Schedule event for the cpu. */
    void schedule(Event& ev, Tick when) { cpu->schedule(ev, when); }

    BaseTLB* dTLB() { return cpu->dtb; }

  private:

    /** Pointer to the CPU. */
    O3CPU *cpu;

    /** Pointer to the IEW stage. */
    IEW *iewStage;

    /** Pointer to the MCQ. */
    MCQ *mcq;

    /** Pointer to the dcache port.  Used only for sending. */
    MasterPort *dcachePort;

    /** Particularisation of the MCQSenderState to the MQ. */
    class MQSenderState : public MCQSenderState
    {
        using MCQSenderState::alive;
      public:
        MQSenderState(typename MChkQueue::iterator idx_)
            : MCQSenderState(idx_->request(), true), idx(idx_) { }

        /** The LQ index of the instruction. */
        typename MChkQueue::iterator idx;
        //virtual MCQRequest* request() { return idx->request(); }
        virtual void
        complete()
        {
        }
    };

  public:
    /**
     * Handles writing back and completing the load or store that has
     * returned from memory.
     *
     * @param pkt Response packet from the memory sub-system
     */
    bool recvTimingResp(PacketPtr pkt);

  private:

    /** The MCQUnit thread id. */
    ThreadID mcqID;

  public:
    /** The store queue. */
    CircularQueue<MCQEntry> mchkQueue;

  private:
    /** The number of load instructions in the LQ. */
    int insts;

    /** The number of store instructions in the SQ waiting to writeback. */
    int storesToWB;

    /** Address Mask for a cache block (e.g. ~(cache_block_size-1)) */
    Addr cacheBlockMask;

    /** Whether or not the MCQ is stalled. */
    bool stalled;
    /** The store that causes the stall due to partial store to load
     * forwarding.
     */
    InstSeqNum stallingStoreIsn;
    /** The index of the above store. */
    int stallingLoadIdx;

    /** The packet that needs to be retried. */
    PacketPtr retryPkt;

    /** Whehter or not a store is blocked due to the memory system. */
    bool isStoreBlocked;

    /** Whether or not a store is in flight. */
    bool storeInFlight;

    /** Whether or not there is a packet that couldn't be sent because of
     * a lack of cache ports. */
    bool hasPendingRequest;

    /** The packet that is pending free cache ports. */
    MCQRequest* pendingRequest;

    /** Total number of squashed loads. */
    Stats::Scalar mcqSquashedInsts;

    /** Number of times the MCQ is blocked due to the cache. */
    Stats::Scalar mcqCacheBlocked;

    Stats::Scalar mcqCommittedInsts;

    Stats::Scalar mcqCommittedBndstr;

    Stats::Scalar mcqCommittedBndclr;

    Stats::Scalar mcqCommittedPacma;

    Stats::Scalar mcqCommittedXpacm;

    Stats::Scalar mcqCommittedSignedLoad;

    Stats::Scalar mcqCommittedSignedStore;

    Stats::Scalar mcqCommittedUnsignedLoad;

    Stats::Scalar mcqCommittedUnsignedStore;

    Stats::Scalar mcqBoundCheck;

    Stats::Scalar mcqDataFwdBoundCheck;

    Stats::Scalar mcqOccupancyCheck;

    Stats::Scalar mcqDataFwdOccupancyCheck;

    Stats::Scalar mcqReplayedBoundStore;

    Stats::Scalar mcqReplayedBoundCheck;

    Stats::Scalar mcqNumCounterInc;

    Stats::Formula mcqAvgCounterInc;

    Stats::Scalar bwbNumHits;

    Stats::Scalar bwbNumMisses;

    Stats::Scalar bwbNumEvictions;

    Stats::Scalar mcqSmallBin;

    Stats::Scalar mcqMediumBin;

    Stats::Scalar mcqLargeBin;

    Stats::Scalar numBndStrFailure;

    Stats::Scalar numBndClrFailure;

    Stats::Scalar numBndChkFailure;

  public:
    /** Executes the load at the given index. */
    Fault check(MCQRequest *req, int mck_idx);

    /** Returns the index of the head mchk instruction. */
    int getHead() { return mchkQueue.head(); }

    /** Returns the sequence number of the head mchk instruction. */
    InstSeqNum
    getHeadSeqNum()
    {
        return mchkQueue.front().valid()
            ? mchkQueue.front().instruction()->seqNum
            : 0;
    }

    /** Returns whether or not the MCQ unit is stalled. */
    bool isStalled()  { return stalled; }

    typedef typename CircularQueue<MCQEntry>::iterator MCQIterator;

  private:
    class BWB
    {
      private:
        class BWBEntry
        {
          public:
            uint64_t valid;
            uint64_t PAC;
            uint64_t hash;
            uint64_t index;
            bool occupied;

            BWBEntry()
              : valid(false), PAC(0), hash(0), index(0), occupied(false)
            {
            }

            ~BWBEntry()
            {
            }

            void setPAC(uint64_t PAC) { this->PAC = PAC; }
        };

      public:
        int maxBWBEntries;

        BWB()
        {
        }

        ~BWB()
        {
            m_list.clear();
        }

        MCQUnit *mcqUnit;

        //std::vector<BWBEntry> bwb;

        std::list<BWBEntry*> m_list;
        // TODO: need to have a iterator to reduce search overhead.
        bool isFull() { return m_list.size() >= maxBWBEntries; }

        void eviction();

        void insert(uint64_t PAC, uint64_t hash, uint64_t index);

        void update(uint64_t PAC, uint64_t hash, uint64_t index, bool isBndclr);

        uint64_t getIndex(uint64_t PAC, uint64_t hash, bool isBndstr);

        uint64_t crc12(uint64_t PAC, uint64_t hash);
    };

    BWB bwb;

  public:
    Addr btBaseAddr;

    int btNumWays;

    void cacheUnblocked();

    std::list<DynInstPtr> blockedInsts;

    std::list<DynInstPtr> executeQueue;

    std::list<DynInstPtr> readyInsts;

    std::list<DynInstPtr> notReadyInsts;

    std::list<DynInstPtr> waitingForCommit;
};

template<class Impl>
Fault
MCQUnit<Impl>::check(MCQRequest *req, int mchk_idx)
{
    MCQEntry& mchk_req = mchkQueue[mchk_idx];
    const DynInstPtr& inst = mchk_req.inst;

    mchk_req.setRequest(req);
    assert(inst);

    assert(!(req->mainRequest()->isStrictlyOrdered() ||
                req->mainRequest()->isLLSC() ||
                req->mainRequest()->isMmappedIpr()));

    DPRINTF(MCQUnit, "Check called, mchk idx: %i, paddr: %#x%s\n",
            mchk_idx - 1, req->mainRequest()->getPaddr(), req->isSplit() ? " split" : "");

    // Check possible data forwarding   
    auto iter =  mchkQueue.getIterator(inst->mcqIdx);
    auto store_it = iter;

    // Update stats
    if (iter->state == ST1)
        mcqBoundCheck++;
    else if (iter->state == ST2)
        mcqOccupancyCheck++;

    while (store_it != mchkQueue.begin()) {
        store_it--;
        auto store_inst = store_it->instruction();

        if (!store_inst->isBndInst())
            continue;

        Addr addr = iter->effAddr;
        uint8_t effSize = iter->effSize;

        Addr lbndData = store_it->lbndData;
        Addr ubndData = store_it->ubndData;

        if (iter->PAC == store_it->PAC) {
            DPRINTF(MCQUnit, "Check possible data forwarding!\n");

            if (iter->state == ST1) {
                if (inst->isNeonLoad() && neonBoundCheck(addr, effSize, lbndData, ubndData)) {
                    Addr neon_lbndData = lbndData - (lbndData % 64);
                    Addr neon_ubndData = ubndData - (ubndData % 64) + 64;

                    DPRINTF(MCQUnit, "Passed neonBoundCheck by data forwarding! "
                    "bndAddr: %lu addr: %lu effSize: %u neon_lbndData: %lu neon_ubndData: %lu count: %u "
                    "PAC: %lu [sn:%lli]\n", iter->bndAddr, addr, effSize, neon_lbndData, neon_ubndData,
                    iter->count, iter->PAC, iter->inst_org->seqNum);
                    iter->inst_org->setValidated(true);
                    iter->state = ST5;
                    iter->needUpdate = false;
                    mcqDataFwdBoundCheck++;

                    iter->setRequest(nullptr);
                    req->discard();

                    return NoFault;
                } else if (!inst->isNeonLoad() && boundsCheck(addr, effSize, lbndData, ubndData, inst->isLoad())) {
                    DPRINTF(MCQUnit, "Passed boundsCheck by data forwarding! "
                    "bndAddr: %lu addr: %lu effSize: %u lbndData: %lu ubndData: %lu count: %u "
                    "PAC: %lu [sn:%lli]\n", iter->bndAddr, addr, effSize, lbndData, ubndData,
                    iter->count, iter->PAC, iter->inst_org->seqNum);
                    iter->inst_org->setValidated(true);
                    iter->state = ST5;
                    iter->needUpdate = false;
                    mcqDataFwdBoundCheck++;

                    iter->setRequest(nullptr);
                    req->discard();

                    return NoFault;
                }
            }

            assert((iter->state == ST1 || iter->state == ST2)
                    && "Wrong state in checkForwardAndStall\n");
        }
    }

    // If there's no forwarding case, then go access memory
    DPRINTF(MCQUnit, "Doing memory access for inst [sn:%lli] PC %s\n",
            inst->seqNum, inst->pcState());

    // Allocate memory if this is the first time a load is issued.
    if (!inst->memData) {
        inst->memData = new uint8_t[req->mainRequest()->getSize()];
    }

    // For now, load throughput is constrained by the number of
    // load FUs only, and loads do not consume a cache port (only
    // stores do).
    // @todo We should account for cache port contention
    // and arbitrate between loads and stores.

    // if the cache is not blocked, do cache access
    if (req->senderState() == nullptr) {
        MQSenderState *state = new MQSenderState(
                mchkQueue.getIterator(mchk_idx));
        state->isLoad = true;
        state->inst = inst;
        state->isSplit = req->isSplit();
        req->senderState(state);
    }
    req->buildPackets();

    if (iter->state == ST1)
        req->_packets.back()->isBndLoad = true;
    else if (iter->state == ST2)
        req->_packets.back()->isBndStore = true;
    
    req->sendPacketToCache();

    if (!req->isSent()) {
        iter->setRequest(nullptr);
        req->discard();
        DPRINTF(MCQUnit, "Blocked bound load inst found!\n"); 
        blockedInsts.push_back(inst);
    }

    return NoFault;
}

#endif // __CPU_O3_MCQ_UNIT_HH__
