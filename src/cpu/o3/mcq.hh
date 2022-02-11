
#ifndef __CPU_O3_MCQ_HH__
#define __CPU_O3_MCQ_HH__

#include <map>
#include <queue>

#include "arch/generic/tlb.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/mcq_unit.hh"
#include "enums/SMTQueuePolicy.hh"
#include "mem/port.hh"
#include "sim/sim_object.hh"

struct DerivO3CPUParams;

template <class Impl>
class MCQ
{
  public:
    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::CPUPol::IEW IEW;
    typedef typename Impl::CPUPol::MCQUnit MCQUnit;

    class MCQRequest;
    /** Derived class to hold any sender state the MCQ needs. */
    class MCQSenderState : public Packet::SenderState
    {
        typedef typename Impl::CPUPol::MCQUnit MCQEntry;
        using MChkQueue = CircularQueue<MCQEntry>;

      protected:
        /** The senderState needs to know the MCQRequest who owns it. */
        MCQRequest* _request;

        /** Default constructor. */
        MCQSenderState(MCQRequest* request, bool isLoad_)
            : _request(request), mainPkt(nullptr), pendingPacket(nullptr),
              outstanding(0), isLoad(isLoad_), needWB(isLoad_), isSplit(false),
              pktToSend(false), deleted(false)
          { }
      public:

        /** Instruction which initiated the access to memory. */
        DynInstPtr inst;
        /** The main packet from a split load, used during writeback. */
        PacketPtr mainPkt;
        /** A second packet from a split store that needs sending. */
        PacketPtr pendingPacket;
        /** Number of outstanding packets to complete. */
        uint8_t outstanding;
        /** Whether or not it is a load. */
        bool isLoad;
        /** Whether or not the instruction will need to writeback. */
        bool needWB;
        /** Whether or not this access is split in two. */
        bool isSplit;
        /** Whether or not there is a packet that needs sending. */
        bool pktToSend;
        /** Has the request been deleted?
         * MCQ entries can be squashed before the response comes back. in that
         * case the SenderState knows.
         */
        bool deleted;
        ContextID contextId() { return inst->contextId(); }

        /** Completes a packet and returns whether the access is finished. */
        inline bool isComplete() { return outstanding == 0; }
        inline void deleteRequest() { deleted = true; }
        inline bool alive() { return !deleted; }
        MCQRequest* request() { return _request; }
        virtual void complete() = 0;
        void writebackDone() { _request->writebackDone(); }
    };

    class MCQRequest : public BaseTLB::Translation
    {
      protected:
        typedef uint32_t FlagsStorage;
        typedef ::Flags<FlagsStorage> FlagsType;

        enum Flag : FlagsStorage
        {
            IsLoad              = 0x00000001,
            /** True if this is a store/atomic that writes registers (SC). */
            WbStore             = 0x00000002,
            Delayed             = 0x00000004,
            IsSplit             = 0x00000008,
            /** True if any translation has been sent to TLB. */
            TranslationStarted  = 0x00000010,
            /** True if there are un-replied outbound translations.. */
            TranslationFinished = 0x00000020,
            Sent                = 0x00000040,
            Retry               = 0x00000080,
            Complete            = 0x00000100,
            /** Ownership tracking flags. */
            /** Translation squashed. */
            TranslationSquashed = 0x00000200,
            /** Request discarded */
            Discarded           = 0x00000400,
            /** MCQ resources freed. */
            MCQEntryFreed       = 0x00000800,
            /** Store written back. */
            WritebackScheduled  = 0x00001000,
            WritebackDone       = 0x00002000,
            /** True if this is an atomic request */
            IsAtomic            = 0x00004000
        };
        FlagsType flags;

        enum class State
        {
            NotIssued,
            Translation,
            Request,
            Complete,
            Squashed,
            Fault,
        };
        State _state;
        MCQSenderState* _senderState;
        void setState(const State& newState) { _state = newState; }

        uint32_t numTranslatedFragments;
        uint32_t numInTranslationFragments;

        /** LQ/SQ entry idx. */
        uint32_t _entryIdx;

        void markDelayed() override { flags.set(Flag::Delayed); }
        bool isDelayed() { return flags.isSet(Flag::Delayed); }

      public:
        MCQUnit& _port;
        const DynInstPtr _inst;
        uint32_t _taskId;
        PacketDataPtr _data;
        std::vector<PacketPtr> _packets;
        std::vector<RequestPtr> _requests;
        std::vector<Fault> _fault;
        uint64_t* _res;
        const Addr _addr;
        const uint32_t _size;
        const Request::Flags _flags;
        uint32_t _numOutstandingPackets;
        AtomicOpFunctor *_amo_op;
      protected:
        MCQUnit* mcqUnit() { return &_port; }
        MCQRequest(MCQUnit* port, const DynInstPtr& inst, bool isLoad) :
            _state(State::NotIssued), _senderState(nullptr),
            _port(*port), _inst(inst), _data(nullptr),
            _res(nullptr), _addr(0), _size(0), _flags(0),
            _numOutstandingPackets(0), _amo_op(nullptr)
        {
            flags.set(Flag::IsLoad, isLoad);
            flags.set(Flag::WbStore,
                      _inst->isStoreConditional() || _inst->isAtomic());
            flags.set(Flag::IsAtomic, _inst->isAtomic());
            install();
        }
        MCQRequest(MCQUnit* port, const DynInstPtr& inst, bool isLoad,
                   const Addr& addr, const uint32_t& size,
                   const Request::Flags& flags_,
                   PacketDataPtr data = nullptr, uint64_t* res = nullptr,
                   AtomicOpFunctor* amo_op = nullptr)
            : _state(State::NotIssued), _senderState(nullptr),
            numTranslatedFragments(0),
            numInTranslationFragments(0),
            _port(*port), _inst(inst), _data(data),
            _res(res), _addr(addr), _size(size),
            _flags(flags_),
            _numOutstandingPackets(0),
            _amo_op(amo_op)
        {
            flags.set(Flag::IsLoad, isLoad);
            flags.set(Flag::WbStore,
                      _inst->isStoreConditional() || _inst->isAtomic());
            flags.set(Flag::IsAtomic, _inst->isAtomic());
            install();
        }

        bool
        isLoad() const
        {
            return flags.isSet(Flag::IsLoad);
        }

        bool
        isAtomic() const
        {
            return flags.isSet(Flag::IsAtomic);
        }

        /** Install the request in the LQ/SQ. */
        void install()
        {
            _port.mchkQueue[_inst->mcqIdx].setRequest(this);
        }
        virtual bool
        squashed() const override
        {
            return _inst->isSquashed();
        }

        /**
         * Test if the MCQRequest has been released, i.e. self-owned.
         * An MCQRequest manages itself when the resources on the MCQ are freed
         * but the translation is still going on and the MCQEntry was freed.
         */
        bool
        isReleased()
        {
            return flags.isSet(Flag::MCQEntryFreed) ||
                flags.isSet(Flag::Discarded);
        }

        /** Release the MCQRequest.
         * Notify the sender state that the request it points to is not valid
         * anymore. Understand if the request is orphan (self-managed) and if
         * so, mark it as freed, else destroy it, as this means
         * the end of its life cycle.
         * An MCQRequest is orphan when its resources are released
         * but there is any in-flight translation request to the TLB or access
         * request to the memory.
         */
        void release(Flag reason)
        {
            assert(reason == Flag::MCQEntryFreed || reason == Flag::Discarded);
            if (!isAnyOutstandingRequest()) {
                delete this;
            } else {
                if (_senderState) {
                    _senderState->deleteRequest();
                }
                flags.set(reason);
            }
        }

      public:
        /** Destructor.
         * The MCQRequest owns the request. If the packet has already been
         * sent, the sender state will be deleted upon receiving the reply.
         */
        virtual ~MCQRequest()
        {
            assert(!isAnyOutstandingRequest());
            _inst->savedReq = nullptr;
            if (_senderState)
                delete _senderState;

            for (auto r: _packets)
                delete r;
        };


      public:
        /** Convenience getters/setters. */
        /** @{ */
        /** Set up Context numbers. */
        void
        setContext(const ContextID& context_id)
        {
            request()->setContext(context_id);
        }

        const DynInstPtr&
        instruction()
        {
            return _inst;
        }

        /** Set up virtual request.
         * For a previously allocated Request objects.
         */
        void
        setVirt(int asid, Addr vaddr, unsigned size, Request::Flags flags_,
                MasterID mid, Addr pc)
        {
            request()->setVirt(asid, vaddr, size, flags_, mid, pc);
        }

        void
        taskId(const uint32_t& v)
        {
            _taskId = v;
            for (auto& r: _requests)
                r->taskId(v);
        }

        uint32_t taskId() const { return _taskId; }
        RequestPtr request(int idx = 0) { return _requests.at(idx); }

        const RequestPtr
        request(int idx = 0) const
        {
            return _requests.at(idx);
        }

        Addr getVaddr(int idx = 0) const { return request(idx)->getVaddr(); }
        virtual void initiateTranslation() = 0;

        PacketPtr packet(int idx = 0) { return _packets.at(idx); }

        virtual PacketPtr
        mainPacket()
        {
            assert (_packets.size() == 1);
            return packet();
        }

        virtual RequestPtr
        mainRequest()
        {
            assert (_requests.size() == 1);
            return request();
        }

        void
        senderState(MCQSenderState* st)
        {
            _senderState = st;
            for (auto& pkt: _packets) {
                if (pkt)
                    pkt->senderState = st;
            }
        }

        const MCQSenderState*
        senderState() const
        {
            return _senderState;
        }

        /**
         * Mark senderState as discarded. This will cause to discard response
         * packets from the cache.
         */
        void
        discardSenderState()
        {
            assert(_senderState);
            _senderState->deleteRequest();
        }

        /**
         * Test if there is any in-flight translation or mem access request
         */
        bool
        isAnyOutstandingRequest()
        {
            return numInTranslationFragments > 0 ||
                _numOutstandingPackets > 0 ||
                (flags.isSet(Flag::WritebackScheduled) &&
                 !flags.isSet(Flag::WritebackDone));
        }

        bool
        isSplit() const
        {
            return flags.isSet(Flag::IsSplit);
        }
        /** @} */
        virtual bool recvTimingResp(PacketPtr pkt) = 0;
        virtual void sendPacketToCache() = 0;
        virtual void buildPackets() = 0;

        ///**
        // * Memory mapped IPR accesses
        // */
        //virtual void handleIprWrite(ThreadContext *thread, PacketPtr pkt) = 0;
        //virtual Cycles handleIprRead(ThreadContext *thread, PacketPtr pkt) = 0;

        /**
         * Test if the request accesses a particular cache line.
         */
        virtual bool isCacheBlockHit(Addr blockAddr, Addr cacheBlockMask) = 0;

        /** Update the status to reflect that a packet was sent. */
        void
        packetSent()
        {
            flags.set(Flag::Sent);
        }
        /** Update the status to reflect that a packet was not sent.
         * When a packet fails to be sent, we mark the request as needing a
         * retry. Note that Retry flag is sticky.
         */
        void
        packetNotSent()
        {
            flags.set(Flag::Retry);
            flags.clear(Flag::Sent);
        }

        void sendFragmentToTranslation(int i);
        bool
        isComplete()
        {
            return flags.isSet(Flag::Complete);
        }

        bool
        isInTranslation()
        {
            return _state == State::Translation;
        }

        bool
        isTranslationComplete()
        {
            return flags.isSet(Flag::TranslationStarted) &&
                   !isInTranslation();
        }

        bool
        isTranslationBlocked()
        {
            return _state == State::Translation &&
                flags.isSet(Flag::TranslationStarted) &&
                !flags.isSet(Flag::TranslationFinished);
        }

        bool
        isSent()
        {
            return flags.isSet(Flag::Sent);
        }

        /**
         * The MCQ entry is cleared
         */
        void
        freeMCQEntry()
        {
            release(Flag::MCQEntryFreed);
        }

        /**
         * The request is discarded (e.g. partial store-load forwarding)
         */
        void
        discard()
        {
            release(Flag::Discarded);
        }

        void
        packetReplied()
        {
            assert(_numOutstandingPackets > 0);
            _numOutstandingPackets--;
            if (_numOutstandingPackets == 0 && isReleased())
                delete this;
        }

        void
        writebackScheduled()
        {
            assert(!flags.isSet(Flag::WritebackScheduled));
            flags.set(Flag::WritebackScheduled);
        }

        void
        writebackDone()
        {
            flags.set(Flag::WritebackDone);
            /* If the lsq resources are already free */
            if (isReleased()) {
                delete this;
            }
        }

        void
        squashTranslation()
        {
            assert(numInTranslationFragments == 0);
            flags.set(Flag::TranslationSquashed);
            /* If we are on our own, self-destruct. */
            if (isReleased()) {
                delete this;
            }
        }

        void
        complete()
        {
            flags.set(Flag::Complete);
        }
    };

    class SingleDataRequest : public MCQRequest
    {
      protected:
        /* Given that we are inside templates, children need explicit
         * declaration of the names in the parent class. */
        using Flag = typename MCQRequest::Flag;
        using State = typename MCQRequest::State;
        using MCQRequest::_fault;
        using MCQRequest::_inst;
        using MCQRequest::_packets;
        using MCQRequest::_port;
        using MCQRequest::_res;
        using MCQRequest::_senderState;
        using MCQRequest::_state;
        using MCQRequest::flags;
        using MCQRequest::isLoad;
        using MCQRequest::isTranslationComplete;
        using MCQRequest::mcqUnit;
        using MCQRequest::request;
        using MCQRequest::sendFragmentToTranslation;
        using MCQRequest::setState;
        using MCQRequest::numInTranslationFragments;
        using MCQRequest::numTranslatedFragments;
        using MCQRequest::_numOutstandingPackets;
        using MCQRequest::_amo_op;
      public:
        SingleDataRequest(MCQUnit* port, const DynInstPtr& inst, bool isLoad,
                          const Addr& addr, const uint32_t& size,
                          const Request::Flags& flags_,
                          PacketDataPtr data = nullptr,
                          uint64_t* res = nullptr,
                          AtomicOpFunctor* amo_op = nullptr) :
            MCQRequest(port, inst, isLoad, addr, size, flags_, data, res,
                       amo_op)
        {
            MCQRequest::_requests.push_back(
                    std::make_shared<Request>(inst->getASID(), addr, size,
                    flags_, inst->masterId(), inst->instAddr(),
                    inst->contextId(), amo_op));
            MCQRequest::_requests.back()->setReqInstSeqNum(inst->seqNum);
        }
        inline virtual ~SingleDataRequest() {}
        virtual void initiateTranslation();
        virtual void finish(const Fault &fault, const RequestPtr &req,
                ThreadContext* tc, BaseTLB::Mode mode);
        virtual bool recvTimingResp(PacketPtr pkt);
        virtual void sendPacketToCache();
        virtual void buildPackets();
        //virtual void handleIprWrite(ThreadContext *thread, PacketPtr pkt);
        //virtual Cycles handleIprRead(ThreadContext *thread, PacketPtr pkt);
        virtual bool isCacheBlockHit(Addr blockAddr, Addr cacheBlockMask);
    };

    /** Constructs an MCQ with the given parameters. */
    MCQ(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params);
    ~MCQ() { }

    /** Returns the name of the MCQ. */
    std::string name() const;

    /** Registers statistics of each MCQ unit. */
    void regStats();

    /** Sets the pointer to the list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;
    /** Has the MCQ drained? */
    bool isDrained() const;
    /** Takes over execution from another CPU's thread. */
    void takeOverFrom();

    /** Number of entries needed for the given amount of threads.*/
    int entryAmount(ThreadID num_threads);

    /** Ticks the MCQ. */
    void tick();

    /** Inserts a load into the MCQ. */
    void insertInst(const DynInstPtr &inst);

    /** Execute a instr. */
    void execute();

    /**
     * Squash instructions from a thread until the specified sequence number.
     */
    void
    squash(const InstSeqNum &squashed_num, ThreadID tid)
    {
        thread.at(tid).squash(squashed_num);
    }

    ///** Returns whether or not there was a memory ordering violation. */
    //bool violation();
    ///**
    // * Returns whether or not there was a memory ordering violation for a
    // * specific thread.
    // */
    //bool violation(ThreadID tid) { return thread.at(tid).violation(); }

    /** Returns the head index of the mchk queue for a specific thread. */
    int getHead(ThreadID tid) { return thread.at(tid).getHead(); }

    /** Returns the sequence number of the head of the mchk queue. */
    InstSeqNum
    getHeadSeqNum(ThreadID tid)
    {
        return thread.at(tid).getHeadSeqNum();
    }

    /** Returns the number of instructions in all of the queues. */
    int getCount();
    /** Returns the number of instructions in the queues of one thread. */
    int getCount(ThreadID tid) { return thread.at(tid).getCount(); }

    /** Returns the total number of insts in the load queue. */
    int numInsts();
    /** Returns the total number of loads for a single thread. */
    int numInsts(ThreadID tid) { return thread.at(tid).numInsts(); }

    /** Returns the number of free load entries. */
    unsigned numFreeEntries();

    /** Returns the number of free entries for a specific thread. */
    unsigned numFreeEntries(ThreadID tid);

    /** Returns if the MCQ is full (either LQ or SQ is full). */
    bool isFull();
    /**
     * Returns if the MCQ is full for a specific thread (either LQ or SQ is
     * full).
     */
    bool isFull(ThreadID tid);

    /** Returns if all of the MCQs are empty. */
    bool isEmpty() const;

    /** Debugging function to print out all instructions. */
    void dumpInsts() const;
    /** Debugging function to print out instructions from a specific thread. */
    void dumpInsts(ThreadID tid) const { thread.at(tid).dumpInsts(); }

    Fault check(MCQRequest* req, int mchk_idx);

    /**
     * Retry the previous send that failed.
     */
    void recvReqRetry();

    void completeDataAccess(PacketPtr pkt);

    bool recvTimingResp(PacketPtr pkt);

    void recvTimingSnoopReq(PacketPtr pkt);

    Fault pushRequest(const DynInstPtr& inst, bool isLoad, uint8_t *data,
                      unsigned int size, Addr addr, Request::Flags flags,
                      uint64_t *res, AtomicOpFunctor *amo_op);

    /**
     * Commits stores up until the given sequence number for a specific thread.
     */
    void commitInsts(InstSeqNum &youngest_inst, ThreadID tid)
    { thread.at(tid).commitInsts(youngest_inst); }

    /**
     * Attempts to write back stores until all cache ports are used or the
     * interface becomes blocked.
     */
    void writebackStores();
    /** Same as above, but only for one thread. */
    void writebackStores(ThreadID tid);

    /** The CPU pointer. */
    O3CPU *cpu;

    /** The IEW stage pointer. */
    IEW *iewStage;

    /** D-cache unblocked */
    void cacheUnblocked();

    /** Is D-cache blocked? */
    bool cacheBlocked() const;
    /** Set D-cache blocked status */
    void cacheBlocked(bool v);
    /** Is any store port available to use? */
    bool cachePortAvailable(bool is_load) const;
    /** Another store port is in use */
    void cachePortBusy(bool is_load);

  protected:
    /** D-cache is blocked */
    bool _cacheBlocked;
    /** The number of cache ports available each cycle (stores only). */
    int cacheStorePorts;
    /** The number of used cache ports in this cycle by stores. */
    int usedStorePorts;
    /** The number of cache ports available each cycle (loads only). */
    int cacheLoadPorts;
    /** The number of used cache ports in this cycle by loads. */
    int usedLoadPorts;

    /** The MCQ policy for SMT mode. */
    SMTQueuePolicy lsqPolicy;

    static uint32_t
    maxMCQAllocation(SMTQueuePolicy pol, uint32_t entries,
            uint32_t numThreads, uint32_t SMTThreshold)
    {
        if (pol == SMTQueuePolicy::Dynamic) {
            return entries;
        } else if (pol == SMTQueuePolicy::Partitioned) {
            //@todo:make work if part_amt doesnt divide evenly.
            return entries / numThreads;
        } else if (pol == SMTQueuePolicy::Threshold) {
            //Divide up by threshold amount
            //@todo: Should threads check the max and the total
            //amount of the MCQ
            return SMTThreshold;
        }
        return 0;
    }

    /** List of Active Threads in System. */
    std::list<ThreadID> *activeThreads;

    /** Total Size of MCQ Entries. */
    unsigned MCQEntries;

    /** Max MCQ Size - Used to Enforce Sharing Policies. */
    unsigned maxMCQEntries;

    /** The MCQ units for individual threads. */
    //temp std::vector<MCQUnit> thread;

    /** Number of Threads. */
    ThreadID numThreads;

    bool needBtAllocated;

    bool isBtAllocated;

  public:
    std::vector<MCQUnit> thread;

    Addr btBaseAddr;

    int64_t btSize;

    int btNumWays;
};

template <class Impl>
Fault
MCQ<Impl>::check(MCQRequest* req, int mchk_idx)
{
    ThreadID tid = cpu->contextToThread(req->request()->contextId());

    return thread.at(tid).check(req, mchk_idx);
}

#endif // __CPU_O3_MCQ_HH__
