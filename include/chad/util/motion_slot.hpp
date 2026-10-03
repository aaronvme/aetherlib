#pragma once

#include <atomic>
#include <cstdint>
#include <utility>

namespace chad {
// Sequence bookkeeping for a motion queue that holds at most one pending
// request. Cancelling marks every sequence issued so far rather than setting a
// flag, so it cannot be lost to or leak into a request it did not cover.
//
// push, pop, finish and cancelAll must be serialised by the caller's lock.
// markCancelled, release and the status queries are lock-free, for a caller
// that cannot wait on a lock a deleted task may still hold.
template <typename Request>
class MotionSlot {
public:
    bool hasPending() const { return mHasPending; }

    // Requires !hasPending(). Returns the sequence assigned to the request.
    uint32_t push(Request&& req) {
        mPending = std::move(req);
        mPendingSeq = mIssued.load() + 1;
        mHasPending = true;
        mIssued.store(mPendingSeq);
        return mPendingSeq;
    }

    // Hands the pending request to the executor, which owns it until finish().
    // A request cancelled while pending is dropped here instead.
    bool pop(Request& out, uint32_t& seq) {
        if (!mHasPending) return false;
        if (cancelled(mPendingSeq)) {
            mHasPending = false;
            advanceCompleted(mPendingSeq);
            return false;
        }
        out = std::move(mPending);
        seq = mPendingSeq;
        mHasPending = false;
        mActiveSeq.store(seq);
        mActive.store(true);
        return true;
    }

    void finish(uint32_t seq) {
        advanceCompleted(seq);
        // Anything cancelled after this was popped was dropped from the slot
        // and will never reach finish() itself.
        advanceCompleted(mCancelled.load());
        mActive.store(false);
    }

    // Cancels every request issued so far and returns the last of them.
    uint32_t cancelAll() {
        const uint32_t through = markCancelled();
        mHasPending = false;
        if (!mActive.load()) advanceCompleted(through);
        return through;
    }

    // cancelAll without touching the slot; pop() drops what it covered.
    uint32_t markCancelled() {
        const uint32_t through = mIssued.load();
        advance(mCancelled, through);
        return through;
    }

    // Marks everything up to seq finished without waiting for the executor.
    void release(uint32_t seq) { advanceCompleted(seq); }

    // Finished or released: the executor no longer owns the output.
    bool completed(uint32_t seq) const { return reached(mCompleted.load(), seq); }

    bool cancelled(uint32_t seq) const { return reached(mCancelled.load(), seq); }
    uint32_t cancelledThrough() const { return mCancelled.load(); }

    // Finished, or cancelled and so never going to be.
    bool done(uint32_t seq) const {
        return reached(mCompleted.load(), seq) || cancelled(seq);
    }

    bool settled() const { return reached(mCompleted.load(), mIssued.load()); }

    // The executor still holds a request issued at or before seq.
    bool runningThrough(uint32_t seq) const {
        return mActive.load() && reached(seq, mActiveSeq.load());
    }

private:
    // Wrap-safe "mark is at or past seq".
    static bool reached(uint32_t mark, uint32_t seq) {
        return static_cast<int32_t>(mark - seq) >= 0;
    }

    static void advance(std::atomic<uint32_t>& mark, uint32_t seq) {
        uint32_t cur = mark.load();
        while (!reached(cur, seq) && !mark.compare_exchange_weak(cur, seq)) {}
    }

    void advanceCompleted(uint32_t seq) { advance(mCompleted, seq); }

    Request mPending{};
    uint32_t mPendingSeq{0};
    bool mHasPending{false};

    std::atomic<uint32_t> mIssued{0};
    std::atomic<uint32_t> mCompleted{0};
    std::atomic<uint32_t> mCancelled{0};
    std::atomic<uint32_t> mActiveSeq{0};
    std::atomic<bool> mActive{false};
};
}  // namespace chad
