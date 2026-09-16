/**
 * @file tracker.hpp
 * @brief Master-slave timestamp tracker; thread-safe.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_BASE_TRACKER_HPP
#define GS130_BASE_TRACKER_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace gs130 {
namespace base {

/**
 * Corrects slave-clock sample timestamps against a master clock.
 *
 * The two clocks are fed independently: the master side uploads the absolute time of
 * its frame edges, the slave side uploads one call per sample, and anchor samples
 * additionally carry the offset between the trigger edge and the sample point.  The
 * tracker assumes the master advances by a fixed nominal period and that a slave sample
 * belongs to the edge it follows, then emits one corrected timestamp per sample.
 *
 * All timestamps are 64-bit nanosecond counts. Returned values use the master's
 * absolute epoch; the interval between slave samples is estimated between FSYNC
 * anchors and applied within that master-aligned timeline.
 *
 * Thread safety: the master and the slave side are normally driven from different
 * threads, so every public member serialises on one internal mutex.  The tracker is
 * non-copyable and movable (a move transfers the lock, the source must not be used).
 *
 * Ownership: the tracker owns its state; the caller keeps ownership of every pointer
 * it passes in, and none of them is retained past the call.
 */
class TimestampTracker {
public:
    /**
     * @param master_cycle_ns Nominal master period in nanoseconds, i.e. the distance
     *        between two master edges (e.g. one frame period).  It is used for the
     *        anchor prediction and must be greater than zero. Passing zero would make
     *        phase alignment unable to advance.
     */
    explicit TimestampTracker(uint32_t master_cycle_ns);

    // copying disabled
    TimestampTracker(const TimestampTracker &) = delete;
    TimestampTracker &operator=(const TimestampTracker &) = delete;

    // moving allowed (std::mutex is not movable, so the lock is held via unique_ptr; the
    // source object is left without a lock and must not be used again)
    TimestampTracker(TimestampTracker &&other) noexcept
        : master_cycle_ns_(other.master_cycle_ns_), master_(other.master_),
          slave_(other.slave_), ready_(std::move(other.ready_)),
          mtx_(std::move(other.mtx_))
    {
    }
    TimestampTracker &operator=(TimestampTracker &&other) noexcept
    {
        if(this != &other){
            master_cycle_ns_ = other.master_cycle_ns_;
            master_          = other.master_;
            slave_           = other.slave_;
            ready_           = std::move(other.ready_);
            mtx_             = std::move(other.mtx_);
        }
        return *this;
    }

    // upload the master clock's absolute time as the phase reference
    // The first call also fixes the starting edge; a literal 0 is reserved as "not set
    // yet", so it must never be passed as a real timestamp.
    void update_master_timestamp_ns(uint64_t timestamp_ns);

    // upload a slave clock sample. delta == NULL means an ordinary sample;
    // otherwise it is an anchor sample (offset from trigger edge to sample point, ns).
    /**
     * @param delta_time_ns Null for an ordinary sample; otherwise points to the
     *        edge-to-sample offset of an anchor sample, in nanoseconds.  The value is
     *        read during the call only, so the caller may pass a temporary.
     */
    void feed_slave_sample(const uint64_t *delta_time_ns);

    // pop corrected timestamps of ready samples, newest first; returns false when none remain.
    // the first call returns the current frame (anchor sample), then the previous frame, the one before, ...
    /**
     * Pop the newest corrected timestamp.
     * @param[out] slave_timestamp_ns Receives the corrected absolute timestamp,
     *        aligned to the master clock epoch.
     * @return false when @p slave_timestamp_ns is null or nothing is ready, which is the
     *         case until the handshake has completed; true when a timestamp was returned.
     */
    bool get_timestamp(uint64_t *slave_timestamp_ns);

    // number of ready timestamps (for assertion before external pairing)
    /** @return Number of timestamps waiting to be popped or taken. */
    std::size_t ready_count() const;

    // take all ready timestamps (newest first: first element = current anchor sample) and clear them
    /**
     * Remove every ready timestamp and hand them to the caller.
     * @return Newest first, so element 0 is the current anchor sample and the last
     *         element is the oldest one; empty when nothing was ready.
     */
    std::vector<uint64_t> take_ready();

    // clear ready timestamps (discarding unpaired ones)
    /** Discard the ready timestamps without returning them (e.g. when pairing is aborted). */
    void clear_ready();

private:
    // wrap-align to the master clock phase; the caller must hold mtx_ through a public method
    // Reads master_ without taking the lock itself, so it is private and lock-free by contract.
    uint64_t align_master_phase(uint64_t predicted) const;

    // absolute time provided by the master clock side
    struct MasterClock {
        uint64_t first_ns = 0;   // first frame
        uint64_t last_ns = 0;    // latest (phase reference)
    };

    // Startup handshake: two master timestamps and one confirming anchor are needed
    // before any sample can be converted.
    enum class Phase { WaitFirstAnchor, WaitTwoMaster, WaitKeyPoint, Tracking };

    // Slave-side state: the samples seen since the last processed anchor, plus the
    // anchor that started that period.
    struct SlaveCounter {
        Phase phase = Phase::WaitFirstAnchor;
        uint64_t last_anchor_edge_timestamp_ns = 0;   // master timestamp of the previous anchor point
        uint64_t last_anchor_sample_timestamp_ns = 0; // slave timestamp of the previous anchor

        // WaitFirstAnchor
        bool has_first_anchor = false;
        uint64_t first_delta_time_ns = 0;

        // Counters, both reset when an anchor is processed; sample_count is the divisor
        // of the gap computation, so it must never be 0 at that point.
        uint32_t anchor_count = 0;
        uint32_t sample_count = 0;
    };

    uint32_t master_cycle_ns_;    // master clock nominal period (frame period)

    MasterClock master_;
    SlaveCounter slave_;

    std::vector<uint64_t> ready_;    // ready timestamps (get pops from the tail)
    // Invariant: filled oldest to newest within one anchor period, so the tail (back())
    // is the current anchor and the front is the oldest retained sample.

    std::unique_ptr<std::mutex> mtx_;
};

} // namespace base
} // namespace gs130

#endif // GS130_BASE_TRACKER_HPP