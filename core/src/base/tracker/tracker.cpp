/**
 * @file tracker.cpp
 * @brief TimestampTracker implementation: handshake state machine and phase alignment.
 *
 * Output timestamps use the master's absolute epoch while interpolating slave-sample
 * spacing between aligned FSYNC anchors. The tracker starts from the master edge of
 * the previous anchor, predicts the
 * current edge by counting anchors on the nominal master period, snaps that prediction
 * to the newest master timestamp, and interpolates the corrections for the samples
 * taken since the previous anchor.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/tracker/tracker.hpp"
#include <cstdio>

namespace gs130 {
namespace base {

TimestampTracker::TimestampTracker(uint32_t master_cycle_ns)
    : master_cycle_ns_(master_cycle_ns), mtx_(std::make_unique<std::mutex>())
{}

// The first master timestamp doubles as the phase reference: it is kept as first_ns and
// stays the "previous anchor edge" until a real anchor arrives.
void TimestampTracker::update_master_timestamp_ns(
    uint64_t timestamp_ns)
{
    std::lock_guard<std::mutex> lock(*mtx_);
    if(master_.first_ns == 0)master_.first_ns = timestamp_ns;
    master_.last_ns = timestamp_ns;

}

// Lock-free; the caller (a public method) holds the lock
// Snaps a predicted edge onto the master clock: the shift is always a whole number of
// nominal periods, so the result stays on the master phase grid while being the edge
// closest to the prediction (within half a period, the correction is not expected to
// exceed that).
uint64_t TimestampTracker::align_master_phase(
    uint64_t predicted) const
{
    uint64_t aligned = master_.last_ns;

    while(aligned > predicted + master_cycle_ns_ / 2)aligned -= master_cycle_ns_;
    while(aligned < predicted - master_cycle_ns_ / 2)aligned += master_cycle_ns_;

    return aligned;
}

// Handshake: the tracker only starts producing timestamps after an anchor sample has
// been seen, two master timestamps have arrived, and another anchor has confirmed the
// key point.  Until then samples are counted and dropped.
void TimestampTracker::feed_slave_sample(
    const uint64_t *delta_time_ns)
{
    std::lock_guard<std::mutex> lock(*mtx_);
    const bool fsync = (delta_time_ns != nullptr);

    switch(slave_.phase){
    case Phase::WaitFirstAnchor: // wait for the first anchor
        // first-anchor branch
        if(fsync && !slave_.has_first_anchor){
            slave_.has_first_anchor = true;
            slave_.first_delta_time_ns = *delta_time_ns;
            slave_.phase = Phase::WaitTwoMaster;
        }
        return ;

    case Phase::WaitTwoMaster:
        // wait for two Master clocks to arrive
        if(master_.first_ns != 0 && master_.first_ns < master_.last_ns)
            slave_.phase = Phase::WaitKeyPoint;

        [[fallthrough]];

    case Phase::WaitKeyPoint: // key frame after two Masters
        // both Phase::WaitTwoMaster and Phase::WaitKeyPoint conditions are met
        if(fsync && slave_.phase == Phase::WaitKeyPoint){
            slave_.phase = Phase::Tracking;
            slave_.last_anchor_edge_timestamp_ns = master_.first_ns;
            slave_.last_anchor_sample_timestamp_ns = master_.first_ns + slave_.first_delta_time_ns;
        }

        // count during the wait phases
        if(fsync)slave_.anchor_count++;
        slave_.sample_count++;

        // intercept non-anchor frames
        if(slave_.phase != Phase::Tracking)return;
        
        // the anchor frame happens to share the Tracking-phase computation
        [[fallthrough]];

    case Phase::Tracking:
        // anchor-frame computation
        if(fsync){
            // Predicted edge of the current anchor, snapped to the master phase, then
            // shifted by the edge-to-sample offset to reach the slave-clock value.
            uint64_t anchor_edge_timestamp_ns = 
                slave_.last_anchor_edge_timestamp_ns + slave_.anchor_count * master_cycle_ns_;
            anchor_edge_timestamp_ns = align_master_phase(anchor_edge_timestamp_ns);
            uint64_t anchor_sample_timestamp_ns = anchor_edge_timestamp_ns + *delta_time_ns;

            // compute the slave-clock interval
            // Mean interval over the samples seen since the previous anchor, measured on
            // the slave clock; it spans the anchor period, so it absorbs the drift between
            // the two clocks.
            uint64_t slave_gap_ns = 
                (anchor_sample_timestamp_ns - slave_.last_anchor_sample_timestamp_ns) / slave_.sample_count;

            // push into ready_: ordinary samples step by gap; the last one is the current anchor
            for(uint32_t j = 1; j <= slave_.sample_count; j++)
                ready_.push_back(slave_.last_anchor_sample_timestamp_ns + slave_gap_ns * j);

            slave_.last_anchor_edge_timestamp_ns = anchor_edge_timestamp_ns;
            slave_.last_anchor_sample_timestamp_ns = anchor_sample_timestamp_ns;

            slave_.sample_count = 0;
        }

        // count the current sample (the one with FSYNC) into the new period
        slave_.sample_count++;
        slave_.anchor_count = 1;
        return ;
    }
}

// Newest entry first: ready_ is filled oldest to newest inside one anchor period.
bool TimestampTracker::get_timestamp(uint64_t *slave_timestamp_ns)
{
    std::lock_guard<std::mutex> lock(*mtx_);
    if(!slave_timestamp_ns || ready_.empty())return false;
    *slave_timestamp_ns = ready_.back();
    ready_.pop_back();
    return true;
}

std::size_t TimestampTracker::ready_count() const
{
    std::lock_guard<std::mutex> lock(*mtx_);
    return ready_.size();
}

std::vector<uint64_t> TimestampTracker::take_ready()
{
    std::lock_guard<std::mutex> lock(*mtx_);
    std::vector<uint64_t> out;
    out.reserve(ready_.size());
    // newest to oldest: back -> front; first element = current anchor sample
    for(auto it = ready_.rbegin(); it != ready_.rend(); ++it)
        out.push_back(*it);
    ready_.clear();
    return out;
}

void TimestampTracker::clear_ready()
{
    std::lock_guard<std::mutex> lock(*mtx_);
    ready_.clear();
}

} // namespace base
} // namespace gs130
