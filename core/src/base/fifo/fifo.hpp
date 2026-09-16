/**
 * @file fifo.hpp
 * @brief Bounded, thread-safe software FIFO (header-only).
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_BASE_FIFO_HPP
#define GS130_BASE_FIFO_HPP

#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "types.hpp"

namespace gs130 {
namespace base {

/**
 * Bounded FIFO of copyable items, safe for one or more producers and consumers.
 *
 * The buffer is allocated once, at construction, so pushing never allocates.  Items
 * are stored by value: T must be default-constructible and copy-assignable, one copy
 * is made on push and one on pop.
 *
 * Validity: the SDK deliberately requires a depth of at least 2. A shallower FIFO
 * is rejected silently and remains inert: operator bool() is false, push() fails,
 * and the accessors report the empty state.
 *
 * Thread safety: push(), pop(), size(), empty(), and full() serialize on the
 * per-object mutex. Individual calls are atomic, but sequences of calls are not.
 * Moving or destroying the object while
 * another thread uses it is not safe.
 *
 * Ownership: the FIFO owns the buffered copies.  The optional Disposer hook is
 * called with a mutable reference to an item this FIFO discards -- on overwrite in
 * FifoMode::DropOld and for every item still queued at destruction -- which lets the
 * owner release a resource the item refers to.  The hook runs while the FIFO mutex
 * is held, so it must not call back into this FIFO; the caller must have joined the
 * producer and consumer threads before the destructor runs.
 */
template <typename T>
class Fifo {
public:
    // Optional hook to release an item the FIFO drops or still holds when it dies
    using Disposer = void (*)(T &);

    /**
     * @param depth    Buffer capacity; must be at least 2, see the class note on validity.
     * @param mode     Policy applied once the buffer is full.
     * @param disposer Optional drop hook, or nullptr to dispose of queued items as plain copies.
     */
    Fifo(std::size_t depth, FifoMode mode, Disposer disposer = nullptr): 
        mode_(mode), buf_(depth), valid_(depth >= 2),
        mtx_(std::make_unique<std::mutex>()), disposer_(disposer)
    {    
    }

    // Disposes of the queued items through the hook; not safe to run concurrently with
    // any other member (the user must have joined producer and consumer threads first).
    ~Fifo()
    {
        if(!valid_ || disposer_ == nullptr)return;
        // the producer and the consumer are joined before the FIFO is destroyed
        for(std::size_t i = 0; i < count_; i++)disposer_(buf_[(tail_ + i) % buf_.size()]);
    }

    // copying disabled
    Fifo(const Fifo &) = delete;
    Fifo &operator=(const Fifo &) = delete;

    // define move behavior
    // Not thread-safe: the moved-from object ends up inert (empty buffer, false
    // operator bool()), and the items it handed over are disposed of by the new owner.
    Fifo(Fifo &&other) noexcept: 
        mode_(other.mode_), buf_(std::move(other.buf_)),
        head_(other.head_), tail_(other.tail_), count_(other.count_),
        valid_(other.valid_), mtx_(std::move(other.mtx_)), disposer_(other.disposer_)
    {
        other.valid_ = false;
    }
    // Move-assignment overwrites this object's contents without running the disposer on
    // them; the source is left inert, exactly as after move construction.
    Fifo &operator=(Fifo &&other) noexcept
    {
        if(this != &other){
            mode_  = other.mode_;
            buf_   = std::move(other.buf_);
            head_  = other.head_;
            tail_  = other.tail_;
            count_ = other.count_;
            valid_ = other.valid_;
            mtx_   = std::move(other.mtx_);
            disposer_ = other.disposer_;
            other.valid_ = false;
        }
        return *this;
    }

    /** @return true when the object is usable (depth >= 2). */
    explicit operator bool() const {return valid_;}

    /**
     * Append a copy of @p item.
     *
     * When the queue is full the configured FifoMode decides: DropNew refuses the item
     * and returns false without touching the queued contents, DropOld overwrites the
     * oldest item (after handing it to the disposer) and still reports success.
     *
     * @return true when the item was enqueued, false when it was refused or the object is inert.
     */
    bool push(const T &item)
    {
        if(!valid_) return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        if(full_unlocked()){
            if(mode_ == FifoMode::DropNew)return false;
            if(disposer_)disposer_(buf_[tail_]);   // release the item being overwritten
            tail_ = (tail_ + 1) % buf_.size();   // overwrite the oldest
        }
        else ++count_;

        buf_[head_] = item;
        head_ = (head_ + 1) % buf_.size();
        return true;
    }

    /**
     * Remove the oldest item and copy it into @p item.
     * @param[out] item Left untouched when nothing is popped.
     * @return true when an item was returned, false when the queue was empty or the object is inert.
     */
    bool pop(T &item)
    {
        if(!valid_)return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        if(count_ == 0)
            return false;
        item = buf_[tail_];
        tail_ = (tail_ + 1) % buf_.size();
        --count_;
        return true;
    }

    /** @return Number of queued items; 0 for an inert object. */
    std::size_t size() const
    {
        if(!valid_)return 0;
        std::lock_guard<std::mutex> lock(*mtx_);
        return count_;
    }

    /** @return true when nothing is queued, which is also the answer for an inert object. */
    bool empty() const
    {
        if(!valid_)return true;
        std::lock_guard<std::mutex> lock(*mtx_);
        return count_ == 0;
    }

    /**
     * @return true when the queue holds capacity() items, i.e. the next push() applies the
     *         full policy; false for an inert object.
     */
    bool full() const
    {
        if(!valid_)return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        return full_unlocked();
    }

    /** @return Configured depth, i.e. the constructor argument; lock-free and constant. */
    std::size_t capacity() const {return buf_.size();}

private:
    bool full_unlocked() const {return count_ == buf_.size();}

    FifoMode mode_;
    std::vector<T> buf_;
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t count_ = 0;
    bool valid_;
    std::unique_ptr<std::mutex> mtx_;
    Disposer disposer_;
};

} // namespace base
} // namespace gs130

#endif // GS130_BASE_FIFO_HPP
