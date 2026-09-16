/**
 * @file pipeline.hpp
 * @brief Platform-independent stereo camera pipeline interface.
 *
 * Platform-independent front end of the stereo camera pipeline: this header declares
 * the handle the SDK layer uses, and the backend that implements it is chosen at
 * build time (see pipeline.cpp). It is the only pipeline header that is compiled for
 * every platform, so no platform type may appear here -- configuration, status codes
 * and camera indices all come from types.hpp.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_DEVICES_PIPELINE_PIPELINE_HPP
#define GS130_DEVICES_PIPELINE_PIPELINE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "types.hpp"

namespace gs130 {
namespace pipeline {

/**
 * One stereo camera pair: CamIndex::Right and CamIndex::Left.
 *
 * Lifecycle: the constructor probes the sensors only; init() acquires hardware and
 * builds the stream, start() begins streaming, stop() stops it, and deinit() tears
 * down a successfully initialized stream. The destructor also calls deinit().
 *
 * Ownership: the object owns the hardware of a successfully initialized stream.
 * init() may update the caller-owned calibration in place but does not retain its
 * pointer. Buffers passed to get_frame() remain owned by the caller.
 *
 * Thread safety: none. Members must not be called concurrently; the SDK layer
 * serializes the calls from a single thread.
 */
class Pipeline {
public:
    /**
     * @brief Probe the candidate buses for both sensors; acquires no hardware.
     *
     * An eye counts as found when its chip id answers at the given I2C address. The
     * first bus in @p bus_list that answers for an eye wins, but every bus is still
     * scanned until both eyes have been found.
     *
     * @param[in] left_addr  I2C address of the left sensor.
     * @param[in] right_addr I2C address of the right sensor.
     * @param[in] bus_list   Candidate I2C bus numbers, probed in array order.
     * @param[in] bus_num    Number of valid entries in @p bus_list.
     */
    Pipeline(uint8_t left_addr, uint8_t right_addr,
             const uint8_t *bus_list, std::size_t bus_num);
    /** @brief Release the pipeline; invokes deinit() as a final teardown attempt. */
    ~Pipeline();

    Pipeline(const Pipeline &)            = delete;
    Pipeline &operator=(const Pipeline &) = delete;

    /** @brief true when both sensors were found during construction, i.e. when init() can run. */
    explicit operator bool() const;

    /**
     * @brief Acquire the hardware and build the stream; streaming stays off.
     *
     * @param[in]     cfg Geometry, output mode, timing, and the I2C-bus -> MIPI RX /
     *                    reset GPIO map of the buses the sensors were found on.
     * @param[in,out] cal Calibration of the pair; owned by the caller, which must keep
     *                    it valid for the duration of this call. Required unless the
     *                    mode is Raw. Raw never touches it; any other mode rescales the
     *                    intrinsics in place to the output geometry (VSE crop + scale).
     * @return Status::Ok on success; Status::ParamError for an already initialized
     *         pipeline, a size the mode does not allow, a missing calibration, an
     *         unmapped bus, or an install angle that is not a multiple of 90 degrees;
     *         Status::NotFound when the sensors were not probed; Status::Unsupported for
     *         a geometry the hardware cannot produce; Status::HwError when a node cannot
     *         be opened. See deinit() for the cleanup guarantee.
     */
    Status init(const PipelineConfig &cfg, StereoImuModel *cal);

    /**
     * @brief Tear down a successfully initialized stream; also called by the destructor.
     *
     * @warning The current backend marks the stream initialized only after all nodes
     *          are built. Consequently, this function is a no-op during partial
     *          initialization and is not a cleanup guarantee for an init() failure.
     */
    void   deinit();

    /**
     * @brief Start streaming, beginning with @p first and then the other eye.
     *
     * @param[in] first Channel started first; the SDK layer passes the FSYNC-bound eye
     *                  (the master clock) here.
     * @return Status::Ok, Status::ParamError when the stream is not initialized, or
     *         Status::HwError when a flow fails to start.
     */
    Status start(CamIndex first);

    /** @brief Stop streaming; the stream stays initialized and can be started again. */
    void   stop();

    /**
     * @brief Copy the next frame of one eye into caller-owned buffers.
     *
     * Rows are copied one by one, so a buffer stride may exceed the frame width.
     *
     * @param[in]  idx          Eye to read from; must be CamIndex::Right or CamIndex::Left.
     * @param[out] y            Luma plane: @p height rows of @p y_stride bytes; the
     *                          caller owns the buffer and sizes it accordingly.
     * @param[out] uv           Interleaved chroma plane (NV12): @p height / 2 rows of
     *                          @p uv_stride bytes; caller-owned.
     * @param[in]  width        Expected frame width in pixels; must match the device frame.
     * @param[in]  height       Expected frame height in pixels; must match the device frame.
     * @param[in]  y_stride     Bytes per row of @p y; must be at least @p width.
     * @param[in]  uv_stride    Bytes per row of @p uv; must be at least @p width.
     * @param[out] timestamp_ns Receives the frame timestamp in nanoseconds.
     * @param[in]  timeout_ms   Maximum time to wait for a frame, in milliseconds.
     * @retval Status::Ok Frame copied.
     * @retval Status::ParamError A null buffer or timestamp, an uninitialized stream,
     *         or a width/height that does not match the device frame.
     * @retval Status::Timeout No frame arrived within @p timeout_ms.
     */
    Status get_frame(CamIndex idx,
                     uint8_t *y, uint8_t *uv,
                     uint32_t width, uint32_t height,
                     uint32_t y_stride, uint32_t uv_stride,
                     uint64_t *timestamp_ns,
                     uint32_t timeout_ms);

private:
    // Pimpl: defined by the selected backend, so no platform header is needed here
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace pipeline
} // namespace gs130

#endif // GS130_DEVICES_PIPELINE_PIPELINE_HPP
