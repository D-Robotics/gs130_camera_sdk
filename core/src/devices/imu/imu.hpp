/**
 * @file imu.hpp
 * @brief Model-independent IMU discovery and FIFO interface.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_DEVICES_IMU_IMU_HPP
#define GS130_DEVICES_IMU_IMU_HPP

#include <cstddef>
#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace gs130 {
namespace imu {

/** Operations and identification data supplied by one IMU model driver. */
struct ModelDesc {
    uint8_t who_am_i_addr; /**< Register used to identify the model. */
    uint8_t who_am_i_val; /**< Expected value of the identification register. */
    const char  *name; /**< Statically allocated model name. */
    const char  *info; /**< Statically allocated, human-readable capabilities. */

    Status (*init)  (base::I2cDevice &bus, const ImuConfig &cfg); /**< Configure the sensor without starting its FIFO. */
    Status (*start) (base::I2cDevice &bus); /**< Begin FIFO streaming. */
    void   (*stop)  (base::I2cDevice &bus); /**< Stop FIFO streaming. */
    Status (*read)  (base::I2cDevice &bus, ImuHwFifo16Packet *out,
                     std::size_t cap, std::size_t *n_out); /**< Read at most cap packets. */
    bool   (*full)(base::I2cDevice &bus); /**< Report and acknowledge FIFO overflow as required by the model. */
    void   (*deinit)(base::I2cDevice &bus); /**< Restore the sensor to its idle state. */
};

/**
 * Model-independent IMU wrapper around one I2C device.
 *
 * Construction opens the bus and probes registered models in registration order.
 * The object owns the I2C descriptor. It is not thread-safe; callers must serialize
 * configuration, streaming, and FIFO access.
 */
class Imu {
public:
    /** Open bus/addr and bind the first registered model whose WHO_AM_I value matches. */
    Imu(uint8_t bus, uint8_t addr);
    /** Run the bound model's deinitialization callback, then close the I2C device. */
    ~Imu();

    // Non-copyable: the object exclusively owns its I2C descriptor.
    Imu(const Imu &)            = delete;
    Imu &operator=(const Imu &) = delete;

    /** @return true when a supported model was identified. */
    explicit operator bool() const { return desc_ != nullptr; }

    /** Configure the bound model; does not start FIFO streaming. */
    Status init(const ImuConfig &cfg);
    /** Start FIFO streaming on the bound model. */
    Status start();
    /** Stop FIFO streaming; model callbacks intentionally report no status. */
    void   stop();

    /**
     * Read up to cap hardware FIFO packets into caller-owned storage.
     * @param[out] out Packet array with room for cap elements.
     * @param[in] cap Maximum number of packets to read; must be non-zero.
     * @param[out] n_out Actual number of packets written; set to zero when none are available.
     */
    Status read(ImuHwFifo16Packet *out, std::size_t cap, std::size_t *n_out);

    /** @return true when the hardware FIFO reports overflow/full; false also covers read failure. */
    bool full();

    /** @return Statically allocated name of the bound model. */
    const char *name() const;
    /** @return Statically allocated capabilities of the bound model. */
    const char *info() const;

private:
    base::I2cDevice  bus_;
    const ModelDesc *desc_ = nullptr;
};

/**
 * Append a statically allocated model descriptor to the process-wide registry.
 * Model translation units call this during static initialization; descriptors must
 * remain valid for the lifetime of the process.
 */
bool register_model(const ModelDesc *desc);

} // namespace imu
} // namespace gs130

// Register a namespace-scope ModelDesc from its model .cpp during static initialization.
#define GS130_IMU_REGISTER_MODEL(DESC) \
    namespace { [[maybe_unused]] const bool gs130_reg_##DESC = ::gs130::imu::register_model(&DESC); }

#endif // GS130_DEVICES_IMU_IMU_HPP
