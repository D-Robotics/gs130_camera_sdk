/**
 * @file eeprom.hpp
 * @brief EEPROM reader for binocular (IMU) distortion and stereo calibration parameters.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_DEVICES_EEPROM_EEPROM_HPP
#define GS130_DEVICES_EEPROM_EEPROM_HPP

#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace gs130 {
namespace eeprom {

/**
 * One EEPROM layout (vendor + model + version), used as the vtable of a detected device.
 *
 * The structure and both strings are read-only for the lifetime of the program: they are
 * static data provided by a model .cpp, never copied and never freed by this module.
 */
struct ModelDesc{
    /** Statically allocated model name exposed through the public API. */
    const char *name;
    /** Multi-line description handed to the user, one "- field: value" item per line. */
    const char *info;

    /**
     * Probe hook: read the identification area and return true when the device matches
     * this layout.  Must not modify the bus state beyond reading, and must be safe to call
     * on a mismatching device (it is called for every registered model in turn).
     */
    bool   (*probe)(base::I2cDevice &bus);
    /** Read hook: fill @p out completely, see Eeprom::read() for the contract. */
    Status (*read) (base::I2cDevice &bus, StereoImuModel *out);
};

/**
 * Calibration EEPROM attached to an I2C bus.
 *
 * Construction probes the registered models against the device at @p addr and binds the
 * first match; the caller detects failure through operator bool().  All accessors except
 * name()/info() talk to the hardware, so they need a bound model and a usable bus.
 *
 * Thread safety: none.  The embedded base::I2cDevice is not thread-safe, and the bound
 * model is fixed at construction but not synchronised with later access.
 */
class Eeprom {
public:
    /**
     * @param bus  I2C bus number the EEPROM is wired to.
     * @param addr 7-bit I2C address of the EEPROM (typically 0x50).
     */
    Eeprom(uint8_t bus, uint8_t addr);
    ~Eeprom() = default;

    // non-copyable
    Eeprom(const Eeprom &)            = delete;
    Eeprom &operator=(const Eeprom &) = delete;

    // bool conversion: true = a model is bound; false = probe failed
    /** @return true when a registered model matched the device. */
    explicit operator bool() const { return desc_ != nullptr; }

    /**
     * Read the calibration from the detected device.
     * @param[out] out Receives the parsed calibration; every field is written, so the
     *             caller's previous contents are replaced.
     * @return Status::Ok, Status::ParamError when @p out is null, or
     *         Status::HwError when the EEPROM transfer fails.
     * @pre The object is valid (operator bool() is true); the bound model provides the
     *      hook, and calling this on a probe failure is undefined behaviour.  The caller
     *      must check the object first.
     */
    Status read(StereoImuModel *out);

    /** @return Model string of the bound model; never null for a valid object. */
    const char *name() const;
    /** @return Description string of the bound model; never null for a valid object. */
    const char *info() const;

private:
    base::I2cDevice  bus_;
    const ModelDesc *desc_ = nullptr;
};

/**
 * Register a model layout in the process-wide registry.
 *
 * Called from the self-registration macro of a model .cpp during static initialisation,
 * so a new model file needs no edit here.  The registry keeps only the pointer: @p desc
 * must refer to storage with static lifetime, and the module never takes ownership of it.
 *
 * @param[in] desc Layout to append; must not be null and must stay valid forever.
 * @return Always true, so that the macro can initialise a bool at file scope.
 */
bool register_model(const ModelDesc *desc);

} // namespace eeprom
} // namespace gs130

// Self-registration macro: call at file scope in a model .cpp, after the ModelDesc definition.
// Expands to a file-local bool, so the descriptor must be a namespace-scope object with
// static lifetime; the macro is a no-op at runtime and its result may be ignored.
#define GS130_EEPROM_REGISTER_MODEL(DESC) \
    namespace { [[maybe_unused]] const bool gs130_reg_##DESC = ::gs130::eeprom::register_model(&DESC); }

#endif // GS130_DEVICES_EEPROM_EEPROM_HPP
