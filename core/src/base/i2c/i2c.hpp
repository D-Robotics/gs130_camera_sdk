/**
 * @file i2c.hpp
 * @brief RAII wrapper for Linux i2c-dev; **NOT** thread-safe.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_BASE_I2C_HPP
#define GS130_BASE_I2C_HPP

#include <cstdint>

#include "types.hpp"

namespace gs130 {
namespace base {

/**
 * Owns one open /dev/i2c-<bus> file descriptor bound to a single slave address.
 *
 * The constructor never fails hard: when the bus cannot be opened or the address
 * cannot be selected the object stays closed, which the caller detects through
 * operator bool() or is_open().  All accessors are const because the wrapper holds
 * no state of its own beyond the descriptor; a transaction is the write/read pair
 * chosen by the device protocol.
 *
 * Thread safety: none.  One I2cDevice must not be used from two threads at once,
 * and transactions are not atomic -- interleaved access to the same bus from another
 * object or process can corrupt the device state.
 *
 * Ownership: the object owns the descriptor and releases it in close(), in the
 * move-assignment target, and in the destructor.  Moving transfers the descriptor
 * and leaves the source closed; the slave address is not re-programmed.
 */
class I2cDevice {
public:
    /**
     * Open /dev/i2c-<bus> and select the slave address.
     * @param bus  I2C bus number, i.e. the '<bus>' of /dev/i2c-<bus>.
     * @param addr 7-bit slave address; not validated, it is handed to the driver
     *             as-is (addresses above 0x7F are meaningless on a 7-bit bus).
     */
    I2cDevice(uint8_t bus, uint8_t addr);
    ~I2cDevice();

    // copying disabled
    I2cDevice(const I2cDevice &)            = delete;
    I2cDevice &operator=(const I2cDevice &) = delete;

    // moving allowed
    I2cDevice(I2cDevice &&other) noexcept;
    I2cDevice &operator=(I2cDevice &&other) noexcept;

    // boolean conversion semantics: true = success; false = failure
    /** Same condition as is_open(); provided so that a constructed device can be tested directly. */
    explicit operator bool() const { return fd_ >= 0; }

    /** @return true while a slave address is selected on an open bus. */
    bool is_open() const { return fd_ >= 0; }
    /** Close the bus if open; idempotent.  Every later transaction fails with Status::ParamError. */
    void close();

    /**
     * Read one 8-bit register.
     * @param[in]  reg Register address (8-bit address phase).
     * @param[out] val Receives the register value.
     * @return Status::Ok, Status::ParamError when closed or @p val is null,
     *         Status::HwError when the address phase or the read fails.
     */
    Status read(uint8_t reg, uint8_t *val) const;

    /**
     * Read a 16-bit register holding a 16-bit value, big-endian on the bus (MSB first).
     * @param[in]  reg Register address, transmitted MSB first.
     * @param[out] val Receives the value reassembled in host byte order.
     * @return Status::Ok, Status::ParamError when closed or @p val is null,
     *         Status::HwError when the address phase or the read fails.
     */
    Status read16(uint16_t reg, uint16_t *val) const;

    /**
     * Write one 8-bit register.
     * @param[in] reg Register address.
     * @param[in] val Value to store.
     * @return Status::Ok, Status::ParamError when closed, Status::HwError on failure.
     */
    Status write(uint8_t reg, uint8_t val) const;

    /**
     * Read-modify-write one register: the bits of @p val selected by @p mask replace
     * the corresponding bits of the current value, every other bit is preserved.
     * The write is skipped when the result equals the value just read.
     * @param[in] reg  Register address.
     * @param[in] mask Bit mask (1 = take the bit from @p val), already in bit position.
     * @param[in] val  New bit values, already shifted into position.
     * @return Status::Ok, or the failure of the underlying read/write.
     */
    Status update(uint8_t reg, uint8_t mask, uint8_t val) const;

    /**
     * Write a 16-bit register and a 16-bit value, both big-endian (MSB first).
     * @param[in] reg Register address, transmitted MSB first.
     * @param[in] val Value to store, transmitted MSB first.
     * @return Status::Ok, Status::ParamError when closed, Status::HwError on failure.
     */
    Status write16(uint16_t reg, uint16_t val) const;

    /**
     * Read @p len consecutive bytes starting at an 8-bit register address.
     * @param[in]  reg Start register address.
     * @param[out] buf Destination, at least @p len bytes.
     * @param[in]  len Byte count, must not be 0.
     * @return Status::Ok, Status::ParamError when closed or @p buf is null or
     *         @p len is 0, Status::HwError when a short transfer completes.
     */
    Status readBurst(uint8_t reg, uint8_t *buf, uint32_t len) const;

    /**
     * Read one byte from a 16-bit register address (only the address phase is 16-bit).
     * @param[in]  reg Register address, transmitted MSB first.
     * @param[out] val Receives the byte.
     * @return Status::Ok, Status::ParamError when closed or @p val is null,
     *         Status::HwError when the address phase or the read fails.
     */
    Status readReg16(uint16_t reg, uint8_t *val) const;

    /**
     * Write one byte to a 16-bit register address (only the address phase is 16-bit).
     * @param[in] reg Register address, transmitted MSB first.
     * @param[in] val Value to store.
     * @return Status::Ok, Status::ParamError when closed, Status::HwError on failure.
     */
    Status writeReg16(uint16_t reg, uint8_t val) const;

    /**
     * Read @p len consecutive bytes starting at a 16-bit register address.
     * @param[in]  reg Start register address, transmitted MSB first.
     * @param[out] buf Destination, at least @p len bytes.
     * @param[in]  len Byte count, must not be 0.
     * @return Status::Ok, Status::ParamError when closed or @p buf is null or
     *         @p len is 0, Status::HwError when a short transfer completes.
     */
    Status readBurst16(uint16_t reg, uint8_t *buf, uint32_t len) const;

private:
    int fd_ = -1;
};

} // namespace base
} // namespace gs130

#endif // GS130_BASE_I2C_HPP