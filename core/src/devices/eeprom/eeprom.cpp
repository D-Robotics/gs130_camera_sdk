/**
 * @file eeprom.cpp
 * @brief EEPROM model registry table and probe loop.
 *
 * The registry is filled by the model files themselves through
 * GS130_EEPROM_REGISTER_MODEL() at static-init time; the probe loop then runs in that
 * registration order, which is the link order of the model objects and is therefore not
 * fixed by this file.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/eeprom/eeprom.hpp"

#include <vector>

namespace gs130 {
namespace eeprom {

namespace {

// Auto-populated model registry: each model file self-registers via
// GS130_EEPROM_REGISTER_MODEL at static-init time.
// Function-local static so that the vector exists before the first registration, whatever
// order the model translation units are initialised in; it holds borrowed pointers only.
std::vector<const ModelDesc *> &registry()
{
    static std::vector<const ModelDesc *> r;   // function-local static: safe init order
    return r;
}

} // namespace

// Appends the borrowed descriptor; the true return value exists only so the
// self-registration macro can initialise a bool.
bool register_model(const ModelDesc *desc)
{
    registry().push_back(desc);
    return true;
}

// The first model whose probe accepts the device wins; a device with no matching model
// leaves desc_ null, i.e. the object is invalid and read() must not be called.
Eeprom::Eeprom(uint8_t bus, uint8_t addr)
    : bus_(bus, addr)
{
    if(!bus_)return;

    for(const ModelDesc *m : registry()){
        if(!m->probe(bus_))continue;
        desc_ = m;
        return;
    }
}

// The three forwarders below require a bound model; the caller's guard is operator bool(),
// which the public API checks before it calls them.
Status Eeprom::read(StereoImuModel *out){return desc_->read(bus_, out);}

const char *Eeprom::name() const{return desc_->name;}

const char *Eeprom::info() const{return desc_->info;}

} // namespace eeprom
} // namespace gs130
