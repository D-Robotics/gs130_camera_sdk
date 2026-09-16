/**
 * @file gs130-detect-camera.cpp
 * @brief Scan I2C buses x addresses for an SC132GS camera sensor.
 *
 * Internal bring-up tool, dispatched by the @c gs130 script as
 * @c "gs130 detect camera": every (bus, address) probe point prints exactly one
 * result line (the chip id on a hit), so the output also shows where nothing
 * answered or where the bus itself could not be opened.
 *
 * Usage: gs130 detect camera -b <bus...> [-a <addr...>]
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/i2c/i2c.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace gs130;

// SC132GS sensor chip identification: the register and value the pipeline probes with
// (see Pipeline::Pipeline), read back as a 16-bit big-endian register value
static constexpr uint16_t kChipIdReg = 0x3107;
static constexpr uint16_t kChipId    = 0x0132;

/**
 * @brief Parse one command-line integer argument.
 *
 * @param[in]  s   Text to parse.
 * @param[out] out Receives the parsed value.
 * @return true when @p s was consumed completely.
 */
static bool parse_int(const char *s, int *out)
{
    char *end = nullptr;
    long v = strtol(s, &end, 0);   // base 0: decimal, 0x-hex, 0-octal
    if(end == s || *end != '\0')return false;
    *out = (int)v;
    return true;
}

/**
 * @brief Probe every requested (bus, address) pair, one result line per point.
 *
 * @param[in] argc,argv @c -b <bus...> (required) and @c -a <addr...> (optional);
 *                      each option takes the values that follow it until the next
 *                      argument starting with '-'.
 * @return 0 when the scan completed (also when no sensor was found), 1 on a usage
 *         or argument error.
 */
int main(int argc, char **argv)
{
    std::vector<int> buses, addrs;
    for(int i = 1; i < argc; i++){
        if(!strcmp(argv[i], "-b") || !strcmp(argv[i], "-a")){
            std::vector<int> &dst = (argv[i][1] == 'b') ? buses : addrs;
            while(i + 1 < argc && argv[i+1][0] != '-'){
                int v;
                if(!parse_int(argv[++i], &v)){ fprintf(stderr, "bad number: %s\n", argv[i]); return 1; }
                dst.push_back(v);
            }
        }
    }
    if(buses.empty()){
        fprintf(stderr, "usage: gs130 detect camera -b <bus...> [-a <addr...>]   (addr default: 0x30 0x32)\n");
        return 1;
    }
    // default probe addresses: the left/right sensor addresses used by the SDK presets
    if(addrs.empty())addrs = {0x30, 0x32};

    printf("CAMERA scan: buses");
    for(int b : buses)printf(" %d", b);
    printf(", addrs");
    for(int a : addrs)printf(" 0x%02x", a);
    printf("\n bus  addr    result\n ---  -----   ----------------\n");

    int n_found = 0, n_unknown = 0, n_empty = 0, n_buserr = 0;
    for(int b : buses){
        base::I2cDevice probe((uint8_t)b, (uint8_t)addrs.front());
        if(!probe){
            printf(" %3d  --      bus open failed\n", b);
            n_buserr++;
            continue;
        }
        for(int a : addrs){
            base::I2cDevice d((uint8_t)b, (uint8_t)a);
            uint16_t id = 0;
            const bool ok = d && (d.read16(kChipIdReg, &id) == Status::Ok);
            if(ok && id == kChipId){
                printf(" %3d  0x%02x   chip-id 0x%04x\n", b, a, id);
                n_found++;
            } else if(ok){
                printf(" %3d  0x%02x   unknown device (id 0x%04x)\n", b, a, id);
                n_unknown++;
            } else {
                printf(" %3d  0x%02x   no device\n", b, a);
                n_empty++;
            }
        }
    }
    printf("found %d, unknown %d, empty %d, bus error %d\n", n_found, n_unknown, n_empty, n_buserr);
    return 0;
}
