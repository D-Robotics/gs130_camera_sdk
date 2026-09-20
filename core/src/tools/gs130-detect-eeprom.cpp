/**
 * @file gs130-detect-eeprom.cpp
 * @brief Scan I2C buses x addresses for a known calibration EEPROM.
 *
 * Internal bring-up tool, dispatched by the @c gs130 script as
 * @c "gs130 detect eeprom": every (bus, address) probe point prints exactly one
 * result line (model name plus its details on a hit), so the output also shows
 * where an unknown device answered and where nothing answered at all.
 *
 * Usage: gs130 detect eeprom -b <bus...> [-a <addr...>]
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/i2c/i2c.hpp"
#include "devices/eeprom/eeprom.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace gs130;

namespace {

// Identification area layout: 15 header bytes [0..14] followed by the checksum byte
// [15].  The authoritative definition of the sizes and of the checksum expression is
// the model file devices/eeprom/union_stereo_imu_fisheye_v1p2r0n4.cpp
// (kHeaderSize / kChecksumOff / kChecksumLen and its probe()); the constants below are
// read-only mirrors used by the miss branch.  Keep them in sync with that file.
constexpr uint16_t kIdAreaSize = 16;   // identification area, addressed 0x0000
constexpr uint8_t kChecksumOff = 15;   // checksum follows the header
constexpr uint8_t kChecksumLen = 14;   // only the first 14 bytes take part
constexpr uint8_t kVendorLen   = 8;    // vendor string, NUL padded within these 8

// Mirror of the model checksum: sum of the first 14 bytes, then (sum % 255) + 1.
static uint8_t id_area_checksum(const uint8_t *raw)
{
    uint16_t sum = 0;
    for(uint8_t i = 0; i < kChecksumLen; i++)sum = (uint16_t)(sum + raw[i]);
    return (uint8_t)((sum % 255) + 1);
}

// True when the identification area carries a self-consistent header.  The registered
// model is matched by Eeprom's own probe; this gate only answers "does the 16-byte
// block look like a header at all", so an unregistered model passes it too.
static bool id_area_ok(const uint8_t *raw)
{
    return id_area_checksum(raw) == raw[kChecksumOff];
}

// Vendor field of the identification area as a NUL-terminated bounded copy, so it can
// be printed with %.8s; the 16-byte read buffer is never handed to %s.
static void id_area_vendor(const uint8_t *raw, char *out /* [kVendorLen + 1] */)
{
    for(uint8_t i = 0; i < kVendorLen; i++)out[i] = (char)raw[i];
    out[kVendorLen] = '\0';
}

// Result-column text of one probe point whose identification area checks out but no
// registered model matched (no leading row prefix).
static void miss_text(const uint8_t *raw, char *out, size_t cap)
{
    char vendor[kVendorLen + 1];
    id_area_vendor(raw, vendor);
    snprintf(out, cap, "%.8s unknown(0x%02x,0x%02x) V%u.%u Rotate-%u-deg %u-Distortion-parameters",
             vendor, raw[8], raw[9], raw[10], raw[11], raw[12], raw[13]);
}

} // namespace

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

// print a (possibly multi-line) string, indenting every line under the result column
static void print_block(const char *indent, const char *text)
{
    if(!text)return;
    for(const char *p = text; *p; ){
        const char *nl = strchr(p, '\n');
        int len = nl ? (int)(nl - p) : (int)strlen(p);
        printf("%s%.*s\n", indent, len, p);
        if(!nl)break;
        p = nl + 1;
    }
}

/**
 * @brief Probe every requested (bus, address) pair, one result line per point.
 *
 * @param[in] argc,argv @c -b <bus...> (required) and @c -a <addr...> (optional);
 *                      each option takes the values that follow it until the next
 *                      argument starting with '-'.
 * @return 0 when the scan completed (also when no EEPROM was found), 1 on a usage
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
        fprintf(stderr, "usage: gs130 detect eeprom -b <bus...> [-a <addr...>]   (addr default: 0x50)\n");
        return 1;
    }
    // default probe address: the calibration EEPROM address used by the SDK presets
    if(addrs.empty())addrs = {0x50};

    printf("EEPROM scan: buses");
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
            // constructor probes the auto-registered model table on this bus/addr
            eeprom::Eeprom ep((uint8_t)b, (uint8_t)a);
            if(ep){
                printf(" %3d  0x%02x   %s\n", b, a, ep.name());
                print_block("                ", ep.info());
                n_found++;
            } else {
                // No registered model matched.  The identification area decides the
                // outcome: a self-consistent 16-byte block carries a header whose model
                // is simply not registered here, anything else is reported as absent.
                base::I2cDevice d((uint8_t)b, (uint8_t)a);
                uint8_t v;
                if(!d) {
                    printf(" %3d  0x%02x   no device\n", b, a);
                    n_empty++;
                } else if(d.read(0x00, &v) != Status::Ok) {
                    printf(" %3d  0x%02x   no device\n", b, a);
                    n_empty++;
                } else {
                    uint8_t raw[kIdAreaSize];
                    char text[128];
                    if(d.readBurst16(0x0000, raw, kIdAreaSize) == Status::Ok
                       && id_area_ok(raw)) {
                        miss_text(raw, text, sizeof(text));
                        printf(" %3d  0x%02x   %s\n", b, a, text);
                        n_unknown++;
                    } else {
                        printf(" %3d  0x%02x   no device\n", b, a);
                        n_empty++;
                    }
                }
            }
        }
    }
    printf("found %d, unknown %d, empty %d, bus error %d\n", n_found, n_unknown, n_empty, n_buserr);
    return 0;
}
