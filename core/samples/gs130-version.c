/**
 * @file gs130-version.c
 * @brief Print the SDK version and the build platform
 *
 * usage: gs130-version
 *
 *   Part of the samples built next to this file; it is what 'gs130 version' runs
 *   and it also runs standalone from the directory holding the other samples.
 *   It takes no arguments.
 *
 * output: a single line on stdout, exactly as main() prints it:
 *
 *   gs130_sdk <version> (platform: <platform>)
 *
 *   <version>  the version string baked into the library at build time
 *              (-DGS130_VERSION, read by the Makefile from the VERSION file one
 *              level above the Makefile). It is the version of the library that
 *              answers here, not the .deb version it was installed from.
 *   <platform> the platform directory name selected at build time, e.g. RDKX5
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"
#include <stdio.h>

int main(void)
{
    printf("gs130_sdk %s (platform: %s)\n", gs130_version(), gs130_platform());
    return 0;
}
