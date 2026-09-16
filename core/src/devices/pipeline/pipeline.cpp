/**
 * @file pipeline.cpp
 * @brief Platform dispatch: the platform backend .cpp is #included here, by path.
 *
 * This translation unit holds no code of its own. The build system selects the
 * platform by directory name and passes the backend file path as
 * -DGS130_PLATFORM_IMPL="devices/pipeline/<platform>/<platform>.cpp".
 * Adding a platform = create src/devices/pipeline/<name>/<name>.cpp; nothing else to touch.
 *
 * The backend defines gs130::pipeline::Pipeline, declares its node helpers in
 * <platform>/<platform>.h and keeps that header out of every other translation unit
 * (it is part of this file's include path only).
 *
 * Note: the backend file is not compiled on its own -- it must not be added to a
 * source list, and it cannot be compiled without the platform headers.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/pipeline/pipeline.hpp"

#ifndef GS130_PLATFORM_IMPL
#  error "No platform selected: the Makefile defines GS130_PLATFORM_IMPL from the platform directory name"
#endif

#include GS130_PLATFORM_IMPL
