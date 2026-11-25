// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Standalone HMD driver.
 * @author Nathaniel Reilly <nathanmreilly@gmail.com>
 * @ingroup drv_standalone
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_standalone Standalone HMD driver
 * @ingroup drv
 *
 * @brief Driver for a Standalone HMD.
 *
 * Not detectable by USB VID/PID, the device is created by the standalone
 * builder (see t_builder_standalone_create) instead of a prober.
 *
 * See @ref writing-driver for additional information.
 */

struct xrt_tracked_slam;

/*!
 * Create a Standalone HMD.
 *
 * @param slam Optional tracker owned by the system's frame context.
 *
 * This is only exposed so that the builder (in the target)
 * can call the construction function (in the driver)
 * @ingroup drv_standalone
 */
struct xrt_device *
standalone_create(struct xrt_tracked_slam *slam);

/*!
 * @dir drivers/standalone
 *
 * @brief @ref drv_standalone files.
 */


#ifdef __cplusplus
}
#endif
