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
 * Not detectable by USB VID/PID, the devices are created by the standalone
 * builder (see t_builder_standalone_create) instead of a prober.
 *
 * ## Layout
 *
 * | File                         | What it fixes |
 * |------------------------------|---------------|
 * | standalone_hw.h              | The hardware: nine cameras, two IMUs, their indices and modes |
 * | standalone_sinks.h           | The headset sink map and the data contract between capture and trackers |
 * | standalone_calibration.h     | Frames and the calibration handed to trackers and devices |
 * | standalone_capture.h         | Capture: an @ref xrt_fs per implementation, in `capture/` |
 * | standalone_slam.h            | SLAM tracker interface, implementations in `slam/` |
 * | standalone_hand.h            | Hand tracker interface, implementations in `hand/` |
 * | standalone_eye.h             | Eye tracker interface, implementations in `eye/` |
 * | standalone_pipeline.h        | Creates one source and three trackers and connects them |
 * | standalone_interface.h       | The three devices, created by the builder |
 *
 * Implementations of capture and of each tracker are interchangeable behind
 * their interface and are selected by name at startup. Each has an empty
 * implementation called "null". The devices, the pipeline and the builder
 * do not change when an implementation does.
 *
 * See @ref writing-driver for additional information.
 */

struct xrt_device;
struct standalone_slam_tracker;
struct standalone_hand_tracker;
struct standalone_eye_tracker;

/*!
 * Create the Standalone HMD: head pose, views and display description.
 *
 * @param slam Where the head pose comes from. Owned by the system's frame context.
 *
 * This is only exposed so that the builder (in the target)
 * can call the construction function (in the driver)
 * @ingroup drv_standalone
 */
struct xrt_device *
standalone_hmd_create(struct standalone_slam_tracker *slam);

/*!
 * Create the Standalone hand tracking device: the joints of both hands.
 *
 * @param hand Where the joints come from. Owned by the system's frame context.
 * @param head The HMD, for the head pose the joints are relative to. Must outlive this device's use.
 *
 * @ingroup drv_standalone
 */
struct xrt_device *
standalone_hand_device_create(struct standalone_hand_tracker *hand, struct xrt_device *head);

/*!
 * Create the Standalone eye tracking device: the gaze.
 *
 * @param eye  Where the gaze comes from. Owned by the system's frame context.
 * @param head The HMD, for the head pose the gaze is relative to. Must outlive this device's use.
 *
 * @ingroup drv_standalone
 */
struct xrt_device *
standalone_eye_device_create(struct standalone_eye_tracker *eye, struct xrt_device *head);

/*!
 * @dir drivers/standalone
 *
 * @brief @ref drv_standalone files.
 */


#ifdef __cplusplus
}
#endif
