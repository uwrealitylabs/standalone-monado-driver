// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Calibration of the Standalone headset, as handed to trackers and
 *         devices.
 *
 * ## Frames
 *
 * - **Head frame.** The frame every tracker reports in and the frame the
 *   core calls the head pose. OpenXR view-space axes: +X right, +Y up, -Z
 *   forward, origin midway between the eyes.
 * - **Camera frame.** OpenCV axes: +X right, +Y down, +Z forward, origin at
 *   the optical centre.
 * - **IMU frame.** The sensor's own axes.
 *
 * Poses are named `head_from_x`: the pose of sensor `x` expressed in the
 * head frame, which is also the transform taking a point in `x` coordinates
 * to head coordinates.
 *
 * @ingroup drv_standalone
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "tracking/t_tracking.h"

#include "standalone_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Calibration of one camera.
 *
 * @ingroup drv_standalone
 */
struct standalone_camera_calibration
{
	//! Image size the calibration is for, pinhole intrinsics and distortion.
	struct t_camera_calibration intrinsics;

	//! Pose of the camera in the head frame.
	struct xrt_pose head_from_camera;
};

/*!
 * Calibration of one IMU.
 *
 * @ingroup drv_standalone
 */
struct standalone_imu_calibration
{
	//! Accelerometer and gyroscope scale, bias and noise.
	struct t_imu_calibration intrinsics;

	//! Pose of the IMU in the head frame.
	struct xrt_pose head_from_imu;

	//! Output data rate of the samples.
	double rate_hz;
};

/*!
 * Calibration of the whole headset, indexed like the hardware.
 *
 * @ingroup drv_standalone
 */
struct standalone_calibration
{
	struct standalone_camera_calibration cams[STANDALONE_CAM_COUNT];
	struct standalone_imu_calibration imus[STANDALONE_IMU_COUNT];

	/*!
	 * True while the values are the placeholders of
	 * @ref standalone_calibration_fill_nominal and not a measurement.
	 * A tracker whose accuracy depends on calibration should say so in
	 * its log when this is set.
	 */
	bool is_nominal;
};

/*!
 * Fill in placeholder values: native image sizes, a principal point at the
 * image centre, a focal length giving a 90 degree horizontal field of view,
 * no distortion, identity IMU intrinsics, and every sensor at the head
 * origin with identity orientation (the front pair is offset by half the
 * baseline each). Sets `is_nominal`.
 *
 * @ingroup drv_standalone
 */
void
standalone_calibration_fill_nominal(struct standalone_calibration *calib);

/*!
 * Load the headset calibration from a file.
 *
 * Not implemented: the file format is not decided. Always fills in the
 * nominal calibration; returns false if a path was given, since it could not
 * be honoured.
 *
 * @param path Path of the calibration file, NULL or empty for none.
 *
 * @ingroup drv_standalone
 */
bool
standalone_calibration_load(const char *path, struct standalone_calibration *out_calib);

#ifdef __cplusplus
}
#endif
