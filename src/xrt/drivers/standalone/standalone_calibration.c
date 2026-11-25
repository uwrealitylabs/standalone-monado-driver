// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Calibration of the Standalone headset.
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "standalone_calibration.h"


void
standalone_calibration_fill_nominal(struct standalone_calibration *calib)
{
	U_ZERO(calib);

	for (uint32_t i = 0; i < STANDALONE_CAM_COUNT; i++) {
		const struct standalone_camera_info *info = standalone_hw_camera_info((enum standalone_camera)i);
		struct standalone_camera_calibration *cc = &calib->cams[i];

		// A 90 degree horizontal field of view: fx = (w / 2) / tan(45 deg).
		const double f = (double)info->width / 2.0;

		cc->intrinsics.image_size_pixels.w = (int)info->width;
		cc->intrinsics.image_size_pixels.h = (int)info->height;
		cc->intrinsics.intrinsics[0][0] = f;
		cc->intrinsics.intrinsics[1][1] = f;
		cc->intrinsics.intrinsics[0][2] = (double)info->width / 2.0;
		cc->intrinsics.intrinsics[1][2] = (double)info->height / 2.0;
		cc->intrinsics.intrinsics[2][2] = 1.0;
		cc->intrinsics.distortion_model = T_DISTORTION_OPENCV_RADTAN_5;

		cc->head_from_camera = (struct xrt_pose)XRT_POSE_IDENTITY;
	}

	// The one extrinsic the board documents fix: the front pair is 67 mm apart.
	calib->cams[STANDALONE_CAM_FRONT_LEFT].head_from_camera.position.x = -STANDALONE_FRONT_BASELINE_M / 2.0f;
	calib->cams[STANDALONE_CAM_FRONT_RIGHT].head_from_camera.position.x = STANDALONE_FRONT_BASELINE_M / 2.0f;

	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		const struct standalone_imu_info *info = standalone_hw_imu_info((enum standalone_imu)i);
		struct standalone_imu_calibration *ic = &calib->imus[i];

		for (uint32_t d = 0; d < 3; d++) {
			ic->intrinsics.accel.transform[d][d] = 1.0;
			ic->intrinsics.gyro.transform[d][d] = 1.0;
		}

		ic->head_from_imu = (struct xrt_pose)XRT_POSE_IDENTITY;
		ic->rate_hz = (double)info->nominal_rate_hz;
	}

	calib->is_nominal = true;
}

bool
standalone_calibration_load(const char *path, struct standalone_calibration *out_calib)
{
	standalone_calibration_fill_nominal(out_calib);

	if (path == NULL || path[0] == '\0') {
		return true;
	}

	//! @todo Decide the calibration file format and parse it here.
	U_LOG_W("Standalone calibration file '%s' ignored: loading is not implemented, using nominal values", path);

	return false;
}
