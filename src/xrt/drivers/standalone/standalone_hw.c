// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hardware description tables of the Standalone headset.
 * @ingroup drv_standalone
 */

#include "standalone_hw.h"

#include <assert.h>

// The eight tracking cameras fill the headset-wide xrt_slam_sinks exactly.
_Static_assert(STANDALONE_CAM_TRACKING_COUNT <= XRT_TRACKING_MAX_CAMS, "Tracking cameras must fit xrt_slam_sinks");
_Static_assert(STANDALONE_IMU_COUNT <= XRT_TRACKING_MAX_IMUS, "IMUs must fit xrt_slam_sinks");

#define AR0234_W 1920
#define AR0234_H 1200
#define OV9281_W 1280
#define OV9281_H 800

/*
 * Frame rates: the AR0234 ports are specified for 1920x1200 RAW10 at 120 fps
 * on four lanes, with no blanket 120 fps requirement; 60 is a guess at a
 * sensible tracking default. The four OV9281 are required to run at 120 fps.
 */
static const struct standalone_camera_info camera_infos[STANDALONE_CAM_COUNT] = {
    [STANDALONE_CAM_FRONT_LEFT] =
        {
            .name = "front_left",
            .sensor = "AR0234CS mono",
            .connector = "J501",
            .stream = STANDALONE_STREAM_FRONT_LEFT,
            .stream_x_offset = 0,
            .width = AR0234_W,
            .height = AR0234_H,
            .max_fps = 120,
            .nominal_fps = 60,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_FRONT_RIGHT,
        },
    [STANDALONE_CAM_FRONT_RIGHT] =
        {
            .name = "front_right",
            .sensor = "AR0234CS mono",
            .connector = "J502",
            .stream = STANDALONE_STREAM_FRONT_RIGHT,
            .stream_x_offset = 0,
            .width = AR0234_W,
            .height = AR0234_H,
            .max_fps = 120,
            .nominal_fps = 60,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_FRONT_LEFT,
        },
    [STANDALONE_CAM_SIDE_LEFT] =
        {
            .name = "side_left",
            .sensor = "AR0234CS mono",
            .connector = "J505",
            .stream = STANDALONE_STREAM_SIDE_LEFT,
            .stream_x_offset = 0,
            .width = AR0234_W,
            .height = AR0234_H,
            .max_fps = 120,
            .nominal_fps = 60,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_SIDE_RIGHT,
        },
    [STANDALONE_CAM_SIDE_RIGHT] =
        {
            .name = "side_right",
            .sensor = "AR0234CS mono",
            .connector = "J503",
            .stream = STANDALONE_STREAM_SIDE_RIGHT,
            .stream_x_offset = 0,
            .width = AR0234_W,
            .height = AR0234_H,
            .max_fps = 120,
            .nominal_fps = 60,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_SIDE_LEFT,
        },
    [STANDALONE_CAM_BOTTOM_LEFT] =
        {
            .name = "bottom_left",
            .sensor = "OV9281",
            .connector = "J1603",
            .stream = STANDALONE_STREAM_BOTTOM_PAIR,
            .stream_x_offset = 0,
            .width = OV9281_W,
            .height = OV9281_H,
            .max_fps = 120,
            .nominal_fps = 120,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_BOTTOM_RIGHT,
        },
    [STANDALONE_CAM_BOTTOM_RIGHT] =
        {
            .name = "bottom_right",
            .sensor = "OV9281",
            .connector = "J1604",
            .stream = STANDALONE_STREAM_BOTTOM_PAIR,
            .stream_x_offset = OV9281_W,
            .width = OV9281_W,
            .height = OV9281_H,
            .max_fps = 120,
            .nominal_fps = 120,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_BOTTOM_LEFT,
        },
    [STANDALONE_CAM_EYE_LEFT] =
        {
            .name = "eye_left",
            .sensor = "OV9281",
            .connector = "J1601",
            .stream = STANDALONE_STREAM_EYE_PAIR,
            .stream_x_offset = 0,
            .width = OV9281_W,
            .height = OV9281_H,
            .max_fps = 120,
            .nominal_fps = 120,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_EYE_RIGHT,
        },
    [STANDALONE_CAM_EYE_RIGHT] =
        {
            .name = "eye_right",
            .sensor = "OV9281",
            .connector = "J1602",
            .stream = STANDALONE_STREAM_EYE_PAIR,
            .stream_x_offset = OV9281_W,
            .width = OV9281_W,
            .height = OV9281_H,
            .max_fps = 120,
            .nominal_fps = 120,
            .mono = true,
            .global_shutter = true,
            .format = XRT_FORMAT_L8,
            .pair = STANDALONE_CAM_EYE_LEFT,
        },
    [STANDALONE_CAM_COLOUR] =
        {
            .name = "colour",
            .sensor = "AR0234 colour (placeholder, sensor pick open)",
            .connector = "J506",
            .stream = STANDALONE_STREAM_COLOUR,
            .stream_x_offset = 0,
            .width = AR0234_W,
            .height = AR0234_H,
            .max_fps = 120,
            .nominal_fps = 30,
            .mono = false,
            .global_shutter = true,
            .format = XRT_FORMAT_R8G8B8,
            .pair = STANDALONE_CAM_COLOUR,
        },
};

static const struct standalone_imu_info imu_infos[STANDALONE_IMU_COUNT] = {
    [STANDALONE_IMU_LEFT] =
        {
            .name = "imu_left",
            .part = "LSM6DSV16X",
            .reference = "U501",
            .nominal_rate_hz = STANDALONE_IMU_NOMINAL_RATE_HZ,
        },
    [STANDALONE_IMU_RIGHT] =
        {
            .name = "imu_right",
            .part = "LSM6DSV16X",
            .reference = "U504",
            .nominal_rate_hz = STANDALONE_IMU_NOMINAL_RATE_HZ,
        },
};

const struct standalone_camera_info *
standalone_hw_camera_info(enum standalone_camera cam)
{
	assert((uint32_t)cam < STANDALONE_CAM_COUNT);
	return &camera_infos[cam];
}

const struct standalone_imu_info *
standalone_hw_imu_info(enum standalone_imu imu)
{
	assert((uint32_t)imu < STANDALONE_IMU_COUNT);
	return &imu_infos[imu];
}
