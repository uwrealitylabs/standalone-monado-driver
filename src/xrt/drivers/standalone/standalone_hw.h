// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hardware description of the Standalone headset: sensors, their
 *         indices and their nominal modes.
 *
 * This file is the single place where the driver is coupled to a hardware
 * revision. Everything here is derived from the motherboard repository
 * (`rb5-standalone-motherboard`), board v4, design state of 2026-11-08:
 *
 * - `architecture.yaml` (blocks, links and features),
 * - `cameras/README.md` and `docs/design-state/cameras.md` (roles, sensors),
 * - `boards/mainboard/facts/camera-aggregator-contract.json` (pair composites),
 * - `electronics/sensor-sync-architecture-2026-10-01.md` (time base, IMUs).
 *
 * Nothing here has been measured on hardware. Values marked "nominal" are the
 * design targets of those documents, not verified modes.
 *
 * The index space defined here is used everywhere in the driver: by the
 * capture source, by the three trackers, and by the calibration. Camera
 * index `n` is also slot `cams[n]` of the headset-wide @ref xrt_slam_sinks
 * that the capture source is started with.
 *
 * @ingroup drv_standalone
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif

//! Hardware revision this description was derived from.
#define STANDALONE_HW_REVISION "rb5-standalone-motherboard board v4 (design state 2026-11-08)"

/*!
 * The nine cameras of the headset. Left and right are the wearer's.
 *
 * The first eight are the mono global-shutter tracking cameras; their index
 * is their slot in the headset-wide @ref xrt_slam_sinks. Stereo pairs occupy
 * adjacent slots, left first. The front pair is deliberately at 0 and 1 so
 * that any stereo + IMU dataset (EuRoC format) drops into the SLAM lane.
 *
 * The colour camera is not a tracking camera and has no slot in
 * @ref xrt_slam_sinks; it is delivered through @ref xrt_fs::stream_start.
 *
 * @ingroup drv_standalone
 */
enum standalone_camera
{
	STANDALONE_CAM_FRONT_LEFT = 0,   //!< J501, AR0234 mono, direct CSI.
	STANDALONE_CAM_FRONT_RIGHT = 1,  //!< J502, AR0234 mono, direct CSI.
	STANDALONE_CAM_SIDE_LEFT = 2,    //!< J505, AR0234 mono, direct CSI.
	STANDALONE_CAM_SIDE_RIGHT = 3,   //!< J503, AR0234 mono, direct CSI.
	STANDALONE_CAM_BOTTOM_LEFT = 4,  //!< J1603, OV9281, FPGA channel 2.
	STANDALONE_CAM_BOTTOM_RIGHT = 5, //!< J1604, OV9281, FPGA channel 3.
	STANDALONE_CAM_EYE_LEFT = 6,     //!< J1601, OV9281, FPGA channel 0. Looks at the left eye.
	STANDALONE_CAM_EYE_RIGHT = 7,    //!< J1602, OV9281, FPGA channel 1. Looks at the right eye.

	//! Number of tracking cameras, equal to XRT_TRACKING_MAX_CAMS.
	STANDALONE_CAM_TRACKING_COUNT = 8,

	STANDALONE_CAM_COLOUR = 8, //!< J506, colour camera (AR0234 colour placeholder, sensor pick open).

	STANDALONE_CAM_COUNT = 9,
};

/*!
 * The two IMUs (LSM6DSV16X on a shared SPI bus). The index is the slot in
 * `imus[]` of the headset-wide @ref xrt_slam_sinks. IMU 0 is the reference
 * IMU: it is the one a single-IMU consumer (and a EuRoC dataset) uses.
 *
 * @ingroup drv_standalone
 */
enum standalone_imu
{
	STANDALONE_IMU_LEFT = 0,  //!< U501, near the left end of the board.
	STANDALONE_IMU_RIGHT = 1, //!< U504, near the right end of the board.
	STANDALONE_IMU_COUNT = 2,
};

/*!
 * The seven image streams the host (QRB5165) receives from the nine sensors.
 *
 * Five cameras have their own CSI port. The four OV9281 go through the FPGA
 * (Efinix Ti60), which sends them as two side-by-side pair composites on two
 * virtual channels of one CSI port. A capture implementation reads these
 * seven streams and splits the two composites, so that downstream of capture
 * there is one frame per camera.
 *
 * Which CSI port each stream arrives on is a device-tree matter and is not
 * encoded here.
 *
 * @ingroup drv_standalone
 */
enum standalone_host_stream
{
	STANDALONE_STREAM_FRONT_LEFT = 0,
	STANDALONE_STREAM_FRONT_RIGHT,
	STANDALONE_STREAM_SIDE_LEFT,
	STANDALONE_STREAM_SIDE_RIGHT,
	STANDALONE_STREAM_COLOUR,
	STANDALONE_STREAM_EYE_PAIR,    //!< 2560x800: eye left | eye right, virtual channel 0.
	STANDALONE_STREAM_BOTTOM_PAIR, //!< 2560x800: bottom left | bottom right, virtual channel 1.
	STANDALONE_STREAM_COUNT,
};

//! Bit for a camera in a camera mask. @ingroup drv_standalone
#define STANDALONE_CAM_BIT(CAM) (1u << (CAM))
//! Bit for an IMU in an IMU mask. @ingroup drv_standalone
#define STANDALONE_IMU_BIT(IMU) (1u << (IMU))

#define STANDALONE_CAMS_FRONT                                                                                          \
	(STANDALONE_CAM_BIT(STANDALONE_CAM_FRONT_LEFT) | STANDALONE_CAM_BIT(STANDALONE_CAM_FRONT_RIGHT))
#define STANDALONE_CAMS_SIDE                                                                                           \
	(STANDALONE_CAM_BIT(STANDALONE_CAM_SIDE_LEFT) | STANDALONE_CAM_BIT(STANDALONE_CAM_SIDE_RIGHT))
#define STANDALONE_CAMS_BOTTOM                                                                                         \
	(STANDALONE_CAM_BIT(STANDALONE_CAM_BOTTOM_LEFT) | STANDALONE_CAM_BIT(STANDALONE_CAM_BOTTOM_RIGHT))
#define STANDALONE_CAMS_EYE (STANDALONE_CAM_BIT(STANDALONE_CAM_EYE_LEFT) | STANDALONE_CAM_BIT(STANDALONE_CAM_EYE_RIGHT))

//! All eight tracking cameras.
#define STANDALONE_CAMS_TRACKING_ALL ((1u << STANDALONE_CAM_TRACKING_COUNT) - 1u)
//! Both IMUs.
#define STANDALONE_IMUS_ALL ((1u << STANDALONE_IMU_COUNT) - 1u)

/*!
 * @name Cameras each tracker consumes by default
 *
 * "Stereo tracking" uses the front pair and the IMUs. Every camera except the
 * colour and the eye cameras takes part in hand tracking (owner decision
 * CAM-06). The eye pair, lit by strobed IR, is for gaze only.
 *
 * An implementation may consume a different set: it decides by which slots of
 * its sinks it fills in. These are the defaults the empty implementations use.
 * @{
 */
#define STANDALONE_CAMS_SLAM_DEFAULT STANDALONE_CAMS_FRONT
#define STANDALONE_CAMS_HAND_DEFAULT (STANDALONE_CAMS_FRONT | STANDALONE_CAMS_SIDE | STANDALONE_CAMS_BOTTOM)
#define STANDALONE_CAMS_EYE_DEFAULT STANDALONE_CAMS_EYE
/*! @} */

/*!
 * @name Other hardware constants
 * @{
 */
//! Frequency of the FPGA counter every FSIN edge, the IMU reference and the host frame marker derive from.
#define STANDALONE_FPGA_TIMEBASE_HZ (24000000)
//! IMU output data rate in ODR-triggered mode (proposed in the sync architecture, not configured anywhere yet).
#define STANDALONE_IMU_NOMINAL_RATE_HZ (1600)
//! Distance between the front camera centres.
#define STANDALONE_FRONT_BASELINE_M (0.067f)
//! One BOE VS035ZSM panel per eye.
#define STANDALONE_PANEL_W_PIXELS (1440)
#define STANDALONE_PANEL_H_PIXELS (1600)
//! Nominal panel refresh rate (the panel supports 80 to 120 Hz).
#define STANDALONE_PANEL_REFRESH_HZ (90)
/*! @} */

/*!
 * Static facts about one camera.
 *
 * @ingroup drv_standalone
 */
struct standalone_camera_info
{
	const char *name;      //!< Short name, e.g. "front_left". Stable; usable in config and dataset folder names.
	const char *sensor;    //!< Sensor part.
	const char *connector; //!< Board connector reference.

	enum standalone_host_stream stream; //!< Host stream that carries this camera.
	uint32_t stream_x_offset;           //!< X offset of this camera's image inside a host stream frame.

	uint32_t width;       //!< Native width in pixels.
	uint32_t height;      //!< Native height in pixels.
	uint32_t max_fps;     //!< Highest frame rate the design targets.
	uint32_t nominal_fps; //!< Default frame rate, used when a capture config does not set one.

	bool mono;                   //!< Monochrome sensor.
	bool global_shutter;         //!< Global shutter sensor.
	enum xrt_format format;      //!< Pixel format of the frames pushed into sinks.
	enum standalone_camera pair; //!< The other camera of the stereo pair, or itself if it has none.
};

/*!
 * Static facts about one IMU.
 *
 * @ingroup drv_standalone
 */
struct standalone_imu_info
{
	const char *name;         //!< Short name, e.g. "imu_left".
	const char *part;         //!< Sensor part.
	const char *reference;    //!< Board reference designator.
	uint32_t nominal_rate_hz; //!< Default output data rate.
};

/*!
 * Get the static description of a camera.
 *
 * @ingroup drv_standalone
 */
const struct standalone_camera_info *
standalone_hw_camera_info(enum standalone_camera cam);

/*!
 * Get the static description of an IMU.
 *
 * @ingroup drv_standalone
 */
const struct standalone_imu_info *
standalone_hw_imu_info(enum standalone_imu imu);

/*!
 * Is this camera the left camera of its stereo pair (or unpaired)?
 *
 * @ingroup drv_standalone
 */
static inline bool
standalone_hw_camera_is_left(enum standalone_camera cam)
{
	return cam == STANDALONE_CAM_COLOUR || ((uint32_t)cam % 2u) == 0u;
}

#ifdef __cplusplus
}
#endif
