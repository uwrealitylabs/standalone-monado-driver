// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SLAM tracker of the Standalone headset: where the head is.
 *
 * ## The interface
 *
 * | Side   | What                                   | Called by |
 * |--------|----------------------------------------|-----------|
 * | Input  | `sinks`: frames, IMU samples, ground truth, device masks | The capture source and the hand tracker, on
 * their threads | | Output | `get_head_pose`: the head pose at a time | The HMD device, on the core's threads |
 *
 * Data arrives when the sensors produce it; queries arrive at arbitrary
 * times, usually for a time slightly in the future. An implementation keeps
 * a timestamped history of its estimates and answers queries from it by
 * interpolation or prediction. A query must return quickly and must never
 * wait for sensor data.
 *
 * ## Adding an implementation
 *
 * Write `slam/standalone_slam_<name>.c` with a create function of type
 * @ref standalone_slam_create_fn, declare it below, add it to the table in
 * standalone_slam.c and to the driver's sources in CMake. Select it with
 * `STANDALONE_SLAM_IMPL=<name>`.
 *
 * @ingroup drv_standalone
 */

#pragma once

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "standalone_calibration.h"
#include "standalone_hw.h"
#include "standalone_sinks.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Configuration of a SLAM tracker.
 *
 * @ingroup drv_standalone
 */
struct standalone_slam_config
{
	//! Name of the implementation, NULL for "null".
	const char *impl;

	//! Headset calibration. Outlives the tracker.
	const struct standalone_calibration *calib;

	//! Cameras the source can deliver, see @ref STANDALONE_CAM_BIT. Do not expect frames from others.
	uint32_t available_cams;

	//! IMUs the source can deliver, see @ref STANDALONE_IMU_BIT.
	uint32_t available_imus;
};

/*!
 * A SLAM tracker.
 *
 * @ingroup drv_standalone
 */
struct standalone_slam_tracker
{
	/*!
	 * Inputs, in the headset sink map (see standalone_sinks.h). The
	 * implementation fills in the slots of the sensors it consumes and
	 * leaves the rest NULL; by default the front pair and the IMUs.
	 * `gt` may be set to receive ground truth during replay. `masks[0]`
	 * may be set to receive hand bounding boxes from the hand tracker.
	 */
	struct xrt_slam_sinks sinks;

	/*!
	 * What the head pose is tracked with: @ref XRT_TRACKING_TYPE_NONE
	 * if the pose is not a measurement of the headset's motion.
	 */
	enum xrt_tracking_type tracking_type;

	/*!
	 * Get the pose of the head frame in the tracking space at a time.
	 *
	 * The tracking space is gravity aligned with +Y up; its origin and
	 * heading are where tracking started. The head frame is defined in
	 * standalone_calibration.h. Velocities are filled in if known. The
	 * relation flags say what is valid and what is tracked; all flags
	 * clear means there is no pose yet.
	 *
	 * @param      sst             The tracker.
	 * @param      at_timestamp_ns Monotonic time the pose is wanted for, may be in the future.
	 * @param[out] out_relation    The head pose.
	 */
	void (*get_head_pose)(struct standalone_slam_tracker *sst,
	                      int64_t at_timestamp_ns,
	                      struct xrt_space_relation *out_relation);
};

/*!
 * @copydoc standalone_slam_tracker::get_head_pose
 *
 * @public @memberof standalone_slam_tracker
 */
static inline void
standalone_slam_tracker_get_head_pose(struct standalone_slam_tracker *sst,
                                      int64_t at_timestamp_ns,
                                      struct xrt_space_relation *out_relation)
{
	sst->get_head_pose(sst, at_timestamp_ns, out_relation);
}

/*!
 * Create function of a SLAM tracker implementation.
 *
 * The tracker must add a @ref xrt_frame_node to @p xfctx that stops its
 * threads in `break_apart` and frees it in `destroy`. It must be ready to
 * receive data when this returns.
 *
 * @param      xfctx  Frame context that will own the tracker.
 * @param      config Configuration, only valid during the call (except `calib`).
 * @param[out] out_sst The tracker.
 * @return XRT_SUCCESS, or an error if the tracker could not be created.
 *
 * @ingroup drv_standalone
 */
typedef xrt_result_t (*standalone_slam_create_fn)(struct xrt_frame_context *xfctx,
                                                  const struct standalone_slam_config *config,
                                                  struct standalone_slam_tracker **out_sst);

/*!
 * Create the SLAM tracker named by `config->impl`.
 *
 * @copydetails standalone_slam_create_fn
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_slam_create(struct xrt_frame_context *xfctx,
                       const struct standalone_slam_config *config,
                       struct standalone_slam_tracker **out_sst);

/*!
 * Is there a SLAM tracker implementation with this name in this build?
 *
 * @ingroup drv_standalone
 */
bool
standalone_slam_has_impl(const char *name);


/*
 *
 * Implementations.
 *
 */

/*!
 * "null": the empty implementation. Accepts and drops the front pair and
 * the IMUs, and reports a fixed identity pose.
 *
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_slam_null_create(struct xrt_frame_context *xfctx,
                            const struct standalone_slam_config *config,
                            struct standalone_slam_tracker **out_sst);

#ifdef XRT_FEATURE_SLAM
/*!
 * "t_slam": Monado's SLAM tracker, which loads an external SLAM system
 * (Basalt by default) through the VIT interface.
 *
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_slam_t_slam_create(struct xrt_frame_context *xfctx,
                              const struct standalone_slam_config *config,
                              struct standalone_slam_tracker **out_sst);
#endif

#ifdef __cplusplus
}
#endif
