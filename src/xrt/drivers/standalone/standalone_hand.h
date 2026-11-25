// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hand tracker of the Standalone headset: where the hands are.
 *
 * ## The interface
 *
 * | Side   | What                                         | Called by |
 * |--------|----------------------------------------------|-----------|
 * | Input  | `sinks`: frames of the hand-tracking cameras | The capture source, on its threads |
 * | Output | `get_hand`: the joints of one hand at a time | The hand device, on the core's threads |
 * | Output | Hand bounding boxes pushed into `config->masks_sink` | The tracker itself, optional |
 *
 * Data arrives when the cameras produce it; queries arrive at arbitrary
 * times. An implementation runs on its own thread, keeps its latest results
 * and answers queries from them. A query must return quickly and must never
 * wait for a frame.
 *
 * Six cameras see the hands: the front, side and bottom pairs. Hands inside
 * the display's field of view are covered in stereo; outside it, one camera
 * may be all there is (owner decisions CAM-18, CAM-19).
 *
 * ## Adding an implementation
 *
 * Write `hand/standalone_hand_<name>.c` with a create function of type
 * @ref standalone_hand_create_fn, declare it below, add it to the table in
 * standalone_hand.c and to the driver's sources in CMake. Select it with
 * `STANDALONE_HAND_IMPL=<name>`.
 *
 * @ingroup drv_standalone
 */

#pragma once

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
 * Configuration of a hand tracker.
 *
 * @ingroup drv_standalone
 */
struct standalone_hand_config
{
	//! Name of the implementation, NULL for "null".
	const char *impl;

	//! Headset calibration. Outlives the tracker.
	const struct standalone_calibration *calib;

	//! Cameras the source can deliver, see @ref STANDALONE_CAM_BIT. Do not expect frames from others.
	uint32_t available_cams;

	/*!
	 * Where to push the bounding boxes of the hands in the SLAM cameras,
	 * so that SLAM ignores features on moving hands. NULL if the SLAM
	 * tracker does not take masks. Outlives the tracker's threads.
	 */
	struct xrt_device_masks_sink *masks_sink;
};

/*!
 * A hand tracker.
 *
 * @ingroup drv_standalone
 */
struct standalone_hand_tracker
{
	/*!
	 * Inputs, in the headset sink map (see standalone_sinks.h). The
	 * implementation fills in the slots of the cameras it consumes and
	 * leaves the rest NULL; by default the front, side and bottom pairs.
	 */
	struct xrt_slam_sinks sinks;

	/*!
	 * Get the joints of one hand.
	 *
	 * The joint poses in `out_value->values` are relative to
	 * `out_value->hand_pose`, and `hand_pose` is expressed in the head
	 * frame (see standalone_calibration.h) at `*out_timestamp_ns`. The
	 * device composes it with the head pose at that time.
	 * `out_value->is_active` is false when the hand is not seen.
	 *
	 * @param      sht                  The tracker.
	 * @param      hand                 Which hand.
	 * @param      desired_timestamp_ns Monotonic time the joints are wanted for.
	 * @param[out] out_value            The joints.
	 * @param[out] out_timestamp_ns     Time the joints are actually for, at most the desired time.
	 */
	void (*get_hand)(struct standalone_hand_tracker *sht,
	                 enum xrt_hand hand,
	                 int64_t desired_timestamp_ns,
	                 struct xrt_hand_joint_set *out_value,
	                 int64_t *out_timestamp_ns);
};

/*!
 * @copydoc standalone_hand_tracker::get_hand
 *
 * @public @memberof standalone_hand_tracker
 */
static inline void
standalone_hand_tracker_get_hand(struct standalone_hand_tracker *sht,
                                 enum xrt_hand hand,
                                 int64_t desired_timestamp_ns,
                                 struct xrt_hand_joint_set *out_value,
                                 int64_t *out_timestamp_ns)
{
	sht->get_hand(sht, hand, desired_timestamp_ns, out_value, out_timestamp_ns);
}

/*!
 * Create function of a hand tracker implementation.
 *
 * The tracker must add a @ref xrt_frame_node to @p xfctx that stops its
 * threads in `break_apart` and frees it in `destroy`. It must be ready to
 * receive data when this returns.
 *
 * @param      xfctx   Frame context that will own the tracker.
 * @param      config  Configuration, only valid during the call (except `calib` and `masks_sink`).
 * @param[out] out_sht The tracker.
 * @return XRT_SUCCESS, or an error if the tracker could not be created.
 *
 * @ingroup drv_standalone
 */
typedef xrt_result_t (*standalone_hand_create_fn)(struct xrt_frame_context *xfctx,
                                                  const struct standalone_hand_config *config,
                                                  struct standalone_hand_tracker **out_sht);

/*!
 * Create the hand tracker named by `config->impl`.
 *
 * @copydetails standalone_hand_create_fn
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_hand_create(struct xrt_frame_context *xfctx,
                       const struct standalone_hand_config *config,
                       struct standalone_hand_tracker **out_sht);

/*!
 * Is there a hand tracker implementation with this name in this build?
 *
 * @ingroup drv_standalone
 */
bool
standalone_hand_has_impl(const char *name);


/*
 *
 * Implementations.
 *
 */

/*!
 * "null": the empty implementation. Accepts and drops the frames of the
 * six hand-tracking cameras, and never sees a hand.
 *
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_hand_null_create(struct xrt_frame_context *xfctx,
                            const struct standalone_hand_config *config,
                            struct standalone_hand_tracker **out_sht);

#ifdef __cplusplus
}
#endif
