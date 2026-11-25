// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Eye tracker of the Standalone headset: where the wearer looks.
 *
 * Monado has no eye tracker interface; this one is the driver's own.
 *
 * ## The interface
 *
 * | Side   | What                                   | Called by |
 * |--------|----------------------------------------|-----------|
 * | Input  | `sinks`: frames of the two eye cameras | The capture source, on its threads |
 * | Output | `get_gaze`: the gaze at a time         | The eye device, on the core's threads |
 *
 * Data arrives when the cameras produce it (120 fps); queries arrive at
 * arbitrary times. An implementation runs on its own thread, keeps its
 * latest results and answers queries from them. A query must return quickly
 * and must never wait for a frame.
 *
 * The eye cameras see the eyes under infrared light that the FPGA strobes
 * inside each exposure. The strobe is hardware timed and is not controlled
 * from here.
 *
 * ## Adding an implementation
 *
 * Write `eye/standalone_eye_<name>.c` with a create function of type
 * @ref standalone_eye_create_fn, declare it below, add it to the table in
 * standalone_eye.c and to the driver's sources in CMake. Select it with
 * `STANDALONE_EYE_IMPL=<name>`.
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
 * Configuration of an eye tracker.
 *
 * @ingroup drv_standalone
 */
struct standalone_eye_config
{
	//! Name of the implementation, NULL for "null".
	const char *impl;

	//! Headset calibration. Outlives the tracker.
	const struct standalone_calibration *calib;

	//! Cameras the source can deliver, see @ref STANDALONE_CAM_BIT. Do not expect frames from others.
	uint32_t available_cams;
};

/*!
 * A gaze estimate.
 *
 * Each gaze is a pose in the head frame (see standalone_calibration.h): the
 * position is where the gaze ray starts and the orientation points -Z along
 * it, the convention of OpenXR's eye gaze interaction. The relation flags of
 * each say whether it is valid and tracked; all flags clear means the eye is
 * not seen (closed, or the headset is off).
 *
 * @ingroup drv_standalone
 */
struct standalone_gaze
{
	//! Monotonic time the estimate is for.
	int64_t timestamp_ns;

	//! One ray for both eyes, from midway between them towards the point looked at.
	struct xrt_space_relation combined;

	//! One ray per eye, from the eye's centre. Index 0 is the left eye.
	struct xrt_space_relation per_eye[2];
};

/*!
 * An eye tracker.
 *
 * @ingroup drv_standalone
 */
struct standalone_eye_tracker
{
	/*!
	 * Inputs, in the headset sink map (see standalone_sinks.h). The
	 * implementation fills in the slots of the cameras it consumes and
	 * leaves the rest NULL; by default the eye pair.
	 */
	struct xrt_slam_sinks sinks;

	/*!
	 * Get the gaze at a time.
	 *
	 * @param      set             The tracker.
	 * @param      at_timestamp_ns Monotonic time the gaze is wanted for.
	 * @param[out] out_gaze        The gaze, see @ref standalone_gaze.
	 */
	void (*get_gaze)(struct standalone_eye_tracker *set, int64_t at_timestamp_ns, struct standalone_gaze *out_gaze);
};

/*!
 * @copydoc standalone_eye_tracker::get_gaze
 *
 * @public @memberof standalone_eye_tracker
 */
static inline void
standalone_eye_tracker_get_gaze(struct standalone_eye_tracker *set,
                                int64_t at_timestamp_ns,
                                struct standalone_gaze *out_gaze)
{
	set->get_gaze(set, at_timestamp_ns, out_gaze);
}

/*!
 * Create function of an eye tracker implementation.
 *
 * The tracker must add a @ref xrt_frame_node to @p xfctx that stops its
 * threads in `break_apart` and frees it in `destroy`. It must be ready to
 * receive data when this returns.
 *
 * @param      xfctx   Frame context that will own the tracker.
 * @param      config  Configuration, only valid during the call (except `calib`).
 * @param[out] out_set The tracker.
 * @return XRT_SUCCESS, or an error if the tracker could not be created.
 *
 * @ingroup drv_standalone
 */
typedef xrt_result_t (*standalone_eye_create_fn)(struct xrt_frame_context *xfctx,
                                                 const struct standalone_eye_config *config,
                                                 struct standalone_eye_tracker **out_set);

/*!
 * Create the eye tracker named by `config->impl`.
 *
 * @copydetails standalone_eye_create_fn
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_eye_create(struct xrt_frame_context *xfctx,
                      const struct standalone_eye_config *config,
                      struct standalone_eye_tracker **out_set);

/*!
 * Is there an eye tracker implementation with this name in this build?
 *
 * @ingroup drv_standalone
 */
bool
standalone_eye_has_impl(const char *name);


/*
 *
 * Implementations.
 *
 */

/*!
 * "null": the empty implementation. Accepts and drops the frames of the
 * eye pair, and never sees an eye.
 *
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_eye_null_create(struct xrt_frame_context *xfctx,
                           const struct standalone_eye_config *config,
                           struct standalone_eye_tracker **out_set);

#ifdef __cplusplus
}
#endif
