// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Standalone eye tracking device: exposes the gaze of the eye
 *         tracker to the core.
 *
 * The same for every eye tracker implementation. The tracker reports the
 * gaze in the head frame; this device places it in the tracking space with
 * the head pose of the same time. The core reads it as the
 * @ref XRT_INPUT_GENERIC_EYE_GAZE_POSE input, which is what OpenXR's eye
 * gaze interaction uses.
 *
 * @ingroup drv_standalone
 */

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_api.h"
#include "math/m_space.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_var.h"

#include "standalone_eye.h"
#include "standalone_interface.h"

#include <stdio.h>


DEBUG_GET_ONCE_LOG_OPTION(standalone_eye_log, "STANDALONE_EYE_LOG", U_LOGGING_WARN)

/*!
 * A standalone eye tracking device.
 *
 * @implements xrt_device
 */
struct standalone_eye_device
{
	struct xrt_device base;

	enum u_logging_level log_level;

	//! Where the gaze comes from. Owned by the frame context, outlives the device.
	struct standalone_eye_tracker *tracker;

	//! Where the head pose comes from.
	struct xrt_device *head;
};

static inline struct standalone_eye_device *
standalone_eye_device(struct xrt_device *xdev)
{
	return (struct standalone_eye_device *)xdev;
}

static void
standalone_eye_device_destroy(struct xrt_device *xdev)
{
	struct standalone_eye_device *sed = standalone_eye_device(xdev);

	u_var_remove_root(sed);

	u_device_free(&sed->base);
}

static xrt_result_t
standalone_eye_device_get_tracked_pose(struct xrt_device *xdev,
                                       enum xrt_input_name name,
                                       int64_t at_timestamp_ns,
                                       struct xrt_space_relation *out_relation)
{
	struct standalone_eye_device *sed = standalone_eye_device(xdev);

	if (name != XRT_INPUT_GENERIC_EYE_GAZE_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&sed->base, sed->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct standalone_gaze gaze;
	standalone_eye_tracker_get_gaze(sed->tracker, at_timestamp_ns, &gaze);

	if (gaze.combined.relation_flags == XRT_SPACE_RELATION_BITMASK_NONE) {
		// The eyes are not seen.
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return XRT_SUCCESS;
	}

	// The gaze is in the head frame.
	struct xrt_space_relation head_relation = XRT_SPACE_RELATION_ZERO;
	xrt_result_t xret =
	    xrt_device_get_tracked_pose(sed->head, XRT_INPUT_GENERIC_HEAD_POSE, at_timestamp_ns, &head_relation);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	// tracking_from_gaze = tracking_from_head * head_from_gaze
	struct xrt_relation_chain xrc = {0};
	m_relation_chain_push_relation(&xrc, &gaze.combined);
	m_relation_chain_push_relation(&xrc, &head_relation);
	m_relation_chain_resolve(&xrc, out_relation);

	return XRT_SUCCESS;
}

struct xrt_device *
standalone_eye_device_create(struct standalone_eye_tracker *tracker, struct xrt_device *head)
{
	if (tracker == NULL || head == NULL) {
		U_LOG_E("The Standalone eye device needs an eye tracker and the head device");
		return NULL;
	}

	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_NO_FLAGS;

	// One input: the combined gaze.
	struct standalone_eye_device *sed = U_DEVICE_ALLOCATE(struct standalone_eye_device, flags, 1, 0);
	if (sed == NULL) {
		return NULL;
	}

	sed->log_level = debug_get_log_option_standalone_eye_log();
	sed->tracker = tracker;
	sed->head = head;

	u_device_populate_function_pointers(&sed->base, standalone_eye_device_get_tracked_pose,
	                                    standalone_eye_device_destroy);
	sed->base.update_inputs = u_device_noop_update_inputs;

	// The gaze is composed with the head pose, so it lives in the head's tracking space.
	sed->base.tracking_origin = head->tracking_origin;

	snprintf(sed->base.str, XRT_DEVICE_NAME_LEN, "Standalone Eye Tracker");
	snprintf(sed->base.serial, XRT_DEVICE_NAME_LEN, "Standalone Eye Tracker S/N");

	sed->base.name = XRT_DEVICE_EYE_GAZE_INTERACTION;
	sed->base.device_type = XRT_DEVICE_TYPE_EYE_TRACKER;
	sed->base.inputs[0].name = XRT_INPUT_GENERIC_EYE_GAZE_POSE;
	sed->base.supported.orientation_tracking = true;
	sed->base.supported.position_tracking = true;
	sed->base.supported.eye_gaze = true;

	u_var_add_root(sed, "Standalone Eye Tracker", true);
	u_var_add_log_level(sed, &sed->log_level, "log_level");

	return &sed->base;
}
