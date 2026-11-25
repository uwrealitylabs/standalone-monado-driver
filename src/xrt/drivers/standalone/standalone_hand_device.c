// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Standalone hand tracking device: exposes both hands of the hand
 *         tracker to the core.
 *
 * The same for every hand tracker implementation. The tracker reports
 * joints in the head frame; this device places them in the tracking space
 * with the head pose of the time they are for.
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

#include "standalone_hand.h"
#include "standalone_interface.h"

#include <stdio.h>


DEBUG_GET_ONCE_LOG_OPTION(standalone_hand_log, "STANDALONE_HAND_LOG", U_LOGGING_WARN)

/*!
 * A standalone hand tracking device.
 *
 * @implements xrt_device
 */
struct standalone_hand_device
{
	struct xrt_device base;

	enum u_logging_level log_level;

	//! Where the joints come from. Owned by the frame context, outlives the device.
	struct standalone_hand_tracker *tracker;

	//! Where the head pose comes from.
	struct xrt_device *head;
};

static inline struct standalone_hand_device *
standalone_hand_device(struct xrt_device *xdev)
{
	return (struct standalone_hand_device *)xdev;
}

static void
standalone_hand_device_destroy(struct xrt_device *xdev)
{
	struct standalone_hand_device *shd = standalone_hand_device(xdev);

	u_var_remove_root(shd);

	u_device_free(&shd->base);
}

static xrt_result_t
standalone_hand_device_get_tracked_pose(struct xrt_device *xdev,
                                        enum xrt_input_name name,
                                        int64_t at_timestamp_ns,
                                        struct xrt_space_relation *out_relation)
{
	struct standalone_hand_device *shd = standalone_hand_device(xdev);

	// Hands are only exposed as joint sets.
	U_LOG_XDEV_UNSUPPORTED_INPUT(&shd->base, shd->log_level, name);
	return XRT_ERROR_INPUT_UNSUPPORTED;
}

static xrt_result_t
standalone_hand_device_get_hand_tracking(struct xrt_device *xdev,
                                         enum xrt_input_name name,
                                         int64_t desired_timestamp_ns,
                                         struct xrt_hand_joint_set *out_value,
                                         int64_t *out_timestamp_ns)
{
	struct standalone_hand_device *shd = standalone_hand_device(xdev);

	enum xrt_hand hand;
	switch (name) {
	case XRT_INPUT_HT_UNOBSTRUCTED_LEFT: hand = XRT_HAND_LEFT; break;
	case XRT_INPUT_HT_UNOBSTRUCTED_RIGHT: hand = XRT_HAND_RIGHT; break;
	default: U_LOG_XDEV_UNSUPPORTED_INPUT(&shd->base, shd->log_level, name); return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	standalone_hand_tracker_get_hand(shd->tracker, hand, desired_timestamp_ns, out_value, out_timestamp_ns);

	if (!out_value->is_active) {
		return XRT_SUCCESS;
	}

	// The joints are in the head frame of the time they are for.
	struct xrt_space_relation head_relation = XRT_SPACE_RELATION_ZERO;
	xrt_result_t xret =
	    xrt_device_get_tracked_pose(shd->head, XRT_INPUT_GENERIC_HEAD_POSE, *out_timestamp_ns, &head_relation);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	// tracking_from_hand = tracking_from_head * head_from_hand
	struct xrt_relation_chain xrc = {0};
	m_relation_chain_push_relation(&xrc, &out_value->hand_pose);
	m_relation_chain_push_relation(&xrc, &head_relation);
	m_relation_chain_resolve(&xrc, &out_value->hand_pose);

	return XRT_SUCCESS;
}

struct xrt_device *
standalone_hand_device_create(struct standalone_hand_tracker *tracker, struct xrt_device *head)
{
	if (tracker == NULL || head == NULL) {
		U_LOG_E("The Standalone hand device needs a hand tracker and the head device");
		return NULL;
	}

	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_NO_FLAGS;

	// Two inputs: the left and the right hand.
	struct standalone_hand_device *shd = U_DEVICE_ALLOCATE(struct standalone_hand_device, flags, 2, 0);
	if (shd == NULL) {
		return NULL;
	}

	shd->log_level = debug_get_log_option_standalone_hand_log();
	shd->tracker = tracker;
	shd->head = head;

	u_device_populate_function_pointers(&shd->base, standalone_hand_device_get_tracked_pose,
	                                    standalone_hand_device_destroy);
	shd->base.update_inputs = u_device_noop_update_inputs;
	shd->base.get_hand_tracking = standalone_hand_device_get_hand_tracking;

	// Hand poses are composed with the head pose, so they live in the head's tracking space.
	shd->base.tracking_origin = head->tracking_origin;

	snprintf(shd->base.str, XRT_DEVICE_NAME_LEN, "Standalone Hand Tracker");
	snprintf(shd->base.serial, XRT_DEVICE_NAME_LEN, "Standalone Hand Tracker S/N");

	shd->base.name = XRT_DEVICE_HAND_TRACKER;
	shd->base.device_type = XRT_DEVICE_TYPE_HAND_TRACKER;
	shd->base.inputs[0].name = XRT_INPUT_HT_UNOBSTRUCTED_LEFT;
	shd->base.inputs[1].name = XRT_INPUT_HT_UNOBSTRUCTED_RIGHT;
	shd->base.supported.orientation_tracking = true;
	shd->base.supported.position_tracking = true;
	shd->base.supported.hand_tracking = true;

	u_var_add_root(shd, "Standalone Hand Tracker", true);
	u_var_add_log_level(shd, &shd->log_level, "log_level");

	return &shd->base;
}
