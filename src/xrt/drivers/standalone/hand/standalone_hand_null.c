// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Empty hand tracker for the Standalone headset.
 *
 * Has every part a real implementation has, and tracks nothing: it accepts
 * and drops the frames of the six hand-tracking cameras, and never sees a
 * hand. Copy this file to start a real implementation; the places that need
 * code are marked with `@todo`.
 *
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "../standalone_hand.h"

#include <inttypes.h>


/*!
 * Empty hand tracker.
 *
 * @implements standalone_hand_tracker
 * @implements xrt_frame_node
 */
struct hand_null
{
	struct standalone_hand_tracker base;
	struct xrt_frame_node node;

	const struct standalone_calibration *calib;

	//! Where hand bounding boxes for SLAM would go, may be NULL.
	struct xrt_device_masks_sink *masks_sink;

	struct standalone_frame_slot cam_slots[STANDALONE_CAM_TRACKING_COUNT];

	// Written only by the thread that pushes that camera.
	uint64_t frame_count[STANDALONE_CAM_TRACKING_COUNT];
};


/*
 *
 * Inputs.
 *
 */

static void
hand_null_receive_frame(struct xrt_frame_sink *sink, struct xrt_frame *xf)
{
	struct standalone_frame_slot *slot = standalone_frame_slot(sink);
	struct hand_null *t = (struct hand_null *)slot->owner;

	/*
	 * @todo Keep the frame for the tracking thread. This runs on the
	 * capture thread: take a reference and return, do not track here.
	 * A pair is complete when its right frame has arrived (left is
	 * always pushed first, with the same timestamp).
	 */
	t->frame_count[slot->cam]++;
}


/*
 *
 * Output.
 *
 */

static void
hand_null_get_hand(struct standalone_hand_tracker *sht,
                   enum xrt_hand hand,
                   int64_t desired_timestamp_ns,
                   struct xrt_hand_joint_set *out_value,
                   int64_t *out_timestamp_ns)
{
	//! @todo Answer from the latest results of the tracking thread.
	U_ZERO(out_value);
	out_value->is_active = false;

	*out_timestamp_ns = desired_timestamp_ns;
}


/*
 *
 * Frame node functions.
 *
 */

static void
hand_null_break_apart(struct xrt_frame_node *node)
{
	//! @todo Stop and join the tracking thread, drop every frame reference held.
}

static void
hand_null_destroy(struct xrt_frame_node *node)
{
	struct hand_null *t = container_of(node, struct hand_null, node);

	for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
		if (t->base.sinks.cams[i] != NULL) {
			U_LOG_D("Standalone null hand tracker dropped %" PRIu64 " frames from %s", t->frame_count[i],
			        standalone_hw_camera_info((enum standalone_camera)i)->name);
		}
	}

	free(t);
}


/*
 *
 * Exported functions.
 *
 */

xrt_result_t
standalone_hand_null_create(struct xrt_frame_context *xfctx,
                            const struct standalone_hand_config *config,
                            struct standalone_hand_tracker **out_sht)
{
	struct hand_null *t = U_TYPED_CALLOC(struct hand_null);
	if (t == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	t->calib = config->calib;
	t->masks_sink = config->masks_sink;

	// Consume the default cameras, as far as the source has them.
	standalone_sinks_init(                                     //
	    t->cam_slots,                                          //
	    NULL,                                                  //
	    t,                                                     //
	    STANDALONE_CAMS_HAND_DEFAULT & config->available_cams, //
	    0,                                                     //
	    hand_null_receive_frame,                               //
	    NULL,                                                  //
	    &t->base.sinks);                                       //

	t->base.get_hand = hand_null_get_hand;

	t->node.break_apart = hand_null_break_apart;
	t->node.destroy = hand_null_destroy;
	xrt_frame_context_add(xfctx, &t->node);

	*out_sht = &t->base;

	return XRT_SUCCESS;
}
