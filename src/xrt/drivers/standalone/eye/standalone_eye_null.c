// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Empty eye tracker for the Standalone headset.
 *
 * Has every part a real implementation has, and tracks nothing: it accepts
 * and drops the frames of the eye pair, and never sees an eye. Copy this
 * file to start a real implementation; the places that need code are marked
 * with `@todo`.
 *
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "../standalone_eye.h"

#include <inttypes.h>


/*!
 * Empty eye tracker.
 *
 * @implements standalone_eye_tracker
 * @implements xrt_frame_node
 */
struct eye_null
{
	struct standalone_eye_tracker base;
	struct xrt_frame_node node;

	const struct standalone_calibration *calib;

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
eye_null_receive_frame(struct xrt_frame_sink *sink, struct xrt_frame *xf)
{
	struct standalone_frame_slot *slot = standalone_frame_slot(sink);
	struct eye_null *t = (struct eye_null *)slot->owner;

	/*
	 * @todo Keep the frame for the tracking thread. This runs on the
	 * capture thread: take a reference and return, do not track here.
	 */
	t->frame_count[slot->cam]++;
}


/*
 *
 * Output.
 *
 */

static void
eye_null_get_gaze(struct standalone_eye_tracker *set, int64_t at_timestamp_ns, struct standalone_gaze *out_gaze)
{
	//! @todo Answer from the latest results of the tracking thread.
	U_ZERO(out_gaze);
	out_gaze->timestamp_ns = at_timestamp_ns;
}


/*
 *
 * Frame node functions.
 *
 */

static void
eye_null_break_apart(struct xrt_frame_node *node)
{
	//! @todo Stop and join the tracking thread, drop every frame reference held.
}

static void
eye_null_destroy(struct xrt_frame_node *node)
{
	struct eye_null *t = container_of(node, struct eye_null, node);

	for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
		if (t->base.sinks.cams[i] != NULL) {
			U_LOG_D("Standalone null eye tracker dropped %" PRIu64 " frames from %s", t->frame_count[i],
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
standalone_eye_null_create(struct xrt_frame_context *xfctx,
                           const struct standalone_eye_config *config,
                           struct standalone_eye_tracker **out_set)
{
	struct eye_null *t = U_TYPED_CALLOC(struct eye_null);
	if (t == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	t->calib = config->calib;

	// Consume the default cameras, as far as the source has them.
	standalone_sinks_init(                                    //
	    t->cam_slots,                                         //
	    NULL,                                                 //
	    t,                                                    //
	    STANDALONE_CAMS_EYE_DEFAULT & config->available_cams, //
	    0,                                                    //
	    eye_null_receive_frame,                               //
	    NULL,                                                 //
	    &t->base.sinks);                                      //

	t->base.get_gaze = eye_null_get_gaze;

	t->node.break_apart = eye_null_break_apart;
	t->node.destroy = eye_null_destroy;
	xrt_frame_context_add(xfctx, &t->node);

	*out_set = &t->base;

	return XRT_SUCCESS;
}
