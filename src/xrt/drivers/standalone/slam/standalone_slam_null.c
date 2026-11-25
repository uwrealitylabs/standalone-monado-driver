// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Empty SLAM tracker for the Standalone headset.
 *
 * Has every part a real implementation has, and tracks nothing: it accepts
 * and drops the front pair and the IMUs, and reports an identity pose. Copy
 * this file to start a real implementation; the places that need code are
 * marked with `@todo`.
 *
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "../standalone_slam.h"

#include <inttypes.h>


/*!
 * Empty SLAM tracker.
 *
 * @implements standalone_slam_tracker
 * @implements xrt_frame_node
 */
struct slam_null
{
	struct standalone_slam_tracker base;
	struct xrt_frame_node node;

	const struct standalone_calibration *calib;

	struct standalone_frame_slot cam_slots[STANDALONE_CAM_TRACKING_COUNT];
	struct standalone_imu_slot imu_slots[STANDALONE_IMU_COUNT];

	// Written only by the thread that pushes that sensor.
	uint64_t frame_count[STANDALONE_CAM_TRACKING_COUNT];
	uint64_t imu_count[STANDALONE_IMU_COUNT];
};


/*
 *
 * Inputs.
 *
 */

static void
slam_null_receive_frame(struct xrt_frame_sink *sink, struct xrt_frame *xf)
{
	struct standalone_frame_slot *slot = standalone_frame_slot(sink);
	struct slam_null *t = (struct slam_null *)slot->owner;

	/*
	 * @todo Hand the frame to the SLAM system. This runs on the capture
	 * thread: copy or reference the frame and return, do not track here.
	 */
	t->frame_count[slot->cam]++;
}

static void
slam_null_receive_imu(struct xrt_imu_sink *sink, struct xrt_imu_sample *sample)
{
	struct standalone_imu_slot *slot = standalone_imu_slot(sink);
	struct slam_null *t = (struct slam_null *)slot->owner;

	//! @todo Hand the sample to the SLAM system, and keep it for prediction.
	t->imu_count[slot->imu]++;
}


/*
 *
 * Output.
 *
 */

static void
slam_null_get_head_pose(struct standalone_slam_tracker *sst,
                        int64_t at_timestamp_ns,
                        struct xrt_space_relation *out_relation)
{
	/*
	 * @todo Answer from a history of estimates: interpolate inside it,
	 * predict past its end. Until then the head sits still at the origin,
	 * which is enough for the compositor and apps to run.
	 */
	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	relation.pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	relation.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                          XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	*out_relation = relation;
}


/*
 *
 * Frame node functions.
 *
 */

static void
slam_null_break_apart(struct xrt_frame_node *node)
{
	//! @todo Stop and join the tracker's threads. Nothing is pushed to us after every node has done this.
}

static void
slam_null_destroy(struct xrt_frame_node *node)
{
	struct slam_null *t = container_of(node, struct slam_null, node);

	for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
		if (t->base.sinks.cams[i] != NULL) {
			U_LOG_D("Standalone null SLAM tracker dropped %" PRIu64 " frames from %s", t->frame_count[i],
			        standalone_hw_camera_info((enum standalone_camera)i)->name);
		}
	}
	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		if (t->base.sinks.imus[i] != NULL) {
			U_LOG_D("Standalone null SLAM tracker dropped %" PRIu64 " samples from %s", t->imu_count[i],
			        standalone_hw_imu_info((enum standalone_imu)i)->name);
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
standalone_slam_null_create(struct xrt_frame_context *xfctx,
                            const struct standalone_slam_config *config,
                            struct standalone_slam_tracker **out_sst)
{
	struct slam_null *t = U_TYPED_CALLOC(struct slam_null);
	if (t == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	t->calib = config->calib;

	// Consume the default sensors, as far as the source has them.
	standalone_sinks_init(                                     //
	    t->cam_slots,                                          //
	    t->imu_slots,                                          //
	    t,                                                     //
	    STANDALONE_CAMS_SLAM_DEFAULT & config->available_cams, //
	    STANDALONE_IMUS_ALL & config->available_imus,          //
	    slam_null_receive_frame,                               //
	    slam_null_receive_imu,                                 //
	    &t->base.sinks);                                       //

	// Not a measurement of the headset's motion.
	t->base.tracking_type = XRT_TRACKING_TYPE_NONE;
	t->base.get_head_pose = slam_null_get_head_pose;

	t->node.break_apart = slam_null_break_apart;
	t->node.destroy = slam_null_destroy;
	xrt_frame_context_add(xfctx, &t->node);

	*out_sst = &t->base;

	return XRT_SUCCESS;
}
