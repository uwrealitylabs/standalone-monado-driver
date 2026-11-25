// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SLAM tracker for the Standalone headset backed by Monado's
 *         `t_slam`, which runs an external SLAM system (Basalt by default)
 *         through the VIT interface.
 *
 * `t_slam` already owns the sinks, the pose history and the prediction, so
 * this file only adapts it to @ref standalone_slam_tracker:
 *
 * - `t_slam` camera `n` is headset camera `n`, so SLAM takes the front pair
 *   by default, and the side pair too with `SLAM_CAM_COUNT=4`.
 * - `t_slam` reports the pose of the IMU in the SLAM system's world. It is
 *   converted to the pose of the head in a +Y up world here, so that no
 *   device has to know which SLAM system is running.
 *
 * The SLAM system takes its own calibration from the file named by
 * `SLAM_CONFIG`; the headset calibration is only used for the IMU to head
 * transform.
 *
 * @ingroup drv_standalone
 */

#include "math/m_api.h"
#include "math/m_space.h"

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "tracking/t_tracking.h"

#include "../standalone_slam.h"


/*!
 * Adapter from `t_slam` to the Standalone SLAM tracker interface.
 *
 * @implements standalone_slam_tracker
 * @implements xrt_frame_node
 */
struct slam_t_slam
{
	struct standalone_slam_tracker base;
	struct xrt_frame_node node;

	//! The wrapped tracker, owned by the frame context.
	struct xrt_tracked_slam *xts;

	//! Pose of the reference IMU in the head frame.
	struct xrt_pose head_from_imu;
};

/*!
 * Basalt uses +Y down, +Z forward axes. Change basis by 180 degrees around X
 * to the +Y up, -Z forward axes the rest of Monado uses. Same conversion as
 * twrap_slam and the Rift S driver.
 *
 * @todo Depends on the VIT system in use; this assumes Basalt.
 */
static void
correct_relation_from_basalt(struct xrt_space_relation *relation)
{
	relation->pose.position.y = -relation->pose.position.y;
	relation->pose.position.z = -relation->pose.position.z;
	relation->pose.orientation.y = -relation->pose.orientation.y;
	relation->pose.orientation.z = -relation->pose.orientation.z;
	relation->linear_velocity.y = -relation->linear_velocity.y;
	relation->linear_velocity.z = -relation->linear_velocity.z;
	relation->angular_velocity.y = -relation->angular_velocity.y;
	relation->angular_velocity.z = -relation->angular_velocity.z;
}

static void
slam_t_slam_get_head_pose(struct standalone_slam_tracker *sst,
                          int64_t at_timestamp_ns,
                          struct xrt_space_relation *out_relation)
{
	struct slam_t_slam *t = (struct slam_t_slam *)sst;

	struct xrt_space_relation imu_relation = XRT_SPACE_RELATION_ZERO;
	xrt_tracked_slam_get_tracked_pose(t->xts, at_timestamp_ns, &imu_relation);

	const enum xrt_space_relation_flags tracked_bits = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

	if ((imu_relation.relation_flags & tracked_bits) == 0) {
		// No pose yet.
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		return;
	}

	correct_relation_from_basalt(&imu_relation);

	// world_from_head = world_from_imu * imu_from_head
	struct xrt_relation_chain xrc = {0};
	m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &t->head_from_imu);
	m_relation_chain_push_relation(&xrc, &imu_relation);
	m_relation_chain_resolve(&xrc, out_relation);
}

static void
slam_t_slam_break_apart(struct xrt_frame_node *node)
{
	// The wrapped tracker stops itself, it has its own node.
}

static void
slam_t_slam_destroy(struct xrt_frame_node *node)
{
	struct slam_t_slam *t = container_of(node, struct slam_t_slam, node);

	free(t);
}

xrt_result_t
standalone_slam_t_slam_create(struct xrt_frame_context *xfctx,
                              const struct standalone_slam_config *config,
                              struct standalone_slam_tracker **out_sst)
{
	// Reads the SLAM_* options: SLAM_CONFIG, SLAM_CAM_COUNT, VIT_SYSTEM_LIBRARY_PATH, ...
	struct t_slam_tracker_config slam_config;
	t_slam_fill_default_config(&slam_config);

	// Nobody is there to press a button in the debug GUI.
	slam_config.submit_from_start = true;

	int cam_count = slam_config.cam_count;
	if (cam_count < 2 || cam_count > STANDALONE_CAM_TRACKING_COUNT) {
		U_LOG_E("Invalid camera count for Standalone SLAM: %d", cam_count);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	// The SLAM system waits for every camera it was told about.
	const uint32_t wanted_cams = (1u << (uint32_t)cam_count) - 1u;
	if ((config->available_cams & wanted_cams) != wanted_cams) {
		U_LOG_E("Standalone SLAM wants cameras 0x%02x, the source only has 0x%02x", wanted_cams,
		        config->available_cams);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	if ((config->available_imus & STANDALONE_IMU_BIT(STANDALONE_IMU_LEFT)) == 0) {
		U_LOG_E("Standalone SLAM needs IMU 0, the source does not have it");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	struct xrt_tracked_slam *xts = NULL;
	struct xrt_slam_sinks *sinks = NULL;
	if (t_slam_create(xfctx, &slam_config, &xts, &sinks) != 0 || xts == NULL || sinks == NULL) {
		U_LOG_E("Failed to create t_slam");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	if (t_slam_start(xts) != 0) {
		U_LOG_E("Failed to start t_slam");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	struct slam_t_slam *t = U_TYPED_CALLOC(struct slam_t_slam);
	if (t == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	t->xts = xts;
	t->head_from_imu = config->calib->imus[STANDALONE_IMU_LEFT].head_from_imu;

	// SLAM camera n is headset camera n; the one SLAM IMU is the reference IMU.
	for (int i = 0; i < cam_count; i++) {
		t->base.sinks.cams[i] = sinks->cams[i];
	}
	t->base.sinks.cam_count = cam_count;
	t->base.sinks.imus[STANDALONE_IMU_LEFT] = sinks->imus[0];
	t->base.sinks.imu_count = 1;
	t->base.sinks.gt = sinks->gt;
	t->base.sinks.masks[0] = sinks->masks[0];

	t->base.tracking_type = XRT_TRACKING_TYPE_OTHER;
	t->base.get_head_pose = slam_t_slam_get_head_pose;

	t->node.break_apart = slam_t_slam_break_apart;
	t->node.destroy = slam_t_slam_destroy;
	xrt_frame_context_add(xfctx, &t->node);

	*out_sst = &t->base;

	return XRT_SUCCESS;
}
