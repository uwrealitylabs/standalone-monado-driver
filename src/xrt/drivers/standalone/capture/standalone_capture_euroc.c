// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Replay capture for the Standalone headset: Monado's EuRoC player.
 *
 * The player already is an @ref xrt_fs that pushes into an
 * @ref xrt_slam_sinks, but it indexes those sinks by dataset camera. This
 * wraps it in a source that is started with the headset sink map and hands
 * the player the slots the dataset belongs in:
 *
 * - A dataset of the headset's motion starts at slot 0. A two-camera
 *   dataset then feeds the front pair and the reference IMU, and a
 *   recording of this headset made in headset camera order replays every
 *   lane.
 * - A hand or eye dataset starts at the slot of its first camera, e.g. 6
 *   for the eye pair. Its IMU and ground truth are not connected.
 *
 * The player asserts that every dataset has an `imu0/data.csv` and that all
 * cameras of a dataset share timestamps, so datasets with different frame
 * rates are replayed in separate runs.
 *
 * @ingroup drv_standalone
 */

#include "util/u_frameserver.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "euroc/euroc_interface.h"

#include "../standalone_capture.h"

#include <stdio.h>


/*!
 * The EuRoC player behind the headset sink map.
 *
 * @implements xrt_fs
 * @implements xrt_frame_node
 */
struct capture_euroc
{
	struct xrt_fs base;
	struct xrt_frame_node node;

	//! The player, owned by the frame context.
	struct xrt_fs *player;

	uint32_t first_cam; //!< Headset slot of dataset camera 0.
	uint32_t cam_count; //!< Cameras played from the dataset.
};

static inline struct capture_euroc *
capture_euroc(struct xrt_fs *xfs)
{
	return (struct capture_euroc *)xfs;
}

//! Is the dataset one of the headset's own motion, with a usable IMU and ground truth?
static bool
has_motion(uint32_t first_cam)
{
	return first_cam == 0;
}


/*
 *
 * xrt_fs functions: everything but the tracking start goes to the player as is.
 *
 */

static xrt_result_t
capture_euroc_enumerate_modes(struct xrt_fs *xfs, struct xrt_fs_mode **out_modes, uint32_t *out_count)
{
	return xrt_fs_enumerate_modes(capture_euroc(xfs)->player, out_modes, out_count);
}

static xrt_result_t
capture_euroc_configure_capture(struct xrt_fs *xfs, struct xrt_fs_capture_parameters *cp)
{
	return xrt_fs_configure_capture(capture_euroc(xfs)->player, cp);
}

static xrt_result_t
capture_euroc_stream_start(struct xrt_fs *xfs,
                           struct xrt_frame_sink *xs,
                           enum xrt_fs_capture_type capture_type,
                           uint32_t descriptor_index)
{
	return xrt_fs_stream_start(capture_euroc(xfs)->player, xs, capture_type, descriptor_index);
}

static xrt_result_t
capture_euroc_slam_stream_start(struct xrt_fs *xfs, struct xrt_slam_sinks *sinks)
{
	struct capture_euroc *c = capture_euroc(xfs);

	// From headset slots to dataset cameras. The player copies the struct.
	struct xrt_slam_sinks dataset_sinks;
	standalone_sinks_for_dataset(sinks, c->first_cam, c->cam_count, has_motion(c->first_cam), &dataset_sinks);

	return xrt_fs_slam_stream_start(c->player, &dataset_sinks);
}

static xrt_result_t
capture_euroc_stream_stop(struct xrt_fs *xfs)
{
	return xrt_fs_stream_stop(capture_euroc(xfs)->player);
}

static xrt_result_t
capture_euroc_is_running(struct xrt_fs *xfs, bool *out_is_running)
{
	return xrt_fs_is_running(capture_euroc(xfs)->player, out_is_running);
}


/*
 *
 * Frame node functions.
 *
 */

static void
capture_euroc_break_apart(struct xrt_frame_node *node)
{
	// The player stops itself, it has its own node.
}

static void
capture_euroc_destroy(struct xrt_frame_node *node)
{
	struct capture_euroc *c = container_of(node, struct capture_euroc, node);

	free(c);
}


/*
 *
 * Exported functions.
 *
 */

struct xrt_fs *
standalone_capture_euroc_create(struct xrt_frame_context *xfctx,
                                const struct standalone_capture_config *config,
                                struct standalone_capture_info *out_info)
{
	const char *path = config->replay_path;
	if (path == NULL || path[0] == '\0') {
		U_LOG_E("Standalone EuRoC replay needs a dataset path (STANDALONE_EUROC_PATH)");
		return NULL;
	}

	// Reads the dataset to find its camera count and ground truth, and the EUROC_* options.
	struct euroc_player_config player_config;
	euroc_player_fill_default_config_for(&player_config, path);

	// No debug GUI is needed to start the playback.
	player_config.playback.play_from_start = true;
	// The core asks for poses in the monotonic clock, so rebase the dataset's timestamps onto it.
	player_config.playback.use_source_ts = false;

	const int cam_count = player_config.playback.cam_count;
	if (cam_count < 1 || cam_count > player_config.dataset.cam_count) {
		U_LOG_E("Invalid camera count for Standalone EuRoC replay: %d (dataset has %d)", cam_count,
		        player_config.dataset.cam_count);
		return NULL;
	}

	const uint32_t first_cam = config->replay_first_cam;
	if (first_cam + (uint32_t)cam_count > STANDALONE_CAM_TRACKING_COUNT) {
		U_LOG_E(
		    "Standalone EuRoC replay of %d cameras does not fit at slot %u (STANDALONE_EUROC_FIRST_CAM): "
		    "the headset has %d tracking cameras",
		    cam_count, first_cam, STANDALONE_CAM_TRACKING_COUNT);
		return NULL;
	}

	// The dataset's ground truth is a head pose, of no use to a hand or eye dataset.
	if (!has_motion(first_cam)) {
		player_config.playback.gt = false;
	}

	struct xrt_fs *player = euroc_player_create(xfctx, path, &player_config);
	if (player == NULL) {
		return NULL;
	}

	struct capture_euroc *c = U_TYPED_CALLOC(struct capture_euroc);
	if (c == NULL) {
		return NULL;
	}

	c->player = player;
	c->first_cam = first_cam;
	c->cam_count = (uint32_t)cam_count;

	// Frame server: the player's identity, so frames still name their source.
	struct xrt_fs *xfs = &c->base;
	u_frameserver_populate_function_pointers(xfs);
	xfs->enumerate_modes = capture_euroc_enumerate_modes;
	xfs->configure_capture = capture_euroc_configure_capture;
	xfs->stream_start = capture_euroc_stream_start;
	xfs->slam_stream_start = capture_euroc_slam_stream_start;
	xfs->stream_stop = capture_euroc_stream_stop;
	xfs->is_running = capture_euroc_is_running;
	(void)snprintf(xfs->name, sizeof(xfs->name), "%s", player->name);
	(void)snprintf(xfs->product, sizeof(xfs->product), "%s", player->product);
	(void)snprintf(xfs->manufacturer, sizeof(xfs->manufacturer), "%s", player->manufacturer);
	(void)snprintf(xfs->serial, sizeof(xfs->serial), "%s", player->serial);
	xfs->source_id = player->source_id;

	c->node.break_apart = capture_euroc_break_apart;
	c->node.destroy = capture_euroc_destroy;
	xrt_frame_context_add(xfctx, &c->node);

	const struct standalone_camera_info *first_info = standalone_hw_camera_info((enum standalone_camera)first_cam);
	U_LOG_I("Standalone EuRoC replay: %d dataset cameras into slots %u to %u (from %s)%s", cam_count, first_cam,
	        first_cam + (uint32_t)cam_count - 1, first_info->name,
	        has_motion(first_cam) ? ", with IMU" : ", IMU and ground truth not used");

	out_info->cam_mask = ((1u << (uint32_t)cam_count) - 1u) << first_cam;
	out_info->imu_mask = has_motion(first_cam) ? STANDALONE_IMU_BIT(STANDALONE_IMU_LEFT) : 0;
	out_info->has_ground_truth = has_motion(first_cam) && player_config.dataset.has_gt && player_config.playback.gt;
	out_info->is_replay = true;

	return xfs;
}
