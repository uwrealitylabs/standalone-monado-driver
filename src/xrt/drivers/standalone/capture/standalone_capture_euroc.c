// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Replay capture for the Standalone headset: Monado's EuRoC player.
 *
 * The player already is an @ref xrt_fs that pushes into an
 * @ref xrt_slam_sinks, so this is only configuration. Dataset camera `n`
 * lands in slot `cams[n]` of the headset sink map: a two-camera dataset
 * feeds the front pair, and a recording of this headset made with the EuRoC
 * recorder in headset camera order replays every lane.
 *
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "euroc/euroc_interface.h"

#include "../standalone_capture.h"


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

	int cam_count = player_config.playback.cam_count;
	if (cam_count < 1 || cam_count > player_config.dataset.cam_count || cam_count > STANDALONE_CAM_TRACKING_COUNT) {
		U_LOG_E("Invalid camera count for Standalone EuRoC replay: %d (dataset has %d)", cam_count,
		        player_config.dataset.cam_count);
		return NULL;
	}

	struct xrt_fs *xfs = euroc_player_create(xfctx, path, &player_config);
	if (xfs == NULL) {
		return NULL;
	}

	out_info->cam_mask = (1u << (uint32_t)cam_count) - 1u;
	out_info->imu_mask = STANDALONE_IMU_BIT(STANDALONE_IMU_LEFT);
	out_info->has_ground_truth = player_config.dataset.has_gt && player_config.playback.gt;
	out_info->is_replay = true;

	return xfs;
}
