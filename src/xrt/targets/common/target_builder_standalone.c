// Copyright 2022-2023, Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Standalone driver builder.
 * @author Nathaniel Reilly <nathanmreilly@gmail.com>
 * @ingroup xrt_iface
 */

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_system.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_config_json.h"

#include "target_builder_helpers.h"
#include "target_builder_interface.h"

#include "standalone/standalone_interface.h"

#if defined(XRT_FEATURE_SLAM) && defined(XRT_BUILD_DRIVER_EUROC)
#include "euroc/euroc_interface.h"
#include "tracking/t_tracking.h"
#endif

#include <assert.h>

DEBUG_GET_ONCE_OPTION(standalone_euroc_path, "STANDALONE_EUROC_PATH", NULL)

/*
 *
 * Helper functions.
 *
 */

static const char *driver_list[] = {
    "standalone",
};

#if defined(XRT_FEATURE_SLAM) && defined(XRT_BUILD_DRIVER_EUROC)
static xrt_result_t
standalone_start_euroc_replay(struct xrt_frame_context *xfctx, const char *path, struct xrt_tracked_slam **out_slam)
{
	struct euroc_player_config player_config;
	euroc_player_fill_default_config_for(&player_config, path);
	player_config.playback.play_from_start = true;
	// The application requests poses in the runtime's monotonic clock.
	player_config.playback.use_source_ts = false;

	struct t_slam_tracker_config slam_config;
	t_slam_fill_default_config(&slam_config);
	slam_config.submit_from_start = true;
	slam_config.cam_count = player_config.playback.cam_count;
	if (slam_config.cam_count < 2 || slam_config.cam_count > player_config.dataset.cam_count) {
		U_LOG_E("Invalid camera count for Standalone SLAM replay: %d", slam_config.cam_count);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	struct xrt_slam_sinks *sinks = NULL;
	if (t_slam_create(xfctx, &slam_config, out_slam, &sinks) != 0 || t_slam_start(*out_slam) != 0) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	struct xrt_fs *player = euroc_player_create(xfctx, path, &player_config);
	return xrt_fs_slam_stream_start(player, sinks);
}
#endif


/*
 *
 * Member functions.
 *
 */

static xrt_result_t
standalone_estimate_system(struct xrt_builder *xb,
                          cJSON *config,
                          struct xrt_prober *xp,
                          struct xrt_builder_estimate *estimate)
{
	estimate->certain.head = true;
	//! @todo Set left and right to true once hand devices are created.
	estimate->certain.left = false;
	estimate->certain.right = false;
	estimate->priority = 1;

	return XRT_SUCCESS;
}

static xrt_result_t
standalone_open_system_impl(struct xrt_builder *xb,
                           cJSON *config,
                           struct xrt_prober *xp,
                           struct xrt_tracking_origin *origin,
                           struct xrt_system_devices *xsysd,
                           struct xrt_frame_context *xfctx,
                           struct t_builder_options *tbo)
{
	struct xrt_tracked_slam *slam = NULL;
	const char *path = debug_get_option_standalone_euroc_path();
	if (path != NULL && path[0] != '\0') {
#if defined(XRT_FEATURE_SLAM) && defined(XRT_BUILD_DRIVER_EUROC)
		xrt_result_t ret = standalone_start_euroc_replay(xfctx, path, &slam);
		if (ret != XRT_SUCCESS) {
			return ret;
		}
#else
		U_LOG_E("STANDALONE_EUROC_PATH requires XRT_FEATURE_SLAM and XRT_BUILD_DRIVER_EUROC");
		return XRT_ERROR_DEVICE_CREATION_FAILED;
#endif
	}

	tbo->head = standalone_create(slam);
	if (tbo->head == NULL) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}
	xsysd->static_xdevs[xsysd->static_xdev_count++] = tbo->head;

	return XRT_SUCCESS;
}

static void
standalone_destroy(struct xrt_builder *xb)
{
	free(xb);
}


/*
 *
 * 'Exported' functions.
 *
 */

struct xrt_builder *
t_builder_standalone_create(void)
{
	struct t_builder *ub = U_TYPED_CALLOC(struct t_builder);

	// xrt_builder fields.
	ub->base.estimate_system = standalone_estimate_system;
	ub->base.open_system = t_builder_open_system_static_roles;
	ub->base.destroy = standalone_destroy;
	ub->base.identifier = "standalone";
	ub->base.name = "Standalone device builder";
	ub->base.driver_identifiers = driver_list;
	ub->base.driver_identifier_count = ARRAY_SIZE(driver_list);
	ub->base.exclude_from_automatic_discovery = false;

	// t_builder fields.
	ub->open_system_static_roles = standalone_open_system_impl;

	return &ub->base;
}
