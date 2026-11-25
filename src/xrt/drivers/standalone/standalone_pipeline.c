// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The tracking pipeline of the Standalone headset.
 * @ingroup drv_standalone
 */

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "standalone_pipeline.h"


DEBUG_GET_ONCE_OPTION(standalone_euroc_path, "STANDALONE_EUROC_PATH", NULL)
DEBUG_GET_ONCE_OPTION(standalone_capture_impl, "STANDALONE_CAPTURE_IMPL", NULL)
DEBUG_GET_ONCE_OPTION(standalone_slam_impl, "STANDALONE_SLAM_IMPL", NULL)
DEBUG_GET_ONCE_OPTION(standalone_hand_impl, "STANDALONE_HAND_IMPL", NULL)
DEBUG_GET_ONCE_OPTION(standalone_eye_impl, "STANDALONE_EYE_IMPL", NULL)
DEBUG_GET_ONCE_OPTION(standalone_calibration_path, "STANDALONE_CALIBRATION_PATH", NULL)

//! An unset or empty option is no option.
static const char *
non_empty(const char *str)
{
	return (str != NULL && str[0] != '\0') ? str : NULL;
}

static void
pipeline_break_apart(struct xrt_frame_node *node)
{
	// The source and the trackers stop themselves, they have their own nodes.
}

static void
pipeline_destroy(struct xrt_frame_node *node)
{
	struct standalone_pipeline *sp = container_of(node, struct standalone_pipeline, node);

	free(sp);
}

void
standalone_pipeline_fill_default_config(struct standalone_pipeline_config *config)
{
	U_ZERO(config);

	config->euroc_path = non_empty(debug_get_option_standalone_euroc_path());
	config->capture_impl = non_empty(debug_get_option_standalone_capture_impl());
	config->slam_impl = non_empty(debug_get_option_standalone_slam_impl());
	config->hand_impl = non_empty(debug_get_option_standalone_hand_impl());
	config->eye_impl = non_empty(debug_get_option_standalone_eye_impl());
	config->calibration_path = non_empty(debug_get_option_standalone_calibration_path());
}

xrt_result_t
standalone_pipeline_create(struct xrt_frame_context *xfctx,
                           const struct standalone_pipeline_config *config,
                           struct standalone_pipeline **out_sp)
{
	const bool replay = non_empty(config->euroc_path) != NULL;

	// Resolve the defaults.
	const char *capture_impl = non_empty(config->capture_impl);
	if (capture_impl == NULL) {
		capture_impl = replay ? "euroc" : "null";
	}

	const char *slam_impl = non_empty(config->slam_impl);
	if (slam_impl == NULL) {
		slam_impl = (replay && standalone_slam_has_impl("t_slam")) ? "t_slam" : "null";
		if (replay && !standalone_slam_has_impl("t_slam")) {
			U_LOG_W(
			    "Replaying a dataset without a SLAM tracker: t_slam is not in this build "
			    "(needs XRT_FEATURE_SLAM)");
		}
	}

	struct standalone_pipeline *sp = U_TYPED_CALLOC(struct standalone_pipeline);
	if (sp == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	// From here on the frame context owns the pipeline, also if we fail.
	sp->node.break_apart = pipeline_break_apart;
	sp->node.destroy = pipeline_destroy;
	xrt_frame_context_add(xfctx, &sp->node);

	U_LOG_I("Standalone pipeline for %s", STANDALONE_HW_REVISION);

	// 1. Calibration. A file that cannot be loaded is reported there; nominal values are used.
	(void)standalone_calibration_load(config->calibration_path, &sp->calib);

	// 2. The source. Created first so the trackers know what it delivers, started last.
	struct standalone_capture_config capture_config = {
	    .impl = capture_impl,
	    .replay_path = non_empty(config->euroc_path),
	};

	sp->source = standalone_capture_create(xfctx, &capture_config, &sp->source_info);
	if (sp->source == NULL) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	// 3. The trackers.
	struct standalone_slam_config slam_config = {
	    .impl = slam_impl,
	    .calib = &sp->calib,
	    .available_cams = sp->source_info.cam_mask,
	    .available_imus = sp->source_info.imu_mask,
	};

	xrt_result_t xret = standalone_slam_create(xfctx, &slam_config, &sp->slam);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	struct standalone_hand_config hand_config = {
	    .impl = non_empty(config->hand_impl),
	    .calib = &sp->calib,
	    .available_cams = sp->source_info.cam_mask,
	    // The one tracker to tracker connection: hand bounding boxes for SLAM to ignore.
	    .masks_sink = sp->slam->sinks.masks[0],
	};

	xret = standalone_hand_create(xfctx, &hand_config, &sp->hand);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	struct standalone_eye_config eye_config = {
	    .impl = non_empty(config->eye_impl),
	    .calib = &sp->calib,
	    .available_cams = sp->source_info.cam_mask,
	};

	xret = standalone_eye_create(xfctx, &eye_config, &sp->eye);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	// 4. Wiring: one entry sink per sensor, a splitter where trackers share one.
	const struct xrt_slam_sinks *consumers[] = {
	    &sp->slam->sinks,
	    &sp->hand->sinks,
	    &sp->eye->sinks,
	};

	standalone_sinks_merge(xfctx, consumers, ARRAY_SIZE(consumers), &sp->entry_sinks);

	const uint32_t consumed_cams = standalone_sinks_cam_mask(&sp->entry_sinks);
	const uint32_t missing_cams = consumed_cams & ~sp->source_info.cam_mask;
	if (missing_cams != 0) {
		// A tracker ignored `available_cams`. It will wait for frames that never come.
		U_LOG_W("Standalone trackers consume cameras 0x%02x that the source does not deliver", missing_cams);
	}

	// 5. Start. Data flows from here on.
	xret = xrt_fs_slam_stream_start(sp->source, &sp->entry_sinks);
	if (xret != XRT_SUCCESS) {
		U_LOG_E("Failed to start the Standalone capture source");
		return xret;
	}

	U_LOG_I("Standalone pipeline running: cameras 0x%02x, IMUs 0x%x consumed", consumed_cams,
	        standalone_sinks_imu_mask(&sp->entry_sinks));

	*out_sp = sp;

	return XRT_SUCCESS;
}
