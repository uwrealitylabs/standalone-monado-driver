// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Empty capture implementation for the Standalone headset.
 *
 * Has every part a real implementation has, and reads no hardware: it
 * starts, stops and reports all sensors, and pushes nothing. Copy this file
 * to start a real implementation; the places that need code are marked
 * with `@todo`.
 *
 * A real implementation reads the seven host streams (see
 * @ref standalone_host_stream), splits the two pair composites, converts to
 * the contract's pixel format, translates hardware time to the monotonic
 * clock and pushes through the output stage. It reads the two IMUs the same
 * way. See the data contract in standalone_sinks.h.
 *
 * @ingroup drv_standalone
 */

#include "util/u_debug.h"
#include "util/u_frameserver.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "../standalone_capture.h"

#include <stdio.h>
#include <string.h>


DEBUG_GET_ONCE_LOG_OPTION(standalone_capture_log, "STANDALONE_CAPTURE_LOG", U_LOGGING_INFO)

#define CAP_TRACE(c, ...) U_LOG_IFL_T(c->log_level, __VA_ARGS__)
#define CAP_DEBUG(c, ...) U_LOG_IFL_D(c->log_level, __VA_ARGS__)
#define CAP_INFO(c, ...) U_LOG_IFL_I(c->log_level, __VA_ARGS__)
#define CAP_WARN(c, ...) U_LOG_IFL_W(c->log_level, __VA_ARGS__)
#define CAP_ERROR(c, ...) U_LOG_IFL_E(c->log_level, __VA_ARGS__)

/*!
 * Empty capture source.
 *
 * @implements xrt_fs
 * @implements xrt_frame_node
 */
struct capture_null
{
	struct xrt_fs base;
	struct xrt_frame_node node;

	enum u_logging_level log_level;

	//! Resolved configuration: rates are never 0 here.
	struct standalone_capture_config config;

	//! Where data goes and the contract checks on the way.
	struct standalone_capture_output output;

	bool tracking_running; //!< `slam_stream_start` was called.
	bool colour_running;   //!< `stream_start` was called.
};

static inline struct capture_null *
capture_null(struct xrt_fs *xfs)
{
	return (struct capture_null *)xfs;
}


/*
 *
 * Hardware side.
 *
 */

/*!
 * Open and start the sensors somebody consumes.
 */
static bool
start_tracking_sensors(struct capture_null *c)
{
	for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
		enum standalone_camera cam = (enum standalone_camera)i;
		if (!standalone_capture_output_wants_cam(&c->output, cam)) {
			continue;
		}

		//! @todo Open the host stream of this camera and start reading it on a thread.
		CAP_DEBUG(c, "Would start camera %s at %u fps", standalone_hw_camera_info(cam)->name,
		          c->config.cam_fps[i]);
	}

	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		enum standalone_imu imu = (enum standalone_imu)i;
		if (!standalone_capture_output_wants_imu(&c->output, imu)) {
			continue;
		}

		//! @todo Open this IMU and start reading it on a thread.
		CAP_DEBUG(c, "Would start IMU %s at %u Hz", standalone_hw_imu_info(imu)->name,
		          c->config.imu_rate_hz[i]);
	}

	/*
	 * @todo The read threads end in calls to the output stage:
	 *
	 *   standalone_capture_output_push_pair(&c->output, STANDALONE_CAM_FRONT_LEFT, left, right);
	 *   standalone_capture_output_push_imu(&c->output, STANDALONE_IMU_LEFT, &sample);
	 */

	return true;
}

/*!
 * Stop the sensors and join every thread that pushes. After this returns
 * nothing may be pushed.
 */
static void
stop_sensors(struct capture_null *c)
{
	//! @todo Stop and join the read threads, close the devices.
	(void)c;
}


/*
 *
 * xrt_fs functions.
 *
 */

static xrt_result_t
capture_null_enumerate_modes(struct xrt_fs *xfs, struct xrt_fs_mode **out_modes, uint32_t *out_count)
{
	// The modes are those of the colour camera, the only one started with a mode index.
	const struct standalone_camera_info *info = standalone_hw_camera_info(STANDALONE_CAM_COLOUR);

	struct xrt_fs_mode *modes = U_TYPED_ARRAY_CALLOC(struct xrt_fs_mode, 1);
	if (modes == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	modes[0].width = info->width;
	modes[0].height = info->height;
	modes[0].format = info->format;
	modes[0].stereo_format = XRT_STEREO_FORMAT_NONE;

	*out_modes = modes;
	*out_count = 1;

	return XRT_SUCCESS;
}

static xrt_result_t
capture_null_configure_capture(struct xrt_fs *xfs, struct xrt_fs_capture_parameters *cp)
{
	//! @todo Apply exposure and gain to the tracking cameras.
	return XRT_SUCCESS;
}

static xrt_result_t
capture_null_stream_start(struct xrt_fs *xfs,
                          struct xrt_frame_sink *xs,
                          enum xrt_fs_capture_type capture_type,
                          uint32_t descriptor_index)
{
	struct capture_null *c = capture_null(xfs);

	if (xs == NULL || descriptor_index != 0) {
		CAP_ERROR(c, "Colour stream needs a sink and mode 0 (sink %p, mode %u)", (void *)xs, descriptor_index);
		return XRT_ERROR_INVALID_ARGUMENT;
	}

	c->output.colour = xs;

	//! @todo Open and start the colour camera, push with STANDALONE_CAM_COLOUR.
	c->colour_running = true;

	CAP_INFO(c, "Colour stream started (no data: empty implementation)");

	return XRT_SUCCESS;
}

static xrt_result_t
capture_null_slam_stream_start(struct xrt_fs *xfs, struct xrt_slam_sinks *sinks)
{
	struct capture_null *c = capture_null(xfs);

	if (c->tracking_running) {
		CAP_ERROR(c, "Tracking streams already started");
		return XRT_ERROR_INVALID_ARGUMENT;
	}

	standalone_capture_output_set_sinks(&c->output, sinks);

	if (!start_tracking_sensors(c)) {
		standalone_capture_output_set_sinks(&c->output, NULL);
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	c->tracking_running = true;

	CAP_INFO(c, "Tracking streams started for cameras 0x%02x, IMUs 0x%x (no data: empty implementation)",
	         standalone_sinks_cam_mask(&c->output.sinks), standalone_sinks_imu_mask(&c->output.sinks));

	return XRT_SUCCESS;
}

static xrt_result_t
capture_null_stream_stop(struct xrt_fs *xfs)
{
	struct capture_null *c = capture_null(xfs);

	stop_sensors(c);

	c->tracking_running = false;
	c->colour_running = false;

	return XRT_SUCCESS;
}

static xrt_result_t
capture_null_is_running(struct xrt_fs *xfs, bool *out_is_running)
{
	struct capture_null *c = capture_null(xfs);

	*out_is_running = c->tracking_running || c->colour_running;

	return XRT_SUCCESS;
}


/*
 *
 * Frame node functions.
 *
 */

static void
capture_null_break_apart(struct xrt_frame_node *node)
{
	struct capture_null *c = container_of(node, struct capture_null, node);

	// Nothing may push once this returns.
	capture_null_stream_stop(&c->base);
}

static void
capture_null_destroy(struct xrt_frame_node *node)
{
	struct capture_null *c = container_of(node, struct capture_null, node);

	free(c);
}


/*
 *
 * Exported functions.
 *
 */

struct xrt_fs *
standalone_capture_null_create(struct xrt_frame_context *xfctx,
                               const struct standalone_capture_config *config,
                               struct standalone_capture_info *out_info)
{
	struct capture_null *c = U_TYPED_CALLOC(struct capture_null);
	if (c == NULL) {
		return NULL;
	}

	c->log_level = debug_get_log_option_standalone_capture_log();

	// Resolve the configuration against the hardware defaults.
	c->config = *config;
	c->config.impl = NULL;
	c->config.replay_path = NULL;
	for (uint32_t i = 0; i < STANDALONE_CAM_COUNT; i++) {
		if (c->config.cam_fps[i] == 0) {
			c->config.cam_fps[i] = standalone_hw_camera_info((enum standalone_camera)i)->nominal_fps;
		}
	}
	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		if (c->config.imu_rate_hz[i] == 0) {
			c->config.imu_rate_hz[i] = standalone_hw_imu_info((enum standalone_imu)i)->nominal_rate_hz;
		}
	}

	standalone_capture_output_set_sinks(&c->output, NULL);

	// Frame server.
	struct xrt_fs *xfs = &c->base;
	u_frameserver_populate_function_pointers(xfs);
	xfs->enumerate_modes = capture_null_enumerate_modes;
	xfs->configure_capture = capture_null_configure_capture;
	xfs->stream_start = capture_null_stream_start;
	xfs->slam_stream_start = capture_null_slam_stream_start;
	xfs->stream_stop = capture_null_stream_stop;
	xfs->is_running = capture_null_is_running;
	(void)snprintf(xfs->name, sizeof(xfs->name), "Standalone capture (null)");
	(void)snprintf(xfs->product, sizeof(xfs->product), "Standalone headset");
	(void)snprintf(xfs->manufacturer, sizeof(xfs->manufacturer), "UW Reality Labs");
	(void)snprintf(xfs->serial, sizeof(xfs->serial), "standalone-capture");
	memcpy(&xfs->source_id, "STNDCAP\0", sizeof(xfs->source_id));

	// Frame node, stops and frees us with the rest of the graph.
	c->node.break_apart = capture_null_break_apart;
	c->node.destroy = capture_null_destroy;
	xrt_frame_context_add(xfctx, &c->node);

	// All sensors of the headset, none of which will produce data.
	out_info->cam_mask = STANDALONE_CAMS_TRACKING_ALL | STANDALONE_CAM_BIT(STANDALONE_CAM_COLOUR);
	out_info->imu_mask = STANDALONE_IMUS_ALL;
	out_info->has_ground_truth = false;
	out_info->is_replay = false;

	return xfs;
}
