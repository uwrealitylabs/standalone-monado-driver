// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Capture implementation table and the shared output stage.
 * @ingroup drv_standalone
 */

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "standalone_capture.h"

#include <inttypes.h>
#include <string.h>


/*
 *
 * Implementation table.
 *
 */

struct capture_impl
{
	const char *name;
	standalone_capture_create_fn create;
};

static const struct capture_impl capture_impls[] = {
    {"null", standalone_capture_null_create},
#ifdef XRT_BUILD_DRIVER_EUROC
    {"euroc", standalone_capture_euroc_create},
#endif
};

static const struct capture_impl *
find_impl(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(capture_impls); i++) {
		if (strcmp(capture_impls[i].name, name) == 0) {
			return &capture_impls[i];
		}
	}
	return NULL;
}

bool
standalone_capture_has_impl(const char *name)
{
	return name != NULL && find_impl(name) != NULL;
}

struct xrt_fs *
standalone_capture_create(struct xrt_frame_context *xfctx,
                          const struct standalone_capture_config *config,
                          struct standalone_capture_info *out_info)
{
	const char *name = config->impl != NULL ? config->impl : "null";

	const struct capture_impl *impl = find_impl(name);
	if (impl == NULL) {
		U_LOG_E("No Standalone capture implementation named '%s' in this build", name);
		return NULL;
	}

	U_ZERO(out_info);

	struct xrt_fs *xfs = impl->create(xfctx, config, out_info);
	if (xfs == NULL) {
		U_LOG_E("Failed to create Standalone capture implementation '%s'", name);
		return NULL;
	}

	U_LOG_I("Standalone capture: '%s' (cameras 0x%03x, IMUs 0x%x%s)", name, out_info->cam_mask, out_info->imu_mask,
	        out_info->is_replay ? ", replay" : "");

	return xfs;
}


/*
 *
 * Output stage.
 *
 */

void
standalone_capture_output_set_sinks(struct standalone_capture_output *out, const struct xrt_slam_sinks *sinks)
{
	struct xrt_frame_sink *colour = out->colour;

	U_ZERO(out);
	out->colour = colour;

	if (sinks != NULL) {
		out->sinks = *sinks;
	}

	for (uint32_t i = 0; i < STANDALONE_CAM_COUNT; i++) {
		out->last_frame_ts[i] = INT64_MIN;
	}
	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		out->last_imu_ts[i] = INT64_MIN;
	}
}

static struct xrt_frame_sink *
get_cam_sink(const struct standalone_capture_output *out, enum standalone_camera cam)
{
	if (cam == STANDALONE_CAM_COLOUR) {
		return out->colour;
	}
	if ((uint32_t)cam < STANDALONE_CAM_TRACKING_COUNT) {
		return out->sinks.cams[cam];
	}
	return NULL;
}

bool
standalone_capture_output_wants_cam(const struct standalone_capture_output *out, enum standalone_camera cam)
{
	return get_cam_sink(out, cam) != NULL;
}

bool
standalone_capture_output_wants_imu(const struct standalone_capture_output *out, enum standalone_imu imu)
{
	return (uint32_t)imu < STANDALONE_IMU_COUNT && out->sinks.imus[imu] != NULL;
}

//! Contract violations are logged this many times, then only counted.
#define MAX_VIOLATION_LOGS (16)

/*!
 * Count a data contract violation. Returns true if it should also be logged.
 */
static bool
count_violation(uint64_t *counter)
{
	return ++(*counter) <= MAX_VIOLATION_LOGS;
}

/*!
 * Check one frame against the data contract. Returns false if it must be
 * dropped.
 */
static bool
check_frame(struct standalone_capture_output *out, enum standalone_camera cam, const struct xrt_frame *xf)
{
	const struct standalone_camera_info *info = standalone_hw_camera_info(cam);

	if (xf->format != info->format) {
		if (count_violation(&out->cam_violations[cam])) {
			U_LOG_W("Standalone capture: %s frame has format %d, the contract is %d; dropped", info->name,
			        (int)xf->format, (int)info->format);
		}
		return false;
	}

	if (xf->timestamp <= out->last_frame_ts[cam]) {
		if (count_violation(&out->cam_violations[cam])) {
			U_LOG_W("Standalone capture: %s frame timestamp %" PRId64 " does not increase (last %" PRId64
			        "); dropped",
			        info->name, xf->timestamp, out->last_frame_ts[cam]);
		}
		return false;
	}

	return true;
}

void
standalone_capture_output_push_frame(struct standalone_capture_output *out,
                                     enum standalone_camera cam,
                                     struct xrt_frame *xf)
{
	struct xrt_frame_sink *sink = get_cam_sink(out, cam);
	if (sink == NULL) {
		return;
	}

	if (!check_frame(out, cam, xf)) {
		return;
	}

	out->last_frame_ts[cam] = xf->timestamp;
	out->frame_count[cam]++;

	xrt_sink_push_frame(sink, xf);
}

void
standalone_capture_output_push_pair(struct standalone_capture_output *out,
                                    enum standalone_camera left_cam,
                                    struct xrt_frame *left,
                                    struct xrt_frame *right)
{
	const struct standalone_camera_info *info = standalone_hw_camera_info(left_cam);
	enum standalone_camera right_cam = info->pair;

	if (left->timestamp != right->timestamp) {
		if (count_violation(&out->cam_violations[left_cam])) {
			U_LOG_W("Standalone capture: %s pair timestamps differ (%" PRId64 " and %" PRId64 "); dropped",
			        info->name, left->timestamp, right->timestamp);
		}
		return;
	}

	// Both or neither: a consumer of a pair never sees half of it.
	if (get_cam_sink(out, left_cam) != NULL && !check_frame(out, left_cam, left)) {
		return;
	}
	if (get_cam_sink(out, right_cam) != NULL && !check_frame(out, right_cam, right)) {
		return;
	}

	standalone_capture_output_push_frame(out, left_cam, left);
	standalone_capture_output_push_frame(out, right_cam, right);
}

void
standalone_capture_output_push_imu(struct standalone_capture_output *out,
                                   enum standalone_imu imu,
                                   struct xrt_imu_sample *sample)
{
	if (!standalone_capture_output_wants_imu(out, imu)) {
		return;
	}

	if (sample->timestamp_ns <= out->last_imu_ts[imu]) {
		if (count_violation(&out->imu_violations[imu])) {
			U_LOG_W("Standalone capture: %s sample timestamp %" PRId64 " does not increase (last %" PRId64
			        "); dropped",
			        standalone_hw_imu_info(imu)->name, sample->timestamp_ns, out->last_imu_ts[imu]);
		}
		return;
	}

	out->last_imu_ts[imu] = sample->timestamp_ns;
	out->imu_count[imu]++;

	xrt_sink_push_imu(out->sinks.imus[imu], sample);
}
