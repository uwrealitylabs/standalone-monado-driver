// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Helpers for the headset sink map.
 * @ingroup drv_standalone
 */

#include "util/u_misc.h"
#include "util/u_sink.h"

#include "standalone_sinks.h"

#include <assert.h>


void
standalone_sinks_init(struct standalone_frame_slot *cam_slots,
                      struct standalone_imu_slot *imu_slots,
                      void *owner,
                      uint32_t cam_mask,
                      uint32_t imu_mask,
                      void (*push_frame)(struct xrt_frame_sink *, struct xrt_frame *),
                      void (*push_imu)(struct xrt_imu_sink *, struct xrt_imu_sample *),
                      struct xrt_slam_sinks *out_sinks)
{
	U_ZERO(out_sinks);

	for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
		if ((cam_mask & STANDALONE_CAM_BIT(i)) == 0) {
			continue;
		}
		assert(push_frame != NULL);
		cam_slots[i].sink.push_frame = push_frame;
		cam_slots[i].owner = owner;
		cam_slots[i].cam = (enum standalone_camera)i;
		out_sinks->cams[i] = &cam_slots[i].sink;
		out_sinks->cam_count = (int)i + 1;
	}

	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		if ((imu_mask & STANDALONE_IMU_BIT(i)) == 0) {
			continue;
		}
		assert(push_imu != NULL && imu_slots != NULL);
		imu_slots[i].sink.push_imu = push_imu;
		imu_slots[i].owner = owner;
		imu_slots[i].imu = (enum standalone_imu)i;
		out_sinks->imus[i] = &imu_slots[i].sink;
		out_sinks->imu_count = (int)i + 1;
	}
}

void
standalone_sinks_merge(struct xrt_frame_context *xfctx,
                       const struct xrt_slam_sinks *const *consumers,
                       size_t consumer_count,
                       struct xrt_slam_sinks *out_entry)
{
	U_ZERO(out_entry);

	// Cameras.
	for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
		struct xrt_frame_sink *downstreams[U_SINK_MAX_SPLIT_DOWNSTREAMS] = {0};
		size_t count = 0;

		for (size_t c = 0; c < consumer_count; c++) {
			if (consumers[c] == NULL || consumers[c]->cams[i] == NULL) {
				continue;
			}
			assert(count < ARRAY_SIZE(downstreams));
			downstreams[count++] = consumers[c]->cams[i];
		}

		if (count == 0) {
			continue;
		}

		if (count == 1) {
			out_entry->cams[i] = downstreams[0];
		} else {
			u_sink_split_multi_create(xfctx, downstreams, count, &out_entry->cams[i]);
		}
		out_entry->cam_count = (int)i + 1;
	}

	// IMUs, the splitter takes two downstreams so chain them.
	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		struct xrt_imu_sink *merged = NULL;

		for (size_t c = 0; c < consumer_count; c++) {
			if (consumers[c] == NULL || consumers[c]->imus[i] == NULL) {
				continue;
			}
			if (merged == NULL) {
				merged = consumers[c]->imus[i];
			} else {
				u_imu_sink_split_create(xfctx, merged, consumers[c]->imus[i], &merged);
			}
		}

		if (merged == NULL) {
			continue;
		}

		out_entry->imus[i] = merged;
		out_entry->imu_count = (int)i + 1;
	}

	// Ground truth, only meaningful for replay.
	for (size_t c = 0; c < consumer_count; c++) {
		if (consumers[c] != NULL && consumers[c]->gt != NULL) {
			out_entry->gt = consumers[c]->gt;
			break;
		}
	}
}

void
standalone_sinks_for_dataset(const struct xrt_slam_sinks *entry,
                             uint32_t first_cam,
                             uint32_t cam_count,
                             bool with_motion,
                             struct xrt_slam_sinks *out_dataset)
{
	U_ZERO(out_dataset);

	for (uint32_t n = 0; n < cam_count && n < XRT_TRACKING_MAX_CAMS; n++) {
		const uint32_t slot = first_cam + n;
		if (slot >= STANDALONE_CAM_TRACKING_COUNT) {
			break;
		}
		out_dataset->cams[n] = entry->cams[slot];
	}
	out_dataset->cam_count = (int)cam_count;

	if (with_motion) {
		out_dataset->imus[0] = entry->imus[STANDALONE_IMU_LEFT];
		out_dataset->imu_count = 1;
		out_dataset->gt = entry->gt;
	}
}

uint32_t
standalone_sinks_cam_mask(const struct xrt_slam_sinks *sinks)
{
	uint32_t mask = 0;
	for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
		if (sinks->cams[i] != NULL) {
			mask |= STANDALONE_CAM_BIT(i);
		}
	}
	return mask;
}

uint32_t
standalone_sinks_imu_mask(const struct xrt_slam_sinks *sinks)
{
	uint32_t mask = 0;
	for (uint32_t i = 0; i < STANDALONE_IMU_COUNT; i++) {
		if (sinks->imus[i] != NULL) {
			mask |= STANDALONE_IMU_BIT(i);
		}
	}
	return mask;
}
