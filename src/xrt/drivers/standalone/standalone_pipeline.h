// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The tracking pipeline of the Standalone headset: one capture
 *         source, three trackers, and the wiring between them.
 *
 * ```
 *                 xrt_fs                headset sink map
 *  capture source ──────► cams[0,1] ─┬─► SLAM tracker ──► get_head_pose
 *  (live, replay)                    └─► hand tracker ──► get_hand
 *                         cams[2..5] ───► hand tracker
 *                         cams[6,7] ────► eye tracker  ──► get_gaze
 *                         imus[0,1] ────► SLAM tracker
 *                         gt ───────────► SLAM tracker
 *        hand tracker ── device masks ──► SLAM tracker
 * ```
 *
 * Which slots each tracker takes is decided by the tracker (see
 * standalone_sinks.h); the pipeline connects whatever they expose and adds a
 * splitter where two trackers take the same sensor. The colour camera has no
 * consumer and is not started.
 *
 * The pipeline does not create devices and does not need the rest of the
 * runtime: a test or a benchmark can create it on a bare frame context.
 *
 * ## Options
 *
 * | Environment variable           | Meaning | Default |
 * |--------------------------------|---------|---------|
 * | `STANDALONE_EUROC_PATH`        | EuRoC dataset to replay | none: live capture |
 * | `STANDALONE_EUROC_FIRST_CAM`   | Headset slot of the dataset's camera 0: 0 front, 2 side, 4 bottom, 6 eye | 0 |
 * | `STANDALONE_CAPTURE_IMPL`      | Capture implementation  | `euroc` if a dataset is given, else `null` |
 * | `STANDALONE_SLAM_IMPL`         | SLAM tracker implementation | `t_slam` if it is built and a dataset with the front
 * pair and IMU is replayed, else `null` | | `STANDALONE_HAND_IMPL`         | Hand tracker implementation | `null` | |
 * `STANDALONE_EYE_IMPL`          | Eye tracker implementation  | `null` | | `STANDALONE_CALIBRATION_PATH`  | Headset
 * calibration file    | none: nominal values |
 *
 * A dataset replayed from slot 0 is taken to be of the headset's motion and
 * also feeds the reference IMU. A dataset replayed from another slot (hand or
 * eye cameras) only feeds cameras.
 *
 * @ingroup drv_standalone
 */

#pragma once

#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"
#include "xrt/xrt_results.h"

#include "standalone_calibration.h"
#include "standalone_capture.h"
#include "standalone_eye.h"
#include "standalone_hand.h"
#include "standalone_slam.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Configuration of the pipeline. A NULL implementation name means the
 * default described in the file documentation.
 *
 * @ingroup drv_standalone
 */
struct standalone_pipeline_config
{
	const char *capture_impl;
	const char *slam_impl;
	const char *hand_impl;
	const char *eye_impl;

	//! EuRoC dataset to replay, NULL for none.
	const char *euroc_path;

	//! Headset slot that camera 0 of the replayed dataset goes to, see @ref standalone_camera.
	uint32_t euroc_first_cam;

	//! Headset calibration file, NULL for none.
	const char *calibration_path;
};

/*!
 * A created and running pipeline. Owned by the frame context it was created
 * on: every pointer here stays valid until that context destroys its nodes,
 * which the devices holding them must not outlive.
 *
 * @ingroup drv_standalone
 */
struct standalone_pipeline
{
	//! Calibration handed to the trackers; devices use it too.
	struct standalone_calibration calib;

	//! The capture source.
	struct xrt_fs *source;

	//! What the source delivers.
	struct standalone_capture_info source_info;

	struct standalone_slam_tracker *slam;
	struct standalone_hand_tracker *hand;
	struct standalone_eye_tracker *eye;

	//! The headset sink map the source was started with.
	struct xrt_slam_sinks entry_sinks;

	//! Frees this struct with the rest of the graph.
	struct xrt_frame_node node;
};

/*!
 * Fill in the configuration from the environment variables listed in the
 * file documentation.
 *
 * @ingroup drv_standalone
 */
void
standalone_pipeline_fill_default_config(struct standalone_pipeline_config *config);

/*!
 * Create the source and the trackers, connect them, and start the source.
 *
 * On failure nothing is returned; whatever was already created stays on
 * @p xfctx and is torn down with it.
 *
 * @param      xfctx  Frame context that will own everything created.
 * @param      config Configuration, only valid during the call.
 * @param[out] out_sp The pipeline.
 *
 * @ingroup drv_standalone
 */
xrt_result_t
standalone_pipeline_create(struct xrt_frame_context *xfctx,
                           const struct standalone_pipeline_config *config,
                           struct standalone_pipeline **out_sp);

#ifdef __cplusplus
}
#endif
