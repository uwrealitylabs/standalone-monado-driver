// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Capture of the Standalone headset: the source of all sensor data.
 *
 * A capture implementation is an @ref xrt_fs. That is the whole interface;
 * nothing downstream knows which implementation is running, which is what
 * makes live capture, dataset replay and synthetic data interchangeable.
 *
 * ## What an implementation provides
 *
 * | `xrt_fs` member     | Meaning for this headset |
 * |---------------------|--------------------------|
 * | `slam_stream_start` | Start the tracking sensors. The argument is the headset sink map (see standalone_sinks.h):
 * eight camera slots, two IMU slots, ground truth. Push only into non-NULL slots. | | `stream_start`      | Start the
 * colour camera into one frame sink. Independent of `slam_stream_start`; either may be called without the other. | |
 * `enumerate_modes`   | Modes of the colour camera (the `descriptor_index` of `stream_start`). | | `configure_capture`
 * | Exposure and gain. Must not fail because a parameter is unsupported. | | `stream_stop`       | Stop everything,
 * join every thread that pushes. Called from `break_apart`. | | `is_running`        | Whether any stream is started. |
 *
 * An implementation also is an @ref xrt_frame_node added to the frame
 * context in its create function; that is how it is stopped and freed.
 *
 * What it pushes must follow the data contract in standalone_sinks.h. The
 * output stage in this file (@ref standalone_capture_output) checks the
 * parts of that contract that can be checked at run time; implementations
 * push through it rather than into the sinks directly.
 *
 * ## Adding an implementation
 *
 * Write `capture/standalone_capture_<name>.c` with a create function of type
 * @ref standalone_capture_create_fn, declare it below, add it to the table
 * in standalone_capture.c and to the driver's sources in CMake. Select it
 * with `STANDALONE_CAPTURE_IMPL=<name>`.
 *
 * @ingroup drv_standalone
 */

#pragma once

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"
#include "xrt/xrt_tracking.h"

#include "standalone_hw.h"
#include "standalone_sinks.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Configuration of a capture source.
 *
 * @ingroup drv_standalone
 */
struct standalone_capture_config
{
	//! Name of the implementation, NULL for "null".
	const char *impl;

	//! Dataset for replaying implementations, NULL otherwise.
	const char *replay_path;

	//! Headset slot that camera 0 of the replayed dataset goes to, see @ref standalone_camera.
	uint32_t replay_first_cam;

	//! Frame rate per camera, 0 for the camera's nominal rate.
	uint32_t cam_fps[STANDALONE_CAM_COUNT];

	//! Output data rate per IMU, 0 for the IMU's nominal rate.
	uint32_t imu_rate_hz[STANDALONE_IMU_COUNT];
};

/*!
 * What a created source will deliver. Trackers are told this so they do not
 * wait for sensors that never produce data, which matters for replay: a
 * two-camera dataset has no side, bottom or eye cameras.
 *
 * @ingroup drv_standalone
 */
struct standalone_capture_info
{
	uint32_t cam_mask;     //!< Cameras that can deliver frames, see @ref STANDALONE_CAM_BIT.
	uint32_t imu_mask;     //!< IMUs that can deliver samples, see @ref STANDALONE_IMU_BIT.
	bool has_ground_truth; //!< Pushes ground truth poses into the `gt` slot.
	bool is_replay;        //!< Data is replayed or synthetic, not from the headset's sensors.
};

/*!
 * Create function of a capture implementation.
 *
 * The source must be added to @p xfctx. It must not push anything before one
 * of its start functions is called.
 *
 * @param      xfctx    Frame context that will own the source.
 * @param      config   Configuration, only valid during the call.
 * @param[out] out_info What the source will deliver.
 * @return The source, or NULL on failure.
 *
 * @ingroup drv_standalone
 */
typedef struct xrt_fs *(*standalone_capture_create_fn)(struct xrt_frame_context *xfctx,
                                                       const struct standalone_capture_config *config,
                                                       struct standalone_capture_info *out_info);

/*!
 * Create the capture source named by `config->impl`.
 *
 * @copydetails standalone_capture_create_fn
 * @ingroup drv_standalone
 */
struct xrt_fs *
standalone_capture_create(struct xrt_frame_context *xfctx,
                          const struct standalone_capture_config *config,
                          struct standalone_capture_info *out_info);

/*!
 * Is there a capture implementation with this name in this build?
 *
 * @ingroup drv_standalone
 */
bool
standalone_capture_has_impl(const char *name);


/*
 *
 * Implementations.
 *
 */

/*!
 * "null": the empty implementation. Reports every sensor, starts and stops,
 * and pushes nothing. The starting point for a real implementation.
 *
 * @ingroup drv_standalone
 */
struct xrt_fs *
standalone_capture_null_create(struct xrt_frame_context *xfctx,
                               const struct standalone_capture_config *config,
                               struct standalone_capture_info *out_info);

#ifdef XRT_BUILD_DRIVER_EUROC
/*!
 * "euroc": replays a dataset in EuRoC format with Monado's EuRoC player.
 *
 * Dataset camera `n` goes to slot `cams[replay_first_cam + n]`. With a first
 * camera of 0 the dataset is taken to be of the headset's motion: its IMU
 * goes to `imus[0]` and its ground truth to `gt`. With any other first
 * camera (a hand or eye dataset) its IMU and ground truth are not used; the
 * player still needs an `imu0/data.csv` to exist.
 *
 * @ingroup drv_standalone
 */
struct xrt_fs *
standalone_capture_euroc_create(struct xrt_frame_context *xfctx,
                                const struct standalone_capture_config *config,
                                struct standalone_capture_info *out_info);
#endif


/*
 *
 * Output stage, shared by implementations.
 *
 */

/*!
 * The output side of a capture implementation: holds the sinks it was
 * started with and checks the data contract on the way out. Not thread safe
 * per sensor: push one sensor's data from one thread at a time. Different
 * sensors may be pushed from different threads.
 *
 * @ingroup drv_standalone
 */
struct standalone_capture_output
{
	struct xrt_slam_sinks sinks;   //!< Tracking sinks, set by `slam_stream_start`.
	struct xrt_frame_sink *colour; //!< Colour camera sink, set by `stream_start`.

	int64_t last_frame_ts[STANDALONE_CAM_COUNT]; //!< Last timestamp pushed per camera.
	int64_t last_imu_ts[STANDALONE_IMU_COUNT];   //!< Last timestamp pushed per IMU.

	uint64_t frame_count[STANDALONE_CAM_COUNT]; //!< Frames pushed per camera.
	uint64_t imu_count[STANDALONE_IMU_COUNT];   //!< Samples pushed per IMU.

	uint64_t cam_violations[STANDALONE_CAM_COUNT]; //!< Frames dropped for breaking the data contract.
	uint64_t imu_violations[STANDALONE_IMU_COUNT]; //!< Samples dropped for breaking the data contract.
};

/*!
 * Reset the output stage and set the tracking sinks. NULL clears them.
 *
 * @ingroup drv_standalone
 */
void
standalone_capture_output_set_sinks(struct standalone_capture_output *out, const struct xrt_slam_sinks *sinks);

/*!
 * Does anybody consume this camera? If not, do not capture it.
 *
 * @ingroup drv_standalone
 */
bool
standalone_capture_output_wants_cam(const struct standalone_capture_output *out, enum standalone_camera cam);

/*!
 * Does anybody consume this IMU? If not, do not read it.
 *
 * @ingroup drv_standalone
 */
bool
standalone_capture_output_wants_imu(const struct standalone_capture_output *out, enum standalone_imu imu);

/*!
 * Push one frame of one camera downstream. Frames that break the contract
 * (wrong format, timestamp not increasing) are dropped and counted. Does not
 * take ownership: the caller keeps and releases its reference.
 *
 * @ingroup drv_standalone
 */
void
standalone_capture_output_push_frame(struct standalone_capture_output *out,
                                     enum standalone_camera cam,
                                     struct xrt_frame *xf);

/*!
 * Push both frames of a stereo pair, left first. Both are dropped if their
 * timestamps differ.
 *
 * @ingroup drv_standalone
 */
void
standalone_capture_output_push_pair(struct standalone_capture_output *out,
                                    enum standalone_camera left_cam,
                                    struct xrt_frame *left,
                                    struct xrt_frame *right);

/*!
 * Push one IMU sample downstream. Samples whose timestamp does not increase
 * are dropped and counted.
 *
 * @ingroup drv_standalone
 */
void
standalone_capture_output_push_imu(struct standalone_capture_output *out,
                                   enum standalone_imu imu,
                                   struct xrt_imu_sample *sample);

#ifdef __cplusplus
}
#endif
