// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The headset sink map and the data contract at the boundary between
 *         capture and trackers, plus small helpers for implementing sinks.
 *
 * ## The headset sink map
 *
 * One @ref xrt_slam_sinks describes where every tracking sensor of the
 * headset delivers its data. It is what the capture source is started with
 * (@ref xrt_fs::slam_stream_start), and it is also the type each tracker uses
 * to expose its inputs. Slots are indexed by @ref standalone_camera and
 * @ref standalone_imu:
 *
 * | Slot        | Sensor                 | Default consumers |
 * |-------------|------------------------|-------------------|
 * | `cams[0,1]` | Front left, right      | SLAM, hands       |
 * | `cams[2,3]` | Side left, right       | Hands             |
 * | `cams[4,5]` | Bottom left, right     | Hands             |
 * | `cams[6,7]` | Eye left, right        | Eyes              |
 * | `imus[0,1]` | IMU left, right        | SLAM              |
 * | `gt`        | Ground truth (replay)  | SLAM              |
 * | `masks[]`   | Not filled by capture  | Hands push into SLAM's |
 *
 * A NULL slot means nobody consumes that sensor. A source must not push into
 * a NULL slot, and should not spend power capturing that sensor at all.
 *
 * ## Data contract
 *
 * Every source (live capture, replay, synthetic) delivers, and every tracker
 * may rely on, the following:
 *
 * - **Frames.** One @ref xrt_frame per camera per exposure. Tracking cameras
 *   are @ref XRT_FORMAT_L8. The size is the one recorded in the calibration
 *   for that camera (native size unless capture is configured to scale).
 * - **Timestamps.** `xrt_frame::timestamp` and `xrt_imu_sample::timestamp_ns`
 *   are in the monotonic clock (`os_monotonic_get_ns`), frames at the middle
 *   of the exposure. Frames exposed by the same trigger edge carry the same
 *   timestamp, exactly. Timestamps of one sensor strictly increase.
 *   `xrt_frame::source_sequence` is the trigger count and identifies a frame
 *   set across cameras.
 * - **Order.** Within a stereo pair the left frame is pushed before the
 *   right frame. A source never pushes a pair partially: if one frame of a
 *   pair was lost, the other is dropped.
 * - **IMU.** SI units, in the IMU's own axes, one sample per push, at a
 *   steady rate. An IMU sample is pushed before any frame with a later
 *   timestamp whenever the source can guarantee it.
 * - **Threads.** A push is a plain function call on the source's thread. A
 *   tracker that needs more than a few hundred microseconds per push must
 *   queue the data and return (see @ref u_sink_queue_create), or it stalls
 *   capture for every other consumer.
 * - **Lifetime.** A sink that keeps a frame past the push call takes its own
 *   reference with @ref xrt_frame_reference.
 *
 * @ingroup drv_standalone
 */

#pragma once

#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "standalone_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * A frame sink that knows which camera it is for and who owns it. Lets one
 * receive function serve every camera:
 *
 * ```c
 * static void
 * receive_frame(struct xrt_frame_sink *sink, struct xrt_frame *xf)
 * {
 * 	struct standalone_frame_slot *slot = standalone_frame_slot(sink);
 * 	struct my_tracker *t = (struct my_tracker *)slot->owner;
 * 	// slot->cam says which camera xf came from.
 * }
 * ```
 *
 * @ingroup drv_standalone
 */
struct standalone_frame_slot
{
	struct xrt_frame_sink sink; //!< Must be first.
	void *owner;
	enum standalone_camera cam;
};

/*!
 * An IMU sink that knows which IMU it is for and who owns it.
 *
 * @ingroup drv_standalone
 */
struct standalone_imu_slot
{
	struct xrt_imu_sink sink; //!< Must be first.
	void *owner;
	enum standalone_imu imu;
};

static inline struct standalone_frame_slot *
standalone_frame_slot(struct xrt_frame_sink *sink)
{
	return (struct standalone_frame_slot *)sink;
}

static inline struct standalone_imu_slot *
standalone_imu_slot(struct xrt_imu_sink *sink)
{
	return (struct standalone_imu_slot *)sink;
}

/*!
 * Set up one slot per camera in @p cam_mask and one per IMU in @p imu_mask,
 * and point the matching entries of @p out_sinks at them. Slots outside the
 * masks are left untouched and their entries in @p out_sinks stay NULL.
 *
 * @param[out] cam_slots Array of STANDALONE_CAM_TRACKING_COUNT slots.
 * @param[out] imu_slots Array of STANDALONE_IMU_COUNT slots, may be NULL if @p imu_mask is 0.
 * @param      owner     Stored in every slot.
 * @param      cam_mask  Cameras to create sinks for, see @ref STANDALONE_CAM_BIT.
 * @param      imu_mask  IMUs to create sinks for, see @ref STANDALONE_IMU_BIT.
 * @param      push_frame Receive function for frames.
 * @param      push_imu   Receive function for IMU samples, may be NULL if @p imu_mask is 0.
 * @param[out] out_sinks Zeroed, then filled in.
 *
 * @ingroup drv_standalone
 */
void
standalone_sinks_init(struct standalone_frame_slot *cam_slots,
                      struct standalone_imu_slot *imu_slots,
                      void *owner,
                      uint32_t cam_mask,
                      uint32_t imu_mask,
                      void (*push_frame)(struct xrt_frame_sink *, struct xrt_frame *),
                      void (*push_imu)(struct xrt_imu_sink *, struct xrt_imu_sample *),
                      struct xrt_slam_sinks *out_sinks);

/*!
 * Combine the sinks of several consumers into the one set of sinks a source
 * is started with. For each slot: no consumer leaves it NULL, one consumer is
 * connected directly, several consumers get a splitter node that pushes to
 * each in the order given.
 *
 * Ground truth is taken from the first consumer that has a `gt` sink. Mask
 * sinks are not merged, capture does not produce masks.
 *
 * @param      xfctx          Frame context the splitter nodes are added to.
 * @param      consumers      Sinks of each consumer, entries may be NULL.
 * @param      consumer_count Number of entries in @p consumers.
 * @param[out] out_entry      The merged sinks.
 *
 * @ingroup drv_standalone
 */
void
standalone_sinks_merge(struct xrt_frame_context *xfctx,
                       const struct xrt_slam_sinks *const *consumers,
                       size_t consumer_count,
                       struct xrt_slam_sinks *out_entry);

/*!
 * Which camera slots of @p sinks are filled in, as a mask.
 *
 * @ingroup drv_standalone
 */
uint32_t
standalone_sinks_cam_mask(const struct xrt_slam_sinks *sinks);

/*!
 * Which IMU slots of @p sinks are filled in, as a mask.
 *
 * @ingroup drv_standalone
 */
uint32_t
standalone_sinks_imu_mask(const struct xrt_slam_sinks *sinks);

#ifdef __cplusplus
}
#endif
