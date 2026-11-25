// Copyright 2026, UW Reality Labs.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Tests of the Standalone driver's fixed parts: the hardware table,
 *        the sink map helpers, the capture output stage and the pipeline
 *        with its empty implementations.
 *
 * None of this needs hardware, a compositor or a tracker library. Run it
 * after changing an interface or adding an implementation.
 */

#include "catch_amalgamated.hpp"

#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"
#include "xrt/xrt_tracking.h"

#include "standalone/standalone_pipeline.h"

#include <vector>


namespace {

/*!
 * A frame sink that records what it was given.
 */
struct RecordingFrameSink
{
	struct standalone_frame_slot slot = {};
	std::vector<int64_t> timestamps;
	//! Shared between sinks to record the order of pushes across them.
	std::vector<RecordingFrameSink *> *order = nullptr;
	//! The sink to hand out.
	struct xrt_frame_sink &base = slot.sink;

	RecordingFrameSink()
	{
		slot.owner = this;
		slot.sink.push_frame = [](struct xrt_frame_sink *sink, struct xrt_frame *xf) {
			auto *self = static_cast<RecordingFrameSink *>(standalone_frame_slot(sink)->owner);
			self->timestamps.push_back(xf->timestamp);
			if (self->order != nullptr) {
				self->order->push_back(self);
			}
		};
	}
};

/*!
 * An IMU sink that records what it was given.
 */
struct RecordingImuSink
{
	struct standalone_imu_slot slot = {};
	std::vector<int64_t> timestamps;
	//! The sink to hand out.
	struct xrt_imu_sink &base = slot.sink;

	RecordingImuSink()
	{
		slot.owner = this;
		slot.sink.push_imu = [](struct xrt_imu_sink *sink, struct xrt_imu_sample *sample) {
			auto *self = static_cast<RecordingImuSink *>(standalone_imu_slot(sink)->owner);
			self->timestamps.push_back(sample->timestamp_ns);
		};
	}
};

//! A frame as a source would push it. Nothing in these tests keeps a reference.
struct xrt_frame
make_frame(enum standalone_camera cam, int64_t timestamp_ns)
{
	const struct standalone_camera_info *info = standalone_hw_camera_info(cam);

	struct xrt_frame xf = {};
	xf.width = info->width;
	xf.height = info->height;
	xf.format = info->format;
	xf.timestamp = timestamp_ns;
	return xf;
}

struct xrt_imu_sample
make_imu_sample(int64_t timestamp_ns)
{
	struct xrt_imu_sample sample = {};
	sample.timestamp_ns = timestamp_ns;
	sample.accel_m_s2.y = 9.81;
	return sample;
}

struct standalone_pipeline_config
null_config()
{
	// Explicit names, so the environment of the test run does not matter.
	struct standalone_pipeline_config config = {};
	config.capture_impl = "null";
	config.slam_impl = "null";
	config.hand_impl = "null";
	config.eye_impl = "null";
	return config;
}

} // namespace


TEST_CASE("standalone_hw")
{
	// The tracking cameras fill the sink map exactly.
	CHECK(STANDALONE_CAM_TRACKING_COUNT == XRT_TRACKING_MAX_CAMS);
	CHECK(STANDALONE_CAM_COUNT == 9);
	CHECK(STANDALONE_IMU_COUNT <= XRT_TRACKING_MAX_IMUS);

	for (uint32_t i = 0; i < STANDALONE_CAM_COUNT; i++) {
		auto cam = static_cast<enum standalone_camera>(i);
		const struct standalone_camera_info *info = standalone_hw_camera_info(cam);

		CAPTURE(info->name);
		CHECK(info->width > 0);
		CHECK(info->height > 0);
		CHECK(info->nominal_fps > 0);
		CHECK(info->nominal_fps <= info->max_fps);

		// Pairs are symmetric and adjacent, left first.
		CHECK(standalone_hw_camera_info(info->pair)->pair == cam);
		if (info->pair != cam) {
			CHECK((i / 2) == (static_cast<uint32_t>(info->pair) / 2));
			CHECK(standalone_hw_camera_is_left(cam) != standalone_hw_camera_is_left(info->pair));
			CHECK(standalone_hw_camera_info(info->pair)->width == info->width);
			CHECK(standalone_hw_camera_info(info->pair)->height == info->height);
		}

		if (i < STANDALONE_CAM_TRACKING_COUNT) {
			CHECK(info->mono);
			CHECK(info->global_shutter);
			CHECK(info->format == XRT_FORMAT_L8);
		}
	}

	// The pair composites carry the left camera on the left.
	const auto *eye_l = standalone_hw_camera_info(STANDALONE_CAM_EYE_LEFT);
	const auto *eye_r = standalone_hw_camera_info(STANDALONE_CAM_EYE_RIGHT);
	CHECK(eye_l->stream == STANDALONE_STREAM_EYE_PAIR);
	CHECK(eye_r->stream == STANDALONE_STREAM_EYE_PAIR);
	CHECK(eye_l->stream_x_offset == 0);
	CHECK(eye_r->stream_x_offset == eye_l->width);

	// The default consumers do not overlap with the eye cameras, and SLAM shares the front pair with hands.
	CHECK((STANDALONE_CAMS_HAND_DEFAULT & STANDALONE_CAMS_EYE_DEFAULT) == 0);
	CHECK((STANDALONE_CAMS_SLAM_DEFAULT & STANDALONE_CAMS_HAND_DEFAULT) == STANDALONE_CAMS_FRONT);
	CHECK((STANDALONE_CAMS_SLAM_DEFAULT | STANDALONE_CAMS_HAND_DEFAULT | STANDALONE_CAMS_EYE_DEFAULT) ==
	      STANDALONE_CAMS_TRACKING_ALL);
}

TEST_CASE("standalone_calibration")
{
	struct standalone_calibration calib;
	standalone_calibration_fill_nominal(&calib);

	CHECK(calib.is_nominal);
	CHECK(calib.cams[STANDALONE_CAM_EYE_LEFT].intrinsics.image_size_pixels.w == 1280);
	CHECK(calib.cams[STANDALONE_CAM_FRONT_LEFT].intrinsics.image_size_pixels.h == 1200);
	CHECK(calib.cams[STANDALONE_CAM_FRONT_LEFT].head_from_camera.position.x < 0.0f);
	CHECK(calib.cams[STANDALONE_CAM_FRONT_RIGHT].head_from_camera.position.x > 0.0f);
	CHECK(calib.imus[STANDALONE_IMU_LEFT].rate_hz == STANDALONE_IMU_NOMINAL_RATE_HZ);

	// No file: fine. A file: cannot be honoured yet.
	CHECK(standalone_calibration_load(nullptr, &calib));
	CHECK_FALSE(standalone_calibration_load("does-not-exist.json", &calib));
	CHECK(calib.is_nominal);
}

TEST_CASE("standalone_sinks_merge")
{
	struct xrt_frame_context xfctx = {};

	// Three consumers shaped like the default trackers.
	RecordingFrameSink slam_cams[2];
	RecordingImuSink slam_imus[2];
	RecordingFrameSink hand_cams[6];
	RecordingFrameSink eye_cams[2];

	struct xrt_slam_sinks slam = {};
	slam.cams[STANDALONE_CAM_FRONT_LEFT] = &slam_cams[0].base;
	slam.cams[STANDALONE_CAM_FRONT_RIGHT] = &slam_cams[1].base;
	slam.imus[STANDALONE_IMU_LEFT] = &slam_imus[0].base;
	slam.imus[STANDALONE_IMU_RIGHT] = &slam_imus[1].base;

	struct xrt_slam_sinks hand = {};
	for (uint32_t i = 0; i < 6; i++) {
		hand.cams[i] = &hand_cams[i].base;
	}

	struct xrt_slam_sinks eye = {};
	eye.cams[STANDALONE_CAM_EYE_LEFT] = &eye_cams[0].base;
	eye.cams[STANDALONE_CAM_EYE_RIGHT] = &eye_cams[1].base;

	SECTION("all consumers")
	{
		const struct xrt_slam_sinks *consumers[] = {&slam, &hand, &eye};
		struct xrt_slam_sinks entry = {};
		standalone_sinks_merge(&xfctx, consumers, 3, &entry);

		CHECK(standalone_sinks_cam_mask(&entry) == STANDALONE_CAMS_TRACKING_ALL);
		CHECK(standalone_sinks_imu_mask(&entry) == STANDALONE_IMUS_ALL);
		CHECK(entry.cam_count == 8);
		CHECK(entry.imu_count == 2);

		// One consumer: connected directly. Two: through a splitter.
		CHECK(entry.cams[STANDALONE_CAM_SIDE_LEFT] == hand.cams[STANDALONE_CAM_SIDE_LEFT]);
		CHECK(entry.cams[STANDALONE_CAM_EYE_RIGHT] == eye.cams[STANDALONE_CAM_EYE_RIGHT]);
		CHECK(entry.cams[STANDALONE_CAM_FRONT_LEFT] != slam.cams[STANDALONE_CAM_FRONT_LEFT]);
		CHECK(entry.cams[STANDALONE_CAM_FRONT_LEFT] != hand.cams[STANDALONE_CAM_FRONT_LEFT]);
		CHECK(entry.imus[STANDALONE_IMU_LEFT] == slam.imus[STANDALONE_IMU_LEFT]);

		// Every frame reaches every consumer of its camera, once.
		for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
			struct xrt_frame xf = make_frame(static_cast<enum standalone_camera>(i), 1000 + i);
			xrt_sink_push_frame(entry.cams[i], &xf);
		}

		CHECK(slam_cams[0].timestamps == std::vector<int64_t>{1000});
		CHECK(slam_cams[1].timestamps == std::vector<int64_t>{1001});
		for (uint32_t i = 0; i < 6; i++) {
			CHECK(hand_cams[i].timestamps == std::vector<int64_t>{1000 + i});
		}
		CHECK(eye_cams[0].timestamps == std::vector<int64_t>{1006});
		CHECK(eye_cams[1].timestamps == std::vector<int64_t>{1007});
	}

	SECTION("a missing consumer leaves slots empty")
	{
		const struct xrt_slam_sinks *consumers[] = {&slam, nullptr, &eye};
		struct xrt_slam_sinks entry = {};
		standalone_sinks_merge(&xfctx, consumers, 3, &entry);

		CHECK(standalone_sinks_cam_mask(&entry) == (STANDALONE_CAMS_FRONT | STANDALONE_CAMS_EYE));
		CHECK(entry.cams[STANDALONE_CAM_FRONT_LEFT] == slam.cams[STANDALONE_CAM_FRONT_LEFT]);
		CHECK(entry.cams[STANDALONE_CAM_SIDE_LEFT] == nullptr);
		CHECK(entry.cams[STANDALONE_CAM_BOTTOM_RIGHT] == nullptr);
	}

	SECTION("two consumers of one IMU both get its samples")
	{
		RecordingImuSink other_imu;
		struct xrt_slam_sinks other = {};
		other.imus[STANDALONE_IMU_LEFT] = &other_imu.base;

		const struct xrt_slam_sinks *consumers[] = {&slam, &other};
		struct xrt_slam_sinks entry = {};
		standalone_sinks_merge(&xfctx, consumers, 2, &entry);

		struct xrt_imu_sample sample = make_imu_sample(42);
		xrt_sink_push_imu(entry.imus[STANDALONE_IMU_LEFT], &sample);

		CHECK(slam_imus[0].timestamps == std::vector<int64_t>{42});
		CHECK(other_imu.timestamps == std::vector<int64_t>{42});
		CHECK(slam_imus[1].timestamps.empty());
	}

	xrt_frame_context_destroy_nodes(&xfctx);
}

TEST_CASE("standalone_capture_output")
{
	std::vector<RecordingFrameSink *> order;
	RecordingFrameSink left;
	RecordingFrameSink right;
	RecordingImuSink imu;
	left.order = &order;
	right.order = &order;

	struct xrt_slam_sinks sinks = {};
	sinks.cams[STANDALONE_CAM_FRONT_LEFT] = &left.base;
	sinks.cams[STANDALONE_CAM_FRONT_RIGHT] = &right.base;
	sinks.imus[STANDALONE_IMU_LEFT] = &imu.base;

	struct standalone_capture_output out = {};
	standalone_capture_output_set_sinks(&out, &sinks);

	SECTION("only consumed sensors are wanted")
	{
		CHECK(standalone_capture_output_wants_cam(&out, STANDALONE_CAM_FRONT_LEFT));
		CHECK_FALSE(standalone_capture_output_wants_cam(&out, STANDALONE_CAM_EYE_LEFT));
		CHECK_FALSE(standalone_capture_output_wants_cam(&out, STANDALONE_CAM_COLOUR));
		CHECK(standalone_capture_output_wants_imu(&out, STANDALONE_IMU_LEFT));
		CHECK_FALSE(standalone_capture_output_wants_imu(&out, STANDALONE_IMU_RIGHT));

		// Pushing an unconsumed sensor is a no-op, not a crash.
		struct xrt_frame xf = make_frame(STANDALONE_CAM_EYE_LEFT, 10);
		standalone_capture_output_push_frame(&out, STANDALONE_CAM_EYE_LEFT, &xf);
		CHECK(out.frame_count[STANDALONE_CAM_EYE_LEFT] == 0);
	}

	SECTION("a pair goes out left first")
	{
		struct xrt_frame l = make_frame(STANDALONE_CAM_FRONT_LEFT, 100);
		struct xrt_frame r = make_frame(STANDALONE_CAM_FRONT_RIGHT, 100);
		standalone_capture_output_push_pair(&out, STANDALONE_CAM_FRONT_LEFT, &l, &r);

		REQUIRE(order.size() == 2);
		CHECK(order[0] == &left);
		CHECK(order[1] == &right);
	}

	SECTION("a pair with different timestamps is dropped whole")
	{
		struct xrt_frame l = make_frame(STANDALONE_CAM_FRONT_LEFT, 100);
		struct xrt_frame r = make_frame(STANDALONE_CAM_FRONT_RIGHT, 101);
		standalone_capture_output_push_pair(&out, STANDALONE_CAM_FRONT_LEFT, &l, &r);

		CHECK(order.empty());
		CHECK(out.cam_violations[STANDALONE_CAM_FRONT_LEFT] == 1);
	}

	SECTION("timestamps must increase")
	{
		struct xrt_frame a = make_frame(STANDALONE_CAM_FRONT_LEFT, 100);
		struct xrt_frame b = make_frame(STANDALONE_CAM_FRONT_LEFT, 100);
		struct xrt_frame c = make_frame(STANDALONE_CAM_FRONT_LEFT, 101);
		standalone_capture_output_push_frame(&out, STANDALONE_CAM_FRONT_LEFT, &a);
		standalone_capture_output_push_frame(&out, STANDALONE_CAM_FRONT_LEFT, &b);
		standalone_capture_output_push_frame(&out, STANDALONE_CAM_FRONT_LEFT, &c);

		CHECK(left.timestamps == std::vector<int64_t>{100, 101});
		CHECK(out.cam_violations[STANDALONE_CAM_FRONT_LEFT] == 1);

		struct xrt_imu_sample s1 = make_imu_sample(50);
		struct xrt_imu_sample s2 = make_imu_sample(40);
		standalone_capture_output_push_imu(&out, STANDALONE_IMU_LEFT, &s1);
		standalone_capture_output_push_imu(&out, STANDALONE_IMU_LEFT, &s2);

		CHECK(imu.timestamps == std::vector<int64_t>{50});
		CHECK(out.imu_violations[STANDALONE_IMU_LEFT] == 1);
	}

	SECTION("the pixel format is part of the contract")
	{
		struct xrt_frame xf = make_frame(STANDALONE_CAM_FRONT_LEFT, 100);
		xf.format = XRT_FORMAT_YUYV422;
		standalone_capture_output_push_frame(&out, STANDALONE_CAM_FRONT_LEFT, &xf);

		CHECK(left.timestamps.empty());
		CHECK(out.cam_violations[STANDALONE_CAM_FRONT_LEFT] == 1);
	}
}

TEST_CASE("standalone trackers respect what the source delivers")
{
	struct xrt_frame_context xfctx = {};

	struct standalone_calibration calib;
	standalone_calibration_fill_nominal(&calib);

	// A two-camera, one-IMU dataset.
	const uint32_t cams = STANDALONE_CAMS_FRONT;
	const uint32_t imus = STANDALONE_IMU_BIT(STANDALONE_IMU_LEFT);

	struct standalone_slam_config slam_config = {};
	slam_config.calib = &calib;
	slam_config.available_cams = cams;
	slam_config.available_imus = imus;
	struct standalone_slam_tracker *slam = nullptr;
	REQUIRE(standalone_slam_create(&xfctx, &slam_config, &slam) == XRT_SUCCESS);
	CHECK(standalone_sinks_cam_mask(&slam->sinks) == STANDALONE_CAMS_FRONT);
	CHECK(standalone_sinks_imu_mask(&slam->sinks) == imus);

	struct standalone_hand_config hand_config = {};
	hand_config.calib = &calib;
	hand_config.available_cams = cams;
	struct standalone_hand_tracker *hand = nullptr;
	REQUIRE(standalone_hand_create(&xfctx, &hand_config, &hand) == XRT_SUCCESS);
	CHECK(standalone_sinks_cam_mask(&hand->sinks) == STANDALONE_CAMS_FRONT);

	struct standalone_eye_config eye_config = {};
	eye_config.calib = &calib;
	eye_config.available_cams = cams;
	struct standalone_eye_tracker *eye = nullptr;
	REQUIRE(standalone_eye_create(&xfctx, &eye_config, &eye) == XRT_SUCCESS);
	CHECK(standalone_sinks_cam_mask(&eye->sinks) == 0);

	xrt_frame_context_destroy_nodes(&xfctx);
}

TEST_CASE("standalone_pipeline with the empty implementations")
{
	struct xrt_frame_context xfctx = {};

	struct standalone_pipeline_config config = null_config();
	struct standalone_pipeline *sp = nullptr;
	REQUIRE(standalone_pipeline_create(&xfctx, &config, &sp) == XRT_SUCCESS);
	REQUIRE(sp != nullptr);

	REQUIRE(sp->source != nullptr);
	REQUIRE(sp->slam != nullptr);
	REQUIRE(sp->hand != nullptr);
	REQUIRE(sp->eye != nullptr);

	SECTION("every tracking sensor is wired")
	{
		CHECK(standalone_sinks_cam_mask(&sp->entry_sinks) == STANDALONE_CAMS_TRACKING_ALL);
		CHECK(standalone_sinks_imu_mask(&sp->entry_sinks) == STANDALONE_IMUS_ALL);

		CHECK(standalone_sinks_cam_mask(&sp->slam->sinks) == STANDALONE_CAMS_SLAM_DEFAULT);
		CHECK(standalone_sinks_cam_mask(&sp->hand->sinks) == STANDALONE_CAMS_HAND_DEFAULT);
		CHECK(standalone_sinks_cam_mask(&sp->eye->sinks) == STANDALONE_CAMS_EYE_DEFAULT);

		CHECK_FALSE(sp->source_info.is_replay);
		CHECK((sp->source_info.cam_mask & STANDALONE_CAMS_TRACKING_ALL) == STANDALONE_CAMS_TRACKING_ALL);
	}

	SECTION("the source is an xrt_fs and is started")
	{
		bool running = false;
		CHECK(xrt_fs_is_running(sp->source, &running) == XRT_SUCCESS);
		CHECK(running);

		// The modes are those of the colour camera.
		struct xrt_fs_mode *modes = nullptr;
		uint32_t mode_count = 0;
		REQUIRE(xrt_fs_enumerate_modes(sp->source, &modes, &mode_count) == XRT_SUCCESS);
		REQUIRE(mode_count == 1);
		CHECK(modes[0].width == standalone_hw_camera_info(STANDALONE_CAM_COLOUR)->width);
		free(modes);

		struct xrt_fs_capture_parameters cp = {};
		CHECK(xrt_fs_configure_capture(sp->source, &cp) == XRT_SUCCESS);

		// The colour camera can be started on its own.
		RecordingFrameSink colour;
		CHECK(xrt_fs_stream_start(sp->source, &colour.base, XRT_FS_CAPTURE_TYPE_TRACKING, 0) == XRT_SUCCESS);

		CHECK(xrt_fs_stream_stop(sp->source) == XRT_SUCCESS);
		CHECK(xrt_fs_is_running(sp->source, &running) == XRT_SUCCESS);
		CHECK_FALSE(running);
	}

	SECTION("data pushed by a source reaches the trackers without anything breaking")
	{
		int64_t ts = 1000000;
		for (int n = 0; n < 10; n++, ts += 1000000) {
			struct xrt_imu_sample sample = make_imu_sample(ts);
			xrt_sink_push_imu(sp->entry_sinks.imus[STANDALONE_IMU_LEFT], &sample);
			xrt_sink_push_imu(sp->entry_sinks.imus[STANDALONE_IMU_RIGHT], &sample);

			for (uint32_t i = 0; i < STANDALONE_CAM_TRACKING_COUNT; i++) {
				struct xrt_frame xf = make_frame(static_cast<enum standalone_camera>(i), ts);
				xrt_sink_push_frame(sp->entry_sinks.cams[i], &xf);
			}
		}
	}

	SECTION("queries answer immediately")
	{
		struct xrt_space_relation head = {};
		standalone_slam_tracker_get_head_pose(sp->slam, 123456789, &head);
		CHECK((head.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0);
		CHECK(head.pose.orientation.w == 1.0f);
		CHECK(sp->slam->tracking_type == XRT_TRACKING_TYPE_NONE);

		struct xrt_hand_joint_set joints = {};
		joints.is_active = true;
		int64_t joints_ts = 0;
		standalone_hand_tracker_get_hand(sp->hand, XRT_HAND_LEFT, 123456789, &joints, &joints_ts);
		CHECK_FALSE(joints.is_active);
		CHECK(joints_ts == 123456789);

		struct standalone_gaze gaze = {};
		standalone_eye_tracker_get_gaze(sp->eye, 123456789, &gaze);
		CHECK(gaze.combined.relation_flags == XRT_SPACE_RELATION_BITMASK_NONE);
		CHECK(gaze.timestamp_ns == 123456789);
	}

	// Stops the source, then frees everything, the pipeline included.
	xrt_frame_context_destroy_nodes(&xfctx);
}

TEST_CASE("standalone_pipeline with an unknown implementation")
{
	struct xrt_frame_context xfctx = {};

	struct standalone_pipeline_config config = null_config();
	struct standalone_pipeline *sp = nullptr;

	SECTION("capture")
	{
		config.capture_impl = "does-not-exist";
	}
	SECTION("slam")
	{
		config.slam_impl = "does-not-exist";
	}
	SECTION("hand")
	{
		config.hand_impl = "does-not-exist";
	}
	SECTION("eye")
	{
		config.eye_impl = "does-not-exist";
	}

	CHECK(standalone_pipeline_create(&xfctx, &config, &sp) != XRT_SUCCESS);
	CHECK(sp == nullptr);

	CHECK(standalone_capture_has_impl("null"));
	CHECK(standalone_slam_has_impl("null"));
	CHECK(standalone_hand_has_impl("null"));
	CHECK(standalone_eye_has_impl("null"));
	CHECK_FALSE(standalone_slam_has_impl("does-not-exist"));

	// Whatever was created before the failure is torn down here.
	xrt_frame_context_destroy_nodes(&xfctx);
}
