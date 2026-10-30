# Implemented native contract and pending production adapter

Prototype source: upstream 04521d3e99402193e3a4d35fc4bf16ee27fe1d21.
The team fork uses the same reviewed frame interfaces; ../reviewed-sources.json
checks their current bytes before each build/run.
This is the tested local ABI, not an agreed team-specific receiver implementation.

## Factory

native/mock_capture.h exports:

~~~c
struct xrt_fs *wrl_mock_capture_create(
    struct xrt_frame_context *context, const struct wrl_config *config);
~~~

Enumerate mode zero, pass an existing image sink to xrt_fs_stream_start, then stop
and destroy the frame graph. Invalid mode, null sink and double start fail.
Gain/exposure are best-effort synthetic no-ops. Multi-sensor SLAM start returns
XRT_ERROR_NOT_IMPLEMENTED. Stop is interruptible/idempotent. Recreate for a new session.

The factory and validation harness are separate interfaces. Only the latter
creates a new observation receiver and uses ctypes/OpenCV. A real Builder can
use the factory with its existing sink once its source-selection hook is identified.

## Image-only multi-camera factory

~~~c
struct xrt_fs *wrl_mock_capture_group_create(
    struct xrt_frame_context *context,
    const struct wrl_config *configs, uint32_t count);
~~~

The group accepts one to four valid configs with unique camera IDs. The caller
fills the actual pinned xrt_slam_sinks with cam_count=count and distinct, non-null
cams[i] pointers, then calls xrt_fs_slam_stream_start. Configs[i] routes to cams[i];
camera IDs need not equal array indices. One configured camera per route is
required by this prototype; disabled/null camera routes are not supported.

The group uses source_id 0x57524C47 for all frames, so source identity must not
be used as a camera index. It schedules each camera from one shared synthetic
epoch and preserves per-camera sequence/route identity. Equal synthetic times do
not establish physical exposure synchronization.

Despite the API name, this is image-only: nonzero imu_count, non-null IMU sinks,
ground-truth pose sinks or mask sinks return XRT_ERROR_NOT_IMPLEMENTED before
starting. Wrong camera count, null/duplicate camera routes and repeated starts
return XRT_ERROR_INVALID_ARGUMENT. Multi-camera stream_start into one image sink
and group enumerate_modes are unsupported; per-camera layouts are explicit in
the configs. Stop joins every producer; the frame context owns the group and
children. Recreate to restart. No real Builder selection hook is changed.

The group factory's native integration example and test are in
native/capture_group_handoff.cpp. The harness uses the same adapter start method,
then preserves its original per-camera queues and observation/CV receiver.

## Explicit grayscale formats and observation

The original create/create_group factories remain R8 defaults. New
wrl_mock_capture_create_format and wrl_mock_capture_group_create_format accept an
explicit native R8 or L8 enum; other formats fail. These are generated grayscale
bytes, never relabelled physical Bayer data. Mode enumeration on the individual
factory reports the selected format.

For a group returned by this module, wrl_mock_capture_group_schedule sets a
nonzero scheduled epoch before start. wrl_mock_capture_group_stats snapshots the
indexed producer's counters while the source remains alive. These two methods
require this module's group object, not an arbitrary xrt_fs. Source-only
received/processed/retained fields are unused; the pipeline observer owns those
measurements. The combined runner destroys the source before final pipeline
drain and validates retained native image references afterward.

See [the combined demonstration](../capture_e2e/README.md).

## Frames

Actual xrt_frame, one grayscale byte/pixel. Defaults use XRT_FORMAT_R8;
explicit-format factories can select XRT_FORMAT_L8. Stride is width plus
configured padding; allocation covers all rows. Padding is 165. Pixel header
encodes camera/sequence/seed/session; checkerboard and moving rectangle provide CV
features. Receiver checks image identity and full producer/receiver CRC.

Each individually started image source uses source_id = 0x57524C00 + camera index. Four streams are
four native producers/routes, not aliases. source_sequence is the frame index.
Session ID belongs to the fixture/harness, not an invented xrt field.

source_timestamp is simulated nanoseconds from session zero. timestamp adds that
offset to a shared scheduled monotonic epoch. Neither is a measured exposure.
Measured readiness/arrival/CV completion are separate CLOCK_MONOTONIC values.

## Ownership and completion

Producer holds the initial reference. The real queue retains while queued or
consumed. A consumer keeping the native frame must retain it. Final release frees
the independent pixel storage. Shared diagnostic counters survive source deletion,
so a retained frame can outlive the source safely.

The FFI view/data pointers are borrowed during the callback only. Python copies
the bytes before returning and retains only its copy. A separate native test
retains the first frame and verifies unchanged contents after later frames and
source destruction.

Monado push returns void. The validation callback result is instrumentation after
CV; it is not an acknowledgement added to Monado's API. Cleanup stops producers,
joins/clears the unchanged queue and releases observation references. Source stop
alone does not imply that queued processing completed.

## Remaining adapter

The team Builder is standalone_open_system_impl in target_builder_standalone.c.
It currently creates an HMD but no camera receiver or source-selection hook.
Its production receiver factory, accepted mode/memory and clock mapping remain
unimplemented/unconfirmed. This first contribution leaves that Builder unchanged.
Connect a later explicit source option only after its receiver is defined; keep
all downstream code and requirements unchanged. The native
factory is a concrete boundary, not completed Builder or tracking integration.
