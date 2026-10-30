# Native mock Capture → Monado queue → CV prototype

**Local prototype:** new Capture + unchanged native Monado queue + new CV validation
receiver. The team's actual tracker receiver/Builder and full Monado runtime are
not integrated. RB5/EB5 and physical cameras remain untested.

The source factory uses the real pinned xrt_fs/xrt_frame/xrt_frame_sink interfaces.
Only the native CPU queue component is built from upstream Monado; no service,
compositor or OpenXR runtime is started. All Monado source stays unchanged.

## Combined native Capture-to-CV demonstration

The [end-to-end runner](../capture_e2e/README.md) now connects this source directly
to the tested native component graph. It selects explicit L8 output and can use
the original RGB converter before the shared public-frame CV observer.
The command below retains the earlier R8 harness and its narrower coverage.

## Run

From the enclosing repository root:

~~~sh
PYTHONPATH=tests/standalone-camera-mock .venv/bin/python -m capture_mock suite
~~~

This builds the component targets, runs three native checks, executes nine scenarios
and prints an HTML overview path. To run one case:

~~~sh
PYTHONPATH=tests/standalone-camera-mock .venv/bin/python -m capture_mock run --scenario four-camera
~~~

Optional output: --output /tmp/wrl-four-camera-new-run. Existing directories are refused.

Fresh compatible environment:

~~~sh
python3.13 -m venv .venv
.venv/bin/python -m pip install -r tests/standalone-camera-mock/capture_mock/requirements.lock
~~~

Also needs Git, a C/C++ compiler, pthreads and zlib headers/libraries. Tested on
Apple Silicon macOS with Clang 21/Python 3.13.14. Dependencies are pinned to the
versions installed here. The enclosing checkout supplies the upstream source. Reviewed public interfaces
and processing files must match ../reviewed-sources.json. Other worktree changes
are allowed and recorded in each manifest; the prototype exact-HEAD restriction
is no longer used.

Native builds use build/capture-mock-OS-ARCH. Superseded build caches were removed
by cleanup; the wrapper regenerates its own build when needed.

## Cases

All modes are explicit prototype fixtures, not physical AR0234 requirements.
The receiver accepts native grayscale R8; the old Bayer suggestion was not selected.

| Case | Expected outcome |
| --- | --- |
| one-camera | 150 frames, 640×480, target 30 FPS, valid image delivery and CV |
| four-camera | Four native producers, 150 frames each, distinct routes |
| group-four-camera | Same four images through one image-only xrt_slam_sinks bundle; camera routes remain distinct with a shared source ID |
| padded-rows | Four streams preserve 16 padding bytes per row |
| slow-consumer | Deliberate delay causes measured/accounted queue drops |
| interrupted-stream | Camera 2 stops at 15 frames; others finish 60 frames |
| invalid-format | One unsupported format is clearly rejected |
| invalid-size | One incorrect payload length is rejected before reading |
| invalid-stride | One incorrect stride is clearly rejected |

The unmodified queue drops the oldest queued frame on overflow. Normal cases expect
no unexplained losses. No team performance threshold is invented. Independent
pixel allocations plus bounded queues limit live storage to queue capacity and
producer/consumer/retained-reference overhead. No preallocated buffer pool is claimed.

## Results and clocks

Each run saves a pre-run manifest/config, frames.jsonl, summary.json, report.html
and input/annotated samples per camera. Logs include camera/session/sequence,
synthetic source/published time, measured frame-ready/arrival/CV-completion time,
layout, producer CRC, decoded pixel identity, image hash, corners or explicit error.

Measured stages share CLOCK_MONOTONIC on this host. Receive-to-CV includes FFI/
Python scheduling, validation, CV and injected consumer delay; it excludes
annotation/file encoding. Scheduled synthetic timestamps are not sensor latency.
Pixels/config/seed reproduce; execution timing and overload drop schedules vary.
Each run also preserves the exact authored source under source_snapshot/tests/standalone-camera-mock/capture_mock,
so uncommitted source changes do not make earlier recordings irreproducible.

## Tests

~~~sh
PYTHONPATH=tests/standalone-camera-mock .venv/bin/python -m unittest discover -s tests/standalone-camera-mock/tests -v
PYTHONPATH=tests/standalone-camera-mock .venv/bin/python -m capture_mock build
~~~

Native tests exercise the unchanged queue and public source lifecycle, including
retaining a frame beyond source destruction. The native bundle test verifies shuffled
camera IDs through four unchanged queues using only public frame fields, and rejects
missing/duplicate routes or unsupported IMU/pose/mask requests before starting. Python tests check independently
specified pixels, padding, four identities, errors, interruption, manual stop/
recreate, deterministic pixels and overwrite protection.

## Later Linux/RB5/EB5 — candidate, not tested here

Dockerfile is a component-test image, not the team's full graphics/runtime image.
The Linux container example remains untested. It needs the complete team
checkout including Git metadata. From the repository root:

~~~sh
docker build -t wrl-camera-mock -f tests/standalone-camera-mock/capture_mock/Dockerfile tests/standalone-camera-mock/capture_mock
docker run --rm --user "$(id -u):$(id -g)" --mount "type=bind,src=$PWD,dst=/workspace" \
  --workdir /workspace wrl-camera-mock python tests/standalone-camera-mock/run.py
~~~

No camera devices, privileged mode, host networking or display are needed by this
CPU-buffer prototype. Source is mounted at runtime; builds are platform-specific.
The Docker image and board package compatibility have not been executed/verified.
Record actual board OS/BSP/container separately. Local performance is not board performance.

## Current integration coverage

Both start modes preserve the original queue and CV path. The bundle adapter
sends configs[i] to the supplied cams[i] sink. The shared source ID identifies
the group; camera identity comes from the receiver route and is checked against
the image header. It is not inferred from that source ID.

For a direct native integration example, see native/capture_group_handoff.cpp.
It uses the public group factory, actual xrt_slam_sinks, unchanged queues and
ordinary native sinks. The Python harness adds a separate observation receiver
for CV/logging. No team Builder or tracker is inserted into that test.

The validation receiver currently expects the mock's original frame allocation
behind an unchanged queue, to read diagnostic readiness/CRC. It cannot be moved
after an arbitrary image converter as-is. Public factory users receive normal
xrt_frame data; the independent native handoff test reads no private frame owner.
An actual conversion stage needs its own declared image/metadata expectations.

## Remaining work

- The team Builder on dev exists but creates no camera sinks. Select its
  production receiver before adding a native Capture source-selection hook. Confirm format, memory,
  camera routing, timestamps and calibration needs. See INTERFACE.md.
- A test sink is not the production receiver. Missing receiver/adapter is an
  explicit integration gap; unchanged queue delivery alone is not full tracking.
- Gain/exposure controls are synthetic no-ops. Image-only bundle start is now
  supported via the existing xrt_fs_slam_stream_start API; this method name does
  not mean SLAM or IMU processing has been tested. IMU/pose/mask requests are
  rejected explicitly. Both individual and bundled camera routes are tested.
  Restart means stop/teardown/new session; in-place restart is unsupported.
- Verify board builds/runs later. Physical capture/CSI/ISP, exposure sync, IMU and
  full SLAM/hand/eye/OpenXR/display correctness require separate evidence.
- Reese review and owner-confirmed delivery dates remain unspecified.

No commits/pushes, Notion changes, team messages or board changes are performed.
