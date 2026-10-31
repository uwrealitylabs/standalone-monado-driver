# End-to-end native mock Capture demonstration

**Local prototype:** native mock Capture → native L8 image buffers → public-frame
input guard → unchanged optional L8-to-RGB converter → unchanged bounded queues →
optional unchanged stereo pairing → shared public-frame CV observer → OpenCV
corners, annotated images and matching source/receiver logs.

This joins the two previously separate experiments. Python does not submit the
image buffers: the native Capture factory generates and schedules every frame.
The CV observer reads public frame fields, including after conversion allocates
a different native frame. It never interprets a mock-private owner pointer.

The team production receiver/Builder, full SLAM/hand tracking, Qualcomm capture
stack and RB5/EB5 are not exercised.

## One command

From the enclosing team repository root:

~~~sh
.venv/bin/python tests/standalone-camera-mock/run.py
~~~

This builds the local components, runs five native checks, the 31 original Python
tests plus five integration checks, and the twelve declared cases in cases.json. It prints a new HTML overview path. To run only the
normal four-camera RGB route:

~~~sh
.venv/bin/python tests/standalone-camera-mock/run.py --scenario four-rgb
~~~

Use --output with a fresh destination if needed. Existing output folders are
refused. Dependencies are capture_mock/requirements.lock, a C/C++ compiler, pthreads,
zlib and the enclosing Monado checkout. Reviewed dependency hashes are checked
against ../reviewed-sources.json; authored test changes are recorded separately.
Builds are platform-specific under build/capture-e2e-OS-ARCH.

## What each case checks

- one-l8, four-l8, four-rgb: 640×480, target 30 FPS, 150 frames per camera.
  Direct L8 and native RGB conversion must preserve full pixels and image identity.
- padded-rgb: explicit 16-byte input padding, four sources and native conversion.
- interrupted-l8: camera 2 ends after 15 frames while three others finish 60.
- invalid-format-rgb, invalid-size-l8, invalid-stride-rgb: one malformed frame is
  rejected clearly at the public input guard before an unchecked native converter
  can read it. Subsequent valid frames continue.
- slow-rgb: queue capacity two and 35 ms receiver delay deliberately cause drops.
  The logs must account for the missing sequences and retained images remain valid.
- manual-stop-rgb and restart-l8: stop all producers, cleanly release the graph,
  then create a new session whose pixel header and logs identify session 2.
  In-place source restart is not claimed.
- stereo-rgb: explicit two-camera pairing only. It checks left/right delivery
  through the existing pairer; it is not a four-camera synchronization result.

These are fixture settings, not confirmed physical camera requirements.

## Native wiring

New source selection is in native/session.cpp:

1. Build the existing component graph with matrix_create.
2. Create an L8 Capture group with wrl_mock_capture_group_create_format.
3. Supply its explicit camera sinks using xrt_fs_slam_stream_start. The API name
   does not imply a SLAM tracker: the bundle contains images only.
4. The source-output guard checks actual public format/layout before handing the
   same frame to matrix_accept_frame. No Python image copy is injected into the
   graph. The guard observes metadata/CRC and reports errors.
5. The original native converter and queue process those frames. Optional pairing
   is selected explicitly, never silently added.
6. Stop/join producers, snapshot source statistics, destroy the source graph,
   drain/stop processing and check retained pixels before final release.

Sources and processing use separate native frame contexts. Retained original
frames can outlive Capture destruction through native references. The source
statistics are snapshots before processing teardown; source_live_frames_before_teardown
is deliberately not reported as a final leak count. The processing wrapper's
final live count and native sanitizer checks cover release of outstanding references.

## Formats and backwards compatibility

The old R8 factories and their FFI config remain unchanged in behavior.
New explicit-format factories accept R8 or L8. Both describe this generator's
actual grayscale pixels; no physical Bayer/RAW data is relabelled.

The normal end-to-end source is L8. The RGB route uses the original upstream
converter to replicate grayscale into RGB channels. Expected pixels are calculated
independently using the existing Python fixture and compared in full after receipt.

## Logs and timing

Every run writes its manifest/config and source snapshots before native source
startup. Logs include camera/session/sequence, native layout, source identity,
synthetic source/published timestamps and measured stage times. Annotated filenames
are linked to the specific receiver log record that saved them.

source-frames.jsonl records the first public observation of completed Capture
output, its original CRC and guard errors. This is not the mock's private internal
ready timestamp. frames.jsonl records receipt and CV completion from the shared
observer. The scheduled monotonic epoch is saved before starting producers.

Measured stages use CLOCK_MONOTONIC on the same host. Receive-to-CV includes
FFI/GIL scheduling, independent full-image verification, grayscale conversion,
OpenCV and any deliberate receiver delay; it excludes annotation saving.
Observed FPS measures local processing delivery, not physical sensor throughput.
Synthetic source/exposure timestamps do not measure sensor latency.

The same per-camera identity/source timestamp must survive conversion. Pairing
may average published times; the unmodified source times remain separate.

## Evidence and reproduction review

The report provides counts, gaps/duplicates, median/p95/max processing time,
observed processing rate, error messages, native cleanup results and samples.
Manifests include source/input configuration hashes, dependency versions,
upstream revision, native library hashes and exact run commands.

Another member can reproduce the command after this contribution is shared, inspect annotated frames and match their
camera/sequence/session to the two logs. That independent review remains pending;
no team message, Notion update, Git commit/push or board change is performed.
The diagnostic logging shim formats component messages only; upstream image
processing and queue/pairing code remain unchanged.
