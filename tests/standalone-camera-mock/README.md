# Standalone mock camera → Monado components → CV

This optional development test generates repeatable images in native mock Capture,
passes them through the enclosing repository's unmodified image components, and
runs OpenCV corner detection on the received pixels. It does not start the HMD
Builder, a Monado service, compositor, tracker, or OpenXR application.

```text
Native mock camera 0 ─ guard ─ [RGB converter] ─ queue 0 ─ public-frame CV
Native mock camera 1 ─ guard ─ [RGB converter] ─ queue 1 ─ public-frame CV
Native mock camera 2 ─ guard ─ [RGB converter] ─ queue 2 ─ public-frame CV
Native mock camera 3 ─ guard ─ [RGB converter] ─ queue 3 ─ public-frame CV
                                                            ↓
                                           frame logs + annotated samples
```

Normal cases use a synthetic 640×480, 30 FPS fixture, with 150 images per camera.
These are development settings, not confirmed physical sensor requirements.
The default four-camera paths have no stereo pairing. The separate stereo case
uses two cameras and does not demonstrate four-camera exposure synchronization.

## Set up and run

From the **repository root**, using Python 3.13, Git, a C/C++ compiler, pthreads
and zlib development headers/libraries:

```sh
python3.13 -m venv .venv
.venv/bin/python -m pip install -r tests/standalone-camera-mock/capture_mock/requirements.lock
.venv/bin/python tests/standalone-camera-mock/run.py
```

The final command builds the isolated native targets, runs five native checks,
the 31 carried-over Python tests plus five port-specific checks, and all twelve
end-to-end scenarios. It prints the HTML report and validation record paths.
Build errors and unexpected test outcomes return a nonzero exit status.
The test is independent of the normal runtime build and does not require
`XRT_BUILD_DRIVER_STANDALONE=ON`. That option still enables the team's HMD driver
when building the runtime separately.

For a focused four-camera demonstration:

```sh
.venv/bin/python tests/standalone-camera-mock/run.py --scenario four-l8
.venv/bin/python tests/standalone-camera-mock/run.py --scenario four-rgb
```

Focused runs execute the five native checks and the selected demonstration;
they do not rerun the complete Python suite. Add `--output /absolute/fresh/path`
to choose a destination. Existing output directories are always refused.
The command resolves paths from its own location, not the current directory.

The dependency lock is retained from the validated prototype. The integration
is validated on Apple Silicon macOS with Python 3.13; Linux/container and RB5/EB5
execution require their own recorded results. No camera or network access is
needed for the test itself after checkout and dependency installation.

## What to inspect

Generated files stay under `results/validation/<run>/` by default and are ignored
by Git. The last printed `validation.json` records test totals and pass/fail state.
Its `report` points to the suite overview, or the focused case report.

Each case contains:

- `config.json` and a pre-start `manifest.json`: effective settings, seed,
  dependency versions, Git revision/dirty state, commands and source/library hashes.
- `source-frames.jsonl`: completed source-output observations and input errors.
- `frames.jsonl`: receiver/CV observations, camera/session/sequence, layout,
  timestamps, image hash, detected corners and sample filename.
- `summary.json`: per-stream counts, gaps, duplicates, measured timing,
  cleanup checks and historical pixel-baseline comparison.
- `camera-N-annotated.png`: the first processed frame from camera N, with corners
  drawn. Match its filename to `sample_file` in the receiver log, then match
  camera ID and sequence to the source log. Not every frame is saved as a PNG.

The suite shares one verified source snapshot through per-case relative links;
retain the complete suite directory when sharing the generated report.
Normal-case pixels are checked against 1,350 observations from the pre-port run
identified in `fixtures/normal-pixels.json`. Timings and overload drop sequences
are measured afresh and are not expected to match old execution times.

## Source protection and modules

`reviewed-sources.json` records the reviewed hashes of public/config headers,
queue, converter, pairer, frame/format helpers and their included utility/OS
headers. Changed or missing protected files stop the run with a clear error.
Authored test changes and later commits are allowed and recorded; a clean
worktree is not falsely claimed. Compatibility changes require an explicit
review before updating the checked-in baseline.

The source and sink contracts are in [capture_mock/INTERFACE.md](capture_mock/INTERFACE.md).
The three packages remain separate active dependencies:

- `capture_mock`: native `xrt_fs` factories, original R8 harness and fixtures.
- `pipeline_matrix`: component graph and public-frame CV observer.
- `capture_e2e`: L8 source startup, input guard and end-to-end orchestration.

The isolated CMake targets compile processing files directly from this checkout.
`component_log.c` supplies only diagnostic log transport in those test libraries;
it is never linked into runtime targets. No HMD/Builder or upstream processing
source is patched, and no separate/nested Monado checkout is used.

## Meaning of the results

The tests cover identity, full pixels, layout, metadata, concurrency of logical
streams, buffering, format rejection, interruption, stop/recreate, retained-buffer
validity and cleanup. Simulated source time remains separate from measured
`CLOCK_MONOTONIC` source-output, arrival and CV-completion observations.
Receive-to-CV includes scheduling/FFI, validation, conversion to grayscale and CV;
it excludes annotated-image saving. It is not sensor latency.

The current receiver is a diagnostic sink. Production receiver/Builder wiring,
SLAM, hand tracking, Qualcomm capture, board execution, physical sensor support,
exposure synchronization and IMU alignment remain untested.
The next integration step is an explicitly selected receiving pipeline and
Builder lifecycle; this change does not create another substitute tracker.

## Validated results and review

The October 28, 2026 local integration run passed **36 Python checks** (31 original
plus five integration checks), **five native checks**, and **all twelve
end-to-end scenarios**, with no skipped Python tests. Both focused launcher
commands passed; the L8 command also passed from outside the repository.

| Normal path | Generated | Received / CV processed | Drops / duplicates |
| --- | ---: | ---: | ---: |
| One-camera L8 | 150 | 150 | 0 / 0 |
| Four-camera L8 | 600 | 600 | 0 / 0 |
| Four-camera RGB | 600 | 600 | 0 / 0 |

All **1,350 normal-frame pixel hashes matched** the pre-port baseline.
Per-camera median receive-to-CV times were **3.97–3.99 ms** for L8 and
**4.52–4.58 ms** for RGB on the tested laptop. Malformed format/size/stride cases
each rejected one frame and processed the remaining 19. The interrupted-stream
case processed all 195 generated frames. Deliberate overload processed 60 of
240 frames and accounted for 180 queue drops. Stop/recreate, stereo ordering,
retained buffers and cleanup passed.

- [Compact measured results and preservation checks](review/results.json).
- [Camera/session/sequence matches for annotated samples](review/sample-frame-records.json).
- Annotated samples: [camera 0](review/samples/camera-0-annotated.png),
  [camera 1](review/samples/camera-1-annotated.png),
  [camera 2](review/samples/camera-2-annotated.png),
  [camera 3](review/samples/camera-3-annotated.png).

The native source/header files, public-frame CV observer and original Python test
files remained byte-identical to the prototype; `port-origin.json` records their
original hashes. Full logs and source snapshots are generated locally by the
launcher and excluded from Git. There was no new sanitizer run for this port.

Independent teammate reproduction and Reese's receiver/interface confirmation
remain pending. For the next contribution, identify the receiving sink factory
for `standalone_open_system_impl`, including image mode, camera routing, clock,
calibration and IMU requirements, before adding Builder startup wiring.
