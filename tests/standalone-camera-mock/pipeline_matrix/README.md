# Native image pipeline compatibility matrix

The [combined native Capture demonstration](../capture_e2e/README.md) now reuses
these components and public-frame CV observer. This matrix retains its declared
after-Capture scope and prior results; it does not silently become a tracker test.

This is the next authorized entry-point experiment. It tests plausible local
software paths without waiting for the team's receiver selection.

**Measured component path:** generated native image fixtures **after Capture** →
optional unchanged Monado format converter → reference-lifetime observation
wrapper → unchanged bounded queue → optional unchanged stereo pairing →
new public-frame CV observer → annotated images and frame logs.

The original Capture lifecycle and its nine scenarios remain separate, preserved
evidence. This matrix is not a full Monado runtime, SLAM or Mercury integration.
No actual camera/CSI/ISP/board path is tested.

## Run

From the enclosing repository root, using its private environment:

~~~sh
PYTHONPATH=tests/standalone-camera-mock .venv/bin/python -m pipeline_matrix
~~~

This component build uses the enclosing checkout, verifies ../reviewed-sources.json
and requires the capture_mock/requirements.lock dependencies. It leaves upstream source unchanged.
A new results directory and source/artifact manifest are created on each run.

For build only:

~~~sh
PYTHONPATH=tests/standalone-camera-mock .venv/bin/python -m pipeline_matrix --build-only
~~~

## Declared scope

plan.json centralizes all settings and axes. The finite matrix contains:

- 108 combinations: R8 / L8 / RGB input × no converter / L8 target / RGB target ×
  zero / 16 padding bytes × one / two / four logical streams × forward / reverse
  input submission order.
- Nine additional deliberate-overload cases: three input formats × three stream
  counts, queue capacity two, 25 ms consumer delay.
- Twelve stereo cases: two / four streams, forward / reverse submission,
  initial pair skew 0 / 500 us / 2 ms. Four streams use two separate stereo
  pairers, not a four-camera tracker.

Normal fixtures contain eight 160×128 frames per stream; overload uses 24.
Source timestamps advance at a virtual 30 FPS. Inputs are submitted without
real-time pacing. Reported execution time is local measured receive-to-CV time,
not sensor latency or measured camera FPS.

The image payload encodes camera and sequence. The observer reads only public
xrt_frame fields and validates complete pixels independently. RGB fixtures repeat
each grayscale value across three channels; the upstream L8-to-RGB converter is
checked against that independently specified output. Padding is explicit and
converted output may use a different tight stride.

## Expected format routes — checked against actual execution

| Input | No converter, diagnostic CV | Native L8 target | Native RGB target |
| --- | --- | --- | --- |
| R8 | Deliverable | Format blocked | Format blocked |
| L8 | Deliverable | Pass-through | Convert grayscale to RGB |
| RGB | Deliverable | Format blocked | Pass-through |

R8 and L8 are different native format enums. A successful no-converter R8 route
does not make R8 acceptable to the upstream SLAM wrapper. FORMAT_BLOCKED means
the real upstream converter logged errors and delivered no output for that route;
its void input callback itself is not an acceptance acknowledgement.

The queue drops the oldest queued frames when full. Converted/sent, received,
processed and unreceived counts remain separate. Output buffers are retained
through graph teardown and checked for unchanged pixels before final release.

## Pairing and timestamps

The actual u_sink_force_genlock component pairs images within its 1 ms window,
serializes delivery left then right, and averages their published timestamps.
Each pair's original source_timestamp is checked separately.

The 2 ms initial mismatch must produce native warnings and no CV output for
that first pair; later matching pairs must recover. This tests software pairing
and rejection, not synchronized physical exposures. Four-camera cases use two
stereo groups, with no claimed global order between those groups.

## Coverage limits

The observation endpoint is a new CV diagnostic, not a production tracker.
The reference-lifetime wrapper adds instrumentation but preserves frame pixels
and public metadata; it reads no upstream/mock-private owner. A small diagnostic
u_log transport replaces only runtime log formatting and writes native messages
to native-diagnostics.log. Converter, queue, pairer and frame/format helpers are
compiled from the original source.

JPEG decode is outside this component build. Bayer, DMA/GPU buffers, BSP services,
arbitrary resolutions, external tracker libraries/calibration and every possible
hardware combination are outside these declared cases. They are not declared
unsupported by this matrix.

tracker-readiness.json records upstream SLAM/Mercury requirements as source
inspection, explicitly separate from measured runs. Reese's exact receiver
contract is needed later to identify the team's selected route, not to execute
this compatibility matrix now.
