# Native BREP threads

`BrepThreads` constructs threads from exact unwrapped helix curves and ruled
surfaces. Triangles are derived display data. Native operations return a
`BrepSolid` or throw; they do not downgrade it to a mesh.

## Dimensions

All lengths use the model's units (millimetres in PaliCAD). `pitch` is the axial
spacing of adjacent crests. `lead = pitch * starts`; handedness changes the
signed axial travel. `ThreadForm` supplies a truncated 60-degree profile:

- External radial depth: `pitch * 17 * sqrt(3) / 48`.
- Internal radial depth: `pitch * 5 * sqrt(3) / 16`.
- External crest width: `pitch / 8`; internal crest width: `pitch / 4`.

These are basic geometric dimensions. Manufacturing fit, process clearance and
tolerance classes are separate decisions; the constructor does not certify a
manufactured fastener or prescribe a print allowance.

For an **external** thread, `radius` is the crest/major radius. For an
**internal** thread, it is the basic minor radius, calculated by subtracting
`ThreadForm(pitch: pitch, internal: true).depth` from the major radius. A bore
larger than that basic minor radius produces shallower engagement while keeping
the nominal major radius. PaliCAD rejects bores smaller than the basic minor
radius or larger than/equal to the major radius with an actionable error.

`origin` is on the axis; `axisZMin` and `axisZMax` are distances along the
normalized axis from that origin. Either limit order is accepted.

```dart
final shaft = BrepPrimitives.cylinder(
  origin: Vector3.zero(), axis: Vector3(0, 0, 1), radius: 3, height: 12,
);
final threaded = BrepThreads.apply(
  shaft,
  origin: Vector3.zero(), axis: Vector3(0, 0, 1),
  radius: 3, pitch: 1, axisZMin: 0, axisZMax: 12,
  length: 6, fromEndA: true, fromEndB: false,
);
final display = IjoOps.tessellateKeep(threaded);
```

`fromEndA` selects the low axial end and `fromEndB` the high end. Both select two
regions; overlapping regions merge. Neither selects the whole interval. A
positive `length` is required, even when the whole interval is selected.

## Construction and transitions

Complete cylindrical shafts and coaxial bores use direct boundary construction.
It joins crest, flank and root patches with shared edges, clips them to the
requested axial regions and retains the unthreaded regions. It avoids repeated
solid booleans for each turn. Interrupted walls use exact profile sweeps clipped
to the requested regions, followed by a BREP subtraction. Multiple starts are
combined as disjoint cutter shells before the subtraction.

`rightHand: false` creates a left-hand thread. `starts` must be positive.
The angular parameter stays unwrapped, including during save/reopen, so a long
helix cannot be confused with its first revolution.

`taper` is radial change per axial unit. `radius` describes the nominal radius
at axial coordinate zero; for example, `taper: -0.25` reduces it by 0.25 mm for
each axial millimetre. This uses native conical helices. It does not allow a
thread profile to cross the axis.

`runoutLength` makes the groove depth and width grow from zero at the first end
and shrink to zero at the other. It must fit twice within every selected region.
This is a gradual geometric lead-in/runout, distinct from a clearance chamfer.
The native API and CLI expose taper and runout; the current PaliCAD thread panel
uses the default straight profile without runout controls.

`BrepThreads.cutter` builds one closed profile sweep. Its `starts` argument
controls lead; callers building every start must supply the angular phases.
`apply` handles this automatically. `subtract` is an unbounded-end convenience
API; use `apply` when the thread must stop at exact axial limits.

## Persistence, interchange and execution

Native JSON and PaliCAD documents retain helix parameters, ruled surfaces,
shared topology and source associations. Mirror and nonsingular affine scale
retain exact transformed BREP geometry.

STEP export represents helices and their ruled surfaces as cubic splines with a
requested distance tolerance, rather than faceted BREP. For native helices,
subdivision uses a fourth-derivative Hermite error bound, including conical
radius change. This is an approximation of the helix for interchange; native
in-memory geometry remains exact. The general intersection-curve fitter has its
own checked tolerances and should not be described as globally certified.

In PaliCAD, native threading and result tessellation run in a killable worker
isolate with the tool's countdown, cancellation and timeout. Successful results
are associated with their BREP before application; a failure/cancellation does
not replace the source body. Direct Dart kernel calls are synchronous; a host
must provide its own scheduling, as PaliCAD does.

The CLI preserves solid input and threads imported mesh input as mesh:

```sh
pali_ijo cylinder --radius 3 --height 12 \
  | pali_ijo threads --diameter 6 --pitch 1 --from a --length 6
```

The CLI accepts `--left-hand`, `--starts`, `--internal`, `--taper` and `--runout`.
Taper and runout require BREP input. Axial limits are measured in the thread's
coordinate frame, including rotated shafts.

## Regression coverage and remaining limits

The thread regressions exercise right/left hand, partial and both-end regions,
multiple starts, nominal female geometry, an interrupted wall, conical threads,
collapsed lead-ins, gradual runouts and chamfered ends. Sequence tests preserve
BREP through document save/reopen, cross-hole subtraction, slicing and STEP
export/import. Other checks cover display qualities, transforms, distance-bounded
helix fitting, invalid inputs and worker timeout without source adoption.

This does not establish that arbitrary CAD geometry is universally supported.
Intersections involving singular/tangent/coincident or highly complex trimmed
surfaces can still reject. Excessive turn/start counts reject before allocating
large sweeps; the general construction budget is 16,384 quarter-turn spans plus
start/end overhead, and the direct wall path has a tighter turn budget. Such
errors preserve BREP and are not permission to silently fall back to triangles.

Related tests:

- `test/brep_threads_test.dart` in PaliCAD.
- `test/brep_thread_sequences_test.dart` in PaliCAD.
- `test/threads_kernel_test.dart` and `test/geometry_worker_responsiveness_test.dart` in PaliCAD.
- `test/brep_thread_guards_test.dart`, `test/cli_test.dart`, `test/sweep_test.dart` in this package.
