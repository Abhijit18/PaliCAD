# Sphere bridge stress case

Requested shape: two cubes joined only through a sphere, then local fillets/chamfers and repeated profile cuts. Implemented in `tool/stress_sphere_bridge.dart` with inputs and outputs saved for each attempted operation. No kernel implementation changes were made during this stress-test pass.

## Geometry and operations

- First cube: x/y/z = 0–20 mm.
- Second cube: x = 24–44 mm, y/z = 0–20 mm. The cubes have a 4 mm gap.
- Connecting sphere: center (22,10,10), radius 8 mm.
- Single upright fillet on the outer edge x=0,y=0: radius 2 mm.
- Single upright chamfer on the opposite cube x=44,y=20: distance 1 mm.
- Separate face-loop fillet on the outer x=0 face: radius 1 mm.
- Nine cutter configurations: two offset/overlapping circle locations in the left cube; a square in the right cube; circular and square cuts through the sphere; a square through both cubes along X; tilted square and circular cuts in the cubes; and a tilted circular cut through the sphere. Circles have radius 1.5 or 2 mm; squares have width 3 or 4 mm. Every cutter is 140 mm long.

Each cutter runs independently against the original joined body and against the fillet-then-chamfer result. A separate cumulative sequence stops when its input becomes invalid. It never reports later stages as passed after an earlier failure.

## Results

33 planned stages: **5 pass, 20 invalid, 8 blocked** under the checks below.

Passing stages: first cube/sphere union, second cube union, single-edge fillet, single-edge chamfer, fillet followed by chamfer. The joined body has 13 faces and a tessellated volume of 16,778.478 mm³. Its analytic reference volume is 16,000 plus the sphere slab between x=20 and x=24, approximately 16,787.492 mm³. The small difference is tessellation error.

The face-loop fillet returns a closed result but loses the original connecting sphere and reduces volume to 7,965.928 mm³. Its corner fillet spheres must not be mistaken for preservation of the original radius-8 bridge. This is why checking only closure or the existence of any sphere face is inadequate.

All eighteen independent cutter attempts fail the acceptance checks. The very first left-cube circle cut has open BREP topology and 64 open boundary edges in the raw display mesh. Some sphere-centered cuts retain paired topology but fail strict closed tessellation. Rectangular bridge cuts can also produce unexpected disconnected components.

The cumulative sequence fails on its first circle cut. Its remaining eight stages, including the intended overlapping-circle cut, are **blocked**, not independently demonstrated cumulative successes or failures. Their cutter shapes are exercised by the independent tests, but the progressively overlapping model cannot yet be built validly.

## Confirmed classifier issue

On the successfully joined input, `BrepClassify.side` reports `outside` for (10,8,10) inside the left cube and (34,10,10) inside the right cube. It also reports `outside` for the bridge center (22,10,10).

`closestOnFace` does not constrain spherical closest points to the trimmed sphere face. For the left-cube probe it returns approximately (14.109,8.685,10), on the part of the supporting sphere buried inside the cube. The boolean uses this classifier to decide which cutter walls survive. This is a strong explanation for the missing hole walls; fixing the trim-aware classifier is the next investigation, not something this run claims to have fixed.

## Checks and limits

Every attempted result checks edge-use closure, strict closed tessellation, positive signed volume, one connected BREP component, retention of the specific original sphere, sample points on incident surfaces, input immutability, and BREP pack/unpack agreement within numerical tolerance. The raw display closure is recorded separately from strict tessellation with repair. Small edge features are rejected if they remove more than 5% of the model. Cuts must remove volume; unions must add it.

These are diagnostic acceptance gates, not a complete geometric validity proof. They do not exhaustively test self-intersection or all possible trim orientations. Two automated foundation tests in `test/sphere_bridge_acceptance_test.dart` additionally check the expected volumes of the bridge and single-edge fillet/chamfer sequence.

Run from `Main/packages/pali_ijo`:

```sh
dart run tool/stress_sphere_bridge.dart ../../debug/sphere-bridge-stress
```

The harness intentionally exits with status 1 when stages fail or are blocked. `Main/debug/sphere-bridge-stress/report.json` contains per-stage metrics. The neighboring `*-input.brep.json`, `*-tool.brep.json`, and `*-result.brep.json` files preserve exact operands and outputs. These are kernel BREP fixtures, not GUI-loadable `.pali` projects. Invalid outputs are saved for diagnosis and never used as accepted inputs to subsequent stages.

## Classifier and face-loop repair follow-up

The subsequent repair run is saved separately in `Main/debug/sphere-bridge-fixed/` so the original failing fixtures remain available. It produces **13 pass, 13 invalid, 7 blocked**.

Two causes are repaired:

- Spherical closest points now respect circular face trims. For solids made from planes, cylinders and circularly trimmed spheres, membership uses analytic ray intersections with trimmed faces. This avoids the invalid nearest-normal inference at concave cube/sphere joins. Ambiguous edge/tangent rays are retried. Unsupported surface/trim families retain the existing classifier; this is not a general replacement for all BREP classification.
- A face-loop fillet's prism rebuild now retains the opposite cap's inner loops and the attached faces, sharing their copied edges. It requires intact prism walls and an identifiable opposite cap. It no longer silently discards the bridge and second cube in this fixture. Fillet and sphere tessellation code is unchanged.

Newly passing stages are the face-loop fillet, the two independent circular-cut locations and right-cube square cut on both the plain and fillet/chamfer body, and the first cumulative circular cut. The second independent circular location is not evidence that overlapping holes work: the cumulative second cut still fails.

Remaining evidence: overlapping circular cuts leave intersecting full-circle inner loops on planar caps, rather than a merged boundary, and the resulting shell has open/mismatched edges. Cuts through the spherical bridge and angled cuts also fail the existing gates. These require further trim/intersection and sewing repairs; changing classification alone does not make them valid.

Regression coverage includes point membership before and after rigid transformation, the closed circular through-cut, and preservation of the full body after the face-loop fillet. The Flutter `brep_sphere_bridge_flow_test.dart` additionally exercises the app engine and verifies BREP retention through cache adoption. These tests do not substitute for manual GUI verification.

## Overlapping-hole follow-up

The next focused repair run (`Main/debug/sphere-bridge-overlap/`) passes **15 stages**, with **13 invalid and 5 blocked**. The cumulative sequence now accepts both overlapping circular cuts and the following square cut in the other cube. Its first remaining failure is the circular cut through the spherical bridge.

Planar assembly now merges a pair of intersecting closed circular hole loops into their two exposed analytic arcs. This is deliberately limited to two coplanar intersecting circles; it does not implement arbitrary multi-loop trimming. Cylinder domain extraction also uses the exact angular span of matching circular cap arcs, avoiding the ambiguous three-sample estimate for arcs longer than a semicircle. No fillet or sphere rendering routine was changed.

New regressions verify overlapping holes on a box and on the sphere bridge, including closure, connectedness, expected removed volume, cavity membership and edge-to-surface consistency. The app-engine regression now performs both cuts consecutively and checks BREP retention after each adoption. Remaining sphere-crossing and angled failures are still recorded, not repaired or hidden by relaxed acceptance gates.

## GUI-reported sketch fallback: trimmed sphere bounds

The user's GUI log identifies `Cylinder–sphere intersection is not implemented yet` from `_collectSphere`, with a 27-face base and a 3-face circular extrusion. The original unsaved model was no longer open, so the regression reproduces the failure mechanism rather than claiming identical coordinates.

A cylinder at x=15,z=10, radius 2, directed along Y intersects the supporting radius-8 sphere centered at (22,10,10), but misses the actual sphere band between x=20 and x=24. Before this repair both plain and face-filleted bridge tests throw the same exception. Conservative sphere-face bounds now incorporate full circular trim constraints and skip proven disjoint face/operand pairs in both sphere and cylinder collection. General non-circular trims retain the full sphere envelope. No tessellation or fillet rendering routine changed.

`test/brep_filleted_bridge_sketch_test.dart` uses the actual scene commit path, Auto combine and -44.31 mm extrusion, on a 27-face bridge with radius-7.56 single-edge fillet, distance-5.43 chamfer and radius-2 face-loop fillet. It checks that no mesh fallback prompt appears, the cache stays BREP, the display is closed and the expected volume is removed. Optional `--dart-define=PALI_FIXTURE_DIR=debug/filleted-bridge-sketch` exports native before/after projects for GUI retesting. These fixtures approximate the reported feature sequence, not the lost original model.

The broader stress sequence remains 15 passing, 13 invalid and 5 blocked. Exposed sphere-crossing and angled failures remain separate work.
