# Pali Ijo debugging notes — 2026-09-11

This is an initial investigation, not a claim of general CAD robustness. Two geometry corrections were implemented. The boolean, slice, hollow, representation, and display issues below remain work items. No third-party implementation code was copied. Existing user changes were left in place.

## Changes made

- `lib/src/brep/intersect.dart`, `_planeCylinder`: preserve the signed projection onto a parallel cutting plane. The old code applied the height sign twice. Tests with radius 5, plane offsets ±3, both normal directions, and translated coordinates reproduced intersection lines 6 units off the plane. They now lie on both surfaces.
- `lib/src/brep/fillet.dart`, `_filletPlanePlane`: project the blend axis onto the incident planes without normalizing the setback to the radius. The correct setback is `r / tan(interiorAngle / 2)`. Before the fix, single upright fillets on 3- and 6-sided prisms placed trim vertices approximately 0.479 and 0.171 units off their cylinder at radius 2. Those geometric residuals now pass a 1e-8 bound.
- `test/brep_geometry_regression_test.dart`: nine tests covering these invariants and closed, correctly sized square/hexagonal prism fillets.
- `tool/diagnose_brep.dart`: repeatable reports for three unresolved defects. Run `dart run tool/diagnose_brep.dart` from the package directory.

The sphere tessellator, fillet tessellator, and face-loop fillet construction were not edited. The triangular prism's corrected analytic trim still encounters a separate cap tessellation defect: its topology check passes but its raw display mesh has three open boundary edges. This case is reported by the diagnostic tool, not counted as a successful modeling result.

## Evidence from the existing GUI log

`Main/debug/ijo-kernel.log`, September 10 at 23:58:13, records an 18-face minus 10-face BREP subtraction returning 61 faces. The subsequent two parts have 57 and 4 faces; both report `topo=false`. The operation nevertheless logs `subtract done` with `parts: 2, brep: true`.

At September 11 00:00:27, the union of “Sphere 3” and “Extrude 1” fails in the non-coaxial cylinder–sphere intersection. The engine falls back to mesh CSG. That later fails with 156 open boundary edges. This establishes a real app failure chain, although the log does not contain full input geometry needed to replay that exact model.

## Priority 1: preserve valid topology through booleans

`BrepBoolean._run` returns after assembly and healing without checking BREP validity. `IjoOps.tessellateKeep` retries at draft quality and ultimately allows an open draft mesh. `BrepHeal.tessellateClosed` accepts a closed mesh before checking BREP topology, and its topology check only counts edge uses. None of these establish complete solid validity.

The diagnostic tool reproduces a no-op failure: fillet edge 0 of a 20-unit box at radius 2, then subtract a 5-unit box translated by (100,100,100). The input is topologically closed; the output is not, despite no overlap and the same seven faces.

Two dangerous repair heuristics need attention:

- `_disconnectNonManifold` treats adjacent normals with dot product above 0.7 as a pinch. Smooth fillet transitions also have aligned normals. A local material-side test and consistent coedge orientation are needed to distinguish them. This is a candidate contributor, not a proven sole cause of the no-op failure.
- `_pairOpenTwins` pairs by endpoint IDs, even pairing a line with an arc, without proving curve equivalence or adjusting coedge reversal for opposite endpoint ordering. Sharing endpoints does not mean two edges trace the same boundary.

Proposed next change: a diagnostic BREP validator for index validity, loop continuity, edge endpoints on curves, edges on incident surfaces, paired orientation, and shell closure. Return separate success, empty, unsupported, and invalid results; commit geometry only after validation. Preserve the old model on failure. Do not automatically mesh-fallback an invalid BREP result.

## Priority 2: intersection curves must trim both incident faces

`_collectCylinder` splits UV rectangles for axial/perpendicular planes. For oblique planes it explicitly keeps the cylinder wall whole, even though the planar side may acquire an ellipse. Nonparallel cylinder pairs are also skipped there. Creating a curve is insufficient if both faces are not trimmed consistently to it.

The geometry model currently supports planes, cylinders, spheres, tori, and cones; edge curves are lines, circles, and ellipses. It has no general spline surface, general intersection curve, or per-face parameter-space trim curve representation. Complex lofts, helical BREP threads, and arbitrary curved intersections require extending this foundation.

Proposed direction: shared intersection records, bounded face-pair candidates, parameter-space loop arrangements, classification of each bounded region, and common edges used by both faces. Classify material membership with robust ray/winding logic and boundary handling; the current nearest-face-normal classifier is fragile around trimmed boundaries and concave configurations. Consolidate the scattered fixed tolerances instead of tuning individual epsilons repeatedly.

## Priority 3: slice is not atomic

`BrepSlice._parts` catches all errors independently for intersection and subtraction. One failure can lose half the model; two failures can silently return the original. Distinguish an expected empty half from an algorithm failure and require the entire split to succeed before replacing a body.

Its cylinder shortcut uses an axis dot threshold of 0.999. The diagnostic tool cuts with normal (0.02,0,1), normalized; it receives horizontal cap normals instead. This is a reproducible wrong-plane result, not just rendering. Restrict special cases to geometrically justified tolerances; general oblique cuts need properly trimmed walls. Half-space bounds should include curved surface extrema, not only topological vertices.

## Priority 4: hollow and concave fillets

`BrepHollow._offsetInward` offsets faces and stitches corresponding boundaries. `_projectToSurfaces` averages projection steps for up to 48 iterations but returns without verifying the final residual. There is no general offset self-intersection/topology-change resolution in this path. Torus offsets take an absolute minor radius after it crosses zero, unlike the explicit collapse rejection for spheres/cylinders. Cone radial offset uses `t*cos(slope)` with the original origin/height; its perpendicular wall-thickness semantics need a geometric regression.

Concave fillets need signed material-side construction and correctly oriented blend faces. The present inverse option is not proof that an arbitrary reentrant edge will add the intended volume. The single-edge angle correction does not claim to fix these cases.

## Representation and application boundary

The cache stores analytic solids and display meshes separately. The app's `booleanBodiesAsync` and `sliceMeshesForBodyAsync` catch `BrepUnsupported` and prepare mesh fallbacks. `SceneNotifier` normally defers committing a conversion for user confirmation. This explains why a conversion prompt can appear for apparently basic operations: the kernel's supported intersection family, not the UI tool name, determines availability.

Threads are explicitly mesh-only in `threadMeshesForBodyAsync`; an up-to-date analytic threaded BREP is not currently constructed. General lofts also fall back to mesh; nonuniform scaling of curved surfaces is unsupported. Primitive BREP creation and some straight/circular/polyline pipe cases already exist.

An old unthreaded BREP must not be retained as though it represents a threaded result. Model representation should be explicit and operation results should carry their BREP and display mesh together, rather than depending on shared last-operation state and mesh matching. The package README is internally inconsistent: its BREP-first introduction is followed by older mesh-only operation descriptions.

## Research informing the proposed architecture

These are conceptual references, not implementation sources:

- [OCCT boolean algorithm specification](https://occt3d.com/dev/doc/overview/html/specification__boolean_operations.html): separates intersection computation from result building, with shared data and tolerance-aware vertex/edge/face interactions. This supports building a reusable intersection/arrangement layer for both booleans and slices.
- [Shewchuk: robust geometric predicates](https://www.cs.cmu.edu/~quake/robust.html): adaptive precision protects geometric sign decisions near degeneracy. It is relevant to orientation and intersection predicates, not a substitute for correct topology.
- [B-rep result repair through intersection-edge inference](https://arxiv.org/abs/2310.10351): identifies inaccurate intersection edges as a source of invalid boolean results and describes tolerance-based repair reasoning. Repair should be justified by geometry, not only endpoint proximity.

## GUI verification sequence

1. Preserve a copy of a failing `.pali` model and identify the exact selected edge/face, operation parameters, and body transform.
2. Try a radius-2 single upright fillet on a hexagonal prism; compare before/after in normal and X-Ray views. Check a box face-loop fillet and sphere for visual regressions.
3. Replay a subtraction away from a fillet, then a second cut and union. Capture both operands' full BREP plus transforms, result, topology diagnostics, and display quality.
4. Slice the same model horizontally, at a slight tilt, through a curved boundary, and then slice one resulting part again. Check material/volume conservation and cap normals.
5. Only after boolean and trim invariants are stable, extend hollow, reentrant fillets, and female threads.

The command-line checks establish specific geometry invariants. They do not substitute for the GUI workflow above.

## Validation results

- Initial focused baseline: 56 existing BREP/kernel/fillet tests passed.
- New regression tests before corrections: four failed (two signed plane offsets and the triangular/hexagonal fillet trim checks). The final nine-test regression file is included in the full run below.
- Full kernel run after corrections: 391 tests passed; `test/cli_test.dart` could not load because lines 54 and 429 still use `IjoDocument.decode(...).solid` as `MeshData`, although it is `BrepSolid`. This existing type mismatch was not changed.
- Selected Flutter integration run (`brep_handshake_test.dart`, `brep_app_bridge_test.dart`, `boolean_flow_test.dart`): 27 passed, four failed. All four failures also reproduce against an isolated kernel copy with the original intersection and fillet math restored: the view-only error wording assertion, inscribed-cylinder subtraction closure, and two unsupported-boolean/fallback assertions.
- Static analysis of the two edited geometry files, regression test, and diagnostic tool: no issues.
- No interactive macOS GUI verification has been performed yet. Do not interpret the passing tests as coverage of arbitrary operation sequences.

## Follow-up: distant subtraction fixed

The first topology break was traced to `_collect` / `_assemble`, before healing: planar fillet caps were sampled into polygons and rebuilt with straight chords. Adjacent cylinder faces retained the true circular edges, leaving unmatched boundaries. The existing source-face-copy path only activated when a new circular hole existed, so an unchanged face could lose its analytic outer loop.

The boolean now preserves source loops when the classification polygon has not been replaced by trimming. This fixes not only the distant single-edge case but also a pocket and a through-cut on the opposite side of a filleted box. Sphere notches retain their existing treatment because they may alter the boundary later in this path.

For a provably disjoint subtraction, the kernel now returns an independent copy of the input instead of reconstructing its topology. `conservativeSolidBounds` encloses full curved extents, including sphere/torus bulges and cylinder/cone axial ranges; it deliberately overestimates trimmed patches. Vertex-only or tessellated bounds are not used. Touching/near-touching bounds (within 1e-6) and overlapping bounds still use the normal boolean pipeline. A separate regression places a cutter outside the solid but inside its bounds to exercise that pipeline.

The initial eight regression cases all failed against the pre-fix kernel. The expanded eleven kernel regressions pass: single-edge and face-loop fillets at three distant offsets, unchanged geometry/input immutability, analytic shared edges, closed display and conserved volume, real pocket/through-cut volume removal, overlapping spheres, conservative bounds on rotated curved primitives, and the overlapping-bounds/nonintersecting cut.

A new Flutter regression also passes: a translated and rotated face-filleted body stays closed and analytic through `booleanBodiesAsync` and `adoptLastOpOrMesh`. Fillet/sphere visualization code is unchanged. General intersecting curved-surface trimming and pinch-repair heuristics remain separate work; this change does not claim to solve them.

Follow-up validation: the broader kernel run reached 402 passing tests, with only the same existing CLI test-file compilation failure. After the conservative sphere-notch guard, the focused BREP/kernel/regression run passed all 55 tests. Static analysis of the changed kernel files and new kernel test reported no issues. Interactive GUI verification is still pending.
