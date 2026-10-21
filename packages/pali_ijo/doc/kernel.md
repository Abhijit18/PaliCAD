# Design notes

Background for working *on* the kernel. For working *with* it, the
[README](../README.md) is the reference.

## The one invariant

**`BrepSolid` is the solid.** Triangle meshes are a tessellation for
viewports, picking, and mesh export. Optional `faceIds` / `edgeIds` on
`MeshData` map display triangles back to B-Rep faces and edges. Mesh is a
cache, not a replacement — the same split a B-Rep file toolkit uses
between a solid and its render mesh.

Imported STL/3MF bodies, and bodies the host explicitly converts to mesh,
are mesh solids and use `MeshBoolean`. A `BrepUnsupported` boolean does
**not** convert the solid to mesh. Do not reconstruct a B-Rep from those
triangles and pretend the analytics survived. Display tessellation never
replaces the solid.

Mesh-only booleans run `MeshBoolean` (plane clip + occupancy).
Output is a closed oriented manifold, or empty.

Minkowski sum is every point of A plus every point of B. Convex ⊕ convex is
the hull of pairwise vertex sums. Mixed convexity unions per-triangle hulls
with the convex operand; both non-convex unions face-pair hulls. Difference
subtracts that swept boundary from A (erosion / inward offset). A sphere at
the origin is a round offset. Hollow insets by a wall thickness (AABB face
inset on filled boxes, plane-miter on other planar solids) then subtracts
an opening prism. Explicit round offset remains Minkowski. Fillet rebuilds
the touched faces as a walking blend: plane–plane → cylinder, circular
rim → torus, three-edge convex vertex → sphere. Inverse is the concave
rolling-ball. Sequential `_filletOne` is only the last resort for a
single isolated edge. Chamfer is the same walk with plane / cone blends.
Mesh fillet/chamfer remain only for imported MeshSolids.

## Modules and their direction of dependency

```
brep/     topology, surfaces, boolean, fillet, hollow, extrude, tessellate.
csg/      triangle CSG fallback (MeshBoolean), constructors, Minkowski.
mesh/     MeshData, weld, seal, regions — display + imported solids.
measure/  analytic B-Rep properties; triangle fallback.
select/   mesh FaceSelect/EdgeSelect and BrepFaceSelect/BrepEdgeSelect.
sweep/    profiles and swept meshes (faceted unless a revolve).
io/       JSON is lossless B-Rep; STEP analytic when possible; STL/3MF mesh.
ops/      IjoOps: solid-in/solid-out, mesh facade for imports.
cli/      commands over ops/ and io/.
```

Nothing under `lib/` may import `package:flutter` or `package:palicad`;
`test/no_flutter_test.dart` enforces it.

## Reporting a kernel bug

Debug builds of PaliCAD write:

* `debug/ijo-kernel.log` — timestamped breadcrumbs (also on stderr)
* `debug/ijo-last.json` — last extrude / fillet / boolean / hollow snapshot

Reproduce the issue, then send both files plus a screenshot.

## Triangle CSG

`MeshBoolean` clips polygons by the other mesh's supporting planes and
classifies fragments with two odd-even ray probes.

CAD failures come in families, not as one-off shapes:

| Family | What goes wrong | Symptom |
| --- | --- | --- |
| Coplanar caps | Sketch / exit face shares a plane with the cutter. A triangle strictly inside another is never split; both copies of the face can vanish. | Extrude-subtract from a face; through-cut that lands on the far wall. Host pads 0.4 mm as a workaround. |
| T-junctions | Infinite-plane clip puts a Steiner vertex on an edge; the opposite triangle is unsplit. | Open edges that extra `stitchTJunctions` passes close. Dense curved walls need more than two passes. |
| Dropped fragments | Ray probes graze every direction → "outside"; or the centroid sits in a crack. | Stitch plateaus; isolate throws `broken solid`. |
| Concave clip fans | Plane split emits an n-gon; `_meshFrom` fans from vertex 0. | Self-overlapping triangles, wrong occupancy. |
| Tolerance | `_eps`, `_probe`, weld-from-bbox, `_samePlane` ~0.08°. | Works at 20 mm, fails at 0.1 mm walls or metre-scale parts. |

`stitchTJunctions` recovers family 2. It must not ear-clip (that invents a
wall). If the open-edge count stops falling, it is family 1 or 3.

The durable fix is finite triangle–triangle intersection, then winding.
Until then, host workarounds (pad a cut off the plane, sample Auto along
the prism) stay.

## Placement belongs to the host

`MeshXform` offers translate, mirror and scale. Where a body sits in a scene
is not the kernel's business.

## `.pali` schema

PaliCAD stores the B-Rep on each body when it has one, plus a mesh cache.
Old mesh-only `.pali` files still load as mesh solids.
