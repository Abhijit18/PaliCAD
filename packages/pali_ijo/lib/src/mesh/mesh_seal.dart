import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import 'mesh_data.dart';
import 'mesh_regions.dart';
import '../ijo_log.dart';
import 'mesh_edit.dart';
import 'mesh_polygon.dart';
import 'mesh_topology.dart';

/// Make a mesh a closed solid: unify coplanar faces, drop junk, split T, cap.
class MeshSeal {
  MeshSeal._();

  /// True when every edge is shared by exactly two triangles (closed solid).
  static int openBoundaryEdgeCount(MeshData mesh) =>
      MeshFaceRegions.openBoundaryEdgeCount(mesh);

  /// A manifold shell needs exactly two *opposite* uses of every edge.
  /// Counting only once-used edges accepted flipped triangles and edges shared
  /// by three or more faces. Position welding also covers triangle soups.
  static bool isClosedSolid(MeshData mesh) {
    if (mesh.faces.isEmpty) return false;
    final positions = <(int, int, int), int>{};
    final ids = <int>[];
    for (final p in mesh.vertices) {
      if (!p.x.isFinite || !p.y.isFinite || !p.z.isFinite) return false;
      final key = (
        (p.x * 1e6).round(),
        (p.y * 1e6).round(),
        (p.z * 1e6).round(),
      );
      ids.add(positions.putIfAbsent(key, () => positions.length));
    }
    final edges = <(int, int), (int, int)>{};
    for (final face in mesh.faces) {
      if (face.length < 3 || face.any((i) => i < 0 || i >= ids.length))
        return false;
      // Sub-tolerance slivers collapse under the same position welding used
      // for edge matching; they have no boundary or renderable area.
      final ring = <int>[];
      for (final i in face) {
        if (ring.isEmpty || ring.last != ids[i]) ring.add(ids[i]);
      }
      if (ring.length > 1 && ring.first == ring.last) ring.removeLast();
      if (ring.toSet().length < 3) continue;
      for (var j = 0; j < ring.length; j++) {
        final a = ring[j], b = ring[(j + 1) % ring.length];
        final key = a < b ? (a, b) : (b, a);
        final old = edges[key] ?? (0, 0);
        if (old.$1 >= 2) return false;
        edges[key] = (old.$1 + 1, old.$2 + (a < b ? 1 : -1));
      }
    }
    return edges.isNotEmpty &&
        edges.values.every((e) => e.$1 == 2 && e.$2 == 0);
  }

  /// Leave a closed solid alone. Weld of a T-split annulus (sphere∪cube
  /// face-extrude) merges Steiner verts on the circle and opens the shell.
  static MeshData preserveClosed(MeshData mesh) {
    if (isClosedSolid(mesh)) {
      if (MeshEdit.signedVolume(mesh) < 0) return MeshEdit.invert(mesh);
      return mesh;
    }
    var m = dropDegenerateFaces(mesh);
    m = MeshEdit.weld(m, epsilon: 1e-5);
    if (!isClosedSolid(m)) {
      m = stitchTJunctions(m);
    }
    if (isClosedSolid(m) && MeshEdit.signedVolume(m) < 0) {
      m = MeshEdit.invert(m);
    }
    return m;
  }

  /// Split T-junctions and weld. Does not ear-clip holes.
  ///
  /// CSG's infinite-plane clips leave Steiner vertices on edges. Splitting
  /// those T-junctions is enough to restore a 2-manifold. [sealSolid] also
  /// caps leftover loops; that invents faces (a one-triangle cube side) and
  /// must not run on boolean output. [snapPlanarVertices] / dropping
  /// T-split slivers afterwards reopens the mesh: do not
  /// snap slivers inside the boolean.
  ///
  /// Tries an open-edge-only fan first (does not walk interior walls), then
  /// the historical full [MeshTopology.splitTJunctions] walk. A full walk
  /// stacked overlapping slivers on a tessellated C minus a through-cylinder
  /// (264v/524f − 64v/124f). Some pockets still need that full walk, so both
  /// paths run from the original mesh and the closer result is kept.
  static MeshData stitchTJunctions(MeshData mesh, {double epsilon = 1e-4}) {
    var m = MeshEdit.triangulate(mesh);
    MeshData finish(MeshData x) {
      if (x.faces.isNotEmpty && MeshEdit.signedVolume(x) < 0) {
        return MeshEdit.invert(x);
      }
      return x;
    }

    if (openBoundaryEdgeCount(m) == 0) return finish(m);
    final openEps = epsilon;
    final weldEps = math.min(1e-5, epsilon);

    // Open-edge fans do not walk interior walls. A full long-edge T-split
    // stacked overlapping slivers on a 132-gon C minus a through-cylinder.
    var openFirst = MeshTopology.splitOpenEdgeTJunctions(m, epsilon: openEps);
    openFirst = MeshEdit.weld(openFirst, epsilon: weldEps);
    if (isClosedSolid(openFirst)) {
      IjoLog.trace(
        'stitch path=open-edge '
        '${IjoLog.size(openFirst.vertices.length, openFirst.faces.length)}',
      );
      return finish(openFirst);
    }

    // Original path: some pockets / fillets still need the full walk.
    var full = MeshTopology.splitTJunctions(m, epsilon: epsilon);
    full = MeshEdit.weld(full, epsilon: weldEps);
    if (!isClosedSolid(full)) {
      full = MeshTopology.splitTJunctions(full, epsilon: openEps);
      full = MeshEdit.weld(full, epsilon: weldEps);
    }
    if (isClosedSolid(full)) {
      IjoLog.trace(
        'stitch path=full '
        '${IjoLog.size(full.vertices.length, full.faces.length)}',
      );
      return finish(full);
    }

    // A full T-walk that balloons the soup (sphere ∪ dense fillet) stacks
    // overlapping slivers and still leaves holes. Keep the open-edge result.
    final cap = math.max(m.faces.length, openFirst.faces.length) * 8 + 64;
    if (full.faces.length > cap) {
      IjoLog.trace(
        'stitch path=full-skip exploded '
        '${IjoLog.size(full.vertices.length, full.faces.length)}',
      );
      full = openFirst;
    }

    var best = openBoundaryEdgeCount(openFirst) <= openBoundaryEdgeCount(full)
        ? openFirst
        : full;
    if (!isClosedSolid(best)) {
      best = MeshTopology.splitOpenEdgeTJunctions(best, epsilon: openEps);
      best = MeshEdit.weld(best, epsilon: weldEps);
    }
    IjoLog.trace(
      'stitch path=recover closed=${isClosedSolid(best)} '
      'open=${openBoundaryEdgeCount(best)} '
      '${IjoLog.size(best.vertices.length, best.faces.length)}',
    );
    return finish(best);
  }

  /// Project vertices whose incident triangles all lie on one plane onto
  /// that plane (coplanar snap). Leaves crease vertices alone.
  static MeshData snapPlanarVertices(MeshData mesh) {
    if (mesh.vertices.isEmpty || mesh.faces.isEmpty) return mesh;
    final verts = [for (final v in mesh.vertices) v.clone()];
    final incident = List<List<int>>.generate(verts.length, (_) => <int>[]);
    for (var f = 0; f < mesh.faces.length; f++) {
      for (final vi in mesh.faces[f]) {
        if (vi >= 0 && vi < incident.length) incident[vi].add(f);
      }
    }

    Vector3? faceNormal(int f) {
      final face = mesh.faces[f];
      if (face.length < 3) return null;
      final n = (mesh.vertices[face[1]] - mesh.vertices[face[0]]).cross(
        mesh.vertices[face[2]] - mesh.vertices[face[0]],
      );
      if (n.length < 1e-12) return null;
      return n.normalized();
    }

    for (var vi = 0; vi < verts.length; vi++) {
      final faces = incident[vi];
      if (faces.isEmpty) continue;
      Vector3? n0;
      Vector3? p0;
      var ok = true;
      for (final f in faces) {
        final n = faceNormal(f);
        if (n == null) continue;
        n0 ??= n;
        p0 ??= mesh.vertices[mesh.faces[f][0]];
        if (n.dot(n0).abs() < MeshFaceRegions.coplanarNormalMinDot) {
          ok = false;
          break;
        }
        final d = (mesh.vertices[mesh.faces[f][0]] - p0).dot(n0);
        if (d.abs() >= MeshFaceRegions.coplanarOffsetEpsilon) {
          ok = false;
          break;
        }
      }
      if (!ok || n0 == null || p0 == null) continue;
      verts[vi] -= n0 * (verts[vi] - p0).dot(n0);
    }
    return MeshData(
      vertices: verts,
      faces: [for (final f in mesh.faces) List<int>.from(f)],
    );
  }

  /// Snap tiny coplanar leftovers onto their parent plane so they join the
  /// same face. Leaves crease slivers (two large neighbors on different
  /// planes) alone so knife edges still split.
  static MeshData absorbSliverFaces(MeshData mesh) {
    if (mesh.faces.length < 2) return mesh;
    final tri = MeshEdit.triangulate(mesh);
    double areaOf(int f) {
      final face = tri.faces[f];
      if (face.length < 3) return 0;
      final n = (tri.vertices[face[1]] - tri.vertices[face[0]]).cross(
        tri.vertices[face[2]] - tri.vertices[face[0]],
      );
      return 0.5 * n.length;
    }

    Vector3? normalOf(int f) {
      final face = tri.faces[f];
      if (face.length < 3) return null;
      final n = (tri.vertices[face[1]] - tri.vertices[face[0]]).cross(
        tri.vertices[face[2]] - tri.vertices[face[0]],
      );
      if (n.length < 1e-12) return null;
      return n.normalized();
    }

    final areas = [for (var i = 0; i < tri.faces.length; i++) areaOf(i)];
    var mean = 0.0;
    for (final a in areas) {
      mean += a;
    }
    mean /= areas.length;
    final sliverLimit = mean * 1e-3 > 1e-6 ? mean * 1e-3 : 1e-6;

    final verts = [for (final v in tri.vertices) v.clone()];
    var snapped = false;
    for (var f = 0; f < tri.faces.length; f++) {
      if (areas[f] >= sliverLimit) continue;
      final nSliver = normalOf(f);
      if (nSliver == null) continue;
      final neighbors = <int>{};
      final face = tri.faces[f];
      for (var i = 0; i < face.length; i++) {
        for (final n in MeshFaceRegions.facesForEdge(
          tri,
          face[i],
          face[(i + 1) % face.length],
        )) {
          if (n != f) neighbors.add(n);
        }
      }
      int? parent;
      var parentArea = sliverLimit;
      var otherLarge = false;
      for (final n in neighbors) {
        if (areas[n] <= sliverLimit) continue;
        final nn = normalOf(n);
        if (nn == null) continue;
        if (nn.dot(nSliver).abs() >= MeshFaceRegions.coplanarNormalMinDot) {
          if (areas[n] > parentArea) {
            parent = n;
            parentArea = areas[n];
          }
        } else if (areas[n] > sliverLimit * 10) {
          otherLarge = true;
        }
      }
      if (parent == null || otherLarge) continue;
      final origin = verts[tri.faces[parent][0]];
      final pn = normalOf(parent);
      if (pn == null) continue;
      for (final vi in face) {
        verts[vi] -= pn * (verts[vi] - origin).dot(pn);
      }
      snapped = true;
    }
    if (!snapped) return tri;
    return dropDegenerateFaces(
      MeshEdit.weld(
        MeshData(
          vertices: verts,
          faces: [for (final f in tri.faces) List<int>.from(f)],
        ),
        epsilon: 1e-5,
      ),
    );
  }

  /// Drop zero-area and collapsed triangles.
  static MeshData dropDegenerateFaces(MeshData mesh) {
    final srcIds = mesh.faceIds;
    final keepIds = srcIds != null && srcIds.length == mesh.faces.length;
    final faces = <List<int>>[];
    final outIds = keepIds ? <int>[] : null;
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final face = mesh.faces[fi];
      if (face.length < 3) continue;
      if (face[0] == face[1] || face[1] == face[2] || face[2] == face[0]) {
        continue;
      }
      final a = mesh.vertices[face[0]];
      final b = mesh.vertices[face[1]];
      final c = mesh.vertices[face[2]];
      if ((b - a).cross(c - a).length2 < 1e-20) continue;
      faces.add(List<int>.from(face));
      outIds?.add(srcIds![fi]);
    }
    return MeshData(
      vertices: [for (final v in mesh.vertices) v.clone()],
      faces: faces,
      faceIds: outIds,
    );
  }

  /// Merges edge-connected coplanar triangles into one polygon per face and
  /// retriangulates, dissolving collinear seams (the waist left by a face
  /// extrude, CSG T-junctions on a flat wall, …).
  static MeshData unifyCoplanarFaces(MeshData mesh) {
    var tri = MeshEdit.triangulate(
      MeshEdit.weld(MeshTopology.splitTJunctions(mesh), epsilon: 1e-4),
    );
    tri = snapPlanarVertices(tri);
    tri = absorbSliverFaces(tri);
    if (tri.faces.length < 2) return tri;

    final used = List<bool>.filled(tri.faces.length, false);
    final outFaces = <List<int>>[];
    final verts = [for (final v in tri.vertices) v.clone()];

    Vector3? faceNormal(int f) {
      final face = tri.faces[f];
      if (face.length < 3) return null;
      final n = (verts[face[1]] - verts[face[0]]).cross(
        verts[face[2]] - verts[face[0]],
      );
      if (n.length < 1e-12) return null;
      return n.normalized();
    }

    void keepOriginal(List<int> region) {
      for (final f in region) {
        outFaces.add(List<int>.from(tri.faces[f]));
      }
    }

    double facesArea(List<int> indices) {
      var a = 0.0;
      for (final f in indices) {
        final face = tri.faces[f];
        if (face.length < 3) continue;
        a +=
            0.5 *
            (verts[face[1]] - verts[face[0]])
                .cross(verts[face[2]] - verts[face[0]])
                .length;
      }
      return a;
    }

    double trisArea(List<List<int>> tris) {
      var a = 0.0;
      for (final face in tris) {
        if (face.length < 3) continue;
        a +=
            0.5 *
            (verts[face[1]] - verts[face[0]])
                .cross(verts[face[2]] - verts[face[0]])
                .length;
      }
      return a;
    }

    void emitGroup(List<int> group) {
      if (group.isEmpty) return;
      final reliable = [
        for (final f in group)
          if (!MeshFaceRegions.isSliverFace(tri, f)) f,
      ];
      if (reliable.isEmpty) {
        keepOriginal(group);
        return;
      }

      var nSum = Vector3.zero();
      for (final f in reliable) {
        final n = faceNormal(f);
        if (n != null) nSum += n;
      }
      if (nSum.length < 1e-12) {
        keepOriginal(group);
        return;
      }
      nSum.normalize();

      List<List<int>>? clip(List<int> src) {
        if (src.isEmpty) return null;
        final loops = MeshFaceRegions.orderedBoundaryLoops(tri, src);
        if (loops.isEmpty || loops.first.length < 3) return null;
        final rings = [
          for (final loop in loops) _dissolveCollinear(verts, loop),
        ].where((l) => l.length >= 3).toList();
        if (rings.isEmpty) return null;
        rings.sort((a, b) {
          double area(List<int> loop) {
            var s = Vector3.zero();
            for (var i = 0; i < loop.length; i++) {
              s += verts[loop[i]].cross(verts[loop[(i + 1) % loop.length]]);
            }
            return s.length;
          }

          return area(b).compareTo(area(a));
        });
        final tris = MeshPolygon.triangulateRings(verts, rings, nSum);
        if (tris.isEmpty) return null;
        final oldArea = facesArea(reliable);
        final newArea = trisArea(tris);
        // Needle-poisoned loops can emit one triangle of a cube quad
        // (viewport hole) or fill a CSG hole. Reject both.
        if (oldArea > 1e-8 &&
            (newArea < oldArea * 0.9 || newArea > oldArea * 1.15)) {
          return null;
        }
        return tris;
      }

      // Prefer the real triangles so crease needles cannot half-fill a
      // cube face. Fall back to the whole group so T-junction slivers can
      // still dissolve a face-extrude waist.
      final tris = clip(reliable) ?? clip(group);
      if (tris == null) {
        keepOriginal(group);
        return;
      }
      outFaces.addAll(tris);
    }

    for (var i = 0; i < tri.faces.length; i++) {
      if (used[i]) continue;
      final region = MeshFaceRegions.growPlanarRegion(tri, i);
      if (region.isEmpty) {
        used[i] = true;
        outFaces.add(List<int>.from(tri.faces[i]));
        continue;
      }
      for (final f in region) {
        used[f] = true;
      }
      // Opposite-winding coplanar leftovers (CSG kept both sheets) must not
      // be ear-clipped as one polygon — that drops half a cube face.
      final seedN = () {
        Vector3? best;
        var bestArea = -1.0;
        for (final f in region) {
          if (MeshFaceRegions.isSliverFace(tri, f)) continue;
          final n = faceNormal(f);
          if (n == null) continue;
          final face = tri.faces[f];
          final area = (verts[face[1]] - verts[face[0]])
              .cross(verts[face[2]] - verts[face[0]])
              .length2;
          if (area > bestArea) {
            bestArea = area;
            best = n;
          }
        }
        return best ?? faceNormal(region.first);
      }();
      if (seedN == null) {
        emitGroup(region);
        continue;
      }
      final plus = <int>[];
      final minus = <int>[];
      for (final f in region) {
        final n = faceNormal(f);
        if (n != null && n.dot(seedN) < 0) {
          minus.add(f);
        } else {
          plus.add(f);
        }
      }
      emitGroup(plus);
      emitGroup(minus);
    }
    final unified = dropDegenerateFaces(
      MeshEdit.weld(MeshData(vertices: verts, faces: outFaces), epsilon: 1e-4),
    );
    // Ear-clip of a concave / holed region can emit a partial fill. Never
    // replace a closed solid with that.
    if (isClosedSolid(tri) && !isClosedSolid(unified)) return tri;
    return unified;
  }

  /// Fill leftover planar holes without a whole-mesh T-junction walk.
  static MeshData capOpenLoops(MeshData mesh) => _capOpenLoops(mesh);

  /// Weld, split T-junctions, and cap leftover holes so modelling ops cannot
  /// return an open / broken shell.
  static MeshData sealSolid(MeshData mesh) {
    var m = dropDegenerateFaces(MeshEdit.triangulate(mesh));
    m = MeshEdit.weld(m, epsilon: 1e-4);
    var open = openBoundaryEdgeCount(m);
    if (open == 0) {
      if (MeshEdit.signedVolume(m) < 0) m = MeshEdit.invert(m);
      return m;
    }
    IjoLog.trace(
      'seal open=$open ${IjoLog.size(m.vertices.length, m.faces.length)} splitting T-junctions',
    );
    m = MeshTopology.splitTJunctions(m);
    m = MeshEdit.weld(m, epsilon: 1e-4);
    open = openBoundaryEdgeCount(m);
    if (open > 0) {
      IjoLog.trace('seal still open=$open, capping loops');
      var capped = _capOpenLoops(m);
      capped = MeshEdit.weld(capped, epsilon: 1e-4);
      capped = dropDegenerateFaces(capped);
      final after = openBoundaryEdgeCount(capped);
      if (after < open) {
        m = capped;
        open = after;
      }
      if (open > 0) {
        IjoLog.trace('seal leftover open=$open after cap');
      }
    }
    if (MeshEdit.signedVolume(m) < 0) m = MeshEdit.invert(m);
    return m;
  }
}

MeshData _capOpenLoops(MeshData mesh) {
  final all = [for (var i = 0; i < mesh.faces.length; i++) i];
  final loops = MeshFaceRegions.orderedBoundaryLoops(mesh, all);
  if (loops.isEmpty) return mesh;
  final verts = [for (final v in mesh.vertices) v.clone()];
  final faces = [for (final f in mesh.faces) List<int>.from(f)];

  Vector3? faceNormal(int f) {
    final face = mesh.faces[f];
    if (face.length < 3) return null;
    final n = (mesh.vertices[face[1]] - mesh.vertices[face[0]]).cross(
      mesh.vertices[face[2]] - mesh.vertices[face[0]],
    );
    if (n.length < 1e-12) return null;
    return n.normalized();
  }

  for (final loop in loops) {
    if (loop.length < 3) continue;
    var nSum = Vector3.zero();
    for (var i = 0; i < loop.length; i++) {
      final a = loop[i];
      final b = loop[(i + 1) % loop.length];
      for (final fi in MeshFaceRegions.facesForEdge(mesh, a, b)) {
        final n = faceNormal(fi);
        if (n != null) nSum += n;
      }
    }
    if (nSum.length < 1e-16) {
      for (var i = 0; i < loop.length; i++) {
        nSum += verts[loop[i]].cross(verts[loop[(i + 1) % loop.length]]);
      }
    }
    if (nSum.length < 1e-16) continue;
    nSum.normalize();
    final tris = _triangulatePlanarLoop(verts, loop, nSum);
    if (tris.isEmpty) continue;
    faces.addAll(tris);
  }
  return MeshData(vertices: verts, faces: faces);
}

List<int> _dissolveCollinear(List<Vector3> verts, List<int> loop) {
  if (loop.length < 4) return List<int>.from(loop);
  bool keep(int prev, int cur, int next) {
    final e1 = verts[cur] - verts[prev];
    final e2 = verts[next] - verts[cur];
    final l1 = e1.length;
    final l2 = e2.length;
    if (l1 < 1e-9 || l2 < 1e-9) return false;
    final cr = e1.cross(e2).length / (l1 * l2);
    final dot = e1.dot(e2) / (l1 * l2);
    return cr > 1e-5 || dot < 0.999999;
  }

  final out = <int>[];
  for (var i = 0; i < loop.length; i++) {
    final prev = loop[(i - 1 + loop.length) % loop.length];
    final cur = loop[i];
    final next = loop[(i + 1) % loop.length];
    if (keep(prev, cur, next)) out.add(cur);
  }
  return out.length >= 3 ? out : List<int>.from(loop);
}

List<List<int>> _triangulatePlanarLoop(
  List<Vector3> verts,
  List<int> loop,
  Vector3 normal,
) {
  if (loop.length == 3) return [List<int>.from(loop)];

  var u = normal.cross(Vector3(1, 0, 0));
  if (u.length2 < 1e-12) u = normal.cross(Vector3(0, 1, 0));
  u.normalize();
  final v = normal.cross(u).normalized();
  final origin = verts[loop.first];
  var ring = [
    for (final i in loop)
      Vector2((verts[i] - origin).dot(u), (verts[i] - origin).dot(v)),
  ];
  var indices = List<int>.from(loop);
  var area = 0.0;
  for (var i = 0; i < ring.length; i++) {
    final a = ring[i];
    final b = ring[(i + 1) % ring.length];
    area += a.x * b.y - b.x * a.y;
  }
  if (area < 0) {
    ring = ring.reversed.toList();
    indices = indices.reversed.toList();
  }

  final n = ring.length;
  if (n == 3) return [indices];

  final active = List<int>.generate(n, (i) => i);
  final tris = <List<int>>[];
  var guard = 0;
  final maxGuard = n * n;

  double cross2(Vector2 a, Vector2 b, Vector2 c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  bool inside(Vector2 p, Vector2 a, Vector2 b, Vector2 c) {
    final c1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    final c2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    final c3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);
    final hasNeg = c1 < -1e-12 || c2 < -1e-12 || c3 < -1e-12;
    final hasPos = c1 > 1e-12 || c2 > 1e-12 || c3 > 1e-12;
    return !(hasNeg && hasPos);
  }

  while (active.length > 3 && guard < maxGuard) {
    guard++;
    var ear = false;
    for (var i = 0; i < active.length; i++) {
      final i0 = active[(i + active.length - 1) % active.length];
      final i1 = active[i];
      final i2 = active[(i + 1) % active.length];
      final a = ring[i0];
      final b = ring[i1];
      final c = ring[i2];
      final cr = cross2(a, b, c);
      if (cr.abs() < 1e-12) {
        active.removeAt(i);
        ear = true;
        break;
      }
      if (cr <= 1e-12) continue;
      var blocked = false;
      for (final j in active) {
        if (j == i0 || j == i1 || j == i2) continue;
        if (inside(ring[j], a, b, c)) {
          blocked = true;
          break;
        }
      }
      if (blocked) continue;
      tris.add([indices[i0], indices[i1], indices[i2]]);
      active.removeAt(i);
      ear = true;
      break;
    }
    if (!ear) break;
  }
  if (active.length != 3) return const [];
  tris.add([indices[active[0]], indices[active[1]], indices[active[2]]]);
  if (tris.isEmpty) return const [];

  final t0 = tris.first;
  final tn = (verts[t0[1]] - verts[t0[0]]).cross(verts[t0[2]] - verts[t0[0]]);
  if (tn.dot(normal) < 0) {
    return [for (final t in tris) t.reversed.toList()];
  }
  return tris;
}
