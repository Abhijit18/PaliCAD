import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../ijo_log.dart';
import '../csg/boolean.dart';
import 'mesh_data.dart';
import 'mesh_edit.dart';
import 'mesh_polygon.dart';
import 'mesh_regions.dart';
import 'mesh_seal.dart';
import 'mesh_topology.dart';
import 'mesh_xform.dart';

/// Primitives and sweeps: cylinder, pipe, prism, extrude, polygon sweep.
class MeshBuild {
  MeshBuild._();

  /// Sweeps [triangles] along [direction] into a closed prism.
  static MeshData extrudeTriangles({
    required List<Vector3> vertices,
    required List<List<int>> triangles,
    required Vector3 direction,
  }) {
    if (direction.length < 1e-12) {
      return MeshData(vertices: const [], faces: const []);
    }
    final bottom = <Vector3>[for (final v in vertices) v.clone()];
    final top = [for (final v in vertices) v + direction];
    final verts = [...bottom, ...top];
    final off = bottom.length;
    final faces = <List<int>>[];

    final edgeCount = <String, int>{};
    String ek(int a, int b) => a < b ? '${a}_$b' : '${b}_$a';
    for (final t in triangles) {
      if (t.length < 3) continue;
      for (var i = 1; i < t.length - 1; i++) {
        final a = t[0];
        final b = t[i];
        final c = t[i + 1];
        // Bottom faces opposite the sweep so the prism is outward.
        faces.add([a, c, b]);
        faces.add([a + off, b + off, c + off]);
        void count(int u, int v) {
          final k = ek(u, v);
          edgeCount[k] = (edgeCount[k] ?? 0) + 1;
        }

        count(a, b);
        count(b, c);
        count(c, a);
      }
    }

    final used = <String>{};
    for (final t in triangles) {
      if (t.length < 3) continue;
      for (var i = 1; i < t.length - 1; i++) {
        final tri = [t[0], t[i], t[i + 1]];
        for (var e = 0; e < 3; e++) {
          final u = tri[e];
          final v = tri[(e + 1) % 3];
          final k = ek(u, v);
          if ((edgeCount[k] ?? 0) != 1) continue;
          if (!used.add('$u:$v')) continue;
          faces.add([u, v, v + off]);
          faces.add([u, v + off, u + off]);
        }
      }
    }
    return MeshData(vertices: verts, faces: faces);
  }

  /// Triangle-prism along [a]→[b] with cross-section [a, p, q] at end A
  /// and the translated triplet at B.
  static MeshData trianglePrism(Vector3 a, Vector3 b, Vector3 p, Vector3 q) {
    final pb = p + (b - a);
    final qb = q + (b - a);
    return MeshData(
      vertices: [a.clone(), p.clone(), q.clone(), b.clone(), pb, qb],
      faces: const [
        [0, 2, 1],
        [3, 4, 5],
        [0, 1, 4],
        [0, 4, 3],
        [1, 2, 5],
        [1, 5, 4],
        [2, 0, 3],
        [2, 3, 5],
      ],
    );
  }

  /// Sweeps a closed polygon [ring] along [delta] into a watertight prism.
  static MeshData sweepPolygon(List<Vector3> ring, Vector3 delta) {
    if (ring.length < 3 || delta.length2 < 1e-20) {
      return MeshData(vertices: const [], faces: const []);
    }
    final n = ring.length;
    final verts = [
      for (final p in ring) p.clone(),
      for (final p in ring) p + delta,
    ];
    final faces = <List<int>>[];
    final faceIds = <int>[];
    for (var i = 0; i < n; i++) {
      final j = (i + 1) % n;
      faces.add([i, j, j + n]);
      faces.add([i, j + n, i + n]);
      faceIds.add(2 + i);
      faceIds.add(2 + i);
    }
    final caps = earClipPolygon(ring);
    if (caps.isEmpty) {
      // Degenerate / self-intersecting: last-resort fan.
      for (var i = 1; i + 1 < n; i++) {
        faces.add([0, i + 1, i]);
        faceIds.add(0);
        faces.add([n, n + i, n + i + 1]);
        faceIds.add(1);
      }
    } else {
      for (final t in caps) {
        faces.add([t[0], t[2], t[1]]);
        faceIds.add(0);
        faces.add([n + t[0], n + t[1], n + t[2]]);
        faceIds.add(1);
      }
    }
    var mesh = MeshData(vertices: verts, faces: faces, faceIds: faceIds);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  /// Prism: triangulate the profile (holes included), then
  /// sweep. Interior diagonals stay 2-manifold; walls come from 1-use edges.
  static MeshData sweepPolygonWithHoles(
    List<Vector3> outer,
    List<List<Vector3>> holes,
    Vector3 delta,
  ) {
    if (outer.length < 3 || delta.length2 < 1e-20) {
      return MeshData(vertices: const [], faces: const []);
    }
    var nHat = Vector3.zero();
    for (var i = 0; i < outer.length; i++) {
      nHat += outer[i].cross(outer[(i + 1) % outer.length]);
    }
    if (nHat.length2 < 1e-20) nHat = delta;
    final verts = <Vector3>[for (final p in outer) p.clone()];
    final rings = <List<int>>[
      [for (var i = 0; i < outer.length; i++) i],
    ];
    for (final hole in holes) {
      if (hole.length < 3) continue;
      final start = verts.length;
      verts.addAll([for (final p in hole) p.clone()]);
      rings.add([for (var i = 0; i < hole.length; i++) start + i]);
    }
    final tris = MeshPolygon.triangulateRings(verts, rings, nHat);
    if (tris.isEmpty) {
      if (rings.length > 1) {
        return MeshData(vertices: const [], faces: const []);
      }
      return sweepPolygon(outer, delta);
    }
    return extrudeTriangles(vertices: verts, triangles: tris, direction: delta);
  }

  /// Closed box covering the +[normal] half-space at [origin].
  ///
  /// Split by plane: one cube face lies on the
  /// plane and the box is sized to the mesh AABB, not a giant multiple of the
  /// diagonal. A huge cutter made our CSG winding probes miss a thin D-tube
  /// plate (volume larger than the AABB).
  static MeshData halfSpace(MeshData mesh, Vector3 origin, Vector3 normal) {
    final n = normal.normalized();
    var u = n.cross(Vector3(0, 0, 1));
    if (u.length2 < 1e-16) u = n.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = n.cross(u).normalized();
    var minU = 0.0, maxU = 0.0, minV = 0.0, maxV = 0.0;
    var minN = 0.0, maxN = 0.0;
    var first = true;
    for (final p in mesh.vertices) {
      final d = p - origin;
      final du = d.dot(u);
      final dv = d.dot(v);
      final dn = d.dot(n);
      if (first) {
        minU = maxU = du;
        minV = maxV = dv;
        minN = maxN = dn;
        first = false;
      } else {
        minU = math.min(minU, du);
        maxU = math.max(maxU, du);
        minV = math.min(minV, dv);
        maxV = math.max(maxV, dv);
        minN = math.min(minN, dn);
        maxN = math.max(maxN, dn);
      }
    }
    final diag = math.sqrt(
      (maxU - minU) * (maxU - minU) +
          (maxV - minV) * (maxV - minV) +
          (maxN - minN) * (maxN - minN),
    );
    final pad = math.max(diag * 0.02, 1.0);
    final u0 = minU - pad;
    final v0 = minV - pad;
    final n1 = math.max(maxN, 0.0) + pad;
    final box = _aabbCube(maxU - minU + 2 * pad, maxV - minV + 2 * pad, n1);
    var out = MeshXform.transformVertices(
      box,
      (p) => origin + u * (u0 + p.x) + v * (v0 + p.y) + n * p.z,
    );
    if (MeshEdit.signedVolume(out) < 0) out = MeshEdit.invert(out);
    return out;
  }

  /// Splits [mesh] by a plane into every vertex-disconnected solid.
  ///
  /// A U-shaped body cut through both arms is three solids (two stubs plus
  /// the base), not two half-spaces.
  ///
  /// Uses a single-plane clip + planar cap. Two CSGs against a half-space
  /// cube is O(n·m) and timed out on a 12k-triangle STL after the clip had
  /// already succeeded.
  static List<MeshData> sliceParts(
    MeshData mesh,
    Vector3 origin,
    Vector3 normal,
  ) {
    final n = normal.normalized();
    if (n.length2 < 1e-20 || mesh.faces.isEmpty) return const [];
    final sw = Stopwatch()..start();
    final clipped = _sliceByPlaneClip(mesh, origin, n);
    if (clipped != null) {
      IjoLog.line(
        'sliceParts plane-clip ${IjoLog.ms(sw)} parts=${clipped.length}',
      );
      return clipped;
    }
    IjoLog.line('sliceParts plane-clip failed, CSG half-space');
    final cutter = halfSpace(mesh, origin, n);
    List<MeshData> side(MeshData piece, Vector3 outward) {
      var m = piece;
      if (m.faces.isEmpty) return const [];
      if (!MeshSeal.isClosedSolid(m)) {
        m = _capPlanarCut(m, origin, outward);
      }
      if (!MeshSeal.isClosedSolid(m)) {
        m = MeshTopology.splitOpenEdgeTJunctions(m);
        m = MeshSeal.capOpenLoops(m);
      }
      // Do not stitchTJunctions on a giant cutter. After a 12k-tri STL the
      // stitch walked half-space edges until the isolate timed out.
      if (!MeshSeal.isClosedSolid(m)) return const [];
      return [
        for (final p in MeshTopology.decompose(m))
          if (p.faces.isNotEmpty && _slicePartPlausible(p))
            MeshEdit.signedVolume(p) < 0 ? MeshEdit.invert(p) : p,
      ];
    }

    final pos = MeshBoolean.run(mesh, cutter, MeshBoolean.intersectOp);
    final neg = MeshBoolean.run(mesh, cutter, MeshBoolean.subtractOp);
    IjoLog.line(
      'sliceParts pos=${IjoLog.size(pos.vertices.length, pos.faces.length)} '
      'neg=${IjoLog.size(neg.vertices.length, neg.faces.length)}',
    );
    final parts = [...side(neg, n), ...side(pos, -n)];
    IjoLog.line('sliceParts csg-done ${IjoLog.ms(sw)} parts=${parts.length}');
    return parts;
  }

  /// Keep `n·(p−origin) ≥ 0` and `≤ 0` as two capped solids, then decompose.
  static List<MeshData>? _sliceByPlaneClip(
    MeshData mesh,
    Vector3 origin,
    Vector3 n,
  ) {
    const eps = 1e-6;
    var hasPos = false;
    var hasNeg = false;
    for (final v in mesh.vertices) {
      final h = (v - origin).dot(n);
      if (h > eps) hasPos = true;
      if (h < -eps) hasNeg = true;
      if (hasPos && hasNeg) break;
    }
    if (!hasPos && !hasNeg) return const [];

    List<MeshData> finish(MeshData open, Vector3 outward) {
      MeshData snapCut(MeshData src) {
        final verts = [for (final v in src.vertices) v.clone()];
        for (var i = 0; i < verts.length; i++) {
          final h = (verts[i] - origin).dot(n);
          if (h.abs() <= 1e-3) verts[i] = verts[i] - n * h;
        }
        return MeshData(
          vertices: verts,
          faces: src.faces,
          faceIds: src.faceIds,
        );
      }

      var m = MeshSeal.dropDegenerateFaces(snapCut(open));
      if (m.faces.isEmpty) {
        IjoLog.line('sliceParts clip empty after drop');
        return const [];
      }
      if (!MeshSeal.isClosedSolid(m)) {
        // Blender bisect / PSLG fill: snap, weld the cut in UV, split
        // on-plane T-junctions, then cap. Full-mesh stitchTJunctions walks
        // every long wall (sliver snap / 12k-tri hang) and still misses
        // hole loops whose vertices sit *inside* a filled cap triangle.
        m = _weldOnPlane(snapCut(m), origin, n);
        m = _splitOnPlaneTJunctions(m, origin, n);
        m = MeshSeal.dropDegenerateFaces(_weldOnPlane(snapCut(m), origin, n));
        m = _capPlanarCut(m, origin, outward);
        if (!MeshSeal.isClosedSolid(m)) {
          m = _splitOnPlaneTJunctions(m, origin, n);
          m = MeshSeal.dropDegenerateFaces(_weldOnPlane(snapCut(m), origin, n));
          m = _capPlanarCut(m, origin, outward);
        }
        if (!MeshSeal.isClosedSolid(m)) {
          m = _dropDanglingTriangles(m);
          if (!MeshSeal.isClosedSolid(m)) {
            // Clip can expose a T-junction on a face that is *not* the cut
            // (a long cap edge with a wall Steiner). On-plane split misses
            // it; capOpenLoops skips the collinear 3-loop (zero area).
            m = MeshTopology.splitOpenEdgeTJunctions(m);
            m = MeshSeal.capOpenLoops(m);
          }
        }
      }
      if (MeshSeal.isClosedSolid(m)) {
        final cleaned = MeshSeal.dropDegenerateFaces(m);
        if (MeshSeal.isClosedSolid(cleaned)) m = cleaned;
      }
      if (!MeshSeal.isClosedSolid(m)) {
        IjoLog.line(
          'sliceParts clip still open '
          'edges=${MeshSeal.openBoundaryEdgeCount(m)} '
          'v=${m.vertices.length} f=${m.faces.length}',
        );
        return const [];
      }
      if (MeshEdit.signedVolume(m) < 0) m = MeshEdit.invert(m);
      return [
        for (final p in MeshTopology.decompose(m))
          if (p.faces.isNotEmpty && _slicePartPlausible(p))
            MeshEdit.signedVolume(p) < 0 ? MeshEdit.invert(p) : p,
      ];
    }

    final pos = hasPos
        ? finish(_clipKeepHalfSpace(mesh, origin, n, eps), -n)
        : const <MeshData>[];
    final neg = hasNeg
        ? finish(_clipKeepHalfSpace(mesh, origin, -n, eps), n)
        : const <MeshData>[];
    if (hasPos && pos.isEmpty) return null;
    if (hasNeg && neg.isEmpty) return null;
    return [...neg, ...pos];
  }

  /// Keeps triangles with `n·(p−origin) ≥ −eps`, splitting those that cross.
  static MeshData _clipKeepHalfSpace(
    MeshData mesh,
    Vector3 origin,
    Vector3 n,
    double eps,
  ) {
    final verts = <Vector3>[for (final v in mesh.vertices) v.clone()];
    final heights = [for (final v in verts) (v - origin).dot(n)];
    for (var i = 0; i < verts.length; i++) {
      if (heights[i].abs() <= eps) {
        verts[i] = verts[i] - n * heights[i];
        heights[i] = 0.0;
      }
    }
    final faces = <List<int>>[];
    final srcIds = mesh.faceIds;
    final keepIds = srcIds != null && srcIds.length == mesh.faces.length;
    final outIds = <int>[];
    final splitAt = <String, int>{};
    final splitAtPos = <String, int>{};

    int addVert(Vector3 p) {
      verts.add(p);
      heights.add((p - origin).dot(n));
      return verts.length - 1;
    }

    int splitEdge(int ia, int ib) {
      final key = ia < ib ? '$ia|$ib' : '$ib|$ia';
      final hit = splitAt[key];
      if (hit != null) return hit;
      final posKey = MeshFaceRegions.edgeKey(verts[ia], verts[ib]);
      final posHit = splitAtPos[posKey];
      if (posHit != null) {
        splitAt[key] = posHit;
        return posHit;
      }
      final ha = heights[ia];
      final hb = heights[ib];
      final denom = ha - hb;
      final t = denom.abs() < 1e-18 ? 0.5 : (ha / denom).clamp(0.0, 1.0);
      var p = verts[ia] + (verts[ib] - verts[ia]) * t;
      p = p - n * (p - origin).dot(n);
      final i = addVert(p);
      splitAt[key] = i;
      splitAtPos[posKey] = i;
      return i;
    }

    void emit(List<int> poly, int? id) {
      if (poly.length < 3) return;
      for (var i = 1; i + 1 < poly.length; i++) {
        faces.add([poly[0], poly[i], poly[i + 1]]);
        if (keepIds && id != null) outIds.add(id);
      }
    }

    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final face = mesh.faces[fi];
      if (face.length < 3) continue;
      for (var t = 1; t + 1 < face.length; t++) {
        final idx = [face[0], face[t], face[t + 1]];
        final s = [
          for (final i in idx)
            heights[i] > eps
                ? 1
                : heights[i] < -eps
                ? -1
                : 0,
        ];
        if (s[0] < 0 && s[1] < 0 && s[2] < 0) continue;
        if (s[0] == 0 && s[1] == 0 && s[2] == 0) continue;
        if (s[0] >= 0 && s[1] >= 0 && s[2] >= 0) {
          emit(idx, keepIds ? srcIds[fi] : null);
          continue;
        }
        final poly = <int>[];
        for (var i = 0; i < 3; i++) {
          final j = (i + 1) % 3;
          if (s[i] >= 0) poly.add(idx[i]);
          if ((s[i] > 0 && s[j] < 0) || (s[i] < 0 && s[j] > 0)) {
            poly.add(splitEdge(idx[i], idx[j]));
          }
        }
        emit(poly, keepIds ? srcIds[fi] : null);
      }
    }

    return MeshData(
      vertices: verts,
      faces: faces,
      faceIds: keepIds && outIds.length == faces.length ? outIds : null,
    );
  }

  /// Merge vertices that lie on the cut and share the same UV cell.
  ///
  /// Clip inserts Steiner points from independent triangles; at ~80 mm they
  /// can differ by 1e-4 mm in-plane. Boundary walking quantizes at 1 nm, so
  /// those twins never chain into hole loops.
  static MeshData _weldOnPlane(
    MeshData mesh,
    Vector3 origin,
    Vector3 n, {
    double planarEps = 1e-3,
  }) {
    final nHat = n.normalized();
    var u = nHat.cross(Vector3(0, 0, 1));
    if (u.length2 < 1e-16) u = nHat.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = nHat.cross(u).normalized();
    final inv = 1.0 / planarEps;
    final map = <String, int>{};
    final verts = <Vector3>[];
    final remap = List<int>.filled(mesh.vertices.length, 0);
    for (var i = 0; i < mesh.vertices.length; i++) {
      final p = mesh.vertices[i];
      final h = (p - origin).dot(nHat);
      String key;
      if (h.abs() <= planarEps) {
        final d = p - origin - nHat * h;
        key = 'p_${(d.dot(u) * inv).round()}_${(d.dot(v) * inv).round()}';
      } else {
        key =
            's_${(p.x / 1e-5).round()}_${(p.y / 1e-5).round()}_${(p.z / 1e-5).round()}';
      }
      final existing = map[key];
      if (existing != null) {
        remap[i] = existing;
      } else {
        remap[i] = verts.length;
        map[key] = verts.length;
        verts.add(h.abs() <= planarEps ? p - nHat * h : p.clone());
      }
    }
    final srcIds = mesh.faceIds;
    final keepIds = srcIds != null && srcIds.length == mesh.faces.length;
    final faces = <List<int>>[];
    final outIds = keepIds ? <int>[] : null;
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final mapped = [for (final i in mesh.faces[fi]) remap[i]];
      if (mapped.length < 3) continue;
      if (mapped[0] == mapped[1] ||
          mapped[1] == mapped[2] ||
          mapped[2] == mapped[0]) {
        continue;
      }
      faces.add(mapped);
      if (keepIds) outIds!.add(srcIds[fi]);
    }
    return MeshData(vertices: verts, faces: faces, faceIds: outIds);
  }

  /// True when [mesh] is a real solid, not a self-intersecting leftover.
  static bool _slicePartPlausible(MeshData mesh) {
    final vol = MeshEdit.signedVolume(mesh).abs();
    if (vol <= 1e-6) return false;
    var minX = 1e300, minY = 1e300, minZ = 1e300;
    var maxX = -1e300, maxY = -1e300, maxZ = -1e300;
    for (final v in mesh.vertices) {
      minX = math.min(minX, v.x);
      minY = math.min(minY, v.y);
      minZ = math.min(minZ, v.z);
      maxX = math.max(maxX, v.x);
      maxY = math.max(maxY, v.y);
      maxZ = math.max(maxZ, v.z);
    }
    final aabb = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
    return vol < aabb * 1.05 + 1;
  }

  /// Split only edges that lie on the cut, at other on-plane vertices.
  ///
  /// Clip keys Steiner points by vertex *index*. Duplicate verts along a
  /// wall leave a T-junction on the plane. Fanning from `ring[0]` (the
  /// MeshTopology default) keeps the long on-plane edge as a remaining
  /// triangle side when the Steiner point is collinear with it. Fan from
  /// the off-plane apex instead, like a bisect fill.
  static MeshData _splitOnPlaneTJunctions(
    MeshData mesh,
    Vector3 origin,
    Vector3 n, {
    double eps = 1e-3,
  }) {
    final tri = MeshEdit.triangulate(mesh);
    if (tri.faces.isEmpty) return tri;
    final verts = tri.vertices;
    final onIdx = <int>[];
    for (var i = 0; i < verts.length; i++) {
      if ((verts[i] - origin).dot(n).abs() <= eps) onIdx.add(i);
    }
    if (onIdx.length < 3) return tri;
    final eps2 = eps * eps;

    List<({int i, double t})> midsOn(int i0, int i1) {
      final a = verts[i0];
      final b = verts[i1];
      final ab = b - a;
      final len2 = ab.length2;
      if (len2 < eps2) return const [];
      final tol2 = math.max(eps2, len2 * 1e-16);
      final mids = <({int i, double t})>[];
      for (final vi in onIdx) {
        if (vi == i0 || vi == i1) continue;
        final ap = verts[vi] - a;
        final t = ap.dot(ab) / len2;
        if (t <= 1e-6 || t >= 1 - 1e-6) continue;
        if ((a + ab * t - verts[vi]).length2 > tol2) continue;
        mids.add((i: vi, t: t));
      }
      mids.sort((x, y) => x.t.compareTo(y.t));
      return mids;
    }

    final srcIds = tri.faceIds;
    final keepIds = srcIds != null && srcIds.length == tri.faces.length;
    final faces = <List<int>>[];
    final outIds = keepIds ? <int>[] : null;

    void emit(int a, int b, int c, int fi) {
      if (a == b || b == c || a == c) return;
      final cr = (verts[b] - verts[a]).cross(verts[c] - verts[a]);
      if (cr.length2 < 1e-20) return;
      faces.add([a, b, c]);
      outIds?.add(srcIds![fi]);
    }

    var fans = 0;
    for (var fi = 0; fi < tri.faces.length; fi++) {
      final face = tri.faces[fi];
      if (face.length < 3) continue;
      final i0 = face[0];
      final i1 = face[1];
      final i2 = face[2];
      final idx = [i0, i1, i2];
      var splitCount = 0;
      var apex = i0;
      var a = i1;
      var b = i2;
      var mids = <({int i, double t})>[];
      for (var e = 0; e < 3; e++) {
        final ea = idx[e];
        final eb = idx[(e + 1) % 3];
        final found = midsOn(ea, eb);
        if (found.isEmpty) continue;
        splitCount++;
        a = ea;
        b = eb;
        apex = idx[(e + 2) % 3];
        mids = found;
      }
      if (splitCount == 1) {
        fans++;
        var prev = a;
        for (final m in mids) {
          emit(apex, prev, m.i, fi);
          prev = m.i;
        }
        emit(apex, prev, b, fi);
        continue;
      }
      emit(i0, i1, i2, fi);
    }
    IjoLog.trace(
      'sliceParts on-plane T-split fans=$fans onVerts=${onIdx.length}',
    );
    return MeshData(
      vertices: [for (final v in verts) v.clone()],
      faces: faces,
      faceIds: outIds,
    );
  }

  /// Isolated triangles (all three edges 1-use) left by a sliver clip.
  static MeshData _dropDanglingTriangles(MeshData mesh) {
    var m = mesh;
    for (var pass = 0; pass < 8; pass++) {
      if (m.faces.isEmpty) return m;
      final uses = <String, int>{};
      for (final face in m.faces) {
        for (var i = 0; i < face.length; i++) {
          final a = face[i];
          final b = face[(i + 1) % face.length];
          final key = MeshFaceRegions.edgeKey(m.vertices[a], m.vertices[b]);
          uses[key] = (uses[key] ?? 0) + 1;
        }
      }
      final srcIds = m.faceIds;
      final keepIds = srcIds != null && srcIds.length == m.faces.length;
      final faces = <List<int>>[];
      final outIds = keepIds ? <int>[] : null;
      var dropped = 0;
      for (var fi = 0; fi < m.faces.length; fi++) {
        final face = m.faces[fi];
        var boundary = 0;
        for (var i = 0; i < face.length; i++) {
          final a = face[i];
          final b = face[(i + 1) % face.length];
          final key = MeshFaceRegions.edgeKey(m.vertices[a], m.vertices[b]);
          if ((uses[key] ?? 0) <= 1) boundary++;
        }
        if (boundary == face.length && face.length >= 3) {
          dropped++;
          continue;
        }
        faces.add(List<int>.from(face));
        if (keepIds) outIds!.add(srcIds[fi]);
      }
      m = MeshData(
        vertices: [for (final v in m.vertices) v.clone()],
        faces: faces,
        faceIds: outIds,
      );
      if (dropped == 0) break;
    }
    return m;
  }

  static bool _mostlyOnPlane(
    MeshData mesh,
    List<int> loop,
    Vector3 origin,
    Vector3 n, {
    double eps = 1e-3,
  }) {
    if (loop.isEmpty) return false;
    var on = 0;
    for (final i in loop) {
      if (i < 0 || i >= mesh.vertices.length) continue;
      if ((mesh.vertices[i] - origin).dot(n).abs() <= eps) on++;
    }
    if (on < 3) return false;
    return on >= loop.length || on >= (loop.length * 0.9).floor();
  }

  /// Fill open boundary loops that lie on the cut with a planar cap (holes nested).
  static MeshData _capPlanarCut(
    MeshData mesh,
    Vector3 origin,
    Vector3 outward,
  ) {
    final all = [for (var i = 0; i < mesh.faces.length; i++) i];
    final loops = MeshFaceRegions.orderedBoundaryLoops(mesh, all);
    if (loops.isEmpty) return mesh;
    final n = outward.normalized();
    var u = n.cross(Vector3(1, 0, 0));
    if (u.length2 < 1e-12) u = n.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = n.cross(u).normalized();
    Vector2 uvOf(Vector3 p) => Vector2(p.dot(u), p.dot(v));

    bool pip(List<Vector2> ring, Vector2 p) {
      var inside = false;
      for (var i = 0, j = ring.length - 1; i < ring.length; j = i++) {
        final a = ring[i];
        final b = ring[j];
        if ((a.y > p.y) == (b.y > p.y)) continue;
        final den = b.y - a.y;
        if (den.abs() < 1e-18) continue;
        if (p.x < (b.x - a.x) * (p.y - a.y) / den + a.x) inside = !inside;
      }
      return inside;
    }

    List<int> insertSteiner(List<int> loop) {
      const eps = 1e-3;
      final eps2 = eps * eps;
      final out = <int>[];
      for (var i = 0; i < loop.length; i++) {
        final i0 = loop[i];
        final i1 = loop[(i + 1) % loop.length];
        out.add(i0);
        final a = mesh.vertices[i0];
        final b = mesh.vertices[i1];
        final ab = b - a;
        final len2 = ab.length2;
        if (len2 < eps2) continue;
        final tol2 = math.max(eps2, len2 * 1e-16);
        final mids = <({int i, double t})>[];
        for (var vi = 0; vi < mesh.vertices.length; vi++) {
          if (vi == i0 || vi == i1) continue;
          if ((mesh.vertices[vi] - origin).dot(n).abs() > eps) continue;
          final t = (mesh.vertices[vi] - a).dot(ab) / len2;
          if (t <= 1e-6 || t >= 1 - 1e-6) continue;
          if ((a + ab * t - mesh.vertices[vi]).length2 > tol2) continue;
          mids.add((i: vi, t: t));
        }
        mids.sort((x, y) => x.t.compareTo(y.t));
        for (final m in mids) {
          out.add(m.i);
        }
      }
      return out;
    }

    final usable = [
      for (final loop in loops)
        if (loop.length >= 3 && _mostlyOnPlane(mesh, loop, origin, n))
          insertSteiner(loop),
    ];
    IjoLog.trace(
      'sliceParts cap loops=${loops.length} planar=${usable.length} '
      'lens=[${[for (final l in loops) l.length].join(',')}]',
    );
    if (usable.isEmpty) {
      IjoLog.line('sliceParts cap no planar loops (all=${loops.length})');
      return mesh;
    }
    final rings2d = [
      for (final loop in usable) [for (final i in loop) uvOf(mesh.vertices[i])],
    ];
    double areaAbs(List<Vector2> ring) {
      var a = 0.0;
      for (var i = 0; i < ring.length; i++) {
        final p = ring[i];
        final q = ring[(i + 1) % ring.length];
        a += p.x * q.y - q.x * p.y;
      }
      return a.abs();
    }

    int vertsInside(List<Vector2> outer, List<Vector2> inner) {
      var nIn = 0;
      for (final p in inner) {
        if (pip(outer, p)) nIn++;
      }
      return nIn;
    }

    final areas = [for (final r in rings2d) areaAbs(r)];
    final parent = List<int>.filled(usable.length, -1);
    for (var i = 0; i < usable.length; i++) {
      var best = -1;
      var bestArea = double.infinity;
      for (var j = 0; j < usable.length; j++) {
        if (j == i || areas[j] <= areas[i] * 1.001) continue;
        final need = math.max(1, (rings2d[i].length * 0.6).ceil());
        if (vertsInside(rings2d[j], rings2d[i]) < need) continue;
        if (areas[j] < bestArea) {
          bestArea = areas[j];
          best = j;
        }
      }
      parent[i] = best;
    }

    final verts = [for (final p in mesh.vertices) p.clone()];
    final faces = [for (final f in mesh.faces) List<int>.from(f)];
    final srcIds = mesh.faceIds;
    final keepIds = srcIds != null && srcIds.length == mesh.faces.length;
    final outIds = keepIds ? [for (final id in srcIds) id] : null;
    var capId = 0;
    if (keepIds) {
      for (final id in srcIds) {
        if (id >= capId) capId = id + 1;
      }
    }

    for (var i = 0; i < usable.length; i++) {
      if (parent[i] != -1) continue;
      final holes = [
        for (var h = 0; h < usable.length; h++)
          if (parent[h] == i) usable[h],
      ];
      var tris = MeshPolygon.triangulateRings(
        verts,
        [usable[i], ...holes],
        n,
        keepCollinear: true,
      );
      if (tris.isEmpty) {
        tris = MeshPolygon.triangulateRings(verts, [usable[i], ...holes], n);
      }
      IjoLog.trace(
        'sliceParts cap fill outer=${usable[i].length} holes=${holes.length} '
        'tris=${tris.length}',
      );
      if (tris.isEmpty) continue;
      for (final tri in tris) {
        faces.add(tri);
        outIds?.add(capId);
      }
    }

    return MeshData(
      vertices: verts,
      faces: faces,
      faceIds: outIds != null && outIds.length == faces.length ? outIds : null,
    );
  }

  static MeshData _aabbCube(double dx, double dy, double dz) {
    final v = [
      Vector3(0, 0, 0),
      Vector3(dx, 0, 0),
      Vector3(dx, dy, 0),
      Vector3(0, dy, 0),
      Vector3(0, 0, dz),
      Vector3(dx, 0, dz),
      Vector3(dx, dy, dz),
      Vector3(0, dy, dz),
    ];
    const faces = [
      [0, 2, 1],
      [0, 3, 2],
      [4, 5, 6],
      [4, 6, 7],
      [0, 1, 5],
      [0, 5, 4],
      [1, 2, 6],
      [1, 6, 5],
      [2, 3, 7],
      [2, 7, 6],
      [3, 0, 4],
      [3, 4, 7],
    ];
    var mesh = MeshData(vertices: v, faces: faces);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  /// Extrude a picked CAD face: T-split the region so Steiner
  /// T-junctions become 2-use interiors, then sweep. Walls come only from
  /// 1-use edges (outer + holes). Extruding the raw triangle soup treats a
  /// long edge and the two shorts on it as three walls — the "towers" along
  /// a sphere∪cube circle.
  static MeshData extrudeRegion({
    required MeshData mesh,
    required List<int> region,
    required Vector3 direction,
  }) {
    if (region.isEmpty || direction.length < 1e-12) {
      return MeshData(vertices: const [], faces: const []);
    }
    final tris = <List<int>>[];
    var nSum = Vector3.zero();
    Vector3? origin;
    for (final fi in region) {
      if (fi < 0 || fi >= mesh.faces.length) continue;
      final face = mesh.faces[fi];
      if (face.length < 3) continue;
      tris.add(List<int>.from(face));
      final cr = (mesh.vertices[face[1]] - mesh.vertices[face[0]]).cross(
        mesh.vertices[face[2]] - mesh.vertices[face[0]],
      );
      if (cr.length2 < 1e-16) continue;
      nSum += cr;
      origin ??= mesh.vertices[face[0]];
    }
    if (tris.isEmpty) {
      return MeshData(vertices: const [], faces: const []);
    }
    // Never T-split against the whole body. Off-plane sphere verts sit
    // near long cube edges in 3D; inserting them pulls the hole off the
    // plane, the holed sweep bails, and soup extrusion grows fins that
    // look like a broken cylinder in the viewport / X-ray.
    if (origin != null && nSum.length2 > 1e-20) {
      final nHat = nSum.normalized();
      final sub = _planarProfile(mesh, tris, origin, nHat);
      if (sub != null && sub.faces.isNotEmpty) {
        final swept = _orientClosed(
          extrudeTriangles(
            vertices: sub.vertices,
            triangles: sub.faces,
            direction: direction,
          ),
        );
        final holed = _prismFromPlanarLoops(sub, direction, nHat);
        // A collapsed-square sweep CSGs cleanly when the hole sits inside
        // the outer. When the hole almost kisses the rim, that square is
        // a 0.04 mm needle frame and subtract stays open — then keep the
        // T-split profile, which shares the body's actual edges.
        const minClear = 0.5;
        if (holed != null && _minHoleClearance(sub) >= minClear) {
          IjoLog.line(
            'extrudeRegion holed-sweep '
            '${IjoLog.size(holed.vertices.length, holed.faces.length)}',
          );
          return holed;
        }
        if (MeshSeal.isClosedSolid(swept)) {
          IjoLog.line(
            'extrudeRegion t-split-sweep '
            '${IjoLog.size(swept.vertices.length, swept.faces.length)}',
          );
          return swept;
        }
        if (holed != null) {
          IjoLog.line(
            'extrudeRegion holed-sweep-fallback '
            '${IjoLog.size(holed.vertices.length, holed.faces.length)}',
          );
          return holed;
        }
      }
    }
    IjoLog.trace('extrudeRegion empty profile');
    return MeshData(vertices: const [], faces: const []);
  }

  /// Push [region] into the solid by [direction] without CSG.
  ///
  /// The outer loop is shared with adjacent sides — adding prism walls there
  /// duplicates those faces (X-ray "ghost" square and vertical ribs). Recede
  /// the sides to the new floor and only sweep the hole (the cylinder).
  static MeshData pocketRegion({
    required MeshData mesh,
    required List<int> region,
    required Vector3 direction,
  }) {
    if (region.isEmpty || direction.length < 1e-12) {
      return MeshData(vertices: const [], faces: const []);
    }
    final regionSet = {for (final i in region) i};
    final tris = <List<int>>[];
    var nSum = Vector3.zero();
    Vector3? origin;
    for (final fi in region) {
      if (fi < 0 || fi >= mesh.faces.length) continue;
      final face = mesh.faces[fi];
      if (face.length < 3) continue;
      tris.add(List<int>.from(face));
      final cr = (mesh.vertices[face[1]] - mesh.vertices[face[0]]).cross(
        mesh.vertices[face[2]] - mesh.vertices[face[0]],
      );
      if (cr.length2 < 1e-16) continue;
      nSum += cr;
      origin ??= mesh.vertices[face[0]];
    }
    if (tris.isEmpty || origin == null || nSum.length2 < 1e-20) {
      return MeshData(vertices: const [], faces: const []);
    }
    final pushedOrigin = origin;
    final sub = _planarProfile(
      mesh,
      tris,
      origin,
      nSum.normalized(),
      project: false,
    );
    if (sub == null || sub.faces.isEmpty) {
      return MeshData(vertices: const [], faces: const []);
    }
    final loops = MeshFaceRegions.orderedBoundaryLoops(sub, [
      for (var i = 0; i < sub.faces.length; i++) i,
    ]);
    if (loops.isEmpty) {
      return MeshData(vertices: const [], faces: const []);
    }
    var outerI = 0;
    var outerA = _ringArea2([for (final i in loops[0]) sub.vertices[i]]);
    for (var i = 1; i < loops.length; i++) {
      final a = _ringArea2([for (final j in loops[i]) sub.vertices[j]]);
      if (a > outerA) {
        outerA = a;
        outerI = i;
      }
    }
    final nHat = direction.normalized();
    final planeP = sub.vertices[loops[outerI].first] + direction;
    const planeEps = MeshFaceRegions.planarGrowOffsetEpsilon;
    final faceN = nSum.normalized();

    bool onPushedPlane(int fi) {
      for (final vi in mesh.faces[fi]) {
        if ((mesh.vertices[vi] - pushedOrigin).dot(faceN).abs() >= planeEps) {
          return false;
        }
      }
      return true;
    }

    bool onOuterWall(int fi) {
      final f = mesh.faces[fi];
      if (f.length < 3) return false;
      final outer = loops[outerI];
      for (var i = 0; i < outer.length; i++) {
        final a = sub.vertices[outer[i]];
        final b = sub.vertices[outer[(i + 1) % outer.length]];
        final n = (b - a).cross(direction);
        if (n.length2 < 1e-16) continue;
        n.normalize();
        var ok = true;
        for (final vi in f) {
          if ((mesh.vertices[vi] - a).dot(n).abs() >= planeEps) {
            ok = false;
            break;
          }
        }
        if (ok) return true;
      }
      return false;
    }

    final verts = [for (final v in mesh.vertices) v.clone()];
    final faces = <List<int>>[];

    void addClipped(int fi) {
      final f = mesh.faces[fi];
      if (f.length < 3) return;
      for (var t = 1; t + 1 < f.length; t++) {
        final poly = _clipTriKeep(
          mesh.vertices[f[0]],
          mesh.vertices[f[t]],
          mesh.vertices[f[t + 1]],
          planeP,
          nHat,
        );
        if (poly.length < 3) continue;
        final ids = <int>[];
        for (final p in poly) {
          ids.add(verts.length);
          verts.add(p.clone());
        }
        for (var k = 1; k + 1 < ids.length; k++) {
          faces.add([ids[0], ids[k], ids[k + 1]]);
        }
      }
    }

    for (var i = 0; i < mesh.faces.length; i++) {
      if (regionSet.contains(i) || onPushedPlane(i)) continue;
      if (onOuterWall(i)) {
        addClipped(i);
      } else {
        faces.add(List<int>.from(mesh.faces[i]));
      }
    }

    final minHole = outerA * 1e-4;
    final outerPts = _collapseColinearRing([
      for (final i in loops[outerI]) sub.vertices[i].clone(),
    ]);
    final holes = <List<Vector3>>[];
    for (var li = 0; li < loops.length; li++) {
      if (li == outerI) continue;
      final ring = loops[li];
      if (ring.length < 3) continue;
      if (_ringArea2([for (final i in ring) sub.vertices[i]]) <= minHole) {
        continue;
      }
      holes.add([for (final i in ring) sub.vertices[i].clone()]);
    }

    // Retriangulate square-minus-circle. Copying the CSG Steiner fans onto
    // the floor shows radial lines in X-ray / face highlight.
    final capVerts = <Vector3>[...outerPts];
    final capRings = <List<int>>[
      [for (var i = 0; i < outerPts.length; i++) i],
    ];
    for (final hole in holes) {
      final start = capVerts.length;
      capVerts.addAll(hole);
      capRings.add([for (var i = 0; i < hole.length; i++) start + i]);
    }
    var capTris = MeshPolygon.triangulateRings(capVerts, capRings, faceN);
    final cleanCap = capTris.isNotEmpty;
    if (!cleanCap) {
      capVerts
        ..clear()
        ..addAll([for (final p in sub.vertices) p.clone()]);
      capTris = [for (final t in sub.faces) List<int>.from(t)];
    }

    final farOff = verts.length;
    verts.addAll([for (final p in capVerts) p + direction]);
    for (final t in capTris) {
      if (t.length < 3) continue;
      faces.add([farOff + t[0], farOff + t[1], farOff + t[2]]);
    }

    if (cleanCap) {
      var holeFar = farOff + outerPts.length;
      for (final hole in holes) {
        final nearOff = verts.length;
        verts.addAll([for (final p in hole) p.clone()]);
        final n = hole.length;
        for (var i = 0; i < n; i++) {
          final j = (i + 1) % n;
          faces.add([nearOff + i, nearOff + j, holeFar + j]);
          faces.add([nearOff + i, holeFar + j, holeFar + i]);
        }
        holeFar += n;
      }
    } else {
      for (var li = 0; li < loops.length; li++) {
        if (li == outerI) continue;
        final ring = loops[li];
        if (ring.length < 3) continue;
        if (_ringArea2([for (final i in ring) sub.vertices[i]]) <= minHole) {
          continue;
        }
        final nearOff = verts.length;
        verts.addAll([for (final i in ring) sub.vertices[i].clone()]);
        final n = ring.length;
        for (var i = 0; i < n; i++) {
          final j = (i + 1) % n;
          faces.add([nearOff + i, nearOff + j, farOff + ring[j]]);
          faces.add([nearOff + i, farOff + ring[j], farOff + ring[i]]);
        }
      }
    }

    var out = MeshData(vertices: verts, faces: faces);
    out = MeshSeal.preserveClosed(out);
    IjoLog.line(
      'pocketRegion ${IjoLog.size(out.vertices.length, out.faces.length)} '
      'closed=${MeshSeal.isClosedSolid(out)} holes=${loops.length - 1}',
    );
    return out;
  }

  /// Keep the [nHat] side of [planeP], including the plane.
  static List<Vector3> _clipTriKeep(
    Vector3 a,
    Vector3 b,
    Vector3 c,
    Vector3 planeP,
    Vector3 nHat,
  ) {
    const eps = 1e-9;
    double s(Vector3 p) => (p - planeP).dot(nHat);
    Vector3 hit(Vector3 p, Vector3 q, double sp, double sq) {
      final t = sp / (sp - sq);
      return p + (q - p) * t;
    }

    var poly = [a, b, c];
    var scores = [s(a), s(b), s(c)];
    if (scores.every((v) => v < -eps)) return const [];
    if (scores.every((v) => v >= -eps)) return poly;
    final out = <Vector3>[];
    for (var i = 0; i < poly.length; i++) {
      final j = (i + 1) % poly.length;
      final p = poly[i];
      final q = poly[j];
      final sp = scores[i];
      final sq = scores[j];
      final pin = sp >= -eps;
      final qin = sq >= -eps;
      if (pin && qin) {
        out.add(q);
      } else if (pin && !qin) {
        out.add(hit(p, q, sp, sq));
      } else if (!pin && qin) {
        out.add(hit(p, q, sp, sq));
        out.add(q);
      }
    }
    return out;
  }

  /// Region triangles projected onto [nHat], T-split among their own verts.
  ///
  /// Do not drop slightly-off-plane slivers: that punches fake holes in the
  /// profile (23 loops instead of outer+circle). Do not insert off-plane
  /// body verts: those are sphere points that sit near cube rims in 3D.
  static MeshData? _planarProfile(
    MeshData mesh,
    List<List<int>> tris,
    Vector3 origin,
    Vector3 nHat, {
    bool project = true,
  }) {
    final keep = <int>{};
    for (final t in tris) {
      keep.addAll(t);
    }
    if (keep.isEmpty) return null;
    final remap = <int, int>{};
    final planeVerts = <Vector3>[];
    for (final i in keep) {
      remap[i] = planeVerts.length;
      final p = mesh.vertices[i];
      planeVerts.add(project ? p - nHat * (p - origin).dot(nHat) : p.clone());
    }
    var sub = MeshTopology.splitTJunctions(
      MeshData(
        vertices: planeVerts,
        faces: [
          for (final t in tris) [for (final i in t) remap[i]!],
        ],
      ),
    );
    return MeshEdit.weld(sub, epsilon: 1e-5);
  }

  static MeshData _orientClosed(MeshData prism) {
    var out = prism;
    if (!MeshSeal.isClosedSolid(out)) {
      out = MeshSeal.stitchTJunctions(out);
    }
    if (out.faces.isNotEmpty && MeshEdit.signedVolume(out) < 0) {
      out = MeshEdit.invert(out);
    }
    return out;
  }

  /// Smallest distance from a hole vertex to the outer loop.
  static double _minHoleClearance(MeshData profile) {
    final loops = MeshFaceRegions.orderedBoundaryLoops(profile, [
      for (var i = 0; i < profile.faces.length; i++) i,
    ]);
    if (loops.length < 2) return double.infinity;
    List<Vector3> ring(List<int> ids) => [
      for (final i in ids) profile.vertices[i],
    ];
    final rings = [for (final loop in loops) ring(loop)];
    var outerI = 0;
    var outerA = _ringArea2(rings[0]);
    for (var i = 1; i < rings.length; i++) {
      final a = _ringArea2(rings[i]);
      if (a > outerA) {
        outerA = a;
        outerI = i;
      }
    }
    final outer = rings[outerI];
    final minHole = outerA * 1e-4;
    var minD = double.infinity;
    for (var i = 0; i < rings.length; i++) {
      if (i == outerI) continue;
      // A leftover CSG sliver loop sits on the outer or the hole and
      // would make clearance look like 0, forcing a T-split sweep of a
      // profile that already has a clean square+circle holed sweep.
      if (_ringArea2(rings[i]) <= minHole) continue;
      for (final p in rings[i]) {
        for (var e = 0; e < outer.length; e++) {
          final d = _pointSegDist(p, outer[e], outer[(e + 1) % outer.length]);
          if (d < minD) minD = d;
        }
      }
    }
    return minD;
  }

  static double _pointSegDist(Vector3 p, Vector3 a, Vector3 b) {
    final ab = b - a;
    final len2 = ab.length2;
    if (len2 < 1e-20) return (p - a).length;
    final t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    return (a + ab * t - p).length;
  }

  static MeshData? _prismFromPlanarLoops(
    MeshData profile,
    Vector3 direction, [
    Vector3? planeNormal,
  ]) {
    if (profile.faces.isEmpty) return null;
    final n0 =
        planeNormal ??
        (profile.vertices[profile.faces[0][1]] -
                profile.vertices[profile.faces[0][0]])
            .cross(
              profile.vertices[profile.faces[0][2]] -
                  profile.vertices[profile.faces[0][0]],
            );
    if (n0.length2 < 1e-20) return null;
    final nHat = n0.normalized();
    const eps = MeshFaceRegions.planarGrowOffsetEpsilon;
    final origin = profile.vertices[profile.faces[0][0]];
    for (final f in profile.faces) {
      for (final vi in f) {
        if ((profile.vertices[vi] - origin).dot(nHat).abs() >= eps) {
          return null;
        }
      }
    }
    final loops = MeshFaceRegions.orderedBoundaryLoops(profile, [
      for (var i = 0; i < profile.faces.length; i++) i,
    ]);
    if (loops.isEmpty) return null;
    List<Vector3> ring(List<int> ids) => [
      for (final i in ids) profile.vertices[i].clone(),
    ];
    final rings = [for (final loop in loops) ring(loop)];
    var outerI = 0;
    var outerA = _ringArea2(rings[0]);
    for (var i = 1; i < rings.length; i++) {
      final a = _ringArea2(rings[i]);
      if (a > outerA) {
        outerA = a;
        outerI = i;
      }
    }
    final minHole = outerA * 1e-4;
    var outer = _collapseColinearRing(rings[outerI]);
    final holes = <List<Vector3>>[
      for (var i = 0; i < rings.length; i++)
        if (i != outerI && _ringArea2(rings[i]) > minHole) rings[i],
    ];
    IjoLog.line(
      'extrudeRegion loops=${rings.length} outer=${outer.length} '
      'holes=${holes.length}',
    );
    var prism = sweepPolygonWithHoles(outer, holes, direction);
    if (prism.faces.isEmpty) return null;
    if (!MeshSeal.isClosedSolid(prism)) {
      prism = MeshSeal.stitchTJunctions(prism);
    }
    if (!MeshSeal.isClosedSolid(prism)) return null;
    if (MeshEdit.signedVolume(prism) < 0) prism = MeshEdit.invert(prism);
    return prism;
  }

  static List<Vector3> _collapseColinearRing(List<Vector3> ring) {
    if (ring.length < 4) return ring;
    const eps = MeshFaceRegions.planarGrowOffsetEpsilon;
    bool colinear(Vector3 a, Vector3 m, Vector3 b) {
      final ab = b - a;
      final len2 = ab.length2;
      if (len2 < 1e-20) return true;
      final t = (m - a).dot(ab) / len2;
      if (t <= 1e-6 || t >= 1 - 1e-6) return false;
      return (a + ab * t - m).length2 < eps * eps;
    }

    var pts = List<Vector3>.from(ring);
    var changed = true;
    var guard = pts.length + 2;
    while (changed && pts.length > 3 && guard-- > 0) {
      changed = false;
      for (var i = 0; i < pts.length; i++) {
        final prev = pts[(i - 1 + pts.length) % pts.length];
        final next = pts[(i + 1) % pts.length];
        if (!colinear(prev, pts[i], next)) continue;
        pts.removeAt(i);
        changed = true;
        break;
      }
    }
    return pts;
  }

  static double _ringArea2(List<Vector3> ring) {
    var n = Vector3.zero();
    for (var i = 0; i < ring.length; i++) {
      n += ring[i].cross(ring[(i + 1) % ring.length]);
    }
    return n.length2;
  }

  /// Ear-clip a planar simple polygon into triangles (vertex indices into [ring]).
  static List<List<int>> earClipPolygon(List<Vector3> ring) {
    final n = ring.length;
    if (n < 3) return const [];
    if (n == 3)
      return [
        [0, 1, 2],
      ];
    var normal = Vector3.zero();
    for (var i = 0; i < n; i++) {
      normal += ring[i].cross(ring[(i + 1) % n]);
    }
    if (normal.length2 < 1e-20) return const [];
    normal.normalize();
    var u = normal.cross(Vector3(0, 0, 1));
    if (u.length < 1e-8) u = normal.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = normal.cross(u);
    final xs = [for (final p in ring) p.dot(u)];
    final ys = [for (final p in ring) p.dot(v)];

    double cross(int a, int b, int c) =>
        (xs[b] - xs[a]) * (ys[c] - ys[a]) - (ys[b] - ys[a]) * (xs[c] - xs[a]);

    bool inside(int a, int b, int c, int p) {
      final c1 = cross(a, b, p);
      final c2 = cross(b, c, p);
      final c3 = cross(c, a, p);
      return (c1 >= -1e-12 && c2 >= -1e-12 && c3 >= -1e-12) ||
          (c1 <= 1e-12 && c2 <= 1e-12 && c3 <= 1e-12);
    }

    final idx = [for (var i = 0; i < n; i++) i];
    final tris = <List<int>>[];
    var guard = n * n;
    while (idx.length > 3 && guard-- > 0) {
      var clipped = false;
      for (var i = 0; i < idx.length; i++) {
        final prev = idx[(i - 1 + idx.length) % idx.length];
        final cur = idx[i];
        final next = idx[(i + 1) % idx.length];
        if (cross(prev, cur, next) <= 1e-12) continue;
        var ear = true;
        for (final p in idx) {
          if (p == prev || p == cur || p == next) continue;
          if (inside(prev, cur, next, p)) {
            ear = false;
            break;
          }
        }
        if (!ear) continue;
        tris.add([prev, cur, next]);
        idx.removeAt(i);
        clipped = true;
        break;
      }
      if (!clipped) break;
    }
    if (idx.length == 3) tris.add([idx[0], idx[1], idx[2]]);
    return tris.length == n - 2 ? tris : const [];
  }

  /// Lathes a closed [profile] around [axis] through [origin].
  ///
  /// Each profile vertex is projected to (radius, height) in the axis frame,
  /// then swept through [azimuths] (or [segments] equal steps). Used to turn
  /// one meridional fillet/chamfer cross-section into a single solid cutter.
  static MeshData revolvePolygon(
    List<Vector3> profile,
    Vector3 origin,
    Vector3 axis, {
    int segments = 32,
    List<double>? azimuths,
    Vector3? radialZero,
  }) {
    if (profile.length < 3) {
      return MeshData(vertices: const [], faces: const []);
    }
    final t = axis.normalized();
    Vector3 u;
    if (radialZero != null) {
      u = radialZero - t * radialZero.dot(t);
      if (u.length < 1e-10) {
        u = t.cross(Vector3(0, 0, 1));
        if (u.length < 1e-8) u = t.cross(Vector3(0, 1, 0));
      }
    } else {
      u = t.cross(Vector3(0, 0, 1));
      if (u.length < 1e-8) u = t.cross(Vector3(0, 1, 0));
    }
    u.normalize();
    final v = t.cross(u).normalized();

    final rz = <(double r, double z)>[];
    for (final p in profile) {
      final d = p - origin;
      final z = d.dot(t);
      final rad = d - t * z;
      rz.add((rad.length, z));
    }

    final thetas = <double>[];
    var closedRev = true;
    if (azimuths != null && azimuths.length >= 3) {
      thetas.addAll(azimuths);
      thetas.sort();
      final span = thetas.last - thetas.first;
      closedRev = span >= math.pi * 2 - 1e-6;
      if (closedRev && (span - math.pi * 2).abs() < 1e-6) {
        thetas.removeLast();
      }
    } else {
      final nSeg = segments < 3 ? 3 : segments;
      for (var s = 0; s < nSeg; s++) {
        thetas.add(s / nSeg * math.pi * 2);
      }
    }

    final n = profile.length;
    final ns = thetas.length;
    if (ns < 2) {
      return MeshData(vertices: const [], faces: const []);
    }
    final verts = <Vector3>[];
    for (final ang in thetas) {
      final cr = u * math.cos(ang) + v * math.sin(ang);
      for (final (r, z) in rz) {
        verts.add(origin + t * z + cr * r);
      }
    }
    final faces = <List<int>>[];
    final belt = closedRev ? ns : ns - 1;
    for (var s = 0; s < belt; s++) {
      final s0 = s * n;
      final s1 = (closedRev ? (s + 1) % ns : s + 1) * n;
      for (var i = 0; i < n; i++) {
        final j = (i + 1) % n;
        faces.add([s0 + i, s0 + j, s1 + j]);
        faces.add([s0 + i, s1 + j, s1 + i]);
      }
    }
    if (!closedRev) {
      void cap(int station, {required bool flip}) {
        final base = station * n;
        var c = Vector3.zero();
        for (var i = 0; i < n; i++) {
          c += verts[base + i];
        }
        c = c / n.toDouble();
        final ci = verts.length;
        verts.add(c);
        for (var i = 0; i < n; i++) {
          final a = base + i;
          final b = base + (i + 1) % n;
          faces.add(flip ? [ci, b, a] : [ci, a, b]);
        }
      }

      cap(0, flip: true);
      cap(ns - 1, flip: false);
    }
    var mesh = MeshData(vertices: verts, faces: faces);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  /// Geodesic icosphere. Prefer this over [uvSphere] for CSG: a UV mesh
  /// concentrates many sliver triangles at the poles, and those fans leave
  /// the boolean open.
  static MeshData icosphere({
    required Vector3 center,
    required double radius,
    int subdivisions = 3,
  }) {
    const t = 1.618033988749895;
    var verts = <Vector3>[
      Vector3(-1, t, 0),
      Vector3(1, t, 0),
      Vector3(-1, -t, 0),
      Vector3(1, -t, 0),
      Vector3(0, -1, t),
      Vector3(0, 1, t),
      Vector3(0, -1, -t),
      Vector3(0, 1, -t),
      Vector3(t, 0, -1),
      Vector3(t, 0, 1),
      Vector3(-t, 0, -1),
      Vector3(-t, 0, 1),
    ];
    for (final v in verts) {
      v.normalize();
    }
    var faces = <List<int>>[
      [0, 11, 5],
      [0, 5, 1],
      [0, 1, 7],
      [0, 7, 10],
      [0, 10, 11],
      [1, 5, 9],
      [5, 11, 4],
      [11, 10, 2],
      [10, 7, 6],
      [7, 1, 8],
      [3, 9, 4],
      [3, 4, 2],
      [3, 2, 6],
      [3, 6, 8],
      [3, 8, 9],
      [4, 9, 5],
      [2, 4, 11],
      [6, 2, 10],
      [8, 6, 7],
      [9, 8, 1],
    ];
    final depth = subdivisions < 0 ? 0 : subdivisions;
    for (var s = 0; s < depth; s++) {
      final next = <List<int>>[];
      final mid = <String, int>{};
      int midpoint(int a, int b) {
        final key = a < b ? '$a-$b' : '$b-$a';
        final existing = mid[key];
        if (existing != null) return existing;
        final p = (verts[a] + verts[b])..normalize();
        final i = verts.length;
        verts.add(p);
        mid[key] = i;
        return i;
      }

      for (final f in faces) {
        final a = midpoint(f[0], f[1]);
        final b = midpoint(f[1], f[2]);
        final c = midpoint(f[2], f[0]);
        next.add([f[0], a, c]);
        next.add([f[1], b, a]);
        next.add([f[2], c, b]);
        next.add([a, b, c]);
      }
      faces = next;
    }
    verts = [for (final v in verts) center + v * radius];
    var mesh = MeshData(vertices: verts, faces: faces);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  /// UV sphere (lat/lon). Poles are high-valence sliver fans — use
  /// [icosphere] when the mesh will go through a boolean.
  static MeshData uvSphere({
    required Vector3 center,
    required double radius,
    int latSegments = 24,
    int lonSegments = 32,
  }) {
    final nLat = latSegments < 3 ? 3 : latSegments;
    final nLon = lonSegments < 3 ? 3 : lonSegments;
    final verts = <Vector3>[];
    for (var i = 0; i <= nLat; i++) {
      final phi = math.pi * i / nLat;
      final sp = math.sin(phi);
      final cp = math.cos(phi);
      for (var j = 0; j < nLon; j++) {
        final th = math.pi * 2 * j / nLon;
        verts.add(
          center +
              Vector3(
                radius * sp * math.cos(th),
                radius * sp * math.sin(th),
                radius * cp,
              ),
        );
      }
    }
    final faces = <List<int>>[];
    for (var i = 0; i < nLat; i++) {
      for (var j = 0; j < nLon; j++) {
        final j1 = (j + 1) % nLon;
        final a = i * nLon + j;
        final b = i * nLon + j1;
        final c = (i + 1) * nLon + j1;
        final d = (i + 1) * nLon + j;
        if (i > 0) faces.add([a, b, d]);
        if (i + 1 < nLat) faces.add([b, c, d]);
      }
    }
    var mesh = MeshData(vertices: verts, faces: faces);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  static MeshData cylinder({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double height,
    int segments = 24,
  }) {
    final t = axis.normalized();
    var u = t.cross(Vector3(0, 0, 1));
    if (u.length < 1e-8) u = t.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = t.cross(u).normalized();
    final verts = <Vector3>[];
    for (var ring = 0; ring < 2; ring++) {
      final o = origin + t * (ring == 0 ? 0.0 : height);
      for (var i = 0; i < segments; i++) {
        final ang = (i / segments) * math.pi * 2;
        verts.add(
          o + u * (radius * math.cos(ang)) + v * (radius * math.sin(ang)),
        );
      }
    }
    final bottom = verts.length;
    verts.add(origin.clone());
    final top = verts.length;
    verts.add(origin + t * height);
    final faces = <List<int>>[];
    final faceIds = <int>[];
    for (var i = 0; i < segments; i++) {
      final j = (i + 1) % segments;
      faces.add([i, j, j + segments]);
      faces.add([i, j + segments, i + segments]);
      faceIds.add(0);
      faceIds.add(0);
      faces.add([bottom, j, i]);
      faceIds.add(1);
      faces.add([top, i + segments, j + segments]);
      faceIds.add(2);
    }
    return MeshData(vertices: verts, faces: faces, faceIds: faceIds);
  }

  /// Circular pipe swept along [path] with rotation-minimising frames.
  static MeshData pipe(
    List<Vector3> path,
    double diameter, {
    int segments = 16,
  }) {
    if (path.length < 2 || diameter < 1e-9) {
      return MeshData(vertices: const [], faces: const []);
    }
    final radius = diameter * 0.5;
    final frames = <(Vector3, Vector3, Vector3)>[];
    Vector3? prevU;
    for (var i = 0; i < path.length; i++) {
      Vector3 t;
      if (i == 0) {
        t = path[1] - path[0];
      } else if (i == path.length - 1) {
        t = path[i] - path[i - 1];
      } else {
        t = path[i + 1] - path[i - 1];
      }
      if (t.length < 1e-12) t = Vector3(0, 0, 1);
      t.normalize();
      Vector3 u;
      if (prevU == null) {
        u = t.cross(Vector3(0, 0, 1));
        if (u.length < 1e-8) u = t.cross(Vector3(0, 1, 0));
        u.normalize();
      } else {
        u = prevU - t * prevU.dot(t);
        if (u.length < 1e-8) {
          u = t.cross(Vector3(0, 0, 1));
          if (u.length < 1e-8) u = t.cross(Vector3(0, 1, 0));
        }
        u.normalize();
      }
      prevU = u;
      final v = t.cross(u).normalized();
      frames.add((path[i], u, v));
    }

    final verts = <Vector3>[];
    for (final (p, u, v) in frames) {
      for (var i = 0; i < segments; i++) {
        final ang = (i / segments) * math.pi * 2;
        verts.add(
          p + u * (radius * math.cos(ang)) + v * (radius * math.sin(ang)),
        );
      }
    }
    final faces = <List<int>>[];
    for (var s = 0; s < path.length - 1; s++) {
      final a = s * segments;
      final b = (s + 1) * segments;
      for (var i = 0; i < segments; i++) {
        final j = (i + 1) % segments;
        faces.add([a + i, a + j, b + j]);
        faces.add([a + i, b + j, b + i]);
      }
    }
    final start = verts.length;
    verts.add(path.first.clone());
    final end = verts.length;
    verts.add(path.last.clone());
    for (var i = 0; i < segments; i++) {
      final j = (i + 1) % segments;
      faces.add([start, j, i]);
      final last = (path.length - 1) * segments;
      faces.add([end, last + i, last + j]);
    }
    return MeshData(vertices: verts, faces: faces);
  }

  static MeshData prism({
    required double diameter,
    required double height,
    required int sides,
  }) {
    final n = sides < 3 ? 3 : sides;
    final r = diameter * 0.5;
    final c = Vector3(r, r, 0);
    final ring = [
      for (var i = 0; i < n; i++)
        c +
            Vector3(
              r * math.cos(i * math.pi * 2 / n),
              r * math.sin(i * math.pi * 2 / n),
              0,
            ),
    ];
    return sweepPolygon(ring, Vector3(0, 0, height));
  }

  static MeshData pyramid({
    required double diameter,
    required double height,
    required int sides,
  }) {
    final n = sides < 3 ? 3 : sides;
    final r = diameter * 0.5;
    final c = Vector3(r, r, 0);
    final apex = Vector3(r, r, height);
    final ring = [
      for (var i = 0; i < n; i++)
        c +
            Vector3(
              r * math.cos(i * math.pi * 2 / n),
              r * math.sin(i * math.pi * 2 / n),
              0,
            ),
    ];
    final verts = [...ring, apex];
    final faces = <List<int>>[];
    for (var i = 1; i + 1 < n; i++) {
      faces.add([0, i + 1, i]);
    }
    for (var i = 0; i < n; i++) {
      final j = (i + 1) % n;
      faces.add([i, j, n]);
    }
    var mesh = MeshData(vertices: verts, faces: faces);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  static MeshData cone({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double height,
    int segments = 32,
  }) {
    final a = axis.normalized();
    var u = a.cross(Vector3(0, 0, 1));
    if (u.length < 1e-8) u = a.cross(Vector3(0, 1, 0));
    u.normalize();
    final profile = [origin + u * radius, origin + a * height, origin];
    return revolvePolygon(profile, origin, a, segments: segments);
  }

  static MeshData torus({
    required Vector3 center,
    required Vector3 axis,
    required double majorRadius,
    required double minorRadius,
    int segments = 32,
    int tubeSegments = 16,
  }) {
    final a = axis.normalized();
    var u = a.cross(Vector3(0, 0, 1));
    if (u.length < 1e-8) u = a.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = a.cross(u).normalized();
    final circle = [
      for (var i = 0; i < tubeSegments; i++)
        () {
          final ang = i / tubeSegments * math.pi * 2;
          return center +
              u * (majorRadius + minorRadius * math.cos(ang)) +
              a * (minorRadius * math.sin(ang));
        }(),
    ];
    return revolvePolygon(circle, center, a, segments: segments, radialZero: u);
  }

  /// Sampled circular [ring] swept along [direction] → cylinder mesh.
  static MeshData? tryCylinderFromRing(
    List<Vector3> ring,
    Vector3 direction, {
    int segments = 32,
  }) {
    if (ring.length < 16 || direction.length < 1e-12) return null;
    final axis = direction.normalized();
    final height = direction.length;
    final n = ring.length;
    var u = axis.cross(Vector3(0, 0, 1));
    if (u.length < 1e-8) u = axis.cross(Vector3(0, 1, 0));
    u.normalize();
    final yAxis = axis.cross(u);
    Vector3? fitted;
    {
      final a = ring[0];
      final b = ring[n ~/ 3];
      final c = ring[(2 * n) ~/ 3];
      Vector3 to2(Vector3 p) {
        final d = p - a;
        return Vector3(d.dot(u), d.dot(yAxis), 0);
      }

      final p0 = to2(a);
      final p1 = to2(b);
      final p2 = to2(c);
      final d =
          2 *
          (p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y));
      if (d.abs() >= 1e-18) {
        final a2 = p0.x * p0.x + p0.y * p0.y;
        final b2 = p1.x * p1.x + p1.y * p1.y;
        final c2 = p2.x * p2.x + p2.y * p2.y;
        final ux =
            (a2 * (p1.y - p2.y) + b2 * (p2.y - p0.y) + c2 * (p0.y - p1.y)) / d;
        final uy =
            (a2 * (p2.x - p1.x) + b2 * (p0.x - p2.x) + c2 * (p1.x - p0.x)) / d;
        fitted = a + u * ux + yAxis * uy;
      }
    }
    var c = Vector3.zero();
    if (fitted != null) {
      c = fitted;
    } else {
      for (final p in ring) {
        c += p;
      }
      c /= n.toDouble();
    }
    c -= axis * (c - ring.first).dot(axis);
    var rSum = 0.0;
    var maxAlong = 0.0;
    for (final p in ring) {
      final along = (p - c).dot(axis);
      maxAlong = math.max(maxAlong, along.abs());
      rSum += (p - c - axis * along).length;
    }
    final rMean = rSum / n;
    if (rMean < 1e-6) return null;
    if (maxAlong > math.max(0.5, rMean * 0.05)) return null;
    var maxDev = 0.0;
    for (final p in ring) {
      final radial = p - c;
      final r = (radial - axis * radial.dot(axis)).length;
      maxDev = math.max(maxDev, (r - rMean).abs());
    }
    if (maxDev > math.max(1e-4, rMean * 1e-3)) return null;
    return cylinder(
      origin: c,
      axis: axis,
      radius: rMean,
      height: height,
      segments: math.max(segments, ring.length),
    );
  }
}
