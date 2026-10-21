// Polygon splitting and BSP Boolean fallback: Copyright (c) 2011 Evan Wallace
// MIT license. See THIRD_PARTY_NOTICES.txt and
// licenses/CSG.js-LICENSE.txt in the package root.
//
// Triangle CSG (union / subtract / intersect).
//
// Plane-clip triangle CSG: clip polygons by the other mesh's supporting
// planes, then classify fragments with odd-even ray probes.
//
//   1. Prepare: triangulate, drop degenerates, weld only if the shell stays
//      closed, orient positive volume.
//   2. Split: for each unique supporting plane of the other mesh, clip every
//      polygon that actually hits a triangle on that plane (infinite-plane
//      clip, gated by tri–tri intersection).
//   3. Classify: probe ±normal from each fragment centroid with odd-even ray
//      casts against the *other* solid. Keep the fragment if occupancy of the
//      boolean differs on its two sides. Subtract flips B's kept faces.
//   4. Mesh: weld the polygon soup, drop junk, then T-split until the
//      boundary shrinks no further. Plane clips insert Steiner vertices on
//      edges; those are T-junctions, not missing faces.
//
// That last step cannot invent a face the classifier dropped. If stitch
// plateaus with open edges, classification was wrong (grazing ray, coplanar
// overlap, concave n-gon fan, …). Do not "fix" that by ear-clipping holes
// ([MeshSeal.sealSolid] fabricates walls). See packages/pali_ijo/doc/kernel.md.

import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../ijo_log.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_polygon.dart';
import '../mesh/mesh_edit.dart';
import '../mesh/mesh_regions.dart';
import '../mesh/mesh_seal.dart';

/// Triangle-mesh boolean. Both inputs must be closed, consistently oriented
/// solids. Empty is a valid result (subtract that eats the whole body).
///
/// Failure mode we care about in CAD: the output is a triangle soup whose
/// edges are not all 2-use. That is almost always (a) T-junctions from the
/// plane clip or (b) a fragment that should have been kept and was not.
class MeshBoolean {
  MeshBoolean._();

  static const unionOp = 0;
  static const subtractOp = 1;
  static const intersectOp = 2;

  /// Half-space test and AABB pad. Tight enough that a 32-gon cylinder wall
  /// (chord error ~0.02 mm at Ø10) is not treated as coplanar with a cube
  /// face, loose enough that welded CSG verts still clip.
  static const _eps = 1e-5;

  /// Offset along the fragment normal for the inside/outside occupancy
  /// probes. Must be larger than [_eps] and smaller than typical CAD walls
  /// (a 0.05 mm sheet would need a smaller probe).
  static const _probe = 1e-4;

  static MeshData run(MeshData a, MeshData b, int op, {bool seal = true}) {
    final name = switch (op) {
      subtractOp => 'subtract',
      intersectOp => 'intersect',
      _ => 'union',
    };
    final sw = Stopwatch()..start();
    IjoLog.trace('csg $name in a=${_info(a)} b=${_info(b)}');
    final pa = _prepare(a);
    IjoLog.trace('csg $name prepared A ${_info(pa)}');
    final pb = _prepare(b);
    IjoLog.trace('csg $name prepared B ${_info(pb)}');
    if (pa.faces.isEmpty) {
      IjoLog.trace('csg $name A empty');
      return op == subtractOp
          ? MeshData(vertices: const [], faces: const [])
          : pb;
    }
    if (pb.faces.isEmpty) {
      IjoLog.trace('csg $name B empty');
      return op == intersectOp
          ? MeshData(vertices: const [], faces: const [])
          : pa;
    }

    final boxA = _aabb(pa);
    final boxB = _aabb(pb);
    if (!_aabbOverlap(boxA, boxB, pad: _eps)) {
      IjoLog.trace('csg $name disjoint ${IjoLog.ms(sw)}');
      if (op == unionOp) return MeshEdit.combine([pa, pb]);
      if (op == subtractOp) return pa;
      return MeshData(vertices: const [], faces: const []);
    }

    final A = _polys(pa);
    final B = _polys(pb, idOffset: _maxFaceId(A) + 1);
    IjoLog.trace('csg $name split polys A=${A.length} B=${B.length}');
    final carvedA = _splitBounded(A, B);
    final carvedB = _splitBounded(B, A);
    IjoLog.trace('csg $name classify A=${carvedA.length} B=${carvedB.length}');
    final kept = <_Poly>[];
    for (final p in carvedA) {
      if (_onResult(op, fromA: true, p: p, other: pb)) kept.add(p);
    }
    for (final p in carvedB) {
      if (_onResult(op, fromA: false, p: p, other: pa)) {
        if (op == subtractOp) p.flip();
        kept.add(p);
      }
    }
    IjoLog.trace('csg $name kept polys=${kept.length} ${IjoLog.ms(sw)}');
    var mesh = _meshFrom(kept, seal: seal);
    if (seal &&
        !MeshSeal.isClosedSolid(mesh) &&
        MeshSeal.isClosedSolid(pa) &&
        MeshSeal.isClosedSolid(pb)) {
      // Retry ambiguous coplanar fragments using complete solid partitions.
      // Accept only a closed result; no missing boundary is filled with caps.
      final treeA = _Partition(_polys(pa));
      final treeB = _Partition(_polys(pb, idOffset: _maxFaceId(A) + 1));
      if (op == subtractOp) treeA.invert();
      if (op == intersectOp) {
        treeA.invert();
        treeB.clipTo(treeA);
        treeB.invert();
        treeA.clipTo(treeB);
        treeB.clipTo(treeA);
      } else {
        treeA.clipTo(treeB);
        treeB.clipTo(treeA);
        treeB.invert();
        treeB.clipTo(treeA);
        treeB.invert();
      }
      treeA.build(treeB.all());
      if (op != unionOp) treeA.invert();
      final candidate = _meshFrom(treeA.all());
      if (candidate.faces.isEmpty || MeshSeal.isClosedSolid(candidate))
        mesh = candidate;
    }
    IjoLog.trace('csg $name out ${_info(mesh)} ${IjoLog.ms(sw)}');
    return mesh;
  }

  static String _info(MeshData m) =>
      '${IjoLog.size(m.vertices.length, m.faces.length)} '
      'closed=${MeshSeal.isClosedSolid(m)} '
      'vol=${MeshEdit.signedVolume(m).toStringAsFixed(2)}';

  static MeshData _prepare(MeshData mesh) {
    var m = MeshEdit.triangulate(mesh);
    final cleaned = MeshSeal.dropDegenerateFaces(m);
    if (!MeshSeal.isClosedSolid(m) || MeshSeal.isClosedSolid(cleaned))
      m = cleaned;
    if (MeshSeal.isClosedSolid(m)) {
      // A clean primitive can share verts. A prior CSG result is already a
      // T-split annulus — welding Steiner points on the circle opens it.
      final welded = MeshSeal.dropDegenerateFaces(
        MeshEdit.weld(m, epsilon: 1e-7),
      );
      if (MeshSeal.isClosedSolid(welded)) m = welded;
      if (MeshEdit.signedVolume(m) < 0) m = MeshEdit.invert(m);
      return m;
    }
    return MeshSeal.preserveClosed(m);
  }
}

/// Keep a fragment when the boolean occupancy changes across it.
///
/// The fragment is assumed to still lie on its *own* solid (inOwn = true on
/// the −normal side). We only ray-cast the other solid. Two probes fail when:
/// the centroid is on a T-split crack, both rays graze edges, or the
/// fragment is coplanar with the other surface (both probes land in the
/// same occupancy). Union/intersect therefore have a special case that
/// keeps A's copy of a coplanar outer wall. Subtract has no dual — a
/// sketch cap sitting on a face is why extrude-cut pads 0.4 mm off-plane.
bool _onResult(
  int op, {
  required bool fromA,
  required _Poly p,
  required MeshData other,
}) {
  final inOtherInt = _contains(
    other,
    _centroid(p) - p.normal * MeshBoolean._probe,
  );
  final inOtherExt = _contains(
    other,
    _centroid(p) + p.normal * MeshBoolean._probe,
  );
  // Coplanar outer walls (both solids under this face) would otherwise
  // keep two copies.
  if ((op == MeshBoolean.unionOp || op == MeshBoolean.intersectOp) &&
      inOtherInt &&
      !inOtherExt) {
    return fromA;
  }
  bool occ(bool inOwn, bool inOther) {
    switch (op) {
      case MeshBoolean.subtractOp:
        return fromA ? inOwn && !inOther : inOther && !inOwn;
      case MeshBoolean.intersectOp:
        return inOwn && inOther;
      default:
        return inOwn || inOther;
    }
  }

  return occ(true, inOtherInt) != occ(false, inOtherExt);
}

Vector3 _centroid(_Poly p) {
  var c = Vector3.zero();
  for (final v in p.verts) {
    c += v;
  }
  return c / p.verts.length.toDouble();
}

List<_Poly> _polys(MeshData mesh, {int idOffset = 0}) {
  final ids = mesh.faceIds ?? MeshFaceRegions.originalFaceIds(mesh);
  final out = <_Poly>[];
  for (var fi = 0; fi < mesh.faces.length; fi++) {
    final f = mesh.faces[fi];
    if (f.length < 3) continue;
    final id = fi < ids.length ? ids[fi] + idOffset : idOffset;
    final p = _Poly([
      mesh.vertices[f[0]].clone(),
      mesh.vertices[f[1]].clone(),
      mesh.vertices[f[2]].clone(),
    ], faceId: id);
    if (p.ok) out.add(p);
  }
  return out;
}

int _maxFaceId(List<_Poly> polys) {
  var m = -1;
  for (final p in polys) {
    if (p.faceId > m) m = p.faceId;
  }
  return m;
}

/// Choose infinite-plane clip unless *both* soups are dense curved
/// tessellations (UV-sphere, fillet). A cube ∪ sphere must keep the
/// plane path — bounded splits of a cube face by sphere chords were
/// what opened that solid last time.
List<_Poly> _splitBounded(List<_Poly> src, List<_Poly> cutters) {
  final srcGroups = _groupByPlane(src);
  final cutterGroups = _groupByPlane(cutters);
  IjoLog.trace(
    'csg split uniquePlanes src=${srcGroups.length}/${src.length} '
    'cut=${cutterGroups.length}/${cutters.length}',
  );
  // Cube, prism, 32-gon, dense C: keep the infinite-plane path. Only two
  // heavy curved tessellations (UV-sphere ∪ fillet, both hundreds of
  // unique planes) take the mixed split.
  if (srcGroups.length <= 200 || cutterGroups.length <= 200) {
    return _splitByPlaneGroups(src, cutterGroups);
  }
  // Dense cutters: keep infinite-plane clip on tessellated CAD faces
  // (a cube quad, the still-planar wall of a filleted box). Isolated
  // curved facets get a bounded triangle split so one sphere triangle
  // cannot slice the whole fillet.
  final planeSrc = <_Poly>[];
  final curvedSrc = <_Poly>[];
  for (final g in srcGroups) {
    if (g.length >= 2) {
      planeSrc.addAll(g);
    } else {
      curvedSrc.addAll(g);
    }
  }
  IjoLog.trace(
    'csg split path=mixed planar=${planeSrc.length} curved=${curvedSrc.length}',
  );
  final out = <_Poly>[];
  if (planeSrc.isNotEmpty) {
    out.addAll(_splitByPlaneGroups(planeSrc, cutterGroups));
  }
  if (curvedSrc.isNotEmpty) {
    out.addAll(_splitByTriangles(curvedSrc, cutters));
  }
  return out;
}

/// Clip [src] by the supporting planes of [groups].
///
/// Each clip is an *infinite* plane. We only apply a plane if some triangle
/// on it actually intersects the polygon ([_hitsGroup]); otherwise a cube
/// face would slice every unrelated wall in the scene. Coplanar triangles
/// share one plane so a tessellated face is not restabbed N times (that
/// exploded slivers until the isolate timed out).
List<_Poly> _splitByPlaneGroups(List<_Poly> src, List<List<_Poly>> groups) {
  var cur = [for (final p in src) p.clone()];
  for (final g in groups) {
    final pl = g.first;
    if (!pl.ok) continue;
    final next = <_Poly>[];
    for (final p in cur) {
      if (!_hitsGroup(g, p)) {
        next.add(p);
        continue;
      }
      final coplanar = <_Poly>[];
      final front = <_Poly>[];
      final back = <_Poly>[];
      _split(pl, p, coplanar, coplanar, front, back);
      next.addAll(coplanar);
      next.addAll(front);
      next.addAll(back);
    }
    cur = next;
  }
  return cur;
}

/// Insert the finite intersection segment of each overlapping cutter
/// triangle, then rebuild. New vertices are appended — never inserted into
/// the existing ring — so a second endpoint never sees a stale index.
List<_Poly> _splitByTriangles(List<_Poly> src, List<_Poly> cutters) {
  final boxes = [for (final c in cutters) _polyBox(c)];
  final out = <_Poly>[];
  for (final s in src) {
    if (!s.ok) {
      out.add(s);
      continue;
    }
    final sb = _polyBox(s);
    var parts = [s.clone()];
    for (var i = 0; i < cutters.length; i++) {
      if (!_boxOverlap(sb, boxes[i], MeshBoolean._eps)) continue;
      final c = cutters[i];
      if (!c.ok || !_polysIntersect(s, c)) continue;
      final segs = <(Vector3, Vector3)>[];
      for (final ta in _fan(s)) {
        for (final tb in _fan(c)) {
          final seg = _triTriSegment(ta, tb);
          if (seg != null) segs.add(seg);
        }
      }
      if (segs.isEmpty) {
        final next = <_Poly>[];
        for (final p in parts) {
          if (!_polysIntersect(p, c)) {
            next.add(p);
            continue;
          }
          final coplanar = <_Poly>[];
          final front = <_Poly>[];
          final back = <_Poly>[];
          _split(c, p, coplanar, coplanar, front, back);
          next
            ..addAll(coplanar)
            ..addAll(front)
            ..addAll(back);
        }
        parts = next;
        continue;
      }
      for (final seg in segs) {
        final next = <_Poly>[];
        for (final p in parts) {
          next.addAll(_splitPolyByOneSegment(p, seg.$1, seg.$2));
        }
        parts = next;
      }
    }
    out.addAll(parts);
  }
  return out;
}

/// Split a convex polygon along [p]–[q] when both endpoints lie on the
/// boundary after clipping. Vertices are copied, then new points are
/// appended; ring indices 0..n-1 stay the original corners.
List<_Poly> _splitPolyByOneSegment(_Poly poly, Vector3 p, Vector3 q) {
  if (!poly.ok) return [poly];
  final clipped = _clipSegToConvex(poly, p, q);
  if (clipped == null) return [poly];
  final a = clipped.$1;
  final b = clipped.$2;
  if ((a - b).length2 < 1e-16) return [poly];

  final n = poly.verts.length;
  final eps2 = math.max(1e-16, MeshBoolean._eps * MeshBoolean._eps * 16);

  int? vertexOf(Vector3 x) {
    for (var i = 0; i < n; i++) {
      if ((poly.verts[i] - x).length2 <= eps2) return i;
    }
    return null;
  }

  int? edgeOf(Vector3 x) {
    for (var i = 0; i < n; i++) {
      if (_onSeg(poly.verts[i], poly.verts[(i + 1) % n], x, eps2)) return i;
    }
    return null;
  }

  final va = vertexOf(a);
  final vb = vertexOf(b);
  if (va != null && vb != null) {
    final d = (vb - va).abs();
    if (d == 1 || d == n - 1) return [poly];
    return _splitRing(poly, va, vb);
  }

  final ea = va != null ? null : edgeOf(a);
  final eb = vb != null ? null : edgeOf(b);
  if (va == null && ea == null) return [poly];
  if (vb == null && eb == null) return [poly];
  if (ea != null && ea == eb) return [poly];

  // Append both new points before any ring rewrite so [ia]/[ib] stay valid.
  final verts = [for (final v in poly.verts) v.clone()];
  final ia = va ?? verts.length;
  if (va == null) verts.add(a.clone());
  final ib = vb ?? verts.length;
  if (vb == null) verts.add(b.clone());

  final ring = <int>[];
  for (var e = 0; e < n; e++) {
    ring.add(e);
    if (ea == e) ring.add(ia);
    if (eb == e) ring.add(ib);
  }
  return _splitRingVerts(verts, ring, ia, ib, poly.faceId, poly.normal);
}

List<_Poly> _splitRing(_Poly poly, int ia, int ib) {
  final verts = [for (final v in poly.verts) v.clone()];
  final ring = [for (var i = 0; i < verts.length; i++) i];
  return _splitRingVerts(verts, ring, ia, ib, poly.faceId, poly.normal);
}

List<_Poly> _splitRingVerts(
  List<Vector3> verts,
  List<int> ring,
  int ia,
  int ib,
  int faceId,
  Vector3 normal,
) {
  final i0 = ring.indexOf(ia);
  final i1 = ring.indexOf(ib);
  if (i0 < 0 || i1 < 0 || i0 == i1) {
    return [_Poly(verts, faceId: faceId)];
  }
  List<int> walk(int from, int to) {
    final out = <int>[];
    var i = from;
    while (true) {
      out.add(ring[i]);
      if (i == to) break;
      i = (i + 1) % ring.length;
      if (out.length > ring.length) return const [];
    }
    return out;
  }

  final left = walk(i0, i1);
  final right = walk(i1, i0);
  final out = <_Poly>[];
  void emit(List<int> loop) {
    if (loop.length < 3) return;
    final p = _Poly([for (final i in loop) verts[i].clone()], faceId: faceId);
    if (p.ok) {
      if (p.normal.dot(normal) < 0) p.flip();
      out.add(p);
    }
  }

  emit(left);
  emit(right);
  return out.isEmpty ? [_Poly(verts, faceId: faceId)] : out;
}

(Vector3, Vector3)? _clipSegToConvex(_Poly poly, Vector3 p, Vector3 q) {
  final nHat = poly.normal;
  var u = nHat.cross(Vector3(1, 0, 0));
  if (u.length2 < 1e-12) u = nHat.cross(Vector3(0, 1, 0));
  u.normalize();
  final v = nHat.cross(u).normalized();
  final o = poly.verts.first;
  Vector2 to2(Vector3 x) {
    final d = x - o;
    return Vector2(d.dot(u), d.dot(v));
  }

  Vector3 to3(Vector2 x) => o + u * x.x + v * x.y;

  var a = to2(p);
  var b = to2(q);
  var t0 = 0.0;
  var t1 = 1.0;
  final dir = b - a;
  final nv = poly.verts.length;
  for (var i = 0; i < nv; i++) {
    final e0 = to2(poly.verts[i]);
    final e1 = to2(poly.verts[(i + 1) % nv]);
    final edge = e1 - e0;
    final inward = Vector2(-edge.y, edge.x);
    if (inward.length2 < 1e-20) continue;
    final denom = inward.dot(dir);
    final dist = inward.dot(a - e0);
    if (denom.abs() < 1e-14) {
      if (dist < -1e-9) return null;
      continue;
    }
    final t = -dist / denom;
    if (denom > 0) {
      if (t > t0) t0 = t;
    } else {
      if (t < t1) t1 = t;
    }
    if (t0 > t1 + 1e-9) return null;
  }
  if (t1 - t0 < 1e-9) return null;
  return (to3(a + dir * t0), to3(a + dir * t1));
}

bool _onSeg(Vector3 a, Vector3 b, Vector3 p, double eps2) {
  final ab = b - a;
  final ap = p - a;
  final len2 = ab.length2;
  if (len2 < 1e-20) return ap.length2 <= eps2;
  final t = ap.dot(ab) / len2;
  if (t < -1e-6 || t > 1 + 1e-6) return false;
  final proj = a + ab * t.clamp(0.0, 1.0);
  return (proj - p).length2 <= math.max(eps2, len2 * 1e-16);
}

_Aabb _polyBox(_Poly p) {
  var min = Vector3(1e300, 1e300, 1e300);
  var max = Vector3(-1e300, -1e300, -1e300);
  for (final v in p.verts) {
    min = Vector3(
      math.min(min.x, v.x),
      math.min(min.y, v.y),
      math.min(min.z, v.z),
    );
    max = Vector3(
      math.max(max.x, v.x),
      math.max(max.y, v.y),
      math.max(max.z, v.z),
    );
  }
  return _Aabb(min, max);
}

bool _boxOverlap(_Aabb a, _Aabb b, double pad) {
  return a.min.x <= b.max.x + pad &&
      a.max.x >= b.min.x - pad &&
      a.min.y <= b.max.y + pad &&
      a.max.y >= b.min.y - pad &&
      a.min.z <= b.max.z + pad &&
      a.max.z >= b.min.z - pad;
}

(Vector3, Vector3)? _triTriSegment(List<Vector3> a, List<Vector3> b) {
  final pts = <Vector3>[];
  void add(Vector3 p) {
    for (final q in pts) {
      if ((q - p).length2 < 1e-16) return;
    }
    pts.add(p);
  }

  for (var i = 0; i < 3; i++) {
    final ha = _segmentTriHitPoint(a[i], a[(i + 1) % 3], b[0], b[1], b[2]);
    if (ha != null) add(ha);
    final hb = _segmentTriHitPoint(b[i], b[(i + 1) % 3], a[0], a[1], a[2]);
    if (hb != null) add(hb);
    if (_pointOnTri(a[i], b[0], b[1], b[2])) add(a[i]);
    if (_pointOnTri(b[i], a[0], a[1], a[2])) add(b[i]);
  }
  if (pts.length < 2) return null;
  if (pts.length == 2) return (pts[0], pts[1]);
  var best = (pts[0], pts[1]);
  var bestD = (pts[0] - pts[1]).length2;
  for (var i = 0; i < pts.length; i++) {
    for (var j = i + 1; j < pts.length; j++) {
      final d = (pts[i] - pts[j]).length2;
      if (d > bestD) {
        bestD = d;
        best = (pts[i], pts[j]);
      }
    }
  }
  return bestD < 1e-16 ? null : best;
}

Vector3? _segmentTriHitPoint(
  Vector3 p0,
  Vector3 p1,
  Vector3 a,
  Vector3 b,
  Vector3 c,
) {
  final d = p1 - p0;
  if (d.length2 < 1e-20) {
    return _pointOnTri(p0, a, b, c) ? p0 : null;
  }
  final n = (b - a).cross(c - a);
  if (n.length2 < 1e-20) return null;
  n.normalize();
  final h0 = n.dot(p0 - a);
  final h1 = n.dot(p1 - a);
  const eps = MeshBoolean._eps;
  if (h0.abs() <= eps && h1.abs() <= eps) return null;
  final hit = _rayTri(p0, d, a, b, c);
  if (hit != null && hit.$1 >= -1e-6 && hit.$1 <= 1 + 1e-6) {
    return p0 + d * hit.$1;
  }
  if (h0.abs() <= eps && _pointOnTri(p0, a, b, c)) return p0;
  if (h1.abs() <= eps && _pointOnTri(p1, a, b, c)) return p1;
  return null;
}

List<List<_Poly>> _groupByPlane(List<_Poly> planes) {
  // Hash first — a sphere/fillet tessellation is one plane per triangle,
  // and a linear scan was O(n²) (~50M tests on a 7k-face body).
  final buckets = <String, List<_Poly>>{};
  for (final p in planes) {
    if (!p.ok) continue;
    (buckets[_planeKey(p)] ??= []).add(p);
  }
  final groups = <List<_Poly>>[];
  for (final bucket in buckets.values) {
    final local = <List<_Poly>>[];
    for (final p in bucket) {
      var found = false;
      for (final g in local) {
        if (_samePlane(g.first, p)) {
          g.add(p);
          found = true;
          break;
        }
      }
      if (!found) local.add([p]);
    }
    groups.addAll(local);
  }
  return groups;
}

String _planeKey(_Poly p) {
  var nx = p.normal.x, ny = p.normal.y, nz = p.normal.z, w = p.w;
  if (nz < -1e-9 ||
      (nz.abs() < 1e-9 && ny < -1e-9) ||
      (nz.abs() < 1e-9 && ny.abs() < 1e-9 && nx < 0)) {
    nx = -nx;
    ny = -ny;
    nz = -nz;
    w = -w;
  }
  const q = 2000.0;
  return '${(nx * q).round()}_${(ny * q).round()}_${(nz * q).round()}_${(w * q).round()}';
}

bool _samePlane(_Poly a, _Poly b) {
  // |n·n| > 0.999999 is ~0.08°. Cylinder facets at 32-gon are 11° and
  // must *not* group. Faces from a previous CSG that drifted more than
  // this become many groups and get clipped repeatedly.
  final d = a.normal.dot(b.normal);
  if (d.abs() < 0.999999) return false;
  for (final v in b.verts) {
    if ((a.normal.dot(v) - a.w).abs() > MeshBoolean._eps * 2) {
      return false;
    }
  }
  return true;
}

bool _hitsGroup(List<_Poly> group, _Poly poly) {
  for (final t in group) {
    if (_polysIntersect(t, poly)) return true;
  }
  return false;
}

bool _polysIntersect(_Poly a, _Poly b) {
  if (!_polyAabbOverlap(a, b, MeshBoolean._eps)) return false;
  final fa = _fan(a);
  final fb = _fan(b);
  for (final t in fa) {
    for (final u in fb) {
      if (_triTri(t, u)) return true;
    }
  }
  return false;
}

bool _polyAabbOverlap(_Poly a, _Poly b, double pad) {
  var amin = Vector3(1e300, 1e300, 1e300);
  var amax = Vector3(-1e300, -1e300, -1e300);
  var bmin = Vector3(1e300, 1e300, 1e300);
  var bmax = Vector3(-1e300, -1e300, -1e300);
  for (final v in a.verts) {
    amin = Vector3(
      math.min(amin.x, v.x),
      math.min(amin.y, v.y),
      math.min(amin.z, v.z),
    );
    amax = Vector3(
      math.max(amax.x, v.x),
      math.max(amax.y, v.y),
      math.max(amax.z, v.z),
    );
  }
  for (final v in b.verts) {
    bmin = Vector3(
      math.min(bmin.x, v.x),
      math.min(bmin.y, v.y),
      math.min(bmin.z, v.z),
    );
    bmax = Vector3(
      math.max(bmax.x, v.x),
      math.max(bmax.y, v.y),
      math.max(bmax.z, v.z),
    );
  }
  return amin.x <= bmax.x + pad &&
      amax.x >= bmin.x - pad &&
      amin.y <= bmax.y + pad &&
      amax.y >= bmin.y - pad &&
      amin.z <= bmax.z + pad &&
      amax.z >= bmin.z - pad;
}

List<List<Vector3>> _fan(_Poly p) {
  if (p.verts.length < 3) return const [];
  if (p.verts.length == 3) return [p.verts];
  return [
    for (var i = 1; i + 1 < p.verts.length; i++)
      [p.verts[0], p.verts[i], p.verts[i + 1]],
  ];
}

bool _triTri(List<Vector3> a, List<Vector3> b) {
  // Standard: if two triangles pierce, an edge of one hits the other.
  // Misses a coplanar triangle strictly inside another (no edge crosses).
  // Those stay unsplit; classification then sees a coplanar pair. That is
  // the sketch-on-face case — pad the cutter off the plane rather than
  // relying on this test to carve the overlap polygon.
  for (var i = 0; i < 3; i++) {
    if (_segmentHitsTri(a[i], a[(i + 1) % 3], b[0], b[1], b[2])) return true;
    if (_segmentHitsTri(b[i], b[(i + 1) % 3], a[0], a[1], a[2])) return true;
  }
  return false;
}

bool _segmentHitsTri(Vector3 p0, Vector3 p1, Vector3 a, Vector3 b, Vector3 c) {
  final d = p1 - p0;
  if (d.length2 < 1e-20) return _pointOnTri(p0, a, b, c);
  final n = (b - a).cross(c - a);
  if (n.length2 < 1e-20) return false;
  n.normalize();
  final h0 = n.dot(p0 - a);
  final h1 = n.dot(p1 - a);
  if (h0.abs() <= MeshBoolean._eps && h1.abs() <= MeshBoolean._eps) {
    return _pointInTriInterior((p0 + p1) * 0.5, a, b, c);
  }
  final hit = _rayTri(p0, d, a, b, c);
  if (hit == null) return false;
  return hit.$1 >= -1e-6 && hit.$1 <= 1 + 1e-6;
}

bool _pointOnTri(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
  return _bary(p, a, b, c) != null;
}

/// Strict interior (not on an edge). Shared boundary edges must not trigger
/// an infinite-plane cut, or a cylinder wall fans the top cap.
bool _pointInTriInterior(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
  final bary = _bary(p, a, b, c);
  if (bary == null) return false;
  const m = 1e-4;
  return bary.$1 > m && bary.$2 > m && bary.$1 + bary.$2 < 1 - m;
}

(double u, double v)? _bary(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
  final n = (b - a).cross(c - a);
  if (n.length2 < 1e-20) return null;
  n.normalize();
  if ((n.dot(p - a)).abs() > MeshBoolean._eps) return null;
  final d00 = (b - a).dot(b - a);
  final d01 = (b - a).dot(c - a);
  final d11 = (c - a).dot(c - a);
  final d20 = (p - a).dot(b - a);
  final d21 = (p - a).dot(c - a);
  final den = d00 * d11 - d01 * d01;
  if (den.abs() < 1e-20) return null;
  final v = (d11 * d20 - d01 * d21) / den;
  final w = (d00 * d21 - d01 * d20) / den;
  final u = 1 - v - w;
  if (u < -1e-6 || v < -1e-6 || w < -1e-6) return null;
  return (v, w);
}

MeshData _meshFrom(List<_Poly> polys, {bool seal = true}) {
  IjoLog.trace('csg meshFrom polys=${polys.length}');
  final verts = <Vector3>[];
  final faces = <List<int>>[];
  final faceIds = <int>[];
  for (final p in polys) {
    if (p.verts.length < 3) continue;
    final base = verts.length;
    verts.addAll(p.verts);
    // Finite intersection splits can produce concave fragments. Triangulate
    // their actual boundary instead of overlapping it with a vertex fan.
    for (final t in MeshPolygon.triangulateRings(
      p.verts,
      [List.generate(p.verts.length, (i) => i)],
      p.normal,
      keepCollinear: true,
    )) {
      faces.add([for (final i in t) base + i]);
      faceIds.add(p.faceId);
    }
  }
  var out = MeshData(vertices: verts, faces: faces, faceIds: faceIds);
  IjoLog.trace(
    'csg meshFrom soup ${IjoLog.size(out.vertices.length, out.faces.length)} '
    'open=${MeshSeal.openBoundaryEdgeCount(out)}',
  );
  // Drop junk on the soup, then T-split LAST. snapPlanarVertices /
  // dropDegenerate / unifyCoplanarFaces / sealSolid-cap after a closed
  // T-split reopen the mesh (T-stitches are slivers) and ear-clip the
  // fake holes — a one-triangle cube side in the viewport.
  // Snapping slivers inside the boolean breaks a closed shell.
  // Interior Steiner cuts are hidden by originalFaceIds, not remeshed.
  out = MeshEdit.weld(out, epsilon: _weldEps(out));
  out = MeshSeal.dropDegenerateFaces(out);
  out = MeshEdit.dropDuplicateFaces(out);
  IjoLog.trace(
    'csg meshFrom weld ${IjoLog.size(out.vertices.length, out.faces.length)} '
    'open=${MeshSeal.openBoundaryEdgeCount(out)}',
  );
  if (seal) out = _stitchUntilClosed(out);
  IjoLog.trace(
    'csg meshFrom stitch ${IjoLog.size(out.vertices.length, out.faces.length)} '
    'closed=${MeshSeal.isClosedSolid(out)} '
    'open=${MeshSeal.openBoundaryEdgeCount(out)} '
    'seal=$seal',
  );
  return out;
}

/// T-split while the open-edge count falls.
///
/// Plane clips put Steiner vertices in the middle of edges. The opposite
/// triangle still uses the unsplit edge → 1-use + 2 half-edges = a hole
/// that is only a T-junction. Splitting that edge restores the 2-manifold.
///
/// If the count stops falling, a polygon was dropped (classifier) or a
/// concave fan made a real hole. More iterations will not grow a face.
/// Six passes is a ceiling, not a strategy — the 31-gon-through-C case
/// needed three; a missing wall will still be open after six.
MeshData _stitchUntilClosed(MeshData mesh) {
  var out = mesh;
  var open = MeshSeal.openBoundaryEdgeCount(out);
  if (open == 0) return out;
  for (var i = 0; i < 6; i++) {
    out = MeshSeal.stitchTJunctions(out, epsilon: 1e-7);
    final next = MeshSeal.openBoundaryEdgeCount(out);
    if (next == 0) return out;
    if (i > 0 && next >= open) break;
    open = next;
  }
  return out;
}

bool _contains(MeshData mesh, Vector3 point) {
  // Odd-even ray test, several directions so a graze on an edge can retry.
  // All six grazing → "outside". Faceted cylinders and thin walls do that
  // often; the fragment is then dropped and stitch cannot put it back.
  // Winding-number classify is the alternative to rays.
  const dirs = <List<double>>[
    [1.0, 0.137, 0.061],
    [0.061, 1.0, 0.137],
    [0.137, 0.061, 1.0],
    [-1.0, 0.137, 0.061],
    [0.061, -1.0, 0.137],
    [0.137, 0.061, -1.0],
  ];
  for (final d in dirs) {
    final dir = Vector3(d[0], d[1], d[2])..normalize();
    final hits = _rayHits(mesh, point, dir);
    if (hits != null) return hits.isOdd;
  }
  return false;
}

int? _rayHits(MeshData mesh, Vector3 origin, Vector3 direction) {
  const epsilon = 1e-8;
  const edgeEps = 1e-5;
  var count = 0;
  for (final face in mesh.faces) {
    if (face.length < 3) continue;
    for (var tri = 1; tri < face.length - 1; tri++) {
      final a = mesh.vertices[face[0]];
      final b = mesh.vertices[face[tri]];
      final c = mesh.vertices[face[tri + 1]];
      final hit = _rayTri(origin, direction, a, b, c);
      if (hit == null) continue;
      final (t, u, v) = hit;
      if (t <= epsilon) continue;
      if (u < edgeEps || v < edgeEps || u + v > 1 - edgeEps) return null;
      count++;
    }
  }
  return count;
}

(double t, double u, double v)? _rayTri(
  Vector3 origin,
  Vector3 direction,
  Vector3 a,
  Vector3 b,
  Vector3 c,
) {
  const epsilon = 1e-8;
  final edge1 = b - a;
  final edge2 = c - a;
  final pvec = direction.cross(edge2);
  final det = edge1.dot(pvec);
  if (det.abs() < epsilon) return null;
  final invDet = 1 / det;
  final tvec = origin - a;
  final u = tvec.dot(pvec) * invDet;
  if (u < 0 || u > 1) return null;
  final qvec = tvec.cross(edge1);
  final v = direction.dot(qvec) * invDet;
  if (v < 0 || u + v > 1) return null;
  final t = edge2.dot(qvec) * invDet;
  if (t <= epsilon) return null;
  return (t, u, v);
}

class _Poly {
  _Poly(this.verts, {this.faceId = 0}) {
    var area = Vector3.zero();
    for (var i = 1; i + 1 < verts.length; i++) {
      area += (verts[i] - verts[0]).cross(verts[i + 1] - verts[0]);
    }
    normal = area.length2 > 1e-20 ? area.normalized() : Vector3.zero();
    w = verts.isEmpty ? 0 : normal.dot(verts.first);
  }

  List<Vector3> verts;
  int faceId;
  late Vector3 normal;
  late double w;

  bool get ok => verts.length >= 3 && normal.length2 > 0.5;

  _Poly clone() => _Poly([for (final v in verts) v.clone()], faceId: faceId);

  void flip() {
    verts = verts.reversed.map((v) => v.clone()).toList();
    normal = -normal;
    w = -w;
  }
}

// CSG.js attribution: this helper's classification flags, output buckets and
// spanning-edge traversal closely follow CSG.Plane.splitPolygon:
// https://github.com/evanw/csg.js/blob/master/csg.js
// Pali Ijo adds denominator guards, clamped intersections and face IDs.
// The closed-solid BSP fallback below follows CSG.js's clip/invert/build
// Boolean sequence; the primary path uses fragment classification.
void _split(
  _Poly plane,
  _Poly poly,
  List<_Poly> coplanarFront,
  List<_Poly> coplanarBack,
  List<_Poly> front,
  List<_Poly> back,
) {
  // Classify each vertex as front / back / on-plane, then emit the
  // intersection points along spanning edges. Output polygons are not
  // retriangulated here; [_meshFrom] triangulates the retained boundary.
  const coplanar = 0, frontBit = 1, backBit = 2, spanning = 3;
  var type = 0;
  final types = <int>[];
  for (final v in poly.verts) {
    final t = plane.normal.dot(v) - plane.w;
    final loc = t < -MeshBoolean._eps
        ? backBit
        : t > MeshBoolean._eps
        ? frontBit
        : coplanar;
    type |= loc;
    types.add(loc);
  }
  if (type == coplanar) {
    (plane.normal.dot(poly.normal) > 0 ? coplanarFront : coplanarBack).add(
      poly,
    );
    return;
  }
  if (type == frontBit) {
    front.add(poly);
    return;
  }
  if (type == backBit) {
    back.add(poly);
    return;
  }
  final fv = <Vector3>[];
  final bv = <Vector3>[];
  final n = poly.verts.length;
  for (var i = 0; i < n; i++) {
    final j = (i + 1) % n;
    final ti = types[i];
    final tj = types[j];
    final vi = poly.verts[i];
    final vj = poly.verts[j];
    if (ti != backBit) fv.add(vi.clone());
    if (ti != frontBit) bv.add(vi.clone());
    if ((ti | tj) == spanning) {
      final edge = vj - vi;
      final denom = plane.normal.dot(edge);
      final len = edge.length;
      if (denom.abs() < 1e-12 * math.max(len, 1e-12)) {
        continue;
      }
      final t = ((plane.w - plane.normal.dot(vi)) / denom).clamp(0.0, 1.0);
      final v = vi + edge * t;
      fv.add(v.clone());
      bv.add(v.clone());
    }
  }
  if (fv.length >= 3) front.add(_Poly(fv, faceId: poly.faceId));
  if (bv.length >= 3) back.add(_Poly(bv, faceId: poly.faceId));
}

double _weldEps(MeshData mesh) {
  // Scale with the solid so a 1 m part still welds T-splits, but never
  // collapse a 0.05 mm CAD edge. Large scenes therefore weld more than
  // small ones — a second place numeric tolerance is not invariant.
  final box = _aabb(mesh);
  final dx = box.max.x - box.min.x;
  final dy = box.max.y - box.min.y;
  final dz = box.max.z - box.min.z;
  final diag = math.sqrt(dx * dx + dy * dy + dz * dz);
  return (diag * 1e-9).clamp(1e-7, 1e-5);
}

class _Aabb {
  const _Aabb(this.min, this.max);
  final Vector3 min;
  final Vector3 max;
}

_Aabb _aabb(MeshData mesh) {
  var min = Vector3(1e300, 1e300, 1e300);
  var max = Vector3(-1e300, -1e300, -1e300);
  for (final v in mesh.vertices) {
    min = Vector3(
      math.min(min.x, v.x),
      math.min(min.y, v.y),
      math.min(min.z, v.z),
    );
    max = Vector3(
      math.max(max.x, v.x),
      math.max(max.y, v.y),
      math.max(max.z, v.z),
    );
  }
  return _Aabb(min, max);
}

bool _aabbOverlap(_Aabb a, _Aabb b, {double pad = 0}) {
  return a.min.x <= b.max.x + pad &&
      a.max.x >= b.min.x - pad &&
      a.min.y <= b.max.y + pad &&
      a.max.y >= b.min.y - pad &&
      a.min.z <= b.max.z + pad &&
      a.max.z >= b.min.z - pad;
}

/// A solid's oriented planes partition space into inside and outside cells.
class _Partition {
  _Partition(List<_Poly> faces) {
    build(faces);
  }
  _Poly? plane;
  final polygons = <_Poly>[];
  _Partition? front, back;
  void build(List<_Poly> faces) {
    if (faces.isEmpty) return;
    plane ??= faces.first.clone();
    final outside = <_Poly>[], inside = <_Poly>[];
    for (final face in faces)
      _split(plane!, face, polygons, polygons, outside, inside);
    if (outside.isNotEmpty) (front ??= _Partition([])).build(outside);
    if (inside.isNotEmpty) (back ??= _Partition([])).build(inside);
  }

  List<_Poly> all() => [...polygons, ...?front?.all(), ...?back?.all()];
  void invert() {
    plane?.flip();
    for (final p in polygons) p.flip();
    front?.invert();
    back?.invert();
    final swap = front;
    front = back;
    back = swap;
  }

  List<_Poly> clip(List<_Poly> faces) {
    if (plane == null) return faces;
    var outside = <_Poly>[], inside = <_Poly>[];
    for (final face in faces)
      _split(plane!, face, outside, inside, outside, inside);
    if (front != null) outside = front!.clip(outside);
    inside = back?.clip(inside) ?? [];
    return [...outside, ...inside];
  }

  void clipTo(_Partition other) {
    final kept = other.clip(polygons);
    polygons
      ..clear()
      ..addAll(kept);
    front?.clipTo(other);
    back?.clipTo(other);
  }
}
