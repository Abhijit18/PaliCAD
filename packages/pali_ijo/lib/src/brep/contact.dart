import 'dart:math' as math;
import 'package:vector_math/vector_math_64.dart';
import 'geom.dart';
import 'solid.dart';
import 'tessellate.dart';

/// Pair planar sheets around a line contact by their occupied angular sectors.
/// Face connectivity alone cannot resolve a contact when the two sheets join
/// elsewhere (for example, a tongue underneath a trailer frame).
List<List<int>>? planarContactFacePairs(BrepSolid s, Map<int, int> faceEdges) {
  if (faceEdges.length < 4) return null;
  final edge = s.edges[faceEdges.values.first];
  if (edge.curve is! BrepLine || faceEdges.length.isOdd) return null;
  final axis = s.vertices[edge.v1] - s.vertices[edge.v0];
  if (axis.length2 < 1e-16) return null;
  axis.normalize();
  var ref = axis.cross(Vector3(0, 0, 1));
  if (ref.length2 < 1e-12) ref = axis.cross(Vector3(0, 1, 0));
  ref.normalize();
  final bin = axis.cross(ref);
  final rays = <(double, int, bool)>[];
  for (final fi in faceEdges.keys) {
    final eid = faceEdges[fi]!;
    final face = s.faces[fi];
    final plane = face.surface;
    if (plane is! BrepPlane) return null;
    final loops = face.loops.where((l) => l.coedges.any((c) => c.edge == eid));
    if (loops.length != 1) return null;
    final loop = loops.single;
    final coedges = loop.coedges.where((c) => c.edge == eid);
    if (coedges.length != 1) return null;
    final n = face.outwardNormal(0, 0).normalized();
    if (n.dot(axis).abs() > 1e-7) return null;
    final ownEdge = s.edges[eid];
    var tangent = (s.vertices[ownEdge.v1] - s.vertices[ownEdge.v0])
        .normalized();
    if (coedges.single.reversed) tangent = -tangent;
    final points = BrepTessellate.sampleLoop(s, loop, 32);
    var area = Vector3.zero();
    for (var i = 0; i < points.length; i++) {
      area += (points[i] - plane.origin).cross(
        points[(i + 1) % points.length] - plane.origin,
      );
    }
    if (area.dot(n).abs() < 1e-12) return null;
    if ((area.dot(n) < 0) == identical(loop, face.loops.first)) {
      tangent = -tangent;
    }
    final ray = n.cross(tangent).normalized();
    rays.add((
      math.atan2(ray.dot(bin), ray.dot(ref)),
      fi,
      n.dot(axis.cross(ray)) < 0,
    ));
  }
  rays.sort((a, b) => a.$1.compareTo(b.$1));
  final pairs = <List<int>>[];
  for (var i = 0; i < rays.length; i++) {
    final a = rays[i], b = rays[(i + 1) % rays.length];
    final gap = (b.$1 - a.$1 + 2 * math.pi) % (2 * math.pi);
    // Coincident sheets or inconsistent orientations remain an error.
    if (gap < 1e-7 || a.$3 == b.$3) return null;
    if (a.$3) pairs.add([a.$2, b.$2]);
  }
  return pairs.length * 2 == faceEdges.length ? pairs : null;
}

/// Duplicate topological line edges identify sheets touching at a line while
/// keeping separate face pairs. Rebuilding these with coplanar face merging
/// would erase that distinction.
bool hasPairedPlanarContacts(BrepSolid s) {
  final edgeFaces = <int, List<BrepFace>>{};
  for (final f in s.faces) {
    for (final l in f.loops) {
      for (final c in l.coedges) (edgeFaces[c.edge] ??= []).add(f);
    }
  }
  final seen = <(int, int)>{};
  for (final entry in edgeFaces.entries) {
    if (entry.value.length != 2 ||
        entry.value.any((f) => f.surface is! BrepPlane))
      continue;
    final e = s.edges[entry.key];
    if (e.curve is! BrepLine || e.v0 == e.v1) continue;
    final key = e.v0 < e.v1 ? (e.v0, e.v1) : (e.v1, e.v0);
    if (!seen.add(key)) return true;
  }
  return false;
}
