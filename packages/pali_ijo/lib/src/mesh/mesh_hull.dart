import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import 'mesh_data.dart';
import 'mesh_edit.dart';
import 'mesh_seal.dart';

/// Convex hull of a point set, and a convexity test for closed meshes.
class MeshHull {
  MeshHull._();

  /// Incremental 3D convex hull. Degenerate (flat / too few points) sets
  /// yield an empty mesh — not a solid.
  static MeshData ofPoints(List<Vector3> points, {double epsilon = 1e-9}) {
    final verts = _unique(points, epsilon);
    if (verts.length < 4) {
      return MeshData(vertices: const [], faces: const []);
    }
    final seed = _seedTetra(verts, epsilon);
    if (seed == null) {
      return MeshData(vertices: const [], faces: const []);
    }
    final faces = <_Face>[
      _Face(seed[0], seed[1], seed[2], verts),
      _Face(seed[0], seed[3], seed[1], verts),
      _Face(seed[0], seed[2], seed[3], verts),
      _Face(seed[1], seed[3], seed[2], verts),
    ];
    final used = List<bool>.filled(verts.length, false);
    for (final i in seed) {
      used[i] = true;
    }
    final centroid = (verts[seed[0]] +
            verts[seed[1]] +
            verts[seed[2]] +
            verts[seed[3]]) /
        4.0;
    for (final f in faces) {
      f.orientOutward(verts, centroid);
    }

    for (var i = 0; i < verts.length; i++) {
      if (used[i]) continue;
      used[i] = true;
      final p = verts[i];
      var any = false;
      for (final f in faces) {
        f.visible = f.n.dot(p - verts[f.a]) > epsilon;
        if (f.visible) any = true;
      }
      if (!any) continue;

      final directed = <String, (int, int)>{};
      for (final f in faces) {
        if (!f.visible) continue;
        directed['${f.a}|${f.b}'] = (f.a, f.b);
        directed['${f.b}|${f.c}'] = (f.b, f.c);
        directed['${f.c}|${f.a}'] = (f.c, f.a);
      }
      faces.removeWhere((f) => f.visible);
      for (final e in directed.values) {
        if (directed.containsKey('${e.$2}|${e.$1}')) continue;
        final nf = _Face(e.$1, e.$2, i, verts);
        nf.orientOutward(verts, centroid);
        faces.add(nf);
      }
    }

    if (faces.length < 4) {
      return MeshData(vertices: const [], faces: const []);
    }
    var mesh = MeshData(
      vertices: [for (final v in verts) v.clone()],
      faces: [
        for (final f in faces) [f.a, f.b, f.c],
      ],
    );
    mesh = MeshSeal.dropDegenerateFaces(mesh);
    mesh = _usedOnly(mesh);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  /// True when every vertex lies on or behind every triangle plane.
  static bool isConvex(MeshData mesh, {double epsilon = 1e-6}) {
    final m = MeshEdit.triangulate(mesh);
    if (m.faces.length < 4) return false;
    var c = Vector3.zero();
    for (final v in m.vertices) {
      c += v;
    }
    c /= m.vertices.length.toDouble();
    for (final f in m.faces) {
      if (f.length < 3) continue;
      final a = m.vertices[f[0]];
      final b = m.vertices[f[1]];
      final d = m.vertices[f[2]];
      var n = (b - a).cross(d - a);
      if (n.length2 < 1e-20) continue;
      n.normalize();
      if (n.dot(c - a) > 0) n = -n;
      for (final v in m.vertices) {
        if (n.dot(v - a) > epsilon) return false;
      }
    }
    return true;
  }
}

class _Face {
  _Face(this.a, this.b, this.c, List<Vector3> verts)
      : n = _normal(verts[a], verts[b], verts[c]);

  int a;
  int b;
  int c;
  Vector3 n;
  bool visible = false;

  static Vector3 _normal(Vector3 a, Vector3 b, Vector3 c) {
    final n = (b - a).cross(c - a);
    final len = n.length;
    return len < 1e-18 ? Vector3(0, 0, 1) : n / len;
  }

  void orientOutward(List<Vector3> verts, Vector3 centroid) {
    if (n.dot(centroid - verts[a]) > 0) flip(verts);
  }

  void flip(List<Vector3> verts) {
    final t = b;
    b = c;
    c = t;
    n = _normal(verts[a], verts[b], verts[c]);
  }
}

List<Vector3> _unique(List<Vector3> points, double epsilon) {
  final inv = 1.0 / math.max(epsilon, 1e-12);
  final seen = <String, Vector3>{};
  for (final p in points) {
    final key =
        '${(p.x * inv).round()},${(p.y * inv).round()},${(p.z * inv).round()}';
    seen.putIfAbsent(key, () => p.clone());
  }
  return seen.values.toList();
}

MeshData _usedOnly(MeshData mesh) {
  final used = <int>{};
  for (final f in mesh.faces) {
    used.addAll(f);
  }
  if (used.length == mesh.vertices.length) return mesh;
  final map = <int, int>{};
  final verts = <Vector3>[];
  for (var i = 0; i < mesh.vertices.length; i++) {
    if (!used.contains(i)) continue;
    map[i] = verts.length;
    verts.add(mesh.vertices[i].clone());
  }
  return MeshData(
    vertices: verts,
    faces: [
      for (final f in mesh.faces) [for (final i in f) map[i]!],
    ],
  );
}

List<int>? _seedTetra(List<Vector3> verts, double epsilon) {
  var i0 = 0;
  var i1 = 0;
  var best = -1.0;
  for (var i = 0; i < verts.length; i++) {
    for (var j = i + 1; j < verts.length; j++) {
      final d = (verts[i] - verts[j]).length2;
      if (d > best) {
        best = d;
        i0 = i;
        i1 = j;
      }
    }
  }
  if (best < epsilon * epsilon) return null;
  final dir = verts[i1] - verts[i0];
  var i2 = -1;
  var bestArea = 0.0;
  for (var i = 0; i < verts.length; i++) {
    if (i == i0 || i == i1) continue;
    final area = dir.cross(verts[i] - verts[i0]).length2;
    if (area > bestArea) {
      bestArea = area;
      i2 = i;
    }
  }
  if (i2 < 0 || bestArea < epsilon * epsilon) return null;
  final n = (verts[i1] - verts[i0]).cross(verts[i2] - verts[i0]);
  var i3 = -1;
  var bestH = 0.0;
  for (var i = 0; i < verts.length; i++) {
    if (i == i0 || i == i1 || i == i2) continue;
    final h = n.dot(verts[i] - verts[i0]).abs();
    if (h > bestH) {
      bestH = h;
      i3 = i;
    }
  }
  if (i3 < 0 || bestH < epsilon) return null;
  final vol = n.dot(verts[i3] - verts[i0]);
  if (vol < 0) return [i0, i1, i2, i3];
  return [i0, i2, i1, i3];
}
