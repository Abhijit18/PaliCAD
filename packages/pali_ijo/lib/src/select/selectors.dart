import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../measure/bounds.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_regions.dart';
import '../ops/edge_geometry.dart';

/// Unique CAD faces of [mesh]: coplanar / faceId regions, one seed triangle
/// each.
List<List<int>> meshCadFaces(MeshData mesh) {
  final seen = <int>{};
  final out = <List<int>>[];
  for (var i = 0; i < mesh.faces.length; i++) {
    if (!seen.add(i)) continue;
    final region = MeshFaceRegions.selectionRegion(mesh, i);
    if (region.isEmpty) continue;
    for (final t in region) {
      seen.add(t);
    }
    out.add(region);
  }
  return out;
}

Vector3? _regionNormal(MeshData mesh, List<int> region) {
  var n = Vector3.zero();
  for (final fi in region) {
    final fn = EdgeGeometry.faceNormal(mesh, fi);
    if (fn != null) n += fn;
  }
  if (n.length2 < 1e-20) return null;
  return n.normalized();
}

Vector3 _regionCentroid(MeshData mesh, List<int> region) {
  var c = Vector3.zero();
  var n = 0;
  for (final fi in region) {
    final f = mesh.faces[fi];
    if (f.length < 3) continue;
    c += (mesh.vertices[f[0]] + mesh.vertices[f[1]] + mesh.vertices[f[2]]) / 3;
    n++;
  }
  return n == 0 ? Vector3.zero() : c / n.toDouble();
}

bool _regionPlanar(MeshData mesh, List<int> region) {
  final n = _regionNormal(mesh, region);
  if (n == null) return false;
  for (final fi in region) {
    final fn = EdgeGeometry.faceNormal(mesh, fi);
    if (fn == null || fn.dot(n) < 0.98) return false;
  }
  return true;
}

/// A predicate over CAD faces (coplanar / faceId regions).
abstract class FaceSelect {
  const FaceSelect();

  bool matchesRegion(MeshData mesh, List<int> region, int seed);

  /// Seed triangle index of each matching CAD face.
  List<int> resolve(MeshData mesh) => [
        for (final region in meshCadFaces(mesh))
          if (region.isNotEmpty && matchesRegion(mesh, region, region.first))
            region.first,
      ];

  static final FaceSelect all = _FacePred((_, __, ___) => true, 'all');

  static FaceSelect indices(Iterable<int> indices) {
    final set = indices.toSet();
    return _FacePred((_, __, seed) => set.contains(seed), 'indices(${set.length})');
  }

  static FaceSelect normal(Vector3 direction, {double toleranceDeg = 1.0}) {
    final d = direction.normalized();
    final cosLimit = math.cos(toleranceDeg * math.pi / 180);
    return _FacePred(
      (mesh, region, seed) {
        final n = _regionNormal(mesh, region);
        return n != null && n.dot(d) >= cosLimit;
      },
      'normal',
    );
  }

  static FaceSelect planar({Vector3? through, Vector3? normal}) {
    return _FacePred(
      (mesh, region, seed) {
        if (!_regionPlanar(mesh, region)) return false;
        if (through == null && normal == null) return true;
        final n = _regionNormal(mesh, region);
        if (n == null) return false;
        if (normal != null && n.dot(normal.normalized()).abs() < 0.9999) {
          return false;
        }
        if (through != null) {
          final c = _regionCentroid(mesh, region);
          if ((c - through).dot(n).abs() > 1e-4) return false;
        }
        return true;
      },
      'planar',
    );
  }

  static FaceSelect curved() => planar().not;

  static FaceSelect within(IjoBounds box, {double tolerance = 1e-6}) =>
      _FacePred(
        (mesh, region, seed) {
          for (final fi in region) {
            final f = mesh.faces[fi];
            for (final i in f) {
              if (!box.containsPoint(mesh.vertices[i], tolerance: tolerance)) {
                return false;
              }
            }
          }
          return region.isNotEmpty;
        },
        'within',
      );

  static FaceSelect extreme(Vector3 direction, {double tolerance = 1e-6}) {
    final d = direction.normalized();
    return _FaceExtreme(d, tolerance);
  }

  FaceSelect and(FaceSelect other) => _FacePred(
        (m, r, s) => matchesRegion(m, r, s) && other.matchesRegion(m, r, s),
        '($this and $other)',
      );

  FaceSelect or(FaceSelect other) => _FacePred(
        (m, r, s) => matchesRegion(m, r, s) || other.matchesRegion(m, r, s),
        '($this or $other)',
      );

  FaceSelect get not => _FacePred(
        (m, r, s) => !matchesRegion(m, r, s),
        'not $this',
      );

  @override
  String toString() => 'FaceSelect';
}

class _FacePred extends FaceSelect {
  _FacePred(this._fn, this._label);
  final bool Function(MeshData mesh, List<int> region, int seed) _fn;
  final String _label;
  @override
  bool matchesRegion(MeshData mesh, List<int> region, int seed) =>
      _fn(mesh, region, seed);
  @override
  String toString() => _label;
}

class _FaceExtreme extends FaceSelect {
  const _FaceExtreme(this.direction, this.tolerance);
  final Vector3 direction;
  final double tolerance;
  @override
  bool matchesRegion(MeshData mesh, List<int> region, int seed) {
    var best = -double.infinity;
    final scores = <int, double>{};
    for (final r in meshCadFaces(mesh)) {
      final s = _regionCentroid(mesh, r).dot(direction);
      scores[r.first] = s;
      if (s > best) best = s;
    }
    final mine = scores[region.first];
    return mine != null && mine >= best - tolerance;
  }

  @override
  String toString() => 'extreme';
}

/// Feature-edge selection. [resolve] returns vertex-index pairs.
abstract class EdgeSelect {
  const EdgeSelect();

  bool matches(MeshData mesh, int a, int b);

  List<(int, int)> resolve(MeshData mesh) => [
        for (final e in MeshFaceRegions.featureEdges(mesh))
          if (matches(mesh, e.$1, e.$2)) e,
      ];

  List<(Vector3, Vector3)> resolvePoints(MeshData mesh) => [
        for (final e in resolve(mesh)) (mesh.vertices[e.$1], mesh.vertices[e.$2]),
      ];

  static final EdgeSelect all = _EdgePred((_, __, ___) => true, 'all');

  static EdgeSelect indices(Iterable<int> indices) {
    final set = indices.toSet();
    var i = 0;
    return _EdgePred(
      (mesh, a, b) {
        // Feature-edge order is the index space CLI numbers refer to.
        return false;
      },
      'indices(${set.length})',
    ).._indexSet = set;
  }

  static EdgeSelect ofFace(FaceSelect faces) => _EdgesOfFace(faces);

  static EdgeSelect circular({double? radius, double tolerance = 1e-6}) =>
      _EdgePred(
        (mesh, a, b) {
          final adj = MeshFaceRegions.facesForEdge(mesh, a, b);
          if (adj.length < 2) return false;
          return !_regionPlanar(mesh, [adj[0]]) ||
              !_regionPlanar(mesh, [adj[1]]);
        },
        'circular',
      );

  static EdgeSelect linear({Vector3? direction, double toleranceDeg = 1.0}) {
    final d = direction?.normalized();
    final cosLimit = math.cos(toleranceDeg * math.pi / 180);
    return _EdgePred(
      (mesh, a, b) {
        var t = mesh.vertices[b] - mesh.vertices[a];
        if (t.length < 1e-12) return false;
        t.normalize();
        if (d == null) return true;
        return t.dot(d).abs() >= cosLimit;
      },
      'linear',
    );
  }

  static EdgeSelect convex({double toleranceDeg = 0.5}) =>
      _Convexity(wantConvex: true, toleranceDeg: toleranceDeg);

  static EdgeSelect concave({double toleranceDeg = 0.5}) =>
      _Convexity(wantConvex: false, toleranceDeg: toleranceDeg);

  static EdgeSelect longer(double length) => _EdgePred(
        (mesh, a, b) => (mesh.vertices[b] - mesh.vertices[a]).length >= length,
        'longer',
      );

  static EdgeSelect within(IjoBounds box, {double tolerance = 1e-6}) =>
      _EdgePred(
        (mesh, a, b) =>
            box.containsPoint(mesh.vertices[a], tolerance: tolerance) &&
            box.containsPoint(mesh.vertices[b], tolerance: tolerance),
        'within',
      );

  EdgeSelect and(EdgeSelect other) => _EdgePred(
        (m, a, b) => matches(m, a, b) && other.matches(m, a, b),
        '($this and $other)',
      );

  EdgeSelect or(EdgeSelect other) => _EdgePred(
        (m, a, b) => matches(m, a, b) || other.matches(m, a, b),
        '($this or $other)',
      );

  EdgeSelect get not => _EdgePred(
        (m, a, b) => !matches(m, a, b),
        'not $this',
      );

  @override
  String toString() => 'EdgeSelect';
}

class _EdgePred extends EdgeSelect {
  _EdgePred(this._fn, this._label);
  final bool Function(MeshData mesh, int a, int b) _fn;
  final String _label;
  Set<int>? _indexSet;
  @override
  bool matches(MeshData mesh, int a, int b) {
    if (_indexSet != null) {
      final all = MeshFaceRegions.featureEdges(mesh);
      for (var i = 0; i < all.length; i++) {
        if (all[i] == (a, b) || all[i] == (b, a)) {
          return _indexSet!.contains(i);
        }
      }
      return false;
    }
    return _fn(mesh, a, b);
  }

  @override
  String toString() => _label;
}

class _EdgesOfFace extends EdgeSelect {
  const _EdgesOfFace(this.faces);
  final FaceSelect faces;
  @override
  bool matches(MeshData mesh, int a, int b) {
    final seeds = faces.resolve(mesh).toSet();
    if (seeds.isEmpty) return false;
    final adj = MeshFaceRegions.facesForEdge(mesh, a, b);
    for (final fi in adj) {
      final region = MeshFaceRegions.selectionRegion(mesh, fi);
      if (region.any(seeds.contains) || seeds.contains(fi)) return true;
      if (region.isNotEmpty && seeds.contains(region.first)) return true;
    }
    return false;
  }

  @override
  String toString() => 'ofFace($faces)';
}

class _Convexity extends EdgeSelect {
  const _Convexity({required this.wantConvex, required this.toleranceDeg});
  final bool wantConvex;
  final double toleranceDeg;

  @override
  bool matches(MeshData mesh, int a, int b) {
    final adj = MeshFaceRegions.facesForEdge(mesh, a, b);
    if (adj.length < 2) return false;
    final n0 = EdgeGeometry.faceNormal(mesh, adj[0]);
    final n1 = EdgeGeometry.faceNormal(mesh, adj[1]);
    if (n0 == null || n1 == null) return false;
    final mid = (mesh.vertices[a] + mesh.vertices[b]) * 0.5;
    Vector3? third(int fi, int ea, int eb) {
      final f = mesh.faces[fi];
      for (final i in f) {
        if (i != ea && i != eb) return mesh.vertices[i];
      }
      return null;
    }

    final p0 = third(adj[0], a, b);
    final p1 = third(adj[1], a, b);
    if (p0 == null || p1 == null) return false;
    final cosLimit = math.cos(toleranceDeg * math.pi / 180);
    if (n0.dot(n1).abs() > cosLimit && n0.dot(n1) > 0) return false;
    final convex = (p0 - mid).dot(n1) < 0 && (p1 - mid).dot(n0) < 0;
    return wantConvex ? convex : !convex;
  }

  @override
  String toString() => wantConvex ? 'convex' : 'concave';
}
