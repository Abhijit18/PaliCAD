import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_regions.dart';

/// Ordered closed loops that fit a circle well enough to treat as one edge.
///
/// A faceted cylinder rim is n chords of one circle. A CAD kernel fillets that as a
/// single circular edge; the mesh kernel can do the same if we detect the loop
/// and revolve one cutter instead of running n sequential CSG subtracts.
class CircularRim {
  const CircularRim({
    required this.loop,
    required this.origin,
    required this.axis,
    required this.radius,
  });

  /// Ordered unique vertices around the loop (not repeating the start).
  final List<Vector3> loop;
  final Vector3 origin;
  final Vector3 axis;
  final double radius;

  static const minVertices = 8;
  static const maxRadiusDeviation = 0.08;
  static const maxTurnRad = 50 * math.pi / 180;

  /// Returns a fitted circle when [edges] are one closed, nearly circular loop.
  ///
  /// Rejects squares (n < 8, even though a square is concyclic), open chains,
  /// branched selections, and polygons whose turning angle is too sharp.
  static CircularRim? detect(List<(Vector3, Vector3)> edges) {
    final loop = orderedClosedLoop(edges);
    if (loop == null || loop.length < minVertices) return null;
    return fitCircle(loop);
  }

  /// Walk [edges] into a single closed vertex ring, or null.
  static List<Vector3>? orderedClosedLoop(List<(Vector3, Vector3)> edges) {
    if (edges.length < minVertices) return null;
    final adj = <String, List<Vector3>>{};
    void link(Vector3 a, Vector3 b) {
      final k = MeshFaceRegions.quantizePosition(a);
      (adj[k] ??= []).add(b);
    }

    for (final (a, b) in edges) {
      if ((b - a).length < 1e-12) return null;
      link(a, b);
      link(b, a);
    }
    if (adj.length != edges.length) return null;
    for (final nbrs in adj.values) {
      if (nbrs.length != 2) return null;
    }

    final start = edges.first.$1;
    final pts = <Vector3>[start];
    var prevKey = MeshFaceRegions.quantizePosition(start);
    var curr = edges.first.$2;
    for (var i = 1; i < edges.length; i++) {
      pts.add(curr);
      final k = MeshFaceRegions.quantizePosition(curr);
      final nbrs = adj[k];
      if (nbrs == null || nbrs.length != 2) return null;
      final n0 = MeshFaceRegions.quantizePosition(nbrs[0]);
      curr = n0 == prevKey ? nbrs[1] : nbrs[0];
      prevKey = k;
    }
    if (MeshFaceRegions.quantizePosition(curr) !=
        MeshFaceRegions.quantizePosition(start)) {
      return null;
    }
    return pts;
  }

  static CircularRim? fitCircle(List<Vector3> loop) {
    if (loop.length < minVertices) return null;
    var c = Vector3.zero();
    for (final p in loop) {
      c += p;
    }
    c /= loop.length.toDouble();

    var n = Vector3.zero();
    for (var i = 0; i < loop.length; i++) {
      final a = loop[i];
      final b = loop[(i + 1) % loop.length];
      n += Vector3(
        (a.y - b.y) * (a.z + b.z),
        (a.z - b.z) * (a.x + b.x),
        (a.x - b.x) * (a.y + b.y),
      );
    }
    if (n.length < 1e-12) return null;
    n.normalize();

    final radii = <double>[];
    for (final p in loop) {
      final d = p - c;
      final planar = d - n * d.dot(n);
      radii.add(planar.length);
      if (d.dot(n).abs() > 1e-3) {
        final rGuess = planar.length;
        if (d.dot(n).abs() > rGuess * 0.02 + 1e-4) return null;
      }
    }
    var rMean = 0.0;
    for (final r in radii) {
      rMean += r;
    }
    rMean /= radii.length;
    if (rMean < 1e-6) return null;
    for (final r in radii) {
      if ((r - rMean).abs() / rMean > maxRadiusDeviation) return null;
    }

    for (var i = 0; i < loop.length; i++) {
      final prev = loop[(i - 1 + loop.length) % loop.length];
      final cur = loop[i];
      final next = loop[(i + 1) % loop.length];
      final incoming = cur - prev;
      final outgoing = next - cur;
      if (incoming.length < 1e-12 || outgoing.length < 1e-12) return null;
      incoming.normalize();
      outgoing.normalize();
      final turn = math.acos(incoming.dot(outgoing).clamp(-1.0, 1.0));
      if (turn > maxTurnRad) return null;
    }

    return CircularRim(loop: loop, origin: c, axis: n, radius: rMean);
  }

  /// Polar angles of [loop] vertices in the fitted plane, sorted.
  List<double> azimuths() {
    final t = axis.normalized();
    var u = loop.first - origin;
    u = u - t * u.dot(t);
    if (u.length < 1e-10) return const [];
    u.normalize();
    final v = t.cross(u).normalized();
    final az = <double>[];
    for (final p in loop) {
      final d = p - origin;
      final rad = d - t * d.dot(t);
      if (rad.length < 1e-12) continue;
      az.add(math.atan2(rad.dot(v), rad.dot(u)));
    }
    az.sort();
    return az;
  }

  Vector3 get radialZero {
    final t = axis.normalized();
    var u = loop.first - origin;
    u = u - t * u.dot(t);
    return u;
  }

  /// Outward unit radial in the fitted plane, from the first loop vertex.
  Vector3 get radialUnit {
    final u = radialZero;
    if (u.length < 1e-10) return Vector3(1, 0, 0);
    return u.normalized();
  }

  /// [sample] snapped onto the circle in a meridional plane (axis × radial).
  Vector3 meridionalCorner(Vector3 sample) {
    final t = axis.normalized();
    final u = radialUnit;
    final z = (sample - origin).dot(t);
    return origin + t * z + u * radius;
  }

  /// Face normal flattened into the meridional plane so a revolve profile is
  /// a true 2D (radius, height) polygon instead of a skewed 3D banana.
  Vector3 meridionalNormal(Vector3 n) {
    final t = axis.normalized();
    final u = radialUnit;
    final p = u * n.dot(u) + t * n.dot(t);
    if (p.length < 1e-10) return n.normalized();
    return p.normalized();
  }
}
