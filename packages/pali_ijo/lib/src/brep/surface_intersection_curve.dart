part of 'geom.dart';

/// A regular intersection branch of two bounded rational surfaces. Anchors
/// select the branch and transverse sections; every evaluation solves both
/// surface equations. The anchors are not a polygonal definition of the edge.
class BrepSurfaceIntersectionCurve implements BrepCurve {
  BrepSurfaceIntersectionCurve({
    required this.first,
    required this.second,
    required List<Vector4> anchors,
    this.t0 = 0,
    this.t1 = 1,
    this.tolerance = 1e-9,
  }) : anchors = List.unmodifiable(anchors.map((p) => p.clone())) {
    if (anchors.length < 2 ||
        !tolerance.isFinite ||
        tolerance <= 0 ||
        !t0.isFinite ||
        !t1.isFinite ||
        math.min(t0, t1) < 0 ||
        math.max(t0, t1) > 1 ||
        t0 == t1 ||
        anchors.any(
          (p) => List.generate(
            4,
            (i) => p[i],
          ).any((x) => !x.isFinite || x < -1e-7 || x > 1 + 1e-7),
        )) {
      throw BrepUnsupported('Invalid surface intersection branch');
    }
  }
  final BrepSplineSurface first, second;
  final List<Vector4> anchors;
  final double tolerance;
  @override
  final double t0, t1;

  static (Vector3, Vector3, Vector3) normalizedJet(
    BrepSplineSurface s,
    double u,
    double v,
  ) {
    final j = s.jet(s.u0 + u * (s.u1 - s.u0), s.v0 + v * (s.v1 - s.v0));
    return (j.$1, j.$2 * (s.u1 - s.u0), j.$3 * (s.v1 - s.v0));
  }

  static Vector3 point(BrepSplineSurface s, double u, double v) =>
      s.evaluate(s.u0 + u * (s.u1 - s.u0), s.v0 + v * (s.v1 - s.v0));
  static Vector4 direction(
    BrepSplineSurface a,
    BrepSplineSurface b,
    Vector4 x,
  ) {
    final p = normalizedJet(a, x.x, x.y), q = normalizedJet(b, x.z, x.w);
    final c = [p.$2, p.$3, -q.$2, -q.$3];
    final d = Vector4(
      c[1].dot(c[2].cross(c[3])),
      -c[0].dot(c[2].cross(c[3])),
      c[0].dot(c[1].cross(c[3])),
      -c[0].dot(c[1].cross(c[2])),
    );
    final scale = c
        .map((v) => v.length)
        .fold(0.0, (double p, double v) => math.max(p, v));
    if (d.length < 1e-12 * scale * scale * scale)
      throw BrepUnsupported('Surface intersection is tangent or singular');
    return d.normalized();
  }

  static List<double>? _solve(List<List<double>> rows) {
    final m = rows.map((r) => List<double>.of(r)).toList();
    for (final row in m) {
      final scale = row.take(4).map((v) => v.abs()).fold(0.0, math.max);
      if (scale == 0 || !scale.isFinite) return null;
      for (var i = 0; i < 5; i++) row[i] /= scale;
    }
    for (var k = 0; k < 4; k++) {
      var pivot = k;
      for (var i = k + 1; i < 4; i++)
        if (m[i][k].abs() > m[pivot][k].abs()) pivot = i;
      if (m[pivot][k].abs() < 1e-15) return null;
      final tmp = m[k];
      m[k] = m[pivot];
      m[pivot] = tmp;
      final d = m[k][k];
      for (var j = k; j <= 4; j++) m[k][j] /= d;
      for (var i = 0; i < 4; i++)
        if (i != k) {
          final f = m[i][k];
          for (var j = k; j <= 4; j++) m[i][j] -= f * m[k][j];
        }
    }
    return [for (var i = 0; i < 4; i++) m[i][4]];
  }

  /// Newton correction in a fixed transverse parameter section. Null means
  /// convergence could not be verified, never an approximate successful hit.
  static Vector4? correct(
    BrepSplineSurface a,
    BrepSplineSurface b,
    Vector4 seed,
    Vector4 section,
    Vector4 normal,
    double tolerance,
  ) {
    var x = seed.clone();
    for (var k = 0; k < 24; k++) {
      final p = normalizedJet(a, x.x, x.y),
          q = normalizedJet(b, x.z, x.w),
          r = p.$1 - q.$1;
      final h = (x - section).dot(normal);
      if (r.length <= tolerance && h.abs() < 1e-11) return x;
      final c = [p.$2, p.$3, -q.$2, -q.$3];
      final delta = _solve([
        for (var i = 0; i < 3; i++) [for (final v in c) v[i], r[i]],
        [for (var i = 0; i < 4; i++) normal[i], h],
      ]);
      if (delta == null) return null;
      final d = Vector4.array(delta);
      var step = 1.0, accepted = false;
      final merit = r.length2 + h * h;
      while (step >= 1 / 1024) {
        final y = x - d * step;
        if (List.generate(
          4,
          (i) => y[i],
        ).every((v) => v >= -1e-7 && v <= 1 + 1e-7)) {
          final e = point(a, y.x, y.y) - point(b, y.z, y.w),
              g = (y - section).dot(normal);
          if (e.length2 + g * g < merit) {
            x = y;
            accepted = true;
            break;
          }
        }
        step *= .5;
      }
      if (!accepted) return null;
    }
    return null;
  }

  Vector4 parametersAt(double t) {
    final x = t.clamp(0.0, 1.0) * (anchors.length - 1),
        i = math.min(anchors.length - 2, x.floor());
    final a = anchors[i], b = anchors[i + 1], u = x - i;
    Vector4 slope(int k) {
      if (k == 0) return anchors[1] - anchors[0];
      if (k == anchors.length - 1) return anchors[k] - anchors[k - 1];
      final l = anchors[k] - anchors[k - 1],
          r = anchors[k + 1] - anchors[k],
          d = Vector4.zero();
      for (var j = 0; j < 4; j++)
        d[j] = l[j] * r[j] <= 0 ? 0 : 2 * l[j] * r[j] / (l[j] + r[j]);
      return d;
    }

    final da = slope(i), db = slope(i + 1);
    final seed =
        a * (2 * u * u * u - 3 * u * u + 1) +
        da * (u * u * u - 2 * u * u + u) +
        b * (-2 * u * u * u + 3 * u * u) +
        db * (u * u * u - u * u);
    final out = correct(
      first,
      second,
      seed,
      seed,
      direction(first, second, seed),
      tolerance,
    );
    if (out == null)
      throw BrepUnsupported(
        'Surface intersection evaluation did not converge at $t (seed $seed, interval $a to $b)',
      );
    return out;
  }

  @override
  Vector3 evaluate(double t) {
    final p = parametersAt(t);
    return point(first, p.x, p.y);
  }

  @override
  Vector3 tangent(double t) {
    final p = parametersAt(t),
        d = direction(first, second, p),
        j = normalizedJet(first, p.x, p.y);
    final i = math.min(
      anchors.length - 2,
      (t.clamp(0.0, 1.0) * (anchors.length - 1)).floor(),
    );
    return ((j.$2 * d.x + j.$3 * d.y) *
            (d.dot(anchors[i + 1] - anchors[i]) < 0 ? -1 : 1))
        .normalized();
  }

  @override
  double closestParam(Vector3 p) =>
      _curveNearest(this, p, math.max(32, anchors.length));
  @override
  Vector3 closestPoint(Vector3 p) => evaluate(closestParam(p));
  BrepSurfaceIntersectionCurve trimmed(double a, double b) =>
      BrepSurfaceIntersectionCurve(
        first: first,
        second: second,
        anchors: anchors,
        t0: a,
        t1: b,
        tolerance: tolerance,
      );
  BrepSurfaceIntersectionCurve transformed(BrepAffineTransform m) =>
      BrepSurfaceIntersectionCurve(
        first: first.transformed(m),
        second: second.transformed(m),
        anchors: anchors,
        t0: t0,
        t1: t1,
        tolerance:
            tolerance * math.max(1.0, m.x.length + m.y.length + m.z.length),
      );
  @override
  Map<String, dynamic> toJson() => {
    't': 'surface_intersection',
    'first': first.toJson(),
    'second': second.toJson(),
    'anchors': [
      for (final p in anchors) [p.x, p.y, p.z, p.w],
    ],
    'a': t0,
    'b': t1,
    'tolerance': tolerance,
  };
  factory BrepSurfaceIntersectionCurve.fromJson(Map<String, dynamic> j) =>
      BrepSurfaceIntersectionCurve(
        first: BrepSplineSurface.fromJson(j['first'] as Map<String, dynamic>),
        second: BrepSplineSurface.fromJson(j['second'] as Map<String, dynamic>),
        anchors: [
          for (final p in j['anchors'] as List)
            Vector4.array(_splineDoubles(p)),
        ],
        t0: (j['a'] as num).toDouble(),
        t1: (j['b'] as num).toDouble(),
        tolerance: (j['tolerance'] as num).toDouble(),
      );
}
