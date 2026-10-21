part of 'geom.dart';

/// Exact normal offset of a bounded parametric surface. The source surface is
/// retained so imported offsets survive saving without replacing them by mesh.
class BrepOffsetSurface implements BrepSurface {
  BrepOffsetSurface(this.base, this.distance) {
    if (!distance.isFinite) throw BrepUnsupported('Offset must be finite');
  }
  final BrepSurface base;
  final double distance;
  double get u0 =>
      base is BrepSplineSurface ? (base as BrepSplineSurface).u0 : 0;
  double get u1 =>
      base is BrepSplineSurface ? (base as BrepSplineSurface).u1 : 1;
  double get v0 =>
      base is BrepSplineSurface ? (base as BrepSplineSurface).v0 : 0;
  double get v1 =>
      base is BrepSplineSurface ? (base as BrepSplineSurface).v1 : 1;
  late final bool periodicU = [0.0, .5, 1.0].every(
    (v) =>
        (base.evaluate(u0, v0 + v * (v1 - v0)) -
                base.evaluate(u1, v0 + v * (v1 - v0)))
            .length <
        1e-8,
  );
  late final bool periodicV = [0.0, .5, 1.0].every(
    (u) =>
        (base.evaluate(u0 + u * (u1 - u0), v0) -
                base.evaluate(u0 + u * (u1 - u0), v1))
            .length <
        1e-8,
  );
  @override
  Vector3 evaluate(double u, double v) {
    if (periodicU && (u < 0 || u > 1)) u %= 1;
    if (periodicV && (v < 0 || v > 1)) v %= 1;
    final a = u0 + u * (u1 - u0), b = v0 + v * (v1 - v0);
    return base.evaluate(a, b) + base.normalAt(a, b) * distance;
  }

  Vector3 du(double u, double v) {
    u = periodicU ? u % 1 : u.clamp(0.0, 1.0);
    final a = periodicU ? u - 1e-5 : (u - 1e-5).clamp(0.0, 1.0),
        b = periodicU ? u + 1e-5 : (u + 1e-5).clamp(0.0, 1.0);
    return (evaluate(b, v) - evaluate(a, v)) / (b - a);
  }

  Vector3 dv(double u, double v) {
    v = periodicV ? v % 1 : v.clamp(0.0, 1.0);
    final a = periodicV ? v - 1e-5 : (v - 1e-5).clamp(0.0, 1.0),
        b = periodicV ? v + 1e-5 : (v + 1e-5).clamp(0.0, 1.0);
    return (evaluate(u, b) - evaluate(u, a)) / (b - a);
  }

  @override
  Vector3 normalAt(double u, double v) => du(u, v).cross(dv(u, v)).normalized();
  @override
  (double, double) parameters(Vector3 p) {
    final guess = base.parameters(p);
    var best = (0.0, 0.0), bestDistance = double.infinity;
    final seeds = <(double, double)>[
      if (guess != null)
        ((guess.$1 - u0) / (u1 - u0), (guess.$2 - v0) / (v1 - v0)),
      for (var i = 0; i <= 4; i++)
        for (var j = 0; j <= 4; j++) (i / 4, j / 4),
    ];
    for (final seed in seeds) {
      var u = seed.$1.clamp(0.0, 1.0), v = seed.$2.clamp(0.0, 1.0);
      for (var it = 0; it < 30; it++) {
        final r = evaluate(u, v) - p;
        if (r.length2 < 1e-20) return (u, v);
        final x = du(u, v),
            y = dv(u, v),
            xx = x.length2,
            yy = y.length2,
            xy = x.dot(y),
            det = xx * yy - xy * xy;
        if (det <= 1e-20 * xx * yy) break;
        final su = (yy * x.dot(r) - xy * y.dot(r)) / det,
            sv = (xx * y.dot(r) - xy * x.dot(r)) / det;
        var moved = false;
        for (var step = 1.0; step >= 1 / 1024; step *= .5) {
          final a = (u - step * su).clamp(0.0, 1.0),
              b = (v - step * sv).clamp(0.0, 1.0);
          if ((evaluate(a, b) - p).length2 < r.length2) {
            moved = (a - u).abs() + (b - v).abs() > 1e-12;
            u = a;
            v = b;
            break;
          }
        }
        if (!moved) break;
      }
      final d = (evaluate(u, v) - p).length2;
      if (d < bestDistance) {
        bestDistance = d;
        best = (u, v);
      }
      if (d < 1e-16) break;
    }
    return best;
  }

  @override
  Vector3 closestPoint(Vector3 p) {
    final uv = parameters(p);
    return evaluate(uv.$1, uv.$2);
  }

  @override
  Map<String, dynamic> toJson() => {
    't': 'offset',
    'base': base.toJson(),
    'distance': distance,
  };
  factory BrepOffsetSurface.fromJson(Map<String, dynamic> j) =>
      BrepOffsetSurface(
        BrepSurface.fromJson(j['base'] as Map<String, dynamic>),
        (j['distance'] as num).toDouble(),
      );
}
