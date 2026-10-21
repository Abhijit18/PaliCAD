part of 'geom.dart';

/// A branch of the exact intersection of a cylinder and a quadric. The
/// parameter is the carrier's angle. Axial position is evaluated from the
/// quadratic equation at that angle, rather than fitted to a display mesh.
class BrepIntersectionCurve implements BrepCurve {
  BrepIntersectionCurve({
    required this.carrier,
    required this.target,
    required this.branch,
    this.t0 = 0,
    this.t1 = math.pi * 2,
  }) {
    if (target is! BrepCylinder && target is! BrepSphere) {
      throw ArgumentError('Intersection target must be a cylinder or sphere');
    }
    if (branch != -1 && branch != 1) throw ArgumentError('Invalid root branch');
    if (target is BrepCylinder &&
        carrier.axis.cross((target as BrepCylinder).axis).length2 < 1e-12) {
      throw ArgumentError('Parallel cylinders have generator intersections');
    }
  }

  final BrepCylinder carrier;
  final BrepSurface target;
  final int branch;
  @override
  final double t0;
  @override
  final double t1;

  (double, double, double) coefficients(double u) {
    final p = carrier.evaluate(u, 0);
    final s = target;
    if (s is BrepSphere) {
      final d = p - s.center;
      return (1, 2 * d.dot(carrier.axis), d.length2 - s.radius * s.radius);
    }
    final c = s as BrepCylinder;
    final d = p - c.origin;
    final radial = d - c.axis * d.dot(c.axis);
    final direction = carrier.axis - c.axis * carrier.axis.dot(c.axis);
    return (
      direction.length2,
      2 * radial.dot(direction),
      radial.length2 - c.radius * c.radius,
    );
  }

  double discriminant(double u) {
    final (a, b, c) = coefficients(u);
    return b * b - 4 * a * c;
  }

  @override
  Vector3 evaluate(double t) {
    final (a, b, c) = coefficients(t);
    final d = b * b - 4 * a * c;
    final scale = math.max(1.0, b * b + (4 * a * c).abs());
    if (d < -1e-12 * scale) {
      throw StateError('Parameter outside the real intersection branch');
    }
    // A tangency has a double root. Roundoff from rebuilding an orthonormal
    // frame must not split that root by sqrt(machine epsilon) on reload.
    final root = math.sqrt(d.abs() <= 2e-15 * scale ? 0 : math.max(0, d));
    // Avoid cancellation in the smaller quadratic root.
    final q = -.5 * (b + (b >= 0 ? root : -root));
    final direct = (-b + branch * root) / (2 * a);
    final v = q.abs() < 1e-20
        ? direct
        : ((branch == 1) == (b < 0) ? q / a : c / q);
    return carrier.evaluate(t, v);
  }

  @override
  Vector3 tangent(double t) {
    final p = evaluate(t);
    final uv = target.parameters(p)!;
    final n1 = carrier.normalAt(t, 0);
    final n2 = target.normalAt(uv.$1, uv.$2);
    final cross = n1.cross(n2);
    final h = math.min(1e-6, (t1 - t0).abs() / 100);
    final lo = math.max(t0, t - h), hi = math.min(t1, t + h);
    final step = evaluate(hi) - evaluate(lo);
    if (cross.length2 < 1e-20) return step.normalized();
    return (cross.dot(step) < 0 ? -cross : cross).normalized();
  }

  @override
  double closestParam(Vector3 p) {
    var candidate = carrier.parameters(p)!.$1;
    while (candidate < t0) {
      candidate += 2 * math.pi;
    }
    while (candidate > t1 && candidate - 2 * math.pi >= t0) {
      candidate -= 2 * math.pi;
    }
    candidate = candidate.clamp(t0, t1);
    if ((evaluate(candidate) - p).length2 < 1e-16) return candidate;
    const n = 96;
    var best = 0;
    var distance = double.infinity;
    for (var i = 0; i <= n; i++) {
      final t = t0 + (t1 - t0) * i / n;
      final d = (evaluate(t) - p).length2;
      if (d < distance) {
        distance = d;
        best = i;
      }
    }
    var lo = t0 + (t1 - t0) * math.max(0, best - 1) / n;
    var hi = t0 + (t1 - t0) * math.min(n, best + 1) / n;
    for (var i = 0; i < 48; i++) {
      final a = lo + (hi - lo) / 3, b = hi - (hi - lo) / 3;
      if ((evaluate(a) - p).length2 < (evaluate(b) - p).length2) {
        hi = b;
      } else {
        lo = a;
      }
    }
    final mid = (lo + hi) / 2;
    var result = mid;
    for (final t in [t0, t1]) {
      if ((evaluate(t) - p).length2 < (evaluate(result) - p).length2)
        result = t;
    }
    return result;
  }

  @override
  Vector3 closestPoint(Vector3 p) => evaluate(closestParam(p));

  BrepIntersectionCurve trimmed(double start, double end) =>
      BrepIntersectionCurve(
        carrier: carrier,
        target: target,
        branch: branch,
        t0: start,
        t1: end,
      );

  /// A conservative axial envelope, including portions outside this branch.
  (double, double) get axialBounds {
    final s = target;
    if (s is BrepSphere) {
      final z = (s.center - carrier.origin).dot(carrier.axis);
      return (z - s.radius, z + s.radius);
    }
    final c = s as BrepCylinder;
    final direction = carrier.axis - c.axis * carrier.axis.dot(c.axis);
    final d = carrier.origin - c.origin;
    final z = -d.dot(direction) / direction.length2;
    final extent = (carrier.radius + c.radius) / direction.length;
    return (z - extent, z + extent);
  }

  @override
  Map<String, dynamic> toJson() => {
    't': 'intersection',
    'carrier': carrier.toJson(),
    'target': target.toJson(),
    'branch': branch,
    'a': t0,
    'b': t1,
  };
  factory BrepIntersectionCurve.fromJson(Map<String, dynamic> j) =>
      BrepIntersectionCurve(
        carrier: BrepCylinder.fromJson(j['carrier'] as Map<String, dynamic>),
        target: BrepSurface.fromJson(j['target'] as Map<String, dynamic>),
        branch: j['branch'] as int,
        t0: (j['a'] as num).toDouble(),
        t1: (j['b'] as num).toDouble(),
      );
}
