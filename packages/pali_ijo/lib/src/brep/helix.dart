part of 'geom.dart';

/// A circular or conical helix with an unwrapped angular parameter (radians).
/// [lead] is signed axial travel per revolution, not the spacing between starts.
class BrepHelix implements BrepCurve {
  BrepHelix({
    required this.origin,
    required Vector3 axis,
    required this.radius,
    required this.lead,
    this.radiusSlope = 0,
    required this.t0,
    required this.t1,
    Vector3? xDir,
  }) : axis = axis.normalized(),
       xDir = BrepCylinder._planeX(axis.normalized(), xDir) {
    if (!radius.isFinite ||
        !lead.isFinite ||
        !radiusSlope.isFinite ||
        math.min(radiusAt(t0), radiusAt(t1)) <= 0 ||
        !radiusAt(t0).isFinite ||
        !radiusAt(t1).isFinite ||
        !t0.isFinite ||
        !t1.isFinite ||
        t0 == t1 ||
        axis.length2 < 1e-20 ||
        !axis.length2.isFinite ||
        ![
          origin.x,
          origin.y,
          origin.z,
          axis.x,
          axis.y,
          axis.z,
          this.xDir.x,
          this.xDir.y,
          this.xDir.z,
        ].every((v) => v.isFinite)) {
      throw ArgumentError(
        'A helix requires finite dimensions and a nonzero axis/domain',
      );
    }
    yDir = this.axis.cross(this.xDir).normalized();
  }

  final Vector3 origin, axis, xDir;
  late final Vector3 yDir;
  final double radius, lead;

  /// Radial change per radian; zero is a cylindrical helix.
  final double radiusSlope;
  double radiusAt(double t) => radius + radiusSlope * t;
  double get arcLength {
    final speed2 =
        radiusSlope * radiusSlope + math.pow(lead / (2 * math.pi), 2);
    if (radiusSlope == 0)
      return math.sqrt(radius * radius + speed2) * (t1 - t0).abs();
    if ((radiusSlope * (t1 - t0)).abs() <
        1e-6 * math.min(radiusAt(t0), radiusAt(t1))) {
      double speed(double t) => math.sqrt(radiusAt(t) * radiusAt(t) + speed2);
      return (t1 - t0).abs() *
          (speed(t0) + 4 * speed((t0 + t1) / 2) + speed(t1)) /
          6;
    }
    final c = math.sqrt(speed2);
    double integral(double r) =>
        .5 *
        (r * math.sqrt(r * r + speed2) +
            speed2 * math.log((r + math.sqrt(r * r + speed2)) / c));
    return ((integral(radiusAt(t1)) - integral(radiusAt(t0))) / radiusSlope)
        .abs();
  }

  @override
  final double t0, t1;

  Vector3 derivative(double t) =>
      (-xDir * math.sin(t) + yDir * math.cos(t)) * radiusAt(t) +
      (xDir * math.cos(t) + yDir * math.sin(t)) * radiusSlope +
      axis * (lead / (2 * math.pi));

  /// Exact extrema for circular helices, a conservative envelope for conical ones.
  Iterable<Vector3> get boundsPoints sync* {
    final lo = math.min(t0, t1), hi = math.max(t0, t1);
    if (radiusSlope != 0) {
      final r = math.max(radiusAt(lo), radiusAt(hi));
      final a = origin + axis * (lead * lo / (2 * math.pi)),
          b = origin + axis * (lead * hi / (2 * math.pi));
      final min = Vector3.zero(), max = Vector3.zero();
      for (var j = 0; j < 3; j++) {
        final extent = r * math.sqrt(xDir[j] * xDir[j] + yDir[j] * yDir[j]);
        min[j] = math.min(a[j], b[j]) - extent;
        max[j] = math.max(a[j], b[j]) + extent;
      }
      yield min;
      yield max;
      return;
    }
    yield evaluate(lo);
    yield evaluate(hi);
    for (var j = 0; j < 3; j++) {
      final a = radius * xDir[j],
          b = radius * yDir[j],
          c = axis[j] * lead / (2 * math.pi);
      final amplitude = math.sqrt(a * a + b * b);
      if (amplitude < 1e-20 || c.abs() > amplitude) continue;
      final phi = math.atan2(a, b),
          root = math.acos((-c / amplitude).clamp(-1.0, 1.0));
      for (final angle in [root - phi, -root - phi]) {
        final first = ((lo - angle) / (2 * math.pi)).ceil(),
            last = ((hi - angle) / (2 * math.pi)).floor();
        for (var k = first; k <= last; k++)
          yield evaluate(angle + 2 * math.pi * k);
      }
    }
  }

  @override
  Vector3 evaluate(double t) =>
      origin +
      (xDir * math.cos(t) + yDir * math.sin(t)) * radiusAt(t) +
      axis * (lead * t / (2 * math.pi));
  @override
  Vector3 tangent(double t) => derivative(t).normalized();
  @override
  double closestParam(Vector3 p) {
    final d = p - origin, h = lead / (2 * math.pi);
    final angle = math.atan2(d.dot(yDir), d.dot(xDir));
    final rho = math.sqrt(math.pow(d.dot(xDir), 2) + math.pow(d.dot(yDir), 2));
    final den = h * h + radiusSlope * radiusSlope;
    final axial = den > 1e-30
        ? (h * d.dot(axis) + radiusSlope * (rho - radius)) / den
        : (t0 + t1) / 2;
    final turn = ((axial - angle) / (2 * math.pi)).round();
    final lo = math.min(t0, t1), hi = math.max(t0, t1);
    var best = lo, distance = (evaluate(lo) - p).length2;
    void consider(double t) {
      final e = (evaluate(t) - p).length2;
      if (e < distance) {
        distance = e;
        best = t;
      }
    }

    consider(hi);
    for (var k = turn - 1; k <= turn + 1; k++) {
      var t = (angle + 2 * math.pi * k).clamp(lo, hi).toDouble();
      for (var i = 0; i < 24; i++) {
        final r = evaluate(t) - p, du = derivative(t);
        final dd =
            -(xDir * math.cos(t) + yDir * math.sin(t)) * radiusAt(t) +
            (-xDir * math.sin(t) + yDir * math.cos(t)) * (2 * radiusSlope);
        final den = du.length2 + r.dot(dd);
        if (den <= 1e-20) break;
        final step = (r.dot(du) / den).clamp(-math.pi / 2, math.pi / 2);
        var scale = 1.0;
        var next = t;
        while (scale > 1 / 1024) {
          next = (t - step * scale).clamp(lo, hi).toDouble();
          if ((evaluate(next) - p).length2 <= r.length2) break;
          scale *= .5;
        }
        if ((next - t).abs() < 1e-13) break;
        t = next;
      }
      consider(t);
    }
    return best;
  }

  @override
  Vector3 closestPoint(Vector3 p) => evaluate(closestParam(p));
  BrepHelix trimmed(double a, double b) => BrepHelix(
    origin: origin,
    axis: axis,
    xDir: xDir,
    radius: radius,
    lead: lead,
    radiusSlope: radiusSlope,
    t0: a,
    t1: b,
  );
  @override
  Map<String, dynamic> toJson() => {
    't': 'helix',
    'o': _v(origin),
    'n': _v(axis),
    'x': _v(xDir),
    'r': radius,
    'lead': lead,
    if (radiusSlope != 0) 'slope': radiusSlope,
    'a': t0,
    'b': t1,
  };
  factory BrepHelix.fromJson(Map<String, dynamic> j) => BrepHelix(
    origin: _p(j['o']),
    axis: _p(j['n']),
    xDir: _p(j['x']),
    radius: (j['r'] as num).toDouble(),
    lead: (j['lead'] as num).toDouble(),
    radiusSlope: (j['slope'] as num?)?.toDouble() ?? 0,
    t0: (j['a'] as num).toDouble(),
    t1: (j['b'] as num).toDouble(),
  );
}
