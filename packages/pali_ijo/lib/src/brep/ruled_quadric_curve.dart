part of 'geom.dart';

/// An implicit quadric, evaluated in its native coordinates even when affine.
/// Coefficients describe F(origin + direction * t), without subtracting nearly
/// equal samples of F to recover the quadratic term.
class BrepQuadric {
  static bool supports(BrepSurface s) =>
      s is BrepSphere ||
      s is BrepCylinder ||
      s is BrepCone ||
      s is BrepRuledSurface && s.lower is BrepLine && s.upper is BrepLine ||
      s is BrepAffineSurface && supports(s.base);

  static (double, double, double) line(BrepSurface s, Vector3 p, Vector3 d) {
    if (s is BrepAffineSurface) {
      final a = s.transform.inversePoint(p);
      return line(s.base, a, s.transform.inverseVector(d));
    }
    if (s is BrepRuledSurface && s.lower is BrepLine && s.upper is BrepLine) {
      final a = s.evaluate(0, 0),
          b = s.evaluate(1, 0) - a,
          c = s.evaluate(0, 1) - a;
      final e = s.evaluate(1, 1) - a - b - c,
          n = b.cross(c),
          determinant = n.dot(e);
      if (determinant.abs() < 1e-12 * math.max(1.0, n.length * e.length)) {
        if (n.length < 1e-14) throw BrepUnsupported('Collapsed bilinear patch');
        return (0, n.dot(d), n.dot(p - a));
      }
      // In this affine basis a bilinear patch is z=x*y.
      Vector3 coordinates(Vector3 v) => Vector3(
        v.dot(c.cross(e)) / determinant,
        v.dot(e.cross(b)) / determinant,
        v.dot(n) / determinant,
      );
      final x = coordinates(p - a), v = coordinates(d);
      return (-v.x * v.y, v.z - x.x * v.y - x.y * v.x, x.z - x.x * x.y);
    }
    if (s is BrepSphere) {
      final r = p - s.center;
      return (d.length2, 2 * r.dot(d), r.length2 - s.radius * s.radius);
    }
    if (s is BrepCylinder) {
      final r = p - s.origin;
      final x = r - s.axis * r.dot(s.axis), y = d - s.axis * d.dot(s.axis);
      return (y.length2, 2 * x.dot(y), x.length2 - s.radius * s.radius);
    }
    if (s is BrepCone) {
      final r = p - s.origin;
      final z = r.dot(s.axis), dz = d.dot(s.axis);
      final x = r - s.axis * z, y = d - s.axis * dz;
      final slope = s.height.abs() < 1e-18
          ? 0.0
          : (s.topRadius - s.radius) / s.height;
      final radius = s.radius + slope * z, dr = slope * dz;
      return (
        y.length2 - dr * dr,
        2 * (x.dot(y) - radius * dr),
        x.length2 - radius * radius,
      );
    }
    throw BrepUnsupported('Surface has no quadratic implicit equation');
  }

  static double value(BrepSurface s, Vector3 p) =>
      line(s, p, Vector3.zero()).$3;
}

/// Shared boundary of a ruled wall and a quadric. The original surfaces and
/// root branch define the curve; no polyline or spline fit defines the model.
class BrepRuledQuadricCurve implements BrepCurve {
  BrepRuledQuadricCurve({
    required this.surface,
    required this.target,
    required this.branch,
    this.t0 = 0,
    this.t1 = 1,
  }) {
    if (!BrepQuadric.supports(target) ||
        (branch != -1 && branch != 1) ||
        !t0.isFinite ||
        !t1.isFinite ||
        math.min(t0, t1) < -1e-10 ||
        math.max(t0, t1) > 1 + 1e-10 ||
        t0 == t1) {
      throw ArgumentError('Invalid ruled quadric intersection');
    }
  }
  final BrepRuledSurface surface;
  final BrepSurface target;
  final int branch;
  @override
  final double t0, t1;

  (double, double, double) coefficients(double u) =>
      BrepQuadric.line(target, surface.evaluate(u, 0), surface.dv(u, 0));
  double discriminant(double u) {
    final (a, b, c) = coefficients(u);
    return b * b - 4 * a * c;
  }

  double ruling(double u) {
    final (a, b, c) = coefficients(u);
    final scale = math.max(1.0, math.max(a.abs(), math.max(b.abs(), c.abs())));
    if (a.abs() < 1e-14 * scale) {
      if (b.abs() < 1e-14 * scale) {
        throw BrepUnsupported(
          'Intersection contains a coincident or parallel ruling',
        );
      }
      return -c / b;
    }
    final disc = b * b - 4 * a * c;
    if (disc < -1e-11 * math.max(1.0, b * b + (4 * a * c).abs())) {
      throw BrepUnsupported('Parameter outside real ruled intersection branch');
    }
    final root = math.sqrt(math.max(0, disc));
    final q = -.5 * (b + (b >= 0 ? root : -root));
    return q.abs() < 1e-24
        ? -b / (2 * a)
        : ((branch == 1) == (b < 0) ? q / a : c / q);
  }

  @override
  Vector3 evaluate(double t) => surface.evaluate(t, ruling(t));
  @override
  Vector3 tangent(double t) {
    final p = evaluate(t), v = ruling(t), uv = target.parameters(p)!;
    final n = target.normalAt(uv.$1, uv.$2);
    final du = surface.du(t, v), dv = surface.dv(t, v), den = n.dot(dv);
    if (den.abs() > 1e-12 * dv.length) {
      return (du - dv * (n.dot(du) / den)).normalized();
    }
    final h = math.min(1e-6, (t1 - t0).abs() / 100);
    return (evaluate(math.min(math.max(t0, t1), t + h)) -
            evaluate(math.max(math.min(t0, t1), t - h)))
        .normalized();
  }

  @override
  double closestParam(Vector3 p) => _curveNearest(this, p, 48);
  @override
  Vector3 closestPoint(Vector3 p) => evaluate(closestParam(p));
  BrepRuledQuadricCurve trimmed(double a, double b) => BrepRuledQuadricCurve(
    surface: surface,
    target: target,
    branch: branch,
    t0: a,
    t1: b,
  );
  @override
  Map<String, dynamic> toJson() => {
    't': 'ruled_quadric',
    'surface': surface.toJson(),
    'target': target.toJson(),
    'branch': branch,
    'a': t0,
    'b': t1,
  };
  factory BrepRuledQuadricCurve.fromJson(Map<String, dynamic> j) =>
      BrepRuledQuadricCurve(
        surface: BrepRuledSurface.fromJson(
          j['surface'] as Map<String, dynamic>,
        ),
        target: BrepSurface.fromJson(j['target'] as Map<String, dynamic>),
        branch: j['branch'] as int,
        t0: (j['a'] as num).toDouble(),
        t1: (j['b'] as num).toDouble(),
      );
}
