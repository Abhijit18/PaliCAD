import 'dart:math' as math;
import 'package:vector_math/vector_math_64.dart';
import 'geom.dart';
import 'solid.dart';
import 'rational_ruled.dart';

/// Interchange conversion. Conics and affine quadrics are rational and exact.
/// Other bounded curves use adaptive cubic Hermite spans with a stated distance
/// tolerance. This changes the interchange representation, never the source.
class BrepSplineConvert {
  /// Recognizes the complete rational tensor sphere family by its control
  /// net, weights and knot multiplicities, rather than fitting display points.
  static BrepAffineSurface? ellipsoid(BrepSplineSurface s) {
    if (s.degreeU != 2 ||
        s.degreeV != 2 ||
        s.points.length != 9 ||
        s.points.first.length != 5)
      return null;
    final center = (s.points[0][0] + s.points[0][4]) * .5;
    final z = (s.points[0][0] - s.points[0][4]) * .5,
        x = s.points[0][2] - center,
        y = s.points[2][2] - center;
    BrepAffineTransform m;
    try {
      m = BrepAffineTransform(x, y, z, center);
    } on BrepUnsupported {
      return null;
    }
    final candidate = BrepAffineSurface(
          BrepSphere(center: Vector3.zero(), radius: 1),
          m,
        ),
        net = surface(candidate);
    final scale = math.max(1.0, x.length + y.length + z.length),
        weight = s.weights[0][0] / net.weights[0][0];
    bool knotsMatch(List<double> a, List<double> b) {
      if (a.length != b.length) return false;
      final lo = a.first, span = a.last - lo;
      return List.generate(
        a.length,
        (i) => i,
      ).every((i) => ((a[i] - lo) / span - b[i]).abs() < 1e-12);
    }

    if (!knotsMatch(s.knotsU, net.knotsU) || !knotsMatch(s.knotsV, net.knotsV))
      return null;
    for (var i = 0; i < 9; i++) {
      for (var j = 0; j < 5; j++) {
        if ((s.points[i][j] - net.points[i][j]).length > scale * 1e-10 ||
            (s.weights[i][j] - net.weights[i][j] * weight).abs() >
                1e-12 * weight)
          return null;
      }
    }
    return candidate;
  }

  static BrepSplineCurve curve(BrepCurve c, {double tolerance = 1e-7}) {
    if (c is BrepSplineCurve) return c;
    if (c is BrepPlaneSectionCurve) {
      final patch = exactRuled(c.surface);
      if (patch != null) {
        final boundary = curve(c.surface.lower);
        double parameter(double u) {
          if (u == 0 || u == 1) return u;
          final t = boundary.closestParam(c.surface.evaluate(u, 0));
          return (t - boundary.t0) / (boundary.t1 - boundary.t0);
        }

        return BrepRationalRuled.section(
          patch,
          c.plane,
          parameter(c.t0),
          parameter(c.t1),
        );
      }
    }
    if (c is BrepAffineCurve) {
      final maxScale =
          math.max(
            c.transform.x.length,
            math.max(c.transform.y.length, c.transform.z.length),
          ) *
          math.sqrt(3);
      return curve(
        c.base,
        tolerance: tolerance / maxScale,
      ).transformed(c.transform);
    }
    if (c is BrepLine)
      return BrepSplineCurve(
        degree: 1,
        points: [c.a, c.b],
        knots: [0, 0, 1, 1],
      );
    if (c is BrepCircle || c is BrepEllipse) {
      final center = c is BrepCircle ? c.center : (c as BrepEllipse).center;
      final x = c is BrepCircle
          ? c.xDir * c.radius
          : (c as BrepEllipse).xDir * c.rMajor;
      final y = c is BrepCircle
          ? c.yDir * c.radius
          : (c as BrepEllipse).yDir * c.rMinor;
      final n = math.max(1, ((c.t1 - c.t0).abs() / (math.pi / 2)).ceil());
      final points = <Vector3>[],
          weights = <double>[],
          knots = <double>[0, 0, 0];
      for (var i = 0; i < n; i++) {
        final a = c.t0 + (c.t1 - c.t0) * i / n,
            b = c.t0 + (c.t1 - c.t0) * (i + 1) / n,
            m = (a + b) / 2,
            w = math.cos((b - a) / 2);
        if (i == 0) {
          points.add(c.evaluate(a));
          weights.add(1);
        }
        points.add(center + (x * math.cos(m) + y * math.sin(m)) / w);
        weights.add(w);
        points.add(c.evaluate(b));
        weights.add(1);
        knots.addAll(List.filled(i == n - 1 ? 3 : 2, (i + 1) / n));
      }
      return BrepSplineCurve(
        degree: 2,
        points: points,
        knots: knots,
        weights: weights,
      );
    }
    final breaks = fitBreaks([c], tolerance);
    return fitCurve(c, breaks);
  }

  static bool _regularized(BrepCurve c) =>
      c is BrepRuledQuadricCurve ||
      c is BrepIntersectionCurve ||
      c is BrepAffineCurve && _regularized(c.base);
  static Vector3 _fitPoint(BrepCurve c, double t, {bool? regularize}) {
    // Quadratic slowing at both ends removes square-root parameter poles at
    // tangencies while preserving the same bounded geometric curve.
    final u = (regularize ?? _regularized(c))
        ? math.pow(math.sin(math.pi * t / 2), 2).toDouble()
        : t;
    return c.evaluate(c.t0 + (c.t1 - c.t0) * u);
  }

  static Vector3 _derivative(BrepCurve c, double t, {bool? regularize}) {
    final dt = c.t1 - c.t0, u = c.t0 + dt * t;
    if (regularize ?? _regularized(c)) {
      Vector3 point(double t) => _fitPoint(c, t, regularize: true);
      const h = 1e-4;
      if (t < 2 * h)
        return (point(t) * -25 +
                point(t + h) * 48 -
                point(t + 2 * h) * 36 +
                point(t + 3 * h) * 16 -
                point(t + 4 * h) * 3) /
            (12 * h);
      if (t > 1 - 2 * h)
        return (point(t) * 25 -
                point(t - h) * 48 +
                point(t - 2 * h) * 36 -
                point(t - 3 * h) * 16 +
                point(t - 4 * h) * 3) /
            (12 * h);
      return (-point(t + 2 * h) +
              point(t + h) * 8 -
              point(t - h) * 8 +
              point(t - 2 * h)) /
          (12 * h);
    }
    if (c is BrepLine) return c.b - c.a;
    if (c is BrepHelix) return c.derivative(u) * dt;
    if (c is BrepCircle) return c.tangent(u) * (c.radius * dt);
    if (c is BrepEllipse)
      return (-c.xDir * (c.rMajor * math.sin(u)) +
              c.yDir * (c.rMinor * math.cos(u))) *
          dt;
    final h = 1e-5, a = math.max(0.0, t - h), b = math.min(1.0, t + h);
    return (c.evaluate(c.t0 + dt * b) - c.evaluate(c.t0 + dt * a)) / (b - a);
  }

  static List<Vector3> _span(
    BrepCurve c,
    double a,
    double b, {
    bool? regularize,
  }) {
    final x = _fitPoint(c, a, regularize: regularize),
        y = _fitPoint(c, b, regularize: regularize),
        h = (b - a) / 3;
    return [
      x,
      x + _derivative(c, a, regularize: regularize) * h,
      y - _derivative(c, b, regularize: regularize) * h,
      y,
    ];
  }

  static List<double> fitBreaks(
    List<BrepCurve> curves,
    double tolerance, {
    bool? regularize,
  }) {
    final commonMap = regularize ?? curves.any(_regularized);
    if (!tolerance.isFinite || tolerance <= 0)
      throw BrepUnsupported('Spline fit tolerance must be positive');
    final out = <double>[0];
    void refine(double a, double b, int depth) {
      var error = 0.0;
      for (final c in curves) {
        if (c is BrepHelix && !commonMap) {
          // The fourth derivative is bounded by max radius + 4*radial slope.
          // Bound cubic Hermite error even when sampling aliases full turns.
          error = math.max(error, (math.max(c.radiusAt(c.t0),c.radiusAt(c.t1))+4*c.radiusSlope.abs()) * math.pow((b-a)*(c.t1-c.t0), 4) / 384);
        }
        final p = _span(c, a, b, regularize: commonMap);
        for (final t in [.125, .25, .375, .5, .625, .75, .875]) {
          final q =
              p[0] * math.pow(1 - t, 3).toDouble() +
              p[1] * (3 * t * (1 - t) * (1 - t)) +
              p[2] * (3 * t * t * (1 - t)) +
              p[3] * (t * t * t);
          error = math.max(
            error,
            (q - _fitPoint(c, a + (b - a) * t, regularize: commonMap)).length,
          );
        }
      }
      if (error > tolerance * .5) {
        if (depth >= 16)
          throw BrepUnsupported('Spline fitting failed to reach tolerance');
        final m = (a + b) / 2;
        refine(a, m, depth + 1);
        refine(m, b, depth + 1);
      } else {
        out.add(b);
      }
    }

    refine(0, 1, 0);
    return out;
  }

  static BrepSplineCurve fitCurve(
    BrepCurve c,
    List<double> breaks, {
    bool? regularize,
  }) {
    final points = <Vector3>[], knots = <double>[0, 0, 0, 0];
    for (var i = 1; i < breaks.length; i++) {
      final p = _span(c, breaks[i - 1], breaks[i], regularize: regularize);
      points.addAll(i == 1 ? p : p.skip(1));
      knots.addAll(List.filled(i == breaks.length - 1 ? 4 : 3, breaks[i]));
    }
    return BrepSplineCurve(degree: 3, points: points, knots: knots);
  }

  /// Returns null when independent rationalization would change the ruling
  /// correspondence, such as an angular conic paired with a linear parameter.
  static BrepSplineSurface? exactRuled(BrepRuledSurface s) {
    BrepCurve base(BrepCurve c) => c is BrepAffineCurve ? base(c.base) : c;
    bool polynomialParameter(BrepCurve c) =>
        c is BrepLine || c is BrepSplineCurve;
    bool conic(BrepCurve c) => c is BrepCircle || c is BrepEllipse;
    final a = base(s.lower), b = base(s.upper);
    final compatible =
        polynomialParameter(a) && polynomialParameter(b) ||
        conic(a) &&
            conic(b) &&
            ((a.t1 - a.t0).abs() - (b.t1 - b.t0).abs()).abs() < 1e-12;
    if (!compatible) return null;
    return BrepRationalRuled.build(curve(s.lower), curve(s.upper));
  }

  static BrepSplineSurface surface(
    BrepSurface s, {
    List<Vector3> boundary = const [],
    double tolerance = 1e-7,
  }) {
    if (s is BrepSplineSurface) return s;
    if (s is BrepAffineSurface)
      return surface(
        s.base,
        boundary: boundary.map(s.transform.inversePoint).toList(),
        tolerance:
            tolerance /
            (s.transform.x.length +
                s.transform.y.length +
                s.transform.z.length),
      ).transformed(s.transform);
    if (s is BrepRuledSurface) {
      final exact = exactRuled(s);
      if (exact != null) return exact;
      // Both rails must share the same map: changing only one moves rulings.
      final regularize = _regularized(s.lower) || _regularized(s.upper);
      final breaks = fitBreaks(
            [s.lower, s.upper],
            tolerance,
            regularize: regularize,
          ),
          a = fitCurve(s.lower, breaks, regularize: regularize),
          b = fitCurve(s.upper, breaks, regularize: regularize);
      return BrepSplineSurface(
        degreeU: 3,
        degreeV: 1,
        points: [
          for (var i = 0; i < a.points.length; i++) [a.points[i], b.points[i]],
        ],
        knotsU: a.knots,
        knotsV: [0, 0, 1, 1],
      );
    }
    final circle = curve(
      BrepCircle(
        center: Vector3.zero(),
        normal: Vector3(0, 0, 1),
        xDir: Vector3(1, 0, 0),
        radius: 1,
      ),
    );
    Vector3 origin, axis, x, y;
    List<Vector3> meridian;
    List<double> weights, knots;
    int degree;
    if (s is BrepSphere) {
      origin = s.center;
      axis = Vector3(0, 0, 1);
      x = Vector3(1, 0, 0);
      y = Vector3(0, 1, 0);
      final arc = curve(
        BrepCircle(
          center: Vector3.zero(),
          normal: Vector3(0, 1, 0),
          xDir: Vector3(0, 0, 1),
          radius: s.radius,
          t0: 0,
          t1: math.pi,
        ),
      );
      meridian = arc.points;
      weights = arc.weights;
      knots = arc.knots;
      degree = 2;
    } else if (s is BrepTorus) {
      origin = s.center;
      axis = s.axis;
      x = s.xDir;
      y = s.yDir;
      meridian = [
        for (final p in circle.points)
          Vector3(s.majorRadius + s.minorRadius * p.x, 0, s.minorRadius * p.y),
      ];
      weights = circle.weights;
      knots = circle.knots;
      degree = 2;
    } else if (s is BrepCylinder || s is BrepCone) {
      origin = s is BrepCylinder ? s.origin : (s as BrepCone).origin;
      axis = s is BrepCylinder ? s.axis : (s as BrepCone).axis;
      x = s is BrepCylinder ? s.xDir : (s as BrepCone).xDir;
      y = axis.cross(x);
      final zs = [for (final p in boundary) (p - origin).dot(axis)]..sort();
      if (zs.isEmpty)
        throw BrepUnsupported('A bounded cylinder or cone needs rim positions');
      final a = zs.first, b = zs.last;
      meridian = [
        Vector3(
          s is BrepCylinder ? s.radius : (s as BrepCone).radiusAt(a),
          0,
          a,
        ),
        Vector3(
          s is BrepCylinder ? s.radius : (s as BrepCone).radiusAt(b),
          0,
          b,
        ),
      ];
      weights = [1, 1];
      knots = [0, 0, 1, 1];
      degree = 1;
    } else if (s is BrepPlane) {
      final uv = boundary.map((p) => s.parameters(p)!).toList();
      if (uv.isEmpty) throw BrepUnsupported('Plane has no finite bounds');
      final u = uv.map((p) => p.$1).toList()..sort(),
          v = uv.map((p) => p.$2).toList()..sort();
      return BrepSplineSurface(
        degreeU: 1,
        degreeV: 1,
        points: [
          [s.evaluate(u.first, v.first), s.evaluate(u.first, v.last)],
          [s.evaluate(u.last, v.first), s.evaluate(u.last, v.last)],
        ],
        knotsU: [0, 0, 1, 1],
        knotsV: [0, 0, 1, 1],
      );
    } else {
      throw BrepUnsupported('Surface cannot be converted to a spline');
    }
    return BrepSplineSurface(
      degreeU: 2,
      degreeV: degree,
      points: [
        for (final p in circle.points)
          [
            for (final q in meridian)
              origin + x * (p.x * q.x) + y * (p.y * q.x) + axis * q.z,
          ],
      ],
      weights: [
        for (final w in circle.weights) [for (final v in weights) w * v],
      ],
      knotsU: circle.knots,
      knotsV: knots,
    );
  }
}
