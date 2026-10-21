import 'dart:math' as math;
import 'package:vector_math/vector_math_64.dart';
import 'geom.dart';
import 'solid.dart';

/// Rational ruled surface algebra. Products of Bernstein coefficients give a
/// common denominator for two rational boundaries without curve fitting.
class BrepRationalRuled {
  static BrepSplineSurface build(BrepSplineCurve lower, BrepSplineCurve upper) {
    final cuts = <double>{0, 1};
    for (final c in [lower, upper]) {
      if (c.t0 == c.t1) throw BrepUnsupported('Collapsed ruled boundary');
      for (final k in c.knots) {
        final t = (k - c.t0) / (c.t1 - c.t0);
        if (t > 1e-12 && t < 1 - 1e-12) cuts.add(t);
      }
    }
    final ts = cuts.toList()..sort();
    final spans = [
      for (var i = 1; i < ts.length; i++)
        (
          homogeneousSpan(lower, ts[i - 1], ts[i]),
          homogeneousSpan(upper, ts[i - 1], ts[i]),
        ),
    ];
    final common =
        lower.degree == upper.degree &&
        spans.every((span) {
          final ratio = span.$1.first.w / span.$2.first.w;
          return List.generate(span.$1.length, (i) => i).every(
            (i) =>
                (span.$1[i].w - span.$2[i].w * ratio).abs() <
                1e-13 * span.$1[i].w.abs(),
          );
        });
    final degree = common ? lower.degree : lower.degree + upper.degree;
    final rows = <List<Vector3>>[], weights = <List<double>>[];
    final knots = List<double>.filled(degree + 1, 0, growable: true);
    for (var k = 1; k < ts.length; k++) {
      final (a, b) = spans[k - 1];
      final aa = common ? a : _product(a, b.map((p) => p.w).toList());
      final bb = common
          ? b.map((p) => p * (a.first.w / b.first.w)).toList()
          : _product(b, a.map((p) => p.w).toList());
      for (var i = k == 1 ? 0 : 1; i <= degree; i++) {
        final wa = aa[i].w, wb = bb[i].w;
        if (wa <= 0 || wb <= 0)
          throw BrepUnsupported('Invalid rational ruled weight');
        rows.add([
          Vector3(aa[i].x, aa[i].y, aa[i].z) / wa,
          Vector3(bb[i].x, bb[i].y, bb[i].z) / wb,
        ]);
        weights.add([wa, wb]);
      }
      knots.addAll(
        List.filled(k == ts.length - 1 ? degree + 1 : degree, ts[k]),
      );
    }
    return BrepSplineSurface(
      degreeU: degree,
      degreeV: 1,
      points: rows,
      weights: weights,
      knotsU: knots,
      knotsV: [0, 0, 1, 1],
    );
  }

  /// Eliminate the ruling coordinate with the plane equation in homogeneous
  /// coordinates. No sampled fit is involved in the resulting rational curve.
  static BrepSplineCurve section(
    BrepSplineSurface surface,
    BrepPlane plane,
    double start,
    double end,
  ) {
    BrepSplineCurve column(int index) => BrepSplineCurve(
      degree: surface.degreeU,
      points: surface.points.map((r) => r[index]).toList(),
      knots: surface.knotsU,
      weights: surface.weights.map((r) => r[index]).toList(),
      start: start,
      end: end,
    );
    final lower = column(0), upper = column(1), cuts = <double>{0, 1};
    for (final k in surface.knotsU) {
      final u = (k - start) / (end - start);
      if (u > 1e-12 && u < 1 - 1e-12) cuts.add(u);
    }
    final ts = cuts.toList()..sort(), degree = 2 * surface.degreeU;
    final points = <Vector3>[], weights = <double>[];
    final knots = List<double>.filled(degree + 1, 0, growable: true);
    double height(Vector4 p) =>
        plane.normal.dot(Vector3(p.x, p.y, p.z)) -
        plane.normal.dot(plane.origin) * p.w;
    void append(List<Vector4> control, double lo, double hi, int depth) {
      final scale = control.map((p) => p.w.abs()).fold(0.0, math.max);
      if (scale == 0 || !scale.isFinite)
        throw BrepUnsupported(
          'Plane section has no finite isolated rational curve',
        );
      final positive = control.every((p) => p.w > scale * 1e-13),
          negative = control.every((p) => p.w < -scale * 1e-13);
      if (!positive && !negative) {
        if (depth >= 20)
          throw BrepUnsupported(
            'Plane section has an unresolved rational pole',
          );
        final left = <Vector4>[control.first], right = <Vector4>[control.last];
        var row = control;
        while (row.length > 1) {
          row = [
            for (var i = 1; i < row.length; i++) (row[i - 1] + row[i]) * .5,
          ];
          left.add(row.first);
          right.add(row.last);
        }
        final mid = (lo + hi) / 2;
        append(left, lo, mid, depth + 1);
        append(right.reversed.toList(), mid, hi, depth + 1);
        return;
      }
      for (var i = points.isEmpty ? 0 : 1; i < control.length; i++) {
        final p = control[i];
        points.add(Vector3(p.x, p.y, p.z) / p.w);
        weights.add(p.w.abs());
      }
      knots.addAll(List.filled(hi == 1 ? degree + 1 : degree, hi));
    }

    for (var i = 1; i < ts.length; i++) {
      final a = homogeneousSpan(lower, ts[i - 1], ts[i]),
          b = homogeneousSpan(upper, ts[i - 1], ts[i]);
      final x = _product(a, b.map(height).toList()),
          y = _product(b, a.map(height).toList());
      append(
        [for (var j = 0; j < x.length; j++) x[j] - y[j]],
        ts[i - 1],
        ts[i],
        0,
      );
    }
    return BrepSplineCurve(
      degree: degree,
      points: points,
      knots: knots,
      weights: weights,
    );
  }

  static double _choose(int n, int k) {
    var v = 1.0;
    for (var i = 1; i <= k; i++) v = v * (n - i + 1) / i;
    return v;
  }

  static List<Vector4> _product(List<Vector4> a, List<double> b) {
    final p = a.length - 1, q = b.length - 1;
    return [
      for (var k = 0; k <= p + q; k++)
        () {
          var c = Vector4.zero();
          for (var i = math.max(0, k - q); i <= math.min(p, k); i++) {
            final j = k - i;
            c +=
                a[i] *
                (b[j] * _choose(p, i) * _choose(q, j) / _choose(p + q, k));
          }
          return c;
        }(),
    ];
  }

  /// Insert each span endpoint to full interior multiplicity in homogeneous
  /// coordinates. Knot insertion preserves the curve at every parameter.
  static List<Vector4> homogeneousSpan(BrepSplineCurve c, double a, double b) {
    final p = c.degree;
    var points = [
      for (var i = 0; i < c.points.length; i++)
        Vector4(
          c.points[i].x * c.weights[i],
          c.points[i].y * c.weights[i],
          c.points[i].z * c.weights[i],
          c.weights[i],
        ),
    ];
    final knots = List<double>.of(c.knots);
    final x = c.t0 + (c.t1 - c.t0) * a, y = c.t0 + (c.t1 - c.t0) * b;
    final lo = math.min(x, y), hi = math.max(x, y);
    if (lo < knots[p] - 1e-12 || hi > knots[points.length] + 1e-12) {
      throw BrepUnsupported(
        'Wrapped rational ruled boundaries need a seam split',
      );
    }
    for (final t in [lo, hi]) {
      var count = knots.where((v) => (v - t).abs() < 1e-13).length;
      while (count < p) {
        var k = p;
        while (k < points.length - 1 && knots[k + 1] <= t) k++;
        final out = List<Vector4>.generate(
          points.length + 1,
          (_) => Vector4.zero(),
        );
        for (var i = 0; i <= k - p; i++) out[i] = points[i];
        for (var i = k - count; i < points.length; i++) out[i + 1] = points[i];
        for (var i = k - p + 1; i <= k - count; i++) {
          final alpha = (t - knots[i]) / (knots[i + p] - knots[i]);
          out[i] = points[i] * alpha + points[i - 1] * (1 - alpha);
        }
        points = out;
        knots.insert(k + 1, t);
        count++;
      }
    }
    var k = p;
    final mid = (lo + hi) / 2;
    while (k < points.length - 1 && knots[k + 1] <= mid) k++;
    final result = points.sublist(k - p, k + 1);
    return x <= y ? result : result.reversed.toList();
  }
}
