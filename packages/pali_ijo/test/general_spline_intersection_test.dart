import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/spline_intersect.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSplineSurface bowl({double height = 0}) => BrepSplineSurface(
  degreeU: 2,
  degreeV: 2,
  points: [
    for (var i = 0; i < 3; i++)
      [
        for (var j = 0; j < 3; j++)
          Vector3(
            i - 1.0,
            j - 1.0,
            [1.0, -1.0, 1.0][i] + [1.0, -1.0, 1.0][j] + height,
          ),
      ],
  ],
  knotsU: [0, 0, 0, 1, 1, 1],
  knotsV: [0, 0, 0, 1, 1, 1],
);
BrepSplineSurface plane(double z) => BrepSplineSurface(
  degreeU: 1,
  degreeV: 1,
  points: [
    [Vector3(-1, -1, z), Vector3(-1, 1, z)],
    [Vector3(1, -1, z), Vector3(1, 1, z)],
  ],
  knotsU: [0, 0, 1, 1],
  knotsV: [0, 0, 1, 1],
);
void incidence(BrepSurfaceIntersectionCurve c) {
  for (var i = 0; i <= 80; i++) {
    final t = i / 80, uv = c.parametersAt(t), p = c.evaluate(t);
    expect(
      (BrepSurfaceIntersectionCurve.point(c.second, uv.z, uv.w) - p).length,
      lessThan(2e-9),
    );
  }
}

void main() {
  test('analytic rational derivatives agree at both domain endpoints', () {
    final s = bowl();
    for (final u in [0.0, .3, 1.0])
      for (final v in [0.0, .7, 1.0]) {
        final j = s.jet(u, v);
        expect((j.$2 - Vector3(2, 0, 8 * u - 4)).length, lessThan(1e-12));
        expect((j.$3 - Vector3(0, 2, 8 * v - 4)).length, lessThan(1e-12));
      }
  });
  test(
    'interior closed spline intersection is found without boundary seeds',
    () {
      final curves = BrepSplineIntersect.surfaces(bowl(), plane(.3));
      expect(curves.length, 1);
      final c = curves.single;
      incidence(c);
      expect((c.evaluate(0) - c.evaluate(1)).length, lessThan(1e-9));
      for (var i = 0; i <= 40; i++) {
        final p = c.evaluate(i / 40);
        expect(p.x * p.x + p.y * p.y, closeTo(.3, 2e-9));
        expect(p.z, closeTo(.3, 2e-9));
      }
      final restored = BrepCurve.fromJson(c.toJson());
      expect(
        (restored.evaluate(.37) - c.evaluate(.37)).length,
        lessThan(1e-12),
      );
    },
  );
  test('two general quadratic surfaces produce separate open branches', () {
    final b = BrepSplineSurface(
      degreeU: 2,
      degreeV: 2,
      points: [
        for (var i = 0; i < 3; i++)
          [
            for (var j = 0; j < 3; j++)
              Vector3(i - 1.0, j - 1.0, .3 + [1.0, -1.0, 1.0][j]),
          ],
      ],
      knotsU: [0, 0, 0, 1, 1, 1],
      knotsV: [0, 0, 0, 1, 1, 1],
    );
    final curves = BrepSplineIntersect.surfaces(bowl(), b);
    expect(curves.length, 2);
    for (final c in curves) {
      incidence(c);
      expect((c.evaluate(0).y - c.evaluate(1).y).abs(), closeTo(2, 1e-8));
      expect(c.evaluate(.3).x.abs(), closeTo(math.sqrt(.3), 1e-8));
    }
  });
  test(
    'disjoint spline patches with overlapping initial hulls return empty',
    () {
      expect(BrepSplineIntersect.surfaces(bowl(), plane(-.1)), isEmpty);
    },
  );
  test('coincident spline patches reject explicitly', () {
    expect(
      () => BrepSplineIntersect.surfaces(bowl(), bowl()),
      throwsA(isA<BrepUnsupported>()),
    );
  });
  test('spline ray intersections count both sides of a curved patch', () {
    final hits = BrepSplineIntersect.ray(
      bowl(),
      Vector3(-2, 0, .3),
      Vector3(1, 0, 0),
    )!..sort();
    expect(hits.length, 2);
    expect(hits.first, closeTo(2 - math.sqrt(.3), 1e-7));
    expect(hits.last, closeTo(2 + math.sqrt(.3), 1e-7));
  });
  test('small closed spline branch survives adaptive continuation', () {
    final curves = BrepSplineIntersect.surfaces(bowl(), plane(.0009));
    expect(curves.length, 1);
    incidence(curves.single);
  });
  test(
    'weighted nonuniform spline patches intersect in their native knot domains',
    () {
      final s = BrepSplineSurface(
        degreeU: 2,
        degreeV: 2,
        points: [
          for (var i = 0; i < 4; i++)
            [
              for (var j = 0; j < 3; j++)
                Vector3(
                  i.toDouble(),
                  j.toDouble(),
                  [1.0, -2.0, 2.0, -1.0][i] + [0.0, .3, 0.0][j],
                ),
            ],
        ],
        weights: [
          [1, 2, 1],
          [.8, 1.5, .8],
          [1.1, .5, 1.1],
          [1, 1, 1],
        ],
        knotsU: [2, 2, 2, 3.1, 5, 5, 5],
        knotsV: [-1, -1, -1, 2, 2, 2],
      );
      final cuts = BrepIntersect.surfaces(
        s,
        BrepPlane(origin: Vector3.zero(), normal: Vector3(0, 0, 1)),
      );
      expect(cuts.length, 3);
      for (final c in cuts) incidence(c as BrepSurfaceIntersectionCurve);
    },
  );
  test(
    'discontinuous spline domains reject instead of bridging disconnected branches',
    () {
      final s = BrepSplineSurface(
        degreeU: 1,
        degreeV: 1,
        points: [
          [Vector3(0, 0, 0), Vector3(0, 1, 0)],
          [Vector3(1, 0, 0), Vector3(1, 1, 0)],
          [Vector3(3, 0, 1), Vector3(3, 1, 1)],
          [Vector3(4, 0, 1), Vector3(4, 1, 1)],
        ],
        knotsU: [0, 0, .5, .5, 1, 1],
        knotsV: [0, 0, 1, 1],
      );
      expect(
        () => BrepSplineIntersect.surfaces(s, plane(.5)),
        throwsA(isA<BrepUnsupported>()),
      );
    },
  );
}
