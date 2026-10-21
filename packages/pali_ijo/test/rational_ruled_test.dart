import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/spline_convert.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test(
    'ruled rational curves with different knots and weights convert algebraically',
    () {
      final a = BrepSplineCurve(
        degree: 2,
        points: [
          Vector3(0, 0, 0),
          Vector3(2, 3, 0),
          Vector3(4, -1, 0),
          Vector3(6, 0, 0),
        ],
        knots: [0, 0, 0, .4, 1, 1, 1],
        weights: [1, .7, 1.4, 1],
        start: .1,
        end: .95,
      );
      final b = BrepSplineCurve(
        degree: 3,
        points: [
          Vector3(0, 1, 5),
          Vector3(1, 4, 6),
          Vector3(3, -1, 4),
          Vector3(5, 4, 5),
          Vector3(6, 1, 6),
        ],
        knots: [0, 0, 0, 0, .7, 1, 1, 1, 1],
        weights: [1, 1.3, .6, .9, 1],
        start: .95,
        end: .05,
      );
      final source = BrepRuledSurface(a, b),
          converted = BrepSplineConvert.exactRuled(source)!;
      for (var i = 0; i <= 100; i++)
        for (var j = 0; j <= 5; j++) {
          expect(
            (source.evaluate(i / 100, j / 5) -
                    converted.evaluate(i / 100, j / 5))
                .length,
            lessThan(1e-11),
          );
        }
    },
  );
  test(
    'matching angular conics share an exact rational reparameterization',
    () {
      final a = BrepCircle(
        center: Vector3.zero(),
        normal: Vector3(0, 0, 1),
        radius: 3,
        t0: .2,
        t1: 1.8,
      );
      final b = BrepEllipse(
        center: Vector3(1, 0, 5),
        normal: Vector3(0, 0, 1),
        xDir: Vector3(1, 0, 0),
        rMajor: 4,
        rMinor: 2,
        t0: 2,
        t1: 3.6,
      );
      final surface = BrepSplineConvert.exactRuled(BrepRuledSurface(a, b))!;
      for (var i = 0; i <= 50; i++) {
        final t = i / 50,
            p = surface.evaluate(t, 0),
            u = (a.closestParam(p) - a.t0) / (a.t1 - a.t0);
        expect(
          (surface.evaluate(t, .37) - BrepRuledSurface(a, b).evaluate(u, .37))
              .length,
          lessThan(1e-10),
        );
      }
    },
  );
  test('plane sections of bilinear walls have exact rational STEP curves', () {
    final surface = BrepRuledSurface(
      BrepLine(Vector3(0, 0, 0), Vector3(6, 1, 0)),
      BrepLine(Vector3(1, 2, 7), Vector3(5, -1, 6)),
    );
    final plane = BrepPlane(
      origin: Vector3(0, 0, 3),
      normal: Vector3(.1, .2, 1),
    );
    final section = BrepPlaneSectionCurve(
      surface: surface,
      plane: plane,
      t0: .1,
      t1: .9,
    );
    final spline = BrepSplineConvert.curve(section);
    expect(spline.degree, 2);
    for (var i = 0; i <= 100; i++) {
      final p = spline.evaluate(i / 100);
      expect(plane.height(p).abs(), lessThan(1e-11));
      expect((surface.closestPoint(p) - p).length, lessThan(1e-8));
      expect((section.evaluate(.1 + .8 * i / 100) - p).length, lessThan(1e-9));
    }
  });
  test(
    'angular and linear correspondences are not silently reparameterized',
    () {
      final source = BrepRuledSurface(
        BrepLine(Vector3.zero(), Vector3(5, 0, 0)),
        BrepCircle(
          center: Vector3(0, 0, 5),
          normal: Vector3(0, 0, 1),
          radius: 3,
          t0: 0,
          t1: math.pi / 2,
        ),
      );
      expect(BrepSplineConvert.exactRuled(source), isNull);
    },
  );
  test('fitted ruled rails share the implicit boundary parameter mapping', () {
    final carrier = BrepRuledSurface(
      BrepLine(Vector3(-2, 0, -1), Vector3(2, 0, -1)),
      BrepLine(Vector3(-2, 0, 1), Vector3(2, 0, 1)),
    );
    final arc = BrepRuledQuadricCurve(
      surface: carrier,
      target: BrepSphere(center: Vector3.zero(), radius: 1),
      branch: 1,
      t0: .25,
      t1: .75,
    );
    final source = BrepRuledSurface(
      arc,
      BrepLine(Vector3(-1, 2, 2), Vector3(1, 2, 2)),
    );
    final converted = BrepSplineConvert.surface(source, tolerance: 1e-6);
    for (var i = 0; i <= 200; i++) {
      final t = i / 200, u = math.pow(math.sin(math.pi * t / 2), 2).toDouble();
      for (var j = 0; j <= 5; j++) {
        expect(
          (source.evaluate(u, j / 5) - converted.evaluate(t, j / 5)).length,
          lessThan(1e-6),
        );
      }
    }
  });
}
