import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('rational quadratic exactly represents a circular quadrant', () {
    final c = BrepSplineCurve(
      degree: 2,
      points: [Vector3(1, 0, 0), Vector3(1, 1, 0), Vector3(0, 1, 0)],
      knots: [0, 0, 0, 1, 1, 1],
      weights: [1, math.sqrt(.5), 1],
    );
    for (var i = 0; i <= 32; i++)
      expect(c.evaluate(i / 32).length, closeTo(1, 1e-12));
    final p = c.evaluate(.31);
    expect((c.closestPoint(p * 1.2) - p).length, lessThan(1e-7));
    final back = BrepCurve.fromJson(c.toJson());
    expect((back.evaluate(.31) - p).length, lessThan(1e-12));
  });
  test(
    'bilinear surface derivatives and nearest points agree with geometry',
    () {
      final s = BrepSplineSurface(
        degreeU: 1,
        degreeV: 1,
        points: [
          [Vector3.zero(), Vector3(0, 2, 0)],
          [Vector3(2, 0, 0), Vector3(2, 2, 2)],
        ],
        knotsU: [0, 0, 1, 1],
        knotsV: [0, 0, 1, 1],
      );
      final p = s.evaluate(.3, .4), n = Vector3(-.4, -.3, 1).normalized();
      expect((p - Vector3(.6, .8, .24)).length, lessThan(1e-12));
      expect(s.normalAt(.3, .4).dot(n), closeTo(1, 1e-10));
      expect((s.closestPoint(p + n * .1) - p).length, lessThan(1e-6));
      final t = BrepSurface.fromJson(s.toJson());
      expect((t.evaluate(.3, .4) - p).length, lessThan(1e-12));
    },
  );
  test('malformed spline domains and nonpositive weights reject', () {
    for (final knots in <List<double>>[
      [],
      [0, 1],
      [0, 0, 2, 1],
      [0, 0, 0, 0],
    ]) {
      expect(
        () => BrepSplineCurve(
          degree: 1,
          points: [Vector3.zero(), Vector3(1, 0, 0)],
          knots: knots,
        ),
        throwsA(isA<BrepUnsupported>()),
      );
    }
    expect(
      () => BrepSplineCurve(
        degree: 1,
        points: [Vector3.zero(), Vector3(1, 0, 0)],
        knots: [0, 0, 1, 1],
        weights: [1, 0],
      ),
      throwsA(isA<BrepUnsupported>()),
    );
  });

  test('closed spline trim crosses its periodic seam without reversing', () {
    final c = BrepSplineCurve(
      degree: 1,
      points: [
        Vector3(1, 0, 0),
        Vector3(0, 1, 0),
        Vector3(-1, 0, 0),
        Vector3(0, -1, 0),
        Vector3(1, 0, 0),
      ],
      knots: [0, 0, .25, .5, .75, 1, 1],
    );
    final section = c.trimmed(.875, 1.125);
    expect((section.evaluate(1) - Vector3(1, 0, 0)).length, lessThan(1e-12));
    expect(
      (section.evaluate(1.125) - Vector3(.5, .5, 0)).length,
      lessThan(1e-12),
    );
    expect(section.closestParam(Vector3(.75, .25, 0)), closeTo(1.0625, 1e-7));
  });
}
