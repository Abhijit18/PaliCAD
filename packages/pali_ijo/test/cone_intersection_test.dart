import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  final cone = BrepCone(
    origin: Vector3.zero(),
    axis: Vector3(0, 0, 1),
    radius: 10,
    topRadius: 0,
    height: 20,
    xDir: Vector3(1, 0, 0),
  );
  for (final reverse in [false, true]) {
    test('coaxial cone bore produces an exact circle reverse=$reverse', () {
      final cylinder = BrepCylinder(
        origin: Vector3(0, 0, -13),
        axis: Vector3(0, 0, reverse ? -1 : 1),
        radius: 7.5,
        xDir: Vector3(1, 0, 0),
      );
      final curves = BrepIntersect.surfaces(
        reverse ? cone : cylinder,
        reverse ? cylinder : cone,
      );
      expect(curves, hasLength(1));
      final circle = curves.single as BrepCircle;
      expect(circle.radius, closeTo(7.5, 1e-10));
      expect((circle.center - Vector3(0, 0, 5)).length, lessThan(1e-10));
      for (var i = 0; i < 20; i++) {
        final p = circle.evaluate(2 * math.pi * i / 20);
        expect((cone.closestPoint(p) - p).length, lessThan(1e-9));
        expect((cylinder.closestPoint(p) - p).length, lessThan(1e-9));
      }
    });
  }
  for (final offset in [0.0, 1.0, 40.0]) {
    test(
      'parallel cylinder outside bounded cone has no intersection $offset',
      () {
        final cylinder = BrepCylinder(
          origin: Vector3(offset, 0, 0),
          axis: Vector3(0, 0, 1),
          radius: 25,
          xDir: Vector3(1, 0, 0),
        );
        expect(BrepIntersect.surfaces(cylinder, cone), isEmpty);
      },
    );
  }
  test('unhandled crossing remains explicit', () {
    final cylinder = BrepCylinder(
      origin: Vector3(4, 0, 0),
      axis: Vector3(0, 0, 1),
      radius: 7.5,
      xDir: Vector3(1, 0, 0),
    );
    expect(
      () => BrepIntersect.surfaces(cylinder, cone),
      throwsA(isA<BrepUnsupported>()),
    );
  });
  test(
    'split circular trims have consistent ray crossings at their shared seam',
    () {
      final b = BrepBuilder();
      BrepLoop ring(double radius) {
        final a = b.vertex(Vector3(radius, 0, 0));
        final z = b.vertex(Vector3(-radius, 0, 0));
        return BrepLoop([
          BrepCoedge(
            b.edge(
              a,
              z,
              BrepCircle(
                center: Vector3.zero(),
                normal: Vector3(0, 0, 1),
                radius: radius,
                xDir: Vector3(1, 0, 0),
                t0: 0,
                t1: math.pi,
              ),
            ),
          ),
          BrepCoedge(
            b.edge(
              z,
              a,
              BrepCircle(
                center: Vector3.zero(),
                normal: Vector3(0, 0, 1),
                radius: radius,
                xDir: Vector3(1, 0, 0),
                t0: math.pi,
                t1: 2 * math.pi,
              ),
            ),
          ),
        ]);
      }

      b.face(
        BrepPlane(
          origin: Vector3.zero(),
          normal: Vector3(0, 0, 1),
          xDir: Vector3(1, 0, 0),
        ),
        [ring(20), ring(7.5)],
      );
      final solid = b.build();
      for (final y in [0.0, -1e-12, 1e-12]) {
        for (final x in [-30.0, -10.0, 0.0, 10.0, 30.0]) {
          expect(
            BrepClassify.containsPlaneFace(
              solid,
              solid.faces.single,
              Vector3(x, y, 0),
            ),
            x.abs() > 7.5 && x.abs() < 20,
            reason: 'query ($x, $y)',
          );
        }
      }
    },
  );
}
