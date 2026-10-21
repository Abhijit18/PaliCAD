import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  final wall = BrepRuledSurface(
    BrepLine(Vector3(-2, 0, -.8), Vector3(2, 0, -.8)),
    BrepLine(Vector3(-2, 0, .8), Vector3(2, 0, .8)),
  );
  final targets = <BrepSurface>[
    BrepSphere(center: Vector3(.2, 0, 0), radius: .7),
    BrepCone(
      origin: Vector3(0, 0, -1),
      axis: Vector3(0, 0, 1),
      radius: 1,
      topRadius: .4,
      height: 2,
    ),
    BrepAffineSurface(
      BrepSphere(center: Vector3.zero(), radius: .7),
      BrepAffineTransform(
        Vector3(1.5, 0, 0),
        Vector3(0, 1, 0),
        Vector3(0, 0, .8),
        Vector3(.1, 0, 0),
      ),
    ),
    BrepCylinder(
      origin: Vector3(.3, -1, -.8),
      axis: Vector3(0, .3, 1),
      radius: .6,
    ),
  ];
  for (final target in targets) {
    test(
      'bounded ruled/${target.runtimeType} branches satisfy both equations',
      () {
        final curves = BrepIntersect.surfaces(wall, target);
        expect(curves, isNotEmpty);
        for (final c in curves)
          for (var i = 0; i <= 30; i++) {
            final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 30);
            expect((wall.closestPoint(p) - p).length, lessThan(1e-7));
            expect((target.closestPoint(p) - p).length, lessThan(1e-7));
          }
      },
    );
  }
}
