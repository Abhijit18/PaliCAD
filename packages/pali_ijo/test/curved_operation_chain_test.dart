import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'ruled_quadric_intersection_test.dart' as fixture;

void main() {
  test(
    'a loft wall bore supports a subsequent BREP slice and affine transform',
    () {
      final a = fixture.loft(),
          b = BrepPrimitives.cylinder(
            origin: Vector3(-8, .4, 5),
            axis: Vector3(1, 0, 0),
            radius: 1.2,
            height: 16,
          );
      final cut = IjoOps.booleanSolid(a, b, IjoOps.subtractOp);
      final halves = IjoOps.sliceBrep(cut, Vector3(0, 0, 5), Vector3(.1, 0, 1));
      expect(halves.length, 2);
      for (final h in halves) {
        final scaled = BrepXform.scaleAbout(
          h,
          Vector3.zero(),
          Vector3(2, 1, .5),
        );
        expect(BrepHeal.topologyIsClosed(scaled), isTrue);
        expect(MeshKit.isClosedSolid(BrepTessellate.run(scaled)), isTrue);
      }
    },
  );
  test(
    'stationary root isolation finds a branch narrower than its seed spacing',
    () {
      final roots = BrepIntersect.unitRoots(
        (u) => (u - .371234) * (u - .371234) - 1e-8,
      );
      expect(roots.length, 2);
      expect(roots[0], closeTo(.371134, 1e-9));
      expect(roots[1], closeTo(.371334, 1e-9));
    },
  );
  test(
    'arc closest point respects descending trims and roundoff at endpoints',
    () {
      final arc = BrepCircle(
        center: Vector3.zero(),
        normal: Vector3(0, 0, 1),
        radius: 2,
        t0: .2,
        t1: 1.8,
      );
      expect(
        (arc.closestPoint(arc.evaluate(.2)) - arc.evaluate(.2)).length,
        lessThan(1e-12),
      );
      final reverse = BrepCircle(
        center: arc.center,
        normal: arc.normal,
        radius: 2,
        xDir: arc.xDir,
        t0: 1.8,
        t1: .2,
      );
      for (final t in [.2, .7, 1.8])
        expect(
          (reverse.closestPoint(arc.evaluate(t)) - arc.evaluate(t)).length,
          lessThan(1e-12),
        );
    },
  );
}
