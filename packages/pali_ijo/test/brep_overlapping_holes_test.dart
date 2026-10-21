import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'sphere_bridge_acceptance_test.dart' as fixture;

void main() {
  for (final withBridge in [false, true]) {
    test(
      'overlapping circular holes retain analytic walls (bridge=$withBridge)',
      () {
        final base = withBridge
            ? fixture.bridge()
            : BrepPrimitives.box(20, 20, 20);
        final initialVolume = MeshKit.signedVolume(
          IjoOps.tessellateClosed(base),
        );
        BrepSolid cut(BrepSolid s, double x) => IjoOps.booleanSolid(
          s,
          BrepPrimitives.cylinder(
            origin: Vector3(x, 8, -60),
            axis: Vector3(0, 0, 1),
            radius: 2,
            height: 140,
          ),
          IjoOps.subtractOp,
        );
        final first = cut(base, 8);
        final result = cut(first, 10);
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        expect(IjoOps.splitBrep(result), hasLength(1));
        final mesh = IjoOps.tessellateClosed(result);
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        final overlapArea = 8 * math.acos(.5) - math.sqrt(12);
        expect(
          MeshKit.signedVolume(mesh),
          closeTo(initialVolume - 20 * (8 * math.pi - overlapArea), 2),
        );
        for (final p in [
          Vector3(8, 8, 10),
          Vector3(9, 8, 10),
          Vector3(10, 8, 10),
        ]) {
          expect(BrepClassify.side(result, p), BrepSide.outside);
        }
        for (final face in result.faces) {
          for (final loop in face.loops) {
            for (final co in loop.coedges) {
              final curve = result.edges[co.edge].curve;
              for (final t in [.2, .5, .8]) {
                final p = curve.evaluate(curve.t0 + t * (curve.t1 - curve.t0));
                expect(
                  (face.surface.closestPoint(p) - p).length,
                  lessThan(1e-5),
                );
              }
            }
          }
        }
      },
    );
  }
}
