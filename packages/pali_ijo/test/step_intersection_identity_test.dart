import 'dart:convert';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test(
    'STEP retains parent surfaces and restores native quadric intersections',
    () {
      final a = BrepPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 3,
        height: 10,
      );
      final b = BrepPrimitives.cylinder(
        origin: Vector3(-5, .2, 5),
        axis: Vector3(1, 0, 0),
        radius: 1,
        height: 10,
      );
      final source = IjoOps.booleanSolid(a, b, IjoOps.subtractOp);
      expect(source.edges.any((e) => e.curve is BrepIntersectionCurve), isTrue);
      final bytes = IjoStep.writeSolid(source);
      expect(utf8.decode(bytes), contains('INTERSECTION_CURVE('));
      final restored = IjoStep.readSolid(bytes);
      expect(
        restored.edges.where((e) => e.curve is BrepIntersectionCurve).length,
        source.edges.where((e) => e.curve is BrepIntersectionCurve).length,
      );
      expect(BrepHeal.topologyIsClosed(restored), isTrue);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)), isTrue);
      for (final e in restored.edges) {
        final c = e.curve;
        if (c is! BrepIntersectionCurve) continue;
        for (var i = 0; i <= 20; i++) {
          final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 20);
          expect((c.carrier.closestPoint(p) - p).length, lessThan(1e-9));
          expect((c.target.closestPoint(p) - p).length, lessThan(1e-9));
        }
      }
    },
  );
}
