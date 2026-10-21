import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/intersect.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid loft() => BrepLoft.sections([
  [Vector3(-5, -5, 0), Vector3(5, -5, 0), Vector3(5, 5, 0), Vector3(-5, 5, 0)],
  [
    for (var i = 0; i < 32; i++)
      Vector3(
        3 * math.cos(2 * math.pi * i / 32),
        3 * math.sin(2 * math.pi * i / 32),
        10,
      ),
  ],
]);
void main() {
  test(
    'curved loft wall and horizontal cylinder retain implicit shared curves',
    () {
      final source = loft();
      final cylinder = BrepPrimitives.cylinder(
        origin: Vector3(-8, .4, 5),
        axis: Vector3(1, 0, 0),
        radius: 1.2,
        height: 16,
      );
      var count = 0;
      for (final f in source.faces) {
        if (f.surface is! BrepRuledSurface) continue;
        final curves = BrepIntersect.ruledQuadricBranches(
          f.surface as BrepRuledSurface,
          cylinder.faces.firstWhere((f) => f.surface is BrepCylinder).surface,
        );
        for (final c in curves) {
          count++;
          for (var i = 0; i <= 20; i++) {
            final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 20);
            expect((f.surface.closestPoint(p) - p).length, lessThan(1e-7));
            expect(
              (cylinder.faces
                          .firstWhere((f) => f.surface is BrepCylinder)
                          .surface
                          .closestPoint(p) -
                      p)
                  .length,
              lessThan(1e-7),
            );
            final back = BrepCurve.fromJson(c.toJson());
            expect(
              (back.evaluate(c.t0 + (c.t1 - c.t0) * i / 20) - p).length,
              lessThan(1e-9),
            );
          }
        }
      }
      expect(count, greaterThan(0));
    },
  );
  for (final axis in [Vector3(1, 0, 0), Vector3(1, .2, .15).normalized()]) {
    for (final op in [IjoOps.subtractOp, IjoOps.unionOp, IjoOps.intersectOp]) {
      test('curved loft cylinder boolean $op axis=$axis stays closed BREP', () {
        final source = loft(), center = Vector3(0, .4, 5);
        final cylinder = BrepPrimitives.cylinder(
          origin: center - axis * 8,
          axis: axis,
          radius: 1.2,
          height: 16,
        );
        final result = IjoOps.booleanSolid(source, cylinder, op);
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        expect(
          result.edges.any((e) => e.curve is BrepRuledQuadricCurve),
          isTrue,
        );
        final mesh = BrepTessellate.run(result);
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        expect(MeshKit.signedVolume(mesh), greaterThan(0));
        expect(
          BrepClassify.side(result, center),
          op == IjoOps.subtractOp ? BrepSide.outside : BrepSide.inside,
        );
        expect(
          BrepClassify.side(result, Vector3(0, 2.5, 5)),
          op == IjoOps.intersectOp ? BrepSide.outside : BrepSide.inside,
        );
      });
    }
  }
}
