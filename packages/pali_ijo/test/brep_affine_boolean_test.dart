import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid ellipsoid(Vector3 scale, Vector3 translation) {
  return BrepXform.translate(
    BrepXform.scaleAbout(
      BrepPrimitives.sphere(center: Vector3.zero(), radius: 3),
      Vector3.zero(),
      scale,
    ),
    translation,
  );
}

void main() {
  for (final op in [IjoOps.subtractOp, IjoOps.unionOp, IjoOps.intersectOp])
    test('translated ellipsoid boolean $op stays BREP', () {
      final a = ellipsoid(Vector3(2, 1, .5), Vector3.zero()),
          b = ellipsoid(Vector3(2, 1, .5), Vector3(3, 0, 0));
      final result = IjoOps.booleanSolid(a, b, op);
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)), isTrue);
      expect(result.faces.every((f) => f.surface is BrepAffineSurface), isTrue);
    });
  test(
    'disjoint differently stretched solids compose and subtract without trimming',
    () {
      final a = ellipsoid(Vector3(2, 1, .5), Vector3.zero()),
          b = ellipsoid(Vector3(.5, 2, 1), Vector3(30, 0, 0));
      final union = IjoOps.booleanSolid(a, b, IjoOps.unionOp);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(union)), isTrue);
      expect(IjoOps.splitBrep(union).length, 2);
      final difference = IjoOps.booleanSolid(a, b, IjoOps.subtractOp);
      expect(difference.faces.length, a.faces.length);
    },
  );
}
