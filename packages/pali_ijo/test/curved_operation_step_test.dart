import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'ruled_quadric_intersection_test.dart' as fixture;
import 'bilinear_boolean_test.dart' as bilinear;

void main() {
  test('sideways bore through a curved loft exports and imports BREP', () {
    final source = fixture.loft();
    final tool = BrepPrimitives.cylinder(
      origin: Vector3(-8, .4, 5),
      axis: Vector3(1, 0, 0),
      radius: 1.2,
      height: 16,
    );
    final result = IjoOps.booleanSolid(source, tool, IjoOps.subtractOp);
    final back = IjoStep.readSolid(IjoStep.writeSolid(result));
    expect(BrepHeal.topologyIsClosed(back), isTrue);
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(back)), isTrue);
  });
  test('sliced bilinear loft exports exact rational geometry', () {
    final source = bilinear.twisted(Vector3.zero(), .4);
    final parts = IjoOps.sliceBrep(
      source,
      Vector3(0, 0, 3),
      Vector3(.1, .1, 1),
    );
    for (final p in parts) {
      final back = IjoStep.readSolid(IjoStep.writeSolid(p));
      expect(BrepHeal.topologyIsClosed(back), isTrue);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(back)), isTrue);
    }
  });
}
