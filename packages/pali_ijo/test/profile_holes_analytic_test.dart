import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

List<Vector3> circle(double r) => [
  for (var i = 0; i < 64; i++)
    Vector3(r * math.cos(i * math.pi / 32), r * math.sin(i * math.pi / 32), 0),
];
void main() {
  for (final direction in [
    Vector3(0, 0, 3),
    Vector3(0, 0, -3),
    Vector3(1, 2, 3),
  ]) {
    test('analytic annulus with reversed inner winding swept $direction', () {
      final s = BrepExtrude.profileWithHoles(circle(8), [
        circle(3).reversed.toList(),
      ], direction);
      expect(BrepHeal.topologyIsClosed(s), isTrue);
      expect(s.faces, hasLength(4));
      final mesh = IjoOps.tessellateClosed(
        s,
        tessellation: MeshTessellation.fine,
      );
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(MeshKit.signedVolume(mesh), closeTo(math.pi * (64 - 9) * 3, 3));
      expect(BrepClassify.isInside(s, direction * .5), isFalse);
      expect(
        BrepClassify.isInside(s, Vector3(5, 0, 0) + direction * .5),
        isTrue,
      );
    });
  }
}
