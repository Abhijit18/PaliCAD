import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid bridge() {
  final a = BrepPrimitives.box(20, 20, 20);
  final s = BrepPrimitives.sphere(center: Vector3(22, 10, 10), radius: 8);
  return IjoOps.booleanSolid(
    IjoOps.booleanSolid(a, s, IjoOps.unionOp),
    BrepXform.translate(a, Vector3(24, 0, 0)),
    IjoOps.unionOp,
  );
}

void main() {
  test(
    'bridge membership follows material rather than nearest supporting face',
    () {
      final s = bridge();
      for (final p in [
        Vector3(10, 8, 10),
        Vector3(34, 10, 10),
        Vector3(22, 10, 10),
        Vector3(20, 10, 10),
        Vector3(24, 10, 10),
      ]) {
        expect(BrepClassify.side(s, p), BrepSide.inside, reason: '$p');
      }
      for (final p in [
        Vector3(22, 0, 0),
        Vector3(-1, 10, 10),
        Vector3(45, 10, 10),
        Vector3(22, 10, 19),
      ]) {
        expect(BrepClassify.side(s, p), BrepSide.outside, reason: '$p');
      }
      for (final p in [
        Vector3(0, 10, 10),
        Vector3(44, 10, 10),
        Vector3(22, 10, 18),
      ]) {
        expect(BrepClassify.side(s, p), BrepSide.on, reason: '$p');
      }
    },
  );
  test('classification follows a rotated and translated bridge', () {
    final rotation = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .73);
    Vector3 transform(Vector3 p) => rotation.rotated(p) + Vector3(31, -17, 9);
    final s = BrepXform.map(bridge(), transform);
    for (final p in [
      Vector3(10, 8, 10),
      Vector3(34, 10, 10),
      Vector3(22, 10, 10),
    ]) {
      expect(BrepClassify.side(s, transform(p)), BrepSide.inside);
    }
    for (final p in [Vector3(22, 0, 0), Vector3(22, 10, 19)]) {
      expect(BrepClassify.side(s, transform(p)), BrepSide.outside);
    }
    expect(BrepClassify.side(s, transform(Vector3(22, 10, 18))), BrepSide.on);
  });
  test('circle through left cube keeps its wall and the spherical bridge', () {
    final s = bridge();
    final cutter = BrepPrimitives.cylinder(
      origin: Vector3(8, 8, -60),
      axis: Vector3(0, 0, 1),
      radius: 2,
      height: 140,
    );
    final r = IjoOps.booleanSolid(s, cutter, IjoOps.subtractOp);
    expect(BrepHeal.topologyIsClosed(r), isTrue);
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(r)), isTrue);
    expect(r.faces.where((f) => f.surface is BrepCylinder), hasLength(1));
    expect(BrepClassify.side(r, Vector3(8, 8, 10)), BrepSide.outside);
    expect(BrepClassify.side(r, Vector3(12, 8, 10)), BrepSide.inside);
  });
}
