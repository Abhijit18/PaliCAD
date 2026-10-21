import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final radius in [2.0, 100.0]) {
    test('sphere slice covers curved bounds at radius $radius', () {
      final solid = BrepPrimitives.sphere(
        center: Vector3(31, -17, 9),
        radius: radius,
      );
      final parts = IjoOps.sliceBrep(
        solid,
        Vector3(31, -17, 9),
        Vector3(1, 2, 3),
      );
      expect(parts, hasLength(2));
      var volume = 0.0;
      for (final part in parts) {
        expect(BrepHeal.topologyIsClosed(part), isTrue);
        final mesh = IjoOps.tessellateClosed(part);
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        volume += MeshKit.signedVolume(mesh);
      }
      final exact = 4 * math.pi * radius * radius * radius / 3;
      expect(volume, closeTo(exact, exact * .04));
    });
  }
  test('slightly oblique cylinder cut preserves the actual cut plane', () {
    final solid = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 4,
      height: 20,
    );
    final n = Vector3(.02, 0, 1).normalized();
    final origin = Vector3(0, 0, 10);
    final parts = IjoOps.sliceBrep(solid, origin, n);
    expect(parts, hasLength(2));
    for (final part in parts) {
      final cut = part.faces
          .where((f) => f.surface is BrepPlane)
          .map((f) => f.surface as BrepPlane)
          .where((p) => p.height(origin).abs() < 1e-6)
          .toList();
      expect(cut, isNotEmpty);
      expect(cut.every((p) => p.normal.cross(n).length < 1e-8), isTrue);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(part)), isTrue);
    }
  });
  test('distant slice is an unchanged independent BREP', () {
    final solid = BrepPrimitives.box(10, 20, 30);
    final parts = IjoOps.sliceBrep(solid, Vector3(1e8, 0, 0), Vector3(1, 0, 0));
    expect(parts, hasLength(1));
    expect(parts.single.toJson(), solid.toJson());
    expect(identical(parts.single, solid), isFalse);
  });
  test('slice rejects invalid planes', () {
    final solid = BrepPrimitives.box(1, 1, 1);
    for (final n in [Vector3.zero(), Vector3(double.nan, 0, 1)]) {
      expect(
        () => IjoOps.sliceBrep(solid, Vector3.zero(), n),
        throwsA(isA<BrepUnsupported>()),
      );
    }
  });
}
