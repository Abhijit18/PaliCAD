import 'dart:convert';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final radius in [1.0, 2.0, 4.0]) {
    test(
      'all box edges share twelve rounds and eight corners at r=$radius',
      () {
        final box = BrepPrimitives.box(20, 20, 20);
        final original = jsonEncode(box.toJson());
        final rounded = IjoOps.filletSolid(
          box,
          BrepEdgeSelect.convex().resolve(box),
          radius: radius,
          inverse: false,
        );
        expect(
          rounded.faces.where((f) => f.surface is BrepCylinder),
          hasLength(12),
        );
        expect(
          rounded.faces.where((f) => f.surface is BrepSphere),
          hasLength(8),
        );
        expect(
          rounded.faces.where((f) => f.surface is BrepPlane),
          hasLength(6),
        );
        expect(BrepHeal.topologyIsClosed(rounded), isTrue);
        final core = 20 - 2 * radius;
        final exact =
            core * core * core +
            6 * core * core * radius +
            3 * math.pi * core * radius * radius +
            4 / 3 * math.pi * math.pow(radius, 3);
        final mesh = IjoOps.tessellateClosed(
          rounded,
          tessellation: MeshTessellation.fine,
        );
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        expect(IjoMeasure.volume(mesh), closeTo(exact, exact * .001));
        expect(jsonEncode(box.toJson()), original);
      },
    );
  }
  test('zero spherical wall thickness leaves the original geometry intact', () {
    final sphere = BrepPrimitives.sphere(center: Vector3.zero(), radius: 20);
    final mesh = BrepTessellate.run(sphere);
    final result = BrepHollow.fromMeshOpening(
      sphere,
      mesh,
      [0],
      Vector3(0, 0, 1),
      wallThicknessMm: 0,
    );
    expect(result.toJson(), equals(sphere.toJson()));
    expect(identical(result, sphere), isFalse);
  });
  for (final normal in [Vector3(0, 0, 1), Vector3(1, 2, 3).normalized()]) {
    test(
      'picked sphere opening retains an analytic hollow shell along $normal',
      () {
        final sphere = BrepPrimitives.sphere(
          center: Vector3(7, -3, 11),
          radius: 20,
        );
        final mesh = BrepTessellate.run(sphere);
        final picked = FaceSelect.extreme(normal).resolve(mesh);
        final hollow = BrepHollow.fromMeshOpening(
          sphere,
          mesh,
          picked,
          normal,
          wallThicknessMm: 1,
        );
        expect(
          hollow.faces.where((f) => f.surface is BrepSphere),
          hasLength(2),
        );
        expect(BrepHeal.topologyIsClosed(hollow), isTrue);
        final copy = BrepSolid.fromJson(
          jsonDecode(jsonEncode(hollow.toJson())),
        );
        expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(copy)), isTrue);
        expect(BrepClassify.isInside(copy, Vector3(7, -3, 11)), isFalse);
        expect(
          BrepClassify.isInside(copy, Vector3(7, -3, 11) - normal * 19.5),
          isTrue,
        );
      },
    );
  }
}
