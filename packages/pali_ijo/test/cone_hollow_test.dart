import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final axis in [Vector3(0, 0, 1), Vector3(1, -2, -3).normalized()]) {
    for (final openBase in [true, false]) {
      test(
        'cone hollow retains analytic walls (axis $axis, open $openBase)',
        () {
          final origin = Vector3(12, -5, 7);
          final solid = BrepPrimitives.cone(
            origin: origin,
            axis: axis,
            radius: 10,
            height: 20,
          );
          final before = solid.toJson();
          final opening = solid.faces.indexWhere((f) => f.surface is BrepPlane);
          final hollow = IjoOps.hollowSolid(
            solid,
            openBase ? [opening] : [],
            -axis,
            wallThicknessMm: 1,
          );
          expect(solid.toJson(), before);
          final cone =
              hollow.faces
                      .singleWhere((f) => f.surface is BrepCone && f.reversed)
                      .surface
                  as BrepCone;
          final dr = math.sqrt(1.25);
          expect(cone.radius, closeTo(10 - dr, 1e-8));
          expect(cone.topRadius, 0);
          expect(cone.height, closeTo(20 - dr * 2, 1e-8));
          expect((10 - cone.radius) / math.sqrt(1.25), closeTo(1, 1e-8));
          final apex = cone.origin + cone.axis * cone.height;
          expect(hollow.vertices.any((v) => (v - apex).length < 1e-7), isTrue);
          for (final face in hollow.faces) {
            for (final loop in face.loops) {
              for (final co in loop.coedges) {
                final edge = hollow.edges[co.edge];
                for (final p in [
                  hollow.vertices[edge.v0],
                  hollow.vertices[edge.v1],
                ]) {
                  expect(
                    (face.surface.closestPoint(p) - p).length,
                    lessThan(1e-6),
                  );
                }
              }
            }
          }
          final mesh = IjoOps.tessellateClosed(hollow);
          expect(MeshKit.isClosedSolid(mesh), isTrue);
          final innerRadius = cone.radiusAt(openBase ? 0 : 1);
          final innerHeight = cone.height - (openBase ? 0 : 1);
          final expectedVolume =
              math.pi /
              3 *
              (100 * 20 - innerRadius * innerRadius * innerHeight);
          expect(
            MeshKit.signedVolume(mesh),
            closeTo(expectedVolume, expectedVolume * .015),
          );
          final restored = BrepSolid.fromJson(hollow.toJson());
          expect(
            MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)),
            isTrue,
          );
        },
      );
    }
  }

  test('cone hollow supports a surface parameterized from its apex', () {
    final solid = BrepPrimitives.cone(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 10,
      height: 20,
    );
    final face = solid.faces.singleWhere((f) => f.surface is BrepCone);
    final original = face.surface as BrepCone;
    face.surface = BrepCone(
      origin: Vector3(0, 0, 20),
      axis: Vector3(0, 0, -1),
      radius: 0,
      topRadius: 10,
      height: 20,
      xDir: original.xDir,
    );
    final hollow = IjoOps.hollowSolid(
      solid,
      [0],
      Vector3(0, 0, -1),
      wallThicknessMm: 1,
    );
    final inner =
        hollow.faces
                .singleWhere((f) => f.surface is BrepCone && f.reversed)
                .surface
            as BrepCone;
    expect(inner.radius, 0);
    expect(inner.topRadius, closeTo(10 - math.sqrt(1.25), 1e-8));
    expect(inner.origin.z, closeTo(20 - math.sqrt(5), 1e-8));
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(hollow)), isTrue);
  });

  for (final dimensions in [
    (20.0, 2.0, 1.0),
    (2.0, 100.0, 1.0),
    (10.0, 20.0, 8.8),
  ]) {
    test('cone hollow handles wide, tall and thick-wall cones $dimensions', () {
      final (radius, height, wall) = dimensions;
      final solid = BrepPrimitives.cone(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: radius,
        height: height,
      );
      final hollow = IjoOps.hollowSolid(
        solid,
        [0],
        Vector3(0, 0, -1),
        wallThicknessMm: wall,
      );
      final mesh = IjoOps.tessellateClosed(hollow);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(MeshKit.signedVolume(mesh), greaterThan(0));
      final inner =
          hollow.faces
                  .singleWhere((f) => f.surface is BrepCone && f.reversed)
                  .surface
              as BrepCone;
      expect(
        (radius - inner.radius) /
            math.sqrt(1 + radius * radius / (height * height)),
        closeTo(wall, 1e-8),
      );
    });
  }

  test('cone hollow rejects a wall that removes the cavity', () {
    final solid = BrepPrimitives.cone(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 10,
      height: 20,
    );
    expect(
      () =>
          IjoOps.hollowSolid(solid, [0], Vector3(0, 0, -1), wallThicknessMm: 9),
      throwsA(isA<BrepUnsupported>()),
    );
  });
}
