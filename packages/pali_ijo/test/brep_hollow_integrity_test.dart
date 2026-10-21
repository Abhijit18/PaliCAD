import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final bottom in [true, false]) {
    test(
      'circular opening rim faces outward when opening ${bottom ? "bottom" : "top"}',
      () {
        final solid = BrepPrimitives.cylinder(
          origin: Vector3.zero(),
          axis: Vector3(0, 0, 1),
          radius: 10,
          height: 8,
        );
        final direction = bottom ? -1.0 : 1.0;
        final opening = solid.faces.indexWhere(
          (f) =>
              f.surface is BrepPlane &&
              f.outwardNormal(0, 0).z * direction > .9,
        );
        final out = IjoOps.hollowSolid(
          solid,
          [opening],
          Vector3(0, 0, direction),
          wallThicknessMm: 1,
        );
        final mesh = IjoOps.tessellateClosed(out);
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        final rimZ = bottom ? 0.0 : 8.0;
        final rimTriangles = mesh.faces.where(
          (f) => f.every((i) => (mesh.vertices[i].z - rimZ).abs() < 1e-6),
        );
        expect(rimTriangles, isNotEmpty);
        for (final face in rimTriangles) {
          final a = mesh.vertices[face[0]],
              b = mesh.vertices[face[1]],
              c = mesh.vertices[face[2]];
          expect((b - a).cross(c - a).z * direction, greaterThan(0));
          expect(((a + b + c) / 3).xy.length, greaterThan(8.9));
        }
      },
    );
  }
  test('frustum hollow has constant perpendicular wall thickness', () {
    final solid = BrepPrimitives.frustum(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 10,
      topRadius: 5,
      height: 10,
    );
    final opening = solid.faces.indexWhere(
      (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > .9,
    );
    final out = IjoOps.hollowSolid(
      solid,
      [opening],
      Vector3(0, 0, 1),
      wallThicknessMm: 1,
    );
    final inner =
        out.faces
                .singleWhere((f) => f.surface is BrepCone && f.reversed)
                .surface
            as BrepCone;
    expect(10 - inner.radius, closeTo(math.sqrt(1.25), 1e-7));
    final mesh = IjoOps.tessellateClosed(out);
    expect(MeshKit.isClosedSolid(mesh), isTrue);
    expect(MeshKit.signedVolume(mesh), greaterThan(0));
  });
  test('acute planar walls meet their offset planes accurately', () {
    final solid = BrepExtrude.polygon([
      Vector3(0, 0, 0),
      Vector3(40, 0, 0),
      Vector3(3, 8, 0),
    ], Vector3(0, 0, 20));
    final top = solid.faces.indexWhere(
      (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > .9,
    );
    final out = IjoOps.hollowSolid(
      solid,
      [top],
      Vector3(0, 0, 1),
      wallThicknessMm: .3,
    );
    for (final face in out.faces) {
      for (final loop in face.loops) {
        for (final co in loop.coedges) {
          final e = out.edges[co.edge];
          for (final p in [out.vertices[e.v0], out.vertices[e.v1]]) {
            expect((face.surface.closestPoint(p) - p).length, lessThan(1e-6));
          }
        }
      }
    }
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(out)), isTrue);
  });
  test(
    'too-thick and nonfinite hollow is rejected without a mesh substitute',
    () {
      final solid = BrepPrimitives.box(10, 10, 10);
      final mesh = BrepTessellate.run(solid);
      for (final thickness in [6.0, double.nan, double.infinity]) {
        expect(
          () => IjoOps.hollowPreferSolid(
            mesh,
            [2],
            Vector3(0, 0, 1),
            solid: solid,
            wallThicknessMm: thickness,
          ),
          throwsA(isA<BrepUnsupported>()),
        );
      }
    },
  );
}
