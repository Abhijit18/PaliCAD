import 'dart:convert';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (var axis = 0; axis < 3; axis++) {
    for (var partIndex = 0; partIndex < 2; partIndex++) {
      for (var faceIndex = 0; faceIndex < 6; faceIndex++) {
        test('hollow split box axis=$axis part=$partIndex face=$faceIndex', () {
          final original = BrepPrimitives.box(80, 60, 50);
          final normal = Vector3.zero()..[axis] = 1;
          final parts = IjoOps.sliceBrep(original, Vector3(40, 30, 25), normal);
          expect(parts, hasLength(2));
          final part = parts[partIndex];
          expect(part.faces, hasLength(6));
          expect(BrepHeal.topologyIsClosed(part), isTrue);
          expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(part)), isTrue);
          final openingNormal = part.faces[faceIndex].outwardNormal(0, 0);
          final hollow = IjoOps.hollowSolid(
            part,
            [faceIndex],
            openingNormal,
            wallThicknessMm: 1,
          );
          expect(BrepHeal.topologyIsClosed(hollow), isTrue);
          final mesh = BrepTessellate.run(hollow);
          expect(
            MeshKit.isClosedSolid(mesh),
            isTrue,
            reason:
                'Hollow must produce a closed, consistently oriented mesh without repair',
          );
          final size = Vector3(80, 60, 50)..[axis] *= .5;
          final inner =
              size - Vector3.all(2) + (openingNormal.clone()..absolute());
          final expectedVolume =
              size.x * size.y * size.z - inner.x * inner.y * inner.z;
          expect(MeshKit.signedVolume(mesh), closeTo(expectedVolume, 1e-5));
          expect(
            MeshKit.isClosedSolid(IjoOps.tessellateClosed(hollow)),
            isTrue,
          );
        });
      }
    }
  }

  test('hollow a saved cut face with either trim winding', () {
    final part = IjoOps.sliceBrep(
      BrepPrimitives.box(80, 60, 50),
      Vector3(40, 30, 25),
      Vector3(1, 0, 0),
    ).first;
    final opening = part.faces.indexWhere((f) => f.reversed);
    expect(opening, greaterThanOrEqualTo(0));
    for (final reverseWire in [false, true]) {
      final input = part.clone();
      final face = input.faces[opening];
      if (reverseWire) {
        face.loops = [
          for (final loop in face.loops)
            BrepLoop([
              for (final co in loop.coedges.reversed)
                BrepCoedge(co.edge, reversed: !co.reversed),
            ]),
        ];
      }
      final saved = BrepSolid.fromJson(
        jsonDecode(jsonEncode(input.toJson())) as Map<String, dynamic>,
      );
      final hollow = IjoOps.hollowSolid(
        saved,
        [opening],
        face.outwardNormal(0, 0),
        wallThicknessMm: 1,
      );
      final mesh = BrepTessellate.run(hollow);
      expect(BrepHeal.topologyIsClosed(hollow), isTrue);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(
        MeshKit.signedVolume(mesh),
        closeTo(40 * 60 * 50 - 39 * 58 * 48, 1e-5),
      );
    }
  });

  test(
    'hollow oblique cut faces after rotation, translation and serialization',
    () {
      final rotation = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .63);
      Vector3 place(Vector3 p) => rotation.rotated(p) + Vector3(123, -90, 42);
      final normal = rotation.rotated(Vector3(.2, .1, 1).normalized());
      final origin = place(Vector3(40, 30, 25));
      final solid = BrepXform.map(BrepPrimitives.box(80, 60, 50), place);
      final parts = IjoOps.sliceBrep(solid, origin, normal);
      expect(parts, hasLength(2));
      for (final sliced in parts) {
        final part = BrepSolid.fromJson(
          jsonDecode(jsonEncode(sliced.toJson())) as Map<String, dynamic>,
        );
        final before = jsonEncode(part.toJson());
        final opening = part.faces.indexWhere(
          (f) =>
              f.surface is BrepPlane &&
              (f.surface as BrepPlane).height(origin).abs() < 1e-6 &&
              f.outwardNormal(0, 0).cross(normal).length < 1e-6,
        );
        expect(opening, greaterThanOrEqualTo(0));
        final shell = IjoOps.hollowSolid(
          part,
          [opening],
          part.faces[opening].outwardNormal(0, 0),
          wallThicknessMm: 1,
        );
        expect(
          jsonEncode(part.toJson()),
          before,
          reason: 'Hollow must not alter its input',
        );
        final restored = BrepSolid.fromJson(
          jsonDecode(jsonEncode(shell.toJson())) as Map<String, dynamic>,
        );
        expect(BrepHeal.topologyIsClosed(restored), isTrue);
        final mesh = BrepTessellate.run(restored);
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        expect(MeshKit.signedVolume(mesh), greaterThan(0));
        expect(
          MeshKit.signedVolume(mesh),
          lessThan(MeshKit.signedVolume(IjoOps.tessellateClosed(part))),
        );
        expect(
          MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)),
          isTrue,
        );
      }
    },
  );
}
