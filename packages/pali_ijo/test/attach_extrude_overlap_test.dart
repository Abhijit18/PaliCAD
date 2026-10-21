import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'support/hooked_extrusion.dart';

void main() {
  for (final rotated in [false, true]) {
    for (final distance in [4.0, 10.0, 12.0, 15.0, 18.0]) {
      test('attach trims return wall: distance=$distance rotated=$rotated', () {
        final source = hookedExtrusion();
        final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .7);
        Vector3 point(Vector3 p) =>
            rotated ? q.rotated(p) + Vector3(12, -31, 7) : p;
        Vector3 vector(Vector3 p) => rotated ? q.rotated(p) : p;
        final face = source.faces.indexWhere(
          (f) =>
              f.surface is BrepPlane &&
              ((f.surface as BrepPlane).origin.y - 5).abs() < 1e-7 &&
              f.outwardNormal(0, 0).y > .99,
        );
        final solid = BrepXform.map(source, point);
        final before = solid.toJson();
        // Exercise the general union's concave cap before healing as well:
        // rotated caps previously fell back to overlapping triangle fans.
        final union = BrepBoolean.run(
          solid,
          BrepExtrude.fromFace(
            solid,
            solid.faces[face],
            vector(Vector3(0, distance, 0)),
          ),
          BrepBoolean.unionOp,
        );
        expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(union)), isTrue);
        expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(solid)), isTrue);
        final out = BrepExtrude.attachFromFace(
          solid,
          face,
          vector(Vector3(0, distance, 0)),
        );
        final overlap = 2 * (distance - 10).clamp(0.0, 5.0) * 10;
        final volume = (560 - 12.5 * math.pi) * 10 + 700 * distance - overlap;
        expect(IjoMeasure.volumeSolid(out), closeTo(volume, 1e-4));
        final mesh = IjoOps.tessellateClosed(out);
        final top = point(Vector3(0, 0, 10));
        final up = vector(Vector3(0, 0, 1));
        expect(
          out.faces
              .where(
                (f) =>
                    f.surface is BrepPlane &&
                    f.outwardNormal(0, 0).dot(up) > .99 &&
                    (f.surface as BrepPlane).height(top).abs() < 1e-7,
              )
              .length,
          1,
          reason: 'the joined top should have no old coplanar seams',
        );
        expect(MeshKit.signedVolume(mesh), closeTo(volume, volume * .002));
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        final restored = BrepSolid.fromJson(out.toJson());
        expect(IjoMeasure.volumeSolid(restored), closeTo(volume, 1e-4));
        // The shared face inside the return wall must disappear. Any face
        // covering this interior point is an untrimmed attachment seam.
        if (distance > 10) {
          final inside = point(
            Vector3(68, 15 + math.min(distance - 10, 5) / 2, 5),
          );
          for (final face in out.faces) {
            final surface = face.surface;
            if (surface is! BrepPlane || surface.height(inside).abs() > 1e-7) {
              continue;
            }
            // A real outside face cannot have a triangle over this point.
            for (var i = 0; i < mesh.faces.length; i++) {
              if (mesh.faceIds?[i] != out.faces.indexOf(face)) continue;
              final f = mesh.faces[i];
              final a = mesh.vertices[f[0]],
                  b = mesh.vertices[f[1]],
                  c = mesh.vertices[f[2]];
              final n = (b - a).cross(c - a);
              final signs = [
                (b - a).cross(inside - a).dot(n),
                (c - b).cross(inside - b).dot(n),
                (a - c).cross(inside - c).dot(n),
              ];
              expect(
                signs.every((s) => s >= -1e-8),
                isFalse,
                reason: 'buried return wall remains inside the union',
              );
            }
          }
        }
        expect(solid.toJson(), before);
      });
    }
  }
}
