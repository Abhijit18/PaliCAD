import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final angle in [7.5, 23.0, 37.0]) {
    for (final transformed in [false, true]) {
      test(
        'rectangle through filleted cube at $angle degrees transformed=$transformed',
        () {
          var box = BrepPrimitives.box(20, 20, 20);
          final edge = box.edges.indexWhere(
            (e) => [
              box.vertices[e.v0],
              box.vertices[e.v1],
            ].every((p) => (p.x - 20).abs() < 1e-8 && (p.z - 20).abs() < 1e-8),
          );
          box = IjoOps.filletSolid(box, [edge], radius: 4, inverse: false);
          final tilt = Quaternion.axisAngle(
            Vector3(1, 0, 0),
            angle * math.pi / 180,
          );
          box = BrepXform.map(
            box,
            (p) => tilt.rotated(p - Vector3.all(10)) + Vector3.all(10),
          );
          var cutter = BrepExtrude.polygon([
            Vector3(5, -50, 9),
            Vector3(6, -50, 9),
            Vector3(6, -50, 11),
            Vector3(5, -50, 11),
          ], Vector3(0, 100, 0));
          if (transformed) {
            final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .63);
            Vector3 map(Vector3 p) => q.rotated(p) + Vector3(31, -17, 9);
            box = BrepXform.map(box, map);
            cutter = BrepXform.map(cutter, map);
          }
          final result = IjoOps.booleanSolid(box, cutter, IjoOps.subtractOp);
          expect(BrepHeal.topologyIsClosed(result), isTrue);
          expect(IjoOps.splitBrep(result), hasLength(1));
          final mesh = IjoOps.tessellateClosed(result);
          expect(MeshKit.isClosedSolid(mesh), isTrue);
          final beforeVolume = MeshKit.signedVolume(
            IjoOps.tessellateClosed(box),
          );
          expect(
            beforeVolume - MeshKit.signedVolume(mesh),
            closeTo(40 / math.cos(angle * math.pi / 180), .1),
          );
          expect(
            result.faces.where((f) => f.surface is BrepCylinder),
            hasLength(1),
          );
        },
      );
    }
  }
}
