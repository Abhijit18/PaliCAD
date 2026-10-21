import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/boundary_boolean.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final order in [
    [1, 2],
    [2, 1],
  ]) {
    test('finger cuts $order retain untouched periodic mounting holes', () {
      final input =
          (jsonDecode(
                    File(
                      'test/fixtures/action_camera_finger_gaps.json',
                    ).readAsStringSync(),
                  )
                  as List)
              .map((s) => BrepSolid.fromJson(s as Map<String, dynamic>))
              .toList();
      var result = input.first;
      for (final index in order) {
        // Exercise the shared arrangement directly: the legacy path must not
        // hide a missing wall in a hole remote from the tool.
        final shared = BrepBoundaryBoolean.subtract(result, input[index]);
        expect(BrepHeal.topologyIsClosed(shared), isTrue);
        result = IjoOps.booleanSolid(result, input[index], IjoOps.subtractOp);
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        final mesh = IjoOps.tessellateKeep(result, op: 'Finger gap');
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        for (final x in [-11.5, 11.5]) {
          for (final y in [-13.505, 13.505]) {
            expect(BrepClassify.isInside(result, Vector3(x, y, 1)), isFalse);
            expect(
              BrepClassify.isInside(result, Vector3(x + 1.5, y, 1)),
              isTrue,
            );
          }
        }
        expect(
          BrepClassify.isInside(result, Vector3(index == 1 ? 3.2 : -3.2, 0, 5)),
          isFalse,
        );
      }
      final beforeNotch = MeshKit.signedVolume(IjoOps.tessellateKeep(result));
      final uncut = result;
      for (final up in [false, true]) {
        final z = up ? 2 - 2.4 : 2.4;
        final notch = BrepExtrude.profile([
          Vector3(0, 8.5, z),
          Vector3(8, 16.5, z),
          Vector3(-8, 16.5, z),
        ], Vector3(0, 0, up ? 2.8 : -2.8));
        result = IjoOps.booleanSolid(uncut, notch, IjoOps.subtractOp);
        expect(
          MeshKit.isClosedSolid(BrepTessellate.run(result)),
          isTrue,
          reason: 'cap must triangulate without repair, up=$up',
        );
        expect(
          beforeNotch - MeshKit.signedVolume(BrepTessellate.run(result)),
          closeTo(128, 1e-4),
        );
      }
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateKeep(result)), isTrue);
      expect(
        beforeNotch - MeshKit.signedVolume(IjoOps.tessellateKeep(result)),
        closeTo(128, 1e-4),
      );
      expect(BrepClassify.isInside(result, Vector3(0, 12, 1)), isFalse);
      expect(BrepClassify.isInside(result, Vector3(0, 8, 1)), isTrue);
      for (final x in [-6.4, 0.0, 6.4]) {
        expect(BrepClassify.isInside(result, Vector3(x, 0, 5)), isTrue);
        expect(BrepClassify.isInside(result, Vector3(x, 0, 9.99)), isFalse);
      }
      for (final x in [-3.2, 3.2]) {
        expect(BrepClassify.isInside(result, Vector3(x, 0, 1.999)), isTrue);
        expect(BrepClassify.isInside(result, Vector3(x, 0, 2.001)), isFalse);
      }
    });
  }
}
