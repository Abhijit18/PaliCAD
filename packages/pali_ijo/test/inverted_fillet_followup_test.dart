import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid savedShape() => BrepSolid.fromJson(
  jsonDecode(
        File(
          'test/fixtures/inverse_fillet_extruded_split.json',
        ).readAsStringSync(),
      )
      as Map<String, dynamic>,
);

void main() {
  for (final edge in [18, 20, 21, 22]) {
    final radii = [
      3.6189741599131207,
      if (edge == 20 || edge == 22) ...[11.752044689472855, 26.5],
    ];
    for (final radius in radii) {
      test('saved cut stem top edge=$edge radius=$radius', () {
        final source = savedShape();
        final before = source.toJson();
        final selected = source.edges[edge];
        final length =
            (source.vertices[selected.v1] - source.vertices[selected.v0])
                .length;
        final result = IjoOps.filletSolid(
          source,
          [edge],
          radius: radius,
          inverse: false,
        );
        expect(source.toJson(), before);
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        for (final quality in [
          MeshTessellation.draft,
          MeshTessellation.standard,
          MeshTessellation.fine,
        ]) {
          expect(
            MeshKit.isClosedSolid(
              IjoOps.tessellateClosed(result, tessellation: quality),
            ),
            isTrue,
          );
        }
        // A convex right-angle fillet removes the square outside a quarter circle.
        expect(
          IjoMeasure.volumeSolid(result),
          closeTo(
            IjoMeasure.volumeSolid(source) -
                length * radius * radius * (1 - math.pi / 4),
            .1,
          ),
        );
        expect(
          result.faces.where((f) => f.surface is BrepCylinder && !f.reversed),
          hasLength(1),
        );
        final restored = BrepSolid.fromJson(result.toJson());
        expect(
          MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)),
          isTrue,
        );
      });
    }
  }
  test('rotated saved cut face retains convex edge classification', () {
    final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .7);
    final source = BrepXform.map(
      savedShape(),
      (p) => q.rotated(p) + Vector3(100, -50, 25),
    );
    final result = IjoOps.filletSolid(
      source,
      [20],
      radius: 11.75,
      inverse: false,
    );
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)), isTrue);
    expect(
      IjoMeasure.volumeSolid(result),
      lessThan(IjoMeasure.volumeSolid(source)),
    );
  });
}
