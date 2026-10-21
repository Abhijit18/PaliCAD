import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid savedBody() => BrepSolid.fromJson(
  jsonDecode(
        File(
          'test/fixtures/inverse_fillet_hollow_chimney.json',
        ).readAsStringSync(),
      )
      as Map<String, dynamic>,
);

void main() {
  for (final rotated in [false, true]) {
    for (final depth in [.25, .5, 1.0, 19.2]) {
      test(
        'saved curved inner face subtract depth=$depth rotated=$rotated',
        () {
          var source = savedBody();
          if (rotated) {
            final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .7);
            source = BrepXform.map(
              source,
              (p) => q.rotated(p) + Vector3(100, -50, 25),
            );
          }
          final before = source.toJson();
          final face = source.faces[3],
              normal = source.faces[3].outwardNormal(0, 0);
          final corrected = BrepHeal.alignCircularTrims(source);
          final cutter = BrepExtrude.fromFace(source, face, normal * -depth);
          // Every swept curve must agree with the topological endpoints before
          // the Boolean sees it; closed edge indices alone missed this bug.
          for (final e in cutter.edges) {
            expect(
              (e.curve.evaluate(e.curve.t0) - cutter.vertices[e.v0]).length,
              lessThan(1e-7),
            );
            expect(
              (e.curve.evaluate(e.curve.t1) - cutter.vertices[e.v1]).length,
              lessThan(1e-7),
            );
          }
          expect(BrepHeal.topologyIsClosed(cutter), isTrue);
          expect(
            MeshKit.isClosedSolid(IjoOps.tessellateClosed(cutter)),
            isTrue,
          );
          final result = IjoOps.booleanSolid(source, cutter, IjoOps.subtractOp);
          expect(source.toJson(), before);
          expect(BrepHeal.topologyIsClosed(result), isTrue);
          for (final quality in [
            MeshTessellation.draft,
            MeshTessellation.fine,
          ]) {
            expect(
              MeshKit.isClosedSolid(
                IjoOps.tessellateClosed(result, tessellation: quality),
              ),
              isTrue,
            );
          }
          // The back wall is exactly 1 mm thick. A deeper cutter must remove
          // that same wall, without extending or damaging the hollow interior.
          final area = IjoMeasure.volumeSolid(cutter) / depth;
          expect(
            IjoMeasure.volumeSolid(result),
            closeTo(
              IjoMeasure.volumeSolid(corrected) - area * math.min(depth, 1),
              .1,
            ),
          );
          final restored = BrepSolid.fromJson(result.toJson());
          expect(
            MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)),
            isTrue,
          );
          final nextCutter = BrepExtrude.fromFace(
            source,
            face,
            normal * -(depth + .1),
          );
          expect(
            MeshKit.isClosedSolid(
              IjoOps.tessellateClosed(
                IjoOps.booleanSolid(restored, nextCutter, IjoOps.subtractOp),
              ),
            ),
            isTrue,
          );
        },
      );
    }
  }
  test('shifted face extrusion repairs arcs before moving the profile', () {
    final source = savedBody(),
        corrected = BrepHeal.alignCircularTrims(savedBody());
    final shift = Vector3(10, 20, 30), dir = Vector3(0, -19.2, 0);
    final actual = BrepExtrude.fromFace(
      source,
      source.faces[3],
      dir,
      originShift: shift,
    );
    final expected = BrepExtrude.fromFace(
      corrected,
      corrected.faces[3],
      dir,
      originShift: shift,
    );
    Map<String, dynamic> geometry(BrepSolid solid) {
      final json = solid.toJson();
      // Each independent sweep receives a new operation provenance ID.
      for (final item in [...json['e'] as List, ...json['f'] as List]) {
        (item as Map).remove('g');
      }
      return json;
    }

    expect(geometry(actual), geometry(expected));
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(actual)), isTrue);
  });
}
