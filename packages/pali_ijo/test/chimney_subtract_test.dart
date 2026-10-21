import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid savedChimney() => BrepSolid.fromJson(
  jsonDecode(
        File(
          'test/fixtures/inverse_fillet_hollow_chimney.json',
        ).readAsStringSync(),
      )
      as Map<String, dynamic>,
);

void main() {
  test(
    'saved circular trims are repaired without moving geometry or input',
    () {
      final source = savedChimney();
      final before = source.toJson();
      final repaired = BrepHeal.alignCircularTrims(source);
      expect(source.toJson(), before);
      expect(repaired.vertices, source.vertices);
      expect(
        repaired.faces.map((f) => f.toJson()),
        source.faces.map((f) => f.toJson()),
      );
      for (var i = 0; i < repaired.edges.length; i++) {
        final edge = repaired.edges[i];
        expect(
          (edge.curve.evaluate(edge.curve.t0) - repaired.vertices[edge.v0])
              .length,
          lessThan(1e-7),
        );
        expect(
          (edge.curve.evaluate(edge.curve.t1) - repaired.vertices[edge.v1])
              .length,
          lessThan(1e-7),
        );
      }
      expect(
        identical(BrepHeal.alignCircularTrims(repaired), repaired),
        isTrue,
      );
      final restored = BrepSolid.fromJson(repaired.toJson());
      expect(
        identical(BrepHeal.alignCircularTrims(restored), restored),
        isTrue,
      );
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)), isTrue);
    },
  );

  for (final angles in [
    (0.0, math.pi / 2),
    (5.5, 7.0),
    (0.0, 1.5 * math.pi),
    (1.5, 0.0),
  ]) {
    test('reversed circular trim preserves its finite arc $angles', () {
      final c = BrepCircle(
        center: Vector3.zero(),
        normal: Vector3(0, 0, 1),
        radius: 5,
        t0: angles.$1,
        t1: angles.$2,
      );
      final b = BrepBuilder();
      final a = b.vertex(c.evaluate(c.t1)), z = b.vertex(c.evaluate(c.t0));
      b.edge(a, z, c);
      final repaired = BrepHeal.alignCircularTrims(b.build());
      final r = repaired.edges.single.curve;
      for (final t in [0.0, .25, .5, .75, 1.0]) {
        expect(
          (r.evaluate(r.t0 + (r.t1 - r.t0) * t) -
                  c.evaluate(c.t1 + (c.t0 - c.t1) * t))
              .length,
          lessThan(1e-7),
        );
      }
    });
  }

  for (final rotate in [false, true]) {
    for (final depth in [
      10.0,
      27.10869294137807,
      30.0,
      50.0,
      52.2760515723295,
      58.14,
      70.0,
      85.0,
    ]) {
      test('saved hollow chimney subtract $depth rotated=$rotate', () {
        var source = savedChimney();
        var normal = Vector3(0, 0, 1);
        if (rotate) {
          final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .7);
          source = BrepXform.map(
            source,
            (p) => q.rotated(p) + Vector3(100, -50, 25),
          );
          normal = q.rotated(normal);
        }
        final before = source.toJson();
        final face =
            source.faces[BrepFaceSelect.extreme(normal).resolve(source).first];
        final cutter = BrepExtrude.fromFace(source, face, normal * -depth);
        final result = IjoOps.booleanSolid(source, cutter, IjoOps.subtractOp);
        expect(source.toJson(), before);
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        for (final quality in [MeshTessellation.draft, MeshTessellation.fine]) {
          expect(
            MeshKit.isClosedSolid(
              IjoOps.tessellateClosed(result, tessellation: quality),
            ),
            isTrue,
          );
        }
        final repaired = BrepHeal.alignCircularTrims(source);
        final volume = IjoMeasure.volumeSolid(result);
        expect(volume, greaterThan(0));
        expect(volume, lessThan(IjoMeasure.volumeSolid(repaired)));
        // The cap prism removes a constant area until it reaches the hollow
        // chamber; between its ceiling and floor no further material is cut.
        if (depth <= 58.14) {
          final expected =
              IjoMeasure.volumeSolid(repaired) -
              36.38059570418677 *
                  11.44897141693362 *
                  math.min(depth, 52.2760515723295);
          expect(volume, closeTo(expected, .2));
        }
        final restored = BrepSolid.fromJson(result.toJson());
        expect(
          MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)),
          isTrue,
        );
        // A second boolean on the repaired, saved result must remain usable.
        final further = BrepExtrude.fromFace(
          source,
          face,
          normal * -(depth + 1),
        );
        final next = IjoOps.booleanSolid(restored, further, IjoOps.subtractOp);
        expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(next)), isTrue);
      });
    }
  }
}
