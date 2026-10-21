import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/cap_join.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid axle(String name) => BrepSolid.fromJson(
  jsonDecode(
        utf8.decode(
          gzip.decode(
            File('test/fixtures/trailer_axle/$name.json.gz').readAsBytesSync(),
          ),
        ),
      )
      as Map<String, dynamic>,
);
bool sameTrim(BrepSolid source, BrepFace old, BrepSolid result, BrepFace face) {
  if (face.reversed != old.reversed || face.loops.length != old.loops.length)
    return false;
  for (var li = 0; li < face.loops.length; li++) {
    final oldEdges = old.loops[li].coedges, newEdges = face.loops[li].coedges;
    if (newEdges.length != oldEdges.length) return false;
    for (var ei = 0; ei < oldEdges.length; ei++) {
      final oc = oldEdges[ei], nc = newEdges[ei];
      final oldCurve = source.edges[oc.edge].curve,
          newCurve = result.edges[nc.edge].curve;
      for (final t in [0.0, .25, .5, .75, 1.0]) {
        final p = oldCurve.evaluate(
          oldCurve.t0 + (oldCurve.t1 - oldCurve.t0) * (oc.reversed ? 1 - t : t),
        );
        final q = newCurve.evaluate(
          newCurve.t0 + (newCurve.t1 - newCurve.t0) * (nc.reversed ? 1 - t : t),
        );
        if ((p - q).length > 1e-7) return false;
      }
    }
  }
  return true;
}

void main() {
  for (final reverse in [false, true]) {
    test(
      'saved mirrored trailer axle joins with exact curved faces reverse=$reverse',
      () {
        final a = axle(reverse ? 'b' : 'a'), b = axle(reverse ? 'a' : 'b');
        final beforeA = jsonEncode(a.toJson()),
            beforeB = jsonEncode(b.toJson());
        final result = IjoOps.booleanSolid(a, b, IjoOps.unionOp);
        expect(result.faces, hasLength(82));
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        expect(IjoOps.splitBrep(result), hasLength(1));
        final display = IjoOps.tessellateKeep(result);
        expect(MeshKit.isClosedSolid(display), isTrue);
        final expected =
            MeshKit.signedVolume(IjoOps.tessellateKeep(a)) +
            MeshKit.signedVolume(IjoOps.tessellateKeep(b));
        // Mixed affine faces can choose different adaptive display samples.
        // Compare display volume within tessellation error, and independently
        // require unchanged exact surfaces and directed trims below.
        expect(
          MeshKit.signedVolume(display),
          closeTo(expected, expected * .001),
        );
        // The internal center cap is removed; every other exact surface survives.
        for (final source in [a, b]) {
          for (final old in source.faces) {
            final matches = result.faces.where(
              (f) =>
                  jsonEncode(f.surface.toJson()) ==
                  jsonEncode(old.surface.toJson()),
            );
            if (matches.isEmpty) {
              expect(old.surface.evaluate(0, 0).x.abs(), lessThan(1e-7));
              continue;
            }
            expect(
              matches.any((face) => sameTrim(source, old, result, face)),
              isTrue,
            );
          }
        }
        final restored = BrepSolid.fromJson(
          jsonDecode(jsonEncode(result.toJson())) as Map<String, dynamic>,
        );
        expect(MeshKit.isClosedSolid(IjoOps.tessellateKeep(restored)), isTrue);
        expect(jsonEncode(a.toJson()), beforeA);
        expect(jsonEncode(b.toJson()), beforeB);
      },
      timeout: const Timeout(Duration(minutes: 2)),
    );
  }
  test(
    'cap join handles independent affine frames and refuses gaps or overlap',
    () {
      final base = BrepPrimitives.box(6, 8, 10);
      final a = BrepXform.affine(
        base,
        BrepAffineTransform.fromFunction((p) => p),
      );
      final b = BrepXform.affine(
        base,
        BrepAffineTransform.fromFunction((p) => Vector3(-p.x, p.y, p.z)),
      );
      final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .67);
      BrepSolid placed(BrepSolid s) => BrepXform.affine(
        s,
        BrepAffineTransform.fromFunction(
          (p) =>
              q.rotated(Vector3(p.x * 2, p.y, p.z * .5)) +
              Vector3(100, -30, 17),
        ),
      );
      final result = joinMatchingPlanarCaps(placed(a), placed(b));
      expect(result, isNotNull);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateKeep(result!)), isTrue);
      expect(IjoOps.splitBrep(result), hasLength(1));
      for (final shift in [-.1, .1]) {
        final shifted = BrepXform.affine(
          b,
          BrepAffineTransform.fromFunction((p) => p + Vector3(shift, 0, 0)),
        );
        expect(joinMatchingPlanarCaps(a, shifted), isNull);
      }
      final partial = BrepXform.affine(
        b,
        BrepAffineTransform.fromFunction((p) => p + Vector3(0, 1, 0)),
      );
      expect(joinMatchingPlanarCaps(a, partial), isNull);
      expect(joinMatchingPlanarCaps(a, a), isNull);
    },
  );
}
