import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final angle in [0.0, .63, 1.57, 2.4]) {
    test('triangle walls ignore buried sphere after rotation $angle', () {
      var base = BrepSolid.fromJson(
        jsonDecode(
              File(
                'test/fixtures/angled_bridge_after_rectangle.json',
              ).readAsStringSync(),
            )
            as Map<String, dynamic>,
      );
      final points = [
        Vector3(31.565503755217875, 48.82, 6.449280316793482),
        Vector3(27.827903324771196, 48.82, 4.664813794630125),
        Vector3(31.242096880390868, 48.82, 2.3201901337493265),
      ];
      var tool = BrepExtrude.polygon(points, Vector3(0, -97.64, 0));
      final rotation = Quaternion.axisAngle(
        Vector3(1, 2, 3).normalized(),
        angle,
      );
      Vector3 map(Vector3 p) => rotation.rotated(p) + Vector3(31, -17, 9);
      base = BrepXform.map(base, map);
      tool = BrepXform.map(tool, map);
      final result = IjoOps.booleanSolid(base, tool, IjoOps.subtractOp);
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      expect(IjoOps.splitBrep(result), hasLength(1));
      final before = BrepTessellate.run(base);
      final after = BrepTessellate.run(result);
      expect(MeshKit.isClosedSolid(after), isTrue);
      final triangleArea =
          (points[1] - points[0]).cross(points[2] - points[0]).length / 2;
      expect(
        MeshKit.signedVolume(before) - MeshKit.signedVolume(after),
        closeTo(triangleArea * 20 / .9914096509721302, .01),
      );
      // A shared edge must lie on BOTH incident surfaces. Sewing two phantom
      // sphere arcs together previously made invalid geometry look closed.
      for (final face in result.faces) {
        for (final loop in face.loops) {
          for (final co in loop.coedges) {
            final c = result.edges[co.edge].curve;
            for (final t in [c.t0, (c.t0 + c.t1) / 2, c.t1]) {
              final p = c.evaluate(t);
              expect((face.surface.closestPoint(p) - p).length, lessThan(1e-6));
            }
          }
        }
      }
      // Compare with the supported in-memory repair of legacy circular trims.
      // Reversed curve parameters can change during repair without changing
      // the face boundary or the stored project.
      base = BrepHeal.alignCircularTrims(BrepHeal.restorePrismaticWalls(base));
      final curved = base.faces.where((f) => f.surface is! BrepPlane).toList();
      final resultCurved = result.faces
          .where((f) => f.surface is! BrepPlane)
          .toList();
      expect(resultCurved, hasLength(curved.length));
      Object geometry(BrepSolid s, BrepFace f) => [
        f.surface.toJson(),
        f.reversed,
        for (final loop in f.loops)
          [for (final co in loop.coedges) s.edges[co.edge].curve.toJson()],
      ];
      for (var i = 0; i < curved.length; i++) {
        expect(
          geometry(result, resultCurved[i]),
          equals(geometry(base, curved[i])),
        );
      }
    });
  }
}
