import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid frame() => BrepSolid.fromJson(
  jsonDecode(File('test/fixtures/concave_fillet/frame.json').readAsStringSync())
      as Map<String, dynamic>,
);
double volume(MeshData mesh) => mesh.faces.fold(
  0.0,
  (v, f) =>
      v +
      mesh.vertices[f[0]].dot(mesh.vertices[f[1]].cross(mesh.vertices[f[2]])) /
          6,
);
void main() {
  for (final mirrored in [false, true]) {
    for (final radius in [3.0, 17.971043513278715]) {
      for (final edge in [2, 17]) {
        test('frame edge $edge radius $radius mirrored $mirrored', () {
          var source = frame();
          if (mirrored) {
            source = BrepXform.mirror(source, Vector3.zero(), Vector3(1, 2, 3));
          }
          final result = IjoOps.filletSolid(
            source,
            [edge],
            radius: radius,
            inverse: false,
          );
          final concave = edge == 17;
          final blend = result.faces.singleWhere(
            (f) => f.surface is BrepCylinder,
          );
          expect(blend.reversed, concave);
          final mesh = IjoOps.tessellateClosed(result, op: 'Fillet');
          expect(MeshKit.isClosedSolid(mesh), isTrue);
          final change = (1 - math.pi / 4) * radius * radius * 20;
          expect(
            volume(mesh),
            closeTo(128000 + (concave ? change : -change), change * .015),
          );
          final uses = <int, List<bool>>{};
          for (final face in result.faces) {
            for (final loop in face.loops) {
              for (final co in loop.coedges) {
                (uses[co.edge] ??= []).add(co.reversed);
              }
            }
          }
          for (final uses in uses.values) {
            expect(uses, hasLength(2));
            expect(uses[0], !uses[1]);
          }
        });
      }
    }
  }
  test('inside fillet is invariant to edge and surface representations', () {
    final source = frame();
    final edge = source.edges[17];
    final oldStart = edge.v0;
    edge.v0 = edge.v1;
    edge.v1 = oldStart;
    edge.curve = BrepLine(source.vertices[edge.v0], source.vertices[edge.v1]);
    for (final face in source.faces) {
      face.loops = [
        for (final loop in face.loops)
          BrepLoop([
            for (final co in loop.coedges)
              BrepCoedge(co.edge, reversed: co.reversed != (co.edge == 17)),
          ]),
      ];
      final plane = face.surface as BrepPlane;
      face.surface = BrepPlane(
        origin: plane.origin,
        normal: -plane.normal,
        xDir: plane.xDir,
      );
      face.reversed = !face.reversed;
    }
    final result = IjoOps.filletSolid(source, [17], radius: 5, inverse: false);
    final mesh = IjoOps.tessellateClosed(result, op: 'Fillet');
    expect(MeshKit.isClosedSolid(mesh), isTrue);
    expect(volume(mesh), closeTo(128000 + (1 - math.pi / 4) * 25 * 20, 2));
  });
  for (final edges in [
    [14, 17, 20, 23],
    [23, 20, 17, 14],
  ]) {
    test('all four inner corners add material with selection order $edges', () {
      final result = IjoOps.filletSolid(
        frame(),
        edges,
        radius: 5,
        inverse: false,
      );
      expect(
        result.faces.where((f) => f.surface is BrepCylinder && f.reversed),
        hasLength(4),
      );
      final mesh = IjoOps.tessellateClosed(result, op: 'Fillet');
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(
        volume(mesh),
        closeTo(128000 + 4 * (1 - math.pi / 4) * 25 * 20, 8),
      );
    });
  }
}
