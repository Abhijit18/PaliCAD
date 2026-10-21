import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final sides in [3, 5, 6]) {
    for (final top in [false, true]) {
      test('inverse prism cap sides=$sides top=$top', () {
        final source = BrepPrimitives.prism(
          diameter: 30,
          height: 12,
          sides: sides,
        );
        final face = BrepFaceSelect.extreme(
          Vector3(0, 0, top ? 1 : -1),
        ).resolve(source).first;
        final ids = [
          for (final e in source.faces[face].loops.single.coedges) e.edge,
        ];
        final result = IjoOps.filletSolid(
          source,
          ids,
          radius: 2,
          inverse: true,
        );
        for (final quality in [MeshTessellation.draft, MeshTessellation.fine]) {
          expect(
            MeshKit.isClosedSolid(
              IjoOps.tessellateClosed(result, tessellation: quality),
            ),
            isTrue,
          );
        }
        expect(
          IjoMeasure.volumeSolid(result),
          lessThan(IjoMeasure.volumeSolid(source)),
        );
        expect(
          result.faces.where((f) => f.surface is BrepCylinder).length,
          sides,
        );
        // Each shared miter must lie on both adjoining cylindrical surfaces.
        for (var ei = 0; ei < result.edges.length; ei++) {
          final edge = result.edges[ei];
          if (edge.curve is! BrepEllipse) continue;
          final faces = BrepFillet.facesForEdge(result, ei);
          expect(faces, hasLength(2));
          for (final fi in faces) {
            final cylinder = result.faces[fi].surface as BrepCylinder;
            for (var k = 0; k <= 10; k++) {
              final point = edge.curve.evaluate(math.pi * k / 20);
              final d = point - cylinder.origin;
              expect(
                (d - cylinder.axis * d.dot(cylinder.axis)).length,
                closeTo(2, 1e-8),
              );
            }
          }
        }
      });
    }
  }
  test('inverse face rejects collapsed offsets without changing source', () {
    final source = BrepPrimitives.box(30, 20, 10);
    final face = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(source).first;
    final ids = [
      for (final e in source.faces[face].loops.single.coedges) e.edge,
    ];
    final before = source.toJson();
    expect(
      () => IjoOps.filletSolid(source, ids, radius: 11, inverse: true),
      throwsA(isA<BrepUnsupported>()),
    );
    expect(source.toJson(), before);
  });
  test('rotated inverse face keeps analytic geometry', () {
    final source = BrepXform.map(
      BrepPrimitives.box(30, 20, 10),
      (p) => Vector3(
        100 + p.x * math.cos(.7) - p.z * math.sin(.7),
        p.y - 33,
        p.x * math.sin(.7) + p.z * math.cos(.7),
      ),
    );
    final ids = [for (final e in source.faces[1].loops.single.coedges) e.edge];
    final result = IjoOps.filletSolid(source, ids, radius: 2, inverse: true);
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)), isTrue);
  });
  for (var fi = 0; fi < 6; fi++) {
    test('inverse box face $fi cuts only its perimeter', () {
      final source = BrepPrimitives.box(30, 20, 10);
      final before = source.toJson();
      final face = source.faces[fi];
      final ids = [for (final c in face.loops.single.coedges) c.edge];
      final result = IjoOps.filletSolid(source, ids, radius: 2, inverse: true);
      expect(source.toJson(), before);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)), isTrue);
      final perimeter = ids.fold<double>(
        0,
        (sum, id) =>
            sum +
            (source.vertices[source.edges[id].v1] -
                    source.vertices[source.edges[id].v0])
                .length,
      );
      // Four quarter-cylinder cuts, subtracting the four overlapping corners.
      final removed = math.pi * perimeter - 4 * 2 * math.pow(2, 3) / 3;
      expect(IjoMeasure.volumeSolid(result), closeTo(6000 - removed, .03));
      expect(result.faces.where((f) => f.surface is BrepCylinder).length, 4);
      final restored = BrepSolid.fromJson(result.toJson());
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(restored)), isTrue);
    });
  }
}
