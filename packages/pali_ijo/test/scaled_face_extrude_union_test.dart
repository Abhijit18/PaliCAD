import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final distance in [30.0, 60.0, 98.26]) {
    test('scaled face extrusion trims buried surfaces $distance', () {
      final source = BrepSolid.fromJson(
        jsonDecode(
              File(
                'test/fixtures/scaled_hook_face_extrude.json',
              ).readAsStringSync(),
            )
            as Map<String, dynamic>,
      );
      const face = 23;
      final direction = Vector3(-distance, 0, 0);
      final out = BrepBoolean.run(
        source,
        BrepExtrude.fromFace(source, source.faces[face], direction),
        BrepBoolean.unionOp,
      );
      final mesh = IjoOps.tessellateClosed(out);
      final corners = [
        for (final co in source.faces[face].loops.single.coedges)
          source.vertices[source.edges[co.edge].v0],
      ];
      final min = Vector3(
        corners.first.x - distance,
        corners.map((p) => p.y).reduce((a, b) => a < b ? a : b),
        corners.map((p) => p.z).reduce((a, b) => a < b ? a : b),
      );
      final max = Vector3(
        corners.first.x,
        corners.map((p) => p.y).reduce((a, b) => a > b ? a : b),
        corners.map((p) => p.z).reduce((a, b) => a > b ? a : b),
      );
      final buried = <int>[];
      for (var i = 0; i < mesh.faces.length; i++) {
        final f = mesh.faces[i];
        final c =
            (mesh.vertices[f[0]] + mesh.vertices[f[1]] + mesh.vertices[f[2]]) /
            3;
        if (c.x > min.x + 1e-5 &&
            c.x < max.x - 1e-5 &&
            c.y > min.y + 1e-5 &&
            c.y < max.y - 1e-5 &&
            c.z > min.z + 1e-5 &&
            c.z < max.z - 1e-5) {
          buried.add(i);
        }
      }
      final local = BrepXform.unwrapAffine(out)!;
      final capCounts = <int, int>{};
      for (final f in local.solid.faces) {
        if (f.surface is BrepPlane && f.outwardNormal(0, 0).z.abs() > .99) {
          final sign = f.outwardNormal(0, 0).z > 0 ? 1 : -1;
          capCounts[sign] = (capCounts[sign] ?? 0) + 1;
        }
      }
      expect(
        capCounts,
        {1: 1, -1: 1},
        reason:
            'the continuous top and bottom must not retain old feature seams',
      );
      final internalEdges = MeshFaceRegions.featureEdges(mesh).where((edge) {
        final c = (mesh.vertices[edge.$1] + mesh.vertices[edge.$2]) / 2;
        return c.x > min.x + 1e-5 &&
            c.x < max.x - 1e-5 &&
            c.y > min.y + 1e-5 &&
            c.y < max.y - 1e-5;
      }).toList();
      expect(
        internalEdges,
        isEmpty,
        reason: 'X-ray must not draw former boundaries within the new slab',
      );
      expect(
        buried,
        isEmpty,
        reason:
            'surfaces inside the newly added rectangular volume must be removed',
      );
      expect(MeshKit.isClosedSolid(mesh), isTrue);
    });
  }
}
