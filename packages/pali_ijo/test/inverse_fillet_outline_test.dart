import 'dart:convert';
import 'dart:io';

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';

void main() {
  for (final quality in [
    MeshTessellation.draft,
    MeshTessellation.standard,
    MeshTessellation.fine,
  ]) {
    test(
      'inverse fillet seam outlines reach all four corners at ${quality.quality.name}',
      () {
        final solid = BrepSolid.fromJson(
          jsonDecode(
                File(
                  'test/fixtures/inverse_fillet_hollow_chimney.json',
                ).readAsStringSync(),
              )
              as Map<String, dynamic>,
        );
        final mesh = IjoOps.tessellateClosed(
          BrepHeal.alignCircularTrims(solid),
          tessellation: quality,
        );
        final all = MeshFaceRegions.featureEdges(mesh);
        final visible = MeshFaceRegions.featureEdges(
          mesh,
          includeSmoothCreases: false,
        );
        final seams = [
          for (var i = 0; i < solid.edges.length; i++)
            if (solid.edges[i].curve is BrepEllipse) i,
        ];
        expect(seams, hasLength(4));
        for (final id in seams) {
          final expected = all
              .where((e) => mesh.brepEdgeOfVertices(e.$1, e.$2) == id)
              .toSet();
          final drawn = visible
              .where((e) => mesh.brepEdgeOfVertices(e.$1, e.$2) == id)
              .toSet();
          expect(expected.length, greaterThan(3));
          expect(
            drawn,
            expected,
            reason:
                'The entire CAD edge $id must remain visible as it flattens toward its corner',
          );
          final c = solid.edges[id].curve;
          for (final end in [c.evaluate(c.t0), c.evaluate(c.t1)]) {
            expect(
              drawn.any(
                (e) =>
                    (mesh.vertices[e.$1] - end).length < 1e-6 ||
                    (mesh.vertices[e.$2] - end).length < 1e-6,
              ),
              isTrue,
            );
          }
        }
        final before = jsonEncode(solid.toJson());
        MeshFaceRegions.prepare(mesh).adopt();
        expect(
          MeshFaceRegions.featureEdges(mesh, includeSmoothCreases: false),
          visible,
        );
        expect(jsonEncode(solid.toJson()), before);
      },
    );
  }
  test('fully tangent fillet joins stay hidden', () {
    final solid = IjoOps.filletSolid(
      BrepPrimitives.box(20, 20, 20),
      [0],
      radius: 3,
      inverse: false,
    );
    final mesh = IjoOps.tessellateClosed(solid);
    final shown = MeshFaceRegions.featureEdges(
      mesh,
      includeSmoothCreases: false,
    );
    var checked = 0;
    for (var i = 0; i < solid.edges.length; i++) {
      final edge = solid.edges[i];
      if (edge.curve is! BrepLine) continue;
      final adjacent = BrepFillet.facesForEdge(solid, i);
      if (adjacent.length != 2) continue;
      final f = solid.faces[adjacent[0]], g = solid.faces[adjacent[1]];
      if (![f.surface, g.surface].any((s) => s is BrepCylinder)) continue;
      final mid = edge.curve.evaluate((edge.curve.t0 + edge.curve.t1) / 2);
      final a = f.surface.parameters(mid)!, b = g.surface.parameters(mid)!;
      if (f.outwardNormal(a.$1, a.$2).dot(g.outwardNormal(b.$1, b.$2)) < .9999) {
        continue;
      }
      checked++;
      expect(
        shown.where((e) => mesh.brepEdgeOfVertices(e.$1, e.$2) == i),
        isEmpty,
      );
    }
    expect(checked, greaterThan(0));
  });
}
