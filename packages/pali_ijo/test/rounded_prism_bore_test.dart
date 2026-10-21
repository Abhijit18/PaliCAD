import 'dart:math' as math;
import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'mixed_corner_cap_blend_test.dart' show roundedCornerPrism;

void main() {
  for (final replay in [false, true]) {
    test(
      'saved segmented circular bore replay=$replay keeps triangles on its wall',
      () {
        var solid = BrepSolid.fromJson(
          jsonDecode(
                File(
                  'test/fixtures/rounded_prism_bore_saved.brep.json',
                ).readAsStringSync(),
              )
              as Map<String, dynamic>,
        );
        if (replay) {
          // Remove only the bore from the saved topology, then repeat the cut.
          final base = BrepSolid(
            vertices: solid.vertices.take(10).toList(),
            edges: solid.edges.take(15).toList(),
            faces: [
              for (final f in solid.faces.take(7))
                BrepFace(
                  surface: f.surface,
                  loops: [f.loops.first],
                  reversed: f.reversed,
                ),
            ],
          );
          final cylinder = solid.faces[7].surface as BrepCylinder;
          final cutter = BrepPrimitives.cylinder(
            origin: cylinder.origin,
            axis: cylinder.axis,
            radius: cylinder.radius,
            height: 150,
          );
          solid = IjoOps.booleanSolid(base, cutter, IjoOps.subtractOp);
        }
        final original = jsonEncode(solid.toJson());
        expect(BrepHeal.topologyIsClosed(solid), isTrue);
        for (final tess in [
          MeshTessellation.draft,
          MeshTessellation.standard,
          MeshTessellation.fine,
        ]) {
          final mesh = BrepTessellate.run(solid, tessellation: tess);
          expect(MeshKit.isClosedSolid(mesh), isTrue);
          final directions = <(int, int), int>{};
          for (final tri in mesh.faces) {
            for (var i = 0; i < 3; i++) {
              final a = tri[i],
                  b = tri[(i + 1) % 3],
                  key = a < b ? (a, b) : (b, a);
              directions.update(
                key,
                (v) => v + (a < b ? 1 : -1),
                ifAbsent: () => a < b ? 1 : -1,
              );
            }
          }
          expect(directions.values.every((v) => v == 0), isTrue);
          expect(
            MeshKit.signedVolume(mesh),
            closeTo(
              189539.11364673858,
              tess == MeshTessellation.draft ? 25 : 5,
            ),
          );
          for (var i = 0; i < mesh.faces.length; i++) {
            final f = solid.faces[mesh.faceIds![i]];
            if (f.surface is! BrepCylinder || !f.reversed) {
              continue;
            }
            final cyl = f.surface as BrepCylinder;
            final pts = [for (final v in mesh.faces[i]) mesh.vertices[v]];
            final p = (pts[0] + pts[1] + pts[2]) / 3;
            final d = p - cyl.origin, radial = d - cyl.axis * d.dot(cyl.axis);
            expect(
              radial.length,
              greaterThan(cyl.radius * .99),
              reason: 'triangle $i at quality ${tess.quality} crosses bore',
            );
          }
          expect(mesh.faces.length, lessThan(2500));
        }
        expect(jsonEncode(solid.toJson()), original);
      },
    );
  }
  for (final offset in [0.0, .217349, 3.260235]) {
    for (final down in [false, true]) {
      for (final padded in [false, true]) {
        test('circular bore down=$down padded=$padded offset=$offset', () {
          final raw = roundedCornerPrism();
          final base = BrepXform.map(
            raw,
            (p) =>
                Vector3(p.x - 31.15458078461306, p.y - 44.263436201599525, p.z),
          );
          final cutter = BrepPrimitives.cylinder(
            origin: Vector3(
              30 + offset - 31.15458078461306,
              20 + offset - 44.263436201599525,
              down ? (padded ? 26 : 25) : (padded ? -1 : 0),
            ),
            axis: Vector3(0, 0, down ? -1 : 1),
            radius: 8 + offset,
            height: padded ? 27 : 25,
          );
          final out = IjoOps.booleanSolid(base, cutter, IjoOps.subtractOp);
          final mesh = BrepTessellate.run(out);
          expect(BrepHeal.topologyIsClosed(out), isTrue);
          expect(MeshKit.isClosedSolid(mesh), isTrue);
          var count = 0;
          var area = 0.0;
          for (var i = 0; i < mesh.faces.length; i++) {
            final f = out.faces[mesh.faceIds![i]];
            final s = f.surface;
            if (s is! BrepCylinder ||
                (s.origin.x - 30 - offset + 31.15458078461306).abs() > 1e-6) {
              continue;
            }
            final p = [for (final v in mesh.faces[i]) mesh.vertices[v]];
            final mid = (p[0] + p[1] + p[2]) / 3;
            final radial = Vector3(
              mid.x - 30 - offset + 31.15458078461306,
              mid.y - 20 - offset + 44.263436201599525,
              0,
            );
            expect(
              radial.length,
              greaterThan((8 + offset) * .99),
              reason: 'bore triangle $i crosses the cavity',
            );
            final n = (p[1] - p[0]).cross(p[2] - p[0]);
            expect(n.dot(radial), lessThan(0));
            area += n.length / 2;
            count++;
          }
          expect(count, greaterThan(0));
          expect(area, closeTo(2 * math.pi * (8 + offset) * 25, 3));
        });
      }
    }
  }
}
