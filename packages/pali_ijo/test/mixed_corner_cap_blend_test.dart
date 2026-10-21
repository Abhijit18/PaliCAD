import 'dart:convert';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'curved_blend_modes_test.dart' show closed;

BrepSolid roundedCornerPrism() {
  final box = BrepPrimitives.box(60, 40, 25);
  final upright = BrepEdgeSelect.linear(
    direction: Vector3(0, 0, 1),
  ).resolve(box).first;
  return IjoOps.filletSolid(box, [upright], radius: 8, inverse: false);
}

List<int> capEdges(BrepSolid s, bool top) => s.faces
    .firstWhere(
      (f) =>
          f.surface is BrepPlane &&
          f.outwardNormal(0, 0).z * (top ? 1 : -1) > .999,
    )
    .loops
    .first
    .coedges
    .map((c) => c.edge)
    .toList();

void main() {
  for (final top in [true, false]) {
    for (final mode in ['fillet', 'default', 'twoDistance', 'setAngle']) {
      test('$mode ${top ? 'top' : 'bottom'} after one upright fillet', () {
        final original = roundedCornerPrism(),
            saved = jsonEncode(original.toJson());
        for (final e in original.edges) {
          expect(
            e.curve.evaluate(e.curve.t0).distanceTo(original.vertices[e.v0]),
            lessThan(1e-7),
          );
          expect(
            e.curve.evaluate(e.curve.t1).distanceTo(original.vertices[e.v1]),
            lessThan(1e-7),
          );
        }
        const d = 2.0;
        final round = mode == 'fillet';
        final options = ChamferOptions(
          mode: mode == 'twoDistance'
              ? ChamferMode.twoDistance
              : mode == 'setAngle'
              ? ChamferMode.setAngle
              : ChamferMode.defaultMode,
          distance2: 3,
          angleDegrees: 30,
        );
        final out = round
            ? IjoOps.filletSolid(
                original,
                capEdges(original, top),
                radius: d,
                inverse: false,
              )
            : IjoOps.chamferSolid(
                original,
                capEdges(original, top),
                d,
                options: options,
              );
        closed(out);
        expect(jsonEncode(original.toJson()), saved);
        final depth = round ? d : options.secondDistance(d, math.pi / 2);
        // Integrate the exact inset area through the blend. Three sharp
        // corners contribute 3*s²; the existing quarter arc contributes pi*s²/4.
        final area = 60 * 40 - (1 - math.pi / 4) * 64;
        final perimeter = 200 - 16 + math.pi * 4;
        final curvature = 3 + math.pi / 4;
        final removed = round
            ? perimeter * d * d * (1 - math.pi / 4) -
                  curvature * d * d * d * (5 / 3 - math.pi / 2)
            : perimeter * d * depth / 2 - curvature * d * d * depth / 3;
        final mesh = IjoOps.tessellateKeep(
          out,
          tessellation: MeshTessellation.fine,
        );
        expect(MeshKit.signedVolume(mesh), closeTo(area * 25 - removed, 8));
        expect(mesh.faces.length, lessThan(15000));
        if (round) {
          for (final f in out.faces.where(
            (f) => f.surface is BrepSplineSurface,
          )) {
            for (final u in [.17, .53, .83]) {
              final cap = f.surface.evaluate(u, 0),
                  wall = f.surface.evaluate(u, 1);
              expect(cap.z, closeTo(top ? 25 : 0, 1e-7));
              expect(wall.z, closeTo(top ? 23 : 2, 1e-7));
              expect(f.outwardNormal(u, 0).z, closeTo(top ? 1 : -1, 1e-6));
              final support = original.faces
                  .where(
                    (s) =>
                        s.surface is! BrepPlane ||
                        s.outwardNormal(0, 0).z.abs() < .1,
                  )
                  .firstWhere(
                    (s) => s.surface.closestPoint(wall).distanceTo(wall) < 2e-5,
                  );
              final uv = support.surface.parameters(wall)!;
              expect(
                f.outwardNormal(u, 1).dot(support.outwardNormal(uv.$1, uv.$2)),
                closeTo(1, 1e-5),
              );
            }
          }
        }
      });
    }
  }
  test('blend rejects exhausted curved radius without modifying source', () {
    final source = roundedCornerPrism(), saved = jsonEncode(source.toJson());
    for (final round in [true, false]) {
      expect(
        () => round
            ? IjoOps.filletSolid(
                source,
                capEdges(source, true),
                radius: 9,
                inverse: false,
              )
            : IjoOps.chamferSolid(source, capEdges(source, true), 9),
        throwsA(isA<BrepUnsupported>()),
      );
      expect(jsonEncode(source.toJson()), saved);
    }
  });
  test('older reversed line parameterizations can blend either cap', () {
    final source = roundedCornerPrism();
    for (final e in source.edges) {
      final c = e.curve;
      if (c is BrepLine && c.a.z == 0 && c.b.z == 0) {
        e.curve = BrepLine(c.b, c.a);
      }
    }
    final saved = jsonEncode(source.toJson());
    for (final top in [true, false]) {
      closed(
        IjoOps.filletSolid(
          source,
          capEdges(source, top),
          radius: 2,
          inverse: false,
        ),
      );
      expect(jsonEncode(source.toJson()), saved);
    }
  });
  test('fillet one cap then chamfer the opposite cap', () {
    final source = roundedCornerPrism();
    final first = IjoOps.filletSolid(
      source,
      capEdges(source, false),
      radius: 2,
      inverse: false,
    );
    final second = IjoOps.chamferSolid(first, capEdges(first, true), 2);
    closed(second);
    expect(second.faces.any((f) => f.surface is BrepSplineSurface), isTrue);
  });
}
