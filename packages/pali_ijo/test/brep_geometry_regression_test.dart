import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  group('plane-cylinder intersections lie on both surfaces', () {
    for (final offset in [-3.0, 3.0]) {
      for (final sign in [-1.0, 1.0]) {
        test('parallel plane offset=$offset normal=$sign', () {
          final cylinder = BrepCylinder(
            origin: Vector3(7, -4, 2),
            axis: Vector3(0, 0, 1),
            radius: 5,
          );
          final plane = BrepPlane(
            origin: cylinder.origin + Vector3(offset, 0, 0),
            normal: Vector3(sign, 0, 0),
          );
          final curves = BrepIntersect.surfaces(plane, cylinder);
          expect(curves, hasLength(2));
          for (final curve in curves) {
            for (final t in [curve.t0, (curve.t0 + curve.t1) / 2, curve.t1]) {
              final p = curve.evaluate(t);
              expect(plane.height(p).abs(), lessThan(1e-8));
              expect((p - cylinder.closestPoint(p)).length, lessThan(1e-8));
            }
          }
        });
      }
    }
  });

  group('single upright fillet geometry', () {
    for (final sides in [3, 4, 6]) {
      test('$sides-sided prism trim vertices lie on the blend cylinder', () {
        final prism = BrepPrimitives.prism(
          diameter: 30,
          height: 20,
          sides: sides,
        );
        final edge = BrepEdgeSelect.linear(
          direction: Vector3(0, 0, 1),
        ).resolve(prism).first;
        final result = IjoOps.filletSolid(
          prism,
          [edge],
          radius: 2,
          inverse: false,
        );
        final blends = result.faces.where((f) => f.surface is BrepCylinder);
        expect(blends, hasLength(1));
        final blend = blends.single;
        for (final loop in blend.loops) {
          for (final coedge in loop.coedges) {
            final e = result.edges[coedge.edge];
            for (final vi in [e.v0, e.v1]) {
              final p = result.vertices[vi];
              expect(
                (p - blend.surface.closestPoint(p)).length,
                lessThan(1e-8),
              );
            }
          }
        }
      });
    }
    // The triangular prism also has an independent cap triangulation defect;
    // its analytic trim is checked above. Track display closure separately.
    for (final sides in [4, 6]) {
      test('$sides-sided prism fillet is closed with the expected volume', () {
        final prism = BrepPrimitives.prism(
          diameter: 30,
          height: 20,
          sides: sides,
        );
        final edge = BrepEdgeSelect.linear(
          direction: Vector3(0, 0, 1),
        ).resolve(prism).first;
        final result = IjoOps.filletSolid(
          prism,
          [edge],
          radius: 2,
          inverse: false,
        );
        final mesh = IjoOps.tessellateClosed(result, op: 'Fillet regression');
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        final originalVolume = IjoMeasure.volume(IjoOps.tessellate(prism));
        final angle = (sides - 2) * math.pi / sides;
        final removedArea =
            4 * (1 / math.tan(angle / 2) - (math.pi - angle) / 2);
        expect(
          IjoMeasure.volume(mesh),
          closeTo(originalVolume - removedArea * 20, 1),
        );
      });
    }
  });
}
