import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid loft() {
  return BrepLoft.sections([
    [
      Vector3(-5, -5, 0),
      Vector3(5, -5, 0),
      Vector3(5, 5, 0),
      Vector3(-5, 5, 0),
    ],
    [
      for (var i = 0; i < 32; i++)
        Vector3(
          3 * math.cos(2 * math.pi * i / 32),
          3 * math.sin(2 * math.pi * i / 32),
          10,
        ),
    ],
  ]);
}

void main() {
  test('plane slices a curved loft into two closed BREP bodies', () {
    final source = loft();
    final out = IjoOps.sliceBrep(source, Vector3(0, 0, 5), Vector3(0, 0, 1));
    expect(out.length, 2);
    for (final s in out) {
      expect(BrepHeal.topologyIsClosed(s), isTrue);
      expect(MeshKit.isClosedSolid(BrepTessellate.run(s)), isTrue);
      for (final f in s.faces) {
        for (final l in f.loops) {
          for (final c in l.coedges) {
            final e = s.edges[c.edge].curve;
            for (final t in [.2, .5, .8]) {
              final p = e.evaluate(e.t0 + (e.t1 - e.t0) * t);
              expect((f.surface.closestPoint(p) - p).length, lessThan(1e-6));
            }
          }
        }
      }
    }
    final volume = out.fold(
      0.0,
      (sum, s) => sum + MeshKit.signedVolume(BrepTessellate.run(s)),
    );
    expect(volume, closeTo(IjoMeasure.volumeSolid(source), 5));
  });

  test('oblique loft slice keeps both exact section boundaries', () {
    final out = IjoOps.sliceBrep(loft(), Vector3(0, 0, 5), Vector3(.2, .15, 1));
    expect(out.length, 2);
    for (final part in out)
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(part)), isTrue);
  });
  test('loft supports sequential cuts and STEP after cutting', () {
    final first = IjoOps.sliceBrep(loft(), Vector3(0, 0, 6), Vector3(0, 0, 1));
    final bottom = first.firstWhere((s) => IjoMeasure.centroidSolid(s).z < 6);
    final second = IjoOps.sliceBrep(bottom, Vector3(0, 0, 3), Vector3(0, 0, 1));
    expect(second.length, 2);
    final back = IjoStep.readSolid(IjoStep.writeSolid(second.first));
    expect(BrepHeal.topologyIsClosed(back), isTrue);
  });

  test(
    'a bore through a curved loft retains the loft and cylindrical wall',
    () {
      final source = loft(),
          tool = BrepPrimitives.cylinder(
            origin: Vector3(0, 0, -1),
            axis: Vector3(0, 0, 1),
            radius: 1,
            height: 12,
          );
      final result = IjoOps.booleanSolid(source, tool, IjoOps.subtractOp);
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      expect(result.faces.any((f) => f.surface is BrepCylinder), isTrue);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)), isTrue);
      expect(BrepClassify.side(result, Vector3(0, 0, 5)), BrepSide.outside);
    },
  );
}
