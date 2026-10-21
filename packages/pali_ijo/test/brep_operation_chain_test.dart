import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void intact(BrepSolid s) {
  expect(BrepHeal.topologyIsClosed(s), isTrue);
  expect(MeshKit.isClosedSolid(BrepTessellate.run(s)), isTrue);
  for (final f in s.faces) {
    for (final loop in f.loops) {
      for (final co in loop.coedges) {
        final c = s.edges[co.edge].curve;
        for (final frac in [0.0, 0.33, 0.67, 1.0]) {
          final p = c.evaluate(c.t0 + (c.t1 - c.t0) * frac);
          expect((p - f.surface.closestPoint(p)).length, lessThan(1e-6));
        }
      }
    }
  }
}

void main() {
  test('closed hollow keeps its cavity when split into bodies', () {
    final shell = IjoOps.hollowSolid(
      BrepPrimitives.box(10, 10, 10),
      [],
      Vector3(0, 0, 1),
      wallThicknessMm: 1,
    );
    intact(shell);
    final parts = IjoOps.splitBrep(shell);
    expect(parts.length, 1);
    expect(IjoMeasure.volumeSolid(parts.single), closeTo(1000 - 512, 1e-6));
  });
  test('closed toroidal pipe cavity stays attached to its exterior', () {
    final path = [
      for (var i = 0; i < 32; i++)
        Vector3(
          20 * math.cos(i * math.pi / 16),
          20 * math.sin(i * math.pi / 16),
          0,
        ),
    ];
    final tube = BrepPipe.run(path, radius: 3, innerRadius: 2, closed: true);
    final parts = IjoOps.splitBrep(tube);
    expect(parts.length, 1);
    expect(parts.single.faces.length, 2);
    expect(
      IjoMeasure.volumeSolid(parts.single),
      closeTo(2 * math.pi * math.pi * 20 * 5, 1e-6),
    );
  });
  test(
    'extruding an annular face preserves the bore and exact curved walls',
    () {
      final tube = BrepPipe.run(
        [Vector3.zero(), Vector3(0, 0, 10)],
        radius: 5,
        innerRadius: 2,
      );
      final face = tube.faces.firstWhere(
        (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > 0.9,
      );
      final extension = BrepExtrude.fromFace(tube, face, Vector3(0, 0, 7));
      intact(extension);
      expect(extension.faces.where((f) => f.loops.length == 2).length, 4);
      expect(
        IjoMeasure.volumeSolid(extension),
        closeTo(math.pi * 21 * 7, 1e-6),
      );
    },
  );
  test('oblique polygon extrusion keeps its actual cap planes', () {
    final prism = BrepExtrude.polygon([
      Vector3(0, 0, 0),
      Vector3(4, 0, 0),
      Vector3(4, 3, 0),
      Vector3(0, 3, 0),
    ], Vector3(3, 2, 5));
    intact(prism);
    expect(IjoMeasure.volumeSolid(prism), closeTo(60, 1e-7));
  });
  test('an oblique circular extrusion is an affine cylinder', () {
    final cylinder = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 3,
      height: 5,
    );
    final cap = cylinder.faces.firstWhere(
      (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > 0,
    );
    final prism = BrepExtrude.fromFace(cylinder, cap, Vector3(2, 1, 6));
    intact(prism);
    expect(prism.faces.every((f) => f.surface is BrepAffineSurface), isTrue);
    expect(IjoMeasure.volumeSolid(prism), closeTo(math.pi * 9 * 6, 1e-6));
  });
  test('hollow an obliquely sliced cylinder reconstructs its ellipse', () {
    final cylinder = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 5,
      height: 20,
    );
    final halves = BrepSlice.parts(
      cylinder,
      Vector3(0, 0, 10),
      Vector3(.1, 0, 1),
    );
    final lower = halves.firstWhere(
      (s) => s.faces.any(
        (f) =>
            f.surface is BrepPlane &&
            ((f.surface as BrepPlane).origin.z).abs() < 1e-8,
      ),
    );
    final cap = lower.faces.indexWhere(
      (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > 0,
    );
    final shell = IjoOps.hollowSolid(
      lower,
      [cap],
      lower.faces[cap].outwardNormal(0, 0),
      wallThicknessMm: 1,
    );
    intact(shell);
  });
  test('bore rim fillet preserves the surrounding block', () {
    final block = IjoOps.booleanSolid(
      BrepPrimitives.box(20, 20, 20),
      BrepPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 4,
        height: 22,
      ),
      IjoOps.subtractOp,
    );
    final edge = block.edges.indexWhere(
      (e) => e.curve is BrepCircle && (e.curve as BrepCircle).center.z > 19,
    );
    final out = IjoOps.filletSolid(block, [edge], radius: 1, inverse: false);
    intact(out);
    expect(IjoMeasure.volumeSolid(out), greaterThan(6800));
    expect(
      IjoMeasure.volumeSolid(out),
      lessThan(IjoMeasure.volumeSolid(block)),
    );
  });
}
