import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

// Independent reference: integrate the overlap of two circular sections in Z.
double overlapVolume(double offset, double bottom, double top) {
  double area(double z) {
    final r = math.sqrt(math.max(0, 100 - z * z)), d = offset, b = 4.0;
    if (d >= r + b) return 0;
    if (d <= (r - b).abs()) return math.pi * math.pow(math.min(r, b), 2);
    return r *
            r *
            math.acos(((d * d + r * r - b * b) / (2 * d * r)).clamp(-1, 1)) +
        b *
            b *
            math.acos(((d * d + b * b - r * r) / (2 * d * b)).clamp(-1, 1)) -
        .5 *
            math.sqrt(
              math.max(
                0,
                (-d + r + b) * (d + r - b) * (d - r + b) * (d + r + b),
              ),
            );
  }

  final lo = math.max(-10.0, bottom), hi = math.min(10.0, top);
  if (hi <= lo) return 0;
  const n = 4000;
  final dz = (hi - lo) / n;
  var sum = area(lo) + area(hi);
  for (var i = 1; i < n; i++) sum += (i.isOdd ? 4 : 2) * area(lo + i * dz);
  return sum * dz / 3;
}

void expectManifold(MeshData mesh) {
  expect(mesh.faces, isNotEmpty);
  final uses = <(int, int), int>{}, balance = <(int, int), int>{};
  for (final f in mesh.faces) {
    for (var i = 0; i < f.length; i++) {
      final a = f[i], b = f[(i + 1) % f.length], key = a < b ? (a, b) : (b, a);
      uses[key] = (uses[key] ?? 0) + 1;
      balance[key] = (balance[key] ?? 0) + (a < b ? 1 : -1);
    }
  }
  expect(uses.values, everyElement(2));
  expect(balance.values, everyElement(0));
  expect(MeshKit.isClosedSolid(mesh), isTrue);
}

void verify({
  required double offset,
  required int op,
  bool reverse = false,
  double bottom = -15,
  double top = 15,
  bool moved = false,
}) {
  final sphere = BrepPrimitives.sphere(center: Vector3.zero(), radius: 10);
  final cylinder = BrepPrimitives.cylinder(
    origin: Vector3(offset, 0, bottom),
    axis: Vector3(0, 0, 1),
    radius: 4,
    height: top - bottom,
  );
  final q = moved
      ? Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .73)
      : Quaternion.identity();
  final shift = moved ? Vector3(125, -73, 42) : Vector3.zero();
  Vector3 map(Vector3 p) => q.rotated(p) + shift;
  final a = BrepXform.map(reverse ? cylinder : sphere, map),
      b = BrepXform.map(reverse ? sphere : cylinder, map);
  final watch = Stopwatch()..start();
  final result = IjoOps.booleanSolid(a, b, op);
  expect(BrepHeal.topologyIsClosed(result), isTrue);
  final mesh = BrepTessellate.run(result);
  expectManifold(mesh);
  final iv = overlapVolume(offset, bottom, top),
      sv = 4 / 3 * math.pi * 1000,
      cv = math.pi * 16 * (top - bottom);
  final volume = op == IjoOps.unionOp
      ? sv + cv - iv
      : op == IjoOps.intersectOp
      ? iv
      : (reverse ? cv : sv) - iv;
  expectManifold(
    BrepTessellate.run(result, tessellation: MeshTessellation.draft),
  );
  final fine = BrepTessellate.run(result, tessellation: MeshTessellation.fine);
  expectManifold(fine);
  expect(
    MeshKit.signedVolume(fine),
    closeTo(volume, math.max(.05, volume * .02)),
  );
  expect(
    result.faces.every(
      (f) =>
          f.surface is BrepSphere ||
          f.surface is BrepCylinder ||
          f.surface is BrepPlane,
    ),
    isTrue,
  );
  final curves = result.edges
      .map((e) => e.curve)
      .whereType<BrepIntersectionCurve>()
      .toList();
  expect(curves, isNotEmpty);
  for (final c in curves) {
    for (var i = 0; i <= 12; i++) {
      final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 12);
      expect((c.carrier.closestPoint(p) - p).length, lessThan(1e-6));
      expect((c.target.closestPoint(p) - p).length, lessThan(1e-6));
    }
  }
  BrepClassify.withTrimCache(() {
    final random = math.Random(307);
    for (var i = 0; i < 60; i++) {
      final p = Vector3(
        -11 + random.nextDouble() * 28,
        -11 + random.nextDouble() * 22,
        -14 + random.nextDouble() * 28,
      );
      final radial = math.sqrt(math.pow(p.x - offset, 2) + p.y * p.y);
      if ((p.length - 10).abs() < .08 ||
          (radial - 4).abs() < .08 ||
          (p.z - bottom).abs() < .08 ||
          (p.z - top).abs() < .08)
        continue;
      var ia = p.length < 10, ib = radial < 4 && p.z > bottom && p.z < top;
      if (reverse) {
        final temp = ia;
        ia = ib;
        ib = temp;
      }
      final expected = op == IjoOps.unionOp
          ? ia || ib
          : op == IjoOps.intersectOp
          ? ia && ib
          : ia && !ib;
      expect(
        BrepClassify.isInside(result, map(p)),
        expected,
        reason: 'point $p',
      );
    }
  });
  expect(watch.elapsed, lessThan(const Duration(seconds: 10)));
}

void main() {
  test('off-axis surface branches lie on both quadrics', () {
    final sphere = BrepSphere(center: Vector3(0, 0, 7), radius: 10);
    for (final offset in [2.0, 8.0, 13.999]) {
      final cylinder = BrepCylinder(
        origin: Vector3(offset, 0, -15),
        axis: Vector3(0, 0, 1),
        radius: 4,
      );
      final curves = BrepIntersect.surfaces(cylinder, sphere);
      expect(curves, isNotEmpty);
      expect(
        BrepIntersect.surfaces(sphere, cylinder),
        hasLength(curves.length),
      );
      for (final c in curves) {
        expect(c, isA<BrepIntersectionCurve>());
        for (var i = 0; i <= 40; i++) {
          final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 40);
          expect((p - cylinder.closestPoint(p)).length, lessThan(1e-7));
          expect((p - sphere.closestPoint(p)).length, lessThan(1e-7));
        }
      }
    }
  });

  for (final offset in [2.0, 8.0, 13.0]) {
    for (final op in [IjoOps.unionOp, IjoOps.subtractOp, IjoOps.intersectOp]) {
      for (final reverse in [false, true]) {
        test(
          'offset=$offset op=$op reverse=$reverse',
          () => verify(offset: offset, op: op, reverse: reverse),
        );
      }
    }
  }
  for (final op in [IjoOps.unionOp, IjoOps.subtractOp, IjoOps.intersectOp]) {
    test(
      'moved oblique blind cylinder op=$op',
      () => verify(offset: 8, op: op, top: 2, moved: true),
    );
    test(
      'moved deep bore op=$op',
      () => verify(offset: 2, op: op, top: 2, moved: true),
    );
  }
  test('sphere cap retains its planar trim after an offset bore', () {
    final s = BrepPrimitives.sphere(center: Vector3.zero(), radius: 10);
    final cap = IjoOps.sliceBrep(
      s,
      Vector3(0, 0, 2),
      Vector3(0, 0, 1),
    ).firstWhere((part) => BrepClassify.isInside(part, Vector3(0, 0, 5)));
    expect(BrepHeal.topologyIsClosed(cap), isTrue);
    expectManifold(BrepTessellate.run(cap));
    final tool = BrepPrimitives.cylinder(
      origin: Vector3(2, 0, -15),
      axis: Vector3(0, 0, 1),
      radius: 4,
      height: 30,
    );
    final cut = IjoOps.booleanSolid(cap, tool, IjoOps.subtractOp);
    expect(BrepHeal.topologyIsClosed(cut), isTrue);
    expectManifold(BrepTessellate.run(cut));
    expect(
      MeshKit.signedVolume(BrepTessellate.run(cut)),
      closeTo(math.pi * 64 * (10 - 8 / 3) - overlapVolume(2, 2, 15), 25),
    );
  });
  test('saved offset bore accepts a second crossing cylinder', () {
    var solid = IjoOps.booleanSolid(
      BrepPrimitives.sphere(center: Vector3.zero(), radius: 10),
      BrepPrimitives.cylinder(
        origin: Vector3(2, 0, -15),
        axis: Vector3(0, 0, 1),
        radius: 4,
        height: 30,
      ),
      IjoOps.subtractOp,
    );
    solid = IjoDocument.decode(IjoDocument.solid(solid).encode()).solid;
    final tool = BrepPrimitives.cylinder(
      origin: Vector3(-15, 2, 1),
      axis: Vector3(1, 0, 0),
      radius: 2,
      height: 30,
    );
    final cut = IjoOps.booleanSolid(solid, tool, IjoOps.subtractOp);
    expect(BrepHeal.topologyIsClosed(cut), isTrue);
    expectManifold(BrepTessellate.run(cut));
    expect(
      MeshKit.signedVolume(BrepTessellate.run(cut)),
      lessThan(MeshKit.signedVolume(BrepTessellate.run(solid))),
    );
  });
}
