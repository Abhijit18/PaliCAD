import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

List<Vector3> square(double r, double z, {double x = 0}) => [
  Vector3(x - r, -r, z),
  Vector3(x + r, -r, z),
  Vector3(x + r, r, z),
  Vector3(x - r, r, z),
];
List<Vector3> circle(double r, double z, {double x = 0}) => [
  for (var i = 0; i < 32; i++)
    Vector3(
      x + r * math.cos(i * math.pi / 16),
      r * math.sin(i * math.pi / 16),
      z,
    ),
];
void closed(BrepSolid s) {
  expect(BrepHeal.topologyIsClosed(s), isTrue);
  final mesh = BrepTessellate.run(s);
  expect(MeshKit.isClosedSolid(mesh), isTrue);
  expect(MeshKit.signedVolume(mesh), greaterThan(0));
}

void main() {
  test('tapered polygon loft retains planar BREP', () {
    final out = IjoOps.loftPreferSolid([
      SweepProfile.closed(square(5, 0)),
      SweepProfile.closed(square(2, 10, x: 3)),
    ]);
    expect(out.solid, isNotNull);
    closed(out.solid!);
    expect(out.solid!.faces.every((f) => f.surface is BrepPlane), isTrue);
    expect(MeshKit.signedVolume(out.mesh), closeTo(520, 1e-6));
  });
  test(
    'square to circle stores exact ruled surfaces and survives serialization',
    () {
      final s = BrepLoft.tryProfiles(square(5, 0), circle(5, 10))!;
      closed(s);
      closed(s.clone());
      final patches = s.faces
          .where((f) => f.surface is BrepRuledSurface)
          .toList();
      expect(patches.length, 4);
      for (final face in patches) {
        final p = face.surface as BrepRuledSurface;
        for (final u in [0.13, 0.4, 0.87]) {
          final lower = p.lower.evaluate(
            p.lower.t0 + (p.lower.t1 - p.lower.t0) * u,
          );
          final upper = p.upper.evaluate(
            p.upper.t0 + (p.upper.t1 - p.upper.t0) * u,
          );
          expect(
            (p.evaluate(u, 0.37) - (lower * 0.63 + upper * 0.37)).length,
            lessThan(1e-10),
          );
          expect(
            (p.closestPoint(p.evaluate(u, 0.37)) - p.evaluate(u, 0.37)).length,
            lessThan(1e-7),
          );
        }
      }
      final volume = MeshKit.signedVolume(BrepTessellate.run(s));
      expect(volume, inExclusiveRange(785, 1000));
    },
  );
  test('multi section loft sews stations without internal caps', () {
    final s = BrepLoft.sections([
      square(5, 0),
      square(3, 5),
      square(4, 10, x: 2),
    ]);
    closed(s);
    expect(s.faces.length, 10);
    expect(
      MeshKit.signedVolume(BrepTessellate.run(s)),
      closeTo((100 + 60 + 36) * 5 / 3 + (36 + 48 + 64) * 5 / 3, 1e-6),
    );
  });
  test('tapered loft holes share annular caps', () {
    final s = BrepLoft.tryProfiles(
      square(10, 0),
      square(7, 10),
      holes: [(square(3, 0), square(2, 10))],
    )!;
    closed(s);
    expect(s.faces.where((f) => f.loops.length == 2).length, 2);
  });
  test('off-axis circular loft does not get snapped to a cone', () {
    final s = BrepLoft.tryProfiles(circle(5, 0), circle(3, 10, x: 0.2))!;
    closed(s);
    expect(s.faces.any((f) => f.surface is BrepRuledSurface), isTrue);
  });
  test('nonplanar, folded and nonfinite section inputs are rejected', () {
    final bad = square(5, 0)..[2].z = 0.1;
    expect(
      () => BrepLoft.sections([bad, square(5, 10)]),
      throwsA(isA<BrepUnsupported>()),
    );
    expect(
      () => BrepLoft.sections([square(5, 0), square(5, 10), square(5, 5)]),
      throwsA(isA<BrepUnsupported>()),
    );
    expect(
      () => BrepLoft.sections([square(double.nan, 0), square(5, 10)]),
      throwsA(isA<BrepUnsupported>()),
    );
  });
}
