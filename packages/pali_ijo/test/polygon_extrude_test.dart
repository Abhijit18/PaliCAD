import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

List<Vector3> ring(int n, double r) => [
  for (var i = 0; i < n; i++)
    Vector3(
      r * math.cos(i * 2 * math.pi / n + .31),
      r * math.sin(i * 2 * math.pi / n + .31),
      0,
    ),
];
List<(Vector3, Vector3)> lines(List<Vector3> p) => [
  for (var i = 0; i < p.length; i++) (p[i], p[(i + 1) % p.length]),
];
void main() {
  for (final n in [...List.generate(18, (i) => i + 3), 32, 64, 128]) {
    test('$n sided source retains planar sides and polygon volume', () {
      final p = ring(n, 20);
      for (final depth in [-7.0, 7.0]) {
        final s = BrepExtrude.sampledProfile(
          p,
          Vector3(0, 0, depth),
          linearEdges: lines(p),
        );
        expect(s.faces, hasLength(n + 2));
        expect(s.faces.every((f) => f.surface is BrepPlane), isTrue);
        expect(s.edges.every((e) => e.curve is BrepLine), isTrue);
        expect(
          IjoMeasure.volumeSolid(s),
          closeTo(n * 200 * math.sin(2 * math.pi / n) * 7, 1e-5),
        );
        expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(s)), isTrue);
      }
    });
  }
  test(
    'polygon outside with circular hole keeps only the hole cylindrical',
    () {
      final p = ring(8, 20), hole = ring(48, 5);
      final s = BrepExtrude.sampledProfile(
        p,
        Vector3(0, 0, 7),
        holes: [hole],
        linearEdges: lines(p),
      );
      expect(s.faces.where((f) => f.surface is BrepPlane), hasLength(10));
      final cylinders = s.faces.map((f) => f.surface).whereType<BrepCylinder>();
      expect(cylinders, hasLength(1));
      expect(cylinders.single.radius, closeTo(5, 1e-6));
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(s)), isTrue);
    },
  );
  test('circle outside with polygon hole keeps hole walls flat', () {
    final p = ring(48, 20), hole = ring(9, 5);
    final s = BrepExtrude.sampledProfile(
      p,
      Vector3(0, 0, 7),
      holes: [hole],
      linearEdges: lines(hole),
    );
    expect(s.faces.where((f) => f.surface is BrepPlane), hasLength(11));
    expect(s.faces.where((f) => f.surface is BrepCylinder), hasLength(1));
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(s)), isTrue);
  });
  test('straight segments and an actual sampled arc share one outline', () {
    final p = [
      Vector3(-10, -10, 0),
      Vector3(10, -10, 0),
      for (var i = 0; i <= 16; i++)
        Vector3(
          10 * math.cos(i * math.pi / 16),
          10 * math.sin(i * math.pi / 16),
          0,
        ),
    ];
    final s = BrepExtrude.sampledProfile(
      p,
      Vector3(0, 0, 5),
      linearEdges: [(p[0], p[1]), (p[1], p[2]), (p.last, p.first)],
    );
    expect(s.faces.where((f) => f.surface is BrepCylinder), hasLength(1));
    expect(s.faces.where((f) => f.surface is BrepPlane), hasLength(5));
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(s)), isTrue);
  });
}
