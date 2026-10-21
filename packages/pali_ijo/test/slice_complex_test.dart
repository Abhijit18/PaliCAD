import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void expectClosed(MeshData m) {
  expect(MeshSeal.isClosedSolid(m), isTrue);
  expect(MeshEdit.signedVolume(m).abs(), greaterThan(1e-6));
}

void main() {
  test('slice a cube through its midplane', () {
    final box = MeshPrimitives.cube(20, 20, 20);
    final parts = IjoOps.sliceParts(box, Vector3(10, 10, 10), Vector3(0, 0, 1));
    expect(parts.length, greaterThanOrEqualTo(2));
    for (final p in parts) {
      expectClosed(p);
    }
  });

  test('slice a cylinder through its axis', () {
    final cyl = MeshPrimitives.cylinder(
      origin: Vector3(0, 0, 0),
      axis: Vector3(0, 0, 1),
      radius: 8,
      height: 20,
    );
    final parts = IjoOps.sliceParts(cyl, Vector3(0, 0, 10), Vector3(0, 0, 1));
    expect(parts, isNotEmpty);
    for (final p in parts) {
      expectClosed(p);
    }
  });

  test('slice a holed extrusion', () {
    final outer = [
      Vector3(0, 0, 0),
      Vector3(30, 0, 0),
      Vector3(30, 20, 0),
      Vector3(0, 20, 0),
    ];
    final inner = [
      Vector3(8, 6, 0),
      Vector3(22, 6, 0),
      Vector3(22, 14, 0),
      Vector3(8, 14, 0),
    ];
    final holed = MeshBuild.sweepPolygonWithHoles(outer, [inner], Vector3(0, 0, 16));
    expectClosed(holed);
    final parts = IjoOps.sliceParts(holed, Vector3(15, 10, 8), Vector3(1, 0, 0));
    expect(parts, isNotEmpty);
    for (final p in parts) {
      expectClosed(p);
    }
  });
}
