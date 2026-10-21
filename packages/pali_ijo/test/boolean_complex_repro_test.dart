import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void expectClosed(MeshData m) {
  expect(MeshSeal.isClosedSolid(m), isTrue);
  expect(MeshEdit.signedVolume(m), greaterThan(0));
}

void main() {
  test('union of two offset cubes stays one closed solid', () {
    final a = MeshPrimitives.cube(20, 20, 20);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(20, 20, 20),
      (p) => p + Vector3(10, 0, 0),
    );
    final u = IjoOps.boolean(a, b, IjoOps.unionOp);
    expectClosed(u);
    expect(IjoOps.splitSolids(u), hasLength(1));
  });

  test('subtract of overlapping cubes stays closed', () {
    final a = MeshPrimitives.cube(30, 20, 20);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(10, 5, -1),
    );
    final cut = IjoOps.boolean(a, b, IjoOps.subtractOp);
    expectClosed(cut);
  });

  test('intersect of overlapping cubes stays closed', () {
    final a = MeshPrimitives.cube(20, 20, 20);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(20, 20, 20),
      (p) => p + Vector3(10, 0, 0),
    );
    final common = IjoOps.boolean(a, b, IjoOps.intersectOp);
    expectClosed(common);
  });
}
