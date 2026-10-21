import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('cube faceIds survive a translate', () {
    final box = MeshPrimitives.cube(10, 10, 10);
    final moved = MeshXform.transformVertices(box, (p) => p + Vector3(3, 0, 0));
    expect(moved.faceIds, box.faceIds);
  });

  test('CSG result stays a closed mesh', () {
    final a = MeshPrimitives.cube(10, 10, 10);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 0, 0),
    );
    final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
    expect(MeshSeal.isClosedSolid(u), isTrue);
    expect(MeshEdit.signedVolume(u), greaterThan(0));
  });

  test('FaceSelect.all resolves every triangle', () {
    final box = MeshPrimitives.cube(4, 4, 4);
    final ids = FaceSelect.all.resolve(box);
    expect(ids, isNotEmpty);
    expect(ids, hasLength(6));
  });
}
