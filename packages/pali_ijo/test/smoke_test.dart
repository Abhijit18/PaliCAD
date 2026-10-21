import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('a box round-trips through the public barrel', () {
    final box = MeshPrimitives.cube(10, 20, 30);
    expect(box.faces, hasLength(12));
    expect(MeshKit.isClosedSolid(box), isTrue);

    final json = box.toJson();
    final again = MeshData.fromJson(json);
    expect(again.faces, hasLength(12));
  });

  test('booleans run without the host application', () {
    final a = MeshPrimitives.cube(10, 10, 10);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 0, 0),
    );
    final union = MeshBoolean.run(a, b, MeshBoolean.unionOp);
    expect(MeshKit.isClosedSolid(union), isTrue);
  });

  test('solid JSON survives a boolean', () {
    final part = MeshBoolean.run(
      MeshPrimitives.cube(20, 20, 20),
      MeshPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 4,
        height: 22,
      ),
      MeshBoolean.subtractOp,
    );
    final again = MeshData.fromJson(part.toJson());
    expect(again.faces.length, part.faces.length);
    expect(MeshKit.isClosedSolid(again), isTrue);
  });

  test('minkowski sum of two cubes is closed', () {
    final sum = MeshMinkowski.sum(
      MeshPrimitives.cube(10, 10, 10),
      MeshPrimitives.cube(10, 10, 10),
    );
    expect(MeshKit.isClosedSolid(sum), isTrue);
  });
}
