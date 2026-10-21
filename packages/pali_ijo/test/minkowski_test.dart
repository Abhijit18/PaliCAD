import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  group('MeshHull', () {
    test('hull of a cube\'s corners is a closed cube', () {
      final cube = MeshPrimitives.cube(10, 10, 10);
      final hull = MeshHull.ofPoints(cube.vertices);
      expect(MeshSeal.isClosedSolid(hull), isTrue);
      expect(MeshEdit.signedVolume(hull), closeTo(1000, 1e-3));
      expect(MeshHull.isConvex(hull), isTrue);
    });

    test('a box is convex and a holed box is not', () {
      final box = MeshPrimitives.cube(20, 20, 20);
      expect(MeshHull.isConvex(box), isTrue);

      final hole = MeshPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 3,
        height: 22,
      );
      final holed = MeshBoolean.run(box, hole, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(holed), isTrue);
      expect(MeshHull.isConvex(holed), isFalse);
    });

    test('an L of two boxes is not convex', () {
      final a = MeshPrimitives.cube(20, 10, 10);
      final b = MeshPrimitives.translate(
        MeshPrimitives.cube(10, 10, 10),
        Vector3(0, 10, 0),
      );
      final el = MeshBoolean.run(a, b, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(el), isTrue);
      expect(MeshHull.isConvex(el), isFalse);
    });
  });

  group('MeshMinkowski', () {
    test('two cubes sum to a closed 20-cube', () {
      final a = MeshPrimitives.cube(10, 10, 10);
      final b = MeshPrimitives.cube(10, 10, 10);
      final sum = MeshMinkowski.sum(a, b);
      expect(MeshSeal.isClosedSolid(sum), isTrue);
      expect(MeshEdit.signedVolume(sum), closeTo(8000, 1.0));
    });

    test('empty kernel leaves the solid unchanged', () {
      final a = MeshPrimitives.cube(10, 10, 10);
      final empty = MeshData(vertices: const [], faces: const []);
      final sum = MeshMinkowski.sum(a, empty);
      expect(MeshEdit.signedVolume(sum), closeTo(1000, 1e-6));
      final diff = MeshMinkowski.difference(a, empty);
      expect(MeshEdit.signedVolume(diff), closeTo(1000, 1e-6));
    });

    test('cube eroded by a centered cube shrinks and stays closed', () {
      final a = MeshPrimitives.cube(10, 10, 10);
      final kernel = MeshPrimitives.translate(
        MeshPrimitives.cube(2, 2, 2),
        Vector3(-1, -1, -1),
      );
      final eroded = MeshMinkowski.difference(a, kernel);
      expect(MeshSeal.isClosedSolid(eroded), isTrue);
      expect(MeshEdit.signedVolume(eroded), lessThan(1000));
      expect(MeshEdit.signedVolume(eroded), greaterThan(100));
    });

    test('cube dilated by a draft sphere grows and stays closed', () {
      final a = MeshPrimitives.cube(10, 10, 10);
      final ball = MeshPrimitives.sphere(
        center: Vector3.zero(),
        radius: 2,
        tessellation: MeshTessellation.draft,
      );
      final grown = MeshMinkowski.sum(a, ball);
      expect(MeshSeal.isClosedSolid(grown), isTrue);
      expect(MeshEdit.signedVolume(grown), greaterThan(1000));
      expect(MeshHull.isConvex(grown), isTrue);
    });

    test('IjoMesh.offset grows a cube', () {
      final grown = IjoMesh.cube(10).offset(
        1,
        tessellation: MeshTessellation.draft,
      );
      expect(MeshSeal.isClosedSolid(grown.mesh), isTrue);
      expect(grown.volume, greaterThan(1000));
    });
  });
}
