import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final length in [20.0, 20000.0]) {
    for (final rotated in [false, true]) {
      test('sparse mesh T-junction length=$length rotated=$rotated', () {
        final q = Quaternion.axisAngle(
          Vector3(1, 2, 3).normalized(),
          rotated ? .71 : 0,
        );
        Vector3 map(Vector3 p) => q.rotated(p) + Vector3(-100, -200, -300);
        final mesh = MeshData(
          vertices: [
            Vector3.zero(),
            Vector3(length, 0, 0),
            Vector3(0, length, 0),
            Vector3(length * .2, length * .8, 0),
            Vector3(length * .2, length * .8, 1),
          ].map(map).toList(),
          faces: [
            [0, 1, 2],
          ],
          faceIds: [41],
        );
        final watch = Stopwatch()..start();
        final result = MeshTopology.splitTJunctions(mesh);
        expect(
          watch.elapsed,
          lessThan(const Duration(seconds: 1)),
          reason:
              'Work must depend on vertices, not thousands of empty bins per millimetre',
        );
        expect(result.faces, hasLength(2));
        expect(
          result.faces.every((f) => f.contains(3)),
          isTrue,
          reason:
              'The split near the far end must not be lost to the traversal guard',
        );
        expect(result.faces.any((f) => f.contains(4)), isFalse);
        expect(result.faceIds, [41, 41]);
        var area = 0.0;
        for (final f in result.faces) {
          area +=
              (result.vertices[f[1]] - result.vertices[f[0]])
                  .cross(result.vertices[f[2]] - result.vertices[f[0]])
                  .length /
              2;
        }
        expect(area, closeTo(length * length / 2, 1e-5));
        expect(mesh.faces, [
          [0, 1, 2],
        ]);
      });
    }
  }
}
