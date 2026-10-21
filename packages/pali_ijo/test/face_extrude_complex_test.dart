import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('extrude a rectangular region of a cube', () {
    final box = MeshPrimitives.cube(20, 20, 20);
    var seed = 0;
    var bestZ = -1e9;
    for (var i = 0; i < box.faces.length; i++) {
      final f = box.faces[i];
      if (f.length < 3) continue;
      final z = (box.vertices[f[0]].z +
              box.vertices[f[1]].z +
              box.vertices[f[2]].z) /
          3;
      if (z > bestZ) {
        bestZ = z;
        seed = i;
      }
    }
    final region = MeshFaceRegions.selectionRegion(box, seed);
    expect(region, isNotEmpty);
    final out = MeshBuild.extrudeRegion(
      mesh: box,
      region: region,
      direction: Vector3(0, 0, 8),
    );
    expect(MeshSeal.isClosedSolid(out), isTrue);
    expect(MeshEdit.signedVolume(out).abs(), closeTo(20 * 20 * 8, 50));
    final grown = MeshBoolean.run(box, out, MeshBoolean.unionOp);
    expect(MeshSeal.isClosedSolid(grown), isTrue);
    expect(MeshEdit.signedVolume(grown), greaterThan(8000));
  });

  test('sweep polygon with a hole is closed', () {
    final outer = [
      Vector3(0, 0, 0),
      Vector3(20, 0, 0),
      Vector3(20, 20, 0),
      Vector3(0, 20, 0),
    ];
    final inner = [
      Vector3(6, 6, 0),
      Vector3(14, 6, 0),
      Vector3(14, 14, 0),
      Vector3(6, 14, 0),
    ];
    final mesh = MeshBuild.sweepPolygonWithHoles(outer, [inner], Vector3(0, 0, 10));
    expect(MeshSeal.isClosedSolid(mesh), isTrue);
    expect(MeshEdit.signedVolume(mesh), closeTo(20 * 20 * 10 - 8 * 8 * 10, 1));
  });
}
