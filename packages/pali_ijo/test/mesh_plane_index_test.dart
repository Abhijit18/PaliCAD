import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

// Independent exhaustive reference for regular (non-sliver) triangles. It
// deliberately uses no spatial index and matches the supporting-plane rule.
List<int> exhaustivePlanes(MeshData mesh) {
  Vector3 cross(int f) {
    final t = mesh.faces[f];
    return (mesh.vertices[t[1]] - mesh.vertices[t[0]]).cross(
      mesh.vertices[t[2]] - mesh.vertices[t[0]],
    );
  }

  final seeds = List.generate(mesh.faces.length, (i) => i)
    ..sort((a, b) => cross(b).length2.compareTo(cross(a).length2));
  final ids = List.filled(mesh.faces.length, -1);
  var id = 0;
  for (final seed in seeds) {
    if (ids[seed] >= 0) continue;
    final normal = cross(seed).normalized();
    final origin = mesh.vertices[mesh.faces[seed][0]];
    for (var f = 0; f < mesh.faces.length; f++) {
      if (ids[f] >= 0) continue;
      if (mesh.faces[f].every(
        (v) =>
            (mesh.vertices[v] - origin).dot(normal).abs() <
            MeshFaceRegions.planarGrowOffsetEpsilon,
      )) {
        ids[f] = id;
      }
    }
    id++;
  }
  return ids;
}

void main() {
  test(
    'cold display preparation is deduplicated and adopts the original mesh',
    () async {
      final mesh = MeshBuild.icosphere(
        center: Vector3.zero(),
        radius: 30,
        subdivisions: 3,
      );
      final before = mesh.toJson();
      final first = MeshFaceRegions.prepareAsync(mesh);
      expect(MeshFaceRegions.prepareAsync(mesh), same(first));
      await first;
      expect(mesh.toJson(), before);
      final normals = MeshFaceRegions.cornerNormals(mesh);
      final planar = MeshFaceRegions.planarFaceNormals(mesh);
      await MeshFaceRegions.prepareAsync(mesh);
      expect(MeshFaceRegions.cornerNormals(mesh), same(normals));
      expect(MeshFaceRegions.planarFaceNormals(mesh), same(planar));
      expect(normals, hasLength(mesh.faces.length));
    },
  );

  for (final offset in [Vector3.zero(), Vector3(100000, -200000, 300000)]) {
    test('indexed plane groups match exhaustive search at $offset', () {
      final random = math.Random(47);
      final vertices = <Vector3>[];
      final faces = <List<int>>[];
      void add(List<Vector3> points) {
        faces.add(List.generate(points.length, (i) => vertices.length + i));
        vertices.addAll(points.map((p) => p + offset));
      }

      for (var plane = 0; plane < 12; plane++) {
        final rotation = Quaternion.axisAngle(
          Vector3(1, 2, 3).normalized(),
          plane * .23,
        );
        for (var triangle = 0; triangle < 20; triangle++) {
          final x = random.nextDouble() * 80, y = random.nextDouble() * 80;
          final z = [0.0, .000999, .001001][triangle % 3];
          add(
            [
              Vector3(x, y, z),
              Vector3(x + 2 + random.nextDouble(), y, z),
              Vector3(x, y + 3, z),
            ].map(rotation.rotated).toList(),
          );
        }
      }
      final mesh = MeshData(vertices: vertices, faces: faces);
      expect(MeshFaceRegions.originalFaceIds(mesh), exhaustivePlanes(mesh));
    });
  }

  test('indexed curved-mesh groups match exhaustive search', () {
    final mesh = MeshBuild.icosphere(
      center: Vector3(2, 3, 4),
      radius: 30,
      subdivisions: 2,
    );
    expect(MeshFaceRegions.originalFaceIds(mesh), exhaustivePlanes(mesh));
  });
}
