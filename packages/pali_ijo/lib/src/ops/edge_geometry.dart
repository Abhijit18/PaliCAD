import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_data.dart';
import '../mesh/mesh_regions.dart';

/// Face and edge queries shared by fillet, chamfer, and hollow.
class EdgeGeometry {
  EdgeGeometry._();

  static Vector3? faceNormal(MeshData mesh, int faceIndex) {
    final face = mesh.faces[faceIndex];
    if (face.length < 3) return null;
    final n = (mesh.vertices[face[1]] - mesh.vertices[face[0]])
        .cross(mesh.vertices[face[2]] - mesh.vertices[face[0]]);
    if (n.length < 1e-14) return null;
    return n.normalized();
  }

  /// Two distinct outward face normals at edge [a]–[b], plus the edge tangent.
  static (Vector3, Vector3, Vector3)? adjacentOutwardNormals(
    MeshData mesh,
    Vector3 a,
    Vector3 b,
  ) {
    final key = MeshFaceRegions.edgeKey(a, b);
    final normals = <Vector3>[];
    for (var f = 0; f < mesh.faces.length; f++) {
      final face = mesh.faces[f];
      for (var i = 0; i < face.length; i++) {
        final va = mesh.vertices[face[i]];
        final vb = mesh.vertices[face[(i + 1) % face.length]];
        if (MeshFaceRegions.edgeKey(va, vb) != key) continue;
        final n = faceNormal(mesh, f);
        if (n == null) continue;
        if (!normals.any((e) => e.dot(n) > 0.98)) {
          normals.add(n);
        }
      }
    }
    if (normals.length < 2) return null;
    var t = b - a;
    if (t.length < 1e-12) return null;
    t.normalize();
    return (normals[0], normals[1], t);
  }
}
