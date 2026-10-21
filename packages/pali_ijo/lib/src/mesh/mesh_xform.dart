import 'package:vector_math/vector_math_64.dart';

import 'mesh_data.dart';

/// Vertex maps: transform, scale, mirror.
class MeshXform {
  MeshXform._();

  static MeshData transformVertices(
    MeshData mesh,
    Vector3 Function(Vector3 v) map, {
    bool reverseWinding = false,
  }) {
    final out = mesh.mappedVertices(map);
    if (!reverseWinding) return out;
    return MeshData(
      vertices: out.vertices,
      faces: [for (final f in out.faces) f.reversed.toList()],
      faceIds: out.faceIds,
      edgeIds: out.edgeIds,
    );
  }

  static MeshData scaleAbout(MeshData mesh, Vector3 center, Vector3 scale) {
    final sx = scale.x;
    final sy = scale.y;
    final sz = scale.z;
    final reverse = sx * sy * sz < 0;
    return transformVertices(
      mesh,
      (v) {
        final d = v - center;
        return Vector3(
          center.x + d.x * sx,
          center.y + d.y * sy,
          center.z + d.z * sz,
        );
      },
      reverseWinding: reverse,
    );
  }

  static MeshData mirror(MeshData mesh, Vector3 origin, Vector3 normal) {
    final n = normal.normalized();
    return transformVertices(
      mesh,
      (v) {
        final d = (v - origin).dot(n);
        return v - n * (2 * d);
      },
      reverseWinding: true,
    );
  }

  /// Center XY, sit on Z=0, and scale so the longest side is [targetMaxDim].
  /// Drops non-finite vertices. STL has no units; this keeps imports on-screen.
  static MeshData fitImported(MeshData mesh, {double targetMaxDim = 120}) {
    if (mesh.vertices.isEmpty || mesh.faces.isEmpty) {
      throw const FormatException('No geometry found in the file.');
    }
    var minX = double.infinity;
    var minY = double.infinity;
    var minZ = double.infinity;
    var maxX = -double.infinity;
    var maxY = -double.infinity;
    var maxZ = -double.infinity;
    var any = false;
    for (final v in mesh.vertices) {
      if (!v.x.isFinite || !v.y.isFinite || !v.z.isFinite) continue;
      any = true;
      if (v.x < minX) minX = v.x;
      if (v.y < minY) minY = v.y;
      if (v.z < minZ) minZ = v.z;
      if (v.x > maxX) maxX = v.x;
      if (v.y > maxY) maxY = v.y;
      if (v.z > maxZ) maxZ = v.z;
    }
    if (!any) {
      throw const FormatException('All vertices in the file are invalid.');
    }
    final cx = (minX + maxX) * 0.5;
    final cy = (minY + maxY) * 0.5;
    final maxDim = [
      maxX - minX,
      maxY - minY,
      maxZ - minZ,
    ].fold<double>(0, (a, b) => a > b ? a : b);
    final scale =
        maxDim > 1e-6 ? (targetMaxDim / maxDim).clamp(0.01, 100.0) : 1.0;
    return MeshData(
      vertices: [
        for (final v in mesh.vertices)
          Vector3(
            (v.x.isFinite ? v.x - cx : 0) * scale,
            (v.y.isFinite ? v.y - cy : 0) * scale,
            (v.z.isFinite ? v.z - minZ : 0) * scale,
          ),
      ],
      faces: [
        for (final f in mesh.faces)
          if (f.length >= 3 &&
              f.every((i) => i >= 0 && i < mesh.vertices.length))
            List<int>.from(f),
      ],
      faceIds: mesh.faceIds,
    );
  }
}
