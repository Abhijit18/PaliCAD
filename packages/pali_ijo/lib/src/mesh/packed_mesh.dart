import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import 'mesh_data.dart';

/// Packed triangle mesh used to hop isolates without sharing [Vector3] graphs.
class PackedMesh {
  PackedMesh(this.verts, this.tris, [this.faceIds]);

  final Float64List verts;
  final Int32List tris;

  /// CAD face id per triangle, when the source mesh had one.
  final Int32List? faceIds;

  factory PackedMesh.fromMesh(MeshData mesh) {
    final verts = Float64List(mesh.vertices.length * 3);
    for (var i = 0; i < mesh.vertices.length; i++) {
      final v = mesh.vertices[i];
      verts[i * 3] = v.x;
      verts[i * 3 + 1] = v.y;
      verts[i * 3 + 2] = v.z;
    }
    final tris = <int>[];
    final outIds = <int>[];
    final srcIds = mesh.faceIds;
    final keepIds = srcIds != null && srcIds.length == mesh.faces.length;
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final face = mesh.faces[fi];
      if (face.length < 3) continue;
      for (var i = 1; i < face.length - 1; i++) {
        tris
          ..add(face[0])
          ..add(face[i])
          ..add(face[i + 1]);
        if (keepIds) outIds.add(srcIds[fi]);
      }
    }
    return PackedMesh(
      verts,
      Int32List.fromList(tris),
      keepIds && outIds.length == tris.length ~/ 3
          ? Int32List.fromList(outIds)
          : null,
    );
  }

  MeshData toMesh() {
    final vertices = <Vector3>[
      for (var i = 0; i < verts.length; i += 3)
        Vector3(verts[i], verts[i + 1], verts[i + 2]),
    ];
    final faces = <List<int>>[
      for (var i = 0; i < tris.length; i += 3)
        [tris[i], tris[i + 1], tris[i + 2]],
    ];
    final ids = faceIds;
    return MeshData(
      vertices: vertices,
      faces: faces,
      faceIds: ids != null && ids.length == faces.length
          ? [for (final id in ids) id]
          : null,
    );
  }
}
