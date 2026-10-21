import 'package:vector_math/vector_math_64.dart';

import 'mesh_data.dart';

/// Clone, weld, invert, combine, triangulate, volume.
class MeshEdit {
  MeshEdit._();

  static MeshData clone(MeshData mesh) => mesh.clone();

  /// Fan-triangulates n-gons into a triangle-only mesh.
  static MeshData triangulate(MeshData mesh) {
    final srcIds = _faceIds(mesh);
    final faces = <List<int>>[];
    final outIds = srcIds == null ? null : <int>[];
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final face = mesh.faces[fi];
      if (face.length < 3) continue;
      if (face.length == 3) {
        faces.add(List<int>.from(face));
        outIds?.add(srcIds![fi]);
        continue;
      }
      for (var i = 1; i < face.length - 1; i++) {
        faces.add([face[0], face[i], face[i + 1]]);
        outIds?.add(srcIds![fi]);
      }
    }
    return MeshData(
      vertices: [for (final v in mesh.vertices) v.clone()],
      faces: faces,
      faceIds: outIds,
      edgeIds: mesh.edgeIds == null ? null : Map<String, int>.from(mesh.edgeIds!),
    );
  }

  static double signedVolume(MeshData mesh) {
    var sum = 0.0;
    for (final face in mesh.faces) {
      if (face.length < 3) continue;
      final a = mesh.vertices[face[0]];
      for (var i = 1; i + 1 < face.length; i++) {
        final b = mesh.vertices[face[i]];
        final c = mesh.vertices[face[i + 1]];
        sum += a.dot(b.cross(c));
      }
    }
    return sum / 6.0;
  }

  static MeshData weld(MeshData mesh, {double epsilon = 1e-5}) {
    if (mesh.vertices.isEmpty) return clone(mesh);
    final inv = 1.0 / epsilon;
    final map = <String, int>{};
    final verts = <Vector3>[];
    final remap = List<int>.filled(mesh.vertices.length, 0);
    for (var i = 0; i < mesh.vertices.length; i++) {
      final v = mesh.vertices[i];
      final key =
          '${(v.x * inv).round()}_${(v.y * inv).round()}_${(v.z * inv).round()}';
      final existing = map[key];
      if (existing != null) {
        remap[i] = existing;
      } else {
        remap[i] = verts.length;
        map[key] = verts.length;
        verts.add(v.clone());
      }
    }
    final srcIds = _faceIds(mesh);
    final faces = <List<int>>[];
    final outIds = srcIds == null ? null : <int>[];
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final face = mesh.faces[fi];
      final mapped = [for (final i in face) remap[i]];
      if (mapped.length < 3) continue;
      if (mapped[0] == mapped[1] ||
          mapped[1] == mapped[2] ||
          mapped[2] == mapped[0]) {
        continue;
      }
      faces.add(mapped);
      outIds?.add(srcIds![fi]);
    }
    return MeshData(vertices: verts, faces: faces, faceIds: outIds);
  }

  /// Keeps one triangle per unique vertex triple. Boolean classify can
  /// emit A and inverted-B copies of the same coplanar wall; those read as
  /// non-manifold fans in the wireframe and hide the real boundary from
  /// picking.
  static MeshData dropDuplicateFaces(MeshData mesh) {
    final srcIds = _faceIds(mesh);
    final kept = <List<int>>[];
    final outIds = srcIds == null ? null : <int>[];
    final seen = <String>{};
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final face = mesh.faces[fi];
      if (face.length < 3) continue;
      final ids = [face[0], face[1], face[2]]..sort();
      if (ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2]) {
        continue;
      }
      final key = '${ids[0]}_${ids[1]}_${ids[2]}';
      if (!seen.add(key)) continue;
      kept.add(List<int>.from(face));
      outIds?.add(srcIds![fi]);
    }
    return MeshData(
      vertices: [for (final v in mesh.vertices) v.clone()],
      faces: kept,
      faceIds: outIds,
    );
  }

  static MeshData invert(MeshData mesh) {
    return MeshData(
      vertices: [for (final v in mesh.vertices) v.clone()],
      faces: [for (final f in mesh.faces) f.reversed.toList()],
      faceIds: mesh.faceIds == null ? null : List<int>.from(mesh.faceIds!),
      edgeIds: mesh.edgeIds == null ? null : Map<String, int>.from(mesh.edgeIds!),
    );
  }

  static MeshData combine(List<MeshData> meshes) {
    final vertices = <Vector3>[];
    final faces = <List<int>>[];
    final outIds = <int>[];
    var keepIds = true;
    var offset = 0;
    var idOffset = 0;
    for (final mesh in meshes) {
      final srcIds = _faceIds(mesh);
      if (srcIds == null) keepIds = false;
      vertices.addAll([for (final v in mesh.vertices) v.clone()]);
      for (var fi = 0; fi < mesh.faces.length; fi++) {
        faces.add([for (final i in mesh.faces[fi]) i + offset]);
        if (srcIds != null) outIds.add(srcIds[fi] + idOffset);
      }
      if (srcIds != null && srcIds.isNotEmpty) {
        var maxId = idOffset;
        for (final id in srcIds) {
          if (id + idOffset > maxId) maxId = id + idOffset;
        }
        idOffset = maxId + 1;
      }
      offset += mesh.vertices.length;
    }
    return MeshData(
      vertices: vertices,
      faces: faces,
      faceIds: keepIds && outIds.length == faces.length ? outIds : null,
    );
  }
}

List<int>? _faceIds(MeshData mesh) {
  final ids = mesh.faceIds;
  if (ids == null || ids.length != mesh.faces.length) return null;
  return ids;
}
