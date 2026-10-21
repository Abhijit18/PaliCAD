import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_data.dart';

/// Wavefront OBJ.
///
/// A better mesh interchange than STL in two ways that matter here: vertices
/// are shared, so topology survives the round trip, and faces may have more
/// than three sides, so a tessellated planar region does not have to be
/// shredded into triangles. It is also plain text, which makes it the format
/// to reach for when debugging.
///
/// What it still does not carry is units or solidity. OBJ is a surface format;
/// nothing in the file says the surface is closed.
class IjoObj {
  IjoObj._();

  /// Writes the mesh, one group per B-Rep face when [MeshData.faceIds] is
  /// present.
  ///
  /// The groups are the point of writing OBJ rather than STL for a
  /// tessellation: they carry the face structure into any viewer, so a
  /// 40-triangle cylinder wall arrives as one named thing instead of 40
  /// anonymous ones.
  static String write(
    MeshData mesh, {
    String name = 'pali_ijo',
    bool groupByFace = true,
  }) {
    final out = StringBuffer()
      ..writeln('# pali ijo')
      ..writeln('o $name');

    String f(double x) {
      // Six decimals is a nanometre at CAD scale, and avoids the noisy
      // 17-digit output of Dart's default double formatting.
      final s = x.toStringAsFixed(6);
      return s == '-0.000000' ? '0.000000' : s;
    }

    for (final v in mesh.vertices) {
      out.writeln('v ${f(v.x)} ${f(v.y)} ${f(v.z)}');
    }

    final ids = groupByFace ? mesh.faceIds : null;
    final order = [for (var i = 0; i < mesh.faces.length; i++) i];
    if (ids != null) {
      order.sort((a, b) {
        final ia = a < ids.length ? ids[a] : 0;
        final ib = b < ids.length ? ids[b] : 0;
        return ia.compareTo(ib);
      });
    }
    var currentGroup = -1;
    for (final i in order) {
      if (ids != null && i < ids.length && ids[i] != currentGroup) {
        currentGroup = ids[i];
        out.writeln('g face_$currentGroup');
      }
      // OBJ indices are 1-based, and negative means relative, so there is no
      // index 0 to collide with.
      out.writeln(
        'f ${mesh.faces[i].map((k) => k + 1).join(' ')}',
      );
    }
    return out.toString();
  }

  /// Reads the vertex and face records, ignoring materials, normals, texture
  /// coordinates and curves.
  ///
  /// Normals are ignored deliberately rather than for expedience: a mesh's
  /// normals are implied by its winding, and a file whose stated normals
  /// disagree with its winding is a file with two answers. The winding wins.
  static MeshData read(String text) {
    final vertices = <Vector3>[];
    final faces = <List<int>>[];
    final groups = <int>[];
    var group = -1;
    var sawGroup = false;

    for (final rawLine in text.split('\n')) {
      final hash = rawLine.indexOf('#');
      final line = (hash < 0 ? rawLine : rawLine.substring(0, hash)).trim();
      if (line.isEmpty) continue;

      final parts = line.split(RegExp(r'\s+'));
      switch (parts.first) {
        case 'v':
          if (parts.length < 4) {
            throw FormatException('OBJ vertex needs 3 coordinates: "$line"');
          }
          vertices.add(
            Vector3(
              double.parse(parts[1]),
              double.parse(parts[2]),
              double.parse(parts[3]),
            ),
          );

        case 'f':
          final face = <int>[];
          for (final token in parts.skip(1)) {
            // "v", "v/vt", "v//vn" and "v/vt/vn" all lead with the vertex.
            final index = int.parse(token.split('/').first);
            // Negative indices count back from the vertices seen so far,
            // which is legal OBJ and which exporters do use.
            face.add(index > 0 ? index - 1 : vertices.length + index);
          }
          if (face.length < 3) continue;
          for (final i in face) {
            if (i < 0 || i >= vertices.length) {
              throw FormatException(
                'OBJ face references vertex ${i + 1} but only '
                '${vertices.length} are defined: "$line"',
              );
            }
          }
          faces.add(face);
          groups.add(group);

        case 'g' || 'o':
          // Group names are not kept, only the boundaries between them, since
          // faceIds is a list of ints. A round trip through [write] therefore
          // preserves which faces are grouped together but not what they were
          // called.
          group++;
          sawGroup = true;
      }
    }

    if (vertices.isEmpty) {
      throw const FormatException('No vertices found in OBJ');
    }

    return MeshData(
      vertices: vertices,
      faces: faces,
      faceIds: sawGroup ? _denseGroups(groups) : null,
    );
  }

  /// Renumbers group labels to a gapless 0..n-1, so an OBJ whose first `g`
  /// came after an `o`, or which has empty groups, still yields face ids that
  /// look like a tessellation's.
  static List<int> _denseGroups(List<int> groups) {
    final seen = <int, int>{};
    return [
      for (final g in groups) seen.putIfAbsent(g, () => seen.length),
    ];
  }
}
