import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_data.dart';

/// Binary and ASCII STL.
///
/// STL is supported because everything reads it, not because it is good.
/// Avoid it as a CAD interchange: an STL is
/// a bag of loose triangles with no shared vertices, no units, no colour, and
/// float32 coordinates. Reading one back gives you 3× the vertices you wrote
/// and no topology, so [read] welds — but welding is a guess about which
/// vertices were meant to be the same, and a model with genuinely coincident
/// but distinct surfaces loses that distinction.
///
/// Prefer 3MF for anything that will be read again. See [IjoThreeMf].
class IjoStl {
  IjoStl._();

  /// Writes binary STL: an 80-byte header, a triangle count, then 50 bytes
  /// per triangle.
  static Uint8List write(MeshData mesh, {String header = 'pali ijo'}) {
    final tris = _triangles(mesh);
    final bytes = Uint8List(84 + tris.length * 50);
    final data = ByteData.sublistView(bytes);

    // The header is free-form, but must not begin with "solid" or readers
    // sniffing the format will try to parse the binary file as ASCII.
    final tag = 'pali ijo — $header';
    for (var i = 0; i < 80 && i < tag.length; i++) {
      final c = tag.codeUnitAt(i);
      bytes[i] = c < 128 ? c : 0x3f; // '?'
    }
    data.setUint32(80, tris.length, Endian.little);

    var p = 84;
    for (final t in tris) {
      final n = _normal(t.a, t.b, t.c);
      for (final v in [n, t.a, t.b, t.c]) {
        data.setFloat32(p, v.x, Endian.little);
        data.setFloat32(p + 4, v.y, Endian.little);
        data.setFloat32(p + 8, v.z, Endian.little);
        p += 12;
      }
      data.setUint16(p, 0, Endian.little); // attribute byte count
      p += 2;
    }
    return bytes;
  }

  /// Writes ASCII STL. Larger and slower to parse than binary; useful when a
  /// human or a diff has to read the result.
  static String writeAscii(MeshData mesh, {String name = 'pali_ijo'}) {
    final out = StringBuffer('solid $name\n');
    String f(double x) => x.toStringAsExponential(6);
    for (final t in _triangles(mesh)) {
      final n = _normal(t.a, t.b, t.c);
      out.writeln('  facet normal ${f(n.x)} ${f(n.y)} ${f(n.z)}');
      out.writeln('    outer loop');
      for (final v in [t.a, t.b, t.c]) {
        out.writeln('      vertex ${f(v.x)} ${f(v.y)} ${f(v.z)}');
      }
      out.writeln('    endloop');
      out.writeln('  endfacet');
    }
    out.writeln('endsolid $name');
    return out.toString();
  }

  /// Reads either flavour, sniffing which from the leading bytes.
  ///
  /// Vertices are welded to [weldTolerance] so the result has usable topology
  /// rather than three unshared corners per triangle. Pass 0 to keep the file
  /// exactly as written.
  static MeshData read(Uint8List bytes, {double weldTolerance = 1e-6}) {
    if (bytes.length < 15) {
      throw const FormatException('File is too short to be an STL');
    }
    final mesh = _looksAscii(bytes)
        ? _readAscii(String.fromCharCodes(bytes))
        : _readBinary(bytes);
    return weldTolerance > 0 ? _weld(mesh, weldTolerance) : mesh;
  }

  /// An ASCII STL starts with "solid", but so do some binary files whose
  /// header happens to say so. Prefer the declared triangle count when it
  /// matches the file length; otherwise look for ASCII keywords in a prefix
  /// so a binary file is never decoded as a giant string (that OOM-crashes).
  static bool _looksAscii(Uint8List bytes) {
    final lead = String.fromCharCodes(
      bytes.take(6),
    ).trimLeft().toLowerCase();
    if (!lead.startsWith('solid')) return false;
    final probeLen = bytes.length < 4096 ? bytes.length : 4096;
    final probe = String.fromCharCodes(bytes.sublist(0, probeLen)).toLowerCase();
    final hasAsciiKeywords =
        probe.contains('facet') && probe.contains('vertex');
    if (bytes.length >= 84) {
      final declared = ByteData.sublistView(bytes).getUint32(80, Endian.little);
      if (declared > 0 && 84 + declared * 50 <= bytes.length) {
        // Declared binary payload fits, including files with trailing bytes.
        // An ASCII STL whose bytes at offset 80 happen to look like a count
        // still has facet/vertex in the prefix.
        return hasAsciiKeywords;
      }
    }
    return hasAsciiKeywords;
  }

  static MeshData _readBinary(Uint8List bytes) {
    final data = ByteData.sublistView(bytes);
    final count = data.getUint32(80, Endian.little);
    final available = (bytes.length - 84) ~/ 50;
    if (count > available) {
      throw FormatException(
        'STL declares $count triangles but only $available fit in the file',
      );
    }

    final vertices = <Vector3>[];
    final faces = <List<int>>[];
    final n = count < available ? count : available;
    for (var i = 0; i < n; i++) {
      // Offset 12 skips the stored normal, which is recomputed from the
      // winding: files disagree with themselves often enough that the
      // vertices are the only trustworthy part.
      final p = 84 + i * 50 + 12;
      final a = Vector3(
        data.getFloat32(p, Endian.little),
        data.getFloat32(p + 4, Endian.little),
        data.getFloat32(p + 8, Endian.little),
      );
      final b = Vector3(
        data.getFloat32(p + 12, Endian.little),
        data.getFloat32(p + 16, Endian.little),
        data.getFloat32(p + 20, Endian.little),
      );
      final c = Vector3(
        data.getFloat32(p + 24, Endian.little),
        data.getFloat32(p + 28, Endian.little),
        data.getFloat32(p + 32, Endian.little),
      );
      if (!_finite(a) || !_finite(b) || !_finite(c)) continue;
      final i0 = vertices.length;
      vertices..add(a)..add(b)..add(c);
      faces.add([i0, i0 + 1, i0 + 2]);
    }
    if (faces.isEmpty) {
      throw const FormatException('No triangles found in binary STL');
    }
    return MeshData(vertices: vertices, faces: faces);
  }

  static final _vertexPattern = RegExp(
    r'vertex\s+(\S+)\s+(\S+)\s+(\S+)',
    caseSensitive: false,
  );

  static MeshData _readAscii(String text) {
    final vertices = <Vector3>[];
    final faces = <List<int>>[];
    for (final m in _vertexPattern.allMatches(text)) {
      final x = double.tryParse(m.group(1)!);
      final y = double.tryParse(m.group(2)!);
      final z = double.tryParse(m.group(3)!);
      if (x == null || y == null || z == null) continue;
      if (!x.isFinite || !y.isFinite || !z.isFinite) continue;
      vertices.add(Vector3(x, y, z));
      if (vertices.length % 3 == 0) {
        final i = vertices.length - 3;
        faces.add([i, i + 1, i + 2]);
      }
    }
    // A trailing partial facet is a truncated file, and dropping the stray
    // vertices is friendlier than refusing the whole model.
    if (vertices.length % 3 != 0) {
      vertices.removeRange(faces.length * 3, vertices.length);
    }
    if (faces.isEmpty) {
      throw const FormatException('No facets found in ASCII STL');
    }
    return MeshData(vertices: vertices, faces: faces);
  }

  /// Merges vertices within [tolerance] using a grid hash, so the cost is
  /// linear rather than quadratic in the vertex count.
  static MeshData _weld(MeshData mesh, double tolerance) {
    final cell = tolerance * 2;
    final buckets = <String, List<int>>{};
    final out = <Vector3>[];
    final remap = List<int>.filled(mesh.vertices.length, 0);
    final tol2 = tolerance * tolerance;

    for (var i = 0; i < mesh.vertices.length; i++) {
      final v = mesh.vertices[i];
      final gx = (v.x / cell).floor();
      final gy = (v.y / cell).floor();
      final gz = (v.z / cell).floor();

      var found = -1;
      // The 27 neighbouring cells, because a point near a cell boundary has
      // its match on the other side of it.
      outer:
      for (var dx = -1; dx <= 1; dx++) {
        for (var dy = -1; dy <= 1; dy++) {
          for (var dz = -1; dz <= 1; dz++) {
            final near = buckets['${gx + dx},${gy + dy},${gz + dz}'];
            if (near == null) continue;
            for (final j in near) {
              if (out[j].distanceToSquared(v) <= tol2) {
                found = j;
                break outer;
              }
            }
          }
        }
      }

      if (found < 0) {
        found = out.length;
        out.add(v);
        (buckets['$gx,$gy,$gz'] ??= []).add(found);
      }
      remap[i] = found;
    }

    final faces = <List<int>>[];
    for (final f in mesh.faces) {
      final t = [for (final i in f) remap[i]];
      // Welding can collapse a sliver triangle to a line, which is a
      // degenerate face rather than a real one.
      if (t[0] != t[1] && t[1] != t[2] && t[0] != t[2]) faces.add(t);
    }
    return MeshData(vertices: out, faces: faces);
  }

  static bool _finite(Vector3 v) =>
      v.x.isFinite && v.y.isFinite && v.z.isFinite;

  static List<_Tri> _triangles(MeshData mesh) {
    final out = <_Tri>[];
    for (final f in mesh.faces) {
      if (f.length < 3) continue;
      // Fan-triangulate, since STL has only triangles and a mesh may carry
      // quads or larger polygons.
      for (var i = 1; i + 1 < f.length; i++) {
        out.add(
          _Tri(
            mesh.vertices[f[0]],
            mesh.vertices[f[i]],
            mesh.vertices[f[i + 1]],
          ),
        );
      }
    }
    return out;
  }

  static Vector3 _normal(Vector3 a, Vector3 b, Vector3 c) {
    final n = (b - a).cross(c - a);
    final len = n.length;
    return len > 1e-18 ? n / len : Vector3.zero();
  }
}

class _Tri {
  _Tri(this.a, this.b, this.c);
  final Vector3 a;
  final Vector3 b;
  final Vector3 c;
}
