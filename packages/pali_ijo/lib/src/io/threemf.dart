import 'dart:convert';
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_data.dart';
import 'zip.dart';

/// One mesh object inside a 3MF file. The core spec allows many of these
/// in one archive, the same way STEP can hold several solids.
class ThreeMfPart {
  const ThreeMfPart(this.mesh, {this.name});

  final MeshData mesh;
  final String? name;
}

/// 3D Manufacturing Format — a ZIP of XML, and the right default for solids.
///
/// This is the format to use when a mesh will be read again. Against STL it
/// wins on every count that matters to a kernel: vertices are shared so
/// topology survives, coordinates are decimal text rather than float32, and
/// the file states its units instead of leaving millimetres to be guessed.
/// It is a *lossless mesh*
/// relative to the tessellation — not analytic CAD (STEP's job).
///
/// The core specification is what is implemented: one model containing one
/// or more objects, each a triangle mesh, with the required
/// `[Content_Types].xml` and `_rels/.rels` parts so the archive is a valid
/// OPC package. The beam lattice, slice, production and material extensions
/// are not implemented, and [read] says so rather than silently dropping them.
class IjoThreeMf {
  IjoThreeMf._();

  static const _modelPath = '3D/3dmodel.model';
  static const _modelNamespace =
      'http://schemas.microsoft.com/3dmanufacturing/core/2015/02';
  static const _relationshipType =
      'http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel';

  static const _units = {
    'micron',
    'millimeter',
    'centimeter',
    'inch',
    'foot',
    'meter',
  };

  /// Writes a single-object 3MF.
  ///
  /// [unit] is one of the units the specification allows: `micron`,
  /// `millimeter`, `centimeter`, `inch`, `foot`, `meter`. The kernel works in
  /// millimetres, so that is the default and callers should have a reason to
  /// change it.
  static Uint8List write(
    MeshData mesh, {
    String name = 'pali_ijo',
    String unit = 'millimeter',
  }) =>
      writeAll([ThreeMfPart(mesh, name: name)], unit: unit);

  /// Writes every part as its own `<object>` in one 3MF.
  static Uint8List writeAll(
    List<ThreeMfPart> parts, {
    String name = 'pali_ijo',
    String unit = 'millimeter',
  }) {
    if (parts.isEmpty) {
      throw ArgumentError.value(parts, 'parts', 'nothing to write');
    }
    if (!_units.contains(unit)) {
      throw ArgumentError.value(
        unit,
        'unit',
        '3MF allows only ${_units.join(', ')}',
      );
    }

    final archive = ZipArchive()
      ..addText('[Content_Types].xml', _contentTypes)
      ..addText('_rels/.rels', _rels)
      ..addText(_modelPath, _model(parts, title: name, unit: unit));
    return archive.toBytes();
  }

  /// Reads the first object's mesh.
  ///
  /// Coordinates come back in millimetres regardless of the file's declared
  /// unit, since the kernel has one length unit and converting on the way in
  /// is the only place the conversion can be done once. Multi-object files
  /// should use [readAll].
  static MeshData read(Uint8List bytes) {
    final parts = readAll(bytes);
    if (parts.isEmpty) {
      throw const FormatException('3MF model has no triangles');
    }
    return parts.first.mesh;
  }

  /// Reads every mesh object in the archive.
  static List<ThreeMfPart> readAll(Uint8List bytes) {
    final archive = ZipArchive.fromBytes(bytes);
    final xml = archive.readText(_modelPath) ?? _findModelPart(archive);
    if (xml == null) {
      throw const FormatException(
        'Not a 3MF: the archive has no 3D model part',
      );
    }
    if (xml.contains('<beamlattice')) {
      throw UnsupportedError(
        'This 3MF uses the beam lattice extension, which pali ijo does not '
        'read. Its beams would be silently lost, so the file is refused '
        'instead.',
      );
    }
    final scale = _scaleToMillimetres(xml);
    final objects = _objectPattern.allMatches(xml).toList();
    if (objects.isEmpty) {
      return [ThreeMfPart(_parseMeshXml(xml, scale))];
    }
    final parts = <ThreeMfPart>[];
    for (final m in objects) {
      final attrs = _attributes(m.group(1)!);
      final body = m.group(2)!;
      if (!_vertexPattern.hasMatch(body)) continue;
      final name = attrs['name'];
      parts.add(
        ThreeMfPart(
          _parseMeshXml(body, scale),
          name: (name == null || name.isEmpty) ? null : name,
        ),
      );
    }
    if (parts.isEmpty) {
      throw const FormatException('3MF model has no triangles');
    }
    return parts;
  }

  /// The model part is conventionally at `3D/3dmodel.model` but the
  /// specification only requires the relationship to point at it, so a file
  /// that put it elsewhere is still valid.
  static String? _findModelPart(ZipArchive archive) {
    for (final name in archive.names) {
      if (name.toLowerCase().endsWith('.model')) return archive.readText(name);
    }
    return null;
  }

  static const _contentTypes = '<?xml version="1.0" encoding="UTF-8"?>\n'
      '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
      '<Default Extension="rels" '
      'ContentType="application/vnd.openxmlformats-package.relationships+xml" />'
      '<Default Extension="model" '
      'ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />'
      '</Types>\n';

  static const _rels = '<?xml version="1.0" encoding="UTF-8"?>\n'
      '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
      '<Relationship Target="/$_modelPath" Id="rel0" Type="$_relationshipType" />'
      '</Relationships>\n';

  static String _model(
    List<ThreeMfPart> parts, {
    required String title,
    required String unit,
  }) {
    String f(double x) {
      final s = x.toStringAsFixed(6);
      return s == '-0.000000' ? '0.000000' : s;
    }

    final out = StringBuffer()
      ..writeln('<?xml version="1.0" encoding="UTF-8"?>')
      ..writeln(
        '<model unit="$unit" xml:lang="en-US" xmlns="$_modelNamespace">',
      )
      ..writeln('  <metadata name="Application">pali ijo</metadata>')
      ..writeln('  <metadata name="Title">${_escape(title)}</metadata>')
      ..writeln('  <resources>');

    for (var i = 0; i < parts.length; i++) {
      final part = parts[i];
      final id = i + 1;
      final label = part.name;
      out.write('    <object id="$id" type="model"');
      if (label != null && label.isNotEmpty) {
        out.write(' name="${_escape(label)}"');
      }
      out
        ..writeln('>')
        ..writeln('      <mesh>')
        ..writeln('        <vertices>');
      for (final v in part.mesh.vertices) {
        out.writeln(
          '          <vertex x="${f(v.x)}" y="${f(v.y)}" z="${f(v.z)}" />',
        );
      }
      out
        ..writeln('        </vertices>')
        ..writeln('        <triangles>');
      for (final face in part.mesh.faces) {
        if (face.length < 3) continue;
        for (var t = 1; t + 1 < face.length; t++) {
          out.writeln(
            '          <triangle v1="${face[0]}" v2="${face[t]}" '
            'v3="${face[t + 1]}" />',
          );
        }
      }
      out
        ..writeln('        </triangles>')
        ..writeln('      </mesh>')
        ..writeln('    </object>');
    }

    out.writeln('  </resources>');
    out.writeln('  <build>');
    for (var i = 0; i < parts.length; i++) {
      out.writeln('    <item objectid="${i + 1}" />');
    }
    out
      ..writeln('  </build>')
      ..writeln('</model>');
    return out.toString();
  }

  static final _objectPattern = RegExp(
    r'<object\b([^>]*)>(.*?)</object>',
    caseSensitive: false,
    dotAll: true,
  );
  static final _vertexPattern = RegExp(
    r'<vertex\b([^>]*)/?>',
    caseSensitive: false,
  );
  static final _trianglePattern = RegExp(
    r'<triangle\b([^>]*)/?>',
    caseSensitive: false,
  );
  static final _attributePattern = RegExp(
    '''([A-Za-z0-9:_]+)\\s*=\\s*["']([^"']*)["']''',
  );
  static final _unitPattern = RegExp(
    '''<model\\b[^>]*\\bunit\\s*=\\s*["']([^"']*)["']''',
    caseSensitive: false,
  );

  static MeshData _parseMeshXml(String xml, double scale) {
    final vertices = <Vector3>[];
    for (final m in _vertexPattern.allMatches(xml)) {
      final attrs = _attributes(m.group(1)!);
      final x = double.tryParse(attrs['x'] ?? '');
      final y = double.tryParse(attrs['y'] ?? '');
      final z = double.tryParse(attrs['z'] ?? '');
      if (x == null || y == null || z == null) {
        throw FormatException('3MF vertex is missing a coordinate: ${m[0]}');
      }
      vertices.add(Vector3(x * scale, y * scale, z * scale));
    }
    if (vertices.isEmpty) {
      throw const FormatException('3MF model has no vertices');
    }

    final faces = <List<int>>[];
    for (final m in _trianglePattern.allMatches(xml)) {
      final attrs = _attributes(m.group(1)!);
      final a = int.tryParse(attrs['v1'] ?? '');
      final b = int.tryParse(attrs['v2'] ?? '');
      final c = int.tryParse(attrs['v3'] ?? '');
      if (a == null || b == null || c == null) {
        throw FormatException('3MF triangle is missing a vertex: ${m[0]}');
      }
      for (final i in [a, b, c]) {
        if (i < 0 || i >= vertices.length) {
          throw FormatException(
            '3MF triangle references vertex $i but only ${vertices.length} '
            'are defined',
          );
        }
      }
      faces.add([a, b, c]);
    }
    if (faces.isEmpty) {
      throw const FormatException('3MF model has no triangles');
    }

    return MeshData(vertices: vertices, faces: faces);
  }

  /// The file's unit converted to a millimetre factor.
  static double _scaleToMillimetres(String xml) {
    final unit = _unitPattern.firstMatch(xml)?.group(1)?.toLowerCase();
    return switch (unit) {
      null || 'millimeter' => 1.0,
      'micron' => 0.001,
      'centimeter' => 10.0,
      'meter' => 1000.0,
      'inch' => 25.4,
      'foot' => 304.8,
      _ => throw FormatException('3MF declares unknown unit "$unit"'),
    };
  }

  static Map<String, String> _attributes(String source) => {
        for (final m in _attributePattern.allMatches(source))
          m.group(1)!.toLowerCase(): m.group(2)!,
      };

  static String _escape(String s) => const HtmlEscape().convert(s);
}
