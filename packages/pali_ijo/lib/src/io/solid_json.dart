import 'dart:convert';

import '../brep/brep.dart';
import '../mesh/mesh_data.dart';

/// What a document holds.
enum IjoDocKind {
  solid,
  mesh;

  static IjoDocKind parse(String s) => switch (s) {
        'solid' => IjoDocKind.solid,
        'mesh' => IjoDocKind.mesh,
        _ => throw FormatException('Unknown document kind "$s"'),
      };
}

/// A self-describing wrapper around one analytic solid or one mesh.
class IjoDocument {
  IjoDocument.solid(BrepSolid solid, {this.name})
      : kind = IjoDocKind.solid,
        _solid = solid,
        _mesh = null;

  IjoDocument.mesh(MeshData mesh, {this.name})
      : kind = IjoDocKind.mesh,
        _solid = null,
        _mesh = mesh;

  /// Bumped only when a change would break an older reader.
  static const formatVersion = 1;

  final IjoDocKind kind;
  final String? name;
  final BrepSolid? _solid;
  final MeshData? _mesh;

  BrepSolid? get solidOrNull => _solid;

  MeshData? get meshOrNull => _mesh;

  /// The analytic solid. Throws when this document is mesh-only.
  BrepSolid get solid =>
      _solid ??
      (throw FormatException(
        'Expected a solid document but found a mesh. A mesh has no analytic '
        'faces, so it cannot take part in booleans, fillets or hollows.',
      ));

  /// Display / export mesh. A solid is tessellated on demand.
  MeshData get mesh => _mesh ?? BrepTessellate.run(_solid!);

  Map<String, dynamic> toJson() => {
        'pali_ijo': formatVersion,
        'kind': kind.name,
        if (name != null) 'name': name,
        'originWatermark': BrepOriginIds.watermark,
        if (_solid != null) 'solid': _solid.toJson(),
        if (_mesh != null) 'mesh': _mesh.toJson(),
      };

  static IjoDocument fromJson(Map<String, dynamic> json) {
    final version = json['pali_ijo'] ?? json['pona_ijo'];
    if (version is! num) {
      throw FormatException(
        'Not a pali ijo document: no "pali_ijo" version field',
      );
    }
    if (version > formatVersion) {
      throw FormatException(
        'Document is format version $version but this build reads up to '
        '$formatVersion. Upgrade pali ijo to open it.',
      );
    }

    final watermark = json['originWatermark'];
    if (watermark is num) BrepOriginIds.observe(watermark.toInt());

    final kind = IjoDocKind.parse(json['kind'] as String? ?? 'solid');
    final name = json['name'] as String?;

    switch (kind) {
      case IjoDocKind.solid:
        final raw = json['solid'];
        if (raw is! Map) {
          throw const FormatException('Solid document has no "solid" field');
        }
        return IjoDocument.solid(
          BrepSolid.fromJson(raw.cast<String, dynamic>()),
          name: name,
        );
      case IjoDocKind.mesh:
        final raw = json['mesh'];
        if (raw is! Map) {
          throw const FormatException('Mesh document has no "mesh" field');
        }
        return IjoDocument.mesh(
          MeshData.fromJson(raw.cast<String, dynamic>()),
          name: name,
        );
    }
  }

  String encode({bool pretty = false}) => pretty
      ? const JsonEncoder.withIndent('  ').convert(toJson())
      : jsonEncode(toJson());

  static IjoDocument decode(String text) {
    final Object? parsed;
    try {
      parsed = jsonDecode(text);
    } on FormatException catch (e) {
      throw FormatException('Document is not valid JSON: ${e.message}');
    }
    if (parsed is! Map) {
      throw const FormatException('Document root must be a JSON object');
    }
    return fromJson(parsed.cast<String, dynamic>());
  }
}
