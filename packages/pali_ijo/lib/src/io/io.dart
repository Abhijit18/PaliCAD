/// Import and export: solid JSON, STEP, STL, OBJ and 3MF.
library;

import 'dart:convert';
import 'dart:typed_data';

import '../mesh/mesh_data.dart';
import 'obj.dart';
import 'solid_json.dart';
import 'step.dart';
import 'step_analytic.dart' show IjoStepBrep;
import 'stl.dart';
import 'threemf.dart';

export 'obj.dart';
export 'solid_json.dart';
export 'step.dart';
export 'stl.dart';
export 'threemf.dart';
export 'zip.dart';

/// The file formats this kernel reads and writes.
enum IjoFormat {
  /// The kernel's own format: an analytic solid, losslessly. See
  /// [IjoDocument].
  json('json', mesh: false),

  /// STEP AP214. Primitives keep their surfaces; mesh solids write as
  /// planar faces. Reading is analytic surfaces plus planar facets. See
  /// [IjoStep].
  step('step', mesh: false),

  /// 3MF. The recommended mesh format — shared vertices, decimal
  /// coordinates, declared units.
  threeMf('3mf', mesh: true),

  /// Wavefront OBJ. Text, shared vertices, polygons rather than only
  /// triangles.
  obj('obj', mesh: true),

  /// Binary STL. Universally supported and structurally the worst of these:
  /// loose triangles, float32, no units.
  stl('stl', mesh: true),

  /// ASCII STL.
  stlAscii('stla', mesh: true);

  const IjoFormat(this.extension, {required this.mesh});

  /// Conventional file extension, without the dot.
  final String extension;

  /// Whether the format carries only triangles. Writing a solid to a mesh
  /// format tessellates it, which is a one-way door: the analytic surfaces
  /// that booleans and fillets need do not come back.
  final bool mesh;

  /// Whether the format's bytes are text rather than binary.
  bool get isText => this != IjoFormat.threeMf && this != IjoFormat.stl;

  /// The format for a path or extension, or null when unrecognised.
  ///
  /// `.stl` maps to binary on write; ASCII is only ever chosen explicitly,
  /// since binary is a quarter the size and every reader handles both.
  static IjoFormat? forPath(String path) {
    final dot = path.lastIndexOf('.');
    final ext = (dot < 0 ? path : path.substring(dot + 1)).toLowerCase();
    return switch (ext) {
      'json' => IjoFormat.json,
      'step' || 'stp' || 'p21' => IjoFormat.step,
      '3mf' => IjoFormat.threeMf,
      'obj' => IjoFormat.obj,
      'stl' => IjoFormat.stl,
      'stla' => IjoFormat.stlAscii,
      _ => null,
    };
  }

  static IjoFormat parse(String path) =>
      forPath(path) ??
      (throw FormatException(
        'Unrecognised format "$path". Use one of: '
        '${IjoFormat.values.map((f) => f.extension).join(', ')}',
      ));

  /// The format [bytes] appear to be in, from their leading content.
  ///
  /// This exists for the case with no filename to go on: bytes arriving on
  /// stdin. All four formats are self-identifying enough to tell apart
  /// reliably, so a pipeline does not have to restate at every stage what it
  /// is carrying.
  ///
  /// Returns null rather than guessing when nothing matches.
  static IjoFormat? sniff(Uint8List bytes) {
    if (bytes.length < 2) return null;

    // ZIP local file header. 3MF is the only archive here.
    if (bytes[0] == 0x50 && bytes[1] == 0x4b) return IjoFormat.threeMf;

    // Binary STL has no magic number, but its length is exactly determined
    // by its triangle count, which is a strong enough signal.
    if (bytes.length >= 84) {
      final declared =
          ByteData.sublistView(bytes).getUint32(80, Endian.little);
      if (84 + declared * 50 == bytes.length) return IjoFormat.stl;
    }

    // Everything left is text. Skipping leading whitespace matters because a
    // pretty-printed document may start with a newline.
    var i = 0;
    while (i < bytes.length && bytes[i] <= 0x20) {
      i++;
    }
    if (i >= bytes.length) return null;
    if (bytes[i] == 0x7b) return IjoFormat.json; // '{'

    final head = String.fromCharCodes(
      bytes.sublist(i, i + 512 < bytes.length ? i + 512 : bytes.length),
    ).toLowerCase();
    // STEP is the one text format here that names itself, by mandate.
    if (head.startsWith('iso-10303-21')) return IjoFormat.step;
    if (head.startsWith('solid')) return IjoFormat.stlAscii;
    // OBJ has no header at all, so it is recognised by its records. The
    // newline-anchored forms avoid matching the word "version" in a comment.
    if (RegExp(r'(^|\n)\s*(v|vn|vt|f|g|o|mtllib|usemtl)\s').hasMatch(head)) {
      return IjoFormat.obj;
    }
    return null;
  }
}

/// Reads and writes documents in any supported format.
///
/// This is the layer the CLI and the app talk to, so that choosing a format
/// is a matter of the filename rather than of which class to reach for.
class IjoIo {
  IjoIo._();

  /// Decodes every independent body in a file.
  ///
  /// STEP files may contain multiple solids. Preserve analytic BREP geometry
  /// whenever the reader supports the file; if it does not, return each
  /// faceted STEP body as a mesh instead. Other interchange formats currently
  /// represent one body per file.
  static List<IjoDocument> decodeAll(
    Uint8List bytes, {
    IjoFormat? format,
    String? name,
  }) {
    final f = format ??
        (name == null ? null : IjoFormat.forPath(name)) ??
        IjoFormat.sniff(bytes);
    if (f != IjoFormat.step) {
      return [decode(bytes, format: f, name: name)];
    }

    final label = name == null ? null : _stem(name);
    try {
      final result = IjoStepBrep.readAll(bytes);
      if (result.solids.isNotEmpty) {
        return [
          for (var i = 0; i < result.solids.length; i++)
            IjoDocument.solid(
              result.solids[i],
              name: result.names[i].trim().isNotEmpty
                  ? result.names[i]
                  : result.solids.length == 1
                      ? label
                      : '${label ?? 'Part'} ${i + 1}',
            ),
        ];
      }
    } on Object catch (error) {
      try {
        final result = IjoStep.readAll(bytes);
        return [
          for (var i = 0; i < result.solids.length; i++)
            IjoDocument.mesh(
              result.solids[i],
              name: result.names[i].trim().isNotEmpty
                  ? result.names[i]
                  : result.solids.length == 1
                      ? label
                      : '${label ?? 'Part'} ${i + 1}',
            ),
        ];
      } on Object catch (fallbackError) {
        throw FormatException(
          'STEP BREP import failed: $error. Faceted STEP import also failed: '
          '$fallbackError',
        );
      }
    }

    final result = IjoStep.readAll(bytes);
    return [
      for (var i = 0; i < result.solids.length; i++)
        IjoDocument.mesh(
          result.solids[i],
          name: result.names[i].trim().isNotEmpty
              ? result.names[i]
              : result.solids.length == 1
                  ? label
                  : '${label ?? 'Part'} ${i + 1}',
        ),
    ];
  }

  /// Encodes [document] as [format].
  ///
  /// Writing a solid to a mesh format tessellates it. That loses the analytic
  /// surfaces, and there is no warning here because the caller picked the
  /// extension; [IjoFormat.mesh] is how a caller asks in advance.
  static Uint8List encode(IjoDocument document, IjoFormat format) {
    switch (format) {
      case IjoFormat.json:
        return Uint8List.fromList(utf8.encode(document.encode(pretty: true)));
      case IjoFormat.step:
        final solid = document.solidOrNull;
        if (solid != null) {
          return IjoStep.writeSolid(
            solid,
            name: document.name ?? 'Part',
          );
        }
        return IjoStep.write(
          document.mesh,
          name: document.name ?? 'Part',
        );
      case IjoFormat.threeMf:
        return IjoThreeMf.write(
          document.mesh,
          name: document.name ?? 'pali_ijo',
        );
      case IjoFormat.obj:
        return Uint8List.fromList(
          utf8.encode(
            IjoObj.write(document.mesh, name: document.name ?? 'pali_ijo'),
          ),
        );
      case IjoFormat.stl:
        return IjoStl.write(document.mesh);
      case IjoFormat.stlAscii:
        return Uint8List.fromList(
          utf8.encode(
            IjoStl.writeAscii(
              document.mesh,
              name: document.name ?? 'pali_ijo',
            ),
          ),
        );
    }
  }

  /// Decodes [bytes] as [format], falling back to [name]'s extension and
  /// then to sniffing the content.
  ///
  /// A mesh format yields a mesh document. It is not reconstructed into an
  /// analytic solid: recognising that 40 coplanar triangles were once one
  /// planar face is a separate problem, and guessing at it would produce a
  /// solid whose faces are a fiction.
  static IjoDocument decode(
    Uint8List bytes, {
    IjoFormat? format,
    String? name,
  }) {
    final f = format ??
        (name == null ? null : IjoFormat.forPath(name)) ??
        IjoFormat.sniff(bytes);
    if (f == null) {
      throw FormatException(
        'Cannot tell what format ${name ?? 'these bytes'} is — it does not '
        'look like solid JSON, STEP, 3MF, OBJ or STL.',
      );
    }

    final label = name == null ? null : _stem(name);
    return switch (f) {
      IjoFormat.json => IjoDocument.decode(utf8.decode(bytes)),
      // Only the first body. A STEP file may hold several, and a document
      // holds one — use [IjoStep.readAll] to get them all.
      IjoFormat.step => () {
          try {
            final analytic = IjoStep.readSolids(bytes);
            if (analytic.solids.isNotEmpty) {
              return IjoDocument.solid(
                analytic.solids.first,
                name: analytic.names.first.isNotEmpty
                    ? analytic.names.first
                    : label,
              );
            }
          } catch (_) {}
          final result = IjoStep.readAll(bytes);
          return IjoDocument.mesh(
            result.solids.first,
            name: result.names.first.isNotEmpty ? result.names.first : label,
          );
        }(),
      IjoFormat.threeMf => IjoDocument.mesh(
          IjoThreeMf.read(bytes),
          name: label,
        ),
      IjoFormat.obj => IjoDocument.mesh(
          IjoObj.read(utf8.decode(bytes)),
          name: label,
        ),
      // One reader for both flavours: it sniffs which, so a file misnamed
      // .stl that is really ASCII still opens.
      IjoFormat.stl || IjoFormat.stlAscii => IjoDocument.mesh(
          IjoStl.read(bytes),
          name: label,
        ),
    };
  }

  /// Convenience for the common case of writing a mesh directly.
  static Uint8List encodeMesh(
    MeshData mesh,
    IjoFormat format, {
    String? name,
  }) =>
      encode(IjoDocument.mesh(mesh, name: name), format);

  /// The filename without its directory or extension.
  static String _stem(String path) {
    final slash = path.lastIndexOf(RegExp(r'[/\\]'));
    final base = slash < 0 ? path : path.substring(slash + 1);
    final dot = base.lastIndexOf('.');
    return dot <= 0 ? base : base.substring(0, dot);
  }
}
