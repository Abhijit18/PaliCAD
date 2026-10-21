/// Faceted STEP read: POLY_LOOP faces become a triangle mesh.
library;

import 'dart:convert';
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_build.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_edit.dart';
import 'step_syntax.dart';

class StepUnsupportedException implements Exception {
  StepUnsupportedException(this.message, {this.entities = const {}});

  final String message;
  final Map<String, int> entities;

  @override
  String toString() {
    if (entities.isEmpty) return message;
    final list = (entities.entries.toList()
          ..sort((a, b) => b.value.compareTo(a.value)))
        .map((e) => e.value == 1 ? e.key : '${e.key} ×${e.value}')
        .join(', ');
    return '$message Found: $list.';
  }
}

class StepReadResult {
  StepReadResult({
    required this.solids,
    required this.names,
    required this.unitName,
    required this.unitScale,
    required this.notes,
  });

  final List<MeshData> solids;
  final List<String> names;
  final String unitName;
  final double unitScale;
  final List<String> notes;

  bool get isEmpty => solids.isEmpty;
}

class _StepMeshReader {
  _StepMeshReader(this._bytes);

  final Uint8List _bytes;

  StepReadResult run() {
    final file = StepFile.parse(_decode(_bytes));
    final meshes = <MeshData>[];
    final names = <String>[];
    for (final list in file.entities.values) {
      for (final record in list) {
        if (record.type != 'FACETED_BREP' &&
            record.type != 'MANIFOLD_SOLID_BREP') {
          continue;
        }
        final name = record.arg(0);
        final shellRef = record.arg(1);
        if (shellRef is! StepRef) continue;
        final mesh = _shell(file, shellRef.id);
        if (mesh == null || mesh.faces.isEmpty) continue;
        meshes.add(mesh);
        names.add(name is StepText ? name.value : '');
      }
    }
    if (meshes.isEmpty) {
      throw StepUnsupportedException(
        'This STEP file contains no faceted solid. Export triangles or STL.',
      );
    }
    return StepReadResult(
      solids: meshes,
      names: names,
      unitName: 'mm',
      unitScale: 1,
      notes: const [],
    );
  }

  MeshData? _shell(StepFile file, int id) {
    final shell = file.anyOf(
      id,
      ['CLOSED_SHELL', 'OPEN_SHELL', 'ORIENTED_CLOSED_SHELL'],
    );
    if (shell == null) return null;
    final list = shell.arg(1);
    if (list is! StepList) return null;
    final verts = <Vector3>[];
    final faces = <List<int>>[];
    final indexOf = <String, int>{};

    int vert(Vector3 p) {
      final k =
          '${p.x.toStringAsFixed(9)},${p.y.toStringAsFixed(9)},${p.z.toStringAsFixed(9)}';
      final existing = indexOf[k];
      if (existing != null) return existing;
      final i = verts.length;
      verts.add(p);
      indexOf[k] = i;
      return i;
    }

    for (final item in list.items) {
      if (item is! StepRef) continue;
      final face = file.anyOf(item.id, ['FACE', 'ADVANCED_FACE', 'FACE_SURFACE']);
      if (face == null) continue;
      final bounds = face.arg(1);
      if (bounds is! StepList || bounds.items.isEmpty) continue;
      final first = bounds.items.first;
      final boundId = first is StepRef ? first.id : null;
      if (boundId == null) continue;
      final bound = file.anyOf(
        boundId,
        ['FACE_OUTER_BOUND', 'FACE_BOUND'],
      );
      if (bound == null) continue;
      final loopRef = bound.arg(1);
      if (loopRef is! StepRef) continue;
      final loop = file.anyOf(loopRef.id, ['POLY_LOOP']);
      if (loop == null) continue;
      final pts = loop.arg(1);
      if (pts is! StepList) continue;
      final ring = <Vector3>[];
      for (final p in pts.items) {
        if (p is! StepRef) continue;
        final rec = file.anyOf(p.id, ['CARTESIAN_POINT']);
        if (rec == null) continue;
        final coords = rec.arg(1);
        if (coords is! StepList || coords.items.length < 3) continue;
        double n(int i) {
          final v = coords.items[i];
          return v is StepNumber ? v.value : 0;
        }

        ring.add(Vector3(n(0), n(1), n(2)));
      }
      if (ring.length < 3) continue;
      final ids = [for (final p in ring) vert(p)];
      if (ids.length == 3) {
        faces.add(ids);
      } else {
        final tris = MeshBuild.earClipPolygon(ring);
        for (final t in tris) {
          faces.add([ids[t[0]], ids[t[1]], ids[t[2]]]);
        }
      }
    }
    if (faces.isEmpty) return null;
    var mesh = MeshData(vertices: verts, faces: faces);
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  static String _decode(Uint8List bytes) {
    try {
      return utf8.decode(bytes);
    } on FormatException {
      return latin1.decode(bytes, allowInvalid: true);
    }
  }
}

StepReadResult readStepMeshes(Uint8List bytes) => _StepMeshReader(bytes).run();
