import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../ijo_log.dart';
import '../csg/boolean.dart';
import '../tessellation.dart';
import 'mesh_build.dart';
import 'mesh_data.dart';
import 'mesh_edit.dart';
import 'mesh_offset.dart';
import 'mesh_op.dart';
import 'mesh_regions.dart';
import 'mesh_seal.dart';

/// Shell a closed mesh by insetting the wall, then punch opening faces.
class MeshHollow {
  MeshHollow._();

  static MeshData run(
    MeshData mesh, {
    required List<int> openingFaces,
    required Vector3 openingNormal,
    required double wallThicknessMm,
    MeshTessellation tessellation = MeshTessellation.draft,
  }) {
    final t = wallThicknessMm.abs();
    if (t < 1e-9) return mesh;
    IjoLog.trace(
      'mesh-hollow wall=$t '
      '${IjoLog.size(mesh.vertices.length, mesh.faces.length)}',
    );
    var inner = MeshOffset.run(
      mesh,
      -t,
      tessellation: tessellation,
      kernelSubdivisions: 1,
    );
    final innerVol = MeshEdit.signedVolume(inner).abs();
    final outerVol = MeshEdit.signedVolume(mesh).abs();
    if (inner.faces.isEmpty || innerVol < 1e-8) {
      throw MeshOpException('Hollow wall is too thick for this solid');
    }
    if (innerVol >= outerVol * 0.999) {
      throw MeshOpException('Hollow wall is too thick for this solid');
    }
    if (!MeshSeal.isClosedSolid(inner)) {
      inner = MeshSeal.stitchTJunctions(inner);
    }
    if (!MeshSeal.isClosedSolid(inner)) {
      throw MeshOpException('Hollow could not build an inner wall');
    }

    final stitched = _stitchShell(mesh, inner, openingFaces, openingNormal);
    if (stitched != null) {
      IjoLog.trace(
        'mesh-hollow stitch ${IjoLog.size(stitched.vertices.length, stitched.faces.length)}',
      );
      return stitched;
    }
    if (mesh.faces.length > 80) {
      throw MeshOpException('Hollow could not build a shell for this solid');
    }

    var shell = MeshBoolean.run(mesh, inner, MeshBoolean.subtractOp);
    if (!MeshSeal.isClosedSolid(shell)) {
      shell = MeshSeal.stitchTJunctions(shell);
    }
    final punch = openingFaces.isEmpty
        ? null
        : _openingPunch(mesh, openingFaces, openingNormal, t);
    if (punch != null && punch.faces.isNotEmpty) {
      shell = MeshBoolean.run(shell, punch, MeshBoolean.subtractOp);
      if (!MeshSeal.isClosedSolid(shell)) {
        shell = MeshSeal.stitchTJunctions(shell);
      }
    }
    if (shell.faces.isEmpty || !MeshSeal.isClosedSolid(shell)) {
      throw MeshOpException('Hollow produced a broken solid');
    }
    IjoLog.trace(
      'mesh-hollow done ${IjoLog.size(shell.vertices.length, shell.faces.length)}',
    );
    return shell;
  }

  /// Topology-preserving shell: keep outer faces, reverse the inset inner,
  /// and loft the opening rim. Avoids CSG on concave / filleted meshes.
  static MeshData? _stitchShell(
    MeshData outer,
    MeshData inner,
    List<int> openingFaces,
    Vector3 openingNormal,
  ) {
    if (outer.faces.length != inner.faces.length ||
        outer.vertices.length != inner.vertices.length) {
      return null;
    }
    final region = <int>{};
    for (final seed in openingFaces) {
      if (seed < 0 || seed >= outer.faces.length) continue;
      region.addAll(MeshFaceRegions.hollowOpeningRegion(outer, seed));
    }
    if (region.isEmpty) return null;
    final loops = MeshFaceRegions.orderedBoundaryLoops(outer, region.toList());
    if (loops.isEmpty) return null;

    final nOff = outer.vertices.length;
    final verts = [
      for (final v in outer.vertices) v.clone(),
      for (final v in inner.vertices) v.clone(),
    ];
    final faces = <List<int>>[];
    for (var i = 0; i < outer.faces.length; i++) {
      if (region.contains(i)) continue;
      final f = outer.faces[i];
      if (f.length < 3) continue;
      faces.add(List<int>.from(f));
      faces.add([for (final v in f.reversed) v + nOff]);
    }
    final n = openingNormal.length2 < 1e-20
        ? Vector3(0, 0, 1)
        : openingNormal.normalized();
    for (final loop in loops) {
      if (loop.length < 3) continue;
      for (var i = 0; i < loop.length; i++) {
        final a = loop[i];
        final b = loop[(i + 1) % loop.length];
        if (a == b) continue;
        var triN = (outer.vertices[b] - outer.vertices[a])
            .cross(inner.vertices[b] - outer.vertices[a]);
        if (triN.length2 < 1e-20) {
          triN = (outer.vertices[b] - outer.vertices[a])
              .cross(inner.vertices[a] - outer.vertices[a]);
        }
        final flip = triN.dot(n) < 0;
        if (flip) {
          faces.add([a, a + nOff, b + nOff]);
          faces.add([a, b + nOff, b]);
        } else {
          faces.add([a, b, b + nOff]);
          faces.add([a, b + nOff, a + nOff]);
        }
      }
    }
    var shell = MeshData(vertices: verts, faces: faces);
    if (!MeshSeal.isClosedSolid(shell)) {
      shell = MeshSeal.stitchTJunctions(shell);
    }
    if (shell.faces.isEmpty || !MeshSeal.isClosedSolid(shell)) return null;
    final vol = MeshEdit.signedVolume(shell);
    if (vol.abs() < 1e-8) return null;
    if (vol < 0) shell = MeshEdit.invert(shell);
    return shell;
  }

  static MeshData? _openingPunch(
    MeshData mesh,
    List<int> openingFaces,
    Vector3 openingNormal,
    double t,
  ) {
    final n = openingNormal.normalized();
    if (n.length < 1e-9) return null;
    final region = <int>{};
    for (final seed in openingFaces) {
      if (seed < 0 || seed >= mesh.faces.length) continue;
      region.addAll(MeshFaceRegions.hollowOpeningRegion(mesh, seed));
    }
    if (region.isEmpty) return null;
    final loops = MeshFaceRegions.orderedBoundaryLoops(mesh, region.toList());
    if (loops.isEmpty) return null;
    final outset = math.max(0.05, t * 0.02);
    List<Vector3> ringOf(List<int> loop) => [
          for (final i in loop) mesh.vertices[i] + n * outset,
        ];
    final outer = ringOf(loops.first);
    if (outer.length < 3) return null;
    final holes = [
      for (var i = 1; i < loops.length; i++)
        if (loops[i].length >= 3) ringOf(loops[i]),
    ];
    final depth = t + math.max(0.35, t * 0.2);
    final delta = -n * depth;
    if (holes.isEmpty) return MeshBuild.sweepPolygon(outer, delta);
    return MeshBuild.sweepPolygonWithHoles(outer, holes, delta);
  }
}
