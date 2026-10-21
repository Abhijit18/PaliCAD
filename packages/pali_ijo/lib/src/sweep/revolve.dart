import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_build.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_edit.dart';
import '../mesh/mesh_frame.dart';
import '../mesh/mesh_op.dart';
import 'profile.dart';

/// Turning a planar profile about an axis into a closed triangle mesh.
class IjoRevolve {
  IjoRevolve._();

  /// Revolves [profile] about the axis through [origin] along [direction].
  static MeshData run(
    SweepProfile profile,
    Vector3 origin,
    Vector3 direction, {
    double angle = math.pi * 2,
    int segments = 32,
  }) {
    final axis = direction.normalized();
    if (axis.length2 < 0.5) {
      throw MeshOpException('Revolve needs a non-zero axis direction');
    }
    if (angle.abs() < 1e-9) {
      throw MeshOpException('Revolve needs a non-zero angle');
    }
    final ring = profile.ring(arcSegments: math.max(8, segments));
    if (ring.length < 3) {
      throw MeshOpException('Revolve needs a closed profile');
    }
    final full = angle.abs() >= math.pi * 2 - 1e-9;
    if (full) {
      var mesh = MeshBuild.revolvePolygon(
        ring,
        origin,
        axis,
        segments: math.max(8, segments),
      );
      if (mesh.faces.isEmpty) {
        throw MeshOpException('Revolve produced an empty solid');
      }
      if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
      return mesh;
    }
    final nSeg = math.max(3, segments);
    final thetas = [
      for (var s = 0; s <= nSeg; s++) s / nSeg * angle.abs(),
    ];
    var mesh = MeshBuild.revolvePolygon(
      ring,
      origin,
      axis,
      azimuths: thetas,
    );
    if (mesh.faces.isEmpty) {
      throw MeshOpException('Revolve produced an empty solid');
    }
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  /// A lathe turning: (radius, height) pairs revolved about [axis].
  static MeshData lathe(
    List<(double radius, double height)> outline, {
    Vector3? origin,
    Vector3? axis,
    double angle = math.pi * 2,
    int segments = 32,
  }) {
    if (outline.length < 2) {
      throw MeshOpException('A lathe outline needs at least 2 points');
    }
    final o = origin ?? Vector3.zero();
    final a = (axis ?? Vector3(0, 0, 1)).normalized();
    final radial = meshFrameU(a);

    Vector3 at(double r, double h) => o + radial * r + a * h;

    final ring = <Vector3>[
      for (final (r, h) in outline) at(r, h),
      at(0, outline.last.$2),
      at(0, outline.first.$2),
    ];
    return run(
      SweepProfile.closed(ring, normal: radial.cross(a)),
      o,
      a,
      angle: angle,
      segments: segments,
    );
  }
}
