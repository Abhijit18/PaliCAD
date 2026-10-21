import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_data.dart';
import '../mesh/mesh_frame.dart';
import '../mesh/mesh_op.dart';

/// Builds a solid from a stack of rings.
///
/// Every sweep that is not a revolve reduces to this: generate a ring of
/// points per station along the path, then join consecutive rings with a belt
/// of quads and close the ends. Loft, sweep-along-a-path, pipe and helix
/// differ only in how they place the rings, so they all end up here. One
/// skinning routine to get right, four ways to feed it.
///
/// The output is faceted, and honestly so. A general sweep produces a surface
/// this kernel has no exact name for — the tube around a curved path is not a
/// cylinder, a cone, a torus or a sphere — so the choice is between a faceted
/// solid whose error is bounded and visible in the ring count, or a NURBS
/// representation that every other operation would then have to understand.
/// Facets are the right trade for now, and the wrong answer to hide.
///
/// The output is a closed triangle mesh.
class IjoSkin {
  IjoSkin._();

  /// Joins [rings] into a solid.
  ///
  /// All rings must have the same length and wind the same way. [closed] joins
  /// the last ring back to the first and leaves the ends uncapped, which is
  /// what a sweep around a closed path needs.
  static MeshData solid(
    List<List<Vector3>> rings, {
    bool closed = false,
  }) =>
      mesh(rings, closed: closed);

  /// The triangle mesh of the skin, before healing.
  static MeshData mesh(
    List<List<Vector3>> rings, {
    bool closed = false,
  }) {
    if (rings.length < 2) {
      throw MeshOpException('A skin needs at least 2 sections');
    }
    final n = rings.first.length;
    if (n < 3) {
      throw MeshOpException('A skin section needs at least 3 points');
    }
    for (var i = 1; i < rings.length; i++) {
      if (rings[i].length != n) {
        throw MeshOpException(
          'Skin sections must all have the same point count: section 0 has '
          '$n, section $i has ${rings[i].length}. Resample them first — '
          'SweepProfile.resampled does this.',
        );
      }
    }

    final verts = <Vector3>[];
    final faces = <List<int>>[];
    for (final ring in rings) {
      verts.addAll(ring);
    }

    final beltCount = closed ? rings.length : rings.length - 1;
    for (var s = 0; s < beltCount; s++) {
      final a0 = s * n;
      final b0 = ((s + 1) % rings.length) * n;
      for (var i = 0; i < n; i++) {
        final j = (i + 1) % n;
        final p00 = a0 + i;
        final p01 = a0 + j;
        final p11 = b0 + j;
        final p10 = b0 + i;

        // A quad whose four points are actually coplanar is emitted whole, so
        // a prism's sides stay single faces. Otherwise it is split along the
        // shorter diagonal, which keeps the triangles better shaped and the
        // silhouette closer to the intended surface.
        if (_coplanar(verts[p00], verts[p01], verts[p11], verts[p10])) {
          faces.add([p00, p01, p11, p10]);
        } else if ((verts[p11] - verts[p00]).length2 <
            (verts[p10] - verts[p01]).length2) {
          faces.add([p00, p01, p11]);
          faces.add([p00, p11, p10]);
        } else {
          faces.add([p00, p01, p10]);
          faces.add([p01, p11, p10]);
        }
      }
    }

    if (!closed) {
      // The first cap faces backwards along the stack, the last forwards.
      faces.add([for (var i = n - 1; i >= 0; i--) i]);
      final last = (rings.length - 1) * n;
      faces.add([for (var i = 0; i < n; i++) last + i]);
    }

    return MeshData(vertices: verts, faces: faces);
  }

  static bool _coplanar(Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
    final n = (b - a).cross(c - a);
    final scale = n.length;
    if (scale < 1e-18) return true;
    return (d - a).dot(n) / scale < 1e-7;
  }
}

/// A sequence of stations along a path, each with a frame to place a section
/// in.
///
/// Frames are carried along the path by **parallel transport** rather than
/// rebuilt from the curve's own derivatives at each point. The Frenet frame is
/// the textbook choice and the wrong one here: it is defined by the direction
/// the curve is bending, so it spins wildly where the path is nearly straight
/// and is undefined at an inflection, which shows up as a sudden twist in the
/// middle of an otherwise smooth tube. Parallel transport instead rotates each
/// frame by the smallest rotation that lines it up with the next tangent, so
/// the section never twists more than the path forces it to.
class PathFrames {
  PathFrames._(this.points, this.tangents, this.xDirs, this.yDirs, this.closed);

  /// Frames along the polyline [points].
  ///
  /// [initialX] seeds the first frame's reference direction; a sensible one
  /// perpendicular to the first tangent is chosen when it is omitted or
  /// unusable.
  static PathFrames along(
    List<Vector3> points, {
    bool closed = false,
    Vector3? initialX,
  }) {
    final pts = _dedupe(points, closed: closed);
    if (pts.length < 2) {
      throw MeshOpException('A path needs at least 2 distinct points');
    }

    final tangents = <Vector3>[];
    for (var i = 0; i < pts.length; i++) {
      final prev = i == 0 ? (closed ? pts.last : null) : pts[i - 1];
      final next =
          i == pts.length - 1 ? (closed ? pts.first : null) : pts[i + 1];
      Vector3 t;
      if (prev == null) {
        t = next! - pts[i];
      } else if (next == null) {
        t = pts[i] - prev;
      } else {
        // The average of the two adjacent directions, which bisects the corner
        // and keeps the section's area from pinching there.
        t = (pts[i] - prev).normalized() + (next - pts[i]).normalized();
        if (t.length2 < 1e-18) t = next - pts[i];
      }
      tangents.add(t.normalized());
    }

    var x = initialX == null
        ? meshFrameU(tangents.first)
        : _perpendicular(initialX, tangents.first) ?? meshFrameU(tangents.first);
    final xDirs = <Vector3>[x];
    final yDirs = <Vector3>[tangents.first.cross(x).normalized()];

    for (var i = 1; i < pts.length; i++) {
      x = _transport(x, tangents[i - 1], tangents[i]);
      xDirs.add(x);
      yDirs.add(tangents[i].cross(x).normalized());
    }

    return PathFrames._(pts, tangents, xDirs, yDirs, closed);
  }

  /// Frames whose x always points radially away from the axis through
  /// [origin] along [axis].
  ///
  /// Parallel transport is right when the path is arbitrary and there is no
  /// preferred direction, but wrong when there is one. A thread's
  /// cross-section has to arrive with its depth pointing at the axis at every
  /// station, and transport gives no such guarantee — it starts from whatever
  /// reference direction was picked and carries that along. These frames
  /// answer the question the geometry actually poses.
  ///
  /// It works because the radial direction is always perpendicular to a helix
  /// tangent: the tangent has a tangential part and an axial part, and neither
  /// has a radial component.
  static PathFrames aboutAxis(
    List<Vector3> points,
    Vector3 origin,
    Vector3 axis, {
    bool closed = false,
  }) {
    final a = axis.normalized();
    final base = along(points, closed: closed);
    final xs = <Vector3>[];
    final ys = <Vector3>[];
    for (var i = 0; i < base.length; i++) {
      final d = base.points[i] - origin;
      final radial = d - a * d.dot(a);
      // A station sitting on the axis has no radial direction, so the
      // transported frame is the only sensible answer there.
      final x = radial.length > 1e-9
          ? (_perpendicular(radial, base.tangents[i]) ?? base.xDirs[i])
          : base.xDirs[i];
      xs.add(x);
      ys.add(base.tangents[i].cross(x).normalized());
    }
    return PathFrames._(base.points, base.tangents, xs, ys, closed);
  }

  final List<Vector3> points;
  final List<Vector3> tangents;
  final List<Vector3> xDirs;
  final List<Vector3> yDirs;
  final bool closed;

  int get length => points.length;

  /// Total length of the polyline.
  double get pathLength {
    var sum = 0.0;
    for (var i = 0; i + 1 < points.length; i++) {
      sum += (points[i + 1] - points[i]).length;
    }
    if (closed) sum += (points.first - points.last).length;
    return sum;
  }

  /// How far the transported frame drifts on returning to the start of a
  /// closed path, in radians.
  ///
  /// A closed path generally does not come back to its own starting frame —
  /// the shortfall is the path's holonomy, a real geometric property of the
  /// curve and not an error. [closedTwistCorrection] distributes it so the
  /// tube meets itself cleanly.
  double get holonomy {
    if (!closed) return 0;
    final back = _transport(xDirs.last, tangents.last, tangents.first);
    final c = back.dot(xDirs.first).clamp(-1.0, 1.0);
    final s = back.cross(xDirs.first).dot(tangents.first);
    return math.atan2(s, c);
  }

  /// The same frames, with any closed-path drift spread evenly along the path
  /// so the last section lines up with the first.
  PathFrames get closedTwistCorrection {
    if (!closed) return this;
    final drift = holonomy;
    if (drift.abs() < 1e-12) return this;
    return twisted(drift);
  }

  /// The same frames with a total [radians] of twist applied evenly along the
  /// path, which is how a twisted extrusion or a multi-start thread is made.
  PathFrames twisted(double radians) {
    final xs = <Vector3>[];
    final ys = <Vector3>[];
    for (var i = 0; i < points.length; i++) {
      final frac = points.length == 1 ? 0.0 : i / (points.length - 1);
      final q = Quaternion.axisAngle(tangents[i], radians * frac);
      final x = q.rotated(xDirs[i]).normalized();
      xs.add(x);
      ys.add(tangents[i].cross(x).normalized());
    }
    return PathFrames._(points, tangents, xs, ys, closed);
  }

  /// Places [section], given in its own 2D coordinates, at station [i].
  Vector3 place(int i, double u, double v) =>
      points[i] + xDirs[i] * u + yDirs[i] * v;

  /// The smallest rotation taking [from] to [to], applied to [v].
  ///
  /// Rodrigues about the axis `from × to`. When the tangents are parallel
  /// there is nothing to do, and when they are antiparallel the rotation is
  /// genuinely ambiguous, so the frame is kept as it is rather than guessing.
  static Vector3 _transport(Vector3 v, Vector3 from, Vector3 to) {
    final axis = from.cross(to);
    final s = axis.length;
    if (s < 1e-12) return v.clone();
    final angle = math.atan2(s, from.dot(to));
    final rotated =
        Quaternion.axisAngle(axis / s, angle).rotated(v).normalized();
    // Re-orthogonalise against the new tangent so rounding cannot let the
    // frame drift out of the section plane over a long path.
    return _perpendicular(rotated, to) ?? meshFrameU(to);
  }

  /// Circular fillets at polyline corners so a tube of [radius] can turn.
  ///
  /// A sharp 90° corner with a fat pipe folds the section over itself; healing
  /// that fold is what turned a D-loop pipe into a fan of triangles. Smooth
  /// samples (turning less than about 10°) are left alone so an already-round
  /// arc is not re-filleted at every chord.
  static List<Vector3> filletCorners(
    List<Vector3> path, {
    required double radius,
    bool closed = false,
    int segments = 8,
  }) {
    if (radius <= 1e-9 || path.length < 3) return path;
    final pts = _dedupe(path, closed: closed);
    if (pts.length < 3) return pts;
    final n = pts.length;
    Vector3 at(int i) => pts[(i % n + n) % n];

    final out = <Vector3>[];
    if (!closed) out.add(pts.first);

    final start = closed ? 0 : 1;
    final end = closed ? n : n - 1;
    const minTurn = 0.175; // ~10°
    for (var i = start; i < end; i++) {
      final prev = at(i - 1);
      final cur = pts[i];
      final next = at(i + 1);
      final incoming = cur - prev;
      final outgoing = next - cur;
      final inLen = incoming.length;
      final outLen = outgoing.length;
      if (inLen < 1e-9 || outLen < 1e-9) {
        out.add(cur);
        continue;
      }
      final u = incoming / inLen;
      final v = outgoing / outLen;
      final phi = math.acos(u.dot(v).clamp(-1.0, 1.0));
      if (phi < minTurn) {
        out.add(cur);
        continue;
      }
      final tanHalf = math.tan(phi / 2);
      if (tanHalf.abs() < 1e-9) {
        out.add(cur);
        continue;
      }
      var r = radius;
      var inset = r * tanHalf;
      final maxInset = math.min(inLen, outLen) * 0.45;
      if (inset > maxInset) {
        r = maxInset / tanHalf;
        inset = maxInset;
      }
      if (r < 1e-6 || inset < 1e-6) {
        out.add(cur);
        continue;
      }
      final axis = u.cross(v);
      if (axis.length2 < 1e-20) {
        out.add(cur);
        continue;
      }
      final nHat = axis.normalized();
      final t0 = cur - u * inset;
      final t1 = cur + v * inset;
      final inward = nHat.cross(u);
      if (inward.length2 < 1e-20) {
        out.add(cur);
        continue;
      }
      final center = t0 + inward.normalized() * r;
      out.add(t0);
      final a0 = t0 - center;
      final a1 = t1 - center;
      // vector_math's Quaternion.rotated applies the inverse of axis-angle,
      // so a +phi spin around nHat walked the long way around the corner.
      // Rodrigues from a0 toward a1 stays on the short interior arc.
      final sweep = math.atan2(nHat.dot(a0.cross(a1)), a0.dot(a1));
      final steps = math.max(
        2,
        (sweep.abs() / (math.pi / math.max(4, segments))).ceil(),
      );
      for (var s = 1; s < steps; s++) {
        final ang = sweep * s / steps;
        final cos = math.cos(ang);
        final sin = math.sin(ang);
        final rotated = a0 * cos +
            nHat.cross(a0) * sin +
            nHat * (nHat.dot(a0) * (1 - cos));
        out.add(center + rotated);
      }
      out.add(t1);
    }
    if (!closed) out.add(pts.last);
    return _dedupe(out, closed: closed);
  }

  static Vector3? _perpendicular(Vector3 v, Vector3 axis) {
    final p = v - axis * v.dot(axis);
    return p.length > 1e-9 ? p.normalized() : null;
  }

  static List<Vector3> _dedupe(
    List<Vector3> points, {
    required bool closed,
    double tolerance = 1e-9,
  }) {
    final out = <Vector3>[];
    for (final p in points) {
      if (out.isEmpty || (p - out.last).length > tolerance) out.add(p);
    }
    if (closed) {
      while (out.length > 1 && (out.first - out.last).length <= tolerance) {
        out.removeLast();
      }
    }
    return out;
  }
}
