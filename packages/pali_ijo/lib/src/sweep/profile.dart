import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_frame.dart';
import '../mesh/mesh_op.dart';

/// One side of a closed profile: a straight run or a circular arc.
///
/// Arcs are carried through rather than pre-sampled because [IjoRevolve] can
/// turn them into exact spheres and tori. Sampling them at the door would
/// throw that away and leave a revolved circle as a faceted approximation of
/// a torus, which is the sort of loss that compounds through later booleans.
sealed class ProfileSegment {
  const ProfileSegment();

  Vector3 get start;
  Vector3 get end;

  /// Points along the segment, [start] included and [end] excluded, so that
  /// concatenating a closed profile's segments yields each vertex once.
  List<Vector3> sample(int arcSegments);

  ProfileSegment mapped(Vector3 Function(Vector3 p) f);
  ProfileSegment get reversed;

}

class ProfileLine extends ProfileSegment {
  const ProfileLine(this.start, this.end);

  @override
  final Vector3 start;
  @override
  final Vector3 end;

  double get length => (end - start).length;

  @override
  List<Vector3> sample(int arcSegments) => [start];

  @override
  ProfileSegment mapped(Vector3 Function(Vector3 p) f) =>
      ProfileLine(f(start), f(end));

  @override
  ProfileSegment get reversed => ProfileLine(end, start);

  @override
  String toString() => 'line(${_fmt(start)} → ${_fmt(end)})';
}

/// A circular arc, given by its supporting circle and a parameter range.
class ProfileArc extends ProfileSegment {
  ProfileArc({
    required this.center,
    required Vector3 normal,
    required this.radius,
    required Vector3 xDir,
    required this.t0,
    required this.t1,
  })  : normal = normal.normalized(),
        xDir = _inPlane(xDir, normal.normalized()) {
    yDir = this.normal.cross(this.xDir).normalized();
  }

  static Vector3 _inPlane(Vector3 x, Vector3 n) {
    final p = x - n * x.dot(n);
    if (p.length < 1e-10) {
      throw MeshOpException('An arc needs an xDir off its own axis');
    }
    return p.normalized();
  }

  /// The arc through three points, in the plane they span.
  ///
  /// Returns null when the points are collinear, which has no circle through
  /// it — use a [ProfileLine] there instead.
  static ProfileArc? through(Vector3 a, Vector3 b, Vector3 c) {
    final ab = b - a;
    final ac = c - a;
    final n = ab.cross(ac);
    if (n.length2 < 1e-20) return null;

    // Circumcentre of the triangle, which is the centre of the only circle
    // through all three points.
    final center = a +
        (n.cross(ab) * ac.length2 + ac.cross(n) * ab.length2) /
            (2 * n.length2);

    final radius = (a - center).length;
    final normal = n.normalized();
    final xDir = (a - center).normalized();
    final yDir = normal.cross(xDir);

    double angleOf(Vector3 p) {
      final d = p - center;
      return math.atan2(d.dot(yDir), d.dot(xDir));
    }

    var tb = angleOf(b);
    var tc = angleOf(c);
    // a is at t = 0 by construction; unwrap forward so the arc passes b.
    if (tb < 0) tb += math.pi * 2;
    if (tc < tb) tc += math.pi * 2;

    return ProfileArc(
      center: center,
      normal: normal,
      radius: radius,
      xDir: xDir,
      t0: 0,
      t1: tc,
    );
  }

  final Vector3 center;
  final Vector3 normal;
  final Vector3 xDir;
  late final Vector3 yDir;
  final double radius;
  final double t0;
  final double t1;

  Vector3 pointAt(double t) =>
      center + xDir * (radius * math.cos(t)) + yDir * (radius * math.sin(t));

  double get sweepAngle => t1 - t0;

  bool get isFullTurn => sweepAngle.abs() >= math.pi * 2 - 1e-9;

  @override
  Vector3 get start => pointAt(t0);

  @override
  Vector3 get end => pointAt(t1);

  @override
  List<Vector3> sample(int arcSegments) {
    final n = math.max(1, (arcSegments * sweepAngle.abs() / (2 * math.pi))
        .ceil());
    return [
      for (var i = 0; i < n; i++) pointAt(t0 + sweepAngle * i / n),
    ];
  }

  @override
  ProfileSegment mapped(Vector3 Function(Vector3 p) f) {
    // A general map can turn a circle into something that is not one, so the
    // arc is refitted from three mapped points and falls back to a straight
    // run when they come out collinear.
    final a = f(start);
    final m = f(pointAt((t0 + t1) / 2));
    final b = f(end);
    return through(a, m, b) ?? ProfileLine(a, b);
  }

  @override
  ProfileSegment get reversed => ProfileArc(
        center: center,
        normal: -normal,
        radius: radius,
        xDir: xDir,
        t0: -t1,
        t1: -t0,
      );

  @override
  String toString() => 'arc(r=${radius.toStringAsFixed(3)}, '
      '${(sweepAngle * 180 / math.pi).toStringAsFixed(1)}°)';
}

/// A closed planar profile: the 2D cross-section a sweep carries through
/// space.
///
/// Planarity is required, and checked. It is not a limitation of the sweep
/// code so much as of what a sweep means: a non-planar section has no
/// well-defined inside, so the solid it would produce is not defined either.
/// Both reference kernels stage their sweeps the same way, planar first.
class SweepProfile {
  SweepProfile._(this.segments, this.normal, this.origin, this.xDir);

  /// A profile from a closed ring of points. The ring must not repeat its
  /// first point at the end.
  factory SweepProfile.closed(
    List<Vector3> points, {
    Vector3? normal,
    Vector3? xDir,
    Vector3? anchor,
  }) {
    final ring = _dedupe(points);
    if (ring.length < 3) {
      throw MeshOpException('A profile needs at least 3 distinct points');
    }
    return SweepProfile.of(
      [
        for (var i = 0; i < ring.length; i++)
          ProfileLine(ring[i], ring[(i + 1) % ring.length]),
      ],
      normal: normal,
      xDir: xDir,
      anchor: anchor,
    );
  }

  /// A profile from explicit segments, which must join end to start and close.
  factory SweepProfile.of(
    List<ProfileSegment> segments, {
    Vector3? normal,
    Vector3? xDir,
    Vector3? anchor,
    double tolerance = 1e-6,
  }) {
    if (segments.isEmpty) {
      throw MeshOpException('A profile needs at least one segment');
    }
    for (var i = 0; i < segments.length; i++) {
      final gap =
          (segments[(i + 1) % segments.length].start - segments[i].end).length;
      if (gap > tolerance) {
        throw MeshOpException(
          'Profile is not closed: segment $i ends '
          '${gap.toStringAsFixed(6)} away from the next one',
        );
      }
    }

    final pts = [
      for (final s in segments) ...s.sample(16),
    ];
    final n = normal?.normalized() ?? _newellNormal(pts);
    if (n.length2 < 0.5) {
      throw MeshOpException('Profile is degenerate: it encloses no area');
    }
    final o = _average(pts);
    for (final p in pts) {
      if ((p - o).dot(n).abs() > math.max(tolerance, 1e-9)) {
        throw MeshOpException(
          'Profile is not planar: a point lies '
          '${(p - o).dot(n).abs().toStringAsFixed(6)} off the best-fit plane',
        );
      }
    }
    return SweepProfile._(
      List.unmodifiable(segments),
      n,
      // The anchor is projected onto the profile's own plane, since a point
      // off it could not be the point that rides the path.
      anchor == null ? o : anchor - n * (anchor - o).dot(n),
      _basis(xDir, n),
    );
  }

  static Vector3 _basis(Vector3? want, Vector3 normal) {
    if (want != null) {
      final p = want - normal * want.dot(normal);
      if (p.length > 1e-9) return p.normalized();
    }
    return meshFrameU(normal);
  }

  /// A regular polygon approximating a circle, for pipes and lofts.
  ///
  /// [segments] trades roundness against face count. This is only for the
  /// faceted sweeps; [IjoRevolve] takes a real arc and stays exact.
  factory SweepProfile.circle({
    required double radius,
    Vector3? center,
    Vector3? normal,
    Vector3? xDir,
    int segments = 32,
  }) {
    final c = center ?? Vector3.zero();
    final n = (normal ?? Vector3(0, 0, 1)).normalized();
    final x = _basis(xDir, n);
    final y = n.cross(x);
    final k = math.max(3, segments);
    return SweepProfile.closed(
      [
        for (var i = 0; i < k; i++)
          c +
              x * (radius * math.cos(2 * math.pi * i / k)) +
              y * (radius * math.sin(2 * math.pi * i / k)),
      ],
      normal: n,
      xDir: x,
    );
  }

  /// An axis-aligned rectangle in the plane through [center] with [normal].
  factory SweepProfile.rectangle({
    required double width,
    required double height,
    Vector3? center,
    Vector3? normal,
    Vector3? xDir,
  }) {
    final c = center ?? Vector3.zero();
    final n = (normal ?? Vector3(0, 0, 1)).normalized();
    final x = _basis(xDir, n);
    final y = n.cross(x);
    final hw = width / 2;
    final hh = height / 2;
    return SweepProfile.closed(
      [
        c - x * hw - y * hh,
        c + x * hw - y * hh,
        c + x * hw + y * hh,
        c - x * hw + y * hh,
      ],
      normal: n,
      xDir: x,
    );
  }

  final List<ProfileSegment> segments;

  /// Unit normal of the profile's plane, right-handed about its winding.
  final Vector3 normal;

  /// The point of the profile that rides the path, and the origin of its 2D
  /// coordinates.
  ///
  /// Defaults to the average of the sampled ring, so sweeping a section
  /// centres it on the path — which is what a pipe wants. Pass an `anchor` to
  /// move it: a thread's groove has to hang off the crest radius rather than
  /// straddle it, so its section is anchored at the crest.
  final Vector3 origin;

  /// The profile's own 2D u-axis, in its plane.
  ///
  /// A sweep flattens the profile into these coordinates and rebuilds it in
  /// each frame along the path, so this is what decides which way the section
  /// faces once it gets there. It is settable rather than derived because a
  /// derived one would be arbitrary: a thread's cross-section has to arrive
  /// with its depth pointing at the axis, and nothing about the ring of points
  /// alone says which of its two in-plane directions that is.
  final Vector3 xDir;

  /// The profile's 2D v-axis, completing a right-handed basis with [xDir] and
  /// [normal].
  Vector3 get yDir => normal.cross(xDir).normalized();

  List<Vector3> get points => [for (final s in segments) s.start];

  /// The ring as a polyline, arcs sampled at [arcSegments] per full turn.
  List<Vector3> ring({int arcSegments = 32}) => [
        for (final s in segments) ...s.sample(arcSegments),
      ];

  bool get hasArcs => segments.any((s) => s is ProfileArc);

  /// Enclosed area, always positive.
  ///
  /// Exact for straight segments; arcs contribute their exact circular
  /// segment area on top of the chord polygon.
  double get area {
    final poly = points;
    var sum = 0.0;
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i] - origin;
      final b = poly[(i + 1) % poly.length] - origin;
      sum += a.cross(b).dot(normal);
    }
    var total = sum / 2;
    for (final s in segments) {
      if (s is! ProfileArc) continue;
      final theta = s.sweepAngle;
      // Circular segment: ½r²(θ − sin θ), signed by which way the arc bows
      // relative to the profile's own normal.
      final bulge = 0.5 * s.radius * s.radius * (theta - math.sin(theta));
      total += bulge * (s.normal.dot(normal) >= 0 ? 1 : -1);
    }
    return total.abs();
  }

  /// Area centroid of the chord polygon.
  Vector3 get centroid {
    final poly = points;
    var twiceArea = 0.0;
    var acc = Vector3.zero();
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i] - origin;
      final b = poly[(i + 1) % poly.length] - origin;
      final w = a.cross(b).dot(normal);
      twiceArea += w;
      acc += (a + b) * (w / 3);
    }
    if (twiceArea.abs() < 1e-18) return _average(poly);
    return origin + acc / twiceArea;
  }

  /// The same profile wound the other way, so its normal flips.
  SweepProfile get reversed => SweepProfile._(
        List.unmodifiable([
          for (final s in segments.reversed) s.reversed,
        ]),
        -normal,
        origin,
        xDir,
      );

  SweepProfile mapped(Vector3 Function(Vector3 p) f) => SweepProfile.of(
        [for (final s in segments) s.mapped(f)],
      );

  /// The same profile with a different 2D u-axis. See [xDir].
  SweepProfile withXDir(Vector3 dir) =>
      SweepProfile._(segments, normal, origin, _basis(dir, normal));

  SweepProfile translated(Vector3 d) => mapped((p) => p + d);

  /// Wound so that [normal] agrees with [want].
  SweepProfile orientedTo(Vector3 want) =>
      normal.dot(want) >= 0 ? this : reversed;

  /// The ring resampled to exactly [count] points, spaced evenly by arc
  /// length.
  ///
  /// Lofting needs every section to have the same vertex count so the sides
  /// can be quads, and demanding that of the caller would make lofting a
  /// square to a circle impossible to express.
  List<Vector3> resampled(int count, {int arcSegments = 64}) {
    final poly = ring(arcSegments: arcSegments);
    if (count < 3) {
      throw ArgumentError.value(count, 'count', 'must be >= 3');
    }

    final cum = <double>[0];
    for (var i = 0; i < poly.length; i++) {
      cum.add(cum.last + (poly[(i + 1) % poly.length] - poly[i]).length);
    }
    final total = cum.last;
    if (total < 1e-12) {
      throw MeshOpException('Profile has no perimeter to resample');
    }

    final out = <Vector3>[];
    var seg = 0;
    for (var k = 0; k < count; k++) {
      final target = total * k / count;
      while (seg < poly.length - 1 && cum[seg + 1] < target) {
        seg++;
      }
      final segLen = cum[seg + 1] - cum[seg];
      final t = segLen < 1e-15 ? 0.0 : (target - cum[seg]) / segLen;
      final a = poly[seg];
      final b = poly[(seg + 1) % poly.length];
      out.add(a + (b - a) * t);
    }
    return out;
  }

  @override
  String toString() => 'SweepProfile(${segments.length} segments, '
      'area ${area.toStringAsFixed(4)})';
}

List<Vector3> _dedupe(List<Vector3> points, {double tolerance = 1e-9}) {
  final out = <Vector3>[];
  for (final p in points) {
    if (out.isEmpty || (p - out.last).length > tolerance) out.add(p);
  }
  while (out.length > 1 && (out.first - out.last).length <= tolerance) {
    out.removeLast();
  }
  return out;
}

Vector3 _average(List<Vector3> points) {
  final acc = Vector3.zero();
  for (final p in points) {
    acc.add(p);
  }
  return points.isEmpty ? acc : acc / points.length.toDouble();
}

/// Newell's method: the area-weighted normal of a polygon.
///
/// Preferred over a cross product of two edges because it uses every vertex,
/// so it does not care which corner happens to be nearly straight, and it
/// gives the best-fit plane for points that are only approximately coplanar.
Vector3 _newellNormal(List<Vector3> poly) {
  final n = Vector3.zero();
  for (var i = 0; i < poly.length; i++) {
    final a = poly[i];
    final b = poly[(i + 1) % poly.length];
    n.x += (a.y - b.y) * (a.z + b.z);
    n.y += (a.z - b.z) * (a.x + b.x);
    n.z += (a.x - b.x) * (a.y + b.y);
  }
  return n.length2 < 1e-20 ? Vector3.zero() : n.normalized();
}

String _fmt(Vector3 v) => '${v.x.toStringAsFixed(2)}, '
    '${v.y.toStringAsFixed(2)}, ${v.z.toStringAsFixed(2)}';
