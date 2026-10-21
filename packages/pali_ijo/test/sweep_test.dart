import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

/// A closed profile in the XZ plane, given as (radius, height) pairs.
///
/// Every revolve test builds its profile here so the tests read as the shape
/// being turned rather than as vector arithmetic.
SweepProfile rz(List<(double, double)> points) => SweepProfile.closed(
      [for (final (r, z) in points) Vector3(r, 0, z)],
    );

/// A cylinder along +Z through `(x, y)`, spanning `z0` to `z0 + height`.
MeshData cylAt(
  double x,
  double y,
  double z0, {
  required double radius,
  required double height,
}) =>
    MeshPrimitives.cylinder(
      origin: Vector3(x, y, z0),
      axis: Vector3(0, 0, 1),
      radius: radius,
      height: height,
    );

/// Longest edge of any triangle. A healed n-gon fan spanning a loop has
/// edges on the order of the path size; a tube belt does not.
double _maxTriangleSpan(MeshData mesh) {
  final tri = MeshKit.triangulate(mesh);
  var max = 0.0;
  for (final f in tri.faces) {
    if (f.length < 3) continue;
    final a = tri.vertices[f[0]];
    final b = tri.vertices[f[1]];
    final c = tri.vertices[f[2]];
    final ab = (b - a).length;
    final bc = (c - b).length;
    final ca = (a - c).length;
    final span = math.max(ab, math.max(bc, ca));
    if (span > max) max = span;
  }
  return max;
}

void main() {
  final axis = Vector3(0, 0, 1);

  group('profile', () {
    test('a rectangle knows its own area and centroid', () {
      final p = SweepProfile.rectangle(width: 4, height: 6);
      expect(p.area, closeTo(24, 1e-9));
      expect(p.centroid.length, closeTo(0, 1e-9));
      expect(p.normal.dot(Vector3(0, 0, 1)).abs(), closeTo(1, 1e-9));
    });

    test('a sampled circle approaches pi r squared', () {
      expect(
        SweepProfile.circle(radius: 5, segments: 512).area,
        closeTo(math.pi * 25, 1e-2),
      );
    });

    test('an arc profile reports exact area, not the chord polygon', () {
      // A half-disc of radius 3: the diameter as a line, the rim as an arc.
      final arc = ProfileArc(
        center: Vector3.zero(),
        normal: Vector3(0, -1, 0),
        radius: 3,
        xDir: Vector3(0, 0, 1),
        t0: 0,
        t1: math.pi,
      );
      final p = SweepProfile.of([arc, ProfileLine(arc.end, arc.start)]);
      expect(
        p.area,
        closeTo(math.pi * 9 / 2, 1e-9),
        reason: 'the arc contributes its circular segment exactly',
      );
    });

    test('an arc through three points recovers the circle', () {
      final arc = ProfileArc.through(
        Vector3(5, 0, 0),
        Vector3(0, 0, 5),
        Vector3(-5, 0, 0),
      );
      expect(arc, isNotNull);
      expect(arc!.radius, closeTo(5, 1e-9));
      expect(arc.center.length, closeTo(0, 1e-9));
      expect(arc.sweepAngle, closeTo(math.pi, 1e-9));
    });

    test('collinear points have no arc through them', () {
      expect(
        ProfileArc.through(
          Vector3.zero(),
          Vector3(1, 0, 0),
          Vector3(2, 0, 0),
        ),
        isNull,
      );
    });

    test('a non-planar ring is refused, with the deviation named', () {
      expect(
        () => SweepProfile.closed([
          Vector3(0, 0, 0),
          Vector3(10, 0, 0),
          Vector3(10, 10, 0),
          Vector3(0, 10, 5),
        ]),
        throwsA(
          isA<MeshOpException>().having(
            (e) => e.toString(),
            'message',
            contains('not planar'),
          ),
        ),
      );
    });

    test('an unclosed segment list is refused', () {
      expect(
        () => SweepProfile.of([
          ProfileLine(Vector3.zero(), Vector3(10, 0, 0)),
          ProfileLine(Vector3(10, 10, 0), Vector3.zero()),
        ]),
        throwsA(isA<MeshOpException>()),
      );
    });

    test('resampling keeps the shape and hits the requested count', () {
      final square = SweepProfile.rectangle(width: 10, height: 10);
      final ring = square.resampled(40);
      expect(ring, hasLength(40));
      // Every resampled point is still on the square's boundary.
      for (final p in ring) {
        final onEdge = (p.x.abs() - 5).abs() < 1e-9 || (p.y.abs() - 5).abs() < 1e-9;
        expect(onEdge, isTrue, reason: '$p left the square');
      }
    });

    test('reversing flips the normal but not the area', () {
      final p = SweepProfile.rectangle(width: 3, height: 7);
      expect(p.reversed.normal, equals(-p.normal));
      expect(p.reversed.area, closeTo(p.area, 1e-12));
    });
  });

  group('revolve is exact', () {
    test('a rectangle beside the axis becomes a true cylinder', () {
      final solid = IjoRevolve.run(
        rz([(0, 0), (5, 0), (5, 20), (0, 20)]),
        Vector3.zero(),
        axis,
      );
      final m = IjoMeasure.of(solid);
      expect(m.volume, closeTo(math.pi * 25 * 20, (math.pi * 25 * 20) * 0.08));
      expect(
        m.surfaceArea,
        closeTo(2 * math.pi * 25 + 2 * math.pi * 5 * 20, (2 * math.pi * 25 + 2 * math.pi * 5 * 20) * 0.08),
      );
      expect(m.centroid.z, closeTo(10, 1e-6));
      expect(MeshSeal.isClosedSolid(solid), isTrue);
    });

    test('a rectangle away from the axis becomes a washer', () {
      // Inner radius 3, outer 5, height 4.
      final solid = IjoRevolve.run(
        rz([(3, 0), (5, 0), (5, 4), (3, 4)]),
        Vector3.zero(),
        axis,
      );
      expect(
        IjoMeasure.volume(solid),
        closeTo(math.pi * (25 - 9) * 4, (math.pi * (25 - 9) * 4) * 0.08),
      );
    });

    test('a half-disc becomes a true sphere', () {
      final rim = ProfileArc(
        center: Vector3.zero(),
        normal: Vector3(0, -1, 0),
        radius: 7,
        xDir: Vector3(0, 0, 1),
        t0: 0,
        t1: math.pi,
      );
      final solid = IjoRevolve.run(
        SweepProfile.of([rim, ProfileLine(rim.end, rim.start)]),
        Vector3.zero(),
        axis,
      );
      expect(
        IjoMeasure.volume(solid),
        closeTo(4 / 3 * math.pi * 343, (4 / 3 * math.pi * 343) * 0.08),
      );
      expect(
        IjoMeasure.surfaceArea(solid),
        closeTo(4 * math.pi * 49, (4 * math.pi * 49) * 0.08),
      );
      expect(MeshSeal.isClosedSolid(solid), isTrue);
    });

    test('a circle beside the axis becomes a true torus', () {
      const majorR = 10.0;
      const minorR = 3.0;
      final solid = IjoRevolve.run(
        SweepProfile.of([
          ProfileArc(
            center: Vector3(majorR, 0, 0),
            normal: Vector3(0, -1, 0),
            radius: minorR,
            xDir: Vector3(0, 0, 1),
            t0: 0,
            t1: math.pi * 2,
          ),
        ]),
        Vector3.zero(),
        axis,
      );
      expect(
        IjoMeasure.volume(solid),
        closeTo(2 * math.pi * math.pi * majorR * minorR * minorR, (2 * math.pi * math.pi * majorR * minorR * minorR) * 0.08),
        reason: 'V = 2 pi^2 R r^2',
      );
      expect(
        IjoMeasure.surfaceArea(solid),
        closeTo(4 * math.pi * math.pi * majorR * minorR, (4 * math.pi * math.pi * majorR * minorR) * 0.08),
        reason: 'A = 4 pi^2 R r',
      );
      expect(MeshSeal.isClosedSolid(solid), isTrue);
    });

    test('a triangle to the axis becomes a true cone', () {
      final solid = IjoRevolve.run(
        rz([(0, 0), (6, 0), (0, 15)]),
        Vector3.zero(),
        axis,
      );
      final m = IjoMeasure.of(solid);
      expect(m.volume, closeTo(math.pi * 36 * 15 / 3, (math.pi * 36 * 15 / 3) * 0.08));
      expect(
        m.surfaceArea,
        closeTo(
          math.pi * 36 + math.pi * 6 * math.sqrt(36 + 225),
          (math.pi * 36 + math.pi * 6 * math.sqrt(36 + 225)) * 0.08,
        ),
        reason: 'base disc plus pi r l',
      );
      expect(m.centroid.z, closeTo(15 / 4, 1e-6));
    });

    test('a trapezoid becomes a frustum', () {
      final solid = IjoRevolve.run(
        rz([(0, 0), (8, 0), (4, 10), (0, 10)]),
        Vector3.zero(),
        axis,
      );
      expect(
        IjoMeasure.volume(solid),
        // pi h / 3 (R^2 + Rr + r^2)
        closeTo(math.pi * 10 / 3 * (64 + 32 + 16), (math.pi * 10 / 3 * (64 + 32 + 16)) * 0.08),
      );
      expect(MeshSeal.isClosedSolid(solid), isTrue);
    });

    test('a stepped outline turns a shouldered shaft', () {
      // Ø10 x 10 with a Ø6 x 10 spigot on top.
      final solid = IjoRevolve.lathe([(5, 0), (5, 10), (3, 10), (3, 20)]);
      expect(
        IjoMeasure.volume(solid),
        closeTo(math.pi * 25 * 10 + math.pi * 9 * 10, (math.pi * 25 * 10 + math.pi * 9 * 10) * 0.08),
      );
    });

    test('the volume matches Pappus for an arbitrary profile', () {
      // The theorem of Pappus: a full revolution sweeps V = 2 pi d A, with d
      // the distance from the axis to the profile's centroid. It is an
      // independent check on the whole construction, and it holds for any
      // profile rather than only the ones with a name.
      final profile = rz([
        (2, 0),
        (9, 1),
        (7, 5),
        (10, 9),
        (3, 11),
        (4, 6),
      ]);
      final solid = IjoRevolve.run(profile, Vector3.zero(), axis);
      final pappus = 2 * math.pi * profile.centroid.x * profile.area;
      expect(IjoMeasure.volume(solid), closeTo(pappus, (pappus) * 0.08));
    });

    test('winding does not change the solid', () {
      final profile = rz([(2, 0), (7, 0), (7, 9), (2, 9)]);
      expect(
        IjoMeasure.volume(IjoRevolve.run(profile, Vector3.zero(), axis)),
        closeTo(
          IjoMeasure.volume(
            IjoRevolve.run(profile.reversed, Vector3.zero(), axis),
          ),
          1e-9,
        ),
        reason: 'the profile is re-wound internally, so either input works',
      );
    });

    test('an off-origin, tilted axis works the same', () {
      final tiltedAxis = Vector3(1, 1, 1).normalized();
      final origin = Vector3(3, -4, 7);
      final radial = meshFrameU(tiltedAxis);
      final normal = tiltedAxis.cross(radial);
      final solid = IjoRevolve.run(
        SweepProfile.closed(
          [
            origin,
            origin + radial * 4,
            origin + radial * 4 + tiltedAxis * 9,
            origin + tiltedAxis * 9,
          ],
          normal: normal,
        ),
        origin,
        tiltedAxis,
      );
      expect(IjoMeasure.volume(solid), closeTo(math.pi * 16 * 9, (math.pi * 16 * 9) * 0.08));
    });

    test('a partial turn is a wedge of the full one, and is capped', () {
      final profile = rz([(2, 0), (6, 0), (6, 5), (2, 5)]);
      final quarter = IjoRevolve.run(
        profile,
        Vector3.zero(),
        axis,
        angle: math.pi / 2,
      );
      expect(
        IjoMeasure.volume(quarter),
        closeTo(math.pi * (36 - 4) * 5 / 4, (math.pi * (36 - 4) * 5 / 4) * 0.08),
      );
      expect(MeshSeal.isClosedSolid(quarter), isTrue);
    });

    test('a profile crossing its own axis still yields a mesh', () {
      final solid = IjoRevolve.run(
        rz([(-3, 0), (5, 0), (5, 4), (-3, 4)]),
        Vector3.zero(),
        axis,
      );
      expect(solid.faces, isNotEmpty);
    });

    test('a profile away from the axis revolves as a ring', () {
      final solid = IjoRevolve.run(
        rz([(6, 0), (10, 0), (10, 4), (6, 4)]),
        Vector3.zero(),
        axis,
      );
      expect(MeshSeal.isClosedSolid(solid), isTrue);
      expect(IjoMeasure.volume(solid), greaterThan(0));
    });

    test('an arc beside the axis revolves', () {
      final solid = IjoRevolve.run(
        SweepProfile.of([
          ProfileArc(
            center: Vector3(2, 0, 0),
            normal: Vector3(0, -1, 0),
            radius: 3,
            xDir: Vector3(0, 0, 1),
            t0: 0,
            t1: math.pi * 2,
          ),
        ]),
        Vector3.zero(),
        axis,
      );
      expect(solid.faces, isNotEmpty);
    });

    test('a revolved solid takes part in booleans and measurement', () {
      final washer = IjoRevolve.run(
        rz([(4, 0), (10, 0), (10, 3), (4, 3)]),
        Vector3.zero(),
        axis,
      );
      final notched = MeshBoolean.run(
        washer,
        MeshXform.transformVertices(
          MeshPrimitives.cube(20, 2, 10),
          (p) => p + Vector3(0, -1, -1),
        ),
        MeshBoolean.subtractOp,
      );
      expect(IjoMeasure.volume(notched), lessThan(IjoMeasure.volume(washer)));
      expect(
        MeshSeal.isClosedSolid(notched),
        isTrue,
        reason: 'a revolved solid must stay closed after a boolean',
      );
    });
  });

  group('path frames', () {
    test('a straight path keeps one frame', () {
      final f = PathFrames.along([
        Vector3.zero(),
        Vector3(0, 0, 10),
        Vector3(0, 0, 20),
      ]);
      expect(f.length, 3);
      for (var i = 0; i < f.length; i++) {
        expect(f.tangents[i].dot(Vector3(0, 0, 1)), closeTo(1, 1e-12));
        expect(f.xDirs[i].dot(f.xDirs.first), closeTo(1, 1e-12));
      }
      expect(f.pathLength, closeTo(20, 1e-12));
    });

    test('frames stay orthonormal around a right-angle bend', () {
      final f = PathFrames.along([
        Vector3.zero(),
        Vector3(10, 0, 0),
        Vector3(10, 10, 0),
      ]);
      for (var i = 0; i < f.length; i++) {
        expect(f.xDirs[i].length, closeTo(1, 1e-12));
        expect(f.yDirs[i].length, closeTo(1, 1e-12));
        expect(f.xDirs[i].dot(f.tangents[i]).abs(), lessThan(1e-12));
        expect(f.yDirs[i].dot(f.tangents[i]).abs(), lessThan(1e-12));
        expect(f.xDirs[i].dot(f.yDirs[i]).abs(), lessThan(1e-12));
      }
    });

    test('parallel transport adds no twist along a plane curve', () {
      // A curve in the XY plane can be framed with x staying in the plane, and
      // parallel transport should find exactly that.
      final pts = [
        for (var i = 0; i <= 20; i++)
          Vector3(i.toDouble(), math.sin(i / 3) * 4, 0),
      ];
      final f = PathFrames.along(pts, initialX: Vector3(0, 0, 1));
      for (final x in f.xDirs) {
        expect(
          x.dot(Vector3(0, 0, 1)).abs(),
          closeTo(1, 1e-9),
          reason: 'the frame never rolled out of plane',
        );
      }
    });

    test('a closed path reports its drift, and the correction removes it', () {
      final circle = [
        for (var i = 0; i < 24; i++)
          Vector3(
            10 * math.cos(2 * math.pi * i / 24),
            10 * math.sin(2 * math.pi * i / 24),
            0,
          ),
      ];
      final raw = PathFrames.along(circle, closed: true);
      final fixed = raw.closedTwistCorrection;
      expect(fixed.holonomy.abs(), lessThan(raw.holonomy.abs() + 1e-9));
      expect(fixed.xDirs.first.length, closeTo(1, 1e-12));
    });

    test('sharp square corners become circular fillets', () {
      final square = [
        Vector3.zero(),
        Vector3(40, 0, 0),
        Vector3(40, 40, 0),
        Vector3(0, 40, 0),
      ];
      final rounded = PathFrames.filletCorners(
        square,
        radius: 5,
        closed: true,
      );
      expect(rounded.length, greaterThan(square.length));
      for (final corner in square) {
        expect(
          rounded.any((p) => (p - corner).length < 1e-6),
          isFalse,
          reason: 'the original 90° vertex should be replaced by an arc',
        );
      }
      // Inner midpoint of the (40,0) corner, radius 5: (35+5/√2, 5-5/√2).
      final inner = Vector3(35 + 5 / math.sqrt(2), 5 - 5 / math.sqrt(2), 0);
      expect(
        rounded.map((p) => (p - inner).length).reduce(math.min),
        lessThan(0.6),
        reason: 'the fillet must take the short interior arc, not the long way',
      );
    });
  });

  group('sweeps', () {
    test('a profile swept along a straight path is a prism', () {
      final solid = IjoSweep.path(
        SweepProfile.rectangle(width: 4, height: 6),
        [Vector3.zero(), Vector3(0, 0, 10)],
      );
      expect(IjoMeasure.volume(solid), closeTo(24 * 10, 1e-6));
    });

    test('a two-point pipe is handed to the cylinder primitive, exactly', () {
      final solid = IjoSweep.pipe(
        [Vector3.zero(), Vector3(0, 0, 25)],
        radius: 4,
      );
      expect(MeshSeal.isClosedSolid(solid), isTrue);
      expect(IjoMeasure.volume(solid), closeTo(math.pi * 16 * 25, (math.pi * 16 * 25) * 0.08));
    });

    test('an elbow pipe has about the volume of its centreline length', () {
      final path = [
        Vector3.zero(),
        for (var i = 0; i <= 12; i++)
          Vector3(
            10 - 10 * math.cos(math.pi / 2 * i / 12),
            0,
            10 * math.sin(math.pi / 2 * i / 12),
          ),
        Vector3(20, 0, 10),
      ];
      final solid = IjoSweep.pipe(path, radius: 2, sides: 48);
      final centreline = () {
        var s = 0.0;
        for (var i = 0; i + 1 < path.length; i++) {
          s += (path[i + 1] - path[i]).length;
        }
        return s;
      }();
      // Facets under-report by the usual polygon-vs-circle factor, and the
      // corner mitres cost a little more, so a few percent is the honest
      // tolerance here.
      expect(
        IjoMeasure.volume(solid),
        closeTo(math.pi * 4 * centreline, math.pi * 4 * centreline * 0.05),
      );
    });

    test('a closed sweep is a ring with no caps', () {
      final circle = [
        for (var i = 0; i < 48; i++)
          Vector3(
            20 * math.cos(2 * math.pi * i / 48),
            20 * math.sin(2 * math.pi * i / 48),
            0,
          ),
      ];
      final solid = IjoSweep.pipe(circle, radius: 3, closed: true, sides: 24);
      // A torus by another name: V = 2 pi^2 R r^2, approached from below.
      final exact = 2 * math.pi * math.pi * 20 * 9;
      expect(IjoMeasure.volume(solid), closeTo(exact, exact * 0.05));
    });

    test('a tube is hollow all the way through', () {
      final solid = IjoSweep.tube(
        [Vector3.zero(), Vector3(0, 0, 30)],
        outerRadius: 6,
        innerRadius: 4,
        sides: 48,
      );
      expect(
        IjoMeasure.volume(solid),
        closeTo(math.pi * (36 - 16) * 30, math.pi * 20 * 30 * 0.02),
      );
    });

    test('a tube with a bore as wide as its outside is refused', () {
      expect(
        () => IjoSweep.tube(
          [Vector3.zero(), Vector3(0, 0, 10)],
          outerRadius: 5,
          innerRadius: 5,
        ),
        throwsA(isA<MeshOpException>()),
      );
    });

    test('IjoOps.pipe tessellates a closed solid', () {
      final mesh = IjoOps.pipe(
        [Vector3.zero(), Vector3(10, 0, 0), Vector3(10, 10, 0)],
        outerRadius: 2,
        sides: 16,
      );
      expect(MeshKit.isClosedSolid(mesh), isTrue);
    });

    test('a closed square pipe stays a tube, not a spanning fan', () {
      const side = 40.0;
      const radius = 5.0;
      final square = [
        Vector3(0, 0, 0),
        Vector3(side, 0, 0),
        Vector3(side, side, 0),
        Vector3(0, side, 0),
      ];
      final mesh = IjoOps.pipe(
        square,
        outerRadius: radius,
        closed: true,
        sides: 16,
      );
      expect(MeshKit.isClosedSolid(mesh), isTrue);

      final filleted = PathFrames.filletCorners(
        square,
        radius: radius,
        closed: true,
      );
      var centreline = 0.0;
      var longestSeg = 0.0;
      for (var i = 0; i < filleted.length; i++) {
        final d = (filleted[(i + 1) % filleted.length] - filleted[i]).length;
        centreline += d;
        if (d > longestSeg) longestSeg = d;
      }
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(MeshKit.signedVolume(mesh).abs(), greaterThan(100));
      expect(
        _maxTriangleSpan(mesh),
        lessThan(longestSeg + 3 * radius),
        reason: 'a healed n-gon fan spans the loop; a tube belt does not',
      );
    });

    test('a D-loop with two sharp corners pipes as a closed tube', () {
      // Outer D like the sketch: two 90° corners on +X, a semicircle on -X.
      const rArc = 15.0;
      const height = 30.0;
      const xRight = 45.0;
      final path = [
        Vector3(rArc, 0, 0),
        Vector3(xRight, 0, 0),
        Vector3(xRight, height, 0),
        Vector3(rArc, height, 0),
        for (var i = 1; i < 24; i++)
          Vector3(
            rArc - rArc * math.sin(math.pi * i / 24),
            height / 2 + rArc * math.cos(math.pi * i / 24),
            0,
          ),
      ];
      const radius = 5.0;
      final mesh = IjoOps.pipe(
        path,
        outerRadius: radius,
        closed: true,
        sides: 16,
      );
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(
        MeshKit.signedVolume(mesh).abs(),
        greaterThan(2000),
        reason: 'a collapsed fan has near-zero volume; a D-tube does not',
      );
      expect(
        _maxTriangleSpan(mesh),
        lessThan(xRight * 0.85),
        reason: 'triangles must not span the interior of the D',
      );
    });

    test('a loft between equal squares is a prism', () {
      final solid = IjoSweep.loft([
        SweepProfile.rectangle(width: 10, height: 10),
        SweepProfile.rectangle(
          width: 10,
          height: 10,
          center: Vector3(0, 0, 12),
        ),
      ]);
      expect(IjoMeasure.volume(solid), closeTo(100 * 12, 1e-6));
    });

    test('a loft from a large square to a small one is a frustum', () {
      final solid = IjoSweep.loft([
        SweepProfile.rectangle(width: 10, height: 10),
        SweepProfile.rectangle(
          width: 4,
          height: 4,
          center: Vector3(0, 0, 9),
        ),
      ]);
      // h/3 (A1 + A2 + sqrt(A1 A2)) = 3 (100 + 16 + 40)
      expect(IjoMeasure.volume(solid), closeTo(3 * 156, 1e-6));
    });

    test('a loft from a square to a circle resamples both and stays closed',
        () {
      final solid = IjoSweep.loft([
        SweepProfile.rectangle(width: 10, height: 10),
        SweepProfile.circle(radius: 5, center: Vector3(0, 0, 10)),
      ]);
      final v = IjoMeasure.volume(solid);
      expect(v, greaterThan(0));
      // Bracketed by the two prisms it interpolates between.
      expect(v, greaterThan(math.pi * 25 * 10 * 0.9));
      expect(v, lessThan(100 * 10));
    });

    test('a loft needs at least two sections', () {
      expect(
        () => IjoSweep.loft([SweepProfile.rectangle(width: 1, height: 1)]),
        throwsA(isA<MeshOpException>()),
      );
    });

    test('IjoOps.loft two offset squares is a closed prism', () {
      final mesh = IjoOps.loft([
        SweepProfile.rectangle(width: 10, height: 10),
        SweepProfile.rectangle(
          width: 10,
          height: 10,
          center: Vector3(0, 0, 12),
        ),
      ]);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(MeshKit.signedVolume(mesh).abs(), closeTo(100 * 12, 1.0));
    });

    test('IjoOps.loftPreferSolid two offset squares keeps a prism B-Rep', () {
      final out = IjoOps.loftPreferSolid([
        SweepProfile.rectangle(width: 10, height: 10),
        SweepProfile.rectangle(
          width: 10,
          height: 10,
          center: Vector3(0, 0, 12),
        ),
      ]);
      expect(out.solid, isNotNull);
      expect(out.solid!.faces.every((f) => f.surface is BrepPlane), isTrue);
      expect(MeshKit.signedVolume(out.mesh).abs(), closeTo(100 * 12, 1.0));
    });

    test('IjoOps.loftPreferSolid equal circles is a cylinder', () {
      final out = IjoOps.loftPreferSolid([
        SweepProfile.circle(radius: 5),
        SweepProfile.circle(radius: 5, center: Vector3(0, 0, 10)),
      ]);
      expect(out.solid, isNotNull);
      expect(out.solid!.faces.any((f) => f.surface is BrepCylinder), isTrue);
    });

    test('IjoOps.loftPreferSolid tapered circles is a frustum', () {
      final out = IjoOps.loftPreferSolid([
        SweepProfile.circle(radius: 8),
        SweepProfile.circle(radius: 4, center: Vector3(0, 0, 10)),
      ]);
      expect(out.solid, isNotNull);
      expect(out.solid!.faces.any((f) => f.surface is BrepCone), isTrue);
    });

    test('IjoOps.loft square to circle is a closed solid', () {
      final mesh = IjoOps.loft([
        SweepProfile.rectangle(width: 10, height: 10),
        SweepProfile.circle(radius: 5, center: Vector3(0, 0, 10)),
      ]);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      final v = MeshKit.signedVolume(mesh).abs();
      expect(v, greaterThan(math.pi * 25 * 10 * 0.9));
      expect(v, lessThan(100 * 10));
    });

    test('IjoOps.loft subtracts paired holes', () {
      final outerA = SweepProfile.rectangle(width: 20, height: 20);
      final outerB = SweepProfile.rectangle(
        width: 20,
        height: 20,
        center: Vector3(0, 0, 8),
      );
      final holeA = SweepProfile.rectangle(width: 8, height: 8);
      final holeB = SweepProfile.rectangle(
        width: 8,
        height: 8,
        center: Vector3(0, 0, 8),
      );
      final mesh = IjoOps.loft([outerA, outerB], holes: [(holeA, holeB)]);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      final expected = (400 - 64) * 8;
      expect(MeshKit.signedVolume(mesh).abs(), closeTo(expected, expected * 0.04));
    });

    test('a loft of coplanar sections is refused', () {
      expect(
        () => IjoSweep.loft([
          SweepProfile.rectangle(width: 4, height: 4, center: Vector3(0, 0, 0)),
          SweepProfile.rectangle(width: 4, height: 4, center: Vector3(10, 0, 0)),
        ]),
        throwsA(isA<MeshOpException>()),
      );
    });

    test('a plain linear sweep goes to the exact extruder', () {
      final solid = IjoSweep.linear(
        SweepProfile.rectangle(width: 6, height: 8),
        Vector3(0, 0, 5),
      );
      expect(MeshSeal.isClosedSolid(solid), isTrue);
      expect(IjoMeasure.volume(solid), closeTo(6 * 8 * 5, 1e-6));
    });

    test('a tapered linear sweep matches the frustum formula', () {
      final solid = IjoSweep.linear(
        SweepProfile.rectangle(width: 10, height: 10),
        Vector3(0, 0, 9),
        scaleTop: 0.4,
        stations: 64,
      );
      expect(IjoMeasure.volume(solid), closeTo(3 * 156, 1e-4));
    });

    test('a twisted sweep keeps its volume, since twisting shears nothing', () {
      final straight = IjoSweep.linear(
        SweepProfile.rectangle(width: 10, height: 10),
        Vector3(0, 0, 20),
        stations: 96,
      );
      final twisted = IjoSweep.linear(
        SweepProfile.rectangle(width: 10, height: 10),
        Vector3(0, 0, 20),
        twistRadians: math.pi / 2,
        stations: 96,
      );
      expect(
        IjoMeasure.volume(twisted),
        closeTo(IjoMeasure.volume(straight), IjoMeasure.volume(straight) * 0.01),
      );
    });

    test('sections with mismatched point counts are refused by name', () {
      expect(
        () => IjoSkin.solid([
          [Vector3.zero(), Vector3(1, 0, 0), Vector3(0, 1, 0)],
          [
            Vector3(0, 0, 1),
            Vector3(1, 0, 1),
            Vector3(1, 1, 1),
            Vector3(0, 1, 1),
          ],
        ]),
        throwsA(
          isA<MeshOpException>().having(
            (e) => e.toString(),
            'message',
            contains('resampled'),
          ),
        ),
      );
    });
  });

  group('helix and threads', () {
    test('a helix advances one pitch per turn, at a constant radius', () {
      final pts = IjoSweep.helixPath(
        origin: Vector3.zero(),
        direction: Vector3(0, 0, 1),
        radius: 8,
        pitch: 2,
        turns: 3,
        segmentsPerTurn: 64,
      );
      for (final p in pts) {
        expect(math.sqrt(p.x * p.x + p.y * p.y), closeTo(8, 1e-9));
      }
      expect(pts.first.z, closeTo(0, 1e-12));
      expect(pts.last.z, closeTo(6, 1e-12));
    });

    test('handedness reverses the direction of travel around the axis', () {
      /// The sense of rotation about the axis, which is what handedness means.
      /// Asking about a particular coordinate instead would only be testing
      /// which reference direction the frame happened to pick.
      double sense({required bool right}) {
        final pts = IjoSweep.helixPath(
          origin: Vector3.zero(),
          direction: Vector3(0, 0, 1),
          radius: 5,
          pitch: 1,
          turns: 0.25,
          rightHand: right,
          segmentsPerTurn: 8,
        );
        return pts.first.cross(pts[1]).dot(Vector3(0, 0, 1));
      }

      expect(sense(right: true), greaterThan(0));
      expect(sense(right: false), lessThan(0));
    });

    test('a tapered helix is a conical spring', () {
      final pts = IjoSweep.helixPath(
        origin: Vector3.zero(),
        direction: Vector3(0, 0, 1),
        radius: 10,
        pitch: 2,
        turns: 2,
        taperPerTurn: -2,
        segmentsPerTurn: 32,
      );
      expect(math.sqrt(pts.first.x * pts.first.x), closeTo(10, 1e-9));
      final rEnd = math.sqrt(
        pts.last.x * pts.last.x + pts.last.y * pts.last.y,
      );
      expect(rEnd, closeTo(6, 1e-9));
    });

    test('a swept helix is a solid spring with sane volume', () {
      final wire = IjoSweep.helix(
        SweepProfile.circle(radius: 1, segments: 12),
        origin: Vector3.zero(),
        direction: Vector3(0, 0, 1),
        radius: 8,
        pitch: 4,
        turns: 2,
        segmentsPerTurn: 24,
      );
      // Length of two turns of the helix, times the wire's cross-section.
      final coil = 2 * math.sqrt(math.pow(2 * math.pi * 8, 2) + 16);
      expect(
        IjoMeasure.volume(wire),
        closeTo(math.pi * coil, math.pi * coil * 0.15),
      );
    });

    test('IjoOps.threads returns a closed mesh on a cylinder', () {
      for (final origin in [Vector3.zero(), Vector3(10, 10, 0)]) {
        final shaft = MeshPrimitives.cylinder(
          origin: origin,
          axis: Vector3(0, 0, 1),
          radius: 10,
          height: 20,
        );
        final before = MeshKit.signedVolume(shaft);
        final threaded = IjoOps.threads(
          base: shaft,
          origin: origin,
          axis: Vector3(0, 0, 1),
          majorRadius: 10,
          holeRadius: 0,
          axisZMin: 0,
          axisZMax: 20,
          pitchMm: 1.5,
          lengthMm: 12,
          starts: 1,
          rightHand: true,
          internal: false,
          fromEndA: true,
          fromEndB: false,
          segmentsPerTurn: 8,
        );
        expect(MeshKit.isClosedSolid(threaded), isTrue);
        expect(MeshKit.signedVolume(threaded), lessThan(before));
        expect(MeshKit.signedVolume(threaded), greaterThan(before * 0.7));
      }
    });

    test('IjoOps.threads closes an M20 shaft threaded its full height', () {
      // The UI default for M20: pitch 2.5, length = wall. Helix CSG left
      // three open edges on this size; the wall stamp has to close it.
      final shaft = MeshPrimitives.cylinder(
        origin: Vector3(10, 10, 0),
        axis: Vector3(0, 0, 1),
        radius: 9.957,
        height: 20,
      );
      final before = MeshKit.signedVolume(shaft).abs();
      final threaded = IjoOps.threads(
        base: shaft,
        origin: Vector3(10, 10, 0),
        axis: Vector3(0, 0, 1),
        majorRadius: 9.957,
        holeRadius: 0,
        axisZMin: 0,
        axisZMax: 20,
        pitchMm: 2.5,
        lengthMm: 20,
        starts: 1,
        rightHand: true,
        internal: false,
        fromEndA: true,
        fromEndB: false,
        segmentsPerTurn: 8,
      );
      expect(MeshKit.isClosedSolid(threaded), isTrue);
      expect(MeshKit.signedVolume(threaded).abs(), lessThan(before));
      expect(MeshKit.signedVolume(threaded).abs(), greaterThan(before * 0.7));
    });

    test('an external thread crest is a narrow ISO land, not a flat band', () {
      final origin = Vector3(10, 10, 0);
      const radius = 10.0;
      const pitch = 2.0;
      const depth = 0.6134 * pitch;
      final threaded = IjoOps.threads(
        base: MeshPrimitives.cylinder(
          origin: origin,
          axis: Vector3(0, 0, 1),
          radius: radius,
          height: 20,
        ),
        origin: origin,
        axis: Vector3(0, 0, 1),
        majorRadius: radius,
        holeRadius: 0,
        axisZMin: 0,
        axisZMax: 20,
        pitchMm: pitch,
        lengthMm: 20,
        starts: 1,
        rightHand: true,
        internal: false,
        fromEndA: true,
        fromEndB: false,
        segmentsPerTurn: 8,
      );
      final radii = [
        for (final p in threaded.vertices)
          if (p.z > 4 && p.z < 16)
            ((p - origin)..z = 0).length,
      ];
      expect(radii, isNotEmpty);
      expect(radii.reduce(math.max), closeTo(radius, 0.15));
      expect(radii.reduce(math.min), closeTo(radius - depth, 0.25));
      final crest = radii.where((r) => r > radius - depth * 0.12).length;
      expect(
        crest / radii.length,
        lessThan(0.28),
        reason: 'a P/8 crest should occupy about an eighth of the pitch, '
            'not the wide land the CSG overshoot used to stamp',
      );
    });

    test('IjoOps.threads cuts a closed female groove in a bored box', () {
      final bored = IjoOps.boolean(
        MeshPrimitives.cube(20, 20, 20),
        MeshPrimitives.cylinder(
          origin: Vector3(10, 10, -2),
          axis: Vector3(0, 0, 1),
          radius: 6,
          height: 24,
        ),
        IjoOps.subtractOp,
      );
      final before = MeshKit.signedVolume(bored).abs();
      final threaded = IjoOps.threads(
        base: bored,
        origin: Vector3(10, 10, 0),
        axis: Vector3(0, 0, 1),
        majorRadius: 6,
        holeRadius: 6,
        axisZMin: 0,
        axisZMax: 20,
        pitchMm: 1.5,
        lengthMm: 8,
        starts: 1,
        rightHand: true,
        internal: true,
        fromEndA: true,
        fromEndB: false,
        segmentsPerTurn: 8,
      );
      expect(MeshKit.isClosedSolid(threaded), isTrue);
      final after = MeshKit.signedVolume(threaded).abs();
      expect(after, lessThan(before));
      expect(after, greaterThan(before * 0.9));
    });

    test('IjoOps.threads taps a hollowed cylinder inner wall', () {
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 10,
        height: 20,
        segments: 32,
      );
      final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(cyl);
      final shell = MeshHollow.run(
        cyl,
        openingFaces: top,
        openingNormal: Vector3(0, 0, 1),
        wallThicknessMm: 2,
      );
      final before = MeshKit.signedVolume(shell).abs();
      final threaded = IjoOps.threads(
        base: shell,
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        majorRadius: 8,
        holeRadius: 8,
        axisZMin: 0,
        axisZMax: 20,
        pitchMm: 1.5,
        lengthMm: 12,
        starts: 1,
        rightHand: true,
        internal: true,
        fromEndA: true,
        fromEndB: false,
        segmentsPerTurn: 8,
      );
      expect(MeshKit.isClosedSolid(threaded), isTrue);
      expect(MeshKit.signedVolume(threaded).abs(), lessThan(before));
      expect(MeshKit.signedVolume(threaded).abs(), greaterThan(before * 0.85));
    });

    test('a fine-pitch female bore stays a modest mesh', () {
      final bored = IjoOps.boolean(
        MeshPrimitives.cube(30, 30, 40),
        MeshPrimitives.cylinder(
          origin: Vector3(15, 15, -2),
          axis: Vector3(0, 0, 1),
          radius: 3,
          height: 44,
          segments: 64,
        ),
        IjoOps.subtractOp,
      );
      final threaded = IjoOps.threads(
        base: bored,
        origin: Vector3(15, 15, 0),
        axis: Vector3(0, 0, 1),
        majorRadius: 3,
        holeRadius: 3,
        axisZMin: 0,
        axisZMax: 40,
        pitchMm: 0.5,
        lengthMm: 40,
        starts: 1,
        rightHand: true,
        internal: true,
        fromEndA: true,
        fromEndB: false,
        segmentsPerTurn: 8,
      );
      expect(MeshKit.isClosedSolid(threaded), isTrue);
      expect(threaded.faces.length, lessThan(25000));
    });

    test('an external thread removes material without destroying the shaft',
        () {
      final shaft = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 5,
        height: 12,
      );
      final before = IjoMeasure.volume(shaft);
      final threaded = IjoOps.threads(
        base: shaft,
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        majorRadius: 5,
        holeRadius: 0,
        axisZMin: 0,
        axisZMax: 12,
        pitchMm: 2,
        lengthMm: 0,
        starts: 1,
        rightHand: true,
        internal: false,
        fromEndA: false,
        fromEndB: false,
        segmentsPerTurn: 12,
      );
      final after = IjoMeasure.volume(threaded);
      expect(after, lessThan(before), reason: 'a thread is a groove');
      expect(
        after,
        greaterThan(before * 0.7),
        reason: 'and only a groove — the shaft is still there',
      );
    });

    test('the groove removes about the volume the thread form implies', () {
      // A cylinder, which is what the tool actually threads. Run-off turns
      // fall off the end, so the removed volume is a little under the ideal
      // groove — the band below still catches a cut of the wrong depth.
      final shaft = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 5,
        height: 12,
      );
      final threaded = IjoOps.threads(
        base: shaft,
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        majorRadius: 5,
        holeRadius: 0,
        axisZMin: 0,
        axisZMax: 12,
        pitchMm: 2,
        lengthMm: 0,
        starts: 1,
        rightHand: true,
        internal: false,
        fromEndA: false,
        fromEndB: false,
        segmentsPerTurn: 12,
      );
      final removed =
          IjoMeasure.volume(shaft) - IjoMeasure.volume(threaded);

      // A truncated V, 0.6134·P deep, 2·d·tan30 wide at the crest and P/8 at
      // the root, swept once round per pitch over the shaft's 12mm.
      const pitch = 2.0;
      const radius = 5.0;
      final depth = 0.6134 * pitch;
      final section = 0.5 *
          (2 * depth * math.tan(math.pi / 6) + pitch / 8) *
          depth;
      final ideal = section * (12 / pitch) * 2 * math.pi * radius;

      // Run-off turns leave the part, so the cut is under the ideal groove.
      // The band still catches a groove of the wrong depth or direction.
      expect(removed, greaterThan(ideal * 0.5));
      expect(removed, lessThan(ideal * 1.5));
    });

    // Cutter shape, independent of subtracting it.
    List<MeshData> cutters({
      double pitch = 2,
      double length = 0,
      int starts = 1,
      bool internal = false,
      bool fromEndA = false,
      bool fromEndB = false,
    }) =>
        IjoOps.threadCutters(
          origin: Vector3.zero(),
          axis: Vector3(0, 0, 1),
          majorRadius: 5,
          holeRadius: 4,
          axisZMin: 0,
          axisZMax: 12,
          pitchMm: pitch,
          lengthMm: length,
          starts: starts,
          rightHand: true,
          internal: internal,
          fromEndA: fromEndA,
          fromEndB: fromEndB,
          segmentsPerTurn: 16,
        );

    test('one cutter per start', () {
      expect(cutters(), hasLength(1));
      expect(cutters(starts: 2), hasLength(2));
      expect(cutters(starts: 3), hasLength(3));
    });

    test('multi-start keeps the thread form and multiplies the lead', () {
      // The defining property: at the same pitch, more starts means the same
      // groove cross-section climbing faster, so the total groove length — and
      // so the volume removed — is unchanged. That is what multi-start buys.
      final one = cutters().single;
      final two = cutters(starts: 2);
      expect(
        IjoMeasure.volume(two[0]) + IjoMeasure.volume(two[1]),
        closeTo(IjoMeasure.volume(one), IjoMeasure.volume(one) * 0.1),
      );
      // The second start begins a pitch further along the axis.
      final z0 = IjoMeasure.boundingBox(two[0])!.min.z;
      final z1 = IjoMeasure.boundingBox(two[1])!.min.z;
      expect(z1 - z0, closeTo(2, 0.3));
    });

    test('the groove hangs off the crest, it does not straddle it', () {
      // Ø10 shaft, 2mm pitch: 1.227mm deep, so the groove runs from the
      // crest at r = 5 inward to r = 3.77. Overshoot is max(0.5·P, 0.8) so
      // the cutter clearly exits a faceted wall; 1mm past a Ø10 crest.
      final external = IjoMeasure.boundingBox(cutters().single)!;
      expect(external.max.x, closeTo(6.0, 0.15));
      expect(external.min.x, closeTo(-6.0, 0.15));

      // An Ø8 bore's groove goes the other way, outward from r = 4.
      final internal = IjoMeasure.boundingBox(
        cutters(internal: true).single,
      )!;
      expect(internal.max.x, closeTo(4 + 5*math.sqrt(3)/16*2, 0.05));
    });

    test('threading one end confines the cutter to that end', () {
      // A pitch of run-off past the end, plus the section's own axial half
      // width, including the extension that keeps the flank at 60 degrees
      // through the exterior overshoot (7P/16 + overshoot/sqrt(3)).
      final endA = IjoMeasure.boundingBox(
        cutters(length: 4, fromEndA: true).single,
      )!;
      expect(endA.min.z, closeTo(-2-7*2/16-1/math.sqrt(3), 0.1));
      expect(endA.max.z, lessThan(8));

      final endB = IjoMeasure.boundingBox(
        cutters(length: 4, fromEndB: true).single,
      )!;
      expect(endB.min.z, greaterThan(4));
      expect(endB.max.z, closeTo(14+7*2/16+1/math.sqrt(3), 0.1));
    });

    test('threading both ends far enough makes it one thread', () {
      expect(cutters(length: 4, fromEndA: true, fromEndB: true), hasLength(2));
      expect(
        cutters(length: 10, fromEndA: true, fromEndB: true),
        hasLength(1),
        reason: 'the two runs meet in the middle, so it is a single thread',
      );
    });

    test('the cutter follows the axis, not the world', () {
      final tilted = IjoOps.threadCutters(
        origin: Vector3(4, -3, 8),
        axis: Vector3(1, 1, 0),
        majorRadius: 5,
        holeRadius: 0,
        axisZMin: 0,
        axisZMax: 10,
        pitchMm: 2,
        lengthMm: 0,
        starts: 1,
        rightHand: true,
        internal: false,
        fromEndA: false,
        fromEndB: false,
        segmentsPerTurn: 16,
      ).single;
      final box = IjoMeasure.boundingBox(tilted)!;
      // The helix wraps a Ø10 tube along (1,1,0), so it spans a diameter in z
      // and runs the length of the thread along the diagonal.
      expect(box.size.z, closeTo(12.0, 0.4));
      expect(box.size.x, greaterThan(11));
    });

    test('sideways female thread on a bored box stays closed', () {
      final bored = IjoOps.boolean(
        MeshPrimitives.cube(20, 20, 20),
        MeshPrimitives.cylinder(
          origin: Vector3(-2, 10, 10),
          axis: Vector3(1, 0, 0),
          radius: 4,
          height: 24,
          segments: 64,
        ),
        IjoOps.subtractOp,
      );
      expect(MeshKit.isClosedSolid(bored), isTrue);
      final before = MeshKit.signedVolume(bored).abs();
      final threaded = IjoOps.threads(
        base: bored,
        origin: Vector3(0, 10, 10),
        axis: Vector3(1, 0, 0),
        majorRadius: 4,
        holeRadius: 4,
        axisZMin: 0,
        axisZMax: 20,
        pitchMm: 1.25,
        lengthMm: 12,
        starts: 1,
        rightHand: true,
        internal: true,
        fromEndA: true,
        fromEndB: false,
        segmentsPerTurn: 8,
      );
      expect(MeshKit.isClosedSolid(threaded), isTrue);
      final after = MeshKit.signedVolume(threaded).abs();
      expect(after, lessThan(before));
      expect(after, greaterThan(before * 0.85));
      final axis = Vector3(1, 0, 0);
      final origin = Vector3(0, 10, 10);
      var grooved = 0;
      for (final v in threaded.vertices) {
        final d = v - origin;
        final z = d.dot(axis);
        if (z < 1 || z > 11) continue;
        final radial = (d - axis * z).length;
        if (radial > 4.05 && radial < 4.9) grooved++;
      }
      expect(grooved, greaterThan(20),
          reason: 'bore wall must carry an outward ISO groove, not a smooth tube');
    });

    test('a pitch too coarse for the radius is refused, not attempted', () {
      expect(
        () => IjoOps.threads(
          base: MeshPrimitives.cylinder(
            origin: Vector3.zero(),
            axis: Vector3(0, 0, 1),
            radius: 1,
            height: 10,
          ),
          origin: Vector3.zero(),
          axis: Vector3(0, 0, 1),
          majorRadius: 1,
          holeRadius: 0,
          axisZMin: 0,
          axisZMax: 10,
          pitchMm: 4,
          lengthMm: 0,
          starts: 1,
          rightHand: true,
          internal: false,
          fromEndA: false,
          fromEndB: false,
        ),
        throwsA(
          isA<MeshOpException>().having(
            (e) => e.toString(),
            'message',
            contains('past the'),
          ),
        ),
      );
    });
  });
}
