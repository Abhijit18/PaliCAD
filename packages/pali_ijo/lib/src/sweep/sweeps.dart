import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../csg/boolean.dart';
import '../csg/mesh_solid.dart';
import '../mesh/mesh_build.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_frame.dart';
import '../mesh/mesh_op.dart';
import '../mesh/mesh_seal.dart';
import 'profile.dart';
import 'revolve.dart';
import 'skin.dart';

/// Sweeps: pulling or turning a 2D profile along a trajectory to make a 3D
/// feature.
///
/// [IjoRevolve] is kept separate because it is exact. Everything here is
/// faceted, for the reason set out on [IjoSkin]: the surface a profile sweeps
/// along a curved path has no exact name in this kernel's catalog, so it is
/// approximated by a controllable number of stations rather than misrepresented
/// as something it is not.
class IjoSweep {
  IjoSweep._();

  /// Sweeps [profile] along the polyline [path].
  ///
  /// The profile is re-placed at each point of the path in a parallel
  /// transported frame, so it turns to follow the path without twisting. Its
  /// own plane is ignored beyond fixing its 2D shape — what matters is the
  /// path.
  ///
  /// [twistRadians] adds an even twist along the whole sweep. [closed] joins
  /// the ends into a ring and drops the caps.
  static MeshData path(
    SweepProfile profile,
    List<Vector3> path, {
    bool closed = false,
    double twistRadians = 0,
    int arcSegments = 32,
    int? sectionPoints,
  }) {
    var frames = PathFrames.along(path, closed: closed);
    if (closed) frames = frames.closedTwistCorrection;
    if (twistRadians != 0) frames = frames.twisted(twistRadians);
    return withFrames(
      profile,
      frames,
      arcSegments: arcSegments,
      sectionPoints: sectionPoints,
    );
  }

  /// Sweeps [profile] through [frames], the general form the others build on.
  ///
  /// Use this when the section's orientation along the path matters and the
  /// default parallel transport is not what you want — see
  /// [PathFrames.aboutAxis].
  static MeshData withFrames(
    SweepProfile profile,
    PathFrames frames, {
    int arcSegments = 32,
    int? sectionPoints,
  }) {
    final flat = _flatten(profile, arcSegments, sectionPoints);
    return IjoSkin.solid(
      [
        for (var i = 0; i < frames.length; i++)
          [for (final (u, v) in flat) frames.place(i, u, v)],
      ],
      closed: frames.closed,
    );
  }

  /// A round tube of [radius] following [path].
  ///
  /// The common case of [IjoSweep.path], and the one PaliCAD's pipe tool
  /// needs. A straight two-point path is a cylinder mesh.
  static MeshData pipe(
    List<Vector3> path, {
    required double radius,
    bool closed = false,
    int sides = 32,
  }) {
    if (radius <= 0) {
      throw MeshOpException('A pipe needs a positive radius');
    }
    if (!closed && path.length == 2) {
      final d = path[1] - path[0];
      if (d.length > 1e-12) {
        return MeshPrimitives.cylinder(
          origin: path[0],
          axis: d,
          radius: radius,
          height: d.length,
          segments: sides,
        );
      }
    }
    return IjoSweep.path(
      SweepProfile.circle(radius: radius, segments: sides),
      path,
      closed: closed,
      arcSegments: sides,
    );
  }

  /// [pipe] as a triangle / n-gon mesh, without recovering a B-Rep.
  ///
  /// Healing the skin ([IjoSkin.mesh]) merges every
  /// coplanar tube wall into one n-gon, then fans it from a corner — that is
  /// the exploded mesh on a D-loop with long straight sides. The closed mesh
  /// is CSG truth, so the host never needs those faces.
  ///
  /// Sharp polyline corners are filleted to [radius] first so the section
  /// does not fold over itself on a 90° turn.
  static MeshData pipeMesh(
    List<Vector3> path, {
    required double radius,
    bool closed = false,
    int sides = 32,
  }) {
    if (radius <= 0) {
      throw MeshOpException('A pipe needs a positive radius');
    }
    final rounded = PathFrames.filletCorners(
      path,
      radius: radius,
      closed: closed,
    );
    var frames = PathFrames.along(rounded, closed: closed);
    if (closed) frames = frames.closedTwistCorrection;
    final flat = _flatten(
      SweepProfile.circle(radius: radius, segments: sides),
      sides,
      null,
    );
    var mesh = IjoSkin.mesh(
      [
        for (var i = 0; i < frames.length; i++)
          [for (final (u, v) in flat) frames.place(i, u, v)],
      ],
      closed: frames.closed,
    );
    if (!MeshSeal.isClosedSolid(mesh)) {
      mesh = MeshSeal.sealSolid(mesh);
    }
    return mesh;
  }

  /// A hollow tube: [pipe] of [outerRadius] with a [innerRadius] bore.
  static MeshData tube(
    List<Vector3> path, {
    required double outerRadius,
    required double innerRadius,
    bool closed = false,
    int sides = 32,
  }) {
    if (innerRadius <= 0 || innerRadius >= outerRadius) {
      throw MeshOpException(
        'A tube needs 0 < innerRadius < outerRadius, got '
        '$innerRadius and $outerRadius',
      );
    }
    return MeshBoolean.run(
      pipe(path, radius: outerRadius, closed: closed, sides: sides),
      pipe(path, radius: innerRadius, closed: closed, sides: sides),
      MeshBoolean.subtractOp,
    );
  }

  /// Lofts through [sections] in order.
  ///
  /// Sections need not agree on point count or shape — each is resampled to a
  /// common count by arc length, which is what makes lofting a square to a
  /// circle possible. They are also rotated to start at a consistent place, so
  /// the sides do not spiral: see [_alignedTo].
  ///
  /// Corresponding vertices of successive resampled rings are joined by
  /// [IjoSkin] to form the side walls. Open sequences receive end caps;
  /// closed sequences join the last ring back to the first without caps.
  ///
  /// [closed] loops the last section back to the first.
  static MeshData loft(
    List<SweepProfile> sections, {
    bool closed = false,
    int? sectionPoints,
    int arcSegments = 64,
  }) =>
      IjoSkin.solid(
        _loftRings(
          sections,
          closed: closed,
          sectionPoints: sectionPoints,
          arcSegments: arcSegments,
        ),
        closed: closed,
      );

  /// [loft] as a triangle / n-gon mesh, without recovering a B-Rep.
  ///
  /// Same path the pipe helix cutter uses: [IjoSkin.mesh] only. Healing the
  /// skin back into analytic faces is the slow step and is not needed when
  /// the host already treats the closed mesh as CSG truth.
  static MeshData loftMesh(
    List<SweepProfile> sections, {
    bool closed = false,
    int? sectionPoints,
    int arcSegments = 64,
  }) =>
      IjoSkin.mesh(
        _loftRings(
          sections,
          closed: closed,
          sectionPoints: sectionPoints,
          arcSegments: arcSegments,
        ),
        closed: closed,
      );

  static List<List<Vector3>> _loftRings(
    List<SweepProfile> sections, {
    required bool closed,
    int? sectionPoints,
    int arcSegments = 64,
  }) {
    if (sections.length < 2) {
      throw MeshOpException('A loft needs at least 2 sections');
    }

    // Enough points to represent the most detailed section faithfully, and a
    // multiple of 4 so a rectangle's corners land on stations rather than
    // being rounded off.
    final count = sectionPoints ??
        math.max(
          16,
          4 *
              ((sections
                          .map((s) => s.ring(arcSegments: arcSegments).length)
                          .reduce(math.max) +
                      3) ~/
                  4),
        );

    // Every section is wound the same way, judged against the direction the
    // loft travels, so the belts do not come out inside-out halfway along.
    final travel = (sections.last.origin - sections.first.origin);
    if (!closed) {
      final offFirst =
          (sections.last.origin - sections.first.origin).dot(sections.first.normal);
      final offLast =
          (sections.first.origin - sections.last.origin).dot(sections.last.normal);
      if (offFirst.abs() < 1e-6 && offLast.abs() < 1e-6) {
        throw MeshOpException(
          'A loft needs sections on different planes',
        );
      }
    }
    final heading = travel.length2 < 1e-18 ? sections.first.normal : travel;

    final rings = <List<Vector3>>[];
    for (var i = 0; i < sections.length; i++) {
      final oriented = sections[i].orientedTo(heading);
      final ring = oriented.resampled(count, arcSegments: arcSegments);
      rings.add(i == 0 ? ring : _alignedTo(rings.last, ring));
    }
    return rings;
  }

  /// A helix polyline about the axis through [origin] along [direction].
  ///
  /// [turns] may be fractional. A positive [pitch] advances along the axis per
  /// turn; [rightHand] chooses the direction of rotation. [taperPerTurn]
  /// changes the radius per turn, which makes a conical spring.
  static List<Vector3> helixPath({
    required Vector3 origin,
    required Vector3 direction,
    required double radius,
    required double pitch,
    required double turns,
    bool rightHand = true,
    double taperPerTurn = 0,
    int segmentsPerTurn = 32,
  }) {
    if (radius <= 0) throw MeshOpException('A helix needs a positive radius');
    if (turns <= 0) throw MeshOpException('A helix needs positive turns');

    final axis = direction.normalized();
    final x = meshFrameU(axis);
    final y = axis.cross(x).normalized();
    final steps = math.max(2, (segmentsPerTurn * turns).ceil());
    final sign = rightHand ? 1.0 : -1.0;

    return [
      for (var i = 0; i <= steps; i++)
        () {
          final t = turns * i / steps;
          final a = sign * 2 * math.pi * t;
          final r = radius + taperPerTurn * t;
          return origin +
              x * (r * math.cos(a)) +
              y * (r * math.sin(a)) +
              axis * (pitch * t);
        }(),
    ];
  }

  /// A solid helix: [profile] swept along [helixPath].
  ///
  /// This is the operation threads are made of. The profile's own [xDir] is
  /// carried to the radial direction and its y to the axis, so a thread's
  /// cross-section arrives with its depth pointing at the axis at every turn.
  /// That is why the frames come from [PathFrames.aboutAxis] rather than from
  /// parallel transport, which would let the section roll as it climbs.
  static MeshData helix(
    SweepProfile profile, {
    required Vector3 origin,
    required Vector3 direction,
    required double radius,
    required double pitch,
    required double turns,
    bool rightHand = true,
    double taperPerTurn = 0,
    int segmentsPerTurn = 32,
    int arcSegments = 32,
  }) =>
      IjoSkin.solid(
        _helixRings(
          profile,
          origin: origin,
          direction: direction,
          radius: radius,
          pitch: pitch,
          turns: turns,
          rightHand: rightHand,
          taperPerTurn: taperPerTurn,
          segmentsPerTurn: segmentsPerTurn,
          arcSegments: arcSegments,
        ),
      );

  /// [helix] as a triangle mesh, without recovering a B-Rep.
  static MeshData helixMesh(
    SweepProfile profile, {
    required Vector3 origin,
    required Vector3 direction,
    required double radius,
    required double pitch,
    required double turns,
    bool rightHand = true,
    double taperPerTurn = 0,
    int segmentsPerTurn = 32,
    int arcSegments = 32,
  }) =>
      IjoSkin.mesh(
        _helixRings(
          profile,
          origin: origin,
          direction: direction,
          radius: radius,
          pitch: pitch,
          turns: turns,
          rightHand: rightHand,
          taperPerTurn: taperPerTurn,
          segmentsPerTurn: segmentsPerTurn,
          arcSegments: arcSegments,
        ),
      );

  static List<List<Vector3>> _helixRings(
    SweepProfile profile, {
    required Vector3 origin,
    required Vector3 direction,
    required double radius,
    required double pitch,
    required double turns,
    required bool rightHand,
    required double taperPerTurn,
    required int segmentsPerTurn,
    required int arcSegments,
  }) {
    final frames = PathFrames.aboutAxis(
      helixPath(
        origin: origin,
        direction: direction,
        radius: radius,
        pitch: pitch,
        turns: turns,
        rightHand: rightHand,
        taperPerTurn: taperPerTurn,
        segmentsPerTurn: segmentsPerTurn,
      ),
      origin,
      direction,
    );
    final flat = _flatten(profile, arcSegments, null);
    return [
      for (var i = 0; i < frames.length; i++)
        [for (final (u, v) in flat) frames.place(i, u, v)],
    ];
  }

  /// Extrudes [profile] along [direction] with an even twist and a taper.
  ///
  /// Without either it is a plain prism.
  static MeshData linear(
    SweepProfile profile,
    Vector3 direction, {
    double twistRadians = 0,
    double scaleTop = 1,
    int stations = 32,
    int arcSegments = 32,
  }) {
    if (direction.length < 1e-12) {
      throw MeshOpException('A linear sweep needs a non-zero direction');
    }
    if (twistRadians == 0 && (scaleTop - 1).abs() < 1e-12) {
      return MeshBuild.sweepPolygon(
        profile.ring(arcSegments: arcSegments),
        direction,
      );
    }
    if (scaleTop <= 0) {
      throw MeshOpException('scaleTop must be positive');
    }

    final flat = _flatten(profile, arcSegments, null);
    final n = direction.normalized();
    final x = meshFrameU(n);
    final y = n.cross(x).normalized();
    final base = profile.origin;

    return IjoSkin.solid([
      for (var i = 0; i <= stations; i++)
        () {
          final f = i / stations;
          final s = 1 + (scaleTop - 1) * f;
          final a = twistRadians * f;
          final ca = math.cos(a);
          final sa = math.sin(a);
          final at = base + direction * f;
          return [
            for (final (u, v) in flat)
              at + x * ((u * ca - v * sa) * s) + y * ((u * sa + v * ca) * s),
          ];
        }(),
    ]);
  }

  /// The profile as 2D coordinates in its own plane, wound counter-clockwise
  /// about its normal so every sweep starts from the same convention.
  static List<(double, double)> _flatten(
    SweepProfile profile,
    int arcSegments,
    int? sectionPoints,
  ) {
    final x = profile.xDir;
    final y = profile.yDir;
    final ring = sectionPoints == null
        ? profile.ring(arcSegments: arcSegments)
        : profile.resampled(sectionPoints, arcSegments: arcSegments);
    return [
      for (final p in ring)
        ((p - profile.origin).dot(x), (p - profile.origin).dot(y)),
    ];
  }

  /// [ring] rotated in its own order to start nearest [reference]'s start.
  ///
  /// Resampling two sections independently starts each at an arbitrary place
  /// on its perimeter. Skinning them as-is would run the belts diagonally, so
  /// a loft from a square to a rotated square would come out visibly twisted.
  /// This picks the offset that minimises the total distance between paired
  /// points, which is the offset a person would have chosen.
  static List<Vector3> _alignedTo(
    List<Vector3> reference,
    List<Vector3> ring,
  ) {
    if (reference.length != ring.length) return ring;
    var bestShift = 0;
    var bestCost = double.infinity;
    for (var shift = 0; shift < ring.length; shift++) {
      var cost = 0.0;
      for (var i = 0; i < ring.length; i++) {
        cost += (ring[(i + shift) % ring.length] - reference[i]).length2;
        if (cost >= bestCost) break;
      }
      if (cost < bestCost) {
        bestCost = cost;
        bestShift = shift;
      }
    }
    if (bestShift == 0) return ring;
    return [
      for (var i = 0; i < ring.length; i++) ring[(i + bestShift) % ring.length],
    ];
  }
}
