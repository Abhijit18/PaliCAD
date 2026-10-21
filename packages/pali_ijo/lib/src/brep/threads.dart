import 'dart:math' as math;
import '../ijo_log.dart';
import 'package:vector_math/vector_math_64.dart';
import '../thread/form.dart';
import 'geom.dart';
import 'solid.dart';
import 'boolean.dart';
import 'primitives.dart';
import 'xform.dart';
import 'thread_wall.dart';
import 'provenance.dart';

/// Native helical profile sweeps. The curves and ruled faces are the geometry;
/// tessellation is only a display of those surfaces.
class BrepThreads {
  static void _checkBudget(
    double length,
    double pitch,
    int starts,
    int regions,
  ) {
    // Include the end patches for every start, even when the sweep contains
    // less than a turn. Reject pathological requests before allocating loops.
    final spans = 4 * length / pitch + 4 * starts * regions;
    if (!spans.isFinite || spans > 16384) {
      throw BrepUnsupported('Thread sweep exceeds the current face budget');
    }
  }

  static BrepSolid cutter({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double pitch,
    required double length,
    double taper = 0,
    double runoutLength = 0,
    bool internal = false,
    bool rightHand = true,
    int starts = 1,
    double phase = 0,
  }) {
    if (!runoutLength.isFinite ||
        runoutLength < 0 ||
        runoutLength * 2 > length) {
      throw ArgumentError(
        'Runout length must be between zero and half the threaded length',
      );
    }
    final form = ThreadForm(pitch: pitch, internal: internal);
    if (!radius.isFinite ||
        !length.isFinite ||
        !phase.isFinite ||
        !taper.isFinite ||
        radius <= 0 ||
        length <= 0 ||
        starts < 1 ||
        axis.length2 < 1e-20 ||
        !axis.length2.isFinite ||
        ![
          origin.x,
          origin.y,
          origin.z,
          axis.x,
          axis.y,
          axis.z,
        ].every((x) => x.isFinite)) {
      throw ArgumentError(
        'Threads require finite positive dimensions and a nonzero axis',
      );
    }
    if (!internal && radius <= form.depth)
      throw BrepUnsupported('Thread root reaches the shaft axis');
    final n = axis.normalized(), x = brepFrameU(n);
    final y = n.cross(x), lead = pitch * starts * (rightHand ? 1 : -1);
    final over = math.min(radius / 8, pitch / 128);
    final angle = length / (pitch * starts) * 2 * math.pi;
    if (!angle.isFinite || angle > 8192 * math.pi)
      throw BrepUnsupported('Thread sweep exceeds the current face budget');
    final count = math.max(1, (angle / (math.pi / 2)).ceil());
    final stations = [for (var i = 0; i <= count; i++) angle * i / count];
    final ramp = runoutLength / length * angle;
    if (ramp > 0) stations.addAll([ramp, angle - ramp]);
    stations.sort();
    final angles = <double>[];
    for (final t in stations)
      if (angles.isEmpty || t - angles.last > 1e-12) angles.add(t);
    final spans = angles.length - 1;
    final startZ = rightHand ? 0.0 : length;
    final xx = x * math.cos(phase) + y * math.sin(phase);
    (double, double) profile(int j, double t) {
      final scale = ramp == 0
          ? 1.0
          : math
                .min(1.0, math.min(t / ramp, (angle - t) / ramp))
                .clamp(0.0, 1.0);
      final lowR = internal ? radius - over : radius - form.depth * scale;
      final highR = internal ? radius + form.depth * scale : radius + over;
      final lowW = internal
          ? form.grooveHalfWidth * scale + over / math.sqrt(3)
          : form.rootHalfWidth * scale;
      final highW = internal
          ? form.rootHalfWidth * scale
          : form.grooveHalfWidth * scale + over / math.sqrt(3);
      final q = [
        (highR, -highW),
        (highR, highW),
        (lowR, lowW),
        (lowR, -lowW),
      ][j];
      final z = startZ + lead * t / (2 * math.pi) + q.$2;
      return (q.$1 + taper * z, z);
    }

    BrepHelix helix(int j, double a, double b) {
      final p = profile(j, a),
          q = profile(j, b),
          dr = (q.$1 - p.$1) / (b - a),
          dz = (q.$2 - p.$2) / (b - a);
      return BrepHelix(
        origin: origin + n * (p.$2 - dz * a),
        axis: n,
        xDir: xx,
        radius: p.$1 - dr * a,
        radiusSlope: dr,
        lead: dz * 2 * math.pi,
        t0: a,
        t1: b,
      );
    }

    final build = BrepBuilder();
    int vertex(Vector3 p) {
      for (var i = 0; i < build.vertices.length; i++)
        if ((build.vertices[i] - p).length < 1e-8) return i;
      return build.vertex(p);
    }

    final vertices = [
      for (var i = 0; i <= spans; i++)
        [
          for (var j = 0; j < 4; j++)
            vertex(
              origin +
                  (xx * math.cos(angles[i]) +
                          n.cross(xx) * math.sin(angles[i])) *
                      profile(j, angles[i]).$1 +
                  n * profile(j, angles[i]).$2,
            ),
        ],
    ];
    final rings = [
      for (var i = 0; i <= spans; i++)
        [
          for (var j = 0; j < 4; j++)
            vertices[i][j] == vertices[i][(j + 1) % 4]
                ? -1
                : build.edge(
                    vertices[i][j],
                    vertices[i][(j + 1) % 4],
                    BrepLine(
                      build.vertices[vertices[i][j]],
                      build.vertices[vertices[i][(j + 1) % 4]],
                    ),
                  ),
        ],
    ];
    for (var i = 0; i < spans; i++) {
      final curves = [
        for (var j = 0; j < 4; j++) helix(j, angles[i], angles[i + 1]),
      ];
      final rails = [
        for (var j = 0; j < 4; j++)
          build.edge(vertices[i][j], vertices[i + 1][j], curves[j]),
      ];
      for (var j = 0; j < 4; j++) {
        final k = (j + 1) % 4;
        build.face(BrepRuledSurface(curves[j], curves[k]), [
          BrepLoop([
            BrepCoedge(rails[j]),
            if (rings[i + 1][j] >= 0) BrepCoedge(rings[i + 1][j]),
            BrepCoedge(rails[k], reversed: true),
            if (rings[i][j] >= 0) BrepCoedge(rings[i][j], reversed: true),
          ]),
        ]);
      }
    }
    final yy = n.cross(xx);
    build.face(
      BrepPlane(origin: build.vertices[vertices.first.first], normal: -yy),
      [
        BrepLoop([
          for (final e in rings.first)
            if (e >= 0) BrepCoedge(e),
        ]),
      ],
    );
    final endNormal = -xx * math.sin(angle) + yy * math.cos(angle);
    build.face(
      BrepPlane(origin: build.vertices[vertices.last.first], normal: endNormal),
      [
        BrepLoop([
          for (final e in rings.last.reversed)
            if (e >= 0) BrepCoedge(e, reversed: true),
        ]),
      ],
    );
    return build.build();
  }

  /// Threads the requested axial regions and keeps unaffected BREP faces.
  /// Internal threads on a nominal major-diameter bore form crests inward;
  /// smaller predrilled bores receive outward cuts to the major diameter.
  /// The cutter is clipped before subtraction, so a partial thread cannot
  /// accidentally cut a shoulder or floor beyond its selected length.
  static BrepSolid apply(
    BrepSolid body, {
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double pitch,
    required double axisZMin,
    required double axisZMax,
    required double length,
    double taper = 0,
    double runoutLength = 0,
    bool internal = false,
    bool rightHand = true,
    int starts = 1,
    bool fromEndA = true,
    bool fromEndB = false,
  }) {
    if (starts < 1 ||
        !radius.isFinite ||
        radius <= 0 ||
        axis.length2 < 1e-20 ||
        !axis.length2.isFinite ||
        ![
          origin.x,
          origin.y,
          origin.z,
          axis.x,
          axis.y,
          axis.z,
        ].every((v) => v.isFinite)) {
      throw ArgumentError(
        'Threads require positive radius/start count and a finite nonzero axis',
      );
    }
    final regions = ThreadForm.regions(
      axisZMin: axisZMin,
      axisZMax: axisZMax,
      length: length,
      fromEndA: fromEndA,
      fromEndB: fromEndB,
    );
    final n = axis.normalized(), x = brepFrameU(n), y = n.cross(x);
    final form = ThreadForm(pitch: pitch, internal: internal);
    if (!runoutLength.isFinite ||
        runoutLength < 0 ||
        regions.any((r) => 2 * runoutLength > r.$2 - r.$1)) {
      throw ArgumentError('Runout must fit within each threaded region');
    }
    if (!taper.isFinite) throw ArgumentError('Thread taper must be finite');
    _checkBudget(
      regions.fold(0.0, (sum, r) => sum + r.$2 - r.$1 + 2 * pitch),
      pitch,
      starts,
      regions.length,
    );
    final direct = taper == 0 && runoutLength == 0
        ? BrepThreadWall.tryBuild(
            body,
            origin: origin,
            axis: n,
            radius: radius,
            pitch: pitch,
            lo: math.min(axisZMin, axisZMax),
            hi: math.max(axisZMin, axisZMax),
            regions: regions,
            internal: internal,
            rightHand: rightHand,
            starts: starts,
          )
        : null;
    if (direct != null)
      return BrepProvenance.inheritFrom(direct, body, op: 'threads');
    if (!internal && taper == 0 && runoutLength == 0) {
      final replaced = BrepThreadWall.tryReplaceCylinder(
        body,
        origin: origin,
        axis: n,
        radius: radius,
        pitch: pitch,
        lo: math.min(axisZMin, axisZMax),
        hi: math.max(axisZMin, axisZMax),
        regions: regions,
        rightHand: rightHand,
        starts: starts,
      );
      if (replaced != null)
        return BrepProvenance.inheritFrom(replaced, body, op: 'threads');
    }
    if (internal && taper == 0 && runoutLength == 0) {
      final replaced = BrepThreadWall.tryReplaceBore(
        body,
        origin: origin,
        axis: n,
        radius: radius,
        pitch: pitch,
        lo: math.min(axisZMin, axisZMax),
        hi: math.max(axisZMin, axisZMax),
        regions: regions,
        rightHand: rightHand,
        starts: starts,
      );
      if (replaced != null)
        return BrepProvenance.inheritFrom(replaced, body, op: 'threads');
    }
    // Subtracting from an existing major-diameter bore cannot create crests.
    // If direct wall construction cannot handle it, fail instead of reporting
    // an unchanged bore as a successfully modeled thread.
    if (internal &&
        body.faces.any((face) {
          final surface = face.surface;
          return face.reversed &&
              surface is BrepCylinder &&
              surface.axis.cross(n).length < 1e-8 &&
              (surface.origin - origin).cross(n).length < 1e-7 &&
              surface.radius >= radius + form.depth - 1e-7;
        })) {
      throw BrepUnsupported(
        'Nominal-size female threads require a complete cylindrical bore with clear end faces',
      );
    }
    IjoLog.line(
      'threads using general boundary subtraction internal=$internal',
    );
    final margin =
        math.max(radius + taper * axisZMin, radius + taper * axisZMax) +
        form.depth +
        pitch;
    final tools = <BrepSolid>[];
    for (final region in regions) {
      final extension = runoutLength == 0 ? pitch : 0.0;
      final z0 = region.$1 - extension,
          extent = region.$2 - region.$1 + 2 * extension;
      final clip = BrepXform.map(
        BrepPrimitives.box(2 * margin, 2 * margin, region.$2 - region.$1),
        (p) =>
            origin +
            x * (p.x - margin) +
            y * (p.y - margin) +
            n * (p.z + region.$1),
      );
      for (var start = 0; start < starts; start++) {
        final phase =
            2 * math.pi * start / starts +
            2 *
                math.pi *
                ((rightHand ? z0 : z0 + extent) -
                    math.min(axisZMin, axisZMax)) /
                (pitch * starts * (rightHand ? 1 : -1));
        final sweep = cutter(
          origin: origin + n * z0,
          axis: n,
          radius: radius + taper * z0,
          taper: taper,
          runoutLength: runoutLength,
          pitch: pitch,
          length: extent,
          internal: internal,
          rightHand: rightHand,
          starts: starts,
          phase: phase,
        );
        final tool = BrepBoolean.run(sweep, clip, BrepBoolean.intersectOp);
        tools.add(tool);
      }
    }
    return BrepBoolean.run(body, _compound(tools), BrepBoolean.subtractOp);
  }

  static BrepSolid subtract(
    BrepSolid body, {
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double pitch,
    required double length,
    double taper = 0,
    double runoutLength = 0,
    bool internal = false,
    bool rightHand = true,
    int starts = 1,
  }) {
    if (starts < 1) throw ArgumentError('Thread start count must be positive');
    _checkBudget(length, pitch, starts, 1);
    final tools = <BrepSolid>[];
    for (var start = 0; start < starts; start++) {
      final tool = cutter(
        origin: origin,
        axis: axis,
        radius: radius,
        taper: taper,
        runoutLength: runoutLength,
        pitch: pitch,
        length: length,
        internal: internal,
        rightHand: rightHand,
        starts: starts,
        phase: 2 * math.pi * start / starts,
      );
      tools.add(tool);
    }
    return BrepBoolean.run(body, _compound(tools), BrepBoolean.subtractOp);
  }

  // Standard groove profiles leave a positive crest land between starts.
  // They are disjoint cutter volumes; one compound subtraction avoids making
  // the next start intersect the thread faces produced by the previous start.
  static BrepSolid _compound(List<BrepSolid> solids) {
    if (solids.length == 1) return solids.single;
    final b = BrepBuilder();
    for (final s in solids) {
      final v = b.vertices.length, e = b.edges.length;
      for (final p in s.vertices) b.vertex(p);
      for (final c in s.edges)
        b.edge(c.v0 + v, c.v1 + v, c.curve, origin: c.origin);
      for (final f in s.faces)
        b.face(
          f.surface,
          [
            for (final l in f.loops)
              BrepLoop([
                for (final c in l.coedges)
                  BrepCoedge(c.edge + e, reversed: c.reversed),
              ]),
          ],
          reversed: f.reversed,
          origin: f.origin,
        );
    }
    return b.build();
  }
}
