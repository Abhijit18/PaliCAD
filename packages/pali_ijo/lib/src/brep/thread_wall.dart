import 'dart:math' as math;
import 'package:vector_math/vector_math_64.dart';
import '../thread/form.dart';
import 'geom.dart';
import 'boolean.dart';
import 'solid.dart';
import 'heal.dart';
import 'primitives.dart';

/// Direct boundary construction for complete coaxial cylindrical walls.
/// Work is proportional to the number of turns, without repeated booleans.
/// Parts with interrupted walls continue through the general intersection path.
class BrepThreadWall {
  /// Replace a complete external cylindrical wall on a larger planar part.
  /// Adjacent cap/shoulder loops inherit the new helical rim; the head and its
  /// other faces are retained exactly. Interrupted walls use the general CSG.
  static BrepSolid? tryReplaceCylinder(
    BrepSolid body, {
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double pitch,
    required double lo,
    required double hi,
    required List<(double, double)> regions,
    required bool rightHand,
    required int starts,
  }) {
    final walls = body.faces
        .where(
          (f) =>
              f.surface is BrepCylinder &&
              ((f.surface as BrepCylinder).radius - radius).abs() < 1e-7 &&
              (f.surface as BrepCylinder).axis.cross(axis).length < 1e-8 &&
              ((f.surface as BrepCylinder).origin - origin).cross(axis).length <
                  1e-7,
        )
        .toList();
    if (walls.length != 1 ||
        body.faces.any(
          (f) => f.surface is! BrepPlane && f.surface is! BrepCylinder,
        ))
      return null;
    final wall = walls.single, cylinder = wall.surface as BrepCylinder;
    final n = axis.normalized();
    final rootRadius = radius - ThreadForm(pitch: pitch, internal: false).depth;
    bool insideRoot(BrepCurve c) {
      if (c is BrepCircle)
        return c.normal.cross(n).length < 1e-8 &&
            (c.center - origin).cross(n).length + c.radius < rootRadius - 1e-7;
      if (c is BrepLine)
        return [
          c.a,
          c.b,
        ].every((p) => (p - origin).cross(n).length < rootRadius - 1e-7);
      return false;
    }

    if (wall.reversed ||
        (cylinder.radius - radius).abs() > 1e-7 ||
        cylinder.axis.cross(n).length > 1e-8 ||
        (cylinder.origin - origin).cross(n).length > 1e-7)
      return null;
    final wallEdges = {
      for (final l in wall.loops)
        for (final c in l.coedges) c.edge,
    };
    final rims = <double, Set<int>>{lo: {}, hi: {}};
    for (final id in wallEdges) {
      final c = body.edges[id].curve;
      if (c is BrepLine) {
        if ((c.evaluate(c.t1) - c.evaluate(c.t0)).cross(n).length > 1e-7)
          return null;
        continue;
      }
      if (c is! BrepCircle ||
          (c.radius - radius).abs() > 1e-7 ||
          c.normal.cross(n).length > 1e-8)
        return null;
      final z = (c.center - origin).dot(n);
      final end = (z - lo).abs() < 1e-7
          ? lo
          : (z - hi).abs() < 1e-7
          ? hi
          : null;
      if (end == null) return null;
      rims[end]!.add(id);
    }
    for (final rim in rims.values) {
      final angle = rim.fold(0.0, (v, id) {
        final c = body.edges[id].curve as BrepCircle;
        return v + (c.t1 - c.t0).abs();
      });
      if ((angle - 2 * math.pi).abs() > 1e-7) return null;
    }
    final replacements = <BrepFace, (int, double)>{};
    for (final f in body.faces) {
      if (identical(f, wall)) continue;
      if (f.surface is BrepCylinder) {
        final c = f.surface as BrepCylinder;
        if (c.axis.cross(n).length > 1e-8 ||
            (c.origin - origin).cross(n).length + c.radius >= rootRadius - 1e-7)
          return null;
        continue;
      }
      final plane = f.surface as BrepPlane;
      final z = (plane.origin - origin).dot(n);
      if (plane.normal.cross(n).length < 1e-8) {
        final end = (z - lo).abs() < 1e-7
            ? lo
            : (z - hi).abs() < 1e-7
            ? hi
            : null;
        if (end == null) {
          if (z > lo && z < hi) return null;
          continue;
        }
        final ids = rims[end]!;
        final index = f.loops.indexWhere(
          (l) =>
              l.coedges.length == ids.length &&
              l.coedges.every((c) => ids.contains(c.edge)),
        );
        if (index < 0) {
          if (f.loops.every(
            (l) =>
                l.coedges.every((co) => insideRoot(body.edges[co.edge].curve)),
          ))
            continue;
          return null;
        }
        // Additional boundaries must stay outside the cylinder's envelope.
        // This deliberately excludes holes or pockets intersecting the thread.
        for (var i = 0; i < f.loops.length; i++) {
          if (i == index) continue;
          for (final co in f.loops[i].coedges) {
            final c = body.edges[co.edge].curve;
            if (insideRoot(c)) continue;
            if (c is! BrepLine) return null;
            final a = c.evaluate(c.t0) - origin;
            final b = c.evaluate(c.t1) - origin;
            final p = a - n * a.dot(n), q = b - n * b.dot(n), d = q - p;
            final t = d.length2 == 0
                ? 0.0
                : (-p.dot(d) / d.length2).clamp(0.0, 1.0);
            if ((p + d * t).length < radius - 1e-7) return null;
          }
        }
        replacements[f] = (index, end);
      } else if (plane.normal.dot(n).abs() > 1e-8 ||
          (origin - plane.origin).dot(plane.normal).abs() < radius - 1e-7) {
        return null;
      }
    }
    if (replacements.length != 2 ||
        replacements.values.map((v) => v.$2).toSet().length != 2)
      return null;
    final patch = tryBuild(
      BrepPrimitives.cylinder(
        origin: origin + n * lo,
        axis: n,
        radius: radius,
        height: hi - lo,
      ),
      origin: origin,
      axis: n,
      radius: radius,
      pitch: pitch,
      lo: lo,
      hi: hi,
      regions: regions,
      internal: false,
      rightHand: rightHand,
      starts: starts,
    );
    if (patch == null) return null;
    final caps = <double, BrepFace>{};
    for (final f in patch.faces) {
      final s = f.surface;
      if (s is! BrepPlane) continue;
      final z = (s.origin - origin).dot(n);
      if ((z - lo).abs() < 1e-7) caps[lo] = f;
      if ((z - hi).abs() < 1e-7) caps[hi] = f;
    }
    if (caps.length != 2 || caps.values.any((f) => f.loops.length != 1))
      return null;
    final joined = BrepSolid.compound([body, patch]);
    final offset = body.edges.length;
    BrepLoop remap(BrepLoop l, bool reverse) => BrepLoop([
      for (final c in reverse ? l.coedges.reversed : l.coedges)
        BrepCoedge(c.edge + offset, reversed: c.reversed != reverse),
    ]);
    final faces = <BrepFace>[];
    for (final f in body.faces) {
      if (identical(f, wall)) continue;
      final replacement = replacements[f];
      final loops = List<BrepLoop>.from(f.loops);
      if (replacement != null) {
        final cap = caps[replacement.$2]!;
        final reverse = f.outwardNormal(0, 0).dot(cap.outwardNormal(0, 0)) < 0;
        loops[replacement.$1] = remap(cap.loops.single, reverse);
      }
      faces.add(
        BrepFace(
          surface: f.surface,
          loops: loops,
          reversed: f.reversed,
          origin: f.origin,
        ),
      );
    }
    for (var i = 0; i < patch.faces.length; i++) {
      if (!caps.values.contains(patch.faces[i]))
        faces.add(joined.faces[body.faces.length + i]);
    }
    final result = BrepSolid(
      vertices: joined.vertices,
      edges: joined.edges,
      faces: faces,
    );
    return BrepHeal.topologyIsClosed(result) ? result : null;
  }

  /// Replace a complete bore in a planar part without intersecting every
  /// helical face against the body. Clearance checks reject interrupted bores.
  static BrepSolid? tryReplaceBore(
    BrepSolid body, {
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double pitch,
    required double lo,
    required double hi,
    required List<(double, double)> regions,
    required bool rightHand,
    required int starts,
  }) {
    final majorRadius = radius + ThreadForm(pitch: pitch, internal: true).depth;
    final walls = body.faces
        .where(
          (f) =>
              f.surface is BrepCylinder &&
              f.reversed &&
              (f.surface as BrepCylinder).radius >= radius - 1e-7 &&
              (f.surface as BrepCylinder).radius <= majorRadius + 1e-7 &&
              (f.surface as BrepCylinder).axis.cross(axis).length < 1e-8 &&
              ((f.surface as BrepCylinder).origin - origin).cross(axis).length <
                  1e-7,
        )
        .toList();
    if (walls.length != 1 ||
        body.faces.any(
          (f) => f.surface is! BrepPlane && f.surface is! BrepCylinder,
        ))
      return null;
    final wall = walls.single, cylinder = wall.surface as BrepCylinder;
    final n = axis.normalized();
    final boreRadius = cylinder.radius;
    if (!wall.reversed) return null;
    final wallEdges = {
      for (final l in wall.loops)
        for (final c in l.coedges) c.edge,
    };
    final rims = <double, Set<int>>{lo: {}, hi: {}};
    for (final id in wallEdges) {
      final c = body.edges[id].curve;
      if (c is BrepLine) {
        if ((c.evaluate(c.t1) - c.evaluate(c.t0)).cross(n).length > 1e-7)
          return null;
        continue;
      }
      if (c is! BrepCircle ||
          (c.radius - boreRadius).abs() > 1e-7 ||
          c.normal.cross(n).length > 1e-8)
        return null;
      final z = (c.center - origin).dot(n);
      final end = (z - lo).abs() < 1e-7
          ? lo
          : (z - hi).abs() < 1e-7
          ? hi
          : null;
      if (end == null) return null;
      rims[end]!.add(id);
    }
    for (final rim in rims.values) {
      final angle = rim.fold(0.0, (v, id) {
        final c = body.edges[id].curve as BrepCircle;
        return v + (c.t1 - c.t0).abs();
      });
      if ((angle - 2 * math.pi).abs() > 1e-7) return null;
    }
    final replacements = <BrepFace, (int, double)>{};
    for (final f in body.faces) {
      if (identical(f, wall)) continue;
      if (f.surface is BrepCylinder) {
        final c = f.surface as BrepCylinder;
        // Only accept other cylinders entirely outside the swept envelope.
        // Intersecting or coaxial stepped bores need general trimming.
        final distance = (c.origin - origin).cross(n).length;
        if (c.axis.cross(n).length > 1e-8 ||
            (distance - c.radius <= majorRadius + 1e-7 &&
                c.radius - distance <= majorRadius + 1e-7))
          return null;
        continue;
      }
      final plane = f.surface as BrepPlane;
      final z = (plane.origin - origin).dot(n);
      if (plane.normal.cross(n).length < 1e-8) {
        final end = (z - lo).abs() < 1e-7
            ? lo
            : (z - hi).abs() < 1e-7
            ? hi
            : null;
        if (end == null) {
          if (z > lo && z < hi) return null;
          continue;
        }
        final ids = rims[end]!;
        final index = f.loops.indexWhere(
          (l) =>
              l.coedges.length == ids.length &&
              l.coedges.every((c) => ids.contains(c.edge)),
        );
        if (index < 0) return null;
        // Additional boundaries must stay outside the cylinder's envelope.
        // This deliberately excludes holes or pockets intersecting the thread.
        for (var i = 0; i < f.loops.length; i++) {
          if (i == index) continue;
          for (final co in f.loops[i].coedges) {
            final c = body.edges[co.edge].curve;
            if (c is BrepCircle) {
              final distance = (c.center - origin).cross(n).length;
              if (c.normal.cross(n).length > 1e-8 ||
                  (distance - c.radius <= majorRadius + 1e-7 &&
                      c.radius - distance <= majorRadius + 1e-7))
                return null;
              continue;
            }
            if (c is! BrepLine) return null;
            final a = c.evaluate(c.t0) - origin;
            final b = c.evaluate(c.t1) - origin;
            final p = a - n * a.dot(n), q = b - n * b.dot(n), d = q - p;
            final t = d.length2 == 0
                ? 0.0
                : (-p.dot(d) / d.length2).clamp(0.0, 1.0);
            if ((p + d * t).length <= majorRadius + 1e-7) return null;
          }
        }
        replacements[f] = (index, end);
      } else if (plane.normal.dot(n).abs() > 1e-8 ||
          (origin - plane.origin).dot(plane.normal).abs() <=
              majorRadius + 1e-7) {
        return null;
      }
    }
    if (replacements.length != 2 ||
        replacements.values.map((v) => v.$2).toSet().length != 2)
      return null;
    // A temporary coaxial tube supplies a closed helical bore and its rims.
    // Its outer wall is discarded; the actual body's exterior is retained.
    final tube = BrepBoolean.run(
      BrepPrimitives.cylinder(
        origin: origin + n * lo,
        axis: n,
        radius: majorRadius + pitch,
        height: hi - lo,
      ),
      BrepPrimitives.cylinder(
        origin: origin + n * (lo - pitch),
        axis: n,
        radius: boreRadius,
        height: hi - lo + 2 * pitch,
      ),
      BrepBoolean.subtractOp,
    );
    final patch = tryBuild(
      tube,
      origin: origin,
      axis: n,
      radius: radius,
      pitch: pitch,
      lo: lo,
      hi: hi,
      regions: regions,
      internal: true,
      rightHand: rightHand,
      starts: starts,
    );
    if (patch == null) return null;
    final caps = <double, BrepFace>{};
    for (final f in patch.faces) {
      final s = f.surface;
      if (s is! BrepPlane) continue;
      final z = (s.origin - origin).dot(n);
      if ((z - lo).abs() < 1e-7) caps[lo] = f;
      if ((z - hi).abs() < 1e-7) caps[hi] = f;
    }
    if (caps.length != 2 || caps.values.any((f) => f.loops.length != 2))
      return null;
    final joined = BrepSolid.compound([body, patch]);
    final offset = body.edges.length;
    BrepLoop remap(BrepLoop l, bool reverse) => BrepLoop([
      for (final c in reverse ? l.coedges.reversed : l.coedges)
        BrepCoedge(c.edge + offset, reversed: c.reversed != reverse),
    ]);
    final faces = <BrepFace>[];
    for (final f in body.faces) {
      if (identical(f, wall)) continue;
      final replacement = replacements[f];
      final loops = List<BrepLoop>.from(f.loops);
      if (replacement != null) {
        final cap = caps[replacement.$2]!;
        final reverse = f.outwardNormal(0, 0).dot(cap.outwardNormal(0, 0)) < 0;
        loops[replacement.$1] = remap(cap.loops[1], reverse);
      }
      faces.add(
        BrepFace(
          surface: f.surface,
          loops: loops,
          reversed: f.reversed,
          origin: f.origin,
        ),
      );
    }
    for (var i = 0; i < patch.faces.length; i++) {
      final f = patch.faces[i];
      final surface = f.surface;
      final outerWall =
          surface is BrepCylinder &&
          (surface.radius - (majorRadius + pitch)).abs() < 1e-7;
      if (!caps.values.contains(f) && !outerWall)
        faces.add(joined.faces[body.faces.length + i]);
    }
    final result = BrepSolid(
      vertices: joined.vertices,
      edges: joined.edges,
      faces: faces,
    );
    return BrepHeal.topologyIsClosed(result) ? result : null;
  }

  static BrepSolid? tryBuild(
    BrepSolid body, {
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double pitch,
    required double lo,
    required double hi,
    required List<(double, double)> regions,
    required bool internal,
    required bool rightHand,
    required int starts,
  }) {
    final cylinders = body.faces
        .where((f) => f.surface is BrepCylinder)
        .toList();
    final planes = body.faces.where((f) => f.surface is BrepPlane).toList();
    if (planes.length != 2 ||
        cylinders.isEmpty ||
        cylinders.length > 2 ||
        planes.length + cylinders.length != body.faces.length ||
        body.edges.any((e) => e.curve is! BrepLine && e.curve is! BrepCircle))
      return null;
    final n = axis.normalized(), x = brepFrameU(n), y = n.cross(x);
    final caps = <double>[];
    for (final face in planes) {
      final p = face.surface as BrepPlane;
      if (p.normal.cross(n).length > 1e-8 ||
          face.loops.any(
            (l) =>
                l.coedges.any((c) => body.edges[c.edge].curve is! BrepCircle),
          ))
        return null;
      caps.add((p.origin - origin).dot(n));
    }
    caps.sort();
    if ((caps.first - lo).abs() > 1e-7 || (caps.last - hi).abs() > 1e-7)
      return null;
    final radii = <double>[];
    for (final face in cylinders) {
      final c = face.surface as BrepCylinder;
      if (c.axis.cross(n).length > 1e-8 ||
          (c.origin - origin).cross(n).length > 1e-7)
        return null;
      radii.add(c.radius);
    }
    radii.sort();
    final outer = radii.last, inner = radii.length == 2 ? radii.first : 0.0;
    if ((!internal && (radius - outer).abs() > 1e-7) ||
        internal && (inner == 0 || radius > inner + 1e-7))
      return null;
    final form = ThreadForm(pitch: pitch, internal: internal);
    if (internal && inner > radius + form.depth + 1e-7) return null;
    if ((hi - lo) / pitch + starts > 800)
      throw BrepUnsupported('Thread sweep exceeds the current face budget');
    // A nominal bore is a modeling convention: form the female crests inward
    // from its major diameter. A smaller, predrilled bore still cuts outward.
    final nominalBore = internal && (inner - radius - form.depth).abs() <= 1e-7;
    final wallRadius = internal && !nominalBore ? inner : radius;
    final plainRadius = internal ? inner : radius;
    final depthAmount = internal && !nominalBore
        ? radius + form.depth - inner
        : form.depth;
    final mouth = form.rootHalfWidth + depthAmount / math.sqrt(3);
    if (depthAmount <= 1e-7) return null;
    if (internal && radius + form.depth >= outer - 1e-7 ||
        !internal && outer - form.depth <= inner + 1e-7)
      return null;
    final b = BrepBuilder();
    final uses = <int, List<BrepCoedge>>{}, byPair = <(int, int), List<int>>{};
    int vertex(Vector3 p) {
      for (var i = 0; i < b.vertices.length; i++) {
        if ((b.vertices[i] - p).length < 1e-7) return i;
      }
      return b.vertex(p);
    }

    BrepCoedge edge(BrepCurve c) {
      final backward = c.t1 < c.t0;
      if (backward) {
        c = switch (c) {
          BrepHelix() => c.trimmed(c.t1, c.t0),
          BrepPlaneSectionCurve() => c.trimmed(c.t1, c.t0),
          BrepCircle() => BrepCircle(
            center: c.center,
            normal: c.normal,
            xDir: c.xDir,
            radius: c.radius,
            t0: c.t1,
            t1: c.t0,
          ),
          _ => throw BrepUnsupported(
            'Thread edge has an unsupported parameter direction',
          ),
        };
      }
      final i = vertex(c.evaluate(c.t0)), j = vertex(c.evaluate(c.t1));
      final candidates = byPair.putIfAbsent((
        math.min(i, j),
        math.max(i, j),
      ), () => []);
      for (final id in candidates) {
        final old = b.edges[id], reverse = old.v0 != i;
        if ([.25, .5, .75].every(
          (t) =>
              (old.curve.evaluate(
                        old.curve.t0 +
                            (old.curve.t1 - old.curve.t0) *
                                (reverse ? 1 - t : t),
                      ) -
                      c.evaluate(c.t0 + (c.t1 - c.t0) * t))
                  .length <
              1e-7,
        ))
          return BrepCoedge(id, reversed: reverse != backward);
      }
      final id = b.edge(i, j, c);
      candidates.add(id);
      return BrepCoedge(id, reversed: backward);
    }

    void face(BrepSurface s, List<BrepCoedge> loop, {bool reversed = false}) {
      final oriented = reversed
          ? [
              for (final c in loop.reversed)
                BrepCoedge(c.edge, reversed: !c.reversed),
            ]
          : loop;
      for (final c in oriented) (uses[c.edge] ??= []).add(c);
      b.face(s, [BrepLoop(oriented)], reversed: reversed);
    }

    List<Vector2> clip(List<Vector2> polygon, double Function(Vector2) signed) {
      final out = <Vector2>[];
      for (var i = 0; i < polygon.length; i++) {
        final a = polygon[i],
            d = polygon[(i + 1) % polygon.length],
            fa = signed(a),
            fb = signed(d);
        if (fa >= -1e-12) out.add(a);
        if ((fa < -1e-12 && fb > 1e-12) || (fb < -1e-12 && fa > 1e-12))
          out.add(a + (d - a) * (fa / (fa - fb)));
      }
      return out;
    }

    final lead = pitch * starts * (rightHand ? 1 : -1),
        h = lead / (2 * math.pi);
    for (final region in regions) {
      for (var sector = 0; sector < 4; sector++) {
        final a = sector * math.pi / 2, d = (sector + 1) * math.pi / 2;
        final qlo = region.$1 - math.max(h * a, h * d) - lo,
            qhi = region.$2 - math.min(h * a, h * d) - lo;
        final first = (qlo / pitch - .5).floor(),
            last = (qhi / pitch + .5).ceil();
        for (var turn = first; turn <= last; turn++) {
          final q = [
            -pitch / 2,
            -mouth,
            -form.rootHalfWidth,
            form.rootHalfWidth,
            mouth,
            pitch / 2,
          ];
          final depth = [0.0, 0.0, depthAmount, depthAmount, 0.0, 0.0];
          for (var band = 0; band < 5; band++) {
            final za = lo + turn * pitch + q[band],
                zb = lo + turn * pitch + q[band + 1];
            final ra = wallRadius + (internal ? 1 : -1) * depth[band],
                rb = wallRadius + (internal ? 1 : -1) * depth[band + 1];
            final lower = BrepHelix(
              origin: origin + n * za,
              axis: n,
              xDir: x,
              radius: ra,
              lead: lead,
              t0: a,
              t1: d,
            );
            final upper = BrepHelix(
              origin: origin + n * zb,
              axis: n,
              xDir: x,
              radius: rb,
              lead: lead,
              t0: a,
              t1: d,
            );
            final s = BrepRuledSurface(lower, upper);
            double axial(Vector2 p) =>
                h * (a + (d - a) * p.x) + za + (zb - za) * p.y;
            var poly = [
              Vector2(0, 0),
              Vector2(1, 0),
              Vector2(1, 1),
              Vector2(0, 1),
            ];
            poly = clip(poly, (p) => axial(p) - region.$1);
            poly = clip(poly, (p) => region.$2 - axial(p));
            if (poly.length < 3) continue;
            final co = <BrepCoedge>[];
            for (var k = 0; k < poly.length; k++) {
              final p = poly[k], r = poly[(k + 1) % poly.length];
              if ((p - r).length < 1e-10) continue;
              BrepCurve c;
              if ((p.y - r.y).abs() < 1e-9 &&
                  (p.y.abs() < 1e-9 || (p.y - 1).abs() < 1e-9)) {
                c = (p.y < .5 ? lower : upper).trimmed(
                  a + (d - a) * p.x,
                  a + (d - a) * r.x,
                );
              } else if ((p.x - r.x).abs() < 1e-9) {
                c = BrepLine(s.evaluate(p.x, p.y), s.evaluate(r.x, r.y));
              } else {
                c = BrepPlaneSectionCurve(
                  surface: s,
                  plane: BrepPlane(origin: origin + n * axial(p), normal: n),
                  t0: p.x,
                  t1: r.x,
                );
              }
              if ((c.evaluate(c.t0) - c.evaluate(c.t1)).length > 1e-8)
                co.add(edge(c));
            }
            if (co.length >= 3) face(s, co, reversed: internal);
          }
        }
      }
    }
    // Split the remaining circular walls at every existing endpoint. This
    // makes the thread/plain transitions share real edges instead of T-joints.
    void plain(double r, double low, double high, bool inward) {
      final cuts = <double>[
        0,
        math.pi / 2,
        math.pi,
        3 * math.pi / 2,
        2 * math.pi,
      ];
      for (final p in b.vertices) {
        final delta = p - origin, z = delta.dot(n);
        if ((z - low).abs() > 1e-7 && (z - high).abs() > 1e-7) continue;
        if (((delta - n * z).length - r).abs() > 1e-7) continue;
        var t = math.atan2(delta.dot(y), delta.dot(x));
        if (t < 0) t += 2 * math.pi;
        if (t > 1e-8 && t < 2 * math.pi - 1e-8) cuts.add(t);
      }
      cuts.sort();
      final angles = <double>[];
      for (final t in cuts)
        if (angles.isEmpty || t - angles.last > 1e-8) angles.add(t);
      final cylinder = BrepCylinder(
        origin: origin,
        axis: n,
        xDir: x,
        radius: r,
      );
      for (var i = 1; i < angles.length; i++) {
        final a = angles[i - 1], d = angles[i];
        face(cylinder, [
          edge(
            BrepCircle(
              center: origin + n * low,
              normal: n,
              xDir: x,
              radius: r,
              t0: a,
              t1: d,
            ),
          ),
          edge(BrepLine(cylinder.evaluate(d, low), cylinder.evaluate(d, high))),
          edge(
            BrepCircle(
              center: origin + n * high,
              normal: n,
              xDir: x,
              radius: r,
              t0: d,
              t1: a,
            ),
          ),
          edge(BrepLine(cylinder.evaluate(a, high), cylinder.evaluate(a, low))),
        ], reversed: inward);
      }
    }

    var at = lo;
    for (final region in regions) {
      if (region.$1 - at > 1e-8) plain(plainRadius, at, region.$1, internal);
      at = region.$2;
    }
    if (hi - at > 1e-8) plain(plainRadius, at, hi, internal);
    if (internal) plain(outer, lo, hi, false);
    if (!internal && inner > 0) plain(inner, lo, hi, true);
    final pending = <BrepCoedge>[
      for (final entry in uses.entries)
        if (entry.value.length == 1)
          BrepCoedge(entry.key, reversed: !entry.value.single.reversed),
    ];
    int start(BrepCoedge c) =>
        c.reversed ? b.edges[c.edge].v1 : b.edges[c.edge].v0;
    int end(BrepCoedge c) =>
        c.reversed ? b.edges[c.edge].v0 : b.edges[c.edge].v1;
    final capLoops = <(double, List<BrepLoop>)>[];
    while (pending.isNotEmpty) {
      final loop = <BrepCoedge>[pending.removeLast()],
          first = start(loop.first);
      while (end(loop.last) != first) {
        final i = pending.indexWhere((c) => start(c) == end(loop.last));
        if (i < 0)
          throw BrepUnsupported(
            'Thread boundary could not close an axial transition',
          );
        loop.add(pending.removeAt(i));
      }
      final z = (b.vertices[first] - origin).dot(n);
      if (loop.any(
        (c) => ((b.vertices[start(c)] - origin).dot(n) - z).abs() > 1e-7,
      )) {
        throw BrepUnsupported(
          'Thread boundary has an unpaired longitudinal seam',
        );
      }
      final i = capLoops.indexWhere((c) => (c.$1 - z).abs() < 1e-7);
      if (i < 0)
        capLoops.add((z, [BrepLoop(loop)]));
      else
        capLoops[i].$2.add(BrepLoop(loop));
    }
    for (final cap in capLoops) {
      final z = cap.$1;
      // The pending-edge walk can discover the bore before the outer rim.
      // Planar faces require the enclosing loop first, followed by holes.
      // These caps are coaxial, and the wall-depth guard above guarantees
      // that their radial ranges do not cross.
      double radialExtent(BrepLoop loop) => loop.coedges.fold(0.0, (r, co) {
        final delta = b.vertices[start(co)] - origin;
        return math.max(r, (delta - n * delta.dot(n)).length2);
      });
      cap.$2.sort((a, b) => radialExtent(b).compareTo(radialExtent(a)));
      // Cut grooves face into the removed material. Nominal female threads
      // add crests, so their internal axial transition faces point outward.
      final sign = (z - lo).abs() < 1e-7
          ? -1
          : (z - hi).abs() < 1e-7
          ? 1
          : regions.any((r) => (r.$1 - z).abs() < 1e-7)
          ? (nominalBore ? -1 : 1)
          : (nominalBore ? 1 : -1);
      b.face(
        BrepPlane(origin: origin + n * z, normal: n * sign.toDouble()),
        cap.$2,
      );
    }
    // Separate groove ends can form disjoint planar islands (multi-start
    // threads). They are separate faces, not holes in the first end patch.
    final result = BrepHeal.splitPlanarIslands(b.build());
    if (!BrepHeal.topologyIsClosed(result))
      throw BrepUnsupported('Thread boundary is not a closed manifold');
    return result;
  }
}
