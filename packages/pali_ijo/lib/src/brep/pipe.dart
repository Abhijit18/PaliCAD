import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import 'extrude.dart';
import 'geom.dart';
import 'heal.dart';
import 'primitives.dart';
import 'solid.dart';

part 'loft.dart';

/// Circular pipe as cylinders and torus rings, without a mesh skin.
class BrepPipe {
  BrepPipe._();

  static BrepSolid run(
    List<Vector3> path, {
    required double radius,
    double? innerRadius,
    bool closed = false,
  }) {
    if (innerRadius != null &&
        (!innerRadius.isFinite || innerRadius < 0 || innerRadius >= radius)) {
      throw BrepUnsupported(
        'Pipe inner radius must be smaller than its outer radius',
      );
    }
    final outer = _run(path, radius: radius, closed: closed);
    if (innerRadius == null || innerRadius == 0) return outer;
    final b = BrepBuilder();
    final outside = <int, int>{}, inside = <int, int>{};
    for (var i = 0; i < outer.edges.length; i++) {
      final c = outer.edges[i].curve as BrepCircle;
      final small = BrepCircle(
        center: c.center,
        normal: c.normal,
        radius: innerRadius,
        xDir: c.xDir,
        t0: c.t0,
        t1: c.t1,
      );
      final ov = b.vertex(c.evaluate(c.t0)),
          iv = b.vertex(small.evaluate(small.t0));
      outside[i] = b.edge(ov, ov, c);
      inside[i] = b.edge(iv, iv, small);
    }
    List<BrepLoop> loops(BrepFace face, Map<int, int> map, bool reverse) => [
      for (final l in face.loops)
        BrepLoop([
          for (final co in reverse ? l.coedges.reversed : l.coedges)
            BrepCoedge(map[co.edge]!, reversed: co.reversed != reverse),
        ]),
    ];
    for (final face in outer.faces) {
      final s = face.surface;
      if (s is BrepPlane) {
        b.face(s, [
          ...loops(face, outside, false),
          ...loops(face, inside, true),
        ], reversed: face.reversed);
        continue;
      }
      b.face(s, loops(face, outside, false), reversed: face.reversed);
      final inner = s is BrepCylinder
          ? BrepCylinder(
              origin: s.origin,
              axis: s.axis,
              radius: innerRadius,
              xDir: s.xDir,
            )
          : BrepTorus(
              center: (s as BrepTorus).center,
              axis: s.axis,
              majorRadius: s.majorRadius,
              minorRadius: innerRadius,
              xDir: s.xDir,
            );
      b.face(inner, loops(face, inside, true), reversed: !face.reversed);
    }
    return b.build();
  }

  /// A circular tube along [path].
  ///
  /// Straight runs are one cylinder. A circular spine is one torus (closed)
  /// or a torus sector (open arc). A polyline is cylinders sewn to torus
  /// elbows at the corners — not overlapping-cylinder union.
  static BrepSolid _run(
    List<Vector3> path, {
    required double radius,
    bool closed = false,
  }) {
    if (!radius.isFinite ||
        radius <= 0 ||
        path.any((p) => ![p.x, p.y, p.z].every((v) => v.isFinite))) {
      throw BrepUnsupported('A pipe needs a positive radius');
    }
    final pts = _dedupe(path, closed: closed);
    if (pts.length < 2) {
      throw BrepUnsupported('A pipe needs at least two points');
    }
    final inferredClosed =
        closed || (pts.length >= 4 && (pts.first - pts.last).length <= 1e-8);
    if (inferredClosed && (pts.first - pts.last).length <= 1e-8)
      pts.removeLast();
    if (inferredClosed && pts.length < 3) {
      throw BrepUnsupported('A closed pipe needs at least three points');
    }
    for (
      var i = inferredClosed ? 0 : 1;
      i < (inferredClosed ? pts.length : pts.length - 1);
      i++
    ) {
      final a = (pts[i] - pts[(i - 1 + pts.length) % pts.length]).normalized(),
          b = (pts[(i + 1) % pts.length] - pts[i]).normalized();
      if (a.dot(b) < -1 + 1e-8)
        throw BrepUnsupported('Pipe path reverses direction');
    }
    if (!inferredClosed && (_colinear(pts) || pts.length == 2)) {
      final d = pts.last - pts.first;
      if (d.length < 1e-9) {
        throw BrepUnsupported('Pipe segments collapsed');
      }
      return _sewSpans(
        [
          _PipeSpan.cyl(
            origin: pts.first,
            axis: d.normalized(),
            height: d.length,
            xDir: brepFrameU(d.normalized()),
          ),
        ],
        radius,
        closed: false,
      );
    }
    final torus = _tryCircularSpine(pts, radius, closed: inferredClosed);
    if (torus != null) return torus;

    final simple = _mergeColinear(pts, closed: inferredClosed);
    if (!inferredClosed && simple.length == 2) {
      final d = simple.last - simple.first;
      return _sewSpans(
        [
          _PipeSpan.cyl(
            origin: simple.first,
            axis: d.normalized(),
            height: d.length,
            xDir: brepFrameU(d.normalized()),
          ),
        ],
        radius,
        closed: false,
      );
    }
    return _sewPolyline(simple, radius, closed: inferredClosed);
  }

  static List<Vector3> _dedupe(List<Vector3> path, {required bool closed}) {
    final out = <Vector3>[];
    for (final p in path) {
      if (out.isEmpty || (out.last - p).length > 1e-8) out.add(p.clone());
    }
    if (closed && out.length >= 2 && (out.first - out.last).length <= 1e-8) {
      out.removeLast();
    }
    return out;
  }

  static bool _colinear(List<Vector3> pts) {
    if (pts.length <= 2) return true;
    final d = pts.last - pts.first;
    if (d.length < 1e-9) return false;
    final n = d.normalized();
    for (final p in pts) {
      final q = p - pts.first;
      if ((q - n * q.dot(n)).length > 1e-6) return false;
    }
    return true;
  }

  /// Drop vertices whose incoming/outgoing tangents are still colinear.
  ///
  /// ~1° keeps tessellated arcs intact so [_tryCircularSpine] can run first;
  /// only true straight runs collapse.
  static List<Vector3> _mergeColinear(
    List<Vector3> pts, {
    required bool closed,
  }) {
    if (pts.length < 3) return [for (final p in pts) p.clone()];
    const minTurn = 0.02;
    final n = pts.length;
    Vector3 at(int i) => pts[(i % n + n) % n];
    final keep = <Vector3>[];
    for (var i = 0; i < n; i++) {
      if (!closed && (i == 0 || i == n - 1)) {
        keep.add(pts[i].clone());
        continue;
      }
      final u = at(i) - at(i - 1);
      final v = at(i + 1) - at(i);
      if (u.length < 1e-9 || v.length < 1e-9) continue;
      final phi = math.acos(
        (u.normalized().dot(v.normalized())).clamp(-1.0, 1.0),
      );
      if (phi >= minTurn) keep.add(pts[i].clone());
    }
    if (keep.length < 2) {
      return [pts.first.clone(), pts.last.clone()];
    }
    return keep;
  }

  static BrepSolid? _tryCircularSpine(
    List<Vector3> pts,
    double radius, {
    required bool closed,
  }) {
    if (pts.length < 8) return null;
    final ring = <Vector3>[for (final p in pts) p.clone()];
    if (closed && ring.length >= 2 && (ring.first - ring.last).length <= 1e-3) {
      ring.removeLast();
    }
    if (ring.length < 8) return null;
    final fit = _fitSpineCircle(ring);
    if (fit == null) return null;
    if (fit.radius <= radius + 1e-6) return null;

    final az = _azimuths(ring, fit.center, fit.axis);
    if (az.length < 8) return null;
    final span = _azimuthCover(az);
    final step = (ring[1] - ring[0]).length;
    final gap = (ring.first - ring.last).length;
    final almostFull =
        closed ||
        (span > math.pi * 1.5 &&
            (gap - step).abs() < math.max(1e-7, step * 1e-6) &&
            List.generate(
              ring.length - 1,
              (i) => (ring[i + 1] - ring[i]).length,
            ).every(
              (length) => (length - step).abs() < math.max(1e-7, step * 1e-6),
            ));

    if (almostFull) {
      return BrepPrimitives.torus(
        center: fit.center,
        axis: fit.axis,
        majorRadius: fit.radius,
        minorRadius: radius,
      );
    }
    if (span < 1e-3) return null;
    final xDir = _radialAt(ring.first, fit.center, fit.axis);
    if (xDir == null) return null;
    final sweep = _signedSweep(ring, fit.center, fit.axis, xDir);
    if (sweep.abs() < 1e-3) return null;
    return _torusSector(
      center: fit.center,
      axis: fit.axis,
      xDir: xDir,
      major: fit.radius,
      minor: radius,
      sweep: sweep,
    );
  }

  static ({Vector3 center, Vector3 axis, double radius})? _fitSpineCircle(
    List<Vector3> pts,
  ) {
    final a = pts.first;
    final b = pts.reduce((p, q) => (p - a).length2 > (q - a).length2 ? p : q);
    final u = b - a;
    final c = pts.reduce(
      (p, q) => u.cross(p - a).length2 > u.cross(q - a).length2 ? p : q,
    );
    final v = c - a;
    var normal = u.cross(v);
    final den = 2 * normal.length2;
    if (den < 1e-20) return null;
    final center =
        a + (v.cross(normal) * u.length2 + normal.cross(u) * v.length2) / den;
    final radius = (a - center).length;
    normal.normalize();
    var winding = Vector3.zero();
    for (var i = 1; i < pts.length; i++) {
      winding += (pts[i - 1] - center).cross(pts[i] - center);
    }
    if (winding.dot(normal) < 0) normal = -normal;
    final tolerance = math.max(1e-7, radius * 1e-7);
    for (final p in pts) {
      if ((p - center).dot(normal).abs() > tolerance ||
          ((p - center).length - radius).abs() > tolerance)
        return null;
    }
    return (center: center, axis: normal, radius: radius);
  }

  static List<double> _azimuths(List<Vector3> pts, Vector3 c, Vector3 axis) {
    final t = axis.normalized();
    var u = pts.first - c;
    u = u - t * u.dot(t);
    if (u.length < 1e-10) return const [];
    u.normalize();
    final v = t.cross(u).normalized();
    final az = <double>[];
    for (final p in pts) {
      final d = p - c;
      final rad = d - t * d.dot(t);
      if (rad.length < 1e-12) continue;
      var a = math.atan2(rad.dot(v), rad.dot(u));
      if (a < 0) a += math.pi * 2;
      az.add(a);
    }
    az.sort();
    return az;
  }

  static double _azimuthCover(List<double> az) {
    if (az.length < 2) return 0;
    var maxGap = 0.0;
    for (var i = 0; i < az.length; i++) {
      final next = i + 1 < az.length ? az[i + 1] : az.first + math.pi * 2;
      maxGap = math.max(maxGap, next - az[i]);
    }
    return math.pi * 2 - maxGap;
  }

  static Vector3? _radialAt(Vector3 p, Vector3 c, Vector3 axis) {
    final t = axis.normalized();
    var u = p - c;
    u = u - t * u.dot(t);
    if (u.length < 1e-10) return null;
    return u.normalized();
  }

  static double _signedSweep(
    List<Vector3> pts,
    Vector3 c,
    Vector3 axis,
    Vector3 xDir,
  ) {
    final n = axis.normalized();
    final yDir = n.cross(xDir).normalized();
    double az(Vector3 p) {
      final d = p - c;
      final rad = d - n * d.dot(n);
      return math.atan2(rad.dot(yDir), rad.dot(xDir));
    }

    var sweep = 0.0;
    var prev = az(pts.first);
    for (var i = 1; i < pts.length; i++) {
      var a = az(pts[i]);
      var d = a - prev;
      while (d > math.pi) {
        d -= math.pi * 2;
      }
      while (d < -math.pi) {
        d += math.pi * 2;
      }
      sweep += d;
      prev = a;
    }
    return sweep;
  }

  static BrepSolid _sewPolyline(
    List<Vector3> pts,
    double radius, {
    required bool closed,
  }) {
    final spans = _spansFromPolyline(pts, radius, closed: closed);
    if (spans.isEmpty) {
      throw BrepUnsupported('Pipe segments collapsed');
    }
    return _sewSpans(spans, radius, closed: closed);
  }

  static List<_PipeSpan> _spansFromPolyline(
    List<Vector3> pts,
    double radius, {
    required bool closed,
  }) {
    final n = pts.length;
    if (n < 2) return const [];
    Vector3 at(int i) => pts[(i % n + n) % n];
    final spans = <_PipeSpan>[];

    void addCyl(Vector3 a, Vector3 b) {
      final d = b - a;
      if (d.length < 1e-6) return;
      final axis = d.normalized();
      spans.add(
        _PipeSpan.cyl(
          origin: a.clone(),
          axis: axis,
          height: d.length,
          xDir: brepFrameU(axis),
        ),
      );
    }

    final initialBend = closed
        ? _cornerBend(at(n - 1), at(0), at(1), radius)
        : null;
    var cursor = (initialBend?.t1 ?? at(0)).clone();
    final lastCorner = closed ? n + 1 : n - 1;
    const firstCorner = 1;
    for (var i = firstCorner; i < lastCorner; i++) {
      final prev = at(i - 1);
      final cur = at(i);
      final next = at(i + 1);
      final bend = _cornerBend(prev, cur, next, radius);
      if (bend == null) {
        addCyl(cursor, cur);
        cursor = cur.clone();
        continue;
      }
      addCyl(cursor, bend.t0);
      spans.add(bend.span);
      cursor = bend.t1.clone();
    }
    if (!closed) addCyl(cursor, at(n - 1));
    return spans;
  }

  static ({Vector3 t0, Vector3 t1, _PipeSpan span})? _cornerBend(
    Vector3 prev,
    Vector3 cur,
    Vector3 next,
    double radius,
  ) {
    final incoming = cur - prev;
    final outgoing = next - cur;
    final inLen = incoming.length;
    final outLen = outgoing.length;
    if (inLen < 1e-9 || outLen < 1e-9) return null;
    final u = incoming / inLen;
    final v = outgoing / outLen;
    final phi = math.acos(u.dot(v).clamp(-1.0, 1.0));
    if (phi < 0.02) return null;
    final tanHalf = math.tan(phi / 2);
    if (tanHalf.abs() < 1e-9) return null;
    var r = radius * 2;
    var inset = r * tanHalf;
    final maxInset = math.min(inLen, outLen) * 0.45;
    if (inset > maxInset) {
      r = maxInset / tanHalf;
      inset = maxInset;
    }
    if (r <= radius * (1 + 1e-6) || inset < 1e-6) {
      throw BrepUnsupported('Pipe bend is too tight for this radius');
    }
    final axis = u.cross(v);
    if (axis.length2 < 1e-20) return null;
    final nHat = axis.normalized();
    final t0 = cur - u * inset;
    final t1 = cur + v * inset;
    final inward = nHat.cross(u);
    if (inward.length2 < 1e-20) return null;
    final center = t0 + inward.normalized() * r;
    final xDir = t0 - center;
    if (xDir.length < 1e-10) return null;
    final x = xDir.normalized();
    final a0 = t0 - center;
    final a1 = t1 - center;
    var sweep = math.atan2(nHat.dot(a0.cross(a1)), a0.dot(a1));
    if (sweep.abs() < 1e-6) return null;
    var axisOut = nHat;
    var sweepOut = sweep;
    if (sweepOut < 0) {
      axisOut = -nHat;
      sweepOut = -sweep;
    }
    return (
      t0: t0.clone(),
      t1: t1.clone(),
      span: _PipeSpan.tor(
        center: center.clone(),
        axis: axisOut,
        xDir: x,
        major: r,
        sweep: sweepOut,
      ),
    );
  }

  static BrepSolid _torusSector({
    required Vector3 center,
    required Vector3 axis,
    required Vector3 xDir,
    required double major,
    required double minor,
    required double sweep,
  }) {
    var ax = axis.normalized();
    var x = xDir.normalized();
    var sw = sweep;
    if (sw < 0) {
      ax = -ax;
      sw = -sw;
    }
    return _sewSpans(
      [
        _PipeSpan.tor(
          center: center.clone(),
          axis: ax,
          xDir: x,
          major: major,
          sweep: sw,
        ),
      ],
      minor,
      closed: false,
    );
  }

  static BrepSolid _sewSpans(
    List<_PipeSpan> spans,
    double radius, {
    required bool closed,
  }) {
    spans = [
      for (final span in spans)
        if (span.isCyl)
          span
        else ...[
          for (var i = 0; i < (span.sweep! / (math.pi / 2)).ceil(); i++)
            () {
              final count = (span.sweep! / (math.pi / 2)).ceil();
              final u = span.sweep! * i / count;
              return _PipeSpan.tor(
                center: span.center!,
                axis: span.axis!,
                xDir:
                    span.xDir! * math.cos(u) +
                    span.axis!.cross(span.xDir!) * math.sin(u),
                major: span.major!,
                sweep: span.sweep! / count,
              );
            }(),
        ],
    ];
    final b = BrepBuilder();
    int makeCircle(Vector3 center, Vector3 normal, Vector3 xDir) {
      final v = b.vertex(center + xDir.normalized() * radius);
      return b.edge(
        v,
        v,
        BrepCircle(
          center: center.clone(),
          normal: normal.normalized(),
          radius: radius,
          xDir: xDir,
        ),
      );
    }

    ({Vector3 center, Vector3 normal, Vector3 xDir}) jointOf(
      _PipeSpan span, {
      required bool start,
    }) {
      if (span.isCyl) {
        final n = span.axis!;
        final c = start ? span.origin! : span.origin! + n * span.height!;
        return (center: c, normal: n, xDir: span.xDir!);
      }
      final torus = BrepTorus(
        center: span.center!,
        axis: span.axis!,
        majorRadius: span.major!,
        minorRadius: radius,
        xDir: span.xDir!,
      );
      final u = start ? 0.0 : span.sweep!;
      final cr = torus.xDir * math.cos(u) + torus.yDir * math.sin(u);
      final tangent = -torus.xDir * math.sin(u) + torus.yDir * math.cos(u);
      return (
        center: span.center! + cr * span.major!,
        normal: tangent,
        xDir: torus.axis,
      );
    }

    final startJ = jointOf(spans.first, start: true);
    var prevId = makeCircle(startJ.center, startJ.normal, startJ.xDir);
    var prevCenter = startJ.center;
    var prevNormal = startJ.normal;
    var prevX = startJ.xDir;
    final firstId = prevId;
    final firstCenter = prevCenter;
    final firstNormal = prevNormal;
    final firstX = prevX;

    if (!closed) {
      b.face(BrepPlane(origin: prevCenter, normal: -prevNormal, xDir: prevX), [
        BrepLoop([BrepCoedge(prevId, reversed: true)]),
      ]);
    }

    for (var i = 0; i < spans.length; i++) {
      final span = spans[i];
      final isLast = i == spans.length - 1;
      late final int endId;
      late final Vector3 endCenter;
      late final Vector3 endNormal;
      late final Vector3 endX;
      if (isLast && closed) {
        endId = firstId;
        endCenter = firstCenter;
        endNormal = firstNormal;
        endX = firstX;
      } else {
        final j = jointOf(span, start: false);
        endId = makeCircle(j.center, j.normal, j.xDir);
        endCenter = j.center;
        endNormal = j.normal;
        endX = j.xDir;
      }
      if (span.isCyl) {
        final origin = span.origin!;
        final axis = span.axis!;
        b.face(
          BrepCylinder(
            origin: origin,
            axis: axis,
            radius: radius,
            xDir: span.xDir!,
          ),
          [
            BrepLoop([BrepCoedge(prevId)]),
            BrepLoop([BrepCoedge(endId, reversed: true)]),
          ],
        );
      } else {
        b.face(
          BrepTorus(
            center: span.center!,
            axis: span.axis!,
            majorRadius: span.major!,
            minorRadius: radius,
            xDir: span.xDir!,
          ),
          [
            BrepLoop([BrepCoedge(prevId)]),
            BrepLoop([BrepCoedge(endId, reversed: true)]),
          ],
        );
      }
      prevId = endId;
      prevCenter = endCenter;
      prevNormal = endNormal;
      prevX = endX;
    }

    if (!closed) {
      b.face(BrepPlane(origin: prevCenter, normal: prevNormal, xDir: prevX), [
        BrepLoop([BrepCoedge(prevId)]),
      ]);
    }
    final solid = b.build();
    if (!BrepHeal.topologyIsClosed(solid)) {
      throw BrepUnsupported('Pipe sweep was not a closed solid');
    }
    return solid;
  }
}

class _PipeSpan {
  _PipeSpan.cyl({
    required Vector3 origin,
    required Vector3 axis,
    required this.height,
    required Vector3 xDir,
  }) : isCyl = true,
       origin = origin.clone(),
       axis = axis.clone(),
       xDir = xDir.clone(),
       center = null,
       major = null,
       sweep = null;

  _PipeSpan.tor({
    required Vector3 center,
    required Vector3 axis,
    required Vector3 xDir,
    required this.major,
    required this.sweep,
  }) : isCyl = false,
       origin = null,
       height = null,
       center = center.clone(),
       axis = axis.clone(),
       xDir = xDir.clone();

  final bool isCyl;
  final Vector3? origin;
  final Vector3? axis;
  final double? height;
  final Vector3? xDir;
  final Vector3? center;
  final double? major;
  final double? sweep;
}

/// Two profiles → cylinder, frustum, or translated prism when they allow it.
class BrepLoft {
  BrepLoft._();

  static BrepSolid? tryProfiles(
    List<Vector3> a,
    List<Vector3> b, {
    List<(List<Vector3>, List<Vector3>)> holes = const [],
  }) {
    _LoftRing(a);
    _LoftRing(b);
    if (holes.isEmpty) {
      return tryCircles(a, b) ?? tryExtrude(a, b) ?? _LoftBuilder.run([a, b]);
    }
    return _LoftBuilder.run([a, b], holes: holes);
  }

  /// A sewn, capped skin through parallel planar sections. Interior stations
  /// share edges; they do not leave coincident caps inside the result.
  static BrepSolid sections(
    List<List<Vector3>> sections, {
    List<(List<Vector3>, List<Vector3>)> holes = const [],
  }) {
    if (sections.length == 2) {
      return tryProfiles(sections[0], sections[1], holes: holes)!;
    }
    return _LoftBuilder.run(sections, holes: holes);
  }

  /// Two circular profiles → cylinder or frustum, when they share an axis.
  static BrepSolid? tryCircles(List<Vector3> a, List<Vector3> b) {
    final ca = _fitCircle(a);
    final cb = _fitCircle(b);
    if (ca == null || cb == null) return null;
    final axis = cb.center - ca.center;
    if (axis.length < 1e-6) return null;
    final n = axis.normalized();
    if ((ca.normal.cross(n)).length > 1e-8 &&
        (ca.normal.cross(-n)).length > 1e-8) {
      return null;
    }
    if ((cb.normal.cross(n)).length > 1e-8 &&
        (cb.normal.cross(-n)).length > 1e-8) {
      return null;
    }
    if ((ca.radius - cb.radius).abs() <= 1e-6) {
      return BrepPrimitives.cylinder(
        origin: ca.center,
        axis: axis,
        radius: ca.radius,
        height: axis.length,
      );
    }
    return BrepPrimitives.frustum(
      origin: ca.center,
      axis: axis,
      radius: ca.radius,
      topRadius: cb.radius,
      height: axis.length,
    );
  }

  /// Same polygon translated → analytic prism.
  static BrepSolid? tryExtrude(List<Vector3> a, List<Vector3> b) {
    final d = _uniformTranslation(a, b);
    if (d == null) return null;
    return BrepExtrude.profile(a, d);
  }

  static Vector3? _uniformTranslation(List<Vector3> a, List<Vector3> b) {
    if (a.length != b.length || a.length < 3) return null;
    final na = _ringNormal(a);
    final nb = _ringNormal(b);
    if (na == null || nb == null) return null;
    if ((na.cross(nb)).length > 1e-8 && (na.cross(-nb)).length > 1e-8) {
      return null;
    }
    var ca = Vector3.zero();
    var cb = Vector3.zero();
    for (final p in a) {
      ca += p;
    }
    for (final p in b) {
      cb += p;
    }
    ca /= a.length.toDouble();
    cb /= b.length.toDouble();
    final d = cb - ca;
    if (d.length < 1e-6) return null;
    final tol = math.max(1e-8, d.length * 1e-10);
    if (_matchesTranslation(a, b, d, tol)) return d;
    final reversed = b.reversed.toList();
    if (_matchesTranslation(a, reversed, d, tol)) return d;
    return null;
  }

  static bool _matchesTranslation(
    List<Vector3> a,
    List<Vector3> b,
    Vector3 d,
    double tol,
  ) {
    final n = a.length;
    for (var k = 0; k < n; k++) {
      var ok = true;
      for (var i = 0; i < n; i++) {
        if ((b[(i + k) % n] - a[i] - d).length > tol) {
          ok = false;
          break;
        }
      }
      if (ok) return true;
    }
    return false;
  }

  static Vector3? _ringNormal(List<Vector3> ring) {
    var n = Vector3.zero();
    final c = ring.first;
    for (var i = 1; i < ring.length - 1; i++) {
      n += (ring[i] - c).cross(ring[i + 1] - c);
    }
    if (n.length < 1e-10) return null;
    return n.normalized();
  }

  static ({Vector3 center, Vector3 normal, double radius})? _fitCircle(
    List<Vector3> ring,
  ) {
    if (ring.length < 8) return null;
    final fit = BrepPipe._fitSpineCircle(ring);
    return fit == null
        ? null
        : (center: fit.center, normal: fit.axis, radius: fit.radius);
  }
}
