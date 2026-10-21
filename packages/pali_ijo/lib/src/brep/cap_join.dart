import 'dart:math' as math;
import 'package:vector_math/vector_math_64.dart';
import 'geom.dart';
import 'solid.dart';
import 'heal.dart';
import 'xform.dart';

/// Join matching planar caps without intersecting the remaining surfaces.
/// Only closed bodies proven to occupy opposite half-spaces are eligible.
/// Unrecognized geometry returns null for the ordinary Boolean path.
BrepSolid? joinMatchingPlanarCaps(BrepSolid a, BrepSolid b) {
  const eps = 1e-7;
  bool planar(BrepSurface s) =>
      s is BrepPlane || s is BrepAffineSurface && planar(s.base);
  if (!BrepHeal.topologyIsClosed(a) || !BrepHeal.topologyIsClosed(b)) {
    return null;
  }
  for (var ai = 0; ai < a.faces.length; ai++) {
    final fa = a.faces[ai];
    if (!planar(fa.surface) || fa.loops.length != 1) continue;
    final origin = fa.surface.evaluate(0, 0), normal = fa.outwardNormal(0, 0);
    for (var bi = 0; bi < b.faces.length; bi++) {
      final fb = b.faces[bi];
      if (!planar(fb.surface) ||
          fb.loops.length != 1 ||
          fb.outwardNormal(0, 0).dot(normal) > -1 + 1e-10 ||
          (fb.surface.evaluate(0, 0) - origin).dot(normal).abs() > eps) {
        continue;
      }
      final ac = fa.loops.single.coedges, bc = fb.loops.single.coedges;
      if (ac.isEmpty || ac.length != bc.length) continue;
      final matches = <int, (int, bool)>{};
      final used = <int>{};
      for (final cb in bc) {
        final eb = b.edges[cb.edge];
        for (final ca in ac) {
          if (used.contains(ca.edge)) continue;
          final ea = a.edges[ca.edge];
          final reverse = _sameCapCurve(ea.curve, eb.curve, eps);
          if (reverse == null || (ca.reversed == (cb.reversed != reverse))) {
            continue;
          }
          matches[cb.edge] = (ca.edge, reverse);
          used.add(ca.edge);
          break;
        }
      }
      if (matches.length != bc.length ||
          !_negativeHalfSpace(a, origin, normal, eps) ||
          !_negativeHalfSpace(b, origin, -normal, eps)) {
        continue;
      }
      final vertices = <Vector3>[...a.vertices];
      final vm = <int, int>{};
      var consistent = true;
      void mapVertex(int from, int to) {
        if ((b.vertices[from] - a.vertices[to]).length > eps ||
            vm.containsKey(from) && vm[from] != to) {
          consistent = false;
        }
        vm[from] = to;
      }

      for (final entry in matches.entries) {
        final eb = b.edges[entry.key], ea = a.edges[entry.value.$1];
        mapVertex(eb.v0, entry.value.$2 ? ea.v1 : ea.v0);
        mapVertex(eb.v1, entry.value.$2 ? ea.v0 : ea.v1);
      }
      if (!consistent) continue;
      for (var i = 0; i < b.vertices.length; i++) {
        if (!vm.containsKey(i)) {
          vm[i] = vertices.length;
          vertices.add(b.vertices[i]);
        }
      }
      final edges = <BrepEdge>[...a.edges], em = <int, int>{};
      for (var i = 0; i < b.edges.length; i++) {
        final match = matches[i];
        if (match != null) {
          em[i] = match.$1;
          continue;
        }
        final e = b.edges[i];
        em[i] = edges.length;
        edges.add(
          BrepEdge(
            v0: vm[e.v0]!,
            v1: vm[e.v1]!,
            curve: e.curve,
            origin: e.origin,
          ),
        );
      }
      final result = BrepSolid(
        vertices: vertices,
        edges: edges,
        faces: [
          for (var i = 0; i < a.faces.length; i++)
            if (i != ai) a.faces[i],
          for (var i = 0; i < b.faces.length; i++)
            if (i != bi)
              BrepFace(
                surface: b.faces[i].surface,
                reversed: b.faces[i].reversed,
                origin: b.faces[i].origin,
                loops: [
                  for (final loop in b.faces[i].loops)
                    BrepLoop([
                      for (final co in loop.coedges)
                        BrepCoedge(
                          em[co.edge]!,
                          reversed:
                              co.reversed != (matches[co.edge]?.$2 ?? false),
                        ),
                    ]),
                ],
              ),
        ],
      );
      if (BrepHeal.topologyIsClosed(result)) return result;
    }
  }
  return null;
}

// Affine images of lines and circular arcs can be compared at corresponding
// parameters. Equal angular spans exclude curves with different winding counts.
bool? _sameCapCurve(BrepCurve a, BrepCurve b, double eps) {
  BrepCurve base(BrepCurve c) => c is BrepAffineCurve ? base(c.base) : c;
  final ca = base(a), cb = base(b);
  if (!(ca is BrepLine && cb is BrepLine) &&
      !(ca is BrepCircle &&
          cb is BrepCircle &&
          ((ca.t1 - ca.t0).abs() - (cb.t1 - cb.t0).abs()).abs() < 1e-10)) {
    return null;
  }
  for (final reverse in [false, true]) {
    if ([0.0, .25, .5, .75, 1.0].every(
      (t) =>
          (a.evaluate(a.t0 + (a.t1 - a.t0) * t) -
                  b.evaluate(b.t0 + (b.t1 - b.t0) * (reverse ? 1 - t : t)))
              .length <=
          eps,
    )) {
      return reverse;
    }
  }
  return null;
}

// Conservative support bounds, never a tessellation/sample-based overlap test.
// A ruled surface lies between its rails; positive-weight splines lie within
// their control hull. Cylinders parallel to the cut normal have no interior
// axial extrema. Other surfaces defer to the general Boolean implementation.
bool _negativeHalfSpace(
  BrepSolid solid,
  Vector3 origin,
  Vector3 normal,
  double eps,
) {
  final unwrapped = BrepXform.unwrapAffine(solid);
  if (unwrapped != null) {
    return _negativeHalfSpace(
      unwrapped.solid,
      unwrapped.transform.inversePoint(origin),
      unwrapped.transform.transposeVector(normal),
      eps,
    );
  }
  double? support(BrepCurve c, Vector3 o, Vector3 n) {
    if (c is BrepAffineCurve) {
      return support(
        c.base,
        c.transform.inversePoint(o),
        c.transform.transposeVector(n),
      );
    }
    if (c is BrepLine) return math.max((c.a - o).dot(n), (c.b - o).dot(n));
    if (c is BrepCircle) {
      return (c.center - o).dot(n) + c.radius * n.cross(c.normal).length;
    }
    if (c is BrepSplineCurve && c.weights.every((w) => w > 0)) {
      return c.points.map((p) => (p - o).dot(n)).reduce(math.max);
    }
    return null;
  }

  for (final f in solid.faces) {
    final s = f.surface;
    if (s is BrepRuledSurface) {
      for (final c in [s.lower, s.upper]) {
        final h = support(c, origin, normal);
        if (h == null || h > eps) return false;
      }
    } else if (s is! BrepPlane &&
        !(s is BrepCylinder &&
            s.axis.cross(normal).length <= 1e-10 * normal.length)) {
      return false;
    }
    if (f.loops.isEmpty) return false;
    for (final loop in f.loops) {
      for (final co in loop.coedges) {
        final h = support(solid.edges[co.edge].curve, origin, normal);
        if (h == null || h > eps) return false;
      }
    }
  }
  return true;
}
