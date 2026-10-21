import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../brep/brep.dart';

const _twoPi = math.pi * 2;

/// A face's trimmed region, expressed in its surface's own (u, v) parameters.
///
/// Two shapes cover everything this kernel produces:
///
/// - [FaceDomainRect] — the region is a parameter-space rectangle. This is the
///   whole surface (a bare sphere or torus, which carry no loops at all), or a
///   patch whose rims are closed circles, like a cylinder or cone wall. The
///   seam edge on such a wall is traversed twice in opposite directions, so its
///   loop never closes in (u, v) and a rectangle is the honest reading of it.
/// - [FaceDomainLoops] — the region is bounded by loops that do close in
///   (u, v). Everything a boolean, extrude, fillet or slice leaves behind.
///
/// Both are integrated by Green's theorem or by direct quadrature; neither
/// tessellates.
sealed class FaceDomain {
  /// Whether the region is represented without approximation. False only when a
  /// trim curve on a curved surface traced a path through (u, v) that is not a
  /// straight line, which has to be sampled.
  bool get isExact;
}

class FaceDomainRect implements FaceDomain {
  const FaceDomainRect(this.u0, this.u1, this.v0, this.v1);

  final double u0;
  final double u1;
  final double v0;
  final double v1;

  @override
  bool get isExact => true;

  bool contains(double u, double v, {bool uPeriodic = false}) {
    final inV = v >= v0 - 1e-9 && v <= v1 + 1e-9;
    if (!inV) return false;
    if (u >= u0 - 1e-9 && u <= u1 + 1e-9) return true;
    if (!uPeriodic) return false;
    final shifted = u0 + (u - u0) % _twoPi;
    return shifted >= u0 - 1e-9 && shifted <= u1 + 1e-9;
  }
}

class FaceDomainLoops implements FaceDomain {
  FaceDomainLoops(this.loops, {required this.isExact});

  /// Closed polylines / arcs in (u, v). Orientation is not normalised; the
  /// integrator calibrates sign from the area, which must come out positive.
  final List<List<UvSegment>> loops;

  @override
  final bool isExact;

  bool contains(double u, double v) {
    // Even-odd crossing count over every loop at once, which handles holes
    // without needing to know which loop is which.
    var crossings = 0;
    for (final loop in loops) {
      final pts = <Vector2>[];
      for (final seg in loop) {
        const n = 24;
        for (var i = 0; i < n; i++) {
          pts.add(seg.at(i / n));
        }
      }
      for (var i = 0; i < pts.length; i++) {
        final a = pts[i];
        final b = pts[(i + 1) % pts.length];
        if ((a.y > v) == (b.y > v)) continue;
        final x = a.x + (v - a.y) / (b.y - a.y) * (b.x - a.x);
        if (x > u) crossings++;
      }
    }
    return crossings.isOdd;
  }
}

/// One boundary curve of a face, in (u, v).
sealed class UvSegment {
  /// Position at [t] in [0, 1].
  Vector2 at(double t);

  /// dv/dt at [t]. Green's theorem only ever needs the v derivative.
  double dvdt(double t);
}

class UvLine implements UvSegment {
  UvLine(this.a, this.b);

  final Vector2 a;
  final Vector2 b;

  @override
  Vector2 at(double t) => a + (b - a) * t;

  @override
  double dvdt(double t) => b.y - a.y;
}

/// `c + A·cos θ + B·sin θ` with θ sweeping [t0], [t1] as t goes 0 → 1.
///
/// A circle or ellipse edge of a *planar* face maps to exactly this, because
/// projecting onto an orthonormal plane basis is a rotation. That keeps arc
/// boundaries exact instead of polygonal.
class UvArc implements UvSegment {
  UvArc({
    required this.c,
    required this.a,
    required this.b,
    required this.t0,
    required this.t1,
  });

  final Vector2 c;
  final Vector2 a;
  final Vector2 b;
  final double t0;
  final double t1;

  double _theta(double t) => t0 + (t1 - t0) * t;

  @override
  Vector2 at(double t) {
    final th = _theta(t);
    return c + a * math.cos(th) + b * math.sin(th);
  }

  @override
  double dvdt(double t) {
    final th = _theta(t);
    return (-a.y * math.sin(th) + b.y * math.cos(th)) * (t1 - t0);
  }
}

/// Resolves the (u, v) region of [face].
FaceDomain faceDomain(BrepSolid solid, BrepFace face) {
  final surface = face.surface;
  if (surface is BrepAffineSurface) {
    final base = BrepXform.unwrapAffine(solid);
    if (base == null)
      throw BrepUnsupported('Transformed face has inconsistent boundaries');
    return faceDomain(base.solid, base.solid.faces[solid.faces.indexOf(face)]);
  }
  if (surface is BrepSplineSurface) {
    if (face.loops.isNotEmpty)
      throw BrepUnsupported(
        'Trimmed spline domain requires numerical integration',
      );
    return FaceDomainRect(surface.u0, surface.u1, surface.v0, surface.v1);
  }
  if (surface is BrepRuledSurface) {
    if (face.loops.any(
      (l) => l.coedges.any(
        (c) =>
            solid.edges[c.edge].curve is BrepSurfaceIntersectionCurve ||
            solid.edges[c.edge].curve is BrepPlaneSectionCurve ||
            solid.edges[c.edge].curve is BrepRuledQuadricCurve,
      ),
    ))
      throw BrepUnsupported(
        'Trimmed ruled domain requires boundary quadrature',
      );
    return const FaceDomainRect(0, 1, 0, 1);
  }

  // No loops means the untrimmed surface. Only closed surfaces can say this.
  if (face.loops.isEmpty) {
    return switch (surface) {
      BrepSphere() => const FaceDomainRect(0, _twoPi, 0, math.pi),
      BrepTorus() => const FaceDomainRect(0, _twoPi, 0, _twoPi),
      _ => throw BrepUnsupported(
        'face on ${surface.runtimeType} has no trim loops',
      ),
    };
  }

  if (surface is BrepPlane) {
    return _planeLoops(solid, face, surface);
  }
  return _curvedDomain(solid, face, surface);
}

/// Planar faces keep their arcs. A plane's (u, v) is just its orthonormal
/// basis, so lines stay lines and circles stay circles, and the whole boundary
/// integral comes out exact.
FaceDomainLoops _planeLoops(BrepSolid solid, BrepFace face, BrepPlane plane) {
  Vector2 uv(Vector3 p) {
    final d = p - plane.origin;
    return Vector2(d.dot(plane.xDir), d.dot(plane.yDir));
  }

  final loops = <List<UvSegment>>[];
  for (final loop in face.loops) {
    final segs = <UvSegment>[];
    for (final co in loop.coedges) {
      final edge = solid.edges[co.edge];
      final curve = edge.curve;
      final start = solid.vertices[co.reversed ? edge.v1 : edge.v0];

      switch (curve) {
        case BrepHelix():
          throw BrepUnsupported('Helical trims require numerical integration');
        case BrepSurfaceIntersectionCurve():
        case BrepRuledQuadricCurve():
        case BrepPlaneSectionCurve():
        case BrepSplineCurve():
          throw BrepUnsupported(
            'Spline trim domain requires numerical integration',
          );
        case BrepAffineCurve():
          throw BrepUnsupported(
            'Transformed curves require the original face coordinate system',
          );
        case BrepIntersectionCurve():
          throw BrepUnsupported(
            'A quadric intersection cannot bound a planar face',
          );
        case BrepLine():
          // Trust the loop's start vertex over the coedge flag, matching how
          // the tessellator walks loops.
          final forward =
              (start - curve.a).length2 <= (start - curve.b).length2;
          final p0 = forward ? curve.a : curve.b;
          final p1 = forward ? curve.b : curve.a;
          segs.add(UvLine(uv(p0), uv(p1)));

        case BrepCircle():
          final cu = uv(curve.center);
          final au = uv(curve.center + curve.xDir * curve.radius) - cu;
          final bu = uv(curve.center + curve.yDir * curve.radius) - cu;
          if (curve.isClosed) {
            // A closed circle starts and ends at the same vertex, so the start
            // vertex says nothing about direction and the coedge flag is the
            // only signal. It matters: a hole's rim must run against the outer
            // loop or Green's theorem adds the hole instead of subtracting it.
            final lo = curve.t0;
            final hi = curve.t0 + _twoPi;
            segs.add(
              UvArc(
                c: cu,
                a: au,
                b: bu,
                t0: co.reversed ? hi : lo,
                t1: co.reversed ? lo : hi,
              ),
            );
          } else {
            final forward =
                (start - curve.evaluate(curve.t0)).length2 <=
                (start - curve.evaluate(curve.t1)).length2;
            segs.add(
              UvArc(
                c: cu,
                a: au,
                b: bu,
                t0: forward ? curve.t0 : curve.t1,
                t1: forward ? curve.t1 : curve.t0,
              ),
            );
          }

        case BrepEllipse():
          final cu = uv(curve.center);
          final au =
              uv(curve.center + curve.xDir.normalized() * curve.rMajor) - cu;
          final bu = uv(curve.center + curve.yDir * curve.rMinor) - cu;
          final forward =
              (start - curve.evaluate(curve.t0)).length2 <=
              (start - curve.evaluate(curve.t1)).length2;
          segs.add(
            UvArc(
              c: cu,
              a: au,
              b: bu,
              t0: forward ? curve.t0 : curve.t1,
              t1: forward ? curve.t1 : curve.t0,
            ),
          );
      }
    }
    if (segs.isNotEmpty) loops.add(segs);
  }
  return FaceDomainLoops(loops, isExact: true);
}

/// Curved surfaces have no exact (u, v) form for a general trim curve, so each
/// coedge is sampled and then tested for straightness in (u, v). Rims and seams
/// — which is almost all of them — come out as single straight segments and
/// stay exact. Anything genuinely curved in (u, v) is kept as a polyline and
/// the domain reports itself inexact.
FaceDomain _curvedDomain(BrepSolid solid, BrepFace face, BrepSurface surface) {
  const samples = 48;
  final periodicU = surface is! BrepPlane;

  var sawClosedCircle = false;
  final loops = <List<UvSegment>>[];
  var allExact = true;
  var uMin = double.infinity, uMax = double.negativeInfinity;
  var vMin = double.infinity, vMax = double.negativeInfinity;

  for (final loop in face.loops) {
    final raw = <Vector2>[];
    for (final co in loop.coedges) {
      final edge = solid.edges[co.edge];
      final curve = edge.curve;
      if (curve is BrepCircle && curve.isClosed) sawClosedCircle = true;

      final start = solid.vertices[co.reversed ? edge.v1 : edge.v0];
      final forward =
          (start - curve.evaluate(curve.t0)).length2 <=
          (start - curve.evaluate(curve.t1)).length2;
      for (var i = 0; i < samples; i++) {
        final f = i / (samples - 1);
        final t = forward
            ? curve.t0 + (curve.t1 - curve.t0) * f
            : curve.t1 - (curve.t1 - curve.t0) * f;
        final uv = surface.parameters(curve.evaluate(t));
        if (uv == null) continue;
        raw.add(Vector2(uv.$1, uv.$2));
      }
    }
    if (raw.length < 3) continue;

    if (periodicU) _unwrapU(raw);
    for (final p in raw) {
      uMin = math.min(uMin, p.x);
      uMax = math.max(uMax, p.x);
      vMin = math.min(vMin, p.y);
      vMax = math.max(vMax, p.y);
    }

    // A loop that does not return to where it started in (u, v) has crossed
    // the seam an odd number of times, or doubled back along it. Either way
    // Green's theorem cannot be applied to it.
    if ((raw.first - raw.last).length > 1e-6 &&
        (raw.first - raw.last).length > 1e-6 * (1 + uMax - uMin)) {
      return _rectFallback(solid, face, surface, vMin, vMax);
    }

    final segs = <UvSegment>[];
    for (var i = 0; i < raw.length - 1; i++) {
      segs.add(UvLine(raw[i], raw[i + 1]));
    }
    segs.add(UvLine(raw.last, raw.first));
    if (!_isPolyline(raw)) allExact = false;
    loops.add(segs);
  }

  if (sawClosedCircle || loops.isEmpty) {
    return _rectFallback(solid, face, surface, vMin, vMax);
  }
  return FaceDomainLoops(loops, isExact: allExact);
}

/// Full period in u, sampled extent in v. Correct whenever the trim really is
/// the whole revolution, which is what a closed rim means.
FaceDomain _rectFallback(
  BrepSolid solid,
  BrepFace face,
  BrepSurface surface,
  double vMin,
  double vMax,
) {
  if (surface is BrepTorus) {
    // Each closed rim winds around u at a constant v. The first rim alone
    // cannot determine the tube interval; inspect all trims, including the
    // periodic v seam at the lower fillet.
    final d = BrepTessellate.torusFaceDomain(solid, surface, face);
    if (d != null) return FaceDomainRect(d.$1, d.$2, d.$3, d.$4);
  }
  if (surface is BrepCylinder) {
    final d = BrepTessellate.cylinderFaceDomain(solid, face);
    return FaceDomainRect(d.$1, d.$2, d.$3, d.$4);
  }

  var lo = vMin;
  var hi = vMax;
  if (!lo.isFinite || !hi.isFinite || (hi - lo).abs() < 1e-12) {
    final (dLo, dHi) = switch (surface) {
      BrepSphere() => (0.0, math.pi),
      BrepTorus() => (0.0, _twoPi),
      BrepCone() => (0.0, surface.height),
      _ => (0.0, 1.0),
    };
    lo = dLo;
    hi = dHi;
  }
  return FaceDomainRect(0, _twoPi, lo, hi);
}

/// Removes 2π jumps so a loop reads as a continuous path in u.
void _unwrapU(List<Vector2> pts) {
  for (var i = 1; i < pts.length; i++) {
    var du = pts[i].x - pts[i - 1].x;
    while (du > math.pi) {
      pts[i] = Vector2(pts[i].x - _twoPi, pts[i].y);
      du = pts[i].x - pts[i - 1].x;
    }
    while (du < -math.pi) {
      pts[i] = Vector2(pts[i].x + _twoPi, pts[i].y);
      du = pts[i].x - pts[i - 1].x;
    }
  }
}

/// True when [pts] is a chain of straight runs in (u, v), which is what rims
/// (v constant) and seams (u constant) reduce to.
bool _isPolyline(List<Vector2> pts) {
  for (var i = 1; i + 1 < pts.length; i++) {
    final a = pts[i] - pts[i - 1];
    final b = pts[i + 1] - pts[i];
    final scale = a.length * b.length;
    if (scale < 1e-18) continue;
    // Non-collinear only counts against us if the turn is gentle; a genuine
    // corner between two straight runs is fine.
    final cross = (a.x * b.y - a.y * b.x).abs() / scale;
    if (cross > 1e-6 && cross < 0.2) return false;
  }
  return true;
}
