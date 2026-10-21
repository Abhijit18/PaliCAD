part of 'geom.dart';

/// Exact plane intersection of a ruled patch. The scalar parameter follows
/// the lower curve; the ruling coordinate is solved from the plane equation.
class BrepPlaneSectionCurve implements BrepCurve {
  BrepPlaneSectionCurve({
    required this.surface,
    required this.plane,
    this.t0 = 0,
    this.t1 = 1,
  });
  final BrepRuledSurface surface;
  final BrepPlane plane;
  @override
  final double t0, t1;
  double ruling(double u) {
    final a = surface.evaluate(u, 0),
        d = surface.dv(u, 0),
        den = d.dot(plane.normal);
    if (den.abs() < 1e-12 * d.length)
      throw BrepUnsupported('Plane is tangent to the loft ruling');
    return -plane.height(a) / den;
  }

  @override
  Vector3 evaluate(double u) => surface.evaluate(u, ruling(u));
  @override
  Vector3 tangent(double u) {
    final v = ruling(u), du = surface.du(u, v), dv = surface.dv(u, v);
    return (du - dv * (plane.normal.dot(du) / plane.normal.dot(dv)))
        .normalized();
  }

  @override
  double closestParam(Vector3 p) {
    final uv = surface.parameters(p),
        u = uv.$1.clamp(math.min(t0, t1), math.max(t0, t1)).toDouble();
    if ((evaluate(u) - p).length < 1e-8) return u;
    return _curveNearest(this, p, 32);
  }

  @override
  Vector3 closestPoint(Vector3 p) => evaluate(closestParam(p));
  BrepPlaneSectionCurve trimmed(double a, double b) =>
      BrepPlaneSectionCurve(surface: surface, plane: plane, t0: a, t1: b);
  @override
  Map<String, dynamic> toJson() => {
    't': 'plane_section',
    'surface': surface.toJson(),
    'plane': plane.toJson(),
    'a': t0,
    'b': t1,
  };
  factory BrepPlaneSectionCurve.fromJson(Map<String, dynamic> j) =>
      BrepPlaneSectionCurve(
        surface: BrepRuledSurface.fromJson(
          j['surface'] as Map<String, dynamic>,
        ),
        plane: BrepPlane.fromJson(j['plane'] as Map<String, dynamic>),
        t0: (j['a'] as num).toDouble(),
        t1: (j['b'] as num).toDouble(),
      );
}
