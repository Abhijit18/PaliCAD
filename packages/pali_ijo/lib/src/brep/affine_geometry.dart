part of 'geom.dart';

/// An invertible affine map. Columns are images of the three unit vectors.
/// Keeping this map exact allows ellipsoids, sheared cylinders and transformed
/// intersection curves to remain BREP geometry without a mesh conversion.
class BrepAffineTransform {
  BrepAffineTransform(Vector3 x, Vector3 y, Vector3 z, Vector3 offset)
    : x = x.clone(),
      y = y.clone(),
      z = z.clone(),
      offset = offset.clone() {
    if ([
          x,
          y,
          z,
          offset,
        ].any((v) => !v.x.isFinite || !v.y.isFinite || !v.z.isFinite) ||
        !determinant.isFinite ||
        determinant.abs() <= 1e-14 * x.length * y.length * z.length) {
      throw BrepUnsupported('Transform must be finite and invertible');
    }
  }
  final Vector3 x, y, z, offset;
  double get determinant => x.dot(y.cross(z));
  Vector3 vector(Vector3 v) => x * v.x + y * v.y + z * v.z;
  Vector3 point(Vector3 p) => vector(p) + offset;
  Vector3 transposeVector(Vector3 v) => Vector3(x.dot(v), y.dot(v), z.dot(v));
  Vector3 inverseVector(Vector3 p) =>
      Vector3(p.dot(y.cross(z)), p.dot(z.cross(x)), p.dot(x.cross(y))) /
      determinant;
  Vector3 inversePoint(Vector3 p) => inverseVector(p - offset);
  Vector3 normal(Vector3 n) =>
      ((y.cross(z) * n.x + z.cross(x) * n.y + x.cross(y) * n.z) / determinant)
          .normalized();
  BrepAffineTransform then(BrepAffineTransform next) => BrepAffineTransform(
    next.vector(x),
    next.vector(y),
    next.vector(z),
    next.point(offset),
  );
  bool sameAs(BrepAffineTransform other) =>
      (x - other.x).length2 == 0 &&
      (y - other.y).length2 == 0 &&
      (z - other.z).length2 == 0 &&
      (offset - other.offset).length2 == 0;
  List<dynamic> toJson() => [_v(x), _v(y), _v(z), _v(offset)];
  factory BrepAffineTransform.fromJson(List<dynamic> j) =>
      BrepAffineTransform(_p(j[0]), _p(j[1]), _p(j[2]), _p(j[3]));
  factory BrepAffineTransform.fromFunction(Vector3 Function(Vector3) f) {
    final o = f(Vector3.zero());
    return BrepAffineTransform(
      f(Vector3(1, 0, 0)) - o,
      f(Vector3(0, 1, 0)) - o,
      f(Vector3(0, 0, 1)) - o,
      o,
    );
  }
}

class BrepAffineCurve implements BrepCurve {
  BrepAffineCurve(this.base, this.transform);
  final BrepCurve base;
  final BrepAffineTransform transform;
  @override
  double get t0 => base.t0;
  @override
  double get t1 => base.t1;
  @override
  Vector3 evaluate(double t) => transform.point(base.evaluate(t));
  @override
  Vector3 tangent(double t) => transform.vector(base.tangent(t)).normalized();
  @override
  double closestParam(Vector3 p) {
    // The inverse image of a Euclidean nearest point is not in general the
    // nearest point after stretching. Minimize in the transformed metric.
    double distance(double t) => (evaluate(t) - p).length2;
    var best = t0, bestD = distance(t0);
    const count = 32;
    for (var i = 0; i <= count; i++) {
      final lo = t0 + (t1 - t0) * math.max(0, i - 1) / count;
      final hi = t0 + (t1 - t0) * math.min(count, i + 1) / count;
      final mid = t0 + (t1 - t0) * i / count, d = distance(mid);
      if (d > bestD &&
          i > 0 &&
          i < count &&
          (d > distance(lo) || d > distance(hi)))
        continue;
      var a = lo, b = hi;
      const ratio = 0.6180339887498949;
      var c = b - ratio * (b - a), e = a + ratio * (b - a);
      var dc = distance(c), de = distance(e);
      for (var k = 0; k < 48; k++) {
        if (dc < de) {
          b = e;
          e = c;
          de = dc;
          c = b - ratio * (b - a);
          dc = distance(c);
        } else {
          a = c;
          c = e;
          dc = de;
          e = a + ratio * (b - a);
          de = distance(e);
        }
      }
      final t = (a + b) * 0.5, value = distance(t);
      if (value < bestD) {
        best = t;
        bestD = value;
      }
    }
    if (distance(t1) < bestD) best = t1;
    return best;
  }

  @override
  Vector3 closestPoint(Vector3 p) => evaluate(closestParam(p));
  @override
  Map<String, dynamic> toJson() => {
    't': 'affine',
    'b': base.toJson(),
    'm': transform.toJson(),
  };
  factory BrepAffineCurve.fromJson(Map<String, dynamic> j) => BrepAffineCurve(
    BrepCurve.fromJson(j['b'] as Map<String, dynamic>),
    BrepAffineTransform.fromJson(j['m'] as List<dynamic>),
  );
}

class BrepAffineSurface implements BrepSurface {
  BrepAffineSurface(this.base, this.transform);
  final BrepSurface base;
  final BrepAffineTransform transform;
  @override
  Vector3 evaluate(double u, double v) => transform.point(base.evaluate(u, v));
  @override
  Vector3 normalAt(double u, double v) => transform.normal(base.normalAt(u, v));
  Vector3 du(double u, double v) {
    if (base is BrepRuledSurface)
      return transform.vector((base as BrepRuledSurface).du(u, v));
    const h = 1e-5;
    return (evaluate(u + h, v) - evaluate(u - h, v)) / (2 * h);
  }

  Vector3 dv(double u, double v) {
    if (base is BrepRuledSurface)
      return transform.vector((base as BrepRuledSurface).dv(u, v));
    const h = 1e-5;
    return (evaluate(u, v + h) - evaluate(u, v - h)) / (2 * h);
  }

  @override
  (double, double)? parameters(Vector3 p) =>
      base.parameters(transform.inversePoint(p));
  @override
  Vector3 closestPoint(Vector3 p) {
    final initial = parameters(p);
    if (initial == null)
      throw BrepUnsupported('Cannot parameterize transformed surface');
    var best = evaluate(initial.$1, initial.$2), bestD = (best - p).length2;
    // Extra angular starts avoid accepting the wrong stationary point on a
    // highly eccentric ellipsoid. Keep the original UV domain and orientation.
    final starts = [initial];
    if (base is BrepSphere || base is BrepTorus) {
      for (var i = 0; i < 8; i++) {
        for (var j = 1; j < 4; j++) {
          starts.add((i * math.pi / 4, j * math.pi / 4));
        }
      }
    }
    for (final start in starts) {
      var u = start.$1, v = start.$2;
      for (var k = 0; k < 48; k++) {
        final q = evaluate(u, v), r = q - p, x = du(u, v), y = dv(u, v);
        final xx = x.length2,
            xy = x.dot(y),
            yy = y.length2,
            det = xx * yy - xy * xy;
        if (det.abs() < 1e-24) break;
        final su = (yy * x.dot(r) - xy * y.dot(r)) / det;
        final sv = (xx * y.dot(r) - xy * x.dot(r)) / det;
        var fraction = 1.0, moved = false;
        while (fraction >= 1 / 4096) {
          var nu = u - su * fraction, nv = v - sv * fraction;
          if (base is BrepRuledSurface) {
            nu = nu.clamp(0.0, 1.0);
            nv = nv.clamp(0.0, 1.0);
          }
          if ((evaluate(nu, nv) - p).length2 < r.length2) {
            moved = (nu - u).abs() + (nv - v).abs() > 1e-12;
            u = nu;
            v = nv;
            break;
          }
          fraction *= 0.5;
        }
        if (!moved) break;
      }
      final q = evaluate(u, v), d = (q - p).length2;
      if (d < bestD) {
        best = q;
        bestD = d;
      }
    }
    return best;
  }

  @override
  Map<String, dynamic> toJson() => {
    't': 'affine',
    'b': base.toJson(),
    'm': transform.toJson(),
  };
  factory BrepAffineSurface.fromJson(Map<String, dynamic> j) =>
      BrepAffineSurface(
        BrepSurface.fromJson(j['b'] as Map<String, dynamic>),
        BrepAffineTransform.fromJson(j['m'] as List<dynamic>),
      );
}
