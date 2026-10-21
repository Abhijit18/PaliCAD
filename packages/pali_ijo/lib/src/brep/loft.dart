part of 'pipe.dart';

class _LoftRing {
  _LoftRing(List<Vector3> input) {
    for (final p in input) {
      if (!p.x.isFinite || !p.y.isFinite || !p.z.isFinite) {
        throw BrepUnsupported('Loft sections must contain finite coordinates');
      }
      if (points.isEmpty || (p - points.last).length > 1e-8)
        points.add(p.clone());
    }
    if (points.length > 1 && (points.first - points.last).length < 1e-8)
      points.removeLast();
    if (points.length < 3)
      throw BrepUnsupported(
        'Loft sections need at least three distinct points',
      );
    final n = BrepLoft._ringNormal(points);
    if (n == null) throw BrepUnsupported('Loft section has zero area');
    normal = n;
    center = points.reduce((a, b) => a + b) / points.length.toDouble();
    final size = points.map((p) => (p - center).length).reduce(math.max);
    tolerance = math.max(1e-8, size * 1e-9);
    if (points.any((p) => (p - center).dot(normal).abs() > tolerance)) {
      throw BrepUnsupported('Loft sections must be planar');
    }
    // A crossing boundary cannot define a regular cap.
    final plane = BrepPlane(origin: center, normal: normal);
    final uv = [
      for (final p in points)
        Vector2((p - center).dot(plane.xDir), (p - center).dot(plane.yDir)),
    ];
    double cross(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;
    for (var i = 0; i < uv.length; i++) {
      for (var j = i + 2; j < uv.length; j++) {
        if (i == 0 && j == uv.length - 1) continue;
        final a = uv[i],
            b = uv[(i + 1) % uv.length],
            c = uv[j],
            d = uv[(j + 1) % uv.length];
        if (cross(b - a, c - a) * cross(b - a, d - a) < 0 &&
            cross(d - c, a - c) * cross(d - c, b - c) < 0) {
          throw BrepUnsupported('Loft section crosses itself');
        }
      }
    }
    circle = BrepLoft._fitCircle(points);
    if (circle != null) center = circle!.center;
    resetLengths();
  }

  final points = <Vector3>[];
  late Vector3 normal, center;
  late double tolerance;
  ({Vector3 center, Vector3 normal, double radius})? circle;
  List<double> breaks = [];

  void resetLengths() {
    if (circle != null) {
      breaks = [0, 0.25, 0.5, 0.75, 1];
      return;
    }
    breaks = [0];
    for (var i = 0; i < points.length; i++) {
      breaks.add(
        breaks.last + (points[(i + 1) % points.length] - points[i]).length,
      );
    }
    final total = breaks.last;
    breaks = [for (final v in breaks) v / total];
  }

  void align(Vector3 direction, _LoftRing? previous) {
    if (normal.dot(direction) < 0) {
      final first = points.first;
      final reversed = [first, ...points.skip(1).toList().reversed];
      points
        ..clear()
        ..addAll(reversed);
      normal = -normal;
    }
    if (previous != null) {
      final aim = (previous.points.first - previous.center).normalized();
      if (circle != null) {
        points[0] =
            center +
            (aim - normal * aim.dot(normal)).normalized() * circle!.radius;
      } else {
        var best = 0, score = -double.infinity;
        for (var i = 0; i < points.length; i++) {
          final d = (points[i] - center).normalized().dot(aim);
          if (d > score) {
            score = d;
            best = i;
          }
        }
        final rotated = [...points.skip(best), ...points.take(best)];
        points
          ..clear()
          ..addAll(rotated);
      }
    }
    resetLengths();
  }

  Vector3 point(double u) {
    if (circle != null) {
      final x = (points.first - center).normalized(), y = normal.cross(x);
      return center +
          (x * math.cos(u * 2 * math.pi) + y * math.sin(u * 2 * math.pi)) *
              circle!.radius;
    }
    var i = 0;
    while (i + 1 < points.length && u > breaks[i + 1] + 1e-12) i++;
    final t = (u - breaks[i]) / (breaks[i + 1] - breaks[i]);
    return points[i] + (points[(i + 1) % points.length] - points[i]) * t;
  }

  BrepCurve segment(double lo, double hi) => circle == null
      ? BrepLine(point(lo), point(hi))
      : BrepCircle(
          center: center,
          normal: normal,
          radius: circle!.radius,
          xDir: (points.first - center).normalized(),
          t0: lo * 2 * math.pi,
          t1: hi * 2 * math.pi,
        );

  bool contains(Vector3 p) {
    if (circle != null) return (p - center).length < circle!.radius - tolerance;
    final x = brepFrameU(normal), y = normal.cross(x), q = p - center;
    var inside = false;
    for (var i = 0, j = points.length - 1; i < points.length; j = i++) {
      final a = points[i] - center, b = points[j] - center;
      final ay = a.dot(y), by = b.dot(y), py = q.dot(y);
      if ((ay > py) != (by > py) &&
          q.dot(x) < (b.dot(x) - a.dot(x)) * (py - ay) / (by - ay) + a.dot(x))
        inside = !inside;
    }
    return inside;
  }
}

class _LoftBuilder {
  static BrepSolid run(
    List<List<Vector3>> sections, {
    List<(List<Vector3>, List<Vector3>)> holes = const [],
  }) {
    if (sections.length < 2)
      throw BrepUnsupported('Loft needs at least two sections');
    if (holes.isNotEmpty && sections.length != 2) {
      throw BrepUnsupported('Loft holes need a matching loop at every station');
    }
    final outer = [for (final s in sections) _LoftRing(s)];
    final direction = outer.last.center - outer.first.center;
    if (direction.length < 1e-8)
      throw BrepUnsupported('Loft sections must be separated');
    final axis = outer.first.normal.dot(direction) > 0
        ? outer.first.normal
        : -outer.first.normal;
    for (var i = 0; i < outer.length; i++) {
      final ring = outer[i];
      if (ring.normal.cross(axis).length > 1e-8) {
        throw BrepUnsupported(
          'Loft currently requires parallel section planes',
        );
      }
      if (i > 0 &&
          (ring.center - outer[i - 1].center).dot(axis) <= ring.tolerance) {
        throw BrepUnsupported(
          'Loft sections must advance without folding back',
        );
      }
      ring.align(axis, i == 0 ? null : outer[i - 1]);
    }
    final contours = <List<_LoftRing>>[outer];
    for (final (a, b) in holes) {
      final inner = [_LoftRing(a), _LoftRing(b)];
      for (var i = 0; i < 2; i++) {
        if (inner[i].normal.cross(axis).length > 1e-8 ||
            (inner[i].center - outer[i].center).dot(axis).abs() >
                outer[i].tolerance ||
            inner[i].points.any((p) => !outer[i].contains(p))) {
          throw BrepUnsupported('Loft holes must lie inside their section');
        }
        inner[i].align(axis, i == 0 ? null : inner[i - 1]);
      }
      contours.add(inner);
    }
    final b = BrepBuilder();
    final bottom = <BrepLoop>[], top = <BrepLoop>[];
    for (var ci = 0; ci < contours.length; ci++) {
      final rings = contours[ci], inside = ci > 0;
      final sorted = [for (final r in rings) ...r.breaks]..sort();
      final knots = <double>[];
      for (final k in sorted) {
        if (knots.isEmpty || k - knots.last > 1e-10) knots.add(k);
      }
      final n = knots.length - 1;
      final vertices = [
        for (final r in rings)
          [for (final k in knots.take(n)) b.vertex(r.point(k))],
      ];
      final curves = [
        for (final r in rings)
          [for (var j = 0; j < n; j++) r.segment(knots[j], knots[j + 1])],
      ];
      final rims = [
        for (var i = 0; i < rings.length; i++)
          [
            for (var j = 0; j < n; j++)
              b.edge(vertices[i][j], vertices[i][(j + 1) % n], curves[i][j]),
          ],
      ];
      BrepLoop rim(int i, bool reverse) => BrepLoop([
        for (final e in reverse ? rims[i].reversed : rims[i])
          BrepCoedge(e, reversed: reverse),
      ]);
      bottom.add(rim(0, !inside));
      top.add(rim(rings.length - 1, inside));
      // If every side is planar, retain planes for subsequent exact booleans.
      final patches = <List<BrepRuledSurface>>[
        for (var i = 0; i + 1 < rings.length; i++)
          [
            for (var j = 0; j < n; j++)
              BrepRuledSurface(curves[i][j], curves[i + 1][j]),
          ],
      ];
      bool planar(BrepRuledSurface p) {
        if (p.lower is! BrepLine || p.upper is! BrepLine) return false;
        final normal = p.du(0, 0).cross(p.dv(0, 0));
        return normal.length > 1e-12 &&
            (p.evaluate(1, 1) - p.evaluate(0, 0))
                    .dot(normal.normalized())
                    .abs() <
                1e-8;
      }

      final allPlanar = patches.every((row) => row.every(planar));
      for (var i = 0; i + 1 < rings.length; i++) {
        final rails = [
          for (var j = 0; j < n; j++)
            b.edge(
              vertices[i][j],
              vertices[i + 1][j],
              BrepLine(
                b.vertices[vertices[i][j]],
                b.vertices[vertices[i + 1][j]],
              ),
            ),
        ];
        for (var j = 0; j < n; j++) {
          final p = patches[i][j];
          for (final u in [0.0, 0.5, 1.0]) {
            for (final v in [0.0, 0.5, 1.0]) {
              if (p.du(u, v).cross(p.dv(u, v)).length < 1e-10) {
                throw BrepUnsupported('Loft has a collapsed or folded side');
              }
            }
          }
          final loop = [
            BrepCoedge(rims[i][j]),
            BrepCoedge(rails[(j + 1) % n]),
            BrepCoedge(rims[i + 1][j], reversed: true),
            BrepCoedge(rails[j], reversed: true),
          ];
          b.face(
            allPlanar
                ? BrepPlane(
                    origin: p.evaluate(0, 0),
                    normal: p.normalAt(0.5, 0.5),
                  )
                : p,
            [
              BrepLoop(
                inside
                    ? [
                        for (final c in loop.reversed)
                          BrepCoedge(c.edge, reversed: !c.reversed),
                      ]
                    : loop,
              ),
            ],
            reversed: inside,
          );
        }
      }
    }
    b.face(BrepPlane(origin: outer.first.center, normal: -axis), bottom);
    b.face(BrepPlane(origin: outer.last.center, normal: axis), top);
    return b.build();
  }
}
