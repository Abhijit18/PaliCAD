import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_data.dart';
import 'extrude.dart';
import 'geom.dart';
import 'heal.dart';
import 'solid.dart';
import 'xform.dart';

/// Analytic primitives as compact B-Rep (not tessellated walls).
class BrepPrimitives {
  BrepPrimitives._();

  /// Axis-aligned box from the origin to ([dx],[dy],[dz]).
  static BrepSolid box(double dx, double dy, double dz) {
    final b = BrepBuilder();
    final p = [
      b.vertex(Vector3(0, 0, 0)),
      b.vertex(Vector3(dx, 0, 0)),
      b.vertex(Vector3(dx, dy, 0)),
      b.vertex(Vector3(0, dy, 0)),
      b.vertex(Vector3(0, 0, dz)),
      b.vertex(Vector3(dx, 0, dz)),
      b.vertex(Vector3(dx, dy, dz)),
      b.vertex(Vector3(0, dy, dz)),
    ];
    int line(int i, int j) =>
        b.edge(i, j, BrepLine(b.vertices[i], b.vertices[j]));
    final e01 = line(p[0], p[1]);
    final e12 = line(p[1], p[2]);
    final e23 = line(p[2], p[3]);
    final e30 = line(p[3], p[0]);
    final e45 = line(p[4], p[5]);
    final e56 = line(p[5], p[6]);
    final e67 = line(p[6], p[7]);
    final e74 = line(p[7], p[4]);
    final e04 = line(p[0], p[4]);
    final e15 = line(p[1], p[5]);
    final e26 = line(p[2], p[6]);
    final e37 = line(p[3], p[7]);

    BrepLoop loop(List<(int, bool)> es) =>
        BrepLoop([for (final e in es) BrepCoedge(e.$1, reversed: e.$2)]);

    // Bottom z=0, outward -Z. CW from +Z.
    b.face(BrepPlane(origin: Vector3(0, 0, 0), normal: Vector3(0, 0, -1)), [
      loop([(e01, true), (e30, true), (e23, true), (e12, true)]),
    ]);
    // Top z=dz, outward +Z.
    b.face(BrepPlane(origin: Vector3(0, 0, dz), normal: Vector3(0, 0, 1)), [
      loop([(e45, false), (e56, false), (e67, false), (e74, false)]),
    ]);
    // Front y=0, outward -Y.
    b.face(BrepPlane(origin: Vector3(0, 0, 0), normal: Vector3(0, -1, 0)), [
      loop([(e01, false), (e15, false), (e45, true), (e04, true)]),
    ]);
    // Back y=dy, outward +Y.
    b.face(BrepPlane(origin: Vector3(0, dy, 0), normal: Vector3(0, 1, 0)), [
      loop([(e23, false), (e37, false), (e67, true), (e26, true)]),
    ]);
    // Left x=0, outward -X.
    b.face(BrepPlane(origin: Vector3(0, 0, 0), normal: Vector3(-1, 0, 0)), [
      loop([(e30, false), (e04, false), (e74, true), (e37, true)]),
    ]);
    // Right x=dx, outward +X.
    b.face(BrepPlane(origin: Vector3(dx, 0, 0), normal: Vector3(1, 0, 0)), [
      loop([(e12, false), (e26, false), (e56, true), (e15, true)]),
    ]);
    return b.build();
  }

  /// Rebuild an axis-aligned box from its tessellation, or null.
  static BrepSolid? tryAabbBox(MeshData mesh) {
    if (mesh.vertices.length != 8) return null;
    var min = mesh.vertices.first.clone();
    var max = mesh.vertices.first.clone();
    for (final v in mesh.vertices) {
      min = Vector3(
        min.x < v.x ? min.x : v.x,
        min.y < v.y ? min.y : v.y,
        min.z < v.z ? min.z : v.z,
      );
      max = Vector3(
        max.x > v.x ? max.x : v.x,
        max.y > v.y ? max.y : v.y,
        max.z > v.z ? max.z : v.z,
      );
    }
    final dx = max.x - min.x;
    final dy = max.y - min.y;
    final dz = max.z - min.z;
    if (dx < 1e-8 || dy < 1e-8 || dz < 1e-8) return null;
    var corners = 0;
    final seenCorner = <String>{};
    for (final v in mesh.vertices) {
      final onX = (v.x - min.x).abs() < 1e-5 || (v.x - max.x).abs() < 1e-5;
      final onY = (v.y - min.y).abs() < 1e-5 || (v.y - max.y).abs() < 1e-5;
      final onZ = (v.z - min.z).abs() < 1e-5 || (v.z - max.z).abs() < 1e-5;
      if (!onX && !onY && !onZ) return null;
      final n = (onX ? 1 : 0) + (onY ? 1 : 0) + (onZ ? 1 : 0);
      if (n == 3) {
        final k =
            '${v.x.toStringAsFixed(5)}_${v.y.toStringAsFixed(5)}_${v.z.toStringAsFixed(5)}';
        if (seenCorner.add(k)) corners++;
      }
    }
    if (corners != 8) return null;
    final box = BrepPrimitives.box(dx, dy, dz);
    if (min.length2 < 1e-20) return box;
    return BrepXform.translate(box, min);
  }

  /// Cylinder along [axis] from [origin], matching historical PonaCAD placement
  /// when origin is (r,r,0) and axis is +Z.
  static BrepSolid cylinder({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double height,
  }) {
    final a = axis.normalized();
    final xDir = brepFrameU(a);
    final b = BrepBuilder();
    final seamB = origin + xDir * radius;
    final seamT = origin + a * height + xDir * radius;
    final vb = b.vertex(seamB);
    final vt = b.vertex(seamT);
    final seam = b.edge(vb, vt, BrepLine(seamB, seamT));
    final bot = b.edge(
      vb,
      vb,
      BrepCircle(center: origin, normal: -a, radius: radius, xDir: xDir),
    );
    final top = b.edge(
      vt,
      vt,
      BrepCircle(
        center: origin + a * height,
        normal: a,
        radius: radius,
        xDir: xDir,
      ),
    );
    b.face(BrepPlane(origin: origin, normal: -a, xDir: xDir), [
      BrepLoop([BrepCoedge(bot)]),
    ]);
    b.face(BrepPlane(origin: origin + a * height, normal: a, xDir: xDir), [
      BrepLoop([BrepCoedge(top)]),
    ]);
    b.face(BrepCylinder(origin: origin, axis: a, radius: radius, xDir: xDir), [
      BrepLoop([
        BrepCoedge(seam),
        BrepCoedge(top),
        BrepCoedge(seam, reversed: true),
        BrepCoedge(bot, reversed: true),
      ]),
    ]);
    return b.build();
  }

  /// Truncated cone: circular caps of [radius] and [topRadius] along [axis].
  static BrepSolid frustum({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double topRadius,
    required double height,
  }) {
    if (radius <= 1e-9 && topRadius <= 1e-9) {
      throw BrepUnsupported('A frustum needs a positive radius');
    }
    if (height.abs() < 1e-12) {
      throw BrepUnsupported('A frustum needs a positive height');
    }
    if (topRadius <= 1e-9) {
      return cone(origin: origin, axis: axis, radius: radius, height: height);
    }
    if (radius <= 1e-9) {
      final a = axis.normalized();
      return cone(
        origin: origin + a * height,
        axis: -a,
        radius: topRadius,
        height: height,
      );
    }
    if ((radius - topRadius).abs() < 1e-9) {
      return cylinder(
        origin: origin,
        axis: axis,
        radius: radius,
        height: height,
      );
    }
    final a = axis.normalized();
    final xDir = brepFrameU(a);
    final b = BrepBuilder();
    final seamB = origin + xDir * radius;
    final seamT = origin + a * height + xDir * topRadius;
    final vb = b.vertex(seamB);
    final vt = b.vertex(seamT);
    final seam = b.edge(vb, vt, BrepLine(seamB, seamT));
    final bot = b.edge(
      vb,
      vb,
      BrepCircle(center: origin, normal: -a, radius: radius, xDir: xDir),
    );
    final top = b.edge(
      vt,
      vt,
      BrepCircle(
        center: origin + a * height,
        normal: a,
        radius: topRadius,
        xDir: xDir,
      ),
    );
    b.face(BrepPlane(origin: origin, normal: -a, xDir: xDir), [
      BrepLoop([BrepCoedge(bot)]),
    ]);
    b.face(BrepPlane(origin: origin + a * height, normal: a, xDir: xDir), [
      BrepLoop([BrepCoedge(top)]),
    ]);
    b.face(
      BrepCone(
        origin: origin,
        axis: a,
        radius: radius,
        topRadius: topRadius,
        height: height,
        xDir: xDir,
      ),
      [
        BrepLoop([
          BrepCoedge(seam),
          BrepCoedge(top),
          BrepCoedge(seam, reversed: true),
          BrepCoedge(bot, reversed: true),
        ]),
      ],
    );
    return b.build();
  }

  static BrepSolid sphere({required Vector3 center, required double radius}) {
    final b = BrepBuilder();
    b.face(BrepSphere(center: center, radius: radius), const []);
    return b.build();
  }

  /// Regular n-gon prism, circumdiameter [diameter], height along +Z.
  /// Footprint centered at (R, R) like [cylinder].
  static BrepSolid prism({
    required double diameter,
    required double height,
    required int sides,
  }) {
    final n = sides < 3 ? 3 : sides;
    final r = diameter * 0.5;
    final c = Vector3(r, r, 0);
    final ring = [
      for (var i = 0; i < n; i++)
        c +
            Vector3(
              r * math.cos(i * math.pi * 2 / n),
              r * math.sin(i * math.pi * 2 / n),
              0,
            ),
    ];
    return BrepExtrude.polygon(ring, Vector3(0, 0, height));
  }

  /// Regular n-gon pyramid, base circumdiameter [diameter], apex at +Z.
  static BrepSolid pyramid({
    required double diameter,
    required double height,
    required int sides,
  }) {
    final n = sides < 3 ? 3 : sides;
    final r = diameter * 0.5;
    final c = Vector3(r, r, 0);
    final apexP = Vector3(r, r, height);
    final ring = [
      for (var i = 0; i < n; i++)
        c +
            Vector3(
              r * math.cos(i * math.pi * 2 / n),
              r * math.sin(i * math.pi * 2 / n),
              0,
            ),
    ];
    final b = BrepBuilder();
    final apex = b.vertex(apexP);
    final bot = [for (final p in ring) b.vertex(p)];
    final eBot = <int>[];
    final eApex = <int>[];
    for (var i = 0; i < n; i++) {
      final j = (i + 1) % n;
      eBot.add(b.edge(bot[i], bot[j], BrepLine(ring[i], ring[j])));
      eApex.add(b.edge(bot[i], apex, BrepLine(ring[i], apexP)));
    }
    b.face(BrepPlane(origin: ring[0], normal: Vector3(0, 0, -1)), [
      BrepLoop([for (var i = n - 1; i >= 0; i--) BrepCoedge(eBot[i])]),
    ]);
    for (var i = 0; i < n; i++) {
      final j = (i + 1) % n;
      var fn = (ring[j] - ring[i]).cross(apexP - ring[i]);
      if (fn.length < 1e-12) continue;
      fn.normalize();
      if (fn.dot(ring[i] - c) < 0) fn = -fn;
      b.face(BrepPlane(origin: ring[i], normal: fn), [
        BrepLoop([
          BrepCoedge(eBot[i]),
          BrepCoedge(eApex[j]),
          BrepCoedge(eApex[i], reversed: true),
        ]),
      ]);
    }
    return b.build();
  }

  /// Right circular cone, base centered at [origin] like [cylinder].
  static BrepSolid cone({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double height,
  }) {
    final a = axis.normalized();
    final xDir = brepFrameU(a);
    final b = BrepBuilder();
    final apexP = origin + a * height;
    final seamP = origin + xDir * radius;
    final vs = b.vertex(seamP);
    final va = b.vertex(apexP);
    final seam = b.edge(vs, va, BrepLine(seamP, apexP));
    final bot = b.edge(
      vs,
      vs,
      BrepCircle(center: origin, normal: -a, radius: radius, xDir: xDir),
    );
    b.face(BrepPlane(origin: origin, normal: -a, xDir: xDir), [
      BrepLoop([BrepCoedge(bot)]),
    ]);
    b.face(
      BrepCone(
        origin: origin,
        axis: a,
        radius: radius,
        height: height,
        xDir: xDir,
      ),
      [
        BrepLoop([
          BrepCoedge(bot, reversed: true),
          BrepCoedge(seam),
          BrepCoedge(seam, reversed: true),
        ]),
      ],
    );
    return b.build();
  }

  /// Torus (donut) with tube radius [minorRadius] around [center].
  static BrepSolid torus({
    required Vector3 center,
    required Vector3 axis,
    required double majorRadius,
    required double minorRadius,
  }) {
    final b = BrepBuilder();
    b.face(
      BrepTorus(
        center: center,
        axis: axis,
        majorRadius: majorRadius,
        minorRadius: minorRadius,
      ),
      const [],
    );
    return b.build();
  }

  /// Sampled circular [ring] swept along [direction] → analytic cylinder.
  ///
  /// Sketch circles arrive as 32- or 64-gons. Regular polygons with fewer
  /// than 16 vertices stay polygonal (a square's corners also lie on a circle).
  static BrepSolid? tryCylinderFromRing(List<Vector3> ring, Vector3 direction) {
    if (ring.length < 16 || direction.length < 1e-12) return null;
    final axis = direction.normalized();
    final height = direction.length;
    var c = Vector3.zero();
    for (final p in ring) {
      c += p;
    }
    c /= ring.length.toDouble();
    c -= axis * (c - ring.first).dot(axis);
    var rSum = 0.0;
    var maxDev = 0.0;
    for (final p in ring) {
      final radial = p - c;
      final along = radial.dot(axis);
      if (along.abs() > 1e-4) return null;
      final r = (radial - axis * along).length;
      rSum += r;
    }
    final rMean = rSum / ring.length;
    if (rMean < 1e-6) return null;
    for (final p in ring) {
      final radial = p - c;
      final r = (radial - axis * radial.dot(axis)).length;
      maxDev = math.max(maxDev, (r - rMean).abs());
    }
    if (maxDev > math.max(1e-4, rMean * 1e-3)) return null;
    return cylinder(origin: c, axis: axis, radius: rMean, height: height);
  }

  /// Recover a cylinder from a tessellated shaft (sketch-circle extrude).
  static BrepSolid? tryCylinderFromMesh(MeshData mesh) {
    final n = mesh.vertices.length;
    if (n < 16 || n > 260) return null;
    if (mesh.faces.length > 520) return null;
    var min = mesh.vertices.first.clone();
    var max = min.clone();
    for (final v in mesh.vertices) {
      min = Vector3(
        math.min(min.x, v.x),
        math.min(min.y, v.y),
        math.min(min.z, v.z),
      );
      max = Vector3(
        math.max(max.x, v.x),
        math.max(max.y, v.y),
        math.max(max.z, v.z),
      );
    }
    final ext = max - min;
    final axis = ext.x >= ext.y && ext.x >= ext.z
        ? Vector3(1, 0, 0)
        : ext.y >= ext.z
        ? Vector3(0, 1, 0)
        : Vector3(0, 0, 1);
    var z0 = double.infinity;
    var z1 = -double.infinity;
    for (final v in mesh.vertices) {
      final z = v.dot(axis);
      z0 = math.min(z0, z);
      z1 = math.max(z1, z);
    }
    final height = z1 - z0;
    if (height < 1e-6) return null;
    final band = math.max(1e-4, height * 1e-3);
    final bot = <Vector3>[];
    final top = <Vector3>[];
    for (final v in mesh.vertices) {
      final z = v.dot(axis);
      if ((z - z0).abs() <= band) {
        bot.add(v);
      } else if ((z - z1).abs() <= band) {
        top.add(v);
      } else {
        return null;
      }
    }
    if (bot.length < 8 || top.length < 8) return null;
    Vector3 centroid(List<Vector3> pts) {
      var c = Vector3.zero();
      for (final p in pts) {
        c += p;
      }
      return c / pts.length.toDouble();
    }

    final c0 = centroid(bot);
    final c1 = centroid(top);
    final d = c1 - c0;
    if (d.length < 1e-6) return null;
    final a = d.normalized();
    if (a.cross(axis).length > 0.08) return null;
    double ringRadius(List<Vector3> pts, Vector3 c) {
      final rs = [for (final p in pts) (p - c - a * (p - c).dot(a)).length];
      final peak = rs.reduce(math.max);
      if (peak < 1e-6) return 0.0;
      final ring = [
        for (final r in rs)
          if (r > peak * 0.5) r,
      ];
      if (ring.length < 8) return 0.0;
      return ring.reduce((x, y) => x + y) / ring.length;
    }

    final r0 = ringRadius(bot, c0);
    final r1 = ringRadius(top, c1);
    if (r0 < 1e-6 || (r0 - r1).abs() > math.max(1e-3, r0 * 0.02)) return null;
    final tol = math.max(1e-3, r0 * 0.03);
    bool onRing(List<Vector3> pts, Vector3 c) {
      for (final p in pts) {
        final r = (p - c - a * (p - c).dot(a)).length;
        if (r < r0 * 0.2) continue;
        if ((r - r0).abs() > tol) return false;
      }
      return true;
    }

    if (!onRing(bot, c0) || !onRing(top, c1)) return null;
    return cylinder(origin: c0, axis: d, radius: r0, height: d.length);
  }

  /// Polyhedral B-Rep: weld, unify coplanar triangles, outward faces.
  static BrepSolid fromMesh(MeshData mesh) => BrepHeal.fromMesh(mesh);

  /// Planar shell source recovered from a mesh, or null.
  ///
  /// Used when hollow has no cached analytic solid. Dense / curved
  /// tessellations stay on the mesh path — this does not invent cylinders.
  static BrepSolid? tryPlanarSolid(MeshData mesh) {
    if (mesh.faces.length > 160 || mesh.vertices.length > 96) return null;
    try {
      final s = fromMesh(mesh);
      if (s.faces.isEmpty || s.faces.length > 48) return null;
      for (final f in s.faces) {
        if (f.surface is! BrepPlane) return null;
      }
      return s;
    } catch (_) {
      return null;
    }
  }
}
