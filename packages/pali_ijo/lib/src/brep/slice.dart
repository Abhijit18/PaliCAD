import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../measure/bounds.dart';
import 'heal.dart';
import 'intersect.dart';
import 'boolean.dart';
import 'geom.dart';
import 'primitives.dart';
import 'provenance.dart';
import 'solid.dart';
import 'xform.dart';

class BrepSlice {
  BrepSlice._();

  /// Splits [solid] by a plane.
  ///
  /// Every piece inherits the tags of the faces it kept, so both halves of a
  /// cut box agree about which face used to be the top. The cut faces
  /// themselves are new, and each piece's cut face is tagged as created by the
  /// slice.
  static List<BrepSolid> parts(
    BrepSolid solid,
    Vector3 origin,
    Vector3 normal,
  ) => [
    for (final part in _parts(solid, origin, normal))
      BrepProvenance.inheritFrom(part, solid, op: 'slice'),
  ];

  static List<BrepSolid> _parts(
    BrepSolid solid,
    Vector3 origin,
    Vector3 normal,
  ) {
    if (![
          origin.x,
          origin.y,
          origin.z,
          normal.x,
          normal.y,
          normal.z,
        ].every((v) => v.isFinite) ||
        normal.length2 < 1e-24) {
      throw BrepUnsupported('Slice needs a finite plane and a nonzero normal');
    }
    final transformed = BrepXform.unwrapAffine(solid);
    if (transformed != null) {
      final m = transformed.transform;
      return [
        for (final part in _parts(
          transformed.solid,
          m.inversePoint(origin),
          m.transposeVector(normal),
        ))
          BrepXform.affine(part, m),
      ];
    }
    final n = normal.normalized();
    final bounds = conservativeSolidBounds(solid);
    if (bounds == null)
      throw BrepUnsupported('Slice could not bound the solid');
    final signed = <double>[
      for (final x in [bounds.min.x, bounds.max.x])
        for (final y in [bounds.min.y, bounds.max.y])
          for (final z in [bounds.min.z, bounds.max.z])
            (Vector3(x, y, z) - origin).dot(n),
    ];
    if (signed.every((d) => d >= -1e-8) || signed.every((d) => d <= 1e-8))
      return [solid.clone()];
    if (solid.faces.length == 1 &&
        solid.faces.single.surface is BrepSphere &&
        solid.faces.single.loops.isEmpty &&
        !solid.faces.single.reversed) {
      return _splitSphere(
        solid,
        solid.faces.single.surface as BrepSphere,
        origin,
        n,
      );
    }
    if (_isCylinderPrimitive(solid)) {
      final cyl = solid.faces
          .map((f) => f.surface)
          .whereType<BrepCylinder>()
          .first;
      final parts = _splitCappedCylinder(solid, cyl, origin, n);
      if (parts != null) return parts;
    }
    final cutter = _halfSpace(origin, n, solid);
    BrepSolid? half(int op) {
      try {
        final result = BrepBoolean.run(
          solid,
          cutter,
          op,
          sharedBoundary: solid.faces.every((f) => f.surface is BrepPlane),
        );
        // A half that failed must never cause its sibling to be committed
        // alone. Validate both before returning any replacement bodies.
        if (!BrepHeal.topologyIsClosed(result)) {
          throw BrepUnsupported('Slice could not close both sides of the cut');
        }
        BrepHeal.tessellateClosed(result, op: 'Slice');
        return result;
      } on StateError catch (e) {
        if (e.message == 'Boolean produced no geometry') return null;
        rethrow;
      }
    }

    final positive = half(BrepBoolean.intersectOp);
    final negative = half(BrepBoolean.subtractOp);
    if (positive == null || negative == null) {
      // An empty intersection is a tangential/no-op cut, not a partial result.
      if (positive == null && negative == null) {
        throw BrepUnsupported('Slice produced no geometry');
      }
      return [solid.clone()];
    }
    return [...brepSplitSolids(negative), ...brepSplitSolids(positive)];
  }

  static List<BrepSolid> _splitSphere(
    BrepSolid solid,
    BrepSphere sphere,
    Vector3 origin,
    Vector3 n,
  ) {
    final h = (origin - sphere.center).dot(n);
    if (h.abs() >= sphere.radius - 1e-8) return [solid.clone()];
    final circle = BrepCircle(
      center: sphere.center + n * h,
      normal: n,
      radius: math.sqrt(sphere.radius * sphere.radius - h * h),
      xDir: brepFrameU(n),
    );
    BrepSolid cap(bool positive) {
      final b = BrepBuilder();
      final v = b.vertex(circle.evaluate(circle.t0));
      final e = b.edge(v, v, circle);
      b.face(sphere, [
        BrepLoop([BrepCoedge(e, reversed: !positive)]),
      ], origin: solid.faces.single.origin);
      b.face(BrepPlane(origin: circle.center, normal: positive ? -n : n), [
        BrepLoop([BrepCoedge(e, reversed: positive)]),
      ]);
      return b.build();
    }

    return [cap(false), cap(true)];
  }

  /// Split a cylinder between two closed planar rims, including oblique rims.
  /// The new ellipse must stay inside both caps over its entire parameter
  /// interval; a cut that intersects a cap goes through the general trimmer.
  static List<BrepSolid>? _splitCappedCylinder(
    BrepSolid solid,
    BrepCylinder cyl,
    Vector3 origin,
    Vector3 n,
  ) {
    if (n.dot(cyl.axis).abs() < 1e-10) return null;
    final cutPlane = BrepPlane(origin: origin, normal: n);
    final curves = BrepIntersect.surfaces(cutPlane, cyl);
    if (curves.length != 1) return null;
    final cut = curves.single;
    if (cut is! BrepCircle && cut is! BrepEllipse) return null;
    final center = cut is BrepCircle ? cut.center : (cut as BrepEllipse).center;
    final dx = cut is BrepCircle
        ? cut.xDir * cut.radius
        : (cut as BrepEllipse).xDir * cut.rMajor;
    final dy = cut is BrepCircle
        ? cut.yDir * cut.radius
        : (cut as BrepEllipse).yDir * cut.rMinor;
    final caps = solid.faces.where((f) => f.surface is BrepPlane).toList();
    for (final cap in caps) {
      if (cap.loops.length != 1 || cap.loops.single.coedges.length != 1)
        return null;
      final plane = cap.surface as BrepPlane;
      final cn = cap.outwardNormal(0, 0);
      final extent = math.sqrt(
        math.pow(cn.dot(dx), 2) + math.pow(cn.dot(dy), 2),
      );
      if ((center - plane.origin).dot(cn) + extent >= -1e-8) return null;
    }
    final wall = solid.faces.singleWhere((f) => f.surface is BrepCylinder);
    return [
      for (final cap in caps)
        () {
          final b = BrepBuilder();
          final old = solid.edges[cap.loops.single.coedges.single.edge].curve;
          final oldV = b.vertex(old.evaluate(old.t0)),
              newV = b.vertex(cut.evaluate(cut.t0));
          final oldE = b.edge(oldV, oldV, old), newE = b.edge(newV, newV, cut);
          final cn = cap.outwardNormal(0, 0);
          final oldNormal = old is BrepCircle
              ? old.normal
              : (old as BrepEllipse).normal;
          final cutNormal = cut is BrepCircle
              ? cut.normal
              : (cut as BrepEllipse).normal;
          final oldReverse = oldNormal.dot(cn) < 0;
          final newNormal =
              n * (cn.dot(cyl.axis) * n.dot(cyl.axis) > 0 ? -1.0 : 1.0);
          final newReverse = cutNormal.dot(newNormal) < 0;
          b.face(
            BrepPlane(origin: (cap.surface as BrepPlane).origin, normal: cn),
            [
              BrepLoop([BrepCoedge(oldE, reversed: oldReverse)]),
            ],
            origin: cap.origin,
          );
          b.face(BrepPlane(origin: center, normal: newNormal), [
            BrepLoop([BrepCoedge(newE, reversed: newReverse)]),
          ]);
          b.face(
            cyl,
            [
              BrepLoop([BrepCoedge(oldE, reversed: !oldReverse)]),
              BrepLoop([BrepCoedge(newE, reversed: !newReverse)]),
            ],
            reversed: wall.reversed,
            origin: wall.origin,
          );
          return b.build();
        }(),
    ];
  }

  static BrepSolid run(BrepSolid solid, Vector3 origin, Vector3 normal) {
    final p = parts(solid, origin, normal);
    if (p.isEmpty) return solid;
    var acc = p.first;
    for (var i = 1; i < p.length; i++) {
      acc = BrepBoolean.run(acc, p[i], BrepBoolean.unionOp);
    }
    return acc;
  }

  static bool _isCylinderPrimitive(BrepSolid s) {
    if (s.faces.length != 3) return false;
    var cyls = 0;
    var planes = 0;
    for (final f in s.faces) {
      if (f.surface is BrepCylinder) {
        cyls++;
      } else if (f.surface is BrepPlane) {
        planes++;
      } else {
        return false;
      }
    }
    return cyls == 1 && planes == 2;
  }

  /// Half-space as a box just large enough to cover [solid].
  static BrepSolid _halfSpace(Vector3 origin, Vector3 n, BrepSolid solid) {
    final bounds = conservativeSolidBounds(solid);
    if (bounds == null)
      throw BrepUnsupported('Slice could not bound the solid');
    var reach = 0.0;
    for (final x in [bounds.min.x, bounds.max.x]) {
      for (final y in [bounds.min.y, bounds.max.y]) {
        for (final z in [bounds.min.z, bounds.max.z]) {
          reach = math.max(reach, (Vector3(x, y, z) - origin).length);
        }
      }
    }
    final size = math.max(reach * 2, 1.0);
    var u = n.cross(Vector3(0, 0, 1));
    if (u.length < 1e-8) u = n.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = n.cross(u).normalized();
    final box = BrepPrimitives.box(size * 2, size * 2, size);
    final corner = origin - u * size - v * size;
    return BrepXform.map(box, (p) => corner + u * p.x + v * p.y + n * p.z);
  }
}
