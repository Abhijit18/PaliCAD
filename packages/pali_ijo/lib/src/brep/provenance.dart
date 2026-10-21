import 'package:vector_math/vector_math_64.dart';

// Imports the individual files rather than the barrel, so the operations that
// call into this can be imported by it without a cycle.
import 'geom.dart';
import 'origin.dart';
import 'solid.dart';

/// Carries [BrepOrigin] tags from an operation's inputs onto its result.
///
/// Tags are matched by **supporting geometry**, not by index. An output face
/// inherits from the input face lying on the same plane, cylinder, sphere, cone
/// or torus; an output edge inherits from the input edge on the same line,
/// circle or ellipse. Anything with no match is geometry the operation
/// invented, and is tagged as created by it.
///
/// Matching on geometry rather than threading tags through each algorithm is a
/// deliberate trade. It means booleans, fillets, hollows and slices need no
/// changes to participate, which matters when the boolean alone is three
/// thousand lines. It also gets the interesting cases right for free:
///
/// - A face split into three pieces yields three faces on the same plane, so
///   all three inherit the one ancestor. That is the answer you want.
/// - A drilled hole's wall lies on the tool's cylinder, so it inherits from the
///   drill. Also the answer you want.
/// - A fillet's blend surface is a torus that existed in neither input, so it
///   is correctly reported as created.
///
/// The trade is that two input faces on the *same* surface — the coplanar sides
/// of two boxes being unioned — are indistinguishable, and the first one wins.
/// Both are defensible ancestors of the merged face, so this is a genuine
/// ambiguity rather than a wrong answer, but it is a limitation.
class BrepProvenance {
  BrepProvenance._();

  /// Tags [result] in place from [sources], and returns it.
  static BrepSolid inherit(
    BrepSolid result,
    List<BrepSolid> sources, {
    required String op,
  }) {
    final faceTags = <String, BrepOrigin>{};
    final edgeTags = <String, BrepOrigin>{};

    for (final src in sources) {
      for (final f in src.faces) {
        final tag = f.origin;
        if (tag == null) continue;
        faceTags.putIfAbsent(surfaceKey(f.surface), () => tag);
      }
      for (final e in src.edges) {
        final tag = e.origin;
        if (tag == null) continue;
        edgeTags.putIfAbsent(curveKey(e.curve), () => tag);
      }
    }

    // Created entities still get distinct, propagatable identities, so a later
    // operation can inherit from a fillet surface the same way it inherits from
    // an original face. The id is allocated lazily: an operation that invents
    // nothing burns none.
    var opId = 0;
    var createdCount = 0;
    BrepOrigin created() {
      if (opId == 0) opId = BrepOriginIds.next();
      return BrepOrigin(original: opId, entity: createdCount++, createdBy: op);
    }

    for (final f in result.faces) {
      f.origin = faceTags[surfaceKey(f.surface)] ?? created();
    }
    for (final e in result.edges) {
      e.origin = edgeTags[curveKey(e.curve)] ?? created();
    }
    return result;
  }

  /// Same as [inherit] for the many operations that take a single solid.
  static BrepSolid inheritFrom(
    BrepSolid result,
    BrepSolid source, {
    required String op,
  }) => inherit(result, [source], op: op);

  // ── geometric identity ─────────────────────────────────────────────────────

  /// A key identifying the *unbounded* surface a face lies on.
  ///
  /// Trimming is deliberately ignored: the point is to recognise the same
  /// surface after an operation has re-cut it. Orientation is ignored too,
  /// since a plane and its back face are the same plane.
  static String surfaceKey(BrepSurface s) {
    switch (s) {
      case BrepOffsetSurface():
        return 'offset|${s.toJson()}';
      case BrepSplineSurface():
        return 'spline|${s.toJson()}';
      case BrepAffineSurface():
        return 'affine|${s.toJson()}';
      case BrepRuledSurface():
        return 'ruled|${s.toJson()}';
      case BrepPlane():
        // Canonical normal plus signed offset. Flipping the normal flips the
        // offset with it, so both facings hash alike.
        final n = _canonicalDirection(s.normal);
        return 'plane|${_vec(n)}|${_num(s.origin.dot(n))}';

      case BrepCylinder():
        final a = _canonicalDirection(s.axis);
        // Foot of the perpendicular from the world origin pins the axis line
        // without depending on where along it `origin` happens to sit.
        final foot = s.origin - a * s.origin.dot(a);
        return 'cyl|${_vec(a)}|${_num(s.radius)}|${_vec(foot)}';

      case BrepSphere():
        return 'sphere|${_vec(s.center)}|${_num(s.radius)}';

      case BrepCone():
        // A cone is not symmetric under an axis flip — reversing the axis swaps
        // base and top radius — so the axis is used as stored.
        return 'cone|${_vec(s.origin)}|${_vec(s.axis)}|${_num(s.radius)}'
            '|${_num(s.topRadius)}|${_num(s.height)}';

      case BrepTorus():
        final a = _canonicalDirection(s.axis);
        return 'torus|${_vec(s.center)}|${_vec(a)}'
            '|${_num(s.majorRadius)}|${_num(s.minorRadius)}';
    }
  }

  /// A key identifying the *unbounded* curve an edge lies on, so that a
  /// sub-segment left by a boolean still recognises its parent.
  static String curveKey(BrepCurve c) {
    switch (c) {
      case BrepHelix():
        return 'helix|${_vec(c.origin)}|${_vec(c.axis)}|${_vec(c.xDir)}|${_num(c.radius)}|${_num(c.lead)}|${_num(c.radiusSlope)}';
      case BrepSurfaceIntersectionCurve():
        return 'surface-intersection|${c.first.toJson()}|${c.second.toJson()}|${c.anchors}';
      case BrepRuledQuadricCurve():
        return 'ruled-quadric|${surfaceKey(c.surface)}|${surfaceKey(c.target)}|${c.branch}';
      case BrepPlaneSectionCurve():
        return 'section|${c.toJson()}';
      case BrepSplineCurve():
        return 'spline|${c.toJson()}';
      case BrepAffineCurve():
        return 'affine|${c.toJson()}';
      case BrepLine():
        final d = c.b - c.a;
        if (d.length2 < 1e-24) return 'point|${_vec(c.a)}';
        final dir = _canonicalDirection(d);
        final foot = c.a - dir * c.a.dot(dir);
        return 'line|${_vec(dir)}|${_vec(foot)}';

      case BrepCircle():
        final n = _canonicalDirection(c.normal);
        return 'circle|${_vec(c.center)}|${_vec(n)}|${_num(c.radius)}';

      case BrepIntersectionCurve():
        return 'intersection|${surfaceKey(c.carrier)}|${surfaceKey(c.target)}|${c.branch}';
      case BrepEllipse():
        final n = _canonicalDirection(c.normal);
        final x = _canonicalDirection(c.xDir);
        return 'ellipse|${_vec(c.center)}|${_vec(n)}|${_vec(x)}'
            '|${_num(c.rMajor)}|${_num(c.rMinor)}';
    }
  }

  /// Normalises [d] and picks the sign that makes its first significant
  /// component positive, so a direction and its opposite hash alike.
  static Vector3 _canonicalDirection(Vector3 d) {
    if (d.length2 < 1e-24) return Vector3.zero();
    final n = d.normalized();
    for (final c in [n.x, n.y, n.z]) {
      if (c.abs() > 1e-9) return c > 0 ? n : -n;
    }
    return n;
  }

  /// Quantises to 1e-6 so geometry that survived an operation's arithmetic
  /// still matches. Zeroes small values first, which also folds -0.0 into 0.0.
  static String _num(double x) => (x.abs() < 1e-9 ? 0.0 : x).toStringAsFixed(6);

  static String _vec(Vector3 v) => '${_num(v.x)},${_num(v.y)},${_num(v.z)}';
}
