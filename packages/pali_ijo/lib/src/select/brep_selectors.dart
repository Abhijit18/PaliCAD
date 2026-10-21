import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../brep/brep.dart';
import '../measure/bounds.dart';

/// A predicate over the faces of a solid, resolvable to indices.
///
/// The kernel's operations take `List<int>` face and edge indices, and will
/// keep doing so — they are the honest low-level currency. But indices are a
/// terrible thing to *write*, because `filletSolid(solid, [3, 7], ...)` says
/// nothing about which edges those are and stops being true the moment the
/// solid changes.
///
/// A selector says what you meant. `BrepFaceSelect.normal(Vector3(0, 0, 1))` still
/// finds the top of the box after the box has been unioned with something,
/// resized, or reloaded from a file — because it is a question about the
/// geometry, not a memory of an index.
///
/// Selectors compose with [and], [or] and [not], and are resolved by
/// [resolve]. They are declarative and side-effect free, so the same selector
/// can be stored in a document and re-resolved later, which is what makes a
/// parametric history possible.
abstract class BrepFaceSelect {
  const BrepFaceSelect();

  /// Whether [face], at index [index] of [solid], is selected.
  bool matches(BrepSolid solid, BrepFace face, int index);

  /// Indices of the matching faces, ascending.
  List<int> resolve(BrepSolid solid) => [
    for (var i = 0; i < solid.faces.length; i++)
      if (matches(solid, solid.faces[i], i)) i,
  ];

  /// Every face.
  static const BrepFaceSelect all = _FacePredicate(_alwaysFace, 'all');

  /// Faces at the given indices. The escape hatch back to raw indices, for
  /// when the caller genuinely does have a picked index from a viewport.
  static BrepFaceSelect indices(Iterable<int> indices) {
    final set = indices.toSet();
    return _FacePredicate(
      (solid, face, i) => set.contains(i),
      'indices(${set.length})',
    );
  }

  /// Faces whose outward normal points within [toleranceDeg] of [direction].
  ///
  /// The normal is sampled at the middle of the face's parameter domain, so
  /// this is only meaningful for surfaces with a constant normal — planes.
  /// A cylinder's normal varies over its extent, and asking which way a
  /// cylinder faces is not a well-posed question.
  static BrepFaceSelect normal(Vector3 direction, {double toleranceDeg = 1.0}) {
    final d = direction.normalized();
    final cosLimit = math.cos(toleranceDeg * math.pi / 180);
    return _FacePredicate((solid, face, i) {
      final n = _planarNormal(face);
      return n != null && n.dot(d) >= cosLimit;
    }, 'normal(${_fmtVec(d)}, ±$toleranceDeg°)');
  }

  /// Planar faces, optionally only those on the plane through [through] with
  /// the given [normal].
  static BrepFaceSelect planar({Vector3? through, Vector3? normal}) {
    if (through == null && normal == null) {
      return _FacePredicate(
        (solid, face, i) => face.surface is BrepPlane,
        'planar',
      );
    }
    return _FacePredicate(
      (solid, face, i) {
        final s = face.surface;
        if (s is! BrepPlane) return false;
        if (normal != null &&
            s.normal.dot(normal.normalized()).abs() < 0.9999) {
          return false;
        }
        if (through != null && s.height(through).abs() > 1e-6) return false;
        return true;
      },
      'planar(${through == null ? '' : 'through ${_fmtVec(through)}'}'
      '${normal == null ? '' : ' normal ${_fmtVec(normal)}'})',
    );
  }

  /// Faces whose supporting surface is of type [T].
  ///
  /// `BrepFaceSelect.ofType<BrepCylinder>()` selects every cylindrical face, which
  /// is how you find the walls of every hole in a part at once.
  static BrepFaceSelect ofType<T extends BrepSurface>() =>
      _FacePredicate((solid, face, i) => face.surface is T, 'ofType<$T>');

  /// Cylindrical faces, optionally restricted by radius.
  static BrepFaceSelect cylindrical({
    double? radius,
    double tolerance = 1e-6,
  }) => _FacePredicate((solid, face, i) {
    final s = face.surface;
    if (s is! BrepCylinder) return false;
    return radius == null || (s.radius - radius).abs() <= tolerance;
  }, 'cylindrical(${radius ?? 'any'})');

  /// Faces whose every vertex lies inside [box].
  static BrepFaceSelect within(IjoBounds box, {double tolerance = 1e-6}) =>
      _FacePredicate((solid, face, i) {
        final pts = _faceVertices(solid, face);
        if (pts.isEmpty) return false;
        return pts.every((p) => box.containsPoint(p, tolerance: tolerance));
      }, 'within($box)');

  /// Faces whose vertex centroid is the furthest along [direction].
  ///
  /// This is the "top face" selector, and the reason it takes a direction
  /// rather than being called `top` is that a boat hull has no top.
  static BrepFaceSelect extreme(Vector3 direction, {double tolerance = 1e-6}) {
    final d = direction.normalized();
    return _FaceExtreme(d, tolerance);
  }

  /// Faces descended from the face at [index] of [solid].
  ///
  /// Resolve against a *later* solid to find where that face ended up. If a
  /// boolean split it in three, all three come back.
  static BrepFaceSelect descendedFrom(BrepSolid solid, int index) {
    if (index < 0 || index >= solid.faces.length) return _never;
    final tag = solid.faces[index].origin;
    if (tag == null) return _never;
    return withOrigin(tag);
  }

  /// Faces carrying exactly [origin].
  static BrepFaceSelect withOrigin(BrepOrigin origin) => _FacePredicate(
    (solid, face, i) => face.origin == origin,
    'withOrigin($origin)',
  );

  /// Faces belonging to original [id], whatever their entity index.
  static BrepFaceSelect fromOriginal(int id) => _FacePredicate(
    (solid, face, i) => face.origin?.original == id,
    'fromOriginal($id)',
  );

  /// Faces invented by an operation rather than inherited.
  ///
  /// With [by] set, only those from that operation — `createdBy: 'fillet'`
  /// selects the blend surfaces and nothing else.
  static BrepFaceSelect created({String? by}) =>
      _FacePredicate((solid, face, i) {
        final o = face.origin;
        if (o == null || o.isInherited) return false;
        return by == null || o.createdBy == by;
      }, 'created(${by ?? 'any'})');

  BrepFaceSelect and(BrepFaceSelect other) => _FacePredicate(
    (solid, face, i) =>
        matches(solid, face, i) && other.matches(solid, face, i),
    '($this and $other)',
  );

  BrepFaceSelect or(BrepFaceSelect other) => _FacePredicate(
    (solid, face, i) =>
        matches(solid, face, i) || other.matches(solid, face, i),
    '($this or $other)',
  );

  BrepFaceSelect get not =>
      _FacePredicate((solid, face, i) => !matches(solid, face, i), 'not $this');

  static const BrepFaceSelect _never = _FacePredicate(_neverFace, 'none');
}

/// A predicate over the edges of a solid, resolvable to indices.
///
/// The counterpart to [BrepFaceSelect], and the more useful of the two in practice
/// because fillet and chamfer both take edges. `BrepEdgeSelect.ofFace(...)`
/// followed by a fillet is "round the top", said once and still true later.
abstract class BrepEdgeSelect {
  const BrepEdgeSelect();

  bool matches(BrepSolid solid, BrepEdge edge, int index);

  List<int> resolve(BrepSolid solid) => [
    for (var i = 0; i < solid.edges.length; i++)
      if (matches(solid, solid.edges[i], i)) i,
  ];

  static const BrepEdgeSelect all = _EdgePredicate(_alwaysEdge, 'all');

  static BrepEdgeSelect indices(Iterable<int> indices) {
    final set = indices.toSet();
    return _EdgePredicate(
      (solid, edge, i) => set.contains(i),
      'indices(${set.length})',
    );
  }

  /// Edges bounding any face matched by [faces].
  static BrepEdgeSelect ofFace(BrepFaceSelect faces) => _EdgesOfFace(faces);

  /// Edges whose supporting curve is of type [T].
  ///
  /// `BrepEdgeSelect.ofType<BrepCircle>()` is every circular rim.
  static BrepEdgeSelect ofType<T extends BrepCurve>() =>
      _EdgePredicate((solid, edge, i) => edge.curve is T, 'ofType<$T>');

  /// Circular edges, optionally restricted by radius.
  static BrepEdgeSelect circular({double? radius, double tolerance = 1e-6}) =>
      _EdgePredicate((solid, edge, i) {
        final c = edge.curve;
        if (c is! BrepCircle) return false;
        return radius == null || (c.radius - radius).abs() <= tolerance;
      }, 'circular(${radius ?? 'any'})');

  /// Straight edges, optionally only those parallel to [direction].
  static BrepEdgeSelect linear({
    Vector3? direction,
    double toleranceDeg = 1.0,
  }) {
    final d = direction?.normalized();
    final cosLimit = math.cos(toleranceDeg * math.pi / 180);
    return _EdgePredicate((solid, edge, i) {
      final c = edge.curve;
      if (c is! BrepLine) return false;
      if (d == null) return true;
      final t = c.b - c.a;
      if (t.length2 < 1e-24) return false;
      return t.normalized().dot(d).abs() >= cosLimit;
    }, 'linear(${d == null ? 'any' : _fmtVec(d)})');
  }

  /// Edges with both endpoints inside [box].
  static BrepEdgeSelect within(
    IjoBounds box, {
    double tolerance = 1e-6,
  }) => _EdgePredicate((solid, edge, i) {
    if (edge.v0 >= solid.vertices.length || edge.v1 >= solid.vertices.length) {
      return false;
    }
    return box.containsPoint(solid.vertices[edge.v0], tolerance: tolerance) &&
        box.containsPoint(solid.vertices[edge.v1], tolerance: tolerance);
  }, 'within($box)');

  /// Edges longer than [length], by arc length along the curve.
  static BrepEdgeSelect longerThan(double length) => _EdgePredicate(
    (solid, edge, i) => _edgeLength(edge) > length,
    'longerThan($length)',
  );

  /// Convex edges: those where the two adjacent faces turn away from the
  /// material, which are the ones a fillet rounds off rather than fills in.
  ///
  /// Edges that are not shared by exactly two faces are excluded, since
  /// convexity is not defined for them.
  static BrepEdgeSelect convex({double toleranceDeg = 0.5}) =>
      _EdgeConvexity(true, toleranceDeg);

  /// Concave edges: inside corners, which a fillet fills in.
  static BrepEdgeSelect concave({double toleranceDeg = 0.5}) =>
      _EdgeConvexity(false, toleranceDeg);

  static BrepEdgeSelect descendedFrom(BrepSolid solid, int index) {
    if (index < 0 || index >= solid.edges.length) return _never;
    final tag = solid.edges[index].origin;
    if (tag == null) return _never;
    return withOrigin(tag);
  }

  static BrepEdgeSelect withOrigin(BrepOrigin origin) => _EdgePredicate(
    (solid, edge, i) => edge.origin == origin,
    'withOrigin($origin)',
  );

  static BrepEdgeSelect fromOriginal(int id) => _EdgePredicate(
    (solid, edge, i) => edge.origin?.original == id,
    'fromOriginal($id)',
  );

  static BrepEdgeSelect created({String? by}) =>
      _EdgePredicate((solid, edge, i) {
        final o = edge.origin;
        if (o == null || o.isInherited) return false;
        return by == null || o.createdBy == by;
      }, 'created(${by ?? 'any'})');

  BrepEdgeSelect and(BrepEdgeSelect other) => _EdgePredicate(
    (solid, edge, i) =>
        matches(solid, edge, i) && other.matches(solid, edge, i),
    '($this and $other)',
  );

  BrepEdgeSelect or(BrepEdgeSelect other) => _EdgePredicate(
    (solid, edge, i) =>
        matches(solid, edge, i) || other.matches(solid, edge, i),
    '($this or $other)',
  );

  BrepEdgeSelect get not =>
      _EdgePredicate((solid, edge, i) => !matches(solid, edge, i), 'not $this');

  static const BrepEdgeSelect _never = _EdgePredicate(_neverEdge, 'none');
}

// ── implementations ──────────────────────────────────────────────────────────

typedef _FaceTest = bool Function(BrepSolid solid, BrepFace face, int index);
typedef _EdgeTest = bool Function(BrepSolid solid, BrepEdge edge, int index);

bool _alwaysFace(BrepSolid s, BrepFace f, int i) => true;
bool _neverFace(BrepSolid s, BrepFace f, int i) => false;
bool _alwaysEdge(BrepSolid s, BrepEdge e, int i) => true;
bool _neverEdge(BrepSolid s, BrepEdge e, int i) => false;

class _FacePredicate extends BrepFaceSelect {
  const _FacePredicate(this._test, this._label);

  final _FaceTest _test;
  final String _label;

  @override
  bool matches(BrepSolid solid, BrepFace face, int index) =>
      _test(solid, face, index);

  @override
  String toString() => _label;
}

class _EdgePredicate extends BrepEdgeSelect {
  const _EdgePredicate(this._test, this._label);

  final _EdgeTest _test;
  final String _label;

  @override
  bool matches(BrepSolid solid, BrepEdge edge, int index) =>
      _test(solid, edge, index);

  @override
  String toString() => _label;
}

/// Needs to see the whole solid before it can judge any one face, so it
/// overrides [resolve] and caches the winning distance per solid.
class _FaceExtreme extends BrepFaceSelect {
  _FaceExtreme(this._dir, this._tolerance);

  final Vector3 _dir;
  final double _tolerance;

  BrepSolid? _for;
  double _best = double.negativeInfinity;

  double? _score(BrepSolid solid, BrepFace face) {
    final pts = _faceVertices(solid, face);
    if (pts.isEmpty) return null;
    var sum = 0.0;
    for (final p in pts) {
      sum += p.dot(_dir);
    }
    return sum / pts.length;
  }

  void _prime(BrepSolid solid) {
    if (identical(_for, solid)) return;
    _for = solid;
    _best = double.negativeInfinity;
    for (final f in solid.faces) {
      final s = _score(solid, f);
      if (s != null && s > _best) _best = s;
    }
  }

  @override
  bool matches(BrepSolid solid, BrepFace face, int index) {
    _prime(solid);
    final s = _score(solid, face);
    return s != null && s >= _best - _tolerance;
  }

  @override
  String toString() => 'extreme(${_fmtVec(_dir)})';
}

class _EdgesOfFace extends BrepEdgeSelect {
  _EdgesOfFace(this._faces);

  final BrepFaceSelect _faces;

  @override
  bool matches(BrepSolid solid, BrepEdge edge, int index) =>
      resolve(solid).contains(index);

  @override
  List<int> resolve(BrepSolid solid) {
    final out = <int>{};
    for (final fi in _faces.resolve(solid)) {
      for (final loop in solid.faces[fi].loops) {
        for (final co in loop.coedges) {
          out.add(co.edge);
        }
      }
    }
    return out.toList()..sort();
  }

  @override
  String toString() => 'ofFace($_faces)';
}

/// Convexity is a property of the pair of faces meeting at an edge, so this
/// builds the edge → faces map once per solid.
class _EdgeConvexity extends BrepEdgeSelect {
  _EdgeConvexity(this._wantConvex, this._toleranceDeg);

  final bool _wantConvex;
  final double _toleranceDeg;

  BrepSolid? _for;
  Map<int, List<int>> _facesOfEdge = const {};

  void _prime(BrepSolid solid) {
    if (identical(_for, solid)) return;
    _for = solid;
    final map = <int, List<int>>{};
    for (var fi = 0; fi < solid.faces.length; fi++) {
      for (final loop in solid.faces[fi].loops) {
        for (final co in loop.coedges) {
          (map[co.edge] ??= []).add(fi);
        }
      }
    }
    _facesOfEdge = map;
  }

  @override
  bool matches(BrepSolid solid, BrepEdge edge, int index) {
    _prime(solid);
    final adj = _facesOfEdge[index];
    if (adj == null || adj.length != 2) return false;

    final f0 = solid.faces[adj[0]];
    final f1 = solid.faces[adj[1]];
    final n0 = _planarNormal(f0);
    final n1 = _planarNormal(f1);
    if (n0 == null || n1 == null) return false;

    // Tangent faces are neither convex nor concave, and blending one is
    // meaningless.
    final sinLimit = math.sin(_toleranceDeg * math.pi / 180);
    if (n0.cross(n1).length < sinLimit) return false;

    // Each face is asked which side of the *other* face's plane it lies on.
    // At a convex edge — a box corner — each face falls behind the other's
    // outward normal; at a concave edge — the inside corner of a step — each
    // falls in front of it. This is local and exact for planes, which is why
    // it beats comparing against the solid's centroid: an L-shaped part's
    // centroid sits in a position that tells you nothing about its step.
    final s0 = _sideOf(solid, f0, n1, f1);
    final s1 = _sideOf(solid, f1, n0, f0);
    if (s0 == null || s1 == null || s0 != s1) return false;
    return s0 == _wantConvex;
  }

  /// Whether [face] lies behind [otherNormal] — the convex arrangement —
  /// judged from the point of its own vertices furthest from [other]'s plane.
  ///
  /// The furthest point rather than the average, because the vertices shared
  /// with [other] sit exactly on its plane and would otherwise drag a shallow
  /// face's average to nearly zero.
  static bool? _sideOf(
    BrepSolid solid,
    BrepFace face,
    Vector3 otherNormal,
    BrepFace other,
  ) {
    final plane = other.surface;
    if (plane is! BrepPlane) return null;
    final onPlane = plane.origin;

    var best = 0.0;
    for (final p in _faceVertices(solid, face)) {
      final h = (p - onPlane).dot(otherNormal);
      if (h.abs() > best.abs()) best = h;
    }
    if (best.abs() < 1e-9) return null;
    return best < 0;
  }

  @override
  String toString() => _wantConvex ? 'convex' : 'concave';
}

// ── shared helpers ───────────────────────────────────────────────────────────

/// The outward normal of a face with a constant one, else null.
Vector3? _planarNormal(BrepFace face) {
  final s = face.surface;
  if (s is! BrepPlane) return null;
  return face.reversed ? -s.normal : s.normal.clone();
}

List<Vector3> _faceVertices(BrepSolid solid, BrepFace face) {
  final seen = <int>{};
  final out = <Vector3>[];
  for (final loop in face.loops) {
    for (final co in loop.coedges) {
      if (co.edge < 0 || co.edge >= solid.edges.length) continue;
      final e = solid.edges[co.edge];
      for (final vi in [e.v0, e.v1]) {
        if (vi >= 0 && vi < solid.vertices.length && seen.add(vi)) {
          out.add(solid.vertices[vi]);
        }
      }
    }
  }
  return out;
}

double _edgeLength(BrepEdge edge) {
  final c = edge.curve;
  switch (c) {
    case BrepHelix():
      return c.arcLength;
    case BrepSurfaceIntersectionCurve():
    case BrepRuledQuadricCurve():
    case BrepPlaneSectionCurve():
    case BrepSplineCurve():
    case BrepAffineCurve():
    case BrepIntersectionCurve():
      var length = 0.0;
      var last = c.evaluate(c.t0);
      for (var i = 1; i <= 256; i++) {
        final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 256);
        length += (p - last).length;
        last = p;
      }
      return length;
    case BrepLine():
      return (c.b - c.a).length;
    case BrepCircle():
      return c.radius * (c.t1 - c.t0).abs();
    case BrepEllipse():
      // Ramanujan's approximation, scaled to the swept fraction. Good to
      // better than 1e-5 relative, which is far past what a length filter
      // needs to decide.
      final a = c.rMajor;
      final b = c.rMinor;
      final h = ((a - b) * (a - b)) / ((a + b) * (a + b));
      final full =
          math.pi * (a + b) * (1 + (3 * h) / (10 + math.sqrt(4 - 3 * h)));
      return full * ((c.t1 - c.t0).abs() / (2 * math.pi));
  }
}

String _fmtVec(Vector3 v) =>
    '${v.x.toStringAsFixed(3)}, '
    '${v.y.toStringAsFixed(3)}, ${v.z.toStringAsFixed(3)}';
