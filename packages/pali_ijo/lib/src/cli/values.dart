import 'package:vector_math/vector_math_64.dart';

import '../brep/brep.dart';
import '../csg/boolean.dart';
import '../measure/measure.dart';
import '../mesh/mesh_op.dart';
import '../select/select.dart';
import '../sweep/sweep.dart';

/// Raised when an argument's *value* is wrong, as opposed to the argument
/// itself being unknown.
///
/// Separate from `args`' own [FormatException] so the runner can print usage
/// for one and not the other: a misspelled flag needs the flag list, a
/// malformed vector needs to be told what a vector looks like.
class CliValueException implements Exception {
  CliValueException(this.message);
  final String message;
  @override
  String toString() => message;
}

/// Parsers for the value syntax the CLI accepts.
///
/// The syntax is chosen so that every value is a single shell word needing no
/// quoting in the common case: components separated by commas, points by
/// semicolons. `--size 10,20,30` rather than `--dx 10 --dy 20 --dz 30`, and
/// `--path 0,0,0;0,0,50;30,0,50` rather than a file of points.
class CliValues {
  CliValues._();

  static double number(String raw, String what) {
    final v = double.tryParse(raw.trim());
    if (v == null) {
      throw CliValueException('--$what expects a number, got "$raw"');
    }
    if (!v.isFinite) {
      throw CliValueException('--$what must be finite, got "$raw"');
    }
    return v;
  }

  static double positive(String raw, String what) {
    final v = number(raw, what);
    if (v <= 0) {
      throw CliValueException('--$what must be greater than zero, got $v');
    }
    return v;
  }

  static int integer(String raw, String what) {
    final v = int.tryParse(raw.trim());
    if (v == null) {
      throw CliValueException('--$what expects a whole number, got "$raw"');
    }
    return v;
  }

  /// `x,y,z`, or a shorthand axis name.
  ///
  /// The names exist because `--axis z` is what a person types, and getting
  /// `--axis 0,0,1` wrong by one comma is a silent wrong answer rather than an
  /// error.
  static Vector3 vector(String raw, String what) {
    final named = switch (raw.trim().toLowerCase()) {
      'x' || '+x' => Vector3(1, 0, 0),
      '-x' => Vector3(-1, 0, 0),
      'y' || '+y' => Vector3(0, 1, 0),
      '-y' => Vector3(0, -1, 0),
      'z' || '+z' => Vector3(0, 0, 1),
      '-z' => Vector3(0, 0, -1),
      _ => null,
    };
    if (named != null) return named;

    final parts = raw.split(',');
    if (parts.length != 3) {
      throw CliValueException(
        '--$what expects "x,y,z" or an axis name (x, -y, +z), got "$raw"',
      );
    }
    return Vector3(
      number(parts[0], what),
      number(parts[1], what),
      number(parts[2], what),
    );
  }

  /// A non-zero direction. A zero vector has no direction, and normalising it
  /// yields NaN that would surface much later as unexplained geometry.
  static Vector3 direction(String raw, String what) {
    final v = vector(raw, what);
    if (v.length2 < 1e-24) {
      throw CliValueException('--$what must not be the zero vector');
    }
    return v.normalized();
  }

  /// `x,y,z;x,y,z;...`
  static List<Vector3> points(String raw, String what, {int atLeast = 2}) {
    final out = [
      for (final chunk in raw.split(';'))
        if (chunk.trim().isNotEmpty) vector(chunk, what),
    ];
    if (out.length < atLeast) {
      throw CliValueException(
        '--$what needs at least $atLeast points separated by ";", '
        'got ${out.length}',
      );
    }
    return out;
  }

  /// `r,h;r,h;...` — the (radius, height) outline a lathe turns.
  static List<(double, double)> outline(String raw, String what) {
    final out = <(double, double)>[];
    for (final chunk in raw.split(';')) {
      if (chunk.trim().isEmpty) continue;
      final parts = chunk.split(',');
      if (parts.length != 2) {
        throw CliValueException(
          '--$what expects "radius,height;radius,height;..." — each point '
          'has two numbers, got "$chunk"',
        );
      }
      out.add((number(parts[0], what), number(parts[1], what)));
    }
    if (out.length < 3) {
      throw CliValueException(
        '--$what needs at least 3 points to enclose an area, got '
        '${out.length}',
      );
    }
    return out;
  }

  /// A closed planar profile from a ring of points.
  static SweepProfile profile(String raw, String what) {
    final ring = points(raw, what, atLeast: 3);
    try {
      return SweepProfile.closed(ring);
    } on MeshOpException catch (e) {
      throw CliValueException('--$what is not a usable profile: ${e.message}');
    }
  }

  /// `minx,miny,minz;maxx,maxy,maxz`
  static IjoBounds bounds(String raw, String what) {
    final pts = points(raw, what, atLeast: 2);
    if (pts.length != 2) {
      throw CliValueException(
        '--$what expects two corners, "min;max", got ${pts.length} points',
      );
    }
    return IjoBounds(
      Vector3(
        pts[0].x < pts[1].x ? pts[0].x : pts[1].x,
        pts[0].y < pts[1].y ? pts[0].y : pts[1].y,
        pts[0].z < pts[1].z ? pts[0].z : pts[1].z,
      ),
      Vector3(
        pts[0].x > pts[1].x ? pts[0].x : pts[1].x,
        pts[0].y > pts[1].y ? pts[0].y : pts[1].y,
        pts[0].z > pts[1].z ? pts[0].z : pts[1].z,
      ),
    );
  }

  static int booleanOp(String raw) => switch (raw.trim().toLowerCase()) {
        'union' || 'add' || 'or' => MeshBoolean.unionOp,
        'subtract' || 'sub' || 'difference' || 'minus' =>
          MeshBoolean.subtractOp,
        'intersect' || 'and' || 'common' => MeshBoolean.intersectOp,
        _ => throw CliValueException(
            '--op expects union, subtract or intersect, got "$raw"',
          ),
      };
}

/// Parses the `--faces` and `--edges` selector language.
///
/// The point of this grammar is that a shell script should be able to say
/// which faces it means without first running a query to learn their indices.
/// `pali_ijo fillet --edges convex` is stable across a change to the model;
/// `--edges 3,7` is not.
///
/// Terms are joined with `+` for intersection and `/` for union, and prefixed
/// with `!` to negate:
///
///     top                     the topmost face
///     planar+!top             every flat face except the top
///     circular/linear:z       round rims, or edges running along Z
///     created:fillet          surface the fillet invented
///
/// `+` binds tighter than `/`, as `*` does over `+` in arithmetic, so
/// `a+b/c` is `(a and b) or c`.
class CliSelectors {
  CliSelectors._();

  static const faceHelp = 'all | INDICES | planar | plane | cylinder | '
      'sphere | cone | torus | cylindrical:R | normal:DIR | top | bottom | '
      'extreme:DIR | within:MIN;MAX | created[:OP] | original:ID. '
      'Combine with + (and), / (or), ! (not).';

  static const edgeHelp = 'all | INDICES | line | circle | ellipse | '
      'circular[:R] | linear[:DIR] | convex | concave | longer:LEN | '
      'within:MIN;MAX | of-face:FACESPEC | created[:OP] | original:ID. '
      'Combine with + (and), / (or), ! (not).';

  static FaceSelect faces(String spec) =>
      _union(spec, 'faces', _faceTerm, (a, b) => a.or(b), (a, b) => a.and(b));

  static EdgeSelect edges(String spec) =>
      _union(spec, 'edges', _edgeTerm, (a, b) => a.or(b), (a, b) => a.and(b));

  static BrepFaceSelect brepFaces(String spec) => _union(
        spec,
        'faces',
        _brepFaceTerm,
        (a, b) => a.or(b),
        (a, b) => a.and(b),
      );

  static BrepEdgeSelect brepEdges(String spec) => _union(
        spec,
        'edges',
        _brepEdgeTerm,
        (a, b) => a.or(b),
        (a, b) => a.and(b),
      );

  /// Shared shape of both parsers: split on `/`, then on `+`, then parse each
  /// term. Written once because the two grammars differ only in their terms.
  static T _union<T>(
    String spec,
    String what,
    T Function(String term, String what) term,
    T Function(T, T) or,
    T Function(T, T) and,
  ) {
    if (spec.trim().isEmpty) {
      throw CliValueException('--$what is empty');
    }
    T? result;
    for (final group in spec.split('/')) {
      T? conjunction;
      for (final piece in group.split('+')) {
        final parsed = term(piece.trim(), what);
        conjunction =
            conjunction == null ? parsed : and(conjunction, parsed);
      }
      if (conjunction == null) continue;
      result = result == null ? conjunction : or(result, conjunction);
    }
    if (result == null) {
      throw CliValueException('--$what is empty');
    }
    return result;
  }

  /// Splits `name:argument`, tolerating a missing argument.
  static (String, String?) _split(String term, String what) {
    if (term.isEmpty) throw CliValueException('--$what has an empty term');
    final colon = term.indexOf(':');
    return colon < 0
        ? (term.toLowerCase(), null)
        : (
            term.substring(0, colon).toLowerCase(),
            term.substring(colon + 1),
          );
  }

  static String _require(String? arg, String name, String what) =>
      arg ??
      (throw CliValueException(
        '--$what term "$name" needs an argument, as in "$name:..."',
      ));

  static FaceSelect _faceTerm(String raw, String what) {
    if (raw.startsWith('!')) {
      return _faceTerm(raw.substring(1).trim(), what).not;
    }
    final (name, arg) = _split(raw, what);

    // A bare list of numbers is the escape hatch: whatever the grammar cannot
    // say, indices can.
    if (arg == null && _looksNumeric(name)) {
      return FaceSelect.indices(_indices(name, what));
    }

    return switch (name) {
      'all' => FaceSelect.all,
      'planar' || 'plane' || 'flat' => FaceSelect.planar(),
      'cylinder' ||
      'sphere' ||
      'spherical' ||
      'cone' ||
      'conical' ||
      'torus' ||
      'toroidal' ||
      'curved' ||
      'cylindrical' =>
        FaceSelect.curved(),
      'normal' => FaceSelect.normal(
          CliValues.direction(_require(arg, name, what), what),
        ),
      'extreme' => FaceSelect.extreme(
          CliValues.direction(_require(arg, name, what), what),
        ),
      'top' => FaceSelect.extreme(Vector3(0, 0, 1)),
      'bottom' => FaceSelect.extreme(Vector3(0, 0, -1)),
      'within' => FaceSelect.within(
          CliValues.bounds(_require(arg, name, what), what),
        ),
      'created' || 'inherited' || 'original' => FaceSelect.all,
      'indices' => FaceSelect.indices(
          _indices(_require(arg, name, what), what),
        ),
      'none' => FaceSelect.all.not,
      _ => throw CliValueException(
          'Unknown --$what term "$name". Expected: $faceHelp',
        ),
    };
  }

  static EdgeSelect _edgeTerm(String raw, String what) {
    if (raw.startsWith('!')) {
      return _edgeTerm(raw.substring(1).trim(), what).not;
    }
    final (name, arg) = _split(raw, what);

    if (arg == null && _looksNumeric(name)) {
      return EdgeSelect.indices(_indices(name, what));
    }

    return switch (name) {
      'all' => EdgeSelect.all,
      'line' || 'straight' => EdgeSelect.linear(),
      'circle' || 'ellipse' || 'curved' || 'circular' => EdgeSelect.circular(),
      'linear' => EdgeSelect.linear(
          direction: arg == null ? null : CliValues.direction(arg, what),
        ),
      'convex' => EdgeSelect.convex(),
      'concave' => EdgeSelect.concave(),
      'longer' => EdgeSelect.longer(
          CliValues.number(_require(arg, name, what), what),
        ),
      'within' => EdgeSelect.within(
          CliValues.bounds(_require(arg, name, what), what),
        ),
      'of-face' || 'offace' => EdgeSelect.ofFace(
          faces(_require(arg, name, what)),
        ),
      'created' || 'inherited' || 'original' => EdgeSelect.all,
      'indices' => EdgeSelect.indices(
          _indices(_require(arg, name, what), what),
        ),
      'none' => EdgeSelect.all.not,
      _ => throw CliValueException(
          'Unknown --$what term "$name". Expected: $edgeHelp',
        ),
    };
  }

  static BrepFaceSelect _brepFaceTerm(String raw, String what) {
    if (raw.startsWith('!')) {
      return _brepFaceTerm(raw.substring(1).trim(), what).not;
    }
    final (name, arg) = _split(raw, what);
    if (arg == null && _looksNumeric(name)) {
      return BrepFaceSelect.indices(_indices(name, what));
    }
    return switch (name) {
      'all' => BrepFaceSelect.all,
      'planar' || 'plane' || 'flat' => BrepFaceSelect.planar(),
      'cylinder' || 'cylindrical' => BrepFaceSelect.ofType<BrepCylinder>(),
      'sphere' || 'spherical' => BrepFaceSelect.ofType<BrepSphere>(),
      'cone' || 'conical' => BrepFaceSelect.ofType<BrepCone>(),
      'torus' || 'toroidal' => BrepFaceSelect.ofType<BrepTorus>(),
      'normal' => BrepFaceSelect.normal(
          CliValues.direction(_require(arg, name, what), what),
        ),
      'extreme' => BrepFaceSelect.extreme(
          CliValues.direction(_require(arg, name, what), what),
        ),
      'top' => BrepFaceSelect.extreme(Vector3(0, 0, 1)),
      'bottom' => BrepFaceSelect.extreme(Vector3(0, 0, -1)),
      'within' => BrepFaceSelect.within(
          CliValues.bounds(_require(arg, name, what), what),
        ),
      'created' => BrepFaceSelect.created(by: arg),
      'indices' => BrepFaceSelect.indices(
          _indices(_require(arg, name, what), what),
        ),
      'none' => BrepFaceSelect.all.not,
      _ => throw CliValueException(
          'Unknown --$what term "$name". Expected: $faceHelp',
        ),
    };
  }

  static BrepEdgeSelect _brepEdgeTerm(String raw, String what) {
    if (raw.startsWith('!')) {
      return _brepEdgeTerm(raw.substring(1).trim(), what).not;
    }
    final (name, arg) = _split(raw, what);
    if (arg == null && _looksNumeric(name)) {
      return BrepEdgeSelect.indices(_indices(name, what));
    }
    return switch (name) {
      'all' => BrepEdgeSelect.all,
      'line' || 'straight' => BrepEdgeSelect.linear(),
      'circle' || 'circular' => BrepEdgeSelect.circular(),
      'linear' => BrepEdgeSelect.linear(
          direction: arg == null ? null : CliValues.direction(arg, what),
        ),
      'convex' => BrepEdgeSelect.convex(),
      'concave' => BrepEdgeSelect.concave(),
      'longer' => BrepEdgeSelect.longerThan(
          CliValues.number(_require(arg, name, what), what),
        ),
      'within' => BrepEdgeSelect.within(
          CliValues.bounds(_require(arg, name, what), what),
        ),
      'of-face' || 'offace' => BrepEdgeSelect.ofFace(
          brepFaces(_require(arg, name, what)),
        ),
      'created' => BrepEdgeSelect.created(by: arg),
      'indices' => BrepEdgeSelect.indices(
          _indices(_require(arg, name, what), what),
        ),
      'none' => BrepEdgeSelect.all.not,
      _ => throw CliValueException(
          'Unknown --$what term "$name". Expected: $edgeHelp',
        ),
    };
  }

  static bool _looksNumeric(String s) =>
      s.isNotEmpty && RegExp(r'^[0-9]+(,[0-9]+)*$').hasMatch(s);

  static List<int> _indices(String raw, String what) {
    final out = <int>[];
    for (final chunk in raw.split(',')) {
      if (chunk.trim().isEmpty) continue;
      final i = CliValues.integer(chunk, what);
      if (i < 0) {
        throw CliValueException('--$what index must not be negative, got $i');
      }
      out.add(i);
    }
    if (out.isEmpty) {
      throw CliValueException('--$what has no indices');
    }
    return out;
  }
}
