import 'package:vector_math/vector_math_64.dart';

import 'geom.dart';
import 'origin.dart';

/// One use of an edge in a face loop.
class BrepCoedge {
  const BrepCoedge(this.edge, {this.reversed = false});

  final int edge;
  final bool reversed;

  Map<String, dynamic> toJson() => {'e': edge, 'r': reversed};

  factory BrepCoedge.fromJson(Map<String, dynamic> json) =>
      BrepCoedge(json['e'] as int, reversed: json['r'] as bool? ?? false);
}

class BrepLoop {
  const BrepLoop(this.coedges);

  final List<BrepCoedge> coedges;

  Map<String, dynamic> toJson() => {
    'c': [for (final c in coedges) c.toJson()],
  };

  factory BrepLoop.fromJson(Map<String, dynamic> json) => BrepLoop([
    for (final c in json['c'] as List)
      BrepCoedge.fromJson(c as Map<String, dynamic>),
  ]);
}

class BrepEdge {
  BrepEdge({
    required this.v0,
    required this.v1,
    required this.curve,
    this.origin,
  });

  int v0;
  int v1;
  BrepCurve curve;

  /// Ancestry tag. See [BrepOrigin].
  BrepOrigin? origin;

  Map<String, dynamic> toJson() => {
    'a': v0,
    'b': v1,
    'c': curve.toJson(),
    if (origin != null) 'g': origin!.toJson(),
  };

  factory BrepEdge.fromJson(Map<String, dynamic> json) => BrepEdge(
    v0: json['a'] as int,
    v1: json['b'] as int,
    curve: BrepCurve.fromJson(json['c'] as Map<String, dynamic>),
    origin: json['g'] == null
        ? null
        : BrepOrigin.fromJson(json['g'] as Map<String, dynamic>),
  );
}

class BrepFace {
  BrepFace({
    required this.surface,
    required this.loops,
    this.reversed = false,
    this.origin,
  });

  BrepSurface surface;
  List<BrepLoop> loops;
  bool reversed;

  /// Ancestry tag. See [BrepOrigin].
  BrepOrigin? origin;

  Vector3 outwardNormal(double u, double v) {
    final n = surface.normalAt(u, v);
    return reversed ? -n : n;
  }

  Map<String, dynamic> toJson() => {
    's': surface.toJson(),
    'l': [for (final l in loops) l.toJson()],
    'r': reversed,
    if (origin != null) 'g': origin!.toJson(),
  };

  factory BrepFace.fromJson(Map<String, dynamic> json) => BrepFace(
    surface: BrepSurface.fromJson(json['s'] as Map<String, dynamic>),
    loops: [
      for (final l in json['l'] as List)
        BrepLoop.fromJson(l as Map<String, dynamic>),
    ],
    reversed: json['r'] as bool? ?? false,
    origin: json['g'] == null
        ? null
        : BrepOrigin.fromJson(json['g'] as Map<String, dynamic>),
  );
}

/// Closed analytic solid: vertices, edges with curves, faces with surfaces.
class BrepSolid {
  BrepSolid({required this.vertices, required this.edges, required this.faces});

  final List<Vector3> vertices;
  final List<BrepEdge> edges;
  final List<BrepFace> faces;

  BrepSolid clone() => BrepSolid.fromJson(toJson());

  /// Groups disjoint solids without replacing their analytic boundaries.
  /// This is a compound, not a union: callers must resolve overlapping parts.
  factory BrepSolid.compound(Iterable<BrepSolid> solids) {
    final b = BrepBuilder();
    for (final solid in solids) {
      final vertexOffset = b.vertices.length, edgeOffset = b.edges.length;
      for (final v in solid.vertices) {
        b.vertex(v);
      }
      for (final e in solid.edges) {
        b.edge(
          e.v0 + vertexOffset,
          e.v1 + vertexOffset,
          e.curve,
          origin: e.origin,
        );
      }
      for (final f in solid.faces) {
        b.face(
          f.surface,
          [
            for (final loop in f.loops)
              BrepLoop([
                for (final c in loop.coedges)
                  BrepCoedge(c.edge + edgeOffset, reversed: c.reversed),
              ]),
          ],
          reversed: f.reversed,
          origin: f.origin,
        );
      }
    }
    return b.build();
  }

  int get faceCount => faces.length;

  /// The id this solid is an original of, or null if it has been modified.
  ///
  /// Non-null only while every face and edge
  /// still sits at the index it was born at, under one shared original.
  int? get originalId {
    if (faces.isEmpty) return null;
    final first = faces.first.origin;
    if (first == null || first.isCreated) return null;
    final id = first.original;
    for (var i = 0; i < faces.length; i++) {
      final o = faces[i].origin;
      if (o == null || o.isCreated || o.original != id || o.entity != i) {
        return null;
      }
    }
    return id;
  }

  /// A copy declared to be a fresh original, forgetting all prior ancestry.
  ///
  /// Use it when a result should be treated
  /// as a new part rather than as a modification — the point at which you stop
  /// caring that this shape used to be a box.
  BrepSolid asOriginal({int? id}) {
    final out = clone();
    out.stampOriginal(id: id);
    return out;
  }

  /// Tags every face and edge in place as belonging to original [id].
  void stampOriginal({int? id}) {
    final oid = id ?? BrepOriginIds.next();
    for (var i = 0; i < faces.length; i++) {
      faces[i].origin = BrepOrigin(original: oid, entity: i);
    }
    for (var i = 0; i < edges.length; i++) {
      edges[i].origin = BrepOrigin(original: oid, entity: i);
    }
  }

  /// Tags only the entities that have no tag yet, leaving inherited ones alone.
  ///
  /// This is what [BrepBuilder.build] calls, so the invariant "every solid
  /// carries provenance" holds without an operation having to remember to ask.
  /// Costs nothing and burns no id when everything is already tagged.
  void stampUntagged() {
    var needed = false;
    for (final f in faces) {
      if (f.origin == null) {
        needed = true;
        break;
      }
    }
    if (!needed) {
      for (final e in edges) {
        if (e.origin == null) {
          needed = true;
          break;
        }
      }
    }
    if (!needed) return;

    final oid = BrepOriginIds.next();
    for (var i = 0; i < faces.length; i++) {
      faces[i].origin ??= BrepOrigin(original: oid, entity: i);
    }
    for (var i = 0; i < edges.length; i++) {
      edges[i].origin ??= BrepOrigin(original: oid, entity: i);
    }
  }

  Map<String, dynamic> toJson() => {
    'v': [
      for (final p in vertices) [p.x, p.y, p.z],
    ],
    'e': [for (final e in edges) e.toJson()],
    'f': [for (final f in faces) f.toJson()],
  };

  factory BrepSolid.fromJson(Map<String, dynamic> json) {
    return BrepSolid(
      vertices: [
        for (final v in json['v'] as List)
          Vector3(
            (v[0] as num).toDouble(),
            (v[1] as num).toDouble(),
            (v[2] as num).toDouble(),
          ),
      ],
      edges: [
        for (final e in json['e'] as List)
          BrepEdge.fromJson(e as Map<String, dynamic>),
      ],
      faces: [
        for (final f in json['f'] as List)
          BrepFace.fromJson(f as Map<String, dynamic>),
      ],
    );
  }
}

class BrepBuilder {
  final vertices = <Vector3>[];
  final edges = <BrepEdge>[];
  final faces = <BrepFace>[];

  int vertex(Vector3 p) {
    vertices.add(p.clone());
    return vertices.length - 1;
  }

  int edge(int v0, int v1, BrepCurve curve, {BrepOrigin? origin}) {
    edges.add(BrepEdge(v0: v0, v1: v1, curve: curve, origin: origin));
    return edges.length - 1;
  }

  void face(
    BrepSurface surface,
    List<BrepLoop> loops, {
    bool reversed = false,
    BrepOrigin? origin,
  }) {
    faces.add(
      BrepFace(
        surface: surface,
        loops: loops,
        reversed: reversed,
        origin: origin,
      ),
    );
  }

  BrepSolid build() {
    final solid = BrepSolid(vertices: vertices, edges: edges, faces: faces);
    solid.stampUntagged();
    return solid;
  }
}

class BrepUnsupported implements Exception {
  BrepUnsupported(this.message, {this.code});
  final String message;

  /// Stable id for callers that must not mesh-fallback, e.g. [filletOffset].
  final String? code;

  /// Inward offset would erase a convex round (inner radius ≤ 0).
  static const String filletOffset = 'fillet-offset';

  @override
  String toString() => message;
}
