import 'package:vector_math/vector_math_64.dart';

/// Triangular mesh data used for rendering and STL export.
class MeshData {
  MeshData({
    required this.vertices,
    required this.faces,
    this.faceIds,
    this.edgeIds,
    this.meshSolidReason,
  });

  final List<Vector3> vertices;
  final List<List<int>> faces;

  /// Set when this mesh is an explicit MeshSolid fallback (import or a
  /// B-Rep boolean that refused the pair). Analytics are gone.
  final String? meshSolidReason;

  bool get isMeshSolid => meshSolidReason != null;

  /// CAD face id per triangle, when tagged.
  final List<int>? faceIds;

  /// Quantized undirected display segment → CAD edge id.
  final Map<String, int>? edgeIds;

  /// Undirected 1 µm key used by [edgeIds] and coplanar region walks.
  static String segmentKey(Vector3 a, Vector3 b) {
    String q(Vector3 v) =>
        '${(v.x * 1e6).roundToDouble()},${(v.y * 1e6).roundToDouble()},${(v.z * 1e6).roundToDouble()}';
    final ka = q(a);
    final kb = q(b);
    return ka.compareTo(kb) <= 0 ? '$ka|$kb' : '$kb|$ka';
  }

  int? brepFaceOfTriangle(int triangleIndex) {
    final ids = faceIds;
    if (ids == null || triangleIndex < 0 || triangleIndex >= ids.length) {
      return null;
    }
    final id = ids[triangleIndex];
    return id < 0 ? null : id;
  }

  int? brepEdgeOfVertices(int a, int b) {
    final ids = edgeIds;
    if (ids == null ||
        a < 0 ||
        b < 0 ||
        a >= vertices.length ||
        b >= vertices.length) {
      return null;
    }
    final id = ids[segmentKey(vertices[a], vertices[b])];
    if (id == null || id < 0) return null;
    return id;
  }

  List<int> trianglesForBrepFace(int brepFace) {
    final ids = faceIds;
    if (ids == null) return const [];
    return [
      for (var i = 0; i < ids.length; i++)
        if (ids[i] == brepFace) i,
    ];
  }

  List<(int, int)> segmentsForBrepEdge(int brepEdge) {
    final ids = edgeIds;
    if (ids == null) return const [];
    final out = <(int, int)>[];
    final seen = <String>{};
    for (final face in faces) {
      for (var i = 0; i < face.length; i++) {
        final a = face[i];
        final b = face[(i + 1) % face.length];
        if (a == b) continue;
        final key = a < b ? '$a|$b' : '$b|$a';
        if (!seen.add(key)) continue;
        if (ids[segmentKey(vertices[a], vertices[b])] == brepEdge) {
          out.add(a < b ? (a, b) : (b, a));
        }
      }
    }
    return out;
  }

  MeshData clone() => MeshData(
        vertices: [for (final v in vertices) v.clone()],
        faces: [for (final f in faces) List<int>.from(f)],
        faceIds: faceIds == null ? null : List<int>.from(faceIds!),
        edgeIds: edgeIds == null ? null : Map<String, int>.from(edgeIds!),
        meshSolidReason: meshSolidReason,
      );

  MeshData asMeshSolid(String reason) => MeshData(
        vertices: vertices,
        faces: faces,
        faceIds: faceIds,
        edgeIds: edgeIds,
        meshSolidReason: reason,
      );

  MeshData mappedVertices(Vector3 Function(Vector3) map) {
    final verts = [for (final v in vertices) map(v)];
    Map<String, int>? mappedEdges;
    final src = edgeIds;
    if (src != null) {
      mappedEdges = {};
      for (final face in faces) {
        for (var i = 0; i < face.length; i++) {
          final a = face[i];
          final b = face[(i + 1) % face.length];
          final id = src[segmentKey(vertices[a], vertices[b])];
          if (id != null) {
            mappedEdges[segmentKey(verts[a], verts[b])] = id;
          }
        }
      }
    }
    return MeshData(
      vertices: verts,
      faces: faces,
      faceIds: faceIds == null ? null : List<int>.from(faceIds!),
      edgeIds: mappedEdges,
      meshSolidReason: meshSolidReason,
    );
  }

  Map<String, dynamic> toJson() => {
        'vertices': vertices.map((v) => [v.x, v.y, v.z]).toList(),
        'faces': [for (final face in faces) List<int>.from(face)],
        if (faceIds != null) 'faceIds': List<int>.from(faceIds!),
        if (edgeIds != null) 'edgeIds': Map<String, int>.from(edgeIds!),
        if (meshSolidReason != null) 'meshSolidReason': meshSolidReason,
      };

  factory MeshData.fromJson(Map<String, dynamic> json) {
    List<int>? faceIds;
    final rawFaces = json['faceIds'];
    if (rawFaces is List) {
      faceIds = [for (final e in rawFaces) (e as num).toInt()];
    }
    Map<String, int>? edgeIds;
    final rawEdges = json['edgeIds'];
    if (rawEdges is Map) {
      edgeIds = {
        for (final e in rawEdges.entries)
          e.key.toString(): (e.value as num).toInt(),
      };
    }
    return MeshData(
      vertices: (json['vertices'] as List)
          .map(
            (v) => Vector3(
              ((v as List)[0] as num).toDouble(),
              (v[1] as num).toDouble(),
              (v[2] as num).toDouble(),
            ),
          )
          .toList(),
      faces: (json['faces'] as List)
          .map((f) => List<int>.from(f as List))
          .toList(),
      faceIds: faceIds,
      edgeIds: edgeIds,
      meshSolidReason: json['meshSolidReason'] as String?,
    );
  }
}
