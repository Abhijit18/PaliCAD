import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../csg/mesh_solid.dart';
import '../tessellation.dart';
import 'mesh_data.dart';
import 'mesh_edit.dart';

/// Inward / outward offset. Axis-aligned boxes use a face inset; planar
/// solids move vertices onto offset face planes (miter). High-valence
/// vertices (spheres, dense fillets) fall back to area-weighted normals.
///
/// Minkowski-with-a-sphere remains on [MeshMinkowski] for explicit
/// round offsets. Using it here hangs on filleted / tessellated bodies
/// (one convex hull union per triangle).
class MeshOffset {
  MeshOffset._();

  static MeshData run(
    MeshData mesh,
    double radius, {
    MeshTessellation tessellation = MeshTessellation.draft,
    int? kernelSubdivisions,
  }) {
    if (radius.abs() < 1e-12) return mesh;
    final box = _aabbInset(mesh, radius);
    if (box != null) return box;
    return _planeMiterOffset(mesh, radius);
  }

  /// Axis-aligned box grown or shrunk by [radius] along each axis.
  ///
  /// Any tessellation of a filled AABB qualifies (sketch-extruded rectangles
  /// with extra collinear points), not only the 8-vertex / 12-triangle cube.
  static MeshData? _aabbInset(MeshData mesh, double radius) {
    if (mesh.vertices.length < 8 || mesh.faces.length < 12) return null;
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
    final aabbVol =
        (max.x - min.x) * (max.y - min.y) * (max.z - min.z);
    if (aabbVol < 1e-12) return null;
    final vol = MeshEdit.signedVolume(mesh).abs();
    if (vol < aabbVol * 0.99) return null;

    const eps = 1e-5;
    var corners = 0;
    for (final v in mesh.vertices) {
      final onX = (v.x - min.x).abs() < eps || (v.x - max.x).abs() < eps;
      final onY = (v.y - min.y).abs() < eps || (v.y - max.y).abs() < eps;
      final onZ = (v.z - min.z).abs() < eps || (v.z - max.z).abs() < eps;
      if (!onX && !onY && !onZ) return null;
      if (onX && onY && onZ) corners++;
    }
    if (corners < 8) return null;

    // Filleted / cylindrical tessellations still fill most of the AABB.
    // Only a box has exclusively axis-aligned facet normals.
    for (final face in mesh.faces) {
      if (face.length < 3) continue;
      final a = mesh.vertices[face[0]];
      final b = mesh.vertices[face[1]];
      final c = mesh.vertices[face[2]];
      final n = (b - a).cross(c - a);
      if (n.length2 < 1e-20) continue;
      n.normalize();
      final ax = n.x.abs();
      final ay = n.y.abs();
      final az = n.z.abs();
      if (ax < 0.99 && ay < 0.99 && az < 0.99) return null;
    }

    min -= Vector3(radius, radius, radius);
    max += Vector3(radius, radius, radius);
    final dx = max.x - min.x;
    final dy = max.y - min.y;
    final dz = max.z - min.z;
    if (dx < 1e-9 || dy < 1e-9 || dz < 1e-9) {
      return MeshData(vertices: const [], faces: const []);
    }
    return MeshPrimitives.translate(
      MeshPrimitives.cube(dx, dy, dz),
      min,
    );
  }

  /// Offset each vertex onto the intersection of its incident face planes.
  ///
  /// Cube corners move along (1,1,1), not the area-weighted diagonal of a
  /// vertex-normal offset, so the inner solid stays a smaller box (or prism)
  /// and `outer − inner` CSG stays well-separated.
  static MeshData _planeMiterOffset(MeshData mesh, double radius) {
    final n = mesh.vertices.length;
    final incidentN = List<List<Vector3>>.generate(n, (_) => <Vector3>[]);
    final incidentA = List<List<double>>.generate(n, (_) => <double>[]);
    final acc = List<Vector3>.generate(n, (_) => Vector3.zero());

    for (final face in mesh.faces) {
      if (face.length < 3) continue;
      final a = mesh.vertices[face[0]];
      final b = mesh.vertices[face[1]];
      final c = mesh.vertices[face[2]];
      final areaN = (b - a).cross(c - a);
      final len = areaN.length;
      if (len < 1e-16) continue;
      final nrm = areaN / len;
      final area = len * 0.5;
      for (final i in face) {
        if (i < 0 || i >= n) continue;
        incidentN[i].add(nrm);
        incidentA[i].add(area);
        acc[i] += areaN;
      }
    }

    return MeshData(
      vertices: [
        for (var i = 0; i < n; i++)
          mesh.vertices[i] +
              _vertexDelta(incidentN[i], incidentA[i], acc[i], radius),
      ],
      faces: [for (final f in mesh.faces) List<int>.from(f)],
      faceIds: mesh.faceIds == null ? null : List<int>.from(mesh.faceIds!),
      edgeIds: mesh.edgeIds == null ? null : Map<String, int>.from(mesh.edgeIds!),
    );
  }

  static const _samePlaneDot = 0.9995;

  static Vector3 _vertexDelta(
    List<Vector3> normals,
    List<double> areas,
    Vector3 areaWeighted,
    double radius,
  ) {
    if (normals.isEmpty) {
      if (areaWeighted.length2 < 1e-20) return Vector3.zero();
      return areaWeighted.normalized() * radius;
    }

    final unique = <Vector3>[];
    final uniqueArea = <double>[];
    for (var i = 0; i < normals.length; i++) {
      final n = normals[i];
      var merged = false;
      for (var j = 0; j < unique.length; j++) {
        if (unique[j].dot(n) > _samePlaneDot) {
          uniqueArea[j] += areas[i];
          merged = true;
          break;
        }
      }
      if (!merged) {
        unique.add(n);
        uniqueArea.add(areas[i]);
      }
    }

    if (unique.length == 1) return unique.first * radius;
    if (unique.length == 2) return _miter2(unique[0], unique[1], radius);
    if (unique.length == 3) {
      final d = _solve3(unique[0], unique[1], unique[2], radius);
      if (d != null) return d;
    }
    if (unique.length > 3) {
      final idx = [for (var i = 0; i < unique.length; i++) i]
        ..sort((a, b) => uniqueArea[b].compareTo(uniqueArea[a]));
      final d = _solve3(
        unique[idx[0]],
        unique[idx[1]],
        unique[idx[2]],
        radius,
      );
      if (d != null && unique.length <= 4) return d;
    }
    if (areaWeighted.length2 < 1e-20) return Vector3.zero();
    return areaWeighted.normalized() * radius;
  }

  static Vector3 _miter2(Vector3 n0, Vector3 n1, double radius) {
    final c = n0.dot(n1);
    if (c > _samePlaneDot) return n0 * radius;
    if (c < -_samePlaneDot) return Vector3.zero();
    final denom = 1.0 + c;
    if (denom.abs() < 1e-8) {
      final s = n0 + n1;
      if (s.length2 < 1e-16) return n0 * radius;
      return s.normalized() * radius;
    }
    return (n0 + n1) * (radius / denom);
  }

  /// Intersection of three offset planes: N·d = radius.
  static Vector3? _solve3(Vector3 n0, Vector3 n1, Vector3 n2, double radius) {
    final det = n0.dot(n1.cross(n2));
    if (det.abs() < 1e-10) return null;
    return (n1.cross(n2) + n2.cross(n0) + n0.cross(n1)) * (radius / det);
  }
}
