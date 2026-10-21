import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_build.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_edit.dart';
import '../tessellation.dart';
import 'boolean.dart';
import 'minkowski.dart';

/// Box, cylinder, sphere, extrude, and revolve as closed triangle meshes.
class MeshPrimitives {
  MeshPrimitives._();

  /// Axis-aligned box from the origin to [dx, dy, dz].
  static MeshData cube(double dx, double dy, double dz) {
    final v = [
      Vector3(0, 0, 0),
      Vector3(dx, 0, 0),
      Vector3(dx, dy, 0),
      Vector3(0, dy, 0),
      Vector3(0, 0, dz),
      Vector3(dx, 0, dz),
      Vector3(dx, dy, dz),
      Vector3(0, dy, dz),
    ];
    const faces = [
      [0, 2, 1],
      [0, 3, 2],
      [4, 5, 6],
      [4, 6, 7],
      [0, 1, 5],
      [0, 5, 4],
      [1, 2, 6],
      [1, 6, 5],
      [2, 3, 7],
      [2, 7, 6],
      [3, 0, 4],
      [3, 4, 7],
    ];
    const faceIds = [0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5];
    const edgePairs = [
      [0, 1],
      [1, 2],
      [2, 3],
      [3, 0],
      [4, 5],
      [5, 6],
      [6, 7],
      [7, 4],
      [0, 4],
      [1, 5],
      [2, 6],
      [3, 7],
    ];
    final edgeIds = <String, int>{
      for (var i = 0; i < edgePairs.length; i++)
        MeshData.segmentKey(v[edgePairs[i][0]], v[edgePairs[i][1]]): i,
    };
    var mesh = MeshData(
      vertices: v,
      faces: faces,
      faceIds: faceIds,
      edgeIds: edgeIds,
    );
    if (MeshEdit.signedVolume(mesh) < 0) mesh = MeshEdit.invert(mesh);
    return mesh;
  }

  static MeshData cylinder({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double height,
    int segments = 32,
  }) =>
      MeshBuild.cylinder(
        origin: origin,
        axis: axis,
        radius: radius,
        height: height,
        segments: segments,
      );

  static MeshData sphere({
    required Vector3 center,
    required double radius,
    MeshTessellation tessellation = MeshTessellation.standard,
  }) =>
      MeshBuild.icosphere(
        center: center,
        radius: radius,
        subdivisions: tessellation.sphereIcoSubdiv,
      );

  static MeshData extrude(List<Vector3> ring, Vector3 direction) =>
      MeshBuild.sweepPolygon(ring, direction);

  static MeshData revolve(
    List<Vector3> profile,
    Vector3 origin,
    Vector3 axis, {
    int segments = 32,
  }) =>
      MeshBuild.revolvePolygon(profile, origin, axis, segments: segments);

  static MeshData translate(MeshData mesh, Vector3 delta) =>
      mesh.mappedVertices((v) => v + delta);
}

/// Closed triangle mesh with boolean ops.
class IjoMesh {
  IjoMesh._(this.mesh);

  final MeshData mesh;

  factory IjoMesh.fromMesh(MeshData mesh) => IjoMesh._(mesh);

  factory IjoMesh.cube(double dx, [double? dy, double? dz]) =>
      IjoMesh._(MeshPrimitives.cube(dx, dy ?? dx, dz ?? dx));

  IjoMesh boolean(IjoMesh other, int op) =>
      IjoMesh._(MeshBoolean.run(mesh, other.mesh, op));

  IjoMesh operator +(IjoMesh other) =>
      boolean(other, MeshBoolean.unionOp);

  IjoMesh operator -(IjoMesh other) =>
      boolean(other, MeshBoolean.subtractOp);

  IjoMesh operator ^(IjoMesh other) =>
      boolean(other, MeshBoolean.intersectOp);

  IjoMesh minkowskiSum(IjoMesh other) =>
      IjoMesh._(MeshMinkowski.sum(mesh, other.mesh));

  IjoMesh minkowskiDifference(IjoMesh other) =>
      IjoMesh._(MeshMinkowski.difference(mesh, other.mesh));

  /// Grow ([radius] > 0) or shrink ([radius] < 0) by a sphere at the origin.
  IjoMesh offset(
    double radius, {
    MeshTessellation tessellation = MeshTessellation.draft,
  }) {
    if (radius.abs() < 1e-12) return this;
    final ball = IjoMesh._(
      MeshPrimitives.sphere(
        center: Vector3.zero(),
        radius: radius.abs(),
        tessellation: tessellation,
      ),
    );
    return radius > 0 ? minkowskiSum(ball) : minkowskiDifference(ball);
  }

  double get volume => MeshEdit.signedVolume(mesh).abs();

  bool get isEmpty => mesh.faces.isEmpty;
}
