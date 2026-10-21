import 'package:vector_math/vector_math_64.dart';

import 'mesh_data.dart';
import 'mesh_edit.dart';
import 'mesh_build.dart';
import 'mesh_hull.dart';
import 'mesh_seal.dart';
import 'mesh_topology.dart';
import 'mesh_xform.dart';

export 'mesh_build.dart';
export 'mesh_edit.dart';
export 'mesh_polygon.dart';
export 'mesh_hull.dart';
export 'mesh_seal.dart';
export 'mesh_topology.dart';
export 'mesh_xform.dart';
export 'mesh_frame.dart';
export 'mesh_op.dart';
export 'mesh_offset.dart';
export 'mesh_hollow.dart';
export 'mesh_fillet.dart';
export 'mesh_chamfer.dart';
export 'packed_mesh.dart';

/// Public mesh utilities for pali ijo.
///
/// Implementations live in sibling files by topic (`mesh_edit.dart`,
/// `mesh_build.dart`, …). Add a primitive or repair step there, then expose
/// it here so call sites keep using [MeshKit].
class MeshKit {
  MeshKit._();

  static MeshData clone(MeshData mesh) => MeshEdit.clone(mesh);

  static MeshData triangulate(MeshData mesh) => MeshEdit.triangulate(mesh);

  static double signedVolume(MeshData mesh) => MeshEdit.signedVolume(mesh);

  static MeshData weld(MeshData mesh, {double epsilon = 1e-5}) =>
      MeshEdit.weld(mesh, epsilon: epsilon);

  static MeshData invert(MeshData mesh) => MeshEdit.invert(mesh);

  static MeshData combine(List<MeshData> meshes) => MeshEdit.combine(meshes);

  static MeshData splitTJunctions(MeshData mesh, {double epsilon = 1e-4}) =>
      MeshTopology.splitTJunctions(mesh, epsilon: epsilon);

  static List<MeshData> connectedComponents(MeshData mesh) =>
      MeshTopology.connectedComponents(mesh);

  /// Vertex-connected solids (see [MeshTopology.decompose]).
  static List<MeshData> decompose(MeshData mesh) => MeshTopology.decompose(mesh);

  static MeshData transformVertices(
    MeshData mesh,
    Vector3 Function(Vector3 v) map, {
    bool reverseWinding = false,
  }) => MeshXform.transformVertices(mesh, map, reverseWinding: reverseWinding);

  static MeshData scaleAbout(MeshData mesh, Vector3 center, Vector3 scale) =>
      MeshXform.scaleAbout(mesh, center, scale);

  static MeshData mirror(MeshData mesh, Vector3 origin, Vector3 normal) =>
      MeshXform.mirror(mesh, origin, normal);

  static MeshData fitImported(MeshData mesh, {double targetMaxDim = 120}) =>
      MeshXform.fitImported(mesh, targetMaxDim: targetMaxDim);

  static MeshData extrudeTriangles({
    required List<Vector3> vertices,
    required List<List<int>> triangles,
    required Vector3 direction,
  }) => MeshBuild.extrudeTriangles(
    vertices: vertices,
    triangles: triangles,
    direction: direction,
  );

  static MeshData trianglePrism(Vector3 a, Vector3 b, Vector3 p, Vector3 q) =>
      MeshBuild.trianglePrism(a, b, p, q);

  static MeshData sweepPolygon(List<Vector3> ring, Vector3 delta) =>
      MeshBuild.sweepPolygon(ring, delta);

  static MeshData sweepPolygonWithHoles(
    List<Vector3> outer,
    List<List<Vector3>> holes,
    Vector3 delta,
  ) =>
      MeshBuild.sweepPolygonWithHoles(outer, holes, delta);

  static List<MeshData> sliceParts(
    MeshData mesh,
    Vector3 origin,
    Vector3 normal,
  ) =>
      MeshBuild.sliceParts(mesh, origin, normal);

  static MeshData pocketRegion({
    required MeshData mesh,
    required List<int> region,
    required Vector3 direction,
  }) =>
      MeshBuild.pocketRegion(
        mesh: mesh,
        region: region,
        direction: direction,
      );

  static MeshData extrudeRegion({
    required MeshData mesh,
    required List<int> region,
    required Vector3 direction,
  }) =>
      MeshBuild.extrudeRegion(
        mesh: mesh,
        region: region,
        direction: direction,
      );

  static MeshData revolvePolygon(
    List<Vector3> profile,
    Vector3 origin,
    Vector3 axis, {
    int segments = 32,
    List<double>? azimuths,
    Vector3? radialZero,
  }) => MeshBuild.revolvePolygon(
    profile,
    origin,
    axis,
    segments: segments,
    azimuths: azimuths,
    radialZero: radialZero,
  );

  static MeshData icosphere({
    required Vector3 center,
    required double radius,
    int subdivisions = 3,
  }) =>
      MeshBuild.icosphere(
        center: center,
        radius: radius,
        subdivisions: subdivisions,
      );

  static MeshData cylinder({
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required double height,
    int segments = 24,
  }) => MeshBuild.cylinder(
    origin: origin,
    axis: axis,
    radius: radius,
    height: height,
    segments: segments,
  );

  static MeshData pipe(
    List<Vector3> path,
    double diameter, {
    int segments = 16,
  }) => MeshBuild.pipe(path, diameter, segments: segments);

  static MeshData unifyCoplanarFaces(MeshData mesh) =>
      MeshSeal.unifyCoplanarFaces(mesh);

  static int openBoundaryEdgeCount(MeshData mesh) =>
      MeshSeal.openBoundaryEdgeCount(mesh);

  static bool isClosedSolid(MeshData mesh) => MeshSeal.isClosedSolid(mesh);

  static MeshData dropDegenerateFaces(MeshData mesh) =>
      MeshSeal.dropDegenerateFaces(mesh);

  static MeshData preserveClosed(MeshData mesh) =>
      MeshSeal.preserveClosed(mesh);

  static MeshData stitchTJunctions(MeshData mesh, {double epsilon = 1e-4}) =>
      MeshSeal.stitchTJunctions(mesh, epsilon: epsilon);

  static MeshData sealSolid(MeshData mesh) => MeshSeal.sealSolid(mesh);

  static MeshData convexHull(List<Vector3> points) => MeshHull.ofPoints(points);

  static bool isConvex(MeshData mesh, {double epsilon = 1e-6}) =>
      MeshHull.isConvex(mesh, epsilon: epsilon);
}
