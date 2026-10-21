/// pali ijo — a Dart analytic B-Rep CAD kernel with mesh tessellation.
///
/// [BrepSolid] is the solid's truth. [MeshData] is a tessellation for
/// viewports, picking, and mesh export. Mesh CSG ([MeshBoolean]) remains
/// the fallback for imported tessellations and unsupported boolean pairs.
///
/// Placement is deliberately *not* modelled here. A solid's vertices are in
/// its own local frame; where a body sits in a scene belongs to the host
/// application.
library;

// `vector_math_64` is deliberately *not* re-exported. Nearly every consumer
// imports it directly anyway, and re-exporting makes both imports trip
// `unnecessary_import` in every such file. Add it alongside this package:
//
//   dependencies:
//     pali_ijo: ...
//     vector_math: ^2.1.4

export 'src/brep/brep.dart';
export 'src/ijo_log.dart';
export 'src/io/io.dart';
export 'src/csg/boolean.dart';
export 'src/csg/mesh_solid.dart';
export 'src/csg/minkowski.dart';
export 'src/csg/op.dart';
export 'src/measure/measure.dart';
export 'src/mesh/mesh_data.dart';
export 'src/mesh/mesh_kit.dart';
export 'src/mesh/mesh_regions.dart';
export 'src/ops/edge_geometry.dart';
export 'src/ops/loop_geometry.dart';
export 'src/ops/ops.dart';
export 'src/select/select.dart';
export 'src/sweep/sweep.dart';
export 'src/tessellation.dart';

export 'src/thread/form.dart';
