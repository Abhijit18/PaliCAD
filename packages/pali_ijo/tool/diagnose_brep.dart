// Reports fixed and unresolved cases from the 2026-09-11 investigation.
// Run from this package: dart run tool/diagnose_brep.dart
// The distant subtraction now stays closed; slice and acute tessellation
// remain diagnostic cases rather than passing modeling tests.
import 'dart:convert';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  final rounded = IjoOps.filletSolid(
    BrepPrimitives.box(20, 20, 20),
    [0],
    radius: 2,
    inverse: false,
  );
  final far = BrepXform.map(
    BrepPrimitives.box(5, 5, 5),
    (p) => p + Vector3(100, 100, 100),
  );
  final sub = IjoOps.booleanSolid(rounded, far, IjoOps.subtractOp);
  print(
    jsonEncode({
      'case': 'subtract a disjoint box from a filleted box',
      'expected': 'unchanged closed BREP',
      'inputClosed': BrepHeal.topologyIsClosed(rounded),
      'outputClosed': BrepHeal.topologyIsClosed(sub),
      'inputFaces': rounded.faces.length,
      'outputFaces': sub.faces.length,
    }),
  );
  final cylinder = BrepPrimitives.cylinder(
    origin: Vector3.zero(),
    axis: Vector3(0, 0, 1),
    radius: 10,
    height: 20,
  );
  final origin = Vector3(0, 0, 10);
  final normal = Vector3(0.02, 0, 1).normalized();
  final parts = IjoOps.sliceBrep(cylinder, origin, normal);
  final cuts = [
    for (final p in parts)
      for (final f in p.faces)
        if (f.surface is BrepPlane &&
            ((f.surface as BrepPlane).origin.z - 10).abs() < 1e-6)
          (f.surface as BrepPlane).normal,
  ];
  print(
    jsonEncode({
      'case': 'slightly tilted cylinder slice',
      'expected': 'cut faces parallel to requested plane',
      'requestedNormal': normal.storage.toList(),
      'cutNormals': [for (final n in cuts) n.storage.toList()],
    }),
  );
  final prism = BrepPrimitives.prism(diameter: 30, height: 20, sides: 3);
  final edge = BrepEdgeSelect.linear(
    direction: Vector3(0, 0, 1),
  ).resolve(prism).first;
  final blend = IjoOps.filletSolid(prism, [edge], radius: 2, inverse: false);
  final mesh = IjoOps.tessellate(blend);
  print(
    jsonEncode({
      'case': 'single upright triangular prism fillet',
      'expected': 'closed display mesh as well as closed topology',
      'topologyClosed': BrepHeal.topologyIsClosed(blend),
      'displayClosed': MeshKit.isClosedSolid(mesh),
      'openBoundaryEdges': MeshSeal.openBoundaryEdgeCount(mesh),
    }),
  );
}
