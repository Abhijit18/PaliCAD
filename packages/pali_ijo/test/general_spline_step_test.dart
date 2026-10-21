import 'dart:convert';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/spline_convert.dart';
import 'package:pali_ijo/src/brep/spline_intersect.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'general_spline_intersection_test.dart' as curves;
import 'general_spline_boolean_test.dart' as solids;

void main() {
  test(
    'native general intersection curve fits for STEP without changing its locus',
    () {
      final c = BrepSplineIntersect.surfaces(
        curves.bowl(),
        curves.plane(.3),
      ).single;
      final fitted = BrepSplineConvert.curve(c);
      for (var i = 0; i <= 300; i++) {
        expect(
          (c.evaluate(i / 300) - fitted.evaluate(i / 300)).length,
          lessThan(1e-7),
        );
      }
    },
  );
  test(
    'sliced spline dome retains its surface intersection definition through STEP',
    () {
      final pieces = IjoOps.sliceBrep(
        solids.bump(),
        Vector3(0, 0, 1.3),
        Vector3(0, 0, 1),
      );
      final source = pieces.first;
      final bytes = IjoStep.writeSolid(source);
      expect(utf8.decode(bytes), contains('INTERSECTION_CURVE('));
      final restored = IjoStep.readSolid(bytes);
      expect(
        restored.edges
            .where((e) => e.curve is BrepSurfaceIntersectionCurve)
            .length,
        source.edges
            .where((e) => e.curve is BrepSurfaceIntersectionCurve)
            .length,
      );
      expect(BrepHeal.topologyIsClosed(restored), isTrue);
      expect(MeshKit.isClosedSolid(BrepTessellate.run(restored)), isTrue);
    },
  );
  test(
    'an imported non-isoparametric spline trim supports a subsequent cut',
    () {
      final source = IjoOps.sliceBrep(
        solids.bump(),
        Vector3(0, 0, 1.3),
        Vector3(0, 0, 1),
      ).first;
      final imported = BrepSolid(
        vertices: source.vertices,
        faces: source.faces,
        edges: [
          for (final e in source.edges)
            BrepEdge(
              v0: e.v0,
              v1: e.v1,
              curve: e.curve is BrepSurfaceIntersectionCurve
                  ? BrepSplineConvert.curve(e.curve)
                  : e.curve,
              origin: e.origin,
            ),
        ],
      );
      final before = imported.toJson().toString();
      final parts = IjoOps.sliceBrep(
        imported,
        Vector3(1.1, 0, 0),
        Vector3(1, 0, 0),
      );
      expect(parts.length, 2);
      for (final p in parts) {
        expect(BrepHeal.topologyIsClosed(p), isTrue);
        expect(MeshKit.isClosedSolid(BrepTessellate.run(p)), isTrue);
      }
      expect(imported.toJson().toString(), before);
    },
  );
}
