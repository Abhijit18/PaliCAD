import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid savedShape() => BrepSolid.fromJson(
  jsonDecode(
        File(
          'test/fixtures/inverse_fillet_extruded_split.json',
        ).readAsStringSync(),
      )
      as Map<String, dynamic>,
);

void main() {
  test(
    'filled planar seam cleanup preserves shape, selection IDs and source',
    () {
      final source = savedShape();
      final before = source.toJson();
      final cleaned = BrepHeal.mergeFilledPlanarLoops(source);
      expect(source.toJson(), before);
      expect(cleaned.edges[1].toJson(), source.edges[1].toJson());
      expect(cleaned.faces.length, source.faces.length - 1);
      expect(
        IjoMeasure.volumeSolid(cleaned),
        closeTo(IjoMeasure.volumeSolid(source), .01),
      );
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(cleaned)), isTrue);
      expect(
        BrepHeal.mergeFilledPlanarLoops(cleaned).toJson(),
        cleaned.toJson(),
      );
    },
  );
  test('coplanar cleanup preserves an actual through hole', () {
    final source = BrepBoolean.run(
      BrepPrimitives.box(30, 20, 10),
      BrepPrimitives.cylinder(
        origin: Vector3(15, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 3,
        height: 12,
      ),
      BrepBoolean.subtractOp,
    );
    final result = BrepHeal.mergeFilledPlanarLoops(source);
    expect(result.toJson(), source.toJson());
    expect(BrepClassify.side(result, Vector3(15, 10, 5)), BrepSide.outside);
  });
  for (final edge in [0, 1, 2, 3]) {
    for (final radius in [3.0, 19.109999999999562]) {
      test('bottom edge=$edge radius=$radius then subtract', () {
        final source = savedShape();
        final before = source.toJson();
        final rounded = IjoOps.filletSolid(
          source,
          [edge],
          radius: radius,
          inverse: false,
        );
        final edgeLength =
            (source.vertices[source.edges[edge].v0] -
                    source.vertices[source.edges[edge].v1])
                .length;
        expect(source.toJson(), before);
        expect(
          IjoMeasure.volumeSolid(rounded),
          closeTo(
            IjoMeasure.volumeSolid(source) -
                edgeLength * radius * radius * (1 - math.pi / 4),
            .1,
          ),
        );
        for (final quality in [MeshTessellation.draft, MeshTessellation.fine]) {
          expect(
            MeshKit.isClosedSolid(
              IjoOps.tessellateClosed(rounded, tessellation: quality),
            ),
            isTrue,
          );
        }
        final bottoms = rounded.faces
            .where(
              (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z < -.99,
            )
            .toList();
        expect(bottoms, hasLength(1));
        expect(bottoms.single.loops, hasLength(1));
        // The fillet must span the complete edge, with no internal seam.
        final blend = rounded.faces.singleWhere(
          (f) => f.surface is BrepCylinder && !f.reversed,
        );
        final generators = [
          for (final c in blend.loops.single.coedges)
            rounded.edges[c.edge].curve,
        ].whereType<BrepLine>();
        expect(generators, hasLength(2));
        for (final line in generators)
          expect((line.b - line.a).length, closeTo(edgeLength, 1e-7));
        for (final depth in [2.0, 11.79]) {
          final cutter = BrepExtrude.fromFace(
            rounded,
            bottoms.single,
            Vector3(0, 0, depth),
          );
          final result = IjoOps.booleanSolid(
            rounded,
            cutter,
            IjoOps.subtractOp,
          );
          expect(BrepHeal.topologyIsClosed(result), isTrue);
          expect(
            MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)),
            isTrue,
          );
          final b = BrepSolid.fromJson(result.toJson());
          expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(b)), isTrue);
          expect(
            IjoMeasure.volumeSolid(result),
            lessThan(IjoMeasure.volumeSolid(rounded)),
          );
        }
      });
    }
  }
}
