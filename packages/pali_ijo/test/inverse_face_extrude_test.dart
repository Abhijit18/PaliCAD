import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (var fi = 0; fi < 6; fi++) {
    for (final fraction in [.1, .5, 1.0, 1.5]) {
      test('subtract inverse cap face=$fi depth fraction=$fraction', () {
        final box = BrepPrimitives.box(30, 20, 10);
        final cap = box.faces[fi];
        final normal = cap.outwardNormal(0, 0);
        final thickness =
            normal.x.abs() * 30 + normal.y.abs() * 20 + normal.z.abs() * 10;
        final depth = thickness * fraction;
        final rim = [for (final co in cap.loops.single.coedges) co.edge];
        final source = IjoOps.filletSolid(box, rim, radius: 2, inverse: true);
        // Exercise geometry restored from a saved project, too.
        final solid = BrepSolid.fromJson(source.toJson());
        final before = solid.toJson();
        final face = solid.faces.firstWhere(
          (f) =>
              f.surface is BrepPlane && f.outwardNormal(0, 0).dot(normal) > .99,
        );
        final cutter = BrepExtrude.fromFace(solid, face, normal * -depth);
        final result = IjoOps.booleanSolid(solid, cutter, IjoOps.subtractOp);
        expect(solid.toJson(), before);
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        for (final quality in [MeshTessellation.draft, MeshTessellation.fine]) {
          expect(
            MeshKit.isClosedSolid(
              IjoOps.tessellateClosed(result, tessellation: quality),
            ),
            isTrue,
          );
        }
        final area =
            normal.x.abs() * 16 * 6 +
            normal.y.abs() * 26 * 6 +
            normal.z.abs() * 26 * 16;
        expect(
          IjoMeasure.volumeSolid(result),
          closeTo(
            IjoMeasure.volumeSolid(solid) - area * depth.clamp(0, thickness),
            .05,
          ),
        );
        final center = Vector3(15, 10, 5) + normal * (thickness / 2);
        expect(
          BrepClassify.side(
            result,
            center - normal * (depth.clamp(0, thickness) / 2),
          ),
          BrepSide.outside,
        );
        if (fraction < 1) {
          expect(
            BrepClassify.side(
              result,
              center - normal * ((depth + thickness) / 2),
            ),
            BrepSide.inside,
          );
        }
        expect(
          result.faces.where((f) => f.surface is BrepCylinder),
          hasLength(4),
        );
      });
    }
  }
  for (final rotate in [false, true]) {
    test('large inverse cap subtract 83.93 mm rotated=$rotate', () {
      var box = BrepPrimitives.box(160, 120, 100);
      final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box).first;
      final ids = [for (final e in box.faces[top].loops.single.coedges) e.edge];
      var source = IjoOps.filletSolid(box, ids, radius: 20, inverse: true);
      var normal = Vector3(0, 0, 1);
      if (rotate) {
        final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .7);
        source = BrepXform.map(
          source,
          (p) => q.rotated(p) + Vector3(100, -50, 25),
        );
        normal = q.rotated(normal);
      }
      final face = source.faces.firstWhere(
        (f) =>
            f.surface is BrepPlane && f.outwardNormal(0, 0).dot(normal) > .99,
      );
      final cutter = BrepExtrude.fromFace(source, face, normal * -83.93);
      final result = IjoOps.booleanSolid(source, cutter, IjoOps.subtractOp);
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)), isTrue);
      expect(
        IjoMeasure.volumeSolid(result),
        closeTo(IjoMeasure.volumeSolid(source) - 120 * 80 * 83.93, 20),
      );
    });
  }
}
