import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:pali_ijo/src/measure/bounds.dart'
    show conservativeSphereFaceBounds;
import 'package:vector_math/vector_math_64.dart';
import 'sphere_bridge_acceptance_test.dart' as fixture;

void main() {
  test('trim bounds contain spherical face samples after rotation', () {
    final original = fixture.bridge();
    final rotation = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .73);
    final body = BrepXform.map(
      original,
      (p) => rotation.rotated(p) + Vector3(31, -17, 9),
    );
    for (final face in body.faces.where((f) => f.surface is BrepSphere)) {
      final bounds = conservativeSphereFaceBounds(body, face);
      for (final loop in face.loops) {
        for (final co in loop.coedges) {
          final curve = body.edges[co.edge].curve;
          for (var i = 0; i <= 100; i++) {
            expect(
              bounds.containsPoint(
                curve.evaluate(curve.t0 + (curve.t1 - curve.t0) * i / 100),
                tolerance: 1e-6,
              ),
              isTrue,
            );
          }
        }
      }
    }
    final band = original.faces.firstWhere((f) => f.surface is BrepSphere);
    final bounds = conservativeSphereFaceBounds(original, band);
    expect(bounds.min.x, closeTo(20, 1e-8));
    expect(bounds.max.x, closeTo(24, 1e-8));
  });
  for (final rounded in [false, true]) {
    test('cut misses visible sphere patches, rounded=$rounded', () {
      var body = fixture.bridge();
      if (rounded) {
        body = IjoOps.filletSolid(
          body,
          BrepEdgeSelect.ofFace(
            BrepFaceSelect.extreme(Vector3(-1, 0, 0)),
          ).resolve(body),
          radius: 2,
          inverse: false,
        );
      }
      final cutter = BrepPrimitives.cylinder(
        origin: Vector3(15, -50, 10),
        axis: Vector3(0, 1, 0),
        radius: 2,
        height: 120,
      );
      final result = IjoOps.booleanSolid(body, cutter, IjoOps.subtractOp);
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      final display = IjoOps.tessellateClosed(result);
      expect(MeshKit.isClosedSolid(display), isTrue);
      expect(IjoOps.splitBrep(result), hasLength(1));
      expect(
        MeshKit.signedVolume(display),
        closeTo(
          MeshKit.signedVolume(IjoOps.tessellateClosed(body)) -
              80 * 3.141592653589793,
          2,
        ),
      );
      expect(BrepClassify.side(result, Vector3(15, 10, 10)), BrepSide.outside);
    });
  }
}
