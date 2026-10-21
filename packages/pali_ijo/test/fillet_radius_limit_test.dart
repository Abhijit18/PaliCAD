import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('large fillets are bounded by geometry rather than a fixed size', () {
    final solid = BrepPrimitives.box(60000, 60000, 60000);
    final edge = BrepEdgeSelect.linear(
      direction: Vector3(0, 0, 1),
    ).resolve(solid).first;
    expect(BrepFillet.maxConvexRadius(solid, [edge]), closeTo(60000, 1e-6));
    expect(BrepFillet.maxPlanarRadius(solid, [edge]), closeTo(60000, 1e-6));
    expect(BrepFillet.maxBlendDistance(solid, [edge]), closeTo(60000, 1e-6));
    final result = IjoOps.filletSolid(
      solid,
      [edge],
      radius: 15000,
      inverse: false,
    );
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(result)), isTrue);
    expect(
      result.faces
          .where((f) => f.surface is BrepCylinder)
          .any(
            (f) => ((f.surface as BrepCylinder).radius - 15000).abs() < 1e-6,
          ),
      isTrue,
    );
  });

  test('planar limits account for competing selected edges and rotation', () {
    var solid = BrepExtrude.profile([
      Vector3(0, 0, 0),
      Vector3(20, 0, 0),
      Vector3(20, 10, 0),
      Vector3(0, 10, 0),
    ], Vector3(0, 0, 2));
    final corners = [
      for (var i = 0; i < solid.edges.length; i++)
        if ((solid.vertices[solid.edges[i].v0].z -
                    solid.vertices[solid.edges[i].v1].z)
                .abs() >
            1.99)
          i,
    ];
    expect(corners, hasLength(4));
    for (var rotation = 0; rotation < 2; rotation++) {
      expect(
        BrepFillet.maxPlanarRadius(solid, [corners.first]),
        closeTo(10, 1e-8),
      );
      expect(BrepFillet.maxPlanarRadius(solid, corners), closeTo(5, 1e-8));
      solid = BrepXform.map(
        solid,
        (p) =>
            Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .7).rotated(p),
      );
    }
  });
}
