import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

List<Vector3> rectangle(double x0, double y0, double x1, double y1) => [
  Vector3(x0, y0, 0),
  Vector3(x1, y0, 0),
  Vector3(x1, y1, 0),
  Vector3(x0, y1, 0),
];

void main() {
  for (final angle in [0.0, .63]) {
    for (final depth in [2.0, 4.0, 6.0]) {
      test(
        'bridge cut at $depth splits only past shoulder, rotation=$angle',
        () {
          final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), angle);
          Vector3 map(Vector3 p) => q.rotated(p) + Vector3(70, -45, 23);
          final solid = BrepExtrude.profile(
            [
              Vector3(0, 0, 0),
              Vector3(20, 0, 0),
              Vector3(20, 20, 0),
              Vector3(16, 20, 0),
              Vector3(16, 4, 0),
              Vector3(4, 4, 0),
              Vector3(4, 20, 0),
              Vector3(0, 20, 0),
            ].map(map).toList(),
            q.rotated(Vector3(0, 0, 10)),
          );
          final normal = q.rotated(Vector3(0, -1, 0));
          final face = solid.faces.indexWhere(
            (f) => f.outwardNormal(0, 0).dot(normal) > .999,
          );
          // The front has rails, but is not a cap of a complete straight prism.
          expect(BrepExtrude.tryMovePlanarFace(solid, face, -depth), isNull);
          final cutter = BrepExtrude.fromFace(
            solid,
            solid.faces[face],
            normal * -depth,
          );
          final cut = IjoOps.booleanSolid(solid, cutter, IjoOps.subtractOp);
          final parts = IjoOps.splitBrep(cut);
          expect(parts, hasLength(depth < 4 ? 1 : 2));
          expect(
            IjoMeasure.volumeSolid(cut),
            closeTo(depth < 4 ? 2080 - 200 * depth : 80 * (20 - depth), .001),
          );
          for (final part in parts) {
            expect(BrepHeal.topologyIsClosed(part), isTrue);
            final mesh = IjoOps.tessellateClosed(part);
            expect(MeshKit.isClosedSolid(mesh), isTrue);
            expect(MeshTopology.decompose(mesh), hasLength(1));
          }
        },
      );
    }
  }

  test('complete prism with a through hole still uses cap movement', () {
    final solid = BrepExtrude.profileWithHoles(rectangle(0, 0, 20, 20), [
      rectangle(6, 6, 14, 14),
    ], Vector3(0, 0, 10));
    final cap = solid.faces.indexWhere((f) => f.outwardNormal(0, 0).z > .99);
    final moved = BrepExtrude.tryMovePlanarFace(solid, cap, -3);
    expect(moved, isNotNull);
    expect(IjoOps.splitBrep(moved!), hasLength(1));
    expect(IjoMeasure.volumeSolid(moved), closeTo((400 - 64) * 7, .001));
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(moved)), isTrue);
  });

  test('healing a bridged hole preserves the hole and its closed walls', () {
    final solid = BrepExtrude.profileWithHoles(rectangle(0, 0, 20, 20), [
      rectangle(6, 6, 14, 14),
    ], Vector3(0, 0, 10));
    final cap = solid.faces.firstWhere((f) => f.outwardNormal(0, 0).z > .99);
    final outer = cap.loops.first.coedges, inner = cap.loops.last.coedges;
    int start(BrepCoedge co) =>
        co.reversed ? solid.edges[co.edge].v1 : solid.edges[co.edge].v0;
    final a = start(outer.first),
        b = start(inner.first),
        edge = solid.edges.length;
    solid.edges.add(
      BrepEdge(
        v0: a,
        v1: b,
        curve: BrepLine(solid.vertices[a], solid.vertices[b]),
      ),
    );
    cap.loops = [
      BrepLoop([
        BrepCoedge(edge),
        ...inner,
        BrepCoedge(edge, reversed: true),
        ...outer,
      ]),
    ];
    final healed = BrepHeal.run(solid);
    expect(IjoOps.splitBrep(healed), hasLength(1));
    expect(healed.faces.where((f) => f.loops.length == 2), hasLength(2));
    expect(BrepHeal.topologyIsClosed(healed), isTrue);
    expect(IjoMeasure.volumeSolid(healed), closeTo(3360, .001));
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(healed)), isTrue);
  });
}
