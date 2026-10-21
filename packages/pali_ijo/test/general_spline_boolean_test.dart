import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid bump() {
  final box = BrepPrimitives.box(2, 2, 1);
  final top = BrepSplineSurface(
    degreeU: 2,
    degreeV: 2,
    points: [
      for (var i = 0; i < 3; i++)
        [
          for (var j = 0; j < 3; j++)
            Vector3(i.toDouble(), j.toDouble(), i == 1 && j == 1 ? 5 : 1),
        ],
    ],
    knotsU: [0, 0, 0, 1, 1, 1],
    knotsV: [0, 0, 0, 1, 1, 1],
  );
  return BrepSolid(
    vertices: box.vertices,
    edges: box.edges,
    faces: [
      for (final f in box.faces)
        if ((f.surface as BrepPlane).normal.z > .9)
          BrepFace(surface: top, loops: f.loops, reversed: f.reversed)
        else
          f,
    ],
  );
}

void main() {
  test('general spline dome can be sliced through an interior closed loop', () {
    final s = bump();
    expect(BrepHeal.topologyIsClosed(s), isTrue);
    final halves = IjoOps.sliceBrep(s, Vector3(0, 0, 1.3), Vector3(0, 0, 1));
    expect(halves.length, 2);
    for (final h in halves) {
      expect(BrepHeal.topologyIsClosed(h), isTrue);
      expect(
        h.edges.any((e) => e.curve is BrepSurfaceIntersectionCurve),
        isTrue,
      );
      expect(MeshKit.isClosedSolid(BrepTessellate.run(h)), isTrue);
    }
  });
  for (final op in [IjoOps.subtractOp, IjoOps.unionOp, IjoOps.intersectOp]) {
    test('two general spline domes preserve closed BREP for boolean $op', () {
      final a = bump(), b = BrepXform.translate(bump(), Vector3(.7, .3, .2));
      final result = IjoOps.booleanSolid(a, b, op);
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      expect(
        result.edges.any((e) => e.curve is BrepSurfaceIntersectionCurve),
        isTrue,
      );
      final mesh = BrepTessellate.run(result);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(MeshKit.signedVolume(mesh), greaterThan(0));
      expect(
        BrepClassify.side(result, Vector3(1.1, 1.1, .7)),
        op == IjoOps.subtractOp ? BrepSide.outside : BrepSide.inside,
      );
      expect(
        BrepClassify.side(result, Vector3(.2, .2, .5)),
        op == IjoOps.intersectOp ? BrepSide.outside : BrepSide.inside,
      );
    });
  }
  test('spline union supports another oblique slice and reflected scaling', () {
    final joined = IjoOps.booleanSolid(
      bump(),
      BrepXform.translate(bump(), Vector3(.7, .3, .2)),
      IjoOps.unionOp,
    );
    final halves = IjoOps.sliceBrep(
      joined,
      Vector3(0, 0, 1.4),
      Vector3(.1, 0, 1),
    );
    expect(halves.length, 2);
    for (final h in halves) {
      final changed = BrepXform.scaleAbout(
        h,
        Vector3.zero(),
        Vector3(-1.2, .8, 1.1),
      );
      expect(BrepHeal.topologyIsClosed(changed), isTrue);
      expect(MeshKit.isClosedSolid(BrepTessellate.run(changed)), isTrue);
      final restored = BrepSolid.fromJson(changed.toJson());
      expect(BrepHeal.topologyIsClosed(restored), isTrue);
    }
  });

  test(
    'a cylindrical bore crosses a general spline top without losing BREP',
    () {
      final result = IjoOps.booleanSolid(
        bump(),
        BrepPrimitives.cylinder(
          origin: Vector3(1, 1, -1),
          axis: Vector3(0, 0, 1),
          radius: .3,
          height: 4,
        ),
        IjoOps.subtractOp,
      );
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      final mesh = BrepTessellate.run(result);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(BrepClassify.side(result, Vector3(1, 1, .5)), BrepSide.outside);
      expect(BrepClassify.side(result, Vector3(.5, .5, .5)), BrepSide.inside);
      final ids = mesh.edgeIds?.values.toSet() ?? <int>{};
      for (var i = 0; i < result.edges.length; i++)
        if (result.edges[i].curve is BrepSurfaceIntersectionCurve)
          expect(ids, contains(i));
    },
  );

  test('identical spline body set operations retain their exact boundary', () {
    final s = bump(), before = s.toJson().toString();
    for (final op in [IjoOps.unionOp, IjoOps.intersectOp]) {
      final result = IjoOps.booleanSolid(s, s, op);
      expect(result.faces.length, s.faces.length);
      expect(BrepHeal.topologyIsClosed(result), isTrue);
    }
    expect(
      () => IjoOps.booleanSolid(s, s, IjoOps.subtractOp),
      throwsStateError,
    );
    expect(s.toJson().toString(), before);
  });
}
