import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void expectClosed(MeshData mesh) {
  expect(MeshSeal.isClosedSolid(mesh), isTrue);
  expect(MeshEdit.signedVolume(mesh), greaterThan(0));
}

MeshData tess(BrepSolid s) => IjoOps.tessellateKeep(s, op: 'Boolean');

void main() {
  test('two spheres union stays B-Rep', () {
    final a = BrepPrimitives.sphere(center: Vector3(0, 0, 0), radius: 10);
    final b = BrepPrimitives.sphere(center: Vector3(8, 0, 0), radius: 10);
    final u = IjoOps.booleanSolid(a, b, IjoOps.unionOp);
    expect(u.faces.where((f) => f.surface is BrepSphere).length, 2);
    final mesh = tess(u);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(4 / 3 * math.pi * 10 * 10 * 10));
  });

  test('two spheres subtract stays B-Rep', () {
    final a = BrepPrimitives.sphere(center: Vector3(0, 0, 0), radius: 10);
    final b = BrepPrimitives.sphere(center: Vector3(8, 0, 0), radius: 6);
    final cut = IjoOps.booleanSolid(a, b, IjoOps.subtractOp);
    expect(cut.faces.any((f) => f.surface is BrepSphere), isTrue);
    final mesh = tess(cut);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), lessThan(4 / 3 * math.pi * 10 * 10 * 10));
  });

  test('sphere overlapping a cube edge stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(-2, -2, 10),
      radius: 8,
    );
    final u = IjoOps.booleanSolid(box, sphere, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepSphere), isTrue);
    final mesh = tess(u);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(8000));
  });

  test('sphere overlapping a cube corner stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(-2, -2, -2),
      radius: 8,
    );
    final u = IjoOps.booleanSolid(box, sphere, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepSphere), isTrue);
    final mesh = tess(u);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(8000));
  });

  test('coaxial cylinder union sphere stays B-Rep', () {
    final cyl = BrepPrimitives.cylinder(
      origin: Vector3(10, 10, 0),
      axis: Vector3(0, 0, 1),
      radius: 4,
      height: 20,
    );
    final sphere = BrepPrimitives.sphere(
      center: Vector3(10, 10, 10),
      radius: 8,
    );
    final u = IjoOps.booleanSolid(cyl, sphere, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepSphere), isTrue);
    expect(u.faces.any((f) => f.surface is BrepCylinder), isTrue);
    final mesh = tess(u);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(4 / 3 * math.pi * 8 * 8 * 8));
  });

  test('cube subtract coaxial cylinder stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final cyl = BrepPrimitives.cylinder(
      origin: Vector3(10, 10, -1),
      axis: Vector3(0, 0, 1),
      radius: 4,
      height: 22,
    );
    final cut = IjoOps.booleanSolid(box, cyl, IjoOps.subtractOp);
    expect(cut.faces.any((f) => f.surface is BrepCylinder), isTrue);
    final mesh = tess(cut);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), lessThan(8000));
  });

  test('torus union a containing cube stays B-Rep', () {
    final torus = BrepPrimitives.torus(
      center: Vector3(40, 40, 10),
      axis: Vector3(0, 0, 1),
      majorRadius: 12,
      minorRadius: 4,
    );
    final box = BrepPrimitives.box(80, 80, 20);
    final u = IjoOps.booleanSolid(box, torus, IjoOps.unionOp);
    expectClosed(tess(u));
    expect(IjoMeasure.volume(tess(u)), closeTo(80 * 80 * 20, 200));
  });

  test('cone sitting on a cube stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final cone = BrepPrimitives.cone(
      origin: Vector3(10, 10, 20),
      axis: Vector3(0, 0, 1),
      radius: 6,
      height: 12,
    );
    final u = IjoOps.booleanSolid(box, cone, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepCone), isTrue);
    final mesh = tess(u);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(8000));
  });

  test('torus cut by a face that contains its axis stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final torus = BrepPrimitives.torus(
      center: Vector3(10, 0, 10),
      axis: Vector3(0, 0, 1),
      majorRadius: 6,
      minorRadius: 2,
    );
    final u = IjoOps.booleanSolid(box, torus, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepTorus), isTrue);
    final mesh = tess(u);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(8000));
  });

  test('oblique plane cutting a cone stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final cone = BrepPrimitives.cone(
      origin: Vector3(10, 10, 5),
      axis: Vector3(0.4, 0, 1).normalized(),
      radius: 7,
      height: 25,
    );
    final u = IjoOps.booleanSolid(box, cone, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepCone), isTrue);
    expect(u.edges.any((e) => e.curve is BrepEllipse), isTrue);
    final mesh = tess(u);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(8000));
  });

  BrepSolid prism(Vector3 o, Vector3 u, Vector3 v, Vector3 dir, double w, double h) {
    return BrepExtrude.polygon([
      o,
      o + u * w,
      o + u * w + v * h,
      o + v * h,
    ], dir);
  }

  test('cube ∪ sphere ∪ cube on the other side stays B-Rep', () {
    final boxA = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(10, 10, 22),
      radius: 8,
    );
    final boxB = BrepXform.translate(
      BrepPrimitives.box(20, 20, 20),
      Vector3(0, 0, 24),
    );
    final mid = IjoOps.booleanSolid(boxA, sphere, IjoOps.unionOp);
    expect(mid.faces.any((f) => f.surface is BrepSphere), isTrue);
    expectClosed(tess(mid));
    final body = IjoOps.booleanSolid(mid, boxB, IjoOps.unionOp);
    expect(body.faces.any((f) => f.surface is BrepSphere), isTrue);
    final mesh = tess(body);
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(16000));
    expect(IjoMeasure.volume(mesh), lessThan(18000));
  });

  test('extrude-subtracts through cube-sphere-cube stay B-Rep', () {
    final boxA = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(10, 10, 22),
      radius: 8,
    );
    final boxB = BrepXform.translate(
      BrepPrimitives.box(20, 20, 20),
      Vector3(0, 0, 24),
    );
    var body = IjoOps.booleanSolid(boxA, sphere, IjoOps.unionOp);
    body = IjoOps.booleanSolid(body, boxB, IjoOps.unionOp);
    expectClosed(tess(body));

    void cut(BrepSolid tool, String step) {
      body = IjoOps.booleanSolid(body, tool, IjoOps.subtractOp);
      expect(
        body.faces.any((f) => f.surface is BrepSphere),
        isTrue,
        reason: '$step dropped the sphere',
      );
      expect(
        body.faces.every(
          (f) => f.surface is BrepPlane || f.surface is BrepSphere,
        ),
        isTrue,
        reason: '$step left a non-analytic face',
      );
    }

    cut(
      prism(
        Vector3(-5, 8, 21),
        Vector3(0, 1, 0),
        Vector3(0, 0, 1),
        Vector3(30, 0, 0),
        4,
        2,
      ),
      'X through the spherical waist',
    );
    cut(
      prism(
        Vector3(8, -5, 21),
        Vector3(1, 0, 0),
        Vector3(0, 0, 1),
        Vector3(0, 30, 0),
        4,
        2,
      ),
      'Y crossing the X slot',
    );
    cut(
      prism(
        Vector3(8, 8, -5),
        Vector3(1, 0, 0),
        Vector3(0, 1, 0),
        Vector3(0, 0, 55),
        4,
        4,
      ),
      'Z through both cubes',
    );
    final u = Vector3(1, 0, -0.4).normalized();
    final n = Vector3(0.4, 0, 1).normalized();
    var v = n.cross(u);
    if (v.length < 1e-8) v = Vector3(0, 1, 0);
    v.normalize();
    cut(
      prism(Vector3(10, 6, 12) - u * 2 - v * 2 - n * 10, u, v, n * 40, 4, 4),
      'tilted cut crossing previous slots',
    );
  });
}
