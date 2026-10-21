import 'dart:convert';
import 'dart:math' as math;
import 'package:test/test.dart';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

void closed(BrepSolid solid) {
  expect(BrepHeal.topologyIsClosed(solid), isTrue);
  final mesh = IjoOps.tessellateKeep(solid);
  expect(MeshKit.isClosedSolid(mesh), isTrue);
  final uses = <(int, int), List<int>>{};
  for (final triangle in mesh.faces) {
    for (var i = 0; i < triangle.length; i++) {
      final a = triangle[i], b = triangle[(i + 1) % triangle.length];
      (uses[a < b ? (a, b) : (b, a)] ??= []).add(a < b ? 1 : -1);
    }
  }
  expect(
    uses.values.every((v) => v.length == 2 && v[0] + v[1] == 0),
    isTrue,
    reason: 'Every display edge must have exactly two opposite uses',
  );
  expect(MeshKit.signedVolume(mesh), greaterThan(0));
}

BrepSolid curvedPrism(bool spline) {
  final curves = <BrepCurve>[];
  if (spline) {
    Vector3 turn(Vector3 p, int i) {
      for (var k = 0; k < i; k++) p = Vector3(-p.y, p.x, p.z);
      return p;
    }

    for (var i = 0; i < 4; i++)
      curves.add(
        BrepSplineCurve(
          degree: 3,
          points: [
            for (final p in [
              Vector3(10, 0, 0),
              Vector3(10, 7, 0),
              Vector3(7, 10, 0),
              Vector3(0, 10, 0),
            ])
              turn(p, i),
          ],
          knots: [0, 0, 0, 0, 1, 1, 1, 1],
        ),
      );
  } else {
    curves.addAll([
      BrepLine(Vector3(-10, -5, 0), Vector3(10, -5, 0)),
      BrepCircle(
        center: Vector3(10, 0, 0),
        normal: Vector3(0, 0, 1),
        radius: 5,
        xDir: Vector3(1, 0, 0),
        t0: -math.pi / 2,
        t1: math.pi / 2,
      ),
      BrepLine(Vector3(10, 5, 0), Vector3(-10, 5, 0)),
      BrepCircle(
        center: Vector3(-10, 0, 0),
        normal: Vector3(0, 0, 1),
        radius: 5,
        xDir: Vector3(1, 0, 0),
        t0: math.pi / 2,
        t1: 3 * math.pi / 2,
      ),
    ]);
  }
  return BrepExtrude.fromCurves(
    [for (final c in curves) c.evaluate(c.t0)],
    curves,
    Vector3(0, 0, 10),
  );
}

void main() {
  test('inner conical rim keeps cavity face orientation', () {
    final block = BrepPrimitives.box(30, 30, 15);
    final tool = BrepPrimitives.cone(
      origin: Vector3(15, 15, -2),
      axis: Vector3(0, 0, 1),
      radius: 8,
      height: 20,
    );
    final bore = IjoOps.booleanSolid(block, tool, IjoOps.subtractOp);
    final id = bore.edges.indexWhere(
      (e) =>
          e.curve is BrepCircle &&
          (e.curve as BrepCircle).center.z.abs() < 1e-7,
    );
    expect(id, greaterThanOrEqualTo(0));
    closed(IjoOps.chamferSolid(bore, [id], 1));
    closed(IjoOps.filletSolid(bore, [id], radius: 1, inverse: false));
  });
  test(
    'asymmetric bore chamfer enlarges the opening and keeps outer walls',
    () {
      final block = BrepPrimitives.box(30, 30, 15);
      final tool = BrepPrimitives.cylinder(
        origin: Vector3(15, 15, -1),
        axis: Vector3(0, 0, 1),
        radius: 5,
        height: 17,
      );
      final bore = IjoOps.booleanSolid(block, tool, IjoOps.subtractOp);
      final id = bore.edges.indexWhere(
        (e) =>
            e.curve is BrepCircle &&
            ((e.curve as BrepCircle).center.z - 15).abs() < 1e-6,
      );
      expect(id, greaterThanOrEqualTo(0));
      final out = IjoOps.chamferSolid(
        bore,
        [id],
        2,
        options: const ChamferOptions(
          mode: ChamferMode.twoDistance,
          distance2: 3,
        ),
      );
      closed(out);
      expect(
        out.edges.any(
          (e) =>
              e.curve is BrepCircle &&
              ((e.curve as BrepCircle).radius - 7).abs() < 1e-6,
        ),
        isTrue,
      );
      expect(out.faces.where((f) => f.surface is BrepCone), hasLength(1));
    },
  );
  test('a later circular chamfer preserves an existing torus fillet', () {
    final cylinder = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 10,
      height: 20,
    );
    int at(BrepSolid s, double z) => s.edges.indexWhere(
      (e) =>
          e.curve is BrepCircle &&
          ((e.curve as BrepCircle).center.z - z).abs() < 1e-6 &&
          BrepFillet.facesForEdge(s, s.edges.indexOf(e)).length == 2,
    );
    final fillet = IjoOps.filletSolid(
      cylinder,
      [at(cylinder, 20)],
      radius: 1,
      inverse: false,
    );
    final result = IjoOps.chamferSolid(
      fillet,
      [at(fillet, 0)],
      2,
      options: const ChamferOptions(
        mode: ChamferMode.twoDistance,
        distance2: 3,
      ),
    );
    expect(result.faces.where((f) => f.surface is BrepTorus), hasLength(1));
    closed(result);
  });
  test(
    'rotated, reflected, uniformly scaled cylinder blend keeps millimetres',
    () {
      final cylinder = BrepPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 10,
        height: 20,
      );
      final id = cylinder.edges.indexWhere(
        (e) => e.curve is BrepCircle && (e.curve as BrepCircle).center.z == 20,
      );
      final transform = BrepAffineTransform(
        Vector3(0, -2, 0),
        Vector3(-2, 0, 0),
        Vector3(0, 0, 2),
        Vector3(11, 25, 3),
      );
      final placed = BrepXform.affine(cylinder, transform);
      final frame = BrepFillet.chamferFrame(placed, id)!,
          local = BrepFillet.chamferFrame(cylinder, id)!;
      expect((frame.$1 - transform.point(local.$1)).length, lessThan(1e-8));
      expect(
        frame.$2.dot(
          (transform.point(local.$1 + local.$2) - transform.point(local.$1))
              .normalized(),
        ),
        closeTo(1, 1e-8),
      );
      expect(
        frame.$3.dot(
          (transform.point(local.$1 + local.$3) - transform.point(local.$1))
              .normalized(),
        ),
        closeTo(1, 1e-8),
      );
      final out = IjoOps.chamferSolid(
        placed,
        [id],
        2,
        options: const ChamferOptions(
          mode: ChamferMode.twoDistance,
          distance2: 4,
        ),
      );
      final expected = BrepXform.affine(
        IjoOps.chamferSolid(
          cylinder,
          [id],
          1,
          options: const ChamferOptions(
            mode: ChamferMode.twoDistance,
            distance2: 2,
          ),
        ),
        transform,
      );
      closed(out);
      expect(
        MeshKit.signedVolume(IjoOps.tessellateKeep(out)),
        closeTo(MeshKit.signedVolume(IjoOps.tessellateKeep(expected)), 1e-5),
      );
    },
  );
  for (final spline in [true, false]) {
    for (final round in [true, false]) {
      test(
        '${spline ? 'cubic spline' : 'line and partial arc'} rim ${round ? 'fillet' : 'chamfer'}',
        () {
          final input = curvedPrism(spline),
              before = jsonEncode(
                curvedPrism(
                  spline,
                ).vertices.map((v) => v.storage.toList()).toList(),
              );
          final cap = input.faces.firstWhere(
            (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > .9,
          );
          final ids = cap.loops.single.coedges.map((c) => c.edge).toList();
          final solid = round
              ? IjoOps.filletSolid(input, ids, radius: 1, inverse: false)
              : IjoOps.chamferSolid(
                  input,
                  ids,
                  1,
                  options: const ChamferOptions(
                    mode: ChamferMode.twoDistance,
                    distance2: 2,
                  ),
                );
          closed(solid);
          final mesh = BrepTessellate.run(solid);
          expect(MeshKit.isClosedSolid(mesh), isTrue);
          expect(mesh.faces.length, lessThan(80000));
          expect(
            jsonEncode(input.vertices.map((v) => v.storage.toList()).toList()),
            before,
          );
          if (round) {
            final patches = solid.faces.where(
              (f) => f.surface is BrepSplineSurface,
            );
            expect(patches, isNotEmpty);
            for (final face in patches) {
              final surface = face.surface as BrepSplineSurface;
              for (final u in [.2, .5, .8]) {
                final a = surface.evaluate(u, 0), b = surface.evaluate(u, 1);
                expect(a.z, closeTo(10, 1e-6));
                expect(b.z, closeTo(9, 1e-6));
                expect(face.outwardNormal(u, 0).z, closeTo(1, 1e-5));
                final center = a - Vector3(0, 0, 1);
                for (final v in [.2, .5, .8])
                  expect(
                    (surface.evaluate(u, v) - center).length,
                    closeTo(1, 2e-5),
                  );
              }
            }
          }
        },
      );
    }
  }
  test('setbacks beyond wall length or local curvature are rejected', () {
    final input = curvedPrism(true);
    final cap = input.faces.firstWhere(
      (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > .9,
    );
    final ids = cap.loops.single.coedges.map((c) => c.edge).toList();
    expect(
      () => IjoOps.chamferSolid(
        input,
        ids,
        1,
        options: const ChamferOptions(
          mode: ChamferMode.twoDistance,
          distance2: 12,
        ),
      ),
      throwsA(isA<BrepUnsupported>()),
    );
    expect(
      () => IjoOps.filletSolid(input, ids, radius: 20, inverse: false),
      throwsA(isA<BrepUnsupported>()),
    );
  });
  test('unequal plane setbacks and angle have the same geometry', () {
    final s = BrepPrimitives.box(20, 30, 40);
    final a = IjoOps.chamferSolid(
      s,
      [0],
      2,
      options: const ChamferOptions(
        mode: ChamferMode.twoDistance,
        distance2: 4,
      ),
    );
    final b = IjoOps.chamferSolid(
      s,
      [0],
      2,
      options: ChamferOptions(
        mode: ChamferMode.setAngle,
        angleDegrees: math.atan(2) * 180 / math.pi,
      ),
    );
    closed(a);
    closed(b);
    expect(
      MeshKit.signedVolume(IjoOps.tessellateKeep(a)),
      closeTo(24000 - 20 * 2 * 4 / 2, .001),
    );
    expect(
      MeshKit.signedVolume(IjoOps.tessellateKeep(a)),
      closeTo(MeshKit.signedVolume(IjoOps.tessellateKeep(b)), 1e-6),
    );
  });
  test('invalid chamfer settings leave input unchanged', () {
    final s = BrepPrimitives.box(20, 30, 40);
    final before = jsonEncode(s.toJson());
    for (final angle in [0.0, 90.0, 180.0, double.nan]) {
      expect(
        () => IjoOps.chamferSolid(
          s,
          [0],
          2,
          options: ChamferOptions(
            mode: ChamferMode.setAngle,
            angleDegrees: angle,
          ),
        ),
        throwsA(isA<BrepUnsupported>()),
      );
    }
    expect(jsonEncode(s.toJson()), before);
  });
  for (final round in [false, true]) {
    test('cone rim ${round ? 'fillet' : 'chamfer'}', () {
      final s = BrepPrimitives.cone(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 10,
        height: 20,
      );
      final id = s.edges.indexWhere((e) => e.curve is BrepCircle);
      final out = round
          ? IjoOps.filletSolid(s, [id], radius: 1, inverse: false)
          : IjoOps.chamferSolid(
              s,
              [id],
              1,
              options: const ChamferOptions(
                mode: ChamferMode.setAngle,
                angleDegrees: 35,
              ),
            );
      closed(out);
    });
  }
  for (final round in [false, true]) {
    test('smooth ellipse rim ${round ? 'fillet' : 'chamfer'} remains BREP', () {
      final curve = BrepEllipse(
        center: Vector3.zero(),
        normal: Vector3(0, 0, 1),
        rMajor: 12,
        rMinor: 8,
        xDir: Vector3(1, 0, 0),
      );
      final s = BrepExtrude.fromCurves(
        [curve.evaluate(curve.t0)],
        [curve],
        Vector3(0, 0, 15),
      );
      final cap = s.faces.firstWhere(
        (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z > .9,
      );
      final ids = cap.loops
          .expand((l) => l.coedges)
          .map((e) => e.edge)
          .toList();
      final out = round
          ? IjoOps.filletSolid(s, ids, radius: 1, inverse: false)
          : IjoOps.chamferSolid(
              s,
              ids,
              1,
              options: const ChamferOptions(
                mode: ChamferMode.twoDistance,
                distance2: 2,
              ),
            );
      closed(out);
      expect(
        out.faces.any(
          (f) =>
              f.surface is BrepSplineSurface || f.surface is BrepRuledSurface,
        ),
        isTrue,
      );
      closed(BrepSolid.fromJson(out.toJson()));
    });
  }
}
