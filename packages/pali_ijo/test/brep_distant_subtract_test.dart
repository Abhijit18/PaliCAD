import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/measure/bounds.dart' show conservativeSolidBounds;
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid movedBox(double x, double y, double z, Vector3 offset) =>
    BrepXform.map(BrepPrimitives.box(x, y, z), (p) => p + offset);

void expectSharedCurves(BrepSolid solid) {
  final uses = <int, int>{};
  for (final face in solid.faces) {
    for (final loop in face.loops) {
      for (final coedge in loop.coedges) {
        uses.update(coedge.edge, (n) => n + 1, ifAbsent: () => 1);
      }
    }
  }
  expect(uses, isNotEmpty);
  for (final entry in uses.entries) {
    expect(
      entry.value,
      2,
      reason: 'Edge ${entry.key} must be shared by two faces',
    );
  }
  expect(BrepHeal.topologyIsClosed(solid), isTrue);
}

double volume(BrepSolid solid) =>
    IjoMeasure.volume(IjoOps.tessellateClosed(solid));

void main() {
  test('conservative bounds include curved extrema after rotation', () {
    final solids = [
      BrepPrimitives.sphere(center: Vector3.zero(), radius: 10),
      BrepPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(1, 2, 3),
        radius: 5,
        height: 20,
      ),
      BrepPrimitives.cone(
        origin: Vector3.zero(),
        axis: Vector3(1, 2, 3),
        radius: 5,
        height: 20,
      ),
      BrepPrimitives.torus(
        center: Vector3.zero(),
        axis: Vector3(1, 2, 3),
        majorRadius: 10,
        minorRadius: 2,
      ),
    ];
    for (final solid in solids) {
      final bounds = conservativeSolidBounds(solid)!;
      for (final p in IjoOps.tessellate(solid).vertices) {
        expect(bounds.containsPoint(p, tolerance: 1e-8), isTrue);
      }
    }
  });

  test('overlapping spheres are not rejected using their sparse vertices', () {
    final a = BrepPrimitives.sphere(center: Vector3.zero(), radius: 10);
    final b = BrepPrimitives.sphere(center: Vector3(15, 0, 0), radius: 10);
    final result = IjoOps.booleanSolid(a, b, IjoOps.subtractOp);
    expect(volume(result), lessThan(volume(a) - 100));
    expect(result.faces, hasLength(2));
  });

  test('nonintersecting cutter within the bounds keeps the fillet', () {
    final input = IjoOps.filletSolid(
      BrepPrimitives.box(20, 20, 20),
      [0],
      radius: 2,
      inverse: false,
    );
    final tool = movedBox(0.1, 0.1, 0.1, Vector3(10, 0.05, 0.05));
    expect(
      conservativeSolidBounds(
        input,
      )!.intersects(conservativeSolidBounds(tool)!),
      isTrue,
    );
    final result = IjoOps.booleanSolid(input, tool, IjoOps.subtractOp);
    expectSharedCurves(result);
    expect(volume(result), closeTo(volume(input), 1e-6));
  });
  for (final faceLoop in [false, true]) {
    for (final offset in [
      Vector3(100, 100, 100),
      Vector3(100, 0, 0),
      Vector3(0, 100, 0),
    ]) {
      test(
        'distant subtract preserves ${faceLoop ? "face-loop" : "single-edge"} fillet at $offset',
        () {
          final box = BrepPrimitives.box(20, 20, 20);
          final edges = faceLoop
              ? BrepEdgeSelect.ofFace(
                  BrepFaceSelect.extreme(Vector3(0, 0, 1)),
                ).resolve(box)
              : [0];
          final input = IjoOps.filletSolid(
            box,
            edges,
            radius: 2,
            inverse: false,
          );
          final before = input.toJson();
          final result = IjoOps.booleanSolid(
            input,
            movedBox(5, 5, 5, offset),
            IjoOps.subtractOp,
          );
          expectSharedCurves(result);
          expect(IjoOps.splitBrep(result), hasLength(1));
          expect(result.faces.length, input.faces.length);
          expect(
            result.edges.where((e) => e.curve is BrepCircle).length,
            input.edges.where((e) => e.curve is BrepCircle).length,
          );
          expect(volume(result), closeTo(volume(input), 1e-6));
          expect(
            input.toJson(),
            before,
            reason: 'Boolean must not modify its input',
          );
        },
      );
    }
  }
  for (final through in [false, true]) {
    test(
      '${through ? "through cut" : "pocket"} removes material away from fillet',
      () {
        final input = IjoOps.filletSolid(
          BrepPrimitives.box(20, 20, 20),
          [0],
          radius: 2,
          inverse: false,
        );
        final tool = through
            ? movedBox(5, 5, 22, Vector3(10, 10, -1))
            : movedBox(5, 5, 5, Vector3(10, 10, 18));
        final result = IjoOps.booleanSolid(input, tool, IjoOps.subtractOp);
        expectSharedCurves(result);
        expect(IjoOps.splitBrep(result), hasLength(1));
        expect(
          result.faces.where((f) => f.surface is BrepCylinder),
          hasLength(1),
        );
        expect(
          volume(result),
          closeTo(volume(input) - (through ? 500 : 50), 1e-6),
        );
      },
    );
  }
}
