import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  group('analytic fillet', () {
    test('box edge fillet is a cylinder and tessellates closed', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final edges = BrepEdgeSelect.linear(
        direction: Vector3(0, 0, 1),
      ).resolve(box);
      expect(edges, isNotEmpty);

      final sw = Stopwatch()..start();
      final filleted = IjoOps.filletSolid(
        box,
        [edges.first],
        radius: 3,
        inverse: false,
      );
      expect(sw.elapsedMilliseconds, lessThan(500));

      expect(filleted.faces.any((f) => f.surface is BrepCylinder), isTrue);
      final mesh = IjoOps.tessellateClosed(filleted, op: 'Fillet');
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(mesh.faceIds, isNotNull);
    });

    test('box face loop fillet stays analytic and closed', () {
      final box = BrepPrimitives.box(30, 20, 10);
      final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
      expect(top, isNotEmpty);
      final rim = BrepEdgeSelect.ofFace(
        BrepFaceSelect.indices(top),
      ).and(BrepEdgeSelect.convex()).resolve(box);
      expect(rim.length, greaterThanOrEqualTo(4));

      final filleted = IjoOps.filletSolid(box, rim, radius: 2, inverse: false);
      expect(
        filleted.faces.where((f) => f.surface is BrepCylinder).length,
        greaterThanOrEqualTo(8),
      );
      expect(
        MeshKit.isClosedSolid(IjoOps.tessellateClosed(filleted, op: 'Fillet')),
        isTrue,
      );
    });

    test('completeCornerEdges pulls uprights from a cube face loop', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
      final rim = BrepEdgeSelect.ofFace(
        BrepFaceSelect.indices(top),
      ).and(BrepEdgeSelect.convex()).resolve(box);
      expect(rim, hasLength(4));
      final expanded = BrepFillet.completeCornerEdges(box, rim);
      expect(expanded, hasLength(8));
      expect(expanded, containsAll(rim));
    });

    test(
      'cube face fillet rounds 8 edges and leaves the opposite face planar',
      () {
        final box = BrepPrimitives.box(20, 20, 20);
        final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
        final rim = BrepEdgeSelect.ofFace(
          BrepFaceSelect.indices(top),
        ).and(BrepEdgeSelect.convex()).resolve(box);

        final sw = Stopwatch()..start();
        final filleted = IjoOps.filletSolid(
          box,
          rim,
          radius: 3,
          inverse: false,
        );
        expect(sw.elapsedMilliseconds, lessThan(500));

        final cylinders = [
          for (final f in filleted.faces)
            if (f.surface is BrepCylinder) f.surface as BrepCylinder,
        ];
        expect(cylinders, hasLength(8));
        final uprights = cylinders.where(
          (c) => c.axis.normalized().dot(Vector3(0, 0, 1)).abs() > 0.9,
        );
        final caps = cylinders.where(
          (c) => c.axis.normalized().dot(Vector3(0, 0, 1)).abs() < 0.3,
        );
        expect(uprights, hasLength(4));
        expect(caps, hasLength(4));

        final spheres = [
          for (final f in filleted.faces)
            if (f.surface is BrepSphere) f.surface as BrepSphere,
        ];
        expect(spheres, hasLength(4));
        for (final s in spheres) {
          expect(s.center.z, closeTo(17, 1e-6));
        }

        final bottom = filleted.faces.where((f) {
          final p = f.surface;
          return p is BrepPlane &&
              p.normal.normalized().dot(Vector3(0, 0, -1)) > 0.9;
        }).toList();
        expect(bottom, isNotEmpty);
        final bot = bottom.first;
        final botLines = bot.loops.first.coedges
            .where((c) => filleted.edges[c.edge].curve is BrepLine)
            .length;
        final botArcs = bot.loops.first.coedges
            .where((c) => filleted.edges[c.edge].curve is BrepCircle)
            .length;
        expect(botLines, 4);
        expect(botArcs, 4);

        expect(
          MeshKit.isClosedSolid(
            IjoOps.tessellateClosed(filleted, op: 'Fillet'),
          ),
          isTrue,
        );
      },
    );

    test('triangle prism face fillet rounds the loop and the uprights', () {
      final tri = BrepPrimitives.prism(diameter: 24, height: 16, sides: 3);
      final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(tri);
      final rim = BrepEdgeSelect.ofFace(
        BrepFaceSelect.indices(top),
      ).and(BrepEdgeSelect.convex()).resolve(tri);
      expect(rim, hasLength(3));
      expect(BrepFillet.completeCornerEdges(tri, rim), hasLength(6));

      final filleted = IjoOps.filletSolid(tri, rim, radius: 2, inverse: false);
      expect(
        filleted.faces.where((f) => f.surface is BrepCylinder),
        hasLength(6),
      );
      expect(
        filleted.faces.where((f) => f.surface is BrepSphere),
        hasLength(3),
      );
      expect(
        MeshKit.isClosedSolid(IjoOps.tessellateClosed(filleted, op: 'Fillet')),
        isTrue,
      );
    });

    test('hex prism face fillet rounds the loop and the uprights', () {
      final hex = BrepPrimitives.prism(diameter: 24, height: 16, sides: 6);
      final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(hex);
      final rim = BrepEdgeSelect.ofFace(
        BrepFaceSelect.indices(top),
      ).and(BrepEdgeSelect.convex()).resolve(hex);
      expect(rim, hasLength(6));
      expect(BrepFillet.completeCornerEdges(hex, rim), hasLength(12));

      final filleted = IjoOps.filletSolid(hex, rim, radius: 2, inverse: false);
      final cylinders = [
        for (final f in filleted.faces)
          if (f.surface is BrepCylinder) f.surface as BrepCylinder,
      ];
      expect(cylinders, hasLength(12));
      expect(
        cylinders.where(
          (c) => c.axis.normalized().dot(Vector3(0, 0, 1)).abs() > 0.9,
        ),
        hasLength(6),
      );
      expect(
        filleted.faces.where((f) => f.surface is BrepSphere),
        hasLength(6),
      );

      final bottom = filleted.faces.firstWhere((f) {
        final p = f.surface;
        return p is BrepPlane &&
            p.normal.normalized().dot(Vector3(0, 0, -1)) > 0.9;
      });
      final botLines = bottom.loops.first.coedges
          .where((c) => filleted.edges[c.edge].curve is BrepLine)
          .length;
      final botArcs = bottom.loops.first.coedges
          .where((c) => filleted.edges[c.edge].curve is BrepCircle)
          .length;
      expect(botLines, 6);
      expect(botArcs, 6);
      expect(
        MeshKit.isClosedSolid(IjoOps.tessellateClosed(filleted, op: 'Fillet')),
        isTrue,
      );
    });

    test('cylinder rim fillet is a torus', () {
      final cyl = BrepPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 8,
        height: 16,
      );
      final rim = BrepEdgeSelect.circular().resolve(cyl);
      expect(rim, isNotEmpty);

      final filleted = IjoOps.filletSolid(
        cyl,
        [rim.first],
        radius: 1.5,
        inverse: false,
      );
      expect(filleted.faces.any((f) => f.surface is BrepTorus), isTrue);
      expect(
        MeshKit.isClosedSolid(IjoOps.tessellateClosed(filleted, op: 'Fillet')),
        isTrue,
      );
    });

    test('box subtract cylinder then fillet the circular rim', () {
      final block = BrepPrimitives.box(30, 20, 10);
      final hole = BrepPrimitives.cylinder(
        origin: Vector3(15, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 4,
        height: 12,
      );
      final part = IjoOps.booleanSolid(block, hole, IjoOps.subtractOp);
      final rim = BrepEdgeSelect.circular().resolve(part);
      expect(rim, isNotEmpty);

      final filleted = IjoOps.filletSolid(
        part,
        [rim.first],
        radius: 1,
        inverse: false,
      );
      expect(
        filleted.faces.any(
          (f) => f.surface is BrepTorus || f.surface is BrepCylinder,
        ),
        isTrue,
      );
      expect(
        MeshKit.isClosedSolid(IjoOps.tessellateClosed(filleted, op: 'Fillet')),
        isTrue,
      );
    });

    test('box corner walk fillets three edges and stays closed', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final sw = Stopwatch()..start();
      final filleted = IjoOps.filletSolid(
        box,
        [0, 8, 3],
        radius: 2,
        inverse: false,
      );
      expect(sw.elapsedMilliseconds, lessThan(500));
      expect(
        filleted.faces.where((f) => f.surface is BrepCylinder).length,
        greaterThanOrEqualTo(2),
      );
      expect(
        filleted.faces.where((f) => f.surface is BrepSphere),
        hasLength(1),
      );
      expect(
        MeshKit.isClosedSolid(IjoOps.tessellateClosed(filleted, op: 'Fillet')),
        isTrue,
      );
    });

    test('two adjacent box edges get a spherical corner, not a point', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final filleted = IjoOps.filletSolid(
        box,
        [4, 7],
        radius: 2,
        inverse: false,
      );
      expect(
        filleted.faces.where((f) => f.surface is BrepSphere),
        hasLength(1),
      );
      expect(
        filleted.faces.where((f) => f.surface is BrepCylinder).length,
        2, // One cylinder for each selected edge; the third edge stays sharp.
      );
      final mesh = IjoOps.tessellateClosed(filleted, op: 'Fillet');
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      final spheres = [
        for (final f in filleted.faces)
          if (f.surface is BrepSphere) f.surface as BrepSphere,
      ];
      final c = spheres.first.center;
      final r = spheres.first.radius;
      final midN = Vector3(-1, -1, 1).normalized();
      final bulge = c + midN * r;
      var nearest = double.infinity;
      for (final v in mesh.vertices) {
        final d = (v - bulge).length;
        if (d < nearest) nearest = d;
      }
      expect(nearest, lessThan(0.35));
    });

    test('cube face chamfer expands to the uprights and stays closed', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
      final rim = BrepEdgeSelect.ofFace(
        BrepFaceSelect.indices(top),
      ).and(BrepEdgeSelect.convex()).resolve(box);
      final chamfered = IjoOps.chamferSolid(box, rim, 3);
      expect(chamfered.faces.where((f) => f.surface is BrepSphere), isEmpty);
      expect(
        MeshKit.isClosedSolid(
          IjoOps.tessellateClosed(chamfered, op: 'Chamfer'),
        ),
        isTrue,
      );
    });

    test('inverse fillet on a box edge is a centered circular cut', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final filleted = IjoOps.filletSolid(box, [0], radius: 2, inverse: true);
      expect(filleted.faces.any((f) => f.surface is BrepCylinder), isTrue);
      expect(
        MeshKit.isClosedSolid(IjoOps.tessellateClosed(filleted, op: 'Fillet')),
        isTrue,
      );
    });
  });
}
