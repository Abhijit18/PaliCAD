import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void expectClosed(MeshData mesh) {
  expect(MeshSeal.isClosedSolid(mesh), isTrue);
  expect(MeshEdit.signedVolume(mesh), greaterThan(0));
}

void main() {
  test('cube is a closed solid with six face ids', () {
    final box = MeshPrimitives.cube(10, 20, 30);
    expectClosed(box);
    expect(box.faces, hasLength(12));
    expect(box.faceIds, isNotNull);
    expect(box.faceIds!.toSet(), {0, 1, 2, 3, 4, 5});
    expect(IjoMeasure.volume(box), closeTo(6000, 1e-6));
  });

  test('mesh CSG union of two cubes is closed', () {
    final a = MeshPrimitives.cube(10, 10, 10);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 0, 0),
    );
    final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
    expectClosed(u);
    expect(IjoMeasure.volume(u), closeTo(1500, 50));
  });

  test('mesh CSG subtract of overlapping cubes is closed', () {
    final a = MeshPrimitives.cube(20, 20, 20);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 5, -1),
    );
    final cut = MeshBoolean.run(a, b, MeshBoolean.subtractOp);
    expectClosed(cut);
    expect(IjoOps.splitSolids(cut), hasLength(1));
  });

  test('chamfer on a cube edge stays closed', () {
    final box = MeshPrimitives.cube(20, 20, 20);
    final out = IjoOps.chamferEdges(box, [
      (Vector3(0, 0, 0), Vector3(0, 0, 20)),
    ], 2);
    expectClosed(out);
  });

  test('hollow L-extrude with a top opening stays closed', () {
    final solid = BrepExtrude.profile([
      Vector3(0, 0, 0),
      Vector3(30, 0, 0),
      Vector3(30, 10, 0),
      Vector3(10, 10, 0),
      Vector3(10, 30, 0),
      Vector3(0, 30, 0),
    ], Vector3(0, 0, 16));
    final mesh = IjoOps.tessellateClosed(solid, op: 'Extrude');
    final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(mesh);
    final out = IjoOps.hollowPreferSolid(
      mesh,
      top,
      Vector3(0, 0, 1),
      wallThicknessMm: 2,
      solid: solid,
    );
    expect(out.solid, isNotNull);
    expectClosed(out.mesh);
    final outer = IjoMeasure.volume(mesh);
    final vol = IjoMeasure.volume(out.mesh);
    expect(vol, lessThan(outer * 0.85));
    expect(vol, greaterThan(outer * 0.15));
  });

  test('hollow L-extrude recovers a planar solid from the mesh', () {
    final ring = [
      Vector3(0, 0, 0),
      Vector3(30, 0, 0),
      Vector3(30, 10, 0),
      Vector3(10, 10, 0),
      Vector3(10, 30, 0),
      Vector3(0, 30, 0),
    ];
    final mesh = MeshBuild.sweepPolygon(ring, Vector3(0, 0, 16));
    final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(mesh);
    final out = IjoOps.hollowPreferSolid(
      mesh,
      top,
      Vector3(0, 0, 1),
      wallThicknessMm: 2,
    );
    expect(out.solid, isNotNull);
    expectClosed(out.mesh);
    final outer = IjoMeasure.volume(mesh);
    final vol = IjoMeasure.volume(out.mesh);
    expect(vol, lessThan(outer * 0.85));
    expect(vol, greaterThan(outer * 0.15));
  });

  test('hollow D-profile extrude stays closed', () {
    final ring = [
      Vector3(0, -8, 0),
      Vector3(12, -8, 0),
      for (var i = 0; i <= 8; i++)
        Vector3(
          12 + 8 * math.sin(i / 8 * math.pi),
          -8 * math.cos(i / 8 * math.pi),
          0,
        ),
      Vector3(12, 8, 0),
      Vector3(0, 8, 0),
    ];
    final solid = BrepExtrude.profile(ring, Vector3(0, 0, 18));
    final mesh = IjoOps.tessellateClosed(solid, op: 'Extrude');
    final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(mesh);
    final out = IjoOps.hollowPreferSolid(
      mesh,
      top,
      Vector3(0, 0, 1),
      wallThicknessMm: 1.5,
      solid: solid,
    );
    expect(out.solid, isNotNull);
    expectClosed(out.mesh);
    final outer = IjoMeasure.volume(mesh);
    final vol = IjoMeasure.volume(out.mesh);
    expect(vol, lessThan(outer * 0.85));
    expect(vol, greaterThan(outer * 0.15));
  });

  test('hollow cylinder with a top opening stays closed', () {
    final solid = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 10,
      height: 20,
    );
    final mesh = IjoOps.tessellateClosed(solid, op: 'Cylinder');
    final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(mesh);
    final out = IjoOps.hollowPreferSolid(
      mesh,
      top,
      Vector3(0, 0, 1),
      wallThicknessMm: 2,
      solid: solid,
    );
    expect(out.solid, isNotNull);
    expectClosed(out.mesh);
    final outer = IjoMeasure.volume(mesh);
    final vol = IjoMeasure.volume(out.mesh);
    expect(vol, lessThan(outer * 0.85));
    expect(vol, greaterThan(outer * 0.2));
  });

  test('hollow box with a top opening stays closed', () {
    final box = MeshPrimitives.cube(20, 20, 20);
    final out = IjoOps.hollow(
      box,
      [0, 1],
      Vector3(0, 0, 1),
      wallThicknessMm: 2,
    );
    expectClosed(out);
  });

  test('hollow box side opening ignores mismatched mesh faceIds', () {
    const w = 20.0, h = 20.0, d = 40.0;
    final mesh = MeshData(
      vertices: [
        Vector3(0, 0, 0),
        Vector3(w, 0, 0),
        Vector3(w, h, 0),
        Vector3(0, h, 0),
        Vector3(0, 0, d),
        Vector3(w, 0, d),
        Vector3(w, h, d),
        Vector3(0, h, d),
      ],
      faces: const [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 5],
        [1, 2, 5],
        [2, 3, 7],
        [2, 7, 6],
        [0, 4, 7],
        [0, 7, 3],
      ],
      faceIds: const [0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5],
    );
    final solid = BrepPrimitives.tryAabbBox(mesh);
    expect(solid, isNotNull);
    final out = IjoOps.hollowPreferSolid(
      mesh,
      const [6, 7],
      Vector3(1, 0, 0),
      wallThicknessMm: 2,
      solid: solid,
    );
    expect(out.solid, isNotNull);
    expectClosed(out.mesh);

    var covers = 0;
    for (final face in out.mesh.faces) {
      final a = out.mesh.vertices[face[0]];
      final b = out.mesh.vertices[face[1]];
      final c = out.mesh.vertices[face[2]];
      final cx = (a.x + b.x + c.x) / 3.0;
      if (cx < w - 0.75) continue;
      final v0y = c.y - a.y, v0z = c.z - a.z;
      final v1y = b.y - a.y, v1z = b.z - a.z;
      final v2y = 10.0 - a.y, v2z = 20.0 - a.z;
      final den = v0y * v1z - v1y * v0z;
      if (den.abs() < 1e-18) continue;
      final u = (v2y * v1z - v1y * v2z) / den;
      final v = (v0y * v2z - v2y * v0z) / den;
      if (u >= -1e-6 && v >= -1e-6 && (u + v) <= 1 + 1e-6) covers++;
    }
    expect(covers, 0, reason: '+X opening must not stay capped');
  });

  test('hollow filleted box top stays a thin shell', () {
    final box = BrepPrimitives.box(40, 40, 40);
    final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
    final rim = BrepEdgeSelect.ofFace(
      BrepFaceSelect.indices(top),
    ).and(BrepEdgeSelect.convex()).resolve(box);
    final filleted = IjoOps.filletSolid(box, rim, radius: 6, inverse: false);
    final mesh = IjoOps.tessellateClosed(filleted, op: 'Fillet');
    final opening = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(mesh);
    final out = IjoOps.hollowPreferSolid(
      mesh,
      opening,
      Vector3(0, 0, 1),
      wallThicknessMm: 2,
      solid: filleted,
    );
    expect(out.solid, isNotNull);
    expectClosed(out.mesh);
    final outer = IjoMeasure.volume(mesh);
    final vol = IjoMeasure.volume(out.mesh);
    expect(vol, lessThan(outer * 0.4));
    expect(vol, greaterThan(outer * 0.05));
    final innerCyls = [
      for (final f in out.solid!.faces)
        if (f.surface is BrepCylinder) (f.surface as BrepCylinder).radius,
    ];
    expect(
      innerCyls.where((r) => (r - 4).abs() < 0.05),
      isNotEmpty,
      reason: 'inner fillets must be outer radius minus wall (6-2=4)',
    );
    expect(
      innerCyls.where((r) => r < 0.05),
      isEmpty,
      reason: 'hollow must not collapse fillets into needle cylinders',
    );
  });

  test('hollow thicker than fillet radius does not mesh-fallback to a box', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
    final rim = BrepEdgeSelect.ofFace(
      BrepFaceSelect.indices(top),
    ).and(BrepEdgeSelect.convex()).resolve(box);
    final filleted = IjoOps.filletSolid(box, rim, radius: 2, inverse: false);
    final mesh = IjoOps.tessellateClosed(filleted, op: 'Fillet');
    final opening = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(mesh);
    expect(
      () => IjoOps.hollowPreferSolid(
        mesh,
        opening,
        Vector3(0, 0, 1),
        wallThicknessMm: 2,
        solid: filleted,
      ),
      throwsA(
        isA<BrepUnsupported>().having(
          (e) => e.code,
          'code',
          BrepUnsupported.filletOffset,
        ),
      ),
    );
  });

  test('hollow 8-edge face-filleted cube keeps inner rounds at r minus t', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final bottom = BrepFaceSelect.extreme(Vector3(0, 0, -1)).resolve(box);
    final bottomEdges = BrepEdgeSelect.ofFace(
      BrepFaceSelect.indices(bottom),
    ).resolve(box).toSet();
    final edges = [
      for (var i = 0; i < box.edges.length; i++)
        if (!bottomEdges.contains(i)) i,
    ];
    expect(edges, hasLength(8));
    final filleted = IjoOps.filletSolid(box, edges, radius: 2, inverse: false);
    final mesh = IjoOps.tessellateClosed(filleted, op: 'Fillet');
    final opening = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(mesh);
    final out = IjoOps.hollowPreferSolid(
      mesh,
      opening,
      Vector3(0, 0, 1),
      wallThicknessMm: 1,
      solid: filleted,
    );
    expect(out.solid, isNotNull);
    expectClosed(out.mesh);
    final innerCyls = [
      for (final f in out.solid!.faces)
        if (f.surface is BrepCylinder) (f.surface as BrepCylinder).radius,
    ];
    expect(
      innerCyls.where((r) => (r - 1).abs() < 0.05),
      isNotEmpty,
      reason: 'inner fillets must be outer radius minus wall (2-1=1)',
    );
    final drop = IjoMeasure.volume(mesh) - IjoMeasure.volume(out.mesh);
    expect(
      drop,
      isNot(closeTo(16 * 16 * 18, 2)),
      reason: 'cavity must follow the outer fillets, not a 16×16×18 box',
    );
  });

  test('extrude polygon is a closed prism', () {
    final mesh = MeshBuild.sweepPolygon([
      Vector3(0, 0, 0),
      Vector3(10, 0, 0),
      Vector3(10, 8, 0),
      Vector3(0, 8, 0),
    ], Vector3(0, 0, 5));
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), closeTo(400, 1e-3));
  });

  test('slice splits a cube into two solids', () {
    final box = MeshPrimitives.cube(20, 20, 20);
    final parts = IjoOps.sliceParts(box, Vector3(10, 10, 10), Vector3(0, 0, 1));
    expect(parts.length, greaterThanOrEqualTo(2));
    for (final p in parts) {
      expectClosed(p);
    }
  });

  test('analytic sphere tessellates closed with an outward +Z pole', () {
    const r = 10.0;
    final solid = BrepPrimitives.sphere(center: Vector3(r, r, r), radius: r);
    final mesh = IjoOps.tessellateClosed(solid, op: 'Sphere');
    expectClosed(mesh);
    expect(MeshFaceRegions.openBoundaryEdgeCount(mesh), 0);
    expect(IjoMeasure.volume(mesh), closeTo(4 / 3 * math.pi * r * r * r, 80));
    var poleArea = 0.0;
    for (final face in mesh.faces) {
      final a = mesh.vertices[face[0]];
      final b = mesh.vertices[face[1]];
      final c = mesh.vertices[face[2]];
      final mid = (a + b + c) / 3;
      if ((mid.z - (r + r)).abs() > r * 0.35) continue;
      final n = (b - a).cross(c - a);
      expect(n.z, greaterThan(0), reason: '+Z pole must face out');
      poleArea += 0.5 * n.length;
    }
    expect(poleArea, greaterThan(0));
  });

  test('analytic sphere union cube stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(10, 10, 22),
      radius: 8,
    );
    final u = IjoOps.booleanSolid(sphere, box, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepSphere), isTrue);
    expect(
      u.faces.where((f) => f.surface is BrepPlane).length,
      greaterThanOrEqualTo(6),
    );
    final mesh = IjoOps.tessellateKeep(u, op: 'Boolean');
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(8000));
    expect(
      IjoMeasure.volume(mesh),
      lessThan(8000 + 4 / 3 * math.pi * 8 * 8 * 8),
    );
    expect(IjoMeasure.boundingBox(mesh)!.max.z, greaterThan(25));
  });

  test('analytic cube union sphere stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(10, 10, 22),
      radius: 8,
    );
    final u = IjoOps.booleanSolid(box, sphere, IjoOps.unionOp);
    expect(u.faces.any((f) => f.surface is BrepSphere), isTrue);
    final mesh = IjoOps.tessellateKeep(u, op: 'Boolean');
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), greaterThan(8000));
    expect(IjoMeasure.boundingBox(mesh)!.max.z, greaterThan(25));
  });

  test('analytic cube union sphere has X-ray silhouettes on the dome', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(10, 10, 22),
      radius: 8,
    );
    final u = IjoOps.booleanSolid(box, sphere, IjoOps.unionOp);
    final mesh = IjoOps.tessellateKeep(u, op: 'Boolean');
    final sil = MeshFaceRegions.silhouetteEdges(
      mesh,
      towardEye: (_) => Vector3(-1, 0, 0),
    );
    expect(
      sil.where((e) {
        final z = (mesh.vertices[e.$1].z + mesh.vertices[e.$2].z) * 0.5;
        return z > 21;
      }),
      isNotEmpty,
      reason: 'X-ray must outline the spherical cap, not only the join circle',
    );
  });

  test('analytic cube subtract sphere stays B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final sphere = BrepPrimitives.sphere(
      center: Vector3(10, 10, 22),
      radius: 8,
    );
    final cut = IjoOps.booleanSolid(box, sphere, IjoOps.subtractOp);
    expect(cut.faces.any((f) => f.surface is BrepSphere), isTrue);
    final mesh = IjoOps.tessellateKeep(cut, op: 'Boolean');
    expectClosed(mesh);
    expect(IjoMeasure.volume(mesh), lessThan(8000));
    expect(IjoMeasure.volume(mesh), greaterThan(7000));
    expect(IjoMeasure.boundingBox(mesh)!.max.z, closeTo(20, 0.2));
  });

  test('cylinder from a sampled ring is closed', () {
    final ring = [
      for (var i = 0; i < 32; i++)
        Vector3(
          5 * math.cos(i / 32 * math.pi * 2),
          5 * math.sin(i / 32 * math.pi * 2),
          0,
        ),
    ];
    final mesh = MeshBuild.tryCylinderFromRing(ring, Vector3(0, 0, 20));
    expect(mesh, isNotNull);
    expectClosed(mesh!);
  });

  test('tryCylinderFromMesh recovers a tessellated circular extrude', () {
    final ring = [
      for (var i = 0; i < 32; i++)
        Vector3(
          5 * math.cos(i / 32 * math.pi * 2),
          5 * math.sin(i / 32 * math.pi * 2),
          0,
        ),
    ];
    final mesh = MeshBuild.tryCylinderFromRing(ring, Vector3(0, 0, 20));
    expect(mesh, isNotNull);
    final solid = BrepPrimitives.tryCylinderFromMesh(mesh!);
    expect(solid, isNotNull);
    expect(solid!.faces.any((f) => f.surface is BrepCylinder), isTrue);
  });

  test('hollow box minus cylinder stays a closed B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
    final shell = IjoOps.hollowSolid(
      box,
      top,
      Vector3(0, 0, 1),
      wallThicknessMm: 1,
    );
    final hole = BrepPrimitives.cylinder(
      origin: Vector3(10, 10, -2),
      axis: Vector3(0, 0, 1),
      radius: 4,
      height: 24,
    );
    final cut = IjoOps.booleanSolid(shell, hole, IjoOps.subtractOp);
    expectClosed(IjoOps.tessellateKeep(cut, op: 'Boolean'));
    expect(cut.faces.any((f) => f.surface is BrepCylinder), isTrue);
  });

  test('filleted box minus cylinder stays a closed B-Rep', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final top = BrepFaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
    final rim = BrepEdgeSelect.ofFace(
      BrepFaceSelect.indices(top),
    ).and(BrepEdgeSelect.convex()).resolve(box);
    final filleted = IjoOps.filletSolid(box, rim, radius: 2, inverse: false);
    final hole = BrepPrimitives.cylinder(
      origin: Vector3(10, 10, -2),
      axis: Vector3(0, 0, 1),
      radius: 4,
      height: 24,
    );
    final cut = IjoOps.booleanSolid(filleted, hole, IjoOps.subtractOp);
    expectClosed(IjoOps.tessellateKeep(cut, op: 'Boolean'));
    expect(cut.faces.any((f) => f.surface is BrepSphere), isTrue);
  });

  test(
    'booleanPreferSolid does not convert B-Rep to mesh when unsupported',
    () {
      final box = BrepPrimitives.box(20, 20, 20);
      final donut = BrepPrimitives.torus(
        center: Vector3(10, -3, 10),
        axis: Vector3(0, 0, 1),
        majorRadius: 8,
        minorRadius: 2,
      );
      final ma = IjoOps.tessellate(box);
      final mb = IjoOps.tessellate(donut);
      expect(
        () => IjoOps.booleanPreferSolid(box, donut, ma, mb, IjoOps.subtractOp),
        throwsA(isA<BrepUnsupported>()),
      );
    },
  );

  test('booleanPreferSolid still mesh-CSGs two mesh-only solids', () {
    final a = MeshPrimitives.cube(10, 10, 10);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 0, 0),
    );
    final out = IjoOps.booleanPreferSolid(null, null, a, b, IjoOps.unionOp);
    expect(out.solid, isNull);
    expectClosed(out.mesh);
  });

  test('booleanPreferSolid refuses mixed B-Rep and mesh', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final blob = IjoOps.tessellate(box);
    expect(
      () => IjoOps.booleanPreferSolid(box, null, blob, blob, IjoOps.subtractOp),
      throwsA(isA<BrepUnsupported>()),
    );
  });

  test('straight pipe prefers a cylinder B-Rep', () {
    final out = IjoOps.pipePreferSolid([
      Vector3.zero(),
      Vector3(0, 0, 40),
    ], outerRadius: 5);
    expect(out.solid, isNotNull);
    expect(out.solid!.faces.any((f) => f.surface is BrepCylinder), isTrue);
    expectClosed(out.mesh);
  });

  test('closed circular pipe prefers a torus B-Rep', () {
    final ring = [
      for (var i = 0; i < 24; i++)
        Vector3(
          15 * math.cos(2 * math.pi * i / 24),
          15 * math.sin(2 * math.pi * i / 24),
          0,
        ),
    ];
    final out = IjoOps.pipePreferSolid(ring, outerRadius: 3, closed: true);
    expect(out.solid, isNotNull);
    expect(out.solid!.faces.any((f) => f.surface is BrepTorus), isTrue);
    expectClosed(out.mesh);
  });

  test('open circular pipe prefers a torus-sector B-Rep', () {
    final arc = [
      for (var i = 0; i <= 16; i++)
        Vector3(
          15 * math.cos(math.pi / 2 * i / 16),
          15 * math.sin(math.pi / 2 * i / 16),
          0,
        ),
    ];
    final out = IjoOps.pipePreferSolid(arc, outerRadius: 3, closed: false);
    expect(out.solid, isNotNull);
    expect(out.solid!.faces.any((f) => f.surface is BrepTorus), isTrue);
    expectClosed(out.mesh);
  });

  test('elbow pipe prefers cylinder and torus B-Rep', () {
    final out = IjoOps.pipePreferSolid([
      Vector3.zero(),
      Vector3(20, 0, 0),
      Vector3(20, 0, 20),
    ], outerRadius: 4);
    expect(out.solid, isNotNull);
    expect(out.solid!.faces.any((f) => f.surface is BrepCylinder), isTrue);
    expect(out.solid!.faces.any((f) => f.surface is BrepTorus), isTrue);
    expectClosed(out.mesh);
  });

  test('faceted circular spine without closed flag is still a torus', () {
    final ring = [
      for (var i = 0; i < 32; i++)
        Vector3(
          15 * math.cos(2 * math.pi * i / 32),
          15 * math.sin(2 * math.pi * i / 32),
          0,
        ),
    ];
    final out = IjoOps.pipePreferSolid(ring, outerRadius: 3, closed: false);
    expect(out.solid, isNotNull);
    expect(out.solid!.faces.any((f) => f.surface is BrepTorus), isTrue);
    expectClosed(out.mesh);
  });
}
