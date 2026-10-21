import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('dense planar tessellation reduces to its four outline edges', () {
    const n = 100;
    final mesh = MeshData(
      vertices: [
        for (var y = 0; y <= n; y++)
          for (var x = 0; x <= n; x++) Vector3(x.toDouble(), y.toDouble(), 0),
      ],
      faces: [
        for (var y = 0; y < n; y++)
          for (var x = 0; x < n; x++) ...[
            [y * (n + 1) + x, y * (n + 1) + x + 1, (y + 1) * (n + 1) + x + 1],
            [y * (n + 1) + x, (y + 1) * (n + 1) + x + 1, (y + 1) * (n + 1) + x],
          ],
      ],
      faceIds: List.filled(2 * n * n, 0),
    );
    final watch = Stopwatch()..start();
    expect(MeshFaceRegions.featureEdges(mesh), hasLength(4));
    expect(watch.elapsed, lessThan(const Duration(seconds: 10)));
  });

  for (final angle in [0.0, 0.63, 1.57]) {
    test(
      'outline splits a shared edge at a T-junction after rotation $angle',
      () {
        final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), angle);
        final mesh = MeshData(
          vertices: [
            Vector3(0, 0, 0),
            Vector3(10, 0, 0),
            Vector3(10, 10, 0),
            Vector3(0, 10, 0),
            Vector3(5, 5, 0),
          ].map((p) => q.rotated(p) + Vector3(-31, 17, -9)).toList(),
          faces: [
            [0, 1, 2],
            [0, 4, 3],
            [4, 2, 3],
          ],
          faceIds: [0, 0, 0],
        );
        final edges = MeshFaceRegions.featureEdges(mesh);
        expect(edges, hasLength(4));
        expect(
          edges.any((e) => e.$1 == 4 || e.$2 == 4),
          isFalse,
          reason: 'The internal diagonal must cancel across the T-junction',
        );
      },
    );
  }

  test('growPlanarRegion absorbs a T-junction sliver on a flat face', () {
    // Two triangles of a 10×10 quad, plus a needle that shares the
    // midpoint of the hypotenuse (boolean leftover) instead of an edge.
    final mesh = MeshData(
      vertices: [
        Vector3(0, 0, 0),
        Vector3(10, 0, 0),
        Vector3(10, 10, 0),
        Vector3(0, 10, 0),
        Vector3(5, 5, 0),
        Vector3(5.01, 5, 1e-4),
        Vector3(5, 5.01, 0),
      ],
      faces: [
        [0, 1, 2],
        [0, 2, 3],
        [4, 5, 6],
      ],
    );
    final grown = MeshFaceRegions.growPlanarRegion(mesh, 0);
    expect(grown, containsAll([0, 1, 2]));
    expect(
      MeshFaceRegions.selectionRegion(mesh, 2).toSet(),
      MeshFaceRegions.selectionRegion(mesh, 0).toSet(),
      reason: 'clicking the leftover needle selects the host quad',
    );
  });

  test('growPlanarRegion does not jump to a disconnected coplanar island', () {
    final mesh = MeshData(
      vertices: [
        Vector3(0, 0, 0),
        Vector3(2, 0, 0),
        Vector3(2, 2, 0),
        Vector3(0, 2, 0),
        Vector3(10, 0, 0),
        Vector3(12, 0, 0),
        Vector3(12, 2, 0),
        Vector3(10, 2, 0),
      ],
      faces: [
        [0, 1, 2],
        [0, 2, 3],
        [4, 5, 6],
        [4, 6, 7],
      ],
    );
    final grown = MeshFaceRegions.growPlanarRegion(mesh, 0);
    expect(grown, unorderedEquals([0, 1]));
  });

  test(
    'polygon-with-holes triangulation only leaves outer and hole edges open',
    () {
      final outer = [
        Vector3(0, 0, 0),
        Vector3(40, 0, 0),
        Vector3(40, 20, 0),
        Vector3(0, 20, 0),
      ];
      final holes = [
        [
          Vector3(4, 4, 0),
          Vector3(12, 4, 0),
          Vector3(12, 12, 0),
          Vector3(4, 12, 0),
        ],
        [
          Vector3(24, 6, 0),
          Vector3(34, 6, 0),
          Vector3(34, 14, 0),
          Vector3(24, 14, 0),
        ],
      ];
      final tris = MeshPolygon.triangulate(outer, holes, Vector3(0, 0, 1));
      expect(tris.length, greaterThan(8));
      final mesh = MeshBuild.sweepPolygonWithHoles(
        outer,
        holes,
        Vector3(0, 0, 5),
      );
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(
        MeshKit.signedVolume(mesh),
        closeTo((40 * 20 - 8 * 8 - 10 * 8) * 5, 1),
      );

      final top = [
        for (var i = 0; i < mesh.faces.length; i++)
          if (mesh.vertices[mesh.faces[i][0]].z > 4.5) i,
      ];
      final region = MeshFaceRegions.growPlanarRegion(mesh, top.first);
      expect(region.toSet(), top.toSet());
      bool onRim(Vector3 m) {
        const holes = [
          [4.0, 4.0, 12.0, 12.0],
          [24.0, 6.0, 34.0, 14.0],
        ];
        for (final h in holes) {
          final on =
              (m.x - h[0]).abs() < 1e-4 ||
              (m.x - h[2]).abs() < 1e-4 ||
              (m.y - h[1]).abs() < 1e-4 ||
              (m.y - h[3]).abs() < 1e-4;
          if (on &&
              m.x >= h[0] - 1e-4 &&
              m.x <= h[2] + 1e-4 &&
              m.y >= h[1] - 1e-4 &&
              m.y <= h[3] + 1e-4) {
            return true;
          }
        }
        return m.x.abs() < 1e-4 ||
            m.y.abs() < 1e-4 ||
            (m.x - 40).abs() < 1e-4 ||
            (m.y - 20).abs() < 1e-4;
      }

      final ghosts = [
        for (final e in MeshFaceRegions.featureEdges(mesh))
          if (mesh.vertices[e.$1].z > 4.5 &&
              mesh.vertices[e.$2].z > 4.5 &&
              !onRim((mesh.vertices[e.$1] + mesh.vertices[e.$2]) * 0.5))
            e,
      ];
      expect(ghosts, isEmpty, reason: 'interior cap creases $ghosts');
    },
  );

  test(
    'selection outline of a holed cap hides fans that boundaryEdges keeps',
    () {
      final box = MeshPrimitives.cube(20, 20, 20);
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 5,
        height: 22,
      );
      final cut = IjoOps.boolean(box, cyl, IjoOps.subtractOp);
      final mesh = cut;
      final top = [
        for (var i = 0; i < mesh.faces.length; i++)
          if (mesh.vertices[mesh.faces[i][0]].z > 19.5) i,
      ];
      expect(top, isNotEmpty);
      final region = MeshFaceRegions.selectionRegion(mesh, top.first);
      expect(region.length, greaterThan(2));
      final boundary = MeshFaceRegions.boundaryEdges(mesh, region);
      final outline = MeshFaceRegions.selectionOutline(mesh, region);
      expect(outline, isNotEmpty);
      expect(outline.length, lessThanOrEqualTo(boundary.length));
      final featureKeys = {
        for (final e in MeshFaceRegions.featureEdges(
          mesh,
          includeSmoothCreases: false,
        ))
          MeshFaceRegions.edgeKey(mesh.vertices[e.$1], mesh.vertices[e.$2]),
      };
      for (final e in outline) {
        expect(
          featureKeys.contains(
            MeshFaceRegions.edgeKey(mesh.vertices[e.$1], mesh.vertices[e.$2]),
          ),
          isTrue,
          reason: 'highlight outline must be a body feature edge',
        );
      }
      expect(outline.length, greaterThan(8));
    },
  );

  test('selection outline of a box face is the four rims', () {
    final mesh = MeshPrimitives.cube(10, 10, 10);
    final top = [
      for (var i = 0; i < mesh.faces.length; i++)
        if (mesh.vertices[mesh.faces[i][0]].z > 9.5) i,
    ].first;
    final region = MeshFaceRegions.selectionRegion(mesh, top);
    final outline = MeshFaceRegions.selectionOutline(mesh, region);
    expect(outline.length, 4);
  });

  test('originalFaceIds is one ID per cube plane, not per triangle', () {
    // No B-Rep faceIds — the CSG display path. Six supporting planes.
    final mesh = MeshData(
      vertices: [
        Vector3(0, 0, 0),
        Vector3(10, 0, 0),
        Vector3(10, 10, 0),
        Vector3(0, 10, 0),
        Vector3(0, 0, 10),
        Vector3(10, 0, 10),
        Vector3(10, 10, 10),
        Vector3(0, 10, 10),
      ],
      faces: [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [3, 2, 6],
        [3, 6, 7],
        [0, 3, 7],
        [0, 7, 4],
        [1, 5, 6],
        [1, 6, 2],
      ],
    );
    final ids = MeshFaceRegions.originalFaceIds(mesh);
    expect(ids.toSet(), hasLength(6));
    expect(MeshFaceRegions.featureEdges(mesh), hasLength(12));
  });

  test('originalFaceIds hides corner-fan radii on a holed square', () {
    // Naive fan from outer corners into a square hole — the leftover
    // look after mesh CSG cuts a planar face.
    final mesh = MeshData(
      vertices: [
        Vector3(0, 0, 0),
        Vector3(10, 0, 0),
        Vector3(10, 10, 0),
        Vector3(0, 10, 0),
        Vector3(3, 3, 0),
        Vector3(7, 3, 0),
        Vector3(7, 7, 0),
        Vector3(3, 7, 0),
      ],
      faces: [
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
      ],
    );
    final ids = MeshFaceRegions.originalFaceIds(mesh);
    expect(ids.toSet(), hasLength(1));
    final outline = MeshFaceRegions.featureEdges(mesh);
    expect(outline, hasLength(8), reason: '4 outer + 4 hole, no radii');
  });

  test('crease sliver on a cube edge does not hide that rim', () {
    // CSG leftover: a zero-area triangle lying on a cube edge. T-split
    // used to count it as a second use of the rim and drop the outline.
    final mesh = MeshData(
      vertices: [
        Vector3(0, 0, 0),
        Vector3(10, 0, 0),
        Vector3(10, 10, 0),
        Vector3(0, 10, 0),
        Vector3(0, 0, 10),
        Vector3(10, 0, 10),
        Vector3(10, 10, 10),
        Vector3(0, 10, 10),
        Vector3(10, 0, 6),
      ],
      faces: [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [3, 2, 6],
        [3, 6, 7],
        [0, 3, 7],
        [0, 7, 4],
        [1, 5, 6],
        [1, 6, 2],
        [1, 8, 5],
      ],
    );
    final rims = MeshFaceRegions.featureEdges(mesh, includeSmoothCreases: false)
        .where((e) {
          final a = mesh.vertices[e.$1];
          final b = mesh.vertices[e.$2];
          return (a.x - 10).abs() < 1e-6 &&
              (b.x - 10).abs() < 1e-6 &&
              a.y.abs() < 1e-6 &&
              b.y.abs() < 1e-6;
        });
    var covered = 0.0;
    for (final e in rims) {
      covered += (mesh.vertices[e.$1] - mesh.vertices[e.$2]).length;
    }
    expect(covered, greaterThan(9.5), reason: 'x=10 y=0 cube edge');
  });

  test('offset sphere∪cube is one CAD face with four 20 mm outer edges', () {
    // Sphere poking through x=0, circle well inside the square so the
    // planar face is one annulus (not the inscribed 4-corner pinch).
    final box = MeshPrimitives.cube(20, 20, 20);
    final sph = MeshPrimitives.sphere(center: Vector3(-8, 10, 10), radius: 10);
    final u = MeshBoolean.run(box, sph, MeshBoolean.unionOp);
    expect(MeshSeal.isClosedSolid(u), isTrue);

    final seeds = <int>[];
    for (var i = 0; i < u.faces.length; i++) {
      final face = u.faces[i];
      if (face.length < 3) continue;
      if (!face.every((vi) => u.vertices[vi].x.abs() < 0.4)) continue;
      final cr = (u.vertices[face[1]] - u.vertices[face[0]]).cross(
        u.vertices[face[2]] - u.vertices[face[0]],
      );
      if (cr.length2 < 1e-8) continue;
      if (cr.normalized().dot(Vector3(-1, 0, 0)) > 0.5) seeds.add(i);
    }
    expect(seeds, isNotEmpty);

    final regions = [
      for (final s in seeds) MeshFaceRegions.selectionRegion(u, s).toSet(),
    ];
    for (final r in regions) {
      expect(
        r,
        regions.first,
        reason:
            'every click on the cut plane is the same CAD face, '
            'not overlapping Steiner fragments',
      );
    }
    expect(regions.first.containsAll(seeds), isTrue);

    int rimCount(bool Function(Vector3 p) on) => MeshFaceRegions.featureEdges(
      u,
      includeSmoothCreases: false,
    ).where((e) => on(u.vertices[e.$1]) && on(u.vertices[e.$2])).length;

    double rimLength(bool Function(Vector3 p) on) {
      var len = 0.0;
      for (final e in MeshFaceRegions.featureEdges(
        u,
        includeSmoothCreases: false,
      )) {
        if (!on(u.vertices[e.$1]) || !on(u.vertices[e.$2])) continue;
        len += (u.vertices[e.$1] - u.vertices[e.$2]).length;
      }
      return len;
    }

    bool x0y0(Vector3 p) => p.x.abs() < 0.4 && p.y.abs() < 0.4;
    bool x0y20(Vector3 p) => p.x.abs() < 0.4 && (p.y - 20).abs() < 0.4;
    bool x0z0(Vector3 p) => p.x.abs() < 0.4 && p.z.abs() < 0.4;
    bool x0z20(Vector3 p) => p.x.abs() < 0.4 && (p.z - 20).abs() < 0.4;

    expect(rimCount(x0y0), 1, reason: 'x=0 y=0 must be one 20 mm edge');
    expect(rimCount(x0y20), 1, reason: 'x=0 y=20 must be one 20 mm edge');
    expect(rimCount(x0z0), 1, reason: 'x=0 z=0 must be one 20 mm edge');
    expect(rimCount(x0z20), 1, reason: 'x=0 z=20 must be one 20 mm edge');
    expect(rimLength(x0y0), closeTo(20, 0.05));
    expect(rimLength(x0y20), closeTo(20, 0.05));
    expect(rimLength(x0z0), closeTo(20, 0.05));
    expect(rimLength(x0z20), closeTo(20, 0.05));

    final outline = MeshFaceRegions.selectionOutline(u, regions.first.toList());
    expect(
      outline.where((e) {
        final a = u.vertices[e.$1];
        final b = u.vertices[e.$2];
        return a.x.abs() < 0.4 &&
            b.x.abs() < 0.4 &&
            ((a.y.abs() < 0.4 && b.y.abs() < 0.4) ||
                ((a.y - 20).abs() < 0.4 && (b.y - 20).abs() < 0.4) ||
                (a.z.abs() < 0.4 && b.z.abs() < 0.4) ||
                ((a.z - 20).abs() < 0.4 && (b.z - 20).abs() < 0.4));
      }).length,
      4,
      reason: 'highlight outline keeps the four outer rims as four strokes',
    );

    expect(MeshFaceRegions.isSliverFace(u, seeds.first), isFalse);

    final tool = MeshBuild.extrudeRegion(
      mesh: u,
      region: regions.first.toList(),
      direction: Vector3(2, 0, 0),
    );
    expect(MeshSeal.isClosedSolid(tool), isTrue);
  });

  test('all-negative faceIds still yield box creases', () {
    final box = MeshPrimitives.cube(10, 10, 10);
    final untagged = MeshData(
      vertices: [for (final v in box.vertices) v.clone()],
      faces: [for (final f in box.faces) List<int>.from(f)],
      faceIds: List<int>.filled(box.faces.length, -1),
    );
    expect(MeshFaceRegions.featureEdges(untagged).length, 12);
    final outline = MeshFaceRegions.selectionOutline(
      untagged,
      MeshFaceRegions.selectionRegion(untagged, 2),
    );
    expect(outline.length, 4);
  });

  test('a tessellated sphere has silhouette edges from the side', () {
    final solid = BrepPrimitives.sphere(center: Vector3.zero(), radius: 10);
    final mesh = IjoOps.tessellateKeep(solid, op: 'Sphere');
    expect(MeshFaceRegions.featureEdges(mesh), isEmpty);
    expect(
      MeshFaceRegions.silhouetteEdges(mesh, towardEye: (_) => Vector3(1, 0, 0)),
      isNotEmpty,
    );
  });
}
