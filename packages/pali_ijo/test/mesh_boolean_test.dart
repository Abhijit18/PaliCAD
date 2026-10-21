import 'dart:math' as math;

import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/csg/boolean.dart';
import 'package:pali_ijo/src/csg/mesh_solid.dart';
import 'package:pali_ijo/src/mesh/mesh_build.dart';
import 'package:pali_ijo/src/mesh/mesh_data.dart';
import 'package:pali_ijo/src/mesh/mesh_edit.dart';
import 'package:pali_ijo/src/mesh/mesh_regions.dart';
import 'package:pali_ijo/src/mesh/mesh_seal.dart';
import 'package:pali_ijo/src/mesh/mesh_topology.dart';
import 'package:pali_ijo/src/mesh/packed_mesh.dart';
import 'package:pali_ijo/src/ops/ops.dart';

bool _onPlaneX(Vector3 p, double x) => (p.x - x).abs() < 0.3;

int _planeFeatureCount(
  MeshData mesh,
  bool Function(Vector3) onPlane, {
  bool includeSmoothCreases = true,
}) {
  return MeshFaceRegions.featureEdges(
    mesh,
    includeSmoothCreases: includeSmoothCreases,
  ).where((e) {
    return onPlane(mesh.vertices[e.$1]) && onPlane(mesh.vertices[e.$2]);
  }).length;
}

/// Feature edges on [onPlane] that are not the outer rectangle or a
/// circular blend of radius [circleR] about [circleCenter] in that plane.
int _radialFanCount({
  required MeshData mesh,
  required bool Function(Vector3) onPlane,
  required bool Function(Vector3) onOuterRim,
  Vector3? circleCenter,
  double? circleR,
}) {
  double? rOf(Vector3 p) {
    if (circleCenter == null || circleR == null) return null;
    return (p - circleCenter).length;
  }

  var n = 0;
  for (final e in MeshFaceRegions.featureEdges(
    mesh,
    includeSmoothCreases: true,
  )) {
    final a = mesh.vertices[e.$1];
    final b = mesh.vertices[e.$2];
    if (!onPlane(a) || !onPlane(b)) continue;
    if (onOuterRim(a) && onOuterRim(b)) continue;
    final ra = rOf(a);
    final rb = rOf(b);
    if (ra != null &&
        rb != null &&
        (ra - circleR!).abs() < 0.7 &&
        (rb - circleR).abs() < 0.7) {
      continue;
    }
    n++;
  }
  return n;
}

int _coplanarIdsOn(MeshData mesh, bool Function(Vector3) vertOnPlane) {
  final ids = MeshFaceRegions.originalFaceIds(mesh);
  final seen = <int>{};
  for (var f = 0; f < mesh.faces.length; f++) {
    final face = mesh.faces[f];
    if (face.length < 3) continue;
    if (!face.every((i) => vertOnPlane(mesh.vertices[i]))) continue;
    final a = mesh.vertices[face[0]];
    final b = mesh.vertices[face[1]];
    final c = mesh.vertices[face[2]];
    final cr = (b - a).cross(c - a);
    // Crease needles on the square's edges also have all verts on this
    // plane; they belong to a neighboring face. Skip them.
    if (cr.length2 < 1e-8) continue;
    seen.add(ids[f]);
  }
  return seen.length;
}

/// Same keep-closed policy FaceExtrudeSession runs on the swept prism.
MeshData _faceExtrudePrism(MeshData mesh, List<int> region, Vector3 dir) {
  return MeshSeal.preserveClosed(
    MeshBuild.extrudeRegion(mesh: mesh, region: region, direction: dir),
  );
}

/// Largest planar −X region on a cube face at x≈0. First-hit seeds can
/// be a sphere sliver that happens to sit near the plane.
int? _minusXPlanarSeed(MeshData mesh) {
  var best = -1;
  var bestN = 0;
  final seen = <int>{};
  for (var i = 0; i < mesh.faces.length; i++) {
    if (seen.contains(i)) continue;
    final face = mesh.faces[i];
    if (face.length < 3) continue;
    if (!face.every((vi) => mesh.vertices[vi].x.abs() < 0.5)) continue;
    final cr = (mesh.vertices[face[1]] - mesh.vertices[face[0]]).cross(
      mesh.vertices[face[2]] - mesh.vertices[face[0]],
    );
    if (cr.length2 < 1e-8) continue;
    if (cr.normalized().dot(Vector3(-1, 0, 0)) <= 0.5) continue;
    final region = MeshFaceRegions.selectionRegion(mesh, i);
    seen.addAll(region);
    if (region.length > bestN) {
      bestN = region.length;
      best = i;
    }
  }
  return best < 0 ? null : best;
}

/// Covered length of a cube-edge after merging feature-edge segments.
double _rimCovered(
  MeshData mesh, {
  required bool Function(Vector3) onEdge,
  required double Function(Vector3) along,
  double length = 40,
}) {
  final segs = <(double, double)>[];
  for (final e in MeshFaceRegions.featureEdges(
    mesh,
    includeSmoothCreases: false,
  )) {
    final a = mesh.vertices[e.$1];
    final b = mesh.vertices[e.$2];
    if (!onEdge(a) || !onEdge(b)) continue;
    var t0 = along(a);
    var t1 = along(b);
    if (t0 > t1) {
      final s = t0;
      t0 = t1;
      t1 = s;
    }
    segs.add((t0.clamp(0, length), t1.clamp(0, length)));
  }
  segs.sort((p, q) => p.$1.compareTo(q.$1));
  var covered = 0.0;
  var cursor = 0.0;
  for (final s in segs) {
    final start = math.max(s.$1, cursor);
    if (s.$2 > start) covered += s.$2 - start;
    if (s.$2 > cursor) cursor = s.$2;
  }
  return covered;
}

void main() {
  group('MeshBoolean', () {
    test('disjoint cubes concatenate to a closed solid', () {
      final a = MeshPrimitives.cube(10, 10, 10);
      final b = MeshPrimitives.translate(
        MeshPrimitives.cube(10, 10, 10),
        Vector3(20, 0, 0),
      );
      final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);
      expect(MeshEdit.signedVolume(u), closeTo(2000, 1e-3));
    });

    test('overlapping cubes union is closed', () {
      final a = MeshPrimitives.cube(10, 10, 10);
      final b = MeshPrimitives.translate(
        MeshPrimitives.cube(10, 10, 10),
        Vector3(5, 0, 0),
      );
      final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);
      expect(MeshEdit.signedVolume(u), closeTo(1500, 1.0));
    });

    test('offset boxes of different height union is closed', () {
      final a = MeshPrimitives.cube(20, 20, 10);
      final b = MeshPrimitives.translate(
        MeshPrimitives.cube(10, 10, 20),
        Vector3(15, 15, 0),
      );
      final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);
      expect(MeshEdit.signedVolume(u), greaterThan(4000));
    });

    test('box minus overlapping box is closed', () {
      final a = MeshPrimitives.cube(20, 20, 20);
      final b = MeshPrimitives.translate(
        MeshPrimitives.cube(10, 10, 10),
        Vector3(5, 5, 5),
      );
      final d = MeshBoolean.run(a, b, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(d), isTrue);
      expect(MeshEdit.signedVolume(d), closeTo(7000, 2.0));
    });

    test('box minus cylinder is closed', () {
      final a = MeshPrimitives.cube(20, 20, 20);
      final b = MeshPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 4,
        height: 22,
      );
      final d = MeshBoolean.run(a, b, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(d), isTrue);
      expect(MeshEdit.signedVolume(d), lessThan(8000));
      expect(MeshEdit.signedVolume(d), greaterThan(5000));
    });

    test('union of already-holed boxes sharing a plane is closed', () {
      final plate = MeshPrimitives.cube(40, 20, 10);
      final hole = MeshPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 3,
        height: 12,
      );
      final holed = MeshBoolean.run(plate, hole, MeshBoolean.subtractOp);
      final other = MeshPrimitives.translate(
        MeshPrimitives.cube(20, 20, 10),
        Vector3(30, 0, 0),
      );
      final u = MeshBoolean.run(holed, other, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);
    });

    test('union a prism flush with a rectangular frame stays closed', () {
      // App log 2026-09-07: isolate csg-union a=52v/104f b=8v/12f hung 45s.
      // Sketch rectangle spanning a frame opening, ends flush with the inner
      // walls, then Union. Cutting by every coplanar wall triangle restabbed
      // the same plane until the isolate timed out.
      final outer = MeshPrimitives.cube(100, 50, 20);
      final hole = MeshPrimitives.translate(
        MeshPrimitives.cube(80, 30, 22),
        Vector3(10, 10, -1),
      );
      final frame = MeshBoolean.run(outer, hole, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(frame), isTrue);

      // Bar spans the hole in X (x=10..90), 10 mm of Y, 50.8 mm up from the
      // top face — flush with the inner X walls, with a 0.1 mm overlap
      // below the rim so the join has area instead of edge-only contact.
      final bar = MeshPrimitives.translate(
        MeshPrimitives.cube(80, 10, 50.9),
        Vector3(10, 20, 19.9),
      );
      expect(bar.vertices.length, 8);
      expect(bar.faces.length, 12);

      final sw = Stopwatch()..start();
      final u = MeshBoolean.run(frame, bar, MeshBoolean.unionOp);
      expect(
        sw.elapsedMilliseconds,
        lessThan(5000),
        reason:
            'flush frame∪bar must not hang CSG (${sw.elapsedMilliseconds}ms) '
            'frame=${frame.vertices.length}v/${frame.faces.length}f',
      );
      var sealed = u;
      if (!MeshSeal.isClosedSolid(sealed)) {
        sealed = MeshSeal.stitchTJunctions(sealed);
      }
      expect(
        MeshSeal.isClosedSolid(sealed),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(sealed)} '
            'out=${sealed.vertices.length}v/${sealed.faces.length}f',
      );
      expect(
        MeshEdit.signedVolume(sealed),
        greaterThan(MeshEdit.signedVolume(frame) + 100),
      );

      // Same bar dropped into the opening (sketch-on-plane extrude through
      // the hole), still flush with the inner X walls.
      final throughBar = MeshPrimitives.translate(
        MeshPrimitives.cube(80, 10, 50.8),
        Vector3(10, 20, 0),
      );
      final sw2 = Stopwatch()..start();
      final u2 = MeshBoolean.run(frame, throughBar, MeshBoolean.unionOp);
      expect(
        sw2.elapsedMilliseconds,
        lessThan(5000),
        reason: 'through-opening ∪ hung ${sw2.elapsedMilliseconds}ms',
      );
      var sealed2 = u2;
      if (!MeshSeal.isClosedSolid(sealed2)) {
        sealed2 = MeshSeal.stitchTJunctions(sealed2);
      }
      expect(MeshSeal.isClosedSolid(sealed2), isTrue);
    });

    test('through-hole top does not fan interior diagonals', () {
      final box = MeshPrimitives.cube(20, 20, 20);
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 5,
        height: 22,
      );
      final cut = MeshBoolean.run(box, cyl, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(cut), isTrue);
      expect(MeshEdit.signedVolume(cut), closeTo(8000 - math.pi * 25 * 20, 80));

      final top = MeshFaceRegions.featureEdges(cut, includeSmoothCreases: false)
          .where((e) {
            final a = cut.vertices[e.$1];
            final b = cut.vertices[e.$2];
            return (a.z - 20).abs() < 0.2 && (b.z - 20).abs() < 0.2;
          })
          .length;
      expect(
        top,
        lessThan(100),
        reason: 'holed cap should outline the square and circle, not fans',
      );
      expect(
        top,
        greaterThan(20),
        reason: 'square + circular hole must remain',
      );
      var outer = 0;
      for (final e in MeshFaceRegions.featureEdges(
        cut,
        includeSmoothCreases: false,
      )) {
        final a = cut.vertices[e.$1];
        final b = cut.vertices[e.$2];
        if ((a.z - 20).abs() > 0.2 || (b.z - 20).abs() > 0.2) continue;
        final onRim =
            (a.x.abs() < 0.2 || (a.x - 20).abs() < 0.2) &&
            (b.x.abs() < 0.2 || (b.x - 20).abs() < 0.2);
        final onSide =
            (a.y.abs() < 0.2 || (a.y - 20).abs() < 0.2) &&
            (b.y.abs() < 0.2 || (b.y - 20).abs() < 0.2);
        if (onRim || onSide) outer++;
      }
      expect(outer, greaterThanOrEqualTo(4), reason: 'box outline of the cap');
    });

    test('circle cut starting on the face does not fan that cap', () {
      // Sketch-on-face extrude: the tool cap is coplanar with the start
      // face, not past it. That is the side that showed tessellation fans.
      final box = MeshPrimitives.cube(40, 40, 20);
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3(20, 20, 0),
        axis: Vector3(0, 0, 1),
        radius: 6,
        height: 20,
        segments: 32,
      );
      final cut = MeshBoolean.run(box, cyl, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(cut), isTrue);

      int capEdges(double z) =>
          MeshFaceRegions.featureEdges(cut, includeSmoothCreases: false).where((
            e,
          ) {
            final a = cut.vertices[e.$1];
            final b = cut.vertices[e.$2];
            return (a.z - z).abs() < 0.25 && (b.z - z).abs() < 0.25;
          }).length;

      final start = capEdges(0);
      final exit = capEdges(20);
      expect(
        start,
        lessThan(80),
        reason: 'start cap (sketch plane) must not show circle-fan radii',
      );
      expect(exit, lessThan(80), reason: 'exit cap must not show fans either');
      expect(start, greaterThan(20));
    });

    test('circle cut on a vertical wall does not fan that face', () {
      // Sketch-on-vertical-face extrude: the tool cap is coplanar with a
      // Y-normal wall. Needle slivers along Z used to be tagged as X-faces
      // and showed as a curtain of vertical lines on the start side only.
      final box = MeshPrimitives.cube(40, 40, 30);
      final tool = MeshPrimitives.cylinder(
        origin: Vector3(20, 0, 15),
        axis: Vector3(0, 1, 0),
        radius: 6,
        height: 40,
      );
      final cut = MeshBoolean.run(box, tool, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(cut), isTrue);

      int wallEdges(double y) {
        final all = MeshFaceRegions.featureEdges(
          cut,
          includeSmoothCreases: true,
        );
        return all.where((e) {
          final a = cut.vertices[e.$1];
          final b = cut.vertices[e.$2];
          return (a.y - y).abs() < 0.25 && (b.y - y).abs() < 0.25;
        }).length;
      }

      final start = wallEdges(0);
      expect(
        start,
        lessThan(110),
        reason:
            'vertical start face must outline the rectangle and hole, '
            'not cap-fan / sliver radii',
      );
      expect(start, greaterThan(8));
    });

    test('circle through a D-wall does not fan the sketch face', () {
      const cx = 50.0;
      const cy = 25.0;
      const r = 12.0;
      const segs = 32;
      final ring = [
        Vector3(10, 10, 0),
        Vector3(50, 10, 0),
        Vector3(50, cy - r, 0),
        for (var i = 1; i < segs; i++)
          Vector3(
            cx + r * math.cos(3 * math.pi / 2 - i / segs * math.pi),
            cy + r * math.sin(3 * math.pi / 2 - i / segs * math.pi),
            0,
          ),
        Vector3(50, cy + r, 0),
        Vector3(50, 40, 0),
        Vector3(10, 40, 0),
      ];
      final body = MeshBuild.sweepPolygon(ring, Vector3(0, 0, 30));
      final tool = MeshPrimitives.cylinder(
        origin: Vector3(30, 10, 15),
        axis: Vector3(0, 1, 0),
        radius: 4,
        height: 30,
      );
      final cut = MeshBoolean.run(body, tool, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(cut), isTrue);

      final start =
          MeshFaceRegions.featureEdges(cut, includeSmoothCreases: true).where((
            e,
          ) {
            final a = cut.vertices[e.$1];
            final b = cut.vertices[e.$2];
            return (a.y - 10).abs() < 0.25 && (b.y - 10).abs() < 0.25;
          }).length;
      expect(
        start,
        lessThan(80),
        reason: 'D-wall sketch face must not show vertical tessellation fans',
      );
      expect(start, greaterThan(20));
    });

    test('sphere sitting on a box face does not fan that face', () {
      final box = MeshPrimitives.cube(40, 30, 20);
      final sph = MeshPrimitives.sphere(center: Vector3(40, 15, 10), radius: 8);
      final u = MeshBoolean.run(box, sph, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);

      bool onWall(Vector3 p) => _onPlaneX(p, 40);
      expect(
        _coplanarIdsOn(u, (p) => (p.x - 40).abs() < 1e-3),
        1,
        reason:
            'Coplanar face group: every triangle on the cut cube face '
            'is one original face, including CSG fans',
      );
      expect(
        _planeFeatureCount(u, onWall),
        lessThan(130),
        reason:
            'box face the sphere sits on must outline the rectangle '
            'and the circular blend, not sphere-fan radii',
      );
      expect(_planeFeatureCount(u, onWall), greaterThan(8));
      expect(
        _radialFanCount(
          mesh: u,
          onPlane: onWall,
          onOuterRim: (p) =>
              p.y.abs() < 0.4 ||
              (p.y - 30).abs() < 0.4 ||
              p.z.abs() < 0.4 ||
              (p.z - 20).abs() < 0.4,
          circleCenter: Vector3(40, 15, 10),
          circleR: 8,
        ),
        lessThan(3),
        reason: 'no radii from the square corners to the blend circle',
      );
    });

    test(
      'app-path tessellated cube union sphere does not fan the cut face',
      () {
        // Same hop the app isolate uses: tessellate B-Rep, pack through
        // PackedMesh (faceIDs now round-trip), then CSG.
        final box = PackedMesh.fromMesh(
          MeshPrimitives.cube(40, 40, 40),
        ).toMesh();
        final sph = PackedMesh.fromMesh(
          MeshPrimitives.sphere(center: Vector3(40, 20, 20), radius: 12),
        ).toMesh();
        final u = MeshBoolean.run(box, sph, MeshBoolean.unionOp);
        expect(MeshSeal.isClosedSolid(u), isTrue);
        expect(u.faceIds, isNotNull);

        bool onWall(Vector3 p) => _onPlaneX(p, 40);
        expect(_coplanarIdsOn(u, (p) => (p.x - 40).abs() < 1e-3), 1);
        expect(_planeFeatureCount(u, onWall), lessThan(130));
        expect(
          _planeFeatureCount(u, onWall, includeSmoothCreases: false),
          lessThan(130),
        );
        expect(
          _radialFanCount(
            mesh: u,
            onPlane: onWall,
            onOuterRim: (p) =>
                p.y.abs() < 0.4 ||
                (p.y - 40).abs() < 0.4 ||
                p.z.abs() < 0.4 ||
                (p.z - 40).abs() < 0.4,
            circleCenter: Vector3(40, 20, 20),
            circleR: 12,
          ),
          lessThan(10),
          reason: 'icosphere blend chords can miss a perfect-radius test',
        );
        expect(
          _rimCovered(
            u,
            onEdge: (p) => onWall(p) && p.y.abs() < 0.4,
            along: (p) => p.z,
          ),
          greaterThan(38),
        );
      },
    );

    test('sphere pocket on a box face does not fan that face', () {
      final box = MeshPrimitives.cube(40, 40, 40);
      final sph = MeshPrimitives.sphere(
        center: Vector3(40, 20, 20),
        radius: 12,
      );
      final d = MeshBoolean.run(box, sph, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(d), isTrue);
      bool onWall(Vector3 p) => _onPlaneX(p, 40);
      expect(_coplanarIdsOn(d, (p) => (p.x - 40).abs() < 1e-3), 1);
      expect(_planeFeatureCount(d, onWall), lessThan(130));
      expect(
        _radialFanCount(
          mesh: d,
          onPlane: onWall,
          onOuterRim: (p) =>
              p.y.abs() < 0.4 ||
              (p.y - 40).abs() < 0.4 ||
              p.z.abs() < 0.4 ||
              (p.z - 40).abs() < 0.4,
          circleCenter: Vector3(40, 20, 20),
          circleR: 12,
        ),
        lessThan(3),
      );
    });

    test('sphere unioned into a D-notch does not fan a planar cap', () {
      const cx = 50.0;
      const cy = 25.0;
      const r = 12.0;
      const segs = 32;
      final ring = [
        Vector3(10, 10, 0),
        Vector3(50, 10, 0),
        Vector3(50, cy - r, 0),
        for (var i = 1; i < segs; i++)
          Vector3(
            cx + r * math.cos(3 * math.pi / 2 - i / segs * math.pi),
            cy + r * math.sin(3 * math.pi / 2 - i / segs * math.pi),
            0,
          ),
        Vector3(50, cy + r, 0),
        Vector3(50, 40, 0),
        Vector3(10, 40, 0),
      ];
      final body = MeshPrimitives.extrude(ring, Vector3(0, 0, 30));
      final sph = MeshPrimitives.sphere(center: Vector3(25, 25, 30), radius: 8);
      final u = MeshBoolean.run(body, sph, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);

      final onCap = MeshFaceRegions.featureEdges(u, includeSmoothCreases: true)
          .where((e) {
            final a = u.vertices[e.$1];
            final b = u.vertices[e.$2];
            return (a.z - 30).abs() < 0.3 && (b.z - 30).abs() < 0.3;
          })
          .length;
      expect(
        onCap,
        lessThan(160),
        reason: 'D-cap the sphere sits on must not show tessellation fans',
      );
      expect(onCap, greaterThan(12));

      final onWall = MeshFaceRegions.featureEdges(u, includeSmoothCreases: true)
          .where((e) {
            final a = u.vertices[e.$1];
            final b = u.vertices[e.$2];
            return (a.y - 10).abs() < 0.3 && (b.y - 10).abs() < 0.3;
          })
          .length;
      expect(
        onWall,
        lessThan(40),
        reason: 'untouched D-wall must stay a clean rectangle',
      );
    });

    test('sphere unioned into a hollow D does not fan the caps', () {
      List<Vector3> dRing(double width, double height, int arcPts) {
        final rad = height / 2;
        final dcx = width - rad;
        return [
          Vector3(0, 0, 0),
          Vector3(dcx, 0, 0),
          for (var i = 1; i < arcPts; i++)
            Vector3(
              dcx + rad * math.cos(-math.pi / 2 + math.pi * i / arcPts),
              rad + rad * math.sin(-math.pi / 2 + math.pi * i / arcPts),
              0,
            ),
          Vector3(dcx, height, 0),
          Vector3(0, height, 0),
        ];
      }

      final outer = dRing(50, 28, 18);
      final inner = [
        for (final p in dRing(40, 18, 18)) Vector3(p.x + 5, p.y + 5, 0),
      ];
      final tube = MeshBuild.sweepPolygonWithHoles(outer, [
        inner,
      ], Vector3(0, 0, 16));
      final sph = MeshPrimitives.sphere(center: Vector3(5, 5, 8), radius: 7);
      final u = MeshBoolean.run(tube, sph, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);

      int capEdges(double z) =>
          MeshFaceRegions.featureEdges(u, includeSmoothCreases: true).where((
            e,
          ) {
            final a = u.vertices[e.$1];
            final b = u.vertices[e.$2];
            return (a.z - z).abs() < 0.3 && (b.z - z).abs() < 0.3;
          }).length;

      expect(
        capEdges(0),
        lessThan(100),
        reason: 'hollow D bottom cap must not fan where the sphere meets it',
      );
      expect(
        capEdges(16),
        lessThan(100),
        reason: 'hollow D top cap must not fan where the sphere meets it',
      );
    });

    test('box union sphere is closed', () {
      final a = MeshPrimitives.cube(40, 30, 20);
      final b = MeshPrimitives.sphere(center: Vector3(40, 15, 10), radius: 8);
      final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);
      expect(MeshEdit.signedVolume(u), closeTo(25060, 80));
    });

    test('UV-sphere ∪ filleted box stays closed without stitch explosion', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final edges = BrepEdgeSelect.convex().resolve(box);
      expect(edges, isNotEmpty);
      final filleted = IjoOps.tessellateClosed(
        IjoOps.filletSolid(box, edges, radius: 2, inverse: false),
        tessellation: MeshTessellation.draft,
        op: 'Fillet',
      );
      expect(MeshSeal.isClosedSolid(filleted), isTrue);
      final sph = MeshBuild.uvSphere(
        center: Vector3(20, 10, 10),
        radius: 8,
        latSegments: 12,
        lonSegments: 16,
      );
      expect(MeshSeal.isClosedSolid(sph), isTrue);
      final u = MeshBoolean.run(sph, filleted, MeshBoolean.unionOp);
      expect(
        MeshSeal.isClosedSolid(u),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(u)} '
            'sph=${sph.vertices.length}v/${sph.faces.length}f '
            'box=${filleted.vertices.length}v/${filleted.faces.length}f '
            'out=${u.vertices.length}v/${u.faces.length}f',
      );
      expect(
        u.faces.length,
        lessThan(20000),
        reason:
            'T-walk must not explode '
            '(${u.vertices.length}v/${u.faces.length}f)',
      );
      expect(MeshEdit.signedVolume(u), greaterThan(8000));
    });

    test('Decompose keeps cube∪sphere as one solid', () {
      // App union uses the sphere as the base (A) and the box as the tool.
      // Knife-edge connectedComponents shatters coplanar 4-use leftovers
      // into hundreds of scraps; uniting every halfedge keeps one body.
      void expectOne(MeshData a, MeshData b, String label) {
        final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
        expect(MeshSeal.isClosedSolid(u), isTrue, reason: label);
        expect(
          MeshTopology.decompose(u),
          hasLength(1),
          reason: '$label must stay one body',
        );
        expect(
          _coplanarIdsOn(u, (p) => (p.x - 40).abs() < 1e-3),
          1,
          reason: '$label cut face is one coplanarID',
        );
        expect(
          _planeFeatureCount(u, (p) => _onPlaneX(p, 40)),
          lessThan(130),
          reason: '$label must not draw cube-face fans',
        );
      }

      final box = MeshPrimitives.cube(40, 40, 40);
      final sph = MeshPrimitives.sphere(
        center: Vector3(40, 20, 20),
        radius: 12,
      );
      expectOne(sph, box, 'sphere ∪ box (app operand order)');
      expectOne(box, sph, 'box ∪ sphere');

      final packedBox = PackedMesh.fromMesh(
        MeshPrimitives.cube(40, 40, 40),
      ).toMesh();
      final packedSph = PackedMesh.fromMesh(
        MeshPrimitives.sphere(center: Vector3(40, 20, 20), radius: 12),
      ).toMesh();
      expectOne(packedSph, packedBox, 'app-path sphere ∪ box');
      expectOne(packedBox, packedSph, 'app-path box ∪ sphere');

      // Outer cube rims must stay complete (T-split used to eat them).
      final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
      bool onX40(Vector3 p) => _onPlaneX(p, 40);
      expect(
        _rimCovered(
          u,
          onEdge: (p) => onX40(p) && p.y.abs() < 0.4,
          along: (p) => p.z,
        ),
        greaterThan(38),
        reason: 'front-bottom cube edge must not disappear',
      );
      expect(
        _rimCovered(
          u,
          onEdge: (p) => onX40(p) && (p.y - 40).abs() < 0.4,
          along: (p) => p.z,
        ),
        greaterThan(38),
      );
      expect(
        _rimCovered(
          u,
          onEdge: (p) => onX40(p) && p.z.abs() < 0.4,
          along: (p) => p.y,
        ),
        greaterThan(38),
      );
      expect(
        _rimCovered(
          u,
          onEdge: (p) => onX40(p) && (p.z - 40).abs() < 0.4,
          along: (p) => p.y,
        ),
        greaterThan(38),
        reason: 'front-top cube edge must not disappear',
      );
      expect(
        _radialFanCount(
          mesh: u,
          onPlane: onX40,
          onOuterRim: (p) =>
              p.y.abs() < 0.4 ||
              (p.y - 40).abs() < 0.4 ||
              p.z.abs() < 0.4 ||
              (p.z - 40).abs() < 0.4,
          circleCenter: Vector3(40, 20, 20),
          circleR: 12,
        ),
        lessThan(5),
        reason: 'putting crease slivers on the host face must not revive fans',
      );

      void expectSolidQuad(
        String name,
        bool Function(Vector3) onPlane,
        Vector3 outward,
      ) {
        var area = 0.0;
        var flipped = 0;
        for (final face in u.faces) {
          if (face.length < 3) continue;
          if (!face.every((i) => onPlane(u.vertices[i]))) continue;
          final cr = (u.vertices[face[1]] - u.vertices[face[0]]).cross(
            u.vertices[face[2]] - u.vertices[face[0]],
          );
          if (cr.length2 < 1e-8) continue;
          area += 0.5 * cr.length;
          if (cr.normalized().dot(outward) < 0) flipped++;
        }
        expect(area, closeTo(1600, 2), reason: '$name must stay a full 40×40');
        expect(flipped, 0, reason: '$name winding must be outward');

        bool onCubeSide(Vector3 a, Vector3 b) {
          bool both(bool Function(Vector3) e) => e(a) && e(b);
          return both((p) => p.x.abs() < 0.4) ||
              both((p) => (p.x - 40).abs() < 0.4) ||
              both((p) => p.y.abs() < 0.4) ||
              both((p) => (p.y - 40).abs() < 0.4) ||
              both((p) => p.z.abs() < 0.4) ||
              both((p) => (p.z - 40).abs() < 0.4);
        }

        final interior =
            MeshFaceRegions.featureEdges(u, includeSmoothCreases: false).where((
              e,
            ) {
              final a = u.vertices[e.$1];
              final b = u.vertices[e.$2];
              if (!onPlane(a) || !onPlane(b)) return false;
              return !onCubeSide(a, b);
            }).length;
        expect(
          interior,
          0,
          reason: '$name must not draw a diagonal or sliver junk',
        );
      }

      expectSolidQuad('x=0', (p) => p.x.abs() < 0.4, Vector3(-1, 0, 0));
      expectSolidQuad('y=0', (p) => p.y.abs() < 0.4, Vector3(0, -1, 0));
      expectSolidQuad('y=40', (p) => (p.y - 40).abs() < 0.4, Vector3(0, 1, 0));
      expectSolidQuad('z=0', (p) => p.z.abs() < 0.4, Vector3(0, 0, -1));
      expectSolidQuad('z=40', (p) => (p.z - 40).abs() < 0.4, Vector3(0, 0, 1));
    });

    test('default 20 mm cube ∪ r=10 sphere keeps uncut sides solid', () {
      // App primitives: 20 mm box (vol 8000) and r=10 icosphere (642 v).
      // Sphere center on a cube face — the join from debug/ijo-last.json.
      // unifyCoplanarFaces used to open this solid (89 holes) and sealSolid
      // capped a side with one triangle (missing half-face in the viewport).
      final sph = MeshPrimitives.sphere(center: Vector3(10, 0, 10), radius: 10);
      final box = MeshPrimitives.cube(20, 20, 20);
      final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);
      expect(
        MeshEdit.signedVolume(u),
        greaterThan(9000),
        reason: 'cube plus spherical cap, not a collapsed leftover',
      );
      expect(MeshTopology.decompose(u), hasLength(1));

      void expectSide(
        String name,
        bool Function(Vector3) onPlane,
        Vector3 outward,
      ) {
        var area = 0.0;
        var flipped = 0;
        for (final face in u.faces) {
          if (face.length < 3) continue;
          if (!face.every((i) => onPlane(u.vertices[i]))) continue;
          final cr = (u.vertices[face[1]] - u.vertices[face[0]]).cross(
            u.vertices[face[2]] - u.vertices[face[0]],
          );
          if (cr.length2 < 1e-8) continue;
          area += 0.5 * cr.length;
          if (cr.normalized().dot(outward) < 0) flipped++;
        }
        expect(
          area,
          greaterThan(350),
          reason: '$name must not lose a cube-side triangle',
        );
        expect(flipped, 0, reason: '$name winding must be outward');
      }

      expectSide('x=0', (p) => p.x.abs() < 0.4, Vector3(-1, 0, 0));
      expectSide('x=20', (p) => (p.x - 20).abs() < 0.4, Vector3(1, 0, 0));
      expectSide('y=20', (p) => (p.y - 20).abs() < 0.4, Vector3(0, 1, 0));
      expectSide('z=0', (p) => p.z.abs() < 0.4, Vector3(0, 0, -1));
      expectSide('z=20', (p) => (p.z - 20).abs() < 0.4, Vector3(0, 0, 1));

      final topInterior =
          MeshFaceRegions.featureEdges(u, includeSmoothCreases: false).where((
            e,
          ) {
            final a = u.vertices[e.$1];
            final b = u.vertices[e.$2];
            if ((a.y - 20).abs() >= 0.4 || (b.y - 20).abs() >= 0.4)
              return false;
            bool alongRim(Vector3 p, Vector3 q) =>
                (p.x.abs() < 0.4 && q.x.abs() < 0.4) ||
                ((p.x - 20).abs() < 0.4 && (q.x - 20).abs() < 0.4) ||
                (p.z.abs() < 0.4 && q.z.abs() < 0.4) ||
                ((p.z - 20).abs() < 0.4 && (q.z - 20).abs() < 0.4);
            return !alongRim(a, b);
          }).length;
      expect(
        topInterior,
        0,
        reason: 'uncut top must not draw a tessellation diagonal',
      );
    });

    test(
      'sphere-on-face union keeps one faceID; extrude has no radial towers',
      () {
        // App join: r=10 sphere centered on the cube's x=0 face.
        final sph = MeshPrimitives.sphere(
          center: Vector3(0, 10, 10),
          radius: 10,
        );
        final box = MeshPrimitives.cube(20, 20, 20);
        final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
        expect(MeshSeal.isClosedSolid(u), isTrue);
        expect(u.faceIds, isNotNull, reason: 'face ids must survive CSG');

        int? seed;
        for (var i = 0; i < u.faces.length; i++) {
          final face = u.faces[i];
          if (face.length < 3) continue;
          if (!face.every((vi) => u.vertices[vi].x.abs() < 0.4)) continue;
          final cr = (u.vertices[face[1]] - u.vertices[face[0]]).cross(
            u.vertices[face[2]] - u.vertices[face[0]],
          );
          if (cr.length2 < 1e-8) continue;
          if (cr.normalized().dot(Vector3(-1, 0, 0)) > 0.5) {
            seed = i;
            break;
          }
        }
        expect(seed, isNotNull, reason: 'cut cube face x=0');
        expect(_coplanarIdsOn(u, (p) => p.x.abs() < 1e-3), 1);

        bool onWall(Vector3 p) => p.x.abs() < 0.3;
        expect(
          _radialFanCount(
            mesh: u,
            onPlane: onWall,
            onOuterRim: (p) =>
                p.y.abs() < 0.4 ||
                (p.y - 20).abs() < 0.4 ||
                p.z.abs() < 0.4 ||
                (p.z - 20).abs() < 0.4,
            circleCenter: Vector3(0, 10, 10),
            circleR: 10,
          ),
          lessThan(5),
          reason: 'Steiner fans on the cut face must not be CAD edges',
        );

        final region = MeshFaceRegions.selectionRegion(u, seed!);
        expect(region.length, greaterThan(2));
        final tool = MeshBuild.extrudeRegion(
          mesh: u,
          region: region,
          direction: Vector3(2.42, 0, 0),
        );
        expect(MeshSeal.isClosedSolid(tool), isTrue);
        expect(
          _radialFanCount(
            mesh: tool,
            onPlane: onWall,
            onOuterRim: (p) =>
                p.y.abs() < 0.4 ||
                (p.y - 20).abs() < 0.4 ||
                p.z.abs() < 0.4 ||
                (p.z - 20).abs() < 0.4,
            circleCenter: Vector3(0, 10, 10),
            circleR: 10,
          ),
          lessThan(5),
          reason: 'extrude walls must be the square and circle, not radii',
        );

        final cut = MeshBoolean.run(u, tool, MeshBoolean.subtractOp);
        expect(MeshSeal.isClosedSolid(cut), isTrue);
        expect(MeshEdit.signedVolume(cut), lessThan(MeshEdit.signedVolume(u)));
      },
    );

    test(
      'inward extrude of cube∪sphere cut face keeps a cylindrical pocket',
      () {
        // App test: join, then push the square-minus-circle face back into
        // the cube. Triangle-soup extrusion of that region subtracted to an
        // open shell (missing cylinder in X-ray).
        final sph = MeshPrimitives.sphere(
          center: Vector3(10, -6, 10),
          radius: 10,
        );
        final box = MeshPrimitives.cube(20, 20, 20);
        final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
        expect(MeshSeal.isClosedSolid(u), isTrue);

        int? seed;
        for (var i = 0; i < u.faces.length; i++) {
          final face = u.faces[i];
          if (face.length < 3) continue;
          if (!face.every((vi) => u.vertices[vi].y.abs() < 0.5)) continue;
          final cr = (u.vertices[face[1]] - u.vertices[face[0]]).cross(
            u.vertices[face[2]] - u.vertices[face[0]],
          );
          if (cr.length2 < 1e-8) continue;
          if (cr.normalized().dot(Vector3(0, -1, 0)) > 0.5) {
            seed = i;
            break;
          }
        }
        expect(seed, isNotNull);
        final region = MeshFaceRegions.selectionRegion(u, seed!);
        expect(region.length, greaterThan(20));
        const depth = 8.0;
        final tool = MeshBuild.extrudeRegion(
          mesh: u,
          region: region,
          direction: Vector3(0, depth, 0),
        );
        expect(MeshSeal.isClosedSolid(tool), isTrue);
        expect(
          tool.faces.length,
          lessThan(1500),
          reason: 'profile sweep of the annulus, not a second copy of the body',
        );
        expect(
          MeshEdit.signedVolume(tool).abs(),
          closeTo((400 - math.pi * 8 * 8) * depth, 40),
          reason:
              'prism is the square minus the blend circle, not a solid brick',
        );

        final cut = MeshBoolean.run(u, tool, MeshBoolean.subtractOp);
        expect(
          MeshSeal.isClosedSolid(cut),
          isTrue,
          reason: 'pocket walls must stay on the solid',
        );
        expect(
          MeshEdit.signedVolume(cut),
          closeTo(
            MeshEdit.signedVolume(u) - MeshEdit.signedVolume(tool).abs(),
            50,
          ),
        );
        var inner = 0;
        for (final e in MeshFaceRegions.featureEdges(
          cut,
          includeSmoothCreases: true,
        )) {
          final mid = (cut.vertices[e.$1] + cut.vertices[e.$2]) * 0.5;
          if (mid.y > 0.5 && mid.y < depth - 0.5) inner++;
        }
        expect(
          inner,
          greaterThan(20),
          reason: 'X-ray must see the cylindrical pocket wall',
        );
      },
    );

    test(
      'inward extrude of almost-inscribed sphere∪cube is a closed pocket',
      () {
        // App log 2026-09-06: sphere poked to x≈−9.1 on a 20 mm cube, then
        // face-extrude of the −X annulus. Soup T-split of the whole body
        // (566v/291f) made fins; X-ray hid the broken barrel.
        final sph = MeshPrimitives.sphere(
          center: Vector3(0.9, 10, 10),
          radius: 10,
        );
        final box = MeshPrimitives.cube(20, 20, 20);
        final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
        expect(MeshSeal.isClosedSolid(u), isTrue);

        final seed = _minusXPlanarSeed(u);
        expect(seed, isNotNull);
        final region = MeshFaceRegions.selectionRegion(u, seed!);
        expect(region.length, greaterThan(10));
        const depth = 7.1;
        final tool = _faceExtrudePrism(u, region, Vector3(depth, 0, 0));
        expect(
          MeshSeal.isClosedSolid(tool),
          isTrue,
          reason: 'weld of a closed T-split prism must not open it',
        );
        expect(
          tool.faces.length,
          lessThan(1500),
          reason: 'profile sweep of the annulus, not a second copy of the body',
        );
        expect(
          tool.vertices.length,
          lessThan(u.vertices.length),
          reason: 'prism verts are the profile, not the whole body',
        );

        final cut = MeshBoolean.run(u, tool, MeshBoolean.subtractOp);
        expect(
          MeshSeal.isClosedSolid(cut),
          isTrue,
          reason: 'pocket walls must stay on the solid',
        );
        expect(
          MeshEdit.signedVolume(cut),
          lessThan(MeshEdit.signedVolume(u) - 1),
        );

        final pocket = MeshBuild.pocketRegion(
          mesh: u,
          region: region,
          direction: Vector3(depth, 0, 0),
        );
        expect(
          MeshSeal.isClosedSolid(pocket),
          isTrue,
          reason: 'inward push must not CSG a knife-thin annulus',
        );
        expect(
          MeshEdit.signedVolume(pocket),
          lessThan(MeshEdit.signedVolume(u) - 1),
        );
        expect(
          _radialFanCount(
            mesh: pocket,
            onPlane: (p) => (p.x - depth).abs() < 0.5,
            onOuterRim: (p) =>
                p.y.abs() < 0.4 ||
                (p.y - 20).abs() < 0.4 ||
                p.z.abs() < 0.4 ||
                (p.z - 20).abs() < 0.4,
            circleCenter: Vector3(depth, 10, 10),
            circleR: 9.5,
          ),
          lessThan(3),
          reason: 'pocket floor must not keep CSG Steiner radii',
        );
      },
    );

    test('inward pocket of user-offset sphere∪cube stays closed', () {
      // App log 2026-09-06 12:00: AABB x≈−9.59, t-split-sweep 488v/976f,
      // isolate CSG subtract then threw "broken solid".
      final sph = MeshPrimitives.sphere(
        center: Vector3(0.41, 10, 10),
        radius: 10,
      );
      final box = MeshPrimitives.cube(20, 20, 20);
      final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);

      final seed = _minusXPlanarSeed(u);
      expect(seed, isNotNull);
      final region = MeshFaceRegions.selectionRegion(u, seed!);
      expect(region.length, greaterThan(10));
      final raw = MeshBuild.extrudeRegion(
        mesh: u,
        region: region,
        direction: Vector3(7.1, 0, 0),
      );
      expect(MeshSeal.isClosedSolid(raw), isTrue);
      final prism = _faceExtrudePrism(u, region, Vector3(7.1, 0, 0));
      expect(
        MeshSeal.isClosedSolid(prism),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(prism)} '
            'raw=${raw.vertices.length}v/${raw.faces.length}f',
      );
      expect(
        prism.faces.length,
        raw.faces.length,
        reason:
            'do not weld a closed T-split annulus; that drops faces '
            'and opens the shell',
      );
      final pocket = MeshBuild.pocketRegion(
        mesh: u,
        region: region,
        direction: Vector3(7.1, 0, 0),
      );
      expect(MeshSeal.isClosedSolid(pocket), isTrue);
      expect(
        MeshEdit.signedVolume(pocket),
        lessThan(MeshEdit.signedVolume(u) - 1),
      );
      var leftoverRim = 0;
      for (final e in MeshFaceRegions.featureEdges(pocket)) {
        final a = pocket.vertices[e.$1];
        final b = pocket.vertices[e.$2];
        if (a.x.abs() > 0.4 || b.x.abs() > 0.4) continue;
        if ((a - b).length > 5) leftoverRim++;
      }
      expect(
        leftoverRim,
        0,
        reason:
            'outer square must recede; X-ray must not keep 20 mm ghost rims',
      );
      expect(
        _radialFanCount(
          mesh: pocket,
          onPlane: (p) => (p.x - 7.1).abs() < 0.5,
          onOuterRim: (p) =>
              p.y.abs() < 0.4 ||
              (p.y - 20).abs() < 0.4 ||
              p.z.abs() < 0.4 ||
              (p.z - 20).abs() < 0.4,
          circleCenter: Vector3(7.1, 10, 10),
          circleR: 9.5,
        ),
        lessThan(3),
        reason: 'pocket floor must not keep CSG Steiner radii',
      );
    });

    test('subtract a center pocket does not X-ray-ring a corner fillet', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final verts = BrepEdgeSelect.linear(
        direction: Vector3(0, 0, 1),
      ).resolve(box);
      final filleted = IjoOps.tessellateKeep(
        IjoOps.filletSolid(box, verts, radius: 2, inverse: false),
        op: 'Fillet',
      );
      final tool = MeshBuild.sweepPolygon([
        Vector3(7, 7, 12),
        Vector3(13, 7, 12),
        Vector3(13, 13, 12),
        Vector3(7, 13, 12),
      ], Vector3(0, 0, 10));
      final cut = MeshBoolean.run(filleted, tool, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(cut), isTrue);
      expect(
        MeshEdit.signedVolume(cut),
        lessThan(MeshEdit.signedVolume(filleted) - 1),
      );

      const zCut = 12.0;
      var above = 0;
      var below = 0;
      for (final f in cut.faces) {
        if (f.length < 3) continue;
        final pts = [for (final i in f) cut.vertices[i]];
        if (!pts.every((p) => p.x < 4 && p.y < 4)) continue;
        final z = pts.map((p) => p.z).reduce((a, b) => a + b) / pts.length;
        if (z > zCut + 0.8) above++;
        if (z < zCut - 0.8) below++;
      }
      expect(
        above,
        greaterThan(0),
        reason: 'corner fillet stays above the pocket',
      );
      expect(
        below,
        greaterThan(0),
        reason: 'corner fillet stays below the pocket',
      );

      var ring = 0;
      for (final e in MeshFaceRegions.featureEdges(
        cut,
        includeSmoothCreases: true,
      )) {
        final a = cut.vertices[e.$1];
        final b = cut.vertices[e.$2];
        if ((a.z - zCut).abs() > 0.3 || (b.z - zCut).abs() > 0.3) continue;
        final mid = (a + b) * 0.5;
        if (mid.x < 4 && mid.y < 4) ring++;
      }
      expect(
        ring,
        0,
        reason:
            'pocket floor plane must not appear as an X-ray cut on a fillet '
            'the subtract does not touch',
      );
    });

    test('subtract a near-miss pocket does not X-ray-ring a corner fillet', () {
      final box = BrepPrimitives.box(20, 20, 20);
      final verts = BrepEdgeSelect.linear(
        direction: Vector3(0, 0, 1),
      ).resolve(box);
      final filleted = IjoOps.tessellateKeep(
        IjoOps.filletSolid(box, verts, radius: 2, inverse: false),
        op: 'Fillet',
      );
      final tool = MeshBuild.sweepPolygon([
        Vector3(2.5, 2.5, 12),
        Vector3(17.5, 2.5, 12),
        Vector3(17.5, 17.5, 12),
        Vector3(2.5, 17.5, 12),
      ], Vector3(0, 0, 10));
      final cut = MeshBoolean.run(filleted, tool, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(cut), isTrue);

      const zCut = 12.0;
      var ring = 0;
      for (final e in MeshFaceRegions.featureEdges(
        cut,
        includeSmoothCreases: true,
      )) {
        final a = cut.vertices[e.$1];
        final b = cut.vertices[e.$2];
        if ((a.z - zCut).abs() > 0.3 || (b.z - zCut).abs() > 0.3) continue;
        final mid = (a + b) * 0.5;
        if (mid.x < 4 && mid.y < 4) ring++;
      }
      expect(
        ring,
        0,
        reason:
            'a pocket that misses the fillet must not leave an X-ray ring '
            'on that blend at the subtract end plane',
      );
    });

    test('subtract rectangular frame from a coplanar box face stays closed', () {
      final box = MeshPrimitives.cube(40, 20, 20);
      final outer = [
        Vector3(6, 0, 4),
        Vector3(34, 0, 4),
        Vector3(34, 0, 16),
        Vector3(6, 0, 16),
      ];
      final inner = [
        Vector3(12, 0, 7),
        Vector3(28, 0, 7),
        Vector3(28, 0, 13),
        Vector3(12, 0, 13),
      ];
      final tool = MeshBuild.sweepPolygonWithHoles(outer, [
        inner,
      ], Vector3(0, 9.86, 0));
      expect(MeshSeal.isClosedSolid(tool), isTrue);
      expect(tool.vertices.length, 16);
      final cut = MeshBoolean.run(box, tool, MeshBoolean.subtractOp);
      expect(
        MeshSeal.isClosedSolid(cut),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(cut)} v=${cut.vertices.length} f=${cut.faces.length}',
      );
      expect(
        MeshEdit.signedVolume(cut),
        lessThan(MeshEdit.signedVolume(box) - 1),
      );
    });

    test('subtract rectangular frame into a U-arm stays closed', () {
      // App log 2026-09-07: Extrude 3 370v/737f minus a 16v/32f ring prism
      // on the end face → splitT 476v/34521f, then "broken solid".
      // Repro: box minus through-cylinder (~353v), rectangular frame whose
      // cap sits on that already-holed face. Starting 0.4 mm outside the
      // face (sketch-subtract pad) keeps CSG closed.
      final box = MeshPrimitives.cube(40, 20, 20);
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3(20, 22, 10),
        axis: Vector3(0, -1, 0),
        radius: 8,
        height: 24,
        segments: 48,
      );
      final u = MeshBoolean.run(box, cyl, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);

      const pad = 0.4;
      final outer = [
        Vector3(6, -pad, 4),
        Vector3(34, -pad, 4),
        Vector3(34, -pad, 16),
        Vector3(6, -pad, 16),
      ];
      // Leave clearance from the cylinder extrema at x=12 and x=28.
      // Exact tangency there makes zero-thickness internal knife edges.
      final inner = [
        Vector3(12.1, -pad, 7),
        Vector3(27.9, -pad, 7),
        Vector3(27.9, -pad, 13),
        Vector3(12.1, -pad, 13),
      ];
      final tool = MeshBuild.sweepPolygonWithHoles(outer, [
        inner,
      ], Vector3(0, 9.86 + pad, 0));
      expect(MeshSeal.isClosedSolid(tool), isTrue);
      final cut = MeshBoolean.run(u, tool, MeshBoolean.subtractOp);
      expect(
        MeshSeal.isClosedSolid(cut),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(cut)} '
            'u=${u.vertices.length}v/${u.faces.length}f '
            'cut=${cut.vertices.length}v/${cut.faces.length}f',
      );
      expect(
        MeshEdit.signedVolume(cut),
        lessThan(MeshEdit.signedVolume(u) - 1),
      );
    });

    test(
      'subtract a 64-gon circle through a filleted U back wall stays closed',
      () {
        // App log 2026-09-07: Extrude 1 (filleted U, 176v/348f) minus a
        // 64-gon sketch circle (128v/252f) through the inner back wall.
        // splitT ~6900 walks / ~7100 tris, then "broken solid".
        List<Vector3> roundedURing({
          required double w,
          required double d,
          required double t,
          required double rOut,
          required double rIn,
          required int arc,
        }) {
          List<Vector3> q(
            double cx,
            double cy,
            double r,
            double a0,
            double a1,
          ) {
            return [
              for (var i = 1; i <= arc; i++)
                Vector3(
                  cx + r * math.cos(a0 + (a1 - a0) * i / arc),
                  cy + r * math.sin(a0 + (a1 - a0) * i / arc),
                  0,
                ),
            ];
          }

          return [
            Vector3(rOut, 0, 0),
            Vector3(w - rOut, 0, 0),
            ...q(w - rOut, rOut, rOut, -math.pi / 2, 0),
            Vector3(w, d, 0),
            Vector3(w - t, d, 0),
            Vector3(w - t, t + rIn, 0),
            ...q(w - t - rIn, t + rIn, rIn, 0, -math.pi / 2),
            Vector3(t + rIn, t, 0),
            ...q(t + rIn, t + rIn, rIn, -math.pi / 2, -math.pi),
            Vector3(t, d, 0),
            Vector3(0, d, 0),
            Vector3(0, rOut, 0),
            ...q(rOut, rOut, rOut, math.pi, math.pi * 1.5),
          ];
        }

        final uSolid = MeshBuild.sweepPolygon(
          roundedURing(w: 80, d: 50, t: 10, rOut: 12, rIn: 8, arc: 32),
          Vector3(0, 0, 45),
        );
        final body = uSolid;
        expect(MeshSeal.isClosedSolid(body), isTrue);

        const segs = 64;
        const r = 8.0;
        const pad = 0.4;
        final yInner = 10.0;
        // Sketch-on-face rings are often a fraction of a millimetre off the
        // stored plane normal, which used to skip the cylinder fit, reseal an
        // open 64-gon via fromMesh, then CSG came back broken.
        final ring = [
          for (var i = 0; i < segs; i++)
            Vector3(
              40 + r * math.cos(i * math.pi * 2 / segs),
              yInner + pad + (i.isEven ? 0.15 : 0),
              22.5 + r * math.sin(i * math.pi * 2 / segs),
            ),
        ];
        final toolSolid = MeshBuild.sweepPolygon(
          ring,
          Vector3(0, -(10 + 2 * pad), 0),
        );
        final tool = toolSolid;
        expect(
          MeshSeal.isClosedSolid(tool),
          isTrue,
          reason:
              '64-gon prism must be closed before CSG '
              'open=${MeshSeal.openBoundaryEdgeCount(tool)} '
              'v=${tool.vertices.length} f=${tool.faces.length}',
        );

        final cut = MeshBoolean.run(body, tool, MeshBoolean.subtractOp);
        var sealed = cut;
        if (!MeshSeal.isClosedSolid(sealed)) {
          sealed = MeshSeal.stitchTJunctions(sealed);
        }
        expect(
          MeshSeal.isClosedSolid(sealed),
          isTrue,
          reason:
              'open=${MeshSeal.openBoundaryEdgeCount(sealed)} '
              'body=${body.vertices.length}v/${body.faces.length}f '
              'cut=${sealed.vertices.length}v/${sealed.faces.length}f',
        );
        expect(
          MeshEdit.signedVolume(sealed).abs(),
          lessThan(MeshEdit.signedVolume(body).abs() - 1),
        );
      },
    );

    test('subtract a cylinder through an existing hole stays closed', () {
      // App log 2026-09-07: sphere∪cube, pocket, fillet, then a 32-seg
      // circle through-cut (525v/1046f − 64v/124f) succeeded. A second
      // circle crossing that hole (765v/1530f − 64v/124f) hit splitT
      // 5688 walks / 8987 faces and "broken solid".
      final box = MeshPrimitives.cube(40, 20, 20);
      const pad = 0.4;
      final through = MeshPrimitives.cylinder(
        origin: Vector3(20, -pad, 10),
        axis: Vector3(0, 1, 0),
        radius: 5,
        height: 20 + 2 * pad,
        segments: 32,
      );
      final holed = MeshBoolean.run(box, through, MeshBoolean.subtractOp);
      expect(MeshSeal.isClosedSolid(holed), isTrue);

      final cross = MeshPrimitives.cylinder(
        origin: Vector3(-pad, 10, 10),
        axis: Vector3(1, 0, 0),
        radius: 4,
        height: 40 + 2 * pad,
        segments: 32,
      );
      expect(MeshSeal.isClosedSolid(cross), isTrue);
      final cut = MeshBoolean.run(holed, cross, MeshBoolean.subtractOp);
      var sealed = cut;
      if (!MeshSeal.isClosedSolid(sealed)) {
        sealed = MeshSeal.stitchTJunctions(sealed);
      }
      expect(
        MeshSeal.isClosedSolid(sealed),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(sealed)} '
            'holed=${holed.vertices.length}v/${holed.faces.length}f '
            'cross=${cross.vertices.length}v/${cross.faces.length}f '
            'cut=${sealed.vertices.length}v/${sealed.faces.length}f '
            'vol=${MeshEdit.signedVolume(sealed).toStringAsFixed(1)}',
      );
      expect(
        MeshEdit.signedVolume(sealed).abs(),
        lessThan(MeshEdit.signedVolume(holed).abs() - 1),
      );
    });

    test('subtract a circle through both arms of a C stays closed', () {
      // App log 2026-09-07: Extrude 1 C 264v/524f minus a 32-gon sketch
      // cylinder 64v/124f through the bore. Full splitT walked every long
      // interior wall (1482 walks, 575v/1756f) and the isolate threw
      // "broken solid".
      const nArc = 65;
      const gap = 40 * math.pi / 180;
      final start = gap;
      final end = 2 * math.pi - gap;
      final loop = <Vector3>[
        for (var i = 0; i <= nArc; i++)
          Vector3(
            20 * math.cos(start + (end - start) * i / nArc),
            20 * math.sin(start + (end - start) * i / nArc),
            0,
          ),
        for (var i = nArc; i >= 0; i--)
          Vector3(
            12 * math.cos(start + (end - start) * i / nArc),
            12 * math.sin(start + (end - start) * i / nArc),
            0,
          ),
      ];
      final body = MeshBuild.sweepPolygon(loop, Vector3(0, 0, 19.17));
      expect(body.vertices.length, 264);
      expect(body.faces.length, 524);
      expect(MeshSeal.isClosedSolid(body), isTrue);

      const segs = 32;
      const r = 4.0;
      final ring = [
        for (var i = 0; i < segs; i++)
          Vector3(
            r * math.cos(i * math.pi * 2 / segs),
            -40,
            9.585 + r * math.sin(i * math.pi * 2 / segs),
          ),
      ];
      final tool = MeshBuild.sweepPolygon(ring, Vector3(0, 80, 0));
      expect(tool.vertices.length, 64);
      expect(tool.faces.length, 124);

      final cut = MeshBoolean.run(body, tool, MeshBoolean.subtractOp);
      expect(
        MeshSeal.isClosedSolid(cut),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(cut)} '
            'cut=${cut.vertices.length}v/${cut.faces.length}f '
            'vol=${MeshEdit.signedVolume(cut).toStringAsFixed(1)}',
      );
      expect(
        MeshEdit.signedVolume(cut).abs(),
        lessThan(MeshEdit.signedVolume(body).abs() - 1),
      );
    });

    test('subtract a 31-gon through a dense C stays closed', () {
      // Instance of the T-junction family: infinite-plane clips on a dense
      // curved wall leave Steiner verts; two stitch passes were not enough.
      // Any through-cut of a fine tessellated cylinder/C is the same bug,
      // independent of "31". App log 2026-09-08: 737v/1474f − 62v/120f.
      const nArc = 183;
      const gap = 40 * math.pi / 180;
      final start = gap;
      final end = 2 * math.pi - gap;
      final loop = <Vector3>[
        for (var i = 0; i <= nArc; i++)
          Vector3(
            20 * math.cos(start + (end - start) * i / nArc),
            20 * math.sin(start + (end - start) * i / nArc),
            0,
          ),
        for (var i = nArc; i >= 0; i--)
          Vector3(
            12 * math.cos(start + (end - start) * i / nArc),
            12 * math.sin(start + (end - start) * i / nArc),
            0,
          ),
      ];
      final body = MeshBuild.sweepPolygon(loop, Vector3(0, 0, 45));
      expect(body.vertices.length, 736);
      expect(MeshSeal.isClosedSolid(body), isTrue);

      const segs = 31;
      const r = 8.0;
      final ring = [
        for (var i = 0; i < segs; i++)
          Vector3(
            r * math.cos(i * math.pi * 2 / segs),
            -40,
            22.5 + r * math.sin(i * math.pi * 2 / segs),
          ),
      ];
      final tool = MeshBuild.sweepPolygon(ring, Vector3(0, 80, 0));
      expect(tool.vertices.length, 62);
      expect(tool.faces.length, 120);

      final cut = MeshBoolean.run(body, tool, MeshBoolean.subtractOp);
      expect(
        MeshSeal.isClosedSolid(cut),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(cut)} '
            'cut=${cut.vertices.length}v/${cut.faces.length}f '
            'vol=${MeshEdit.signedVolume(cut).toStringAsFixed(1)}',
      );
      expect(
        MeshEdit.signedVolume(cut).abs(),
        lessThan(MeshEdit.signedVolume(body).abs() - 1),
      );
    });

    test('crossing circle through a pocketed sphere∪cube hole stays closed', () {
      // Same log: Sphere 2 ∪ Box, inward pocket of the planar face around
      // the sphere, then a through-circle, then a sideways circle that
      // cuts through that hole.
      final sph = MeshPrimitives.sphere(
        center: Vector3(0.41, 10, 10),
        radius: 10,
      );
      final box = MeshPrimitives.cube(20, 20, 20);
      final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);

      int? seed;
      for (var i = 0; i < u.faces.length; i++) {
        final face = u.faces[i];
        if (face.length < 3) continue;
        if (!face.every((vi) => u.vertices[vi].x.abs() < 0.5)) continue;
        final cr = (u.vertices[face[1]] - u.vertices[face[0]]).cross(
          u.vertices[face[2]] - u.vertices[face[0]],
        );
        if (cr.length2 < 1e-8) continue;
        if (cr.normalized().dot(Vector3(-1, 0, 0)) > 0.5) {
          seed = i;
          break;
        }
      }
      expect(seed, isNotNull);
      final region = MeshFaceRegions.selectionRegion(u, seed!);
      final body = MeshBuild.pocketRegion(
        mesh: u,
        region: region,
        direction: Vector3(7.1, 0, 0),
      );
      expect(MeshSeal.isClosedSolid(body), isTrue);

      const pad = 0.4;
      final through = MeshPrimitives.cylinder(
        origin: Vector3(10, -pad, 10),
        axis: Vector3(0, 1, 0),
        radius: 4,
        height: 20 + 2 * pad,
        segments: 32,
      );
      final holed = MeshBoolean.run(body, through, MeshBoolean.subtractOp);
      expect(
        MeshSeal.isClosedSolid(holed),
        isTrue,
        reason:
            'first through-hole open=${MeshSeal.openBoundaryEdgeCount(holed)} '
            'v=${holed.vertices.length} f=${holed.faces.length}',
      );

      final cross = MeshPrimitives.cylinder(
        origin: Vector3(-pad, 10, 10),
        axis: Vector3(1, 0, 0),
        radius: 3.5,
        height: 20 + 2 * pad + 10,
        segments: 32,
      );
      final cut = MeshBoolean.run(holed, cross, MeshBoolean.subtractOp);
      var sealed = cut;
      if (!MeshSeal.isClosedSolid(sealed)) {
        sealed = MeshSeal.stitchTJunctions(sealed);
      }
      expect(
        MeshSeal.isClosedSolid(sealed),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(sealed)} '
            'body=${body.vertices.length}v/${body.faces.length}f '
            'holed=${holed.vertices.length}v/${holed.faces.length}f '
            'cross=${cross.vertices.length}v/${cross.faces.length}f '
            'cut=${sealed.vertices.length}v/${sealed.faces.length}f',
      );
      expect(
        MeshEdit.signedVolume(sealed).abs(),
        lessThan(MeshEdit.signedVolume(holed).abs() - 1),
      );
    });

    test('crossing circle through a box hole stays closed', () {
      final body = MeshPrimitives.cube(40, 20, 20);
      expect(MeshSeal.isClosedSolid(body), isTrue);

      const pad = 0.4;
      final through = MeshPrimitives.cylinder(
        origin: Vector3(20, -pad, 10),
        axis: Vector3(0, 1, 0),
        radius: 5,
        height: 20 + 2 * pad,
        segments: 32,
      );
      final holed = MeshBoolean.run(body, through, MeshBoolean.subtractOp);
      expect(
        MeshSeal.isClosedSolid(holed),
        isTrue,
        reason:
            'first hole open=${MeshSeal.openBoundaryEdgeCount(holed)} '
            'v=${holed.vertices.length} f=${holed.faces.length}',
      );

      final cross = MeshPrimitives.cylinder(
        origin: Vector3(-pad, 10, 10),
        axis: Vector3(1, 0, 0),
        radius: 4,
        height: 40 + 2 * pad,
        segments: 32,
      );
      final cut = MeshBoolean.run(holed, cross, MeshBoolean.subtractOp);
      var sealed = cut;
      if (!MeshSeal.isClosedSolid(sealed)) {
        sealed = MeshSeal.stitchTJunctions(sealed);
      }
      expect(
        MeshSeal.isClosedSolid(sealed),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(sealed)} '
            'body=${body.vertices.length}v/${body.faces.length}f '
            'holed=${holed.vertices.length}v/${holed.faces.length}f '
            'cut=${sealed.vertices.length}v/${sealed.faces.length}f',
      );
    });

    test('crossing circle through a pocketed sphere∪cube stays closed', () {
      final box = MeshPrimitives.cube(20, 20, 20);
      final sph = MeshPrimitives.sphere(
        center: Vector3(0.41, 10, 10),
        radius: 10,
      );
      final u = MeshBoolean.run(sph, box, MeshBoolean.unionOp);
      expect(MeshSeal.isClosedSolid(u), isTrue);

      int? seed;
      for (var i = 0; i < u.faces.length; i++) {
        final face = u.faces[i];
        if (face.length < 3) continue;
        if (!face.every((vi) => u.vertices[vi].x.abs() < 0.5)) continue;
        final cr = (u.vertices[face[1]] - u.vertices[face[0]]).cross(
          u.vertices[face[2]] - u.vertices[face[0]],
        );
        if (cr.length2 < 1e-8) continue;
        if (cr.normalized().dot(Vector3(-1, 0, 0)) > 0.5) {
          seed = i;
          break;
        }
      }
      expect(seed, isNotNull);
      final region = MeshFaceRegions.selectionRegion(u, seed!);
      final body = MeshBuild.pocketRegion(
        mesh: u,
        region: region,
        direction: Vector3(7.1, 0, 0),
      );
      expect(
        MeshSeal.isClosedSolid(body),
        isTrue,
        reason:
            'pocket open=${MeshSeal.openBoundaryEdgeCount(body)} '
            'v=${body.vertices.length} f=${body.faces.length}',
      );

      const pad = 0.4;
      final through = MeshPrimitives.cylinder(
        origin: Vector3(-5, 10, 10),
        axis: Vector3(1, 0, 0),
        radius: 4,
        height: 30 + 2 * pad,
        segments: 32,
      );
      final holed = MeshBoolean.run(body, through, MeshBoolean.subtractOp);
      expect(
        MeshSeal.isClosedSolid(holed),
        isTrue,
        reason:
            'first hole open=${MeshSeal.openBoundaryEdgeCount(holed)} '
            'v=${holed.vertices.length} f=${holed.faces.length}',
      );

      final cross = MeshPrimitives.cylinder(
        origin: Vector3(10, -pad, 10),
        axis: Vector3(0, 1, 0),
        radius: 3.5,
        height: 20 + 2 * pad,
        segments: 32,
      );
      final cut = MeshBoolean.run(holed, cross, MeshBoolean.subtractOp);
      var sealed = cut;
      if (!MeshSeal.isClosedSolid(sealed)) {
        sealed = MeshSeal.stitchTJunctions(sealed);
      }
      expect(
        MeshSeal.isClosedSolid(sealed),
        isTrue,
        reason:
            'open=${MeshSeal.openBoundaryEdgeCount(sealed)} '
            'body=${body.vertices.length}v/${body.faces.length}f '
            'holed=${holed.vertices.length}v/${holed.faces.length}f '
            'cut=${sealed.vertices.length}v/${sealed.faces.length}f',
      );
      expect(
        MeshEdit.signedVolume(sealed).abs(),
        lessThan(MeshEdit.signedVolume(holed).abs() - 1),
      );
    });

    test('Decompose splits only vertex-disconnected solids', () {
      final a = MeshPrimitives.cube(10, 10, 10);
      final b = MeshPrimitives.translate(
        MeshPrimitives.cube(10, 10, 10),
        Vector3(20, 0, 0),
      );
      final u = MeshBoolean.run(a, b, MeshBoolean.unionOp);
      expect(MeshTopology.decompose(u), hasLength(2));

      final bar = MeshPrimitives.cube(30, 10, 10);
      final slot = MeshPrimitives.translate(
        MeshPrimitives.cube(4, 12, 12),
        Vector3(13, -1, -1),
      );
      final cut = MeshBoolean.run(bar, slot, MeshBoolean.subtractOp);
      expect(MeshTopology.decompose(cut), hasLength(2));
    });

    test('Decompose still splits an inscribed cylinder at knife edges', () {
      final box = MeshPrimitives.cube(20, 20, 20);
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3(10, 10, -1),
        axis: Vector3(0, 0, 1),
        radius: 10,
        height: 22,
      );
      final d = MeshBoolean.run(box, cyl, MeshBoolean.subtractOp);
      expect(
        MeshTopology.decompose(d),
        hasLength(4),
        reason: 'zero-thickness tangent walls are 4-use non-coplanar knives',
      );
    });
  });
}
