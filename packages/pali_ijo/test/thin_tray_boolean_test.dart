import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid fixture(String name) => BrepSolid.fromJson(
  jsonDecode(File('test/fixtures/thin_tray/$name.json').readAsStringSync()),
);

BrepSolid rebuiltWall() {
  final s = fixture('wall'), cap = s.faces.first;
  final rings = [
    for (final l in cap.loops)
      [
        for (final co in l.coedges)
          s.vertices[co.reversed ? s.edges[co.edge].v1 : s.edges[co.edge].v0],
      ],
  ];
  return BrepExtrude.polygonWithHoles(
    rings.first,
    rings.skip(1).toList(),
    Vector3(0, 0, 10),
  );
}

BrepSolid hollowTray() {
  final pre = fixture('before_hollow');
  final opening = [
    for (var i = 0; i < pre.faces.length; i++)
      if (pre.faces[i].surface is BrepPlane &&
          pre.faces[i].outwardNormal(0, 0).z > .9)
        i,
  ];
  return BrepHollow.run(pre, opening, Vector3(0, 0, 1), wallThicknessMm: 1);
}

BrepSolid cutter({double x = 0}) => BrepExtrude.profile([
  for (var i = 0; i <= 48; i++)
    Vector3(
      x + 11 * math.cos(math.pi + i * math.pi / 48),
      -101,
      11 + 11 * math.sin(math.pi + i * math.pi / 48),
    ),
], Vector3(0, 202, 0));
void expectGeometry(BrepSolid s) {
  expect(BrepHeal.topologyIsClosed(s), isTrue);
  for (final f in s.faces)
    for (final l in f.loops)
      for (final co in l.coedges) {
        final e = s.edges[co.edge];
        for (final t in [
          e.curve.t0,
          (e.curve.t0 + e.curve.t1) / 2,
          e.curve.t1,
        ]) {
          final p = e.curve.evaluate(t);
          expect((f.surface.closestPoint(p) - p).length, lessThan(1e-6));
        }
        expect(
          (e.curve.evaluate(e.curve.t0) - s.vertices[e.v0]).length,
          lessThan(1e-6),
        );
        expect(
          (e.curve.evaluate(e.curve.t1) - s.vertices[e.v1]).length,
          lessThan(1e-6),
        );
      }
}

void main() {
  test('healing preserves the planes of shallow corner facets', () {
    final wall = rebuiltWall();
    expectGeometry(wall);
    expect(wall.faces.length, greaterThan(163));
  });
  test('hollow rebinds arcs when offset junctions slide along cylinders', () {
    final tray = hollowTray();
    expectGeometry(tray);
    expect(MeshKit.isClosedSolid(BrepTessellate.run(tray)), isTrue);
  });
  test('repair of previously merged prism walls preserves both cap wires', () {
    final old = fixture('wall'), before = jsonEncode(old.toJson());
    final repaired = BrepHeal.restorePrismaticWalls(old);
    expectGeometry(repaired);
    expect(jsonEncode(old.toJson()), before);
    expect(repaired.vertices, old.vertices);
    expect(repaired.faces.take(2).map((f) => f.loops.length), [2, 2]);
  });
  for (final legacy in [false, true])
    for (final reverse in [false, true]) {
      test('1 mm floor joins wall legacy=$legacy reverse=$reverse', () {
        final base = fixture('base'),
            wall = legacy ? fixture('wall') : rebuiltWall();
        final before = [jsonEncode(base.toJson()), jsonEncode(wall.toJson())];
        final out = IjoOps.booleanSolid(
          reverse ? wall : base,
          reverse ? base : wall,
          IjoOps.unionOp,
        );
        expectGeometry(out);
        final parts = IjoOps.splitBrep(out);
        expect(parts, hasLength(1));
        final mesh = IjoOps.tessellateClosed(parts.single);
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        expect(MeshKit.signedVolume(mesh), inInclusiveRange(25680, 25720));
        expect([jsonEncode(base.toJson()), jsonEncode(wall.toJson())], before);
      });
    }
  for (final legacy in [false, true])
    for (final x in [0.0, 2.3]) {
      test(
        'tangent semicircle splits tray into two BREP solids legacy=$legacy x=$x',
        () {
          final tray = legacy ? fixture('tray') : hollowTray();
          final before = jsonEncode(tray.toJson());
          final out = IjoOps.booleanSolid(
            tray,
            cutter(x: x),
            IjoOps.subtractOp,
          );
          expectGeometry(out);
          final parts = IjoOps.splitBrep(out);
          expect(parts, hasLength(2));
          var volume = 0.0;
          for (final part in parts) {
            expectGeometry(part);
            final mesh = IjoOps.tessellateClosed(part);
            expect(MeshKit.isClosedSolid(mesh), isTrue);
            expect(MeshKit.signedVolume(mesh), greaterThan(0));
            volume += MeshKit.signedVolume(mesh);
          }
          final slotArea = 121 * math.acos(10 / 11) - 10 * math.sqrt(21);
          final removed = 198 * slotArea + 121 * math.pi;
          final exact = IjoMeasure.volumeSolid(out);
          expect(
            exact,
            closeTo(
              IjoMeasure.volumeSolid(BrepHeal.alignCircularTrims(tray)) -
                  removed,
              .02,
            ),
          );
          expect(volume, closeTo(exact, 40));
          expect(jsonEncode(tray.toJson()), before);
        },
      );
    }
}
