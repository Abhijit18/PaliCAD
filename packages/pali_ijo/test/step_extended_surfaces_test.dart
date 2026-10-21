import 'dart:convert';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/brep/spline_convert.dart';
import 'package:pali_ijo/src/io/step_syntax.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

// Small authored STEP fixtures, independent of the private engine assembly.
class _Patch {
  final w = StepWriter();
  int point(Vector3 p) => w.add('CARTESIAN_POINT', "'',(${p.x},${p.y},${p.z})");
  int direction(Vector3 p) => w.add('DIRECTION', "'',(${p.x},${p.y},${p.z})");
  int placement(Vector3 origin, Vector3 z, Vector3 x) => w.add(
    'AXIS2_PLACEMENT_3D',
    "'',#${point(origin)},#${direction(z)},#${direction(x)}",
  );
  int spline(List<Vector3> points) => w.add(
    'B_SPLINE_CURVE_WITH_KNOTS',
    "'',${points.length - 1},${StepWriter.refs(points.map(point))},.UNSPECIFIED.,.F.,.F.,(${points.length},${points.length}),(0.,1.),.UNSPECIFIED.",
  );
  int edge(Vector3 a, Vector3 b, int curve, {bool forward = true}) {
    final va = w.add('VERTEX_POINT', "'',#${point(a)}"),
        vb = w.add('VERTEX_POINT', "'',#${point(b)}");
    final e = w.add(
      'EDGE_CURVE',
      "'',#$va,#$vb,#$curve,${forward ? '.T.' : '.F.'}",
    );
    return w.add('ORIENTED_EDGE', "'',*,*,#$e,.T.");
  }

  int line(Vector3 a, Vector3 b) {
    final vector = w.add('VECTOR', "'',#${direction((b - a).normalized())},1.");
    return edge(a, b, w.add('LINE', "'',#${point(a)},#$vector"));
  }

  Uint8List finish(int surface, List<List<int>> loops) {
    final bounds = [
      for (final es in loops)
        w.add(
          'FACE_BOUND',
          "'',#${w.add('EDGE_LOOP', "'',${StepWriter.refs(es)}")},.T.",
        ),
    ];
    final face = w.add(
      'ADVANCED_FACE',
      "'',${StepWriter.refs(bounds)},#$surface,.T.",
    );
    final shell = w.add('CLOSED_SHELL', "'',(#$face)");
    w.add('MANIFOLD_SOLID_BREP', "'Synthetic patch',#$shell");
    return Uint8List.fromList(
      w.bytes(
        description: 'Regression fixture',
        fileName: 'patch.step',
        originator: 'Pali Ijo',
        schema: IjoStep.schema,
      ),
    );
  }
}

void main() {
  test('hyperbola trims retain the exact conic in either edge direction', () {
    for (final reverse in [false, true]) {
      final f = _Patch(),
          at = f.placement(Vector3.zero(), Vector3(0, 0, 1), Vector3(1, 0, 0));
      final curve = f.w.add('HYPERBOLA', "'',#$at,2.,3.");
      Vector3 point(double t) => Vector3(
        math.exp(t) + math.exp(-t),
        1.5 * (math.exp(t) - math.exp(-t)),
        0,
      );
      final a = point(reverse ? 1.3 : -.7), b = point(reverse ? -.7 : 1.3);
      final bytes = f.finish(f.w.add('PLANE', "'',#$at"), [
        [f.edge(a, b, curve, forward: !reverse), f.line(b, a)],
      ]);
      final c = IjoStep.readSolids(
        bytes,
        allowOpenShells: true,
      ).solids.single.edges.first.curve;
      expect(c, isA<BrepSplineCurve>());
      expect((c.evaluate(c.t0) - a).length, lessThan(1e-9));
      expect((c.evaluate(c.t1) - b).length, lessThan(1e-9));
      for (var i = 0; i <= 100; i++) {
        final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 100);
        expect(p.x * p.x / 4 - p.y * p.y / 9, closeTo(1, 1e-10));
      }
    }
  });
  test('unordered planar bounds retain an annulus, including its hole', () {
    final f = _Patch(),
        at = f.placement(Vector3.zero(), Vector3(0, 0, 1), Vector3(1, 0, 0));
    final loops = <List<int>>[];
    for (final radius in [2.0, 5.0]) {
      final p = Vector3(radius, 0, 0), c = f.w.add('CIRCLE', "'',#$at,$radius");
      loops.add([f.edge(p, p, c, forward: radius == 5)]);
    }
    final solid = IjoStep.readSolids(
      f.finish(f.w.add('PLANE', "'',#$at"), loops),
      allowOpenShells: true,
    ).solids.single;
    final mesh = BrepTessellate.run(solid);
    expect(mesh.faces, isNotEmpty);
    var area = 0.0;
    for (final t in mesh.faces) {
      final a = mesh.vertices[t[0]],
          b = mesh.vertices[t[1]],
          c = mesh.vertices[t[2]];
      area += (b - a).cross(c - a).length / 2;
      expect(((a + b + c) / 3).length, greaterThan(1.98));
    }
    expect(area, closeTo(21 * math.pi, 21 * math.pi * .01));
  });
  test(
    'linear extrusion retains a curved spline and displays its entire patch',
    () {
      final f = _Patch(),
          a = Vector3(0, 0, 0),
          b = Vector3(10, 0, 0),
          c = Vector3(10, 0, 4),
          d = Vector3(0, 0, 4);
      final curve = f.spline([a, Vector3(5, 2, 0), b]),
          upper = f.spline([d, Vector3(5, 2, 4), c]);
      final vector = f.w.add(
        'VECTOR',
        "'',#${f.direction(Vector3(0, 0, 1))},1.",
      );
      final bytes = f.finish(
        f.w.add('SURFACE_OF_LINEAR_EXTRUSION', "'',#$curve,#$vector"),
        [
          [
            f.edge(a, b, curve),
            f.line(b, c),
            f.edge(c, d, upper, forward: false),
            f.line(d, a),
          ],
        ],
      );
      expect(
        () => IjoStep.readSolids(bytes),
        throwsA(isA<StepBrepUnsupportedException>()),
      );
      final result = IjoStep.readSolids(bytes, allowOpenShells: true),
          solid = result.solids.single;
      expect(result.notes.join(), contains('open boundaries'));
      final surface = solid.faces.single.surface;
      expect(surface, isA<BrepRuledSurface>());
      for (final z in [0.0, 1.0, 4.0]) {
        final p = Vector3(5, 1, z);
        expect((surface.closestPoint(p) - p).length, lessThan(1e-6));
      }
      final before = jsonEncode(solid.toJson());
      final mesh = BrepTessellate.run(
        solid,
        tessellation: MeshTessellation.importPreview,
      );
      expect(mesh.faceIds!.toSet(), {0});
      expect(
        mesh.vertices.every(
          (p) => p.x.isFinite && p.y.isFinite && p.z.isFinite,
        ),
        isTrue,
      );
      expect(jsonEncode(solid.toJson()), before);
      expect(
        () => IjoStep.readSolids(
          Uint8List.fromList(
            utf8.encode(
              utf8
                  .decode(bytes)
                  .replaceAll(
                    'SURFACE_OF_LINEAR_EXTRUSION',
                    'UNSUPPORTED_SURFACE',
                  ),
            ),
          ),
          allowOpenShells: true,
        ),
        throwsA(isA<StepBrepUnsupportedException>()),
      );
    },
  );
  test('surface of revolution retains an exact rational cylinder', () {
    final f = _Patch(), curve = f.spline([Vector3(3, 0, 0), Vector3(3, 0, 4)]);
    final axis = f.w.add(
      'AXIS1_PLACEMENT',
      "'',#${f.point(Vector3.zero())},#${f.direction(Vector3(0, 0, 1))}",
    );
    final at = f.placement(Vector3.zero(), Vector3(0, 0, 1), Vector3(1, 0, 0)),
        circle = f.w.add('CIRCLE', "'',#$at,3.");
    final bytes = f.finish(
      f.w.add('SURFACE_OF_REVOLUTION', "'',#$curve,#$axis"),
      [
        [f.edge(Vector3(3, 0, 0), Vector3(3, 0, 0), circle)],
      ],
    );
    final s = IjoStep.readSolids(
      bytes,
      allowOpenShells: true,
    ).solids.single.faces.single.surface;
    expect(s, isA<BrepSplineSurface>());
    for (var i = 0; i <= 32; i++) {
      final p = s.evaluate(i / 32, .37);
      expect(math.sqrt(p.x * p.x + p.y * p.y), closeTo(3, 1e-10));
      expect(p.z, closeTo(1.48, 1e-10));
    }
  });
  test(
    'offset spline normals, projection and serialization retain geometry',
    () {
      final base = BrepSplineSurface(
        degreeU: 2,
        degreeV: 1,
        points: [
          [Vector3(0, 0, 0), Vector3(0, 4, 0)],
          [Vector3(2, 0, 2), Vector3(2, 4, 2)],
          [Vector3(4, 0, 0), Vector3(4, 4, 0)],
        ],
        knotsU: [2, 2, 2, 5, 5, 5],
        knotsV: [-2, -2, 3, 3],
      );
      final s = BrepOffsetSurface(base, .3),
          back = BrepSurface.fromJson(
            jsonDecode(jsonEncode(s.toJson())) as Map<String, dynamic>,
          );
      for (var i = 0; i <= 10; i++) {
        final u = i / 10,
            v = .37,
            p = s.evaluate(u, v),
            q = base.evaluate(2 + 3 * u, -2 + 5 * v);
        expect((p - q).length, closeTo(.3, 1e-10));
        expect((p - back.evaluate(u, v)).length, lessThan(1e-10));
        expect((s.closestPoint(p) - p).length, lessThan(1e-7));
        expect(s.normalAt(u, v).length, closeTo(1, 1e-10));
      }
    },
  );
  test(
    'offset surfaces survive STEP export, import and nonuniform transforms',
    () {
      final solid = BrepPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 3.5,
        height: 4,
      );
      final side = solid.faces.firstWhere((f) => f.surface is BrepCylinder);
      final a = BrepCircle(
        center: Vector3.zero(),
        normal: Vector3(0, 0, 1),
        xDir: Vector3(1, 0, 0),
        radius: 3,
      );
      final b = BrepCircle(
        center: Vector3(0, 0, 4),
        normal: Vector3(0, 0, 1),
        xDir: Vector3(1, 0, 0),
        radius: 3,
      );
      side.surface = BrepOffsetSurface(
        BrepSplineConvert.surface(BrepRuledSurface(a, b)),
        .5,
      );
      final bytes = IjoStep.writeSolid(solid);
      expect(utf8.decode(bytes), contains('OFFSET_SURFACE('));
      final back = IjoStep.readSolid(bytes);
      expect(BrepHeal.topologyIsClosed(back), isTrue);
      final s = back.faces
          .firstWhere((f) => f.surface is BrepOffsetSurface)
          .surface;
      for (var i = 0; i <= 20; i++) {
        final p = s.evaluate(i / 20, .25);
        expect(math.sqrt(p.x * p.x + p.y * p.y), closeTo(3.5, 1e-8));
        expect(p.z, closeTo(1, 1e-8));
      }
      final changed = BrepXform.scaleAbout(
        back,
        Vector3.zero(),
        Vector3(2, 1, 3),
      );
      final index = back.faces.indexWhere(
        (f) => f.surface is BrepOffsetSurface,
      );
      final p = s.evaluate(.17, .39),
          q = changed.faces[index].surface.evaluate(.17, .39);
      expect((q - Vector3(2 * p.x, p.y, 3 * p.z)).length, lessThan(1e-8));
      final display = BrepTessellate.run(
        back,
        tessellation: MeshTessellation.importPreview,
      );
      expect(
        display.faceIds!.toSet(),
        containsAll(List.generate(back.faces.length, (i) => i)),
      );
    },
  );

  test('periodic offset derivatives remain finite across the seam', () {
    final circle = BrepCircle(
      center: Vector3.zero(),
      normal: Vector3(0, 0, 1),
      xDir: Vector3(1, 0, 0),
      radius: 3,
    );
    final top = BrepCircle(
      center: Vector3(0, 0, 4),
      normal: Vector3(0, 0, 1),
      xDir: Vector3(1, 0, 0),
      radius: 3,
    );
    final base = BrepSplineConvert.surface(BrepRuledSurface(circle, top));
    final s = BrepOffsetSurface(base, .5);
    expect(s.periodicU, isTrue);
    for (final u in [-.1, 0.0, 1.0, 1.1]) {
      final p = s.evaluate(u, .5), n = s.normalAt(u, .5);
      expect(math.sqrt(p.x * p.x + p.y * p.y), closeTo(3.5, 1e-8));
      expect(n.length, closeTo(1, 1e-8));
    }
  });
}
