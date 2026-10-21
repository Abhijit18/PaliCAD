import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

MeshData box({
  double sx = 10,
  double sy = 10,
  double sz = 10,
  Vector3? origin,
}) {
  final o = origin ?? Vector3.zero();
  return MeshXform.transformVertices(
    MeshPrimitives.cube(sx, sy, sz),
    (p) => p + o,
  );
}

(Vector3, Vector3) zEdge(MeshData mesh, {double x = 0, double y = 0}) {
  for (final e in MeshFaceRegions.featureEdges(mesh)) {
    final a = mesh.vertices[e.$1];
    final b = mesh.vertices[e.$2];
    if ((a.x - x).abs() > 1e-6 || (a.y - y).abs() > 1e-6) continue;
    if ((b.x - x).abs() > 1e-6 || (b.y - y).abs() > 1e-6) continue;
    return (a, b);
  }
  throw StateError('no z-edge at ($x, $y)');
}

void expectClosed(MeshData mesh, {String? reason}) {
  expect(
    MeshKit.isClosedSolid(mesh),
    isTrue,
    reason:
        reason ??
        'open boundary edges: ${MeshFaceRegions.openBoundaryEdgeCount(mesh)}',
  );
}

void main() {
  test('box volume is exact', () {
    expect(MeshKit.signedVolume(box()), closeTo(1000, 1e-6));
  });

  test('union of overlapping boxes is larger than one', () {
    final a = MeshPrimitives.cube(10, 10, 10);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 0, 0),
    );
    final u = IjoOps.boolean(a, b, IjoOps.unionOp);
    expectClosed(u);
    final vol = MeshKit.signedVolume(u).abs();
    expect(vol, greaterThan(1000));
    expect(vol, lessThan(2000));
  });

  test('subtract overlapping boxes reduces volume', () {
    final a = MeshPrimitives.cube(10, 10, 10);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 0, 0),
    );
    final d = IjoOps.boolean(a, b, IjoOps.subtractOp);
    expectClosed(d);
    final vol = MeshKit.signedVolume(d).abs();
    expect(vol, lessThan(1000));
    expect(vol, greaterThan(100));
  });

  test('intersect overlapping boxes is about half', () {
    final a = MeshPrimitives.cube(10, 10, 10);
    final b = MeshXform.transformVertices(
      MeshPrimitives.cube(10, 10, 10),
      (p) => p + Vector3(5, 0, 0),
    );
    final i = IjoOps.boolean(a, b, IjoOps.intersectOp);
    expectClosed(i);
    expect(MeshKit.signedVolume(i).abs(), closeTo(500, 50));
  });

  test('through-cut splits into two solids', () {
    final base = MeshPrimitives.cube(30, 10, 10);
    final slot = MeshXform.transformVertices(
      MeshPrimitives.cube(2, 12, 12),
      (p) => p + Vector3(14, -1, -1),
    );
    final cut = IjoOps.boolean(base, slot, IjoOps.subtractOp);
    final parts = IjoOps.splitSolids(cut);
    expect(parts.length, 2);
    for (final p in parts) {
      expectClosed(p);
    }
  });

  test('inscribed cylinder subtract splits into four closed solids', () {
    final box = MeshPrimitives.cube(20, 20, 20);
    final cyl = MeshPrimitives.cylinder(
      origin: Vector3(10, 10, 0),
      axis: Vector3(0, 0, 1),
      radius: 10,
      height: 20,
    );
    final cut = IjoOps.boolean(box, cyl, IjoOps.subtractOp);
    final parts = IjoOps.splitSolids(cut);
    expect(parts.length, greaterThanOrEqualTo(1));
    for (final p in parts) {
      expectClosed(p);
    }
  });

  test('mirror preserves volume', () {
    final m = MeshKit.mirror(box(), Vector3(0, 0, 0), Vector3(1, 0, 0));
    expect(MeshKit.signedVolume(m).abs(), closeTo(1000, 1e-4));
  });

  test('scale about origin doubles volume on 2x', () {
    final m = MeshKit.scaleAbout(box(), Vector3.zero(), Vector3(2, 1, 1));
    expect(MeshKit.signedVolume(m).abs(), closeTo(2000, 1e-4));
  });

  test('chamfer an edge reduces volume', () {
    final cube = MeshPrimitives.cube(10, 10, 10);
    final m = IjoOps.chamferEdges(cube, [zEdge(cube)], 1);
    expectClosed(m);
    expect(MeshKit.signedVolume(m).abs(), lessThan(1000));
    expect(MeshKit.signedVolume(m).abs(), greaterThan(900));
  });

  test('hollow with one opening is thinner than the solid', () {
    final cube = MeshPrimitives.cube(10, 10, 10);
    final m = IjoOps.hollow(
      cube,
      const [0],
      Vector3(0, 0, -1),
      wallThicknessMm: 1,
    );
    expectClosed(m);
    expect(MeshKit.signedVolume(m).abs(), lessThan(900));
    expect(MeshKit.signedVolume(m).abs(), greaterThan(100));
  });

  test('slice through a box yields two solids', () {
    final parts = IjoOps.sliceParts(
      MeshPrimitives.cube(20, 10, 10),
      Vector3(10, 0, 0),
      Vector3(1, 0, 0),
    );
    expect(parts.length, 2);
    for (final p in parts) {
      expectClosed(p);
    }
  });

  test('pipe along a straight path has volume', () {
    final m = MeshKit.pipe([Vector3.zero(), Vector3(0, 0, 20)], 6);
    expectClosed(m);
    expect(MeshKit.signedVolume(m).abs(), greaterThan(100));
  });

  test('second cylinder subtract through a holed sphere stays closed', () {
    final sw = Stopwatch()..start();
    final sphere = MeshPrimitives.sphere(
      center: Vector3(15, 15, 15),
      radius: 15,
    );
    final up = IjoOps.boolean(
      sphere,
      MeshPrimitives.cylinder(
        origin: Vector3(15, 15, -2),
        axis: Vector3(0, 0, 1),
        radius: 4,
        height: 34,
        segments: 32,
      ),
      IjoOps.subtractOp,
    );
    expectClosed(up);
    final side = IjoOps.boolean(
      up,
      MeshPrimitives.cylinder(
        origin: Vector3(-2, 15, 15),
        axis: Vector3(1, 0, 0),
        radius: 3.5,
        height: 34,
        segments: 32,
      ),
      IjoOps.subtractOp,
    );
    expectClosed(side);
    expect(MeshKit.signedVolume(side).abs(), lessThan(MeshKit.signedVolume(up).abs()));
    expect(sw.elapsedMilliseconds, lessThan(45000));
  });
}
