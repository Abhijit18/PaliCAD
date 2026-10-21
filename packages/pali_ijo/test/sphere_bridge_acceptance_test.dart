// Passing foundation for the broader failing stress sequence in
// tool/stress_sphere_bridge.dart. Cuts are evaluated there, not assumed valid.
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid bridge() {
  final first = IjoOps.booleanSolid(
    BrepPrimitives.box(20, 20, 20),
    BrepPrimitives.sphere(center: Vector3(22, 10, 10), radius: 8),
    IjoOps.unionOp,
  );
  return IjoOps.booleanSolid(
    first,
    BrepXform.translate(BrepPrimitives.box(20, 20, 20), Vector3(24, 0, 0)),
    IjoOps.unionOp,
  );
}

int edge(BrepSolid s, double x, double y) => s.edges.indexWhere(
  (e) =>
      e.curve is BrepLine &&
      [
        s.vertices[e.v0],
        s.vertices[e.v1],
      ].every((v) => (v.x - x).abs() < 1e-6 && (v.y - y).abs() < 1e-6) &&
      (s.vertices[e.v0].z - s.vertices[e.v1].z).abs() > 19.9,
);

double check(BrepSolid s) {
  expect(BrepHeal.topologyIsClosed(s), isTrue);
  expect(IjoOps.splitBrep(s), hasLength(1));
  expect(
    s.faces.any(
      (f) =>
          f.surface is BrepSphere &&
          ((f.surface as BrepSphere).center - Vector3(22, 10, 10)).length <
              1e-6 &&
          ((f.surface as BrepSphere).radius - 8).abs() < 1e-6,
    ),
    isTrue,
  );
  final m = IjoOps.tessellateClosed(s);
  expect(MeshKit.isClosedSolid(m), isTrue);
  expect(BrepHeal.topologyIsClosed(PackedBrep.fromSolid(s).toSolid()), isTrue);
  return MeshKit.signedVolume(m);
}

void main() {
  test('two cubes separated by 4 mm connect only through the sphere', () {
    expect(check(bridge()), closeTo(16000 + math.pi * (64 * 4 - 16 / 3), 25));
  });
  test('face-loop fillet preserves the attached sphere and opposite cube', () {
    final original = bridge();
    final v = check(original);
    final selected = BrepEdgeSelect.ofFace(
      BrepFaceSelect.extreme(Vector3(-1, 0, 0)),
    ).resolve(original);
    final rounded = IjoOps.filletSolid(
      original,
      selected,
      radius: 1,
      inverse: false,
    );
    final roundedVolume = check(rounded);
    expect(roundedVolume, lessThan(v));
    expect(roundedVolume, greaterThan(v * .99));
    expect(check(original), closeTo(v, 1e-8));
    expect(rounded.vertices.any((p) => (p.x - 44).abs() < 1e-8), isTrue);
  });
  test('bridge retains both cubes after one fillet then one chamfer', () {
    final original = bridge();
    final v = check(original);
    final e = edge(original, 0, 0);
    expect(e, greaterThanOrEqualTo(0));
    final rounded = IjoOps.filletSolid(
      original,
      [e],
      radius: 2,
      inverse: false,
    );
    expect(check(rounded), closeTo(v - 20 * 4 * (1 - math.pi / 4), 0.2));
    final ce = edge(rounded, 44, 20);
    expect(ce, greaterThanOrEqualTo(0));
    final beveled = IjoOps.chamferSolid(rounded, [ce], 1);
    expect(check(beveled), closeTo(check(rounded) - 10, 1e-6));
  });
}
