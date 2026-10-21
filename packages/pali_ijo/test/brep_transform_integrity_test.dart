import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('reflection maps every partial circular edge point exactly', () {
    final box = BrepPrimitives.box(20, 20, 20);
    final rounded = IjoOps.filletSolid(box, [0], radius: 2, inverse: false);
    final n = Vector3(1, 2, 3).normalized();
    Vector3 reflect(Vector3 p) => p - n * (2 * p.dot(n));
    final mirrored = BrepXform.mirror(rounded, Vector3.zero(), n);
    for (var i = 0; i < rounded.edges.length; i++) {
      final c = rounded.edges[i].curve, d = mirrored.edges[i].curve;
      for (var k = 0; k <= 8; k++) {
        final t = c.t0 + (c.t1 - c.t0) * k / 8;
        expect((d.evaluate(t) - reflect(c.evaluate(t))).length, lessThan(1e-7));
      }
    }
    expect(MeshKit.isClosedSolid(BrepTessellate.run(mirrored)), isTrue);
    expect(MeshKit.signedVolume(BrepTessellate.run(mirrored)), greaterThan(0));
  });

  test('anisotropic scale keeps rotated planar face normals perpendicular', () {
    final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .7);
    final box = BrepXform.map(BrepPrimitives.box(4, 6, 8), q.rotated);
    final out = BrepXform.scaleAbout(box, Vector3.zero(), Vector3(2, 3, .5));
    for (final f in out.faces) {
      for (final l in f.loops) {
        for (final co in l.coedges) {
          final c = out.edges[co.edge].curve;
          final p = c.evaluate((c.t0 + c.t1) / 2);
          expect((p - f.surface.closestPoint(p)).length, lessThan(1e-7));
        }
      }
    }
    expect(
      MeshKit.signedVolume(BrepTessellate.run(out)),
      closeTo(4 * 6 * 8 * 3, 1e-6),
    );
  });

  test('negative uniform scale keeps outward winding', () {
    final out = BrepXform.scaleAbout(
      BrepPrimitives.box(2, 3, 4),
      Vector3.zero(),
      Vector3.all(-2),
    );
    expect(MeshKit.signedVolume(BrepTessellate.run(out)), closeTo(192, 1e-7));
  });

  test('axial cylinder scale remains analytic', () {
    final cylinder = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 2,
      height: 5,
    );
    final out = BrepXform.scaleAbout(
      cylinder,
      Vector3.zero(),
      Vector3(3, 3, 2),
    );
    final wall = out.faces
        .map((f) => f.surface)
        .whereType<BrepCylinder>()
        .single;
    expect(wall.radius, closeTo(6, 1e-8));
    expect(MeshKit.isClosedSolid(BrepTessellate.run(out)), isTrue);
  });

  test('invalid transforms fail before touching their input', () {
    final box = BrepPrimitives.box(2, 3, 4);
    for (final scale in [
      Vector3(0, 1, 1),
      Vector3(double.nan, 1, 1),
      Vector3.all(double.infinity),
    ]) {
      expect(
        () => BrepXform.scaleAbout(box, Vector3.zero(), scale),
        throwsA(isA<BrepUnsupported>()),
      );
    }
    expect(
      () => BrepXform.mirror(box, Vector3.zero(), Vector3.zero()),
      throwsA(isA<BrepUnsupported>()),
    );
    expect(
      () => BrepAffineTransform(
        Vector3(1e200, 0, 0),
        Vector3(0, 1e200, 0),
        Vector3(0, 0, 1e200),
        Vector3.zero(),
      ),
      throwsA(isA<BrepUnsupported>()),
    );
  });
}
