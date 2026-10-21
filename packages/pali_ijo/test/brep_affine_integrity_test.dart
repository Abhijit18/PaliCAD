import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('anisotropic sphere remains an exact serializable ellipsoid', () {
    final sphere = BrepPrimitives.sphere(center: Vector3.zero(), radius: 2);
    final s = BrepXform.scaleAbout(
      sphere,
      Vector3.zero(),
      Vector3(3, 2, 1),
    ).clone();
    final surface = s.faces.single.surface;
    expect(surface, isA<BrepAffineSurface>());
    for (final uv in [(0.4, 0.8), (1.7, 2.2), (3.1, 1.4)]) {
      final p = surface.evaluate(uv.$1, uv.$2);
      expect(
        p.x * p.x / 36 + p.y * p.y / 16 + p.z * p.z / 4,
        closeTo(1, 1e-10),
      );
      final n = Vector3(p.x / 36, p.y / 16, p.z / 4).normalized();
      expect(surface.normalAt(uv.$1, uv.$2).dot(n), closeTo(1, 1e-10));
      final nearest = surface.closestPoint(p + n * 0.3);
      expect((nearest - p).length, lessThan(1e-6));
    }
    final mesh = BrepTessellate.run(s);
    expect(MeshKit.isClosedSolid(mesh), isTrue);
    expect(MeshKit.signedVolume(mesh), closeTo(4 * math.pi * 6 * 4 * 2 / 3, 5));
    expect(BrepClassify.side(s, Vector3(5, 0, 0)), BrepSide.inside);
    expect(BrepClassify.side(s, Vector3(0, 0, 3)), BrepSide.outside);
  });
  test('reflection and repeated scales compose without nesting maps', () {
    final base = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(1, 1, 1),
      radius: 2,
      height: 10,
    );
    final a = BrepXform.scaleAbout(base, Vector3.zero(), Vector3(3, 1, 2));
    final b = BrepXform.mirror(a, Vector3.zero(), Vector3(1, 2, 3));
    final c = BrepXform.scaleAbout(b, Vector3.zero(), Vector3(.5, 2, 1));
    expect(MeshKit.isClosedSolid(BrepTessellate.run(c)), isTrue);
    expect(MeshKit.signedVolume(BrepTessellate.run(c)), greaterThan(0));
    final unwrapped = BrepXform.unwrapAffine(c)!;
    expect(
      unwrapped.solid.faces.any((f) => f.surface is BrepAffineSurface),
      isFalse,
    );
    for (var i = 0; i < c.edges.length; i++) {
      final original = base.edges[i].curve, mapped = c.edges[i].curve;
      for (var j = 0; j < 7; j++) {
        final t = original.t0 + (original.t1 - original.t0) * j / 6;
        expect(
          (mapped.evaluate(t) - unwrapped.transform.point(original.evaluate(t)))
              .length,
          lessThan(1e-8),
        );
      }
    }
  });
  test('ellipsoid slice returns two closed BREP halves', () {
    final s = BrepXform.scaleAbout(
      BrepPrimitives.sphere(center: Vector3.zero(), radius: 2),
      Vector3.zero(),
      Vector3(3, 2, 1),
    );
    final parts = BrepSlice.parts(s, Vector3.zero(), Vector3(1, 2, 3));
    expect(parts.length, 2);
    for (final part in parts) {
      expect(BrepHeal.topologyIsClosed(part), isTrue);
      expect(MeshKit.isClosedSolid(BrepTessellate.run(part)), isTrue);
      expect(part.faces.every((f) => f.surface is BrepAffineSurface), isTrue);
    }
    final volume = parts.fold(
      0.0,
      (sum, s) => sum + MeshKit.signedVolume(BrepTessellate.run(s)),
    );
    expect(volume, closeTo(64 * math.pi, 64 * math.pi * 0.04));
    final exact = parts.fold(0.0, (sum, s) => sum + IjoMeasure.volumeSolid(s));
    expect(exact, closeTo(64 * math.pi, 1e-5));
  });
  test('stretched ruled loft stays exact after mirror and serialization', () {
    final a = [
      Vector3(-5, -5, 0),
      Vector3(5, -5, 0),
      Vector3(5, 5, 0),
      Vector3(-5, 5, 0),
    ];
    final b = [
      for (var i = 0; i < 32; i++)
        Vector3(
          5 * math.cos(i * math.pi / 16),
          5 * math.sin(i * math.pi / 16),
          10,
        ),
    ];
    final loft = BrepLoft.tryProfiles(a, b)!;
    final out = BrepXform.scaleAbout(
      loft,
      Vector3.zero(),
      Vector3(2, 1, -3),
    ).clone();
    expect(MeshKit.isClosedSolid(BrepTessellate.run(out)), isTrue);
    expect(
      MeshKit.signedVolume(BrepTessellate.run(out)),
      closeTo(MeshKit.signedVolume(BrepTessellate.run(loft)) * 6, 1e-6),
    );
  });
}
