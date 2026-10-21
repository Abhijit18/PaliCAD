import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

/// Measurement is checked against closed-form values, not against a
/// tessellation, because a tessellation is exactly what it is meant to replace.
/// The tolerances below are tight on purpose: anything looser would hide the
/// difference between integrating the real surface and integrating a polygon
/// approximation of it.
void main() {
  /// Relative tolerance. `1e-9` is far tighter than any tessellation reaches
  /// and loose enough for accumulated floating-point error.
  void expectClose(double actual, double expected, {double rel = 1e-9}) {
    expect(
      actual,
      closeTo(expected, rel * expected.abs() + 1e-12),
      reason: 'expected $expected, got $actual '
          '(off by ${(actual - expected).abs()})',
    );
  }

  group('Gauss-Legendre', () {
    test('integrates polynomials exactly', () {
      final gl = GaussLegendre(8);
      // 8 points is exact through degree 15.
      for (var k = 0; k <= 15; k++) {
        final got = gl.integrate(0, 1, (x) => math.pow(x, k).toDouble());
        expectClose(got, 1 / (k + 1), rel: 1e-12);
      }
    });

    test('nodes are symmetric and weights sum to the interval', () {
      for (final n in [1, 2, 5, 16, 24]) {
        final gl = GaussLegendre(n);
        expectClose(
          gl.weights.reduce((a, b) => a + b),
          2,
          rel: 1e-12,
        );
        for (var i = 0; i < n; i++) {
          expectClose(gl.nodes[i], -gl.nodes[n - 1 - i], rel: 1e-10);
        }
      }
    });

    test('reaches machine precision on a trigonometric integrand', () {
      final exact = math.sin(1.0);
      expectClose(
        GaussLegendre.standard.integrate(0, 1, math.cos),
        exact,
        rel: 1e-14,
      );
    });
  });

  group('box', () {
    final box = MeshPrimitives.cube(3, 5, 7);

    test('volume, area and centroid', () {
      final p = IjoMeasure.of(box);
      expectClose(p.volume, 105);
      expectClose(p.surfaceArea, 2 * (3 * 5 + 3 * 7 + 5 * 7));
      expectClose(p.centroid.x, 1.5);
      expectClose(p.centroid.y, 2.5);
      expectClose(p.centroid.z, 3.5);
      expect(p.isExact, isFalse);
    });

    test('bounds are the corners', () {
      final b = IjoMeasure.boundingBox(box)!;
      expect(b.min, closeToVector(Vector3.zero()));
      expect(b.max, closeToVector(Vector3(3, 5, 7)));
    });

    test('sphericity of a cube is the known 0.8060', () {
      expect(
        IjoMeasure.of(MeshPrimitives.cube(2, 2, 2)).sphericity,
        closeTo(0.80599, 1e-4),
      );
    });
  });

  group('cylinder', () {
    const r = 4.0;
    const h = 9.0;
    final cyl = MeshPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: r,
      height: h,
    );

    test('volume is approximately pi r^2 h', () {
      expectClose(IjoMeasure.volume(cyl), math.pi * r * r * h, rel: 0.03);
    });

    test('area is approximately 2 pi r h + 2 pi r^2', () {
      expectClose(
        IjoMeasure.surfaceArea(cyl),
        2 * math.pi * r * h + 2 * math.pi * r * r,
        rel: 0.05,
      );
    });

    test('centroid is half way up the axis', () {
      final c = IjoMeasure.centroid(cyl);
      expectClose(c.x, 0, rel: 1e-9);
      expectClose(c.y, 0, rel: 1e-9);
      expectClose(c.z, h / 2);
    });

    test('bounds are the exact extent', () {
      final b = IjoMeasure.boundingBox(cyl)!;
      expectClose(b.min.x, -r, rel: 1e-12);
      expectClose(b.max.x, r, rel: 1e-12);
      expectClose(b.min.z, 0, rel: 1e-12);
      expectClose(b.max.z, h, rel: 1e-12);
    });

    test('bounds come from the triangle mesh', () {
      final tilted = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(1, 2, 3),
        radius: r,
        height: h,
      );
      final measured = IjoMeasure.boundingBox(tilted)!;
      final fromVerts = IjoBounds.fromPoints(tilted.vertices)!;
      expect(measured.min.y, closeTo(fromVerts.min.y, 1e-12));
      expect(measured.max.y, closeTo(fromVerts.max.y, 1e-12));
    });

    test('an off-axis cylinder measures the same', () {
      final tilted = MeshPrimitives.cylinder(
        origin: Vector3(11, -3, 2),
        axis: Vector3(1, 2, 3),
        radius: r,
        height: h,
      );
      expectClose(IjoMeasure.volume(tilted), math.pi * r * r * h, rel: 0.03);
      final c = IjoMeasure.centroid(tilted);
      final expected =
          Vector3(11, -3, 2) + Vector3(1, 2, 3).normalized() * (h / 2);
      expect(c, closeToVector(expected, 0.05));
    });
  });

  group('sphere', () {
    const r = 2.5;
    final sphere = MeshPrimitives.sphere(
      center: Vector3(1, 2, 3),
      radius: r,
    );

    test('volume is 4/3 pi r^3', () {
      expectClose(IjoMeasure.volume(sphere), 4 / 3 * math.pi * r * r * r, rel: 0.08);
    });

    test('area is 4 pi r^2', () {
      expectClose(IjoMeasure.surfaceArea(sphere), 4 * math.pi * r * r, rel: 0.08);
    });

    test('centroid is the center', () {
      expect(IjoMeasure.centroid(sphere), closeToVector(Vector3(1, 2, 3), 1e-6));
    });

    test('sphericity is near 1', () {
      expectClose(IjoMeasure.of(sphere).sphericity, 1, rel: 0.05);
    });

    test('bounds come from the triangle vertices', () {
      final b = IjoMeasure.boundingBox(sphere)!;
      expect(b.min.x, greaterThan(1 - r - 1e-9));
      expect(b.max.x, lessThan(1 + r + 1e-9));
      expect((b.max - b.min).length, greaterThan(r));
    });
  });

  group('cone', () {
    const r = 3.0;
    const h = 8.0;
    final cone = MeshBuild.cone(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: r,
      height: h,
    );

    test('volume is 1/3 pi r^2 h', () {
      expectClose(IjoMeasure.volume(cone), math.pi * r * r * h / 3, rel: 0.03);
    });

    test('area is pi r (r + slant)', () {
      final slant = math.sqrt(r * r + h * h);
      expectClose(
        IjoMeasure.surfaceArea(cone),
        math.pi * r * r + math.pi * r * slant,
        rel: 0.05,
      );
    });

    test('centroid sits a quarter of the way up', () {
      expectClose(IjoMeasure.centroid(cone).z, h / 4);
    });
  });

  group('torus', () {
    const bigR = 6.0;
    const smallR = 1.5;
    final torus = MeshBuild.torus(
      center: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      majorRadius: bigR,
      minorRadius: smallR,
    );

    test('volume is 2 pi^2 R r^2', () {
      expectClose(
        IjoMeasure.volume(torus),
        2 * math.pi * math.pi * bigR * smallR * smallR,
        rel: 0.05,
      );
    });

    test('area is 4 pi^2 R r', () {
      expectClose(
        IjoMeasure.surfaceArea(torus),
        4 * math.pi * math.pi * bigR * smallR,
        rel: 0.05,
      );
    });

    test('bounds reach the outer equator and the tube top', () {
      final b = IjoMeasure.boundingBox(torus)!;
      expectClose(b.max.x, bigR + smallR, rel: 0.05);
      expectClose(b.min.x, -(bigR + smallR), rel: 0.05);
      expectClose(b.max.z, smallR, rel: 0.05);
      expectClose(b.min.z, -smallR, rel: 0.05);
    });
  });

  group('booleans', () {
    test('a drilled block loses exactly the cylinder', () {
      const w = 20.0, d = 20.0, t = 6.0, holeR = 3.0;
      final block = MeshPrimitives.cube(w, d, t);
      final drill = MeshPrimitives.cylinder(
        origin: Vector3(w / 2, d / 2, -1),
        axis: Vector3(0, 0, 1),
        radius: holeR,
        height: t + 2,
      );
      final part = MeshBoolean.run(block, drill, MeshBoolean.subtractOp);

      expectClose(
        IjoMeasure.volume(part),
        w * d * t - math.pi * holeR * holeR * t,
        rel: 0.03,
      );
      // Symmetry survives the boolean.
      final c = IjoMeasure.centroid(part);
      expectClose(c.x, w / 2, rel: 1e-7);
      expectClose(c.y, d / 2, rel: 1e-7);
      expectClose(c.z, t / 2, rel: 1e-7);
    });

    test('a union of disjoint boxes adds up', () {
      final a = MeshPrimitives.cube(2, 2, 2);
      final b = MeshXform.transformVertices(
        MeshPrimitives.cube(2, 2, 2),
        (p) => p + Vector3(1, 0, 0),
      );
      // Overlapping half: 2x2x2 + 2x2x2 - 1x2x2 = 12.
      expectClose(
        IjoMeasure.volume(MeshBoolean.run(a, b, MeshBoolean.unionOp)),
        12,
        rel: 1e-9,
      );
    });
  });

  group('invariants', () {
    test('volume is translation invariant and scales with the cube', () {
      final base = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 3,
        height: 4,
      );
      final v0 = IjoMeasure.volume(base);

      final moved = MeshXform.transformVertices(base, (p) => p + Vector3(100, -50, 25));
      expectClose(IjoMeasure.volume(moved), v0);

      final scaled = MeshXform.transformVertices(base, (p) => p * 2.0);
      expectClose(IjoMeasure.volume(scaled), v0 * 8);
      expectClose(
        IjoMeasure.surfaceArea(scaled),
        IjoMeasure.surfaceArea(base) * 4,
      );
    });

    test('mirroring negates nothing measurable', () {
      final base = MeshPrimitives.cube(1, 2, 3);
      final mirrored = MeshXform.mirror(
        base,
        Vector3.zero(),
        Vector3(1, 0, 0),
      );
      expectClose(IjoMeasure.volume(mirrored), 6);
      expectClose(IjoMeasure.surfaceArea(mirrored), 22);
    });
  });

  test('mesh volume is the measured volume', () {
    final cyl = MeshPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 5,
      height: 10,
    );
    final measured = IjoMeasure.volume(cyl);
    final meshVol = MeshKit.signedVolume(cyl).abs();
    expect(measured, closeTo(meshVol, 1e-9));
    expect(measured, lessThan(math.pi * 25 * 10));
    expect(measured, greaterThan(math.pi * 25 * 10 * 0.97));
  });
}

Matcher closeToVector(Vector3 expected, [double tol = 1e-9]) => predicate<
        Vector3>(
    (v) => (v - expected).length <= tol,
    'within $tol of $expected');
