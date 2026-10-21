import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  group('mirror', () {
    // A reflection reverses handedness, so loops must be reversed to stay
    // counter-clockwise about the outward normal. Flipping the face normals as
    // well over-corrects and turns the solid inside out, which is invisible to
    // a closedness check and shows up only as negative volume.
    test('keeps the solid right side out', () {
      final box = MeshPrimitives.cube(1, 2, 3);
      final mirrored = MeshXform.mirror(box, Vector3.zero(), Vector3(1, 0, 0));

      expect(IjoMeasure.volume(box), closeTo(6, 1e-9));
      expect(
        IjoMeasure.volume(mirrored),
        closeTo(6, 1e-9),
        reason: 'a mirrored solid still encloses positive volume',
      );
      expect(
        MeshKit.signedVolume(mirrored),
        greaterThan(0),
        reason: 'the tessellation must face outward',
      );
    });

    test('reflects position across the plane', () {
      final box = MeshXform.transformVertices(
        MeshPrimitives.cube(2, 2, 2),
        (p) => p + Vector3(5, 0, 0),
      );
      final mirrored = MeshXform.mirror(box, Vector3.zero(), Vector3(1, 0, 0));
      final b = IjoMeasure.boundingBox(mirrored)!;
      expect(b.min.x, closeTo(-7, 1e-9));
      expect(b.max.x, closeTo(-5, 1e-9));
    });

    test('mirroring twice is the identity', () {
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3(3, 1, 0),
        axis: Vector3(0, 0, 1),
        radius: 2,
        height: 5,
      );
      final there = MeshXform.mirror(cyl, Vector3.zero(), Vector3(1, 0, 0));
      final back = MeshXform.mirror(there, Vector3.zero(), Vector3(1, 0, 0));

      final a = IjoMeasure.of(cyl);
      final z = IjoMeasure.of(back);
      expect(z.volume, closeTo(a.volume, 1e-9));
      expect((z.centroid - a.centroid).length, lessThan(1e-9));
    });

    test('curved faces survive the reflection', () {
      final sphere = MeshPrimitives.sphere(
        center: Vector3(4, 0, 0),
        radius: 2,
      );
      final mirrored = MeshXform.mirror(
        sphere,
        Vector3.zero(),
        Vector3(1, 0, 0),
      );
      expect(IjoMeasure.volume(mirrored), closeTo(IjoMeasure.volume(sphere), 1e-9));
      expect(IjoMeasure.centroid(mirrored).x, closeTo(-4, 1e-9));
    });
  });

  group('scale', () {
    test('uniform scale cubes the volume', () {
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 2,
        height: 4,
      );
      final big = MeshXform.scaleAbout(cyl, Vector3.zero(), Vector3.all(3));
      expect(
        IjoMeasure.volume(big),
        closeTo(IjoMeasure.volume(cyl) * 27, 1e-7),
      );
    });

    test('non-uniform scale of a cylinder stays a closed mesh', () {
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 2,
        height: 4,
      );
      final scaled = MeshXform.scaleAbout(cyl, Vector3.zero(), Vector3(2, 1, 1));
      expect(MeshKit.isClosedSolid(scaled), isTrue);
      expect(IjoMeasure.volume(scaled), greaterThan(IjoMeasure.volume(cyl)));
    });

    test('non-uniform scale of a box works per axis', () {
      final scaled = MeshXform.scaleAbout(
        MeshPrimitives.cube(1, 1, 1),
        Vector3.zero(),
        Vector3(2, 3, 4),
      );
      expect(IjoMeasure.volume(scaled), closeTo(24, 1e-9));
    });
  });
}
