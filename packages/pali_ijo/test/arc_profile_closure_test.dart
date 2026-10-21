import 'dart:math' as math;
import 'package:test/test.dart';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('unannotated hexagon is not treated as a complete circular run', () {
    final ring = [
      for (var i = 0; i < 6; i++)
        Vector3(
          3 * math.cos(i * math.pi / 3),
          3 * math.sin(i * math.pi / 3),
          0,
        ),
    ];
    final solid = BrepExtrude.profile(ring, Vector3(0, 0, 2));
    expect(solid.faces, hasLength(8));
    expect(solid.faces.every((f) => f.surface is BrepPlane), isTrue);
    expect(IjoMeasure.volumeSolid(solid), closeTo(27 * math.sqrt(3), 1e-6));
  });

  for (final sides in [5, 8, 9]) {
    for (final reversed in [false, true]) {
      for (final shift in [0, 3, 8]) {
        test(
          'arc and chord profile closes exactly: $sides sides, reversed=$reversed, shift=$shift',
          () {
            const radius = 38.32288536783661;
            final angle = 2 * math.pi / sides;
            final samples = [
              for (var i = 0; i <= 8; i++)
                Vector3(
                  radius * math.cos(.27 + angle * i / 8),
                  radius * math.sin(.27 + angle * i / 8),
                  0,
                ),
            ];
            final ordered = reversed ? samples.reversed.toList() : samples;
            final ring = [
              for (var i = 0; i < ordered.length; i++)
                ordered[(i + shift) % ordered.length],
            ];
            final solid = BrepExtrude.sampledProfile(
              ring,
              Vector3(0, 0, 17),
              linearEdges: [(samples.first, samples.last)],
            );
            expect(solid.faces, hasLength(4));
            expect(
              solid.faces.where((f) => f.surface is BrepCylinder),
              hasLength(1),
            );
            expect(
              solid.faces.where((f) => f.surface is BrepPlane),
              hasLength(3),
            );
            expect(BrepHeal.topologyIsClosed(solid), isTrue);
            expect(
              MeshKit.isClosedSolid(IjoOps.tessellateClosed(solid)),
              isTrue,
            );
            expect(
              IjoMeasure.volumeSolid(solid),
              closeTo(
                .5 * radius * radius * (angle - math.sin(angle)) * 17,
                1e-6,
              ),
            );
          },
        );
      }
    }
  }
}
