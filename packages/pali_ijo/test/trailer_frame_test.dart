import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final angle in [0.0, .37, 1.2]) {
    for (final height in [50.8, -101.6]) {
      test('frame raw tessellation angle=$angle height=$height', () {
        const l = 14 * 12 * 25.4, w = 7 * 12 * 25.4, b = 2 * 25.4;
        List<Vector3> rect(double x0, double y0, double x1, double y1) => [
          Vector3(x0, y0, 0),
          Vector3(x1, y0, 0),
          Vector3(x1, y1, 0),
          Vector3(x0, y1, 0),
        ];
        final holes = [
          for (var i = 0; i < 4; i++)
            rect(
              b,
              i == 0 ? b : l * i / 4 + b / 2,
              w - b,
              i == 3 ? l - b : l * (i + 1) / 4 - b / 2,
            ),
        ];
        final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), angle);
        Vector3 map(Vector3 p) => q.rotated(p) + Vector3(3456, -5678, 2345);
        final watch = Stopwatch()..start();
        final solid = BrepExtrude.profileWithHoles(
          rect(0, 0, w, l).map(map).toList(),
          [for (final h in holes) h.map(map).toList()],
          q.rotated(Vector3(0, 0, height)),
        );
        final mesh = BrepTessellate.run(solid);
        expect(BrepHeal.topologyIsClosed(solid), isTrue);
        expect(MeshKit.isClosedSolid(mesh), isTrue);
        expect(
          MeshKit.signedVolume(mesh),
          closeTo((2 * b * l + 5 * b * (w - 2 * b)) * height.abs(), .01),
        );
        final edgeUses = <(int, int), int>{};
        final edgeBalance = <(int, int), int>{};
        for (final face in mesh.faces) {
          for (var i = 0; i < face.length; i++) {
            final a = face[i], b = face[(i + 1) % face.length];
            final key = a < b ? (a, b) : (b, a);
            edgeUses[key] = (edgeUses[key] ?? 0) + 1;
            edgeBalance[key] = (edgeBalance[key] ?? 0) + (a < b ? 1 : -1);
          }
        }
        // Closure alone must not hide coincident walls or reversed triangles.
        expect(edgeUses.values, everyElement(2));
        expect(edgeBalance.values, everyElement(0));
        expect(mesh.vertices.length - edgeUses.length + mesh.faces.length, -6);
        expect(watch.elapsed, lessThan(const Duration(seconds: 5)));
        print('Kernel frame: ${watch.elapsedMilliseconds} ms');
      });
    }
  }
}
