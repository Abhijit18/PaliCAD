import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final rotated in [false, true]) {
    test(
      'triangular prism membership near edges and remote faces rotated=$rotated',
      () {
        final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .73);
        Vector3 map(Vector3 p) =>
            rotated ? q.rotated(p) + Vector3(31, -17, 9) : p;
        final prism = BrepXform.map(
          BrepExtrude.polygon([
            Vector3(0, 0, 0),
            Vector3(4, 0, 0),
            Vector3(2, 3, 0),
          ], Vector3(0, 0, 20)),
          map,
        );
        for (final x in [-20.0, -.1, .3, 1.7, 2.3, 3.7, 4.1, 20.0]) {
          for (final y in [-10.0, -.1, .2, 1.0, 2.0, 3.1, 10.0]) {
            for (final z in [-1.0, .1, 10.0, 19.9, 21.0]) {
              final inside =
                  z > 0 && z < 20 && y > 0 && y < 1.5 * x && y < 1.5 * (4 - x);
              final p = Vector3(x, y, z);
              expect(
                BrepClassify.side(prism, map(p)),
                inside ? BrepSide.inside : BrepSide.outside,
                reason: '$p',
              );
            }
          }
        }
        for (final p in [
          Vector3(2, 1, 0),
          Vector3(2, 1, 20),
          Vector3(0, 0, 10),
        ]) {
          expect(BrepClassify.side(prism, map(p)), BrepSide.on);
        }
      },
    );
  }
}
