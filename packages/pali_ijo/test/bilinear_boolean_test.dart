import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid twisted(Vector3 translation, double angle) {
  final ring = [
    Vector3(-3, -3, 0),
    Vector3(3, -3, 0),
    Vector3(3, 3, 0),
    Vector3(-3, 3, 0),
  ];
  return BrepLoft.sections([
    ring.map((p) => p + translation).toList(),
    [
      for (final p in ring)
        Vector3(
              p.x * math.cos(angle) - p.y * math.sin(angle),
              p.x * math.sin(angle) + p.y * math.cos(angle),
              7,
            ) +
            translation,
    ],
  ]);
}

void main() {
  for (final op in [IjoOps.subtractOp, IjoOps.unionOp, IjoOps.intersectOp]) {
    test(
      'twisted bilinear lofts intersect their curved walls: operation $op',
      () {
        final a = twisted(Vector3.zero(), .4),
            b = twisted(Vector3(3, .4, 1), -.3);
        final result = IjoOps.booleanSolid(a, b, op);
        expect(BrepHeal.topologyIsClosed(result), isTrue);
        expect(MeshKit.isClosedSolid(BrepTessellate.run(result)), isTrue);
        expect(
          BrepClassify.side(result, Vector3(1.5, .5, 4)),
          op == IjoOps.subtractOp ? BrepSide.outside : BrepSide.inside,
        );
        expect(
          BrepClassify.side(result, Vector3(-2, 0, 3)),
          op == IjoOps.intersectOp ? BrepSide.outside : BrepSide.inside,
        );
      },
    );
  }
}
