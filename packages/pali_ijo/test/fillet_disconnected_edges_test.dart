import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';

void main() {
  for (final order in [
    [0, 1, 2, 3],
    [3, 2, 1, 0],
    [1, 3, 0, 2],
  ]) {
    test(
      'disconnected upright fillets survive topology rebuild order=$order',
      () {
        final box = BrepPrimitives.box(40, 30, 20);
        final upright = [
          for (var i = 0; i < box.edges.length; i++)
            if ((box.vertices[box.edges[i].v1] - box.vertices[box.edges[i].v0])
                    .z
                    .abs() >
                19)
              i,
        ];
        final rounded = IjoOps.filletSolid(
          box,
          [for (final i in order) upright[i]],
          radius: 5,
          inverse: false,
        );
        expect(BrepHeal.topologyIsClosed(rounded), isTrue);
        for (final cap in rounded.faces.where(
          (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).z.abs() > .99,
        )) {
          final curves = [
            for (final l in cap.loops)
              for (final co in l.coedges) rounded.edges[co.edge].curve,
          ];
          expect(curves.whereType<BrepCircle>(), hasLength(4));
          expect(curves.whereType<BrepLine>(), hasLength(4));
        }
        expect(
          IjoMeasure.volumeSolid(rounded),
          closeTo((1200 - (4 - math.pi) * 25) * 20, 1e-6),
        );
        expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(rounded)), isTrue);
      },
    );
  }
}
