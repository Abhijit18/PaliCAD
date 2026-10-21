import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final splitSharedEdge in [false, true]) {
    for (final reverseFirst in [false, true]) {
      test(
        'merge coplanar wires, split=$splitSharedEdge reverse=$reverseFirst',
        () {
          final builder = BrepBuilder();
          final points = [
            Vector3(0, 0, 0),
            Vector3(10, 0, 0),
            Vector3(20, 0, 0),
            Vector3(20, 10, 0),
            Vector3(10, 10, 0),
            Vector3(0, 10, 0),
            Vector3(10, 5, 0),
          ];
          for (final p in points) {
            builder.vertex(p);
          }
          final edges = <String, int>{};
          BrepLoop wire(List<int> ids) {
            return BrepLoop([
              for (var i = 0; i < ids.length; i++)
                () {
                  final a = ids[i], b = ids[(i + 1) % ids.length];
                  final key = a < b ? '$a:$b' : '$b:$a';
                  final edge = edges.putIfAbsent(
                    key,
                    () => builder.edge(a, b, BrepLine(points[a], points[b])),
                  );
                  return BrepCoedge(
                    edge,
                    reversed: builder.edges[edge].v0 != a,
                  );
                }(),
            ]);
          }

          var left = [0, 1, if (splitSharedEdge) 6, 4, 5];
          var right = [1, if (splitSharedEdge) 6, 4, 3, 2];
          if (reverseFirst) {
            left = left.reversed.toList();
            right = right.reversed.toList();
          }
          // Both surfaces face +Z, but the cutter-derived right wire is wound
          // oppositely to the left wire. Surface normals do not imply trim order.
          final plane = BrepPlane(
            origin: Vector3.zero(),
            normal: Vector3(0, 0, 1),
          );
          builder.face(plane, [wire(left)]);
          builder.face(plane, [wire(right)]);
          final solid = builder.build();
          final merged = BrepHeal.mergeCoplanar(solid);
          expect(merged.faces, hasLength(1));
          final loop = merged.faces.single.loops.single.coedges;
          expect(loop, hasLength(6));
          for (var i = 0; i < loop.length; i++) {
            final a = loop[i], b = loop[(i + 1) % loop.length];
            final ea = merged.edges[a.edge], eb = merged.edges[b.edge];
            expect(a.reversed ? ea.v0 : ea.v1, b.reversed ? eb.v1 : eb.v0);
          }
          final mesh = BrepTessellate.run(merged);
          var area = 0.0;
          for (final f in mesh.faces) {
            area +=
                (mesh.vertices[f[1]] - mesh.vertices[f[0]])
                    .cross(mesh.vertices[f[2]] - mesh.vertices[f[0]])
                    .length /
                2;
          }
          expect(area, closeTo(200, 1e-8));
          expect(
            solid.faces,
            hasLength(2),
            reason: 'Healing must not change the input',
          );
        },
      );
    }
  }
}
