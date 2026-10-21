import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  final box = MeshPrimitives.cube(20, 20, 20);
  print('SMOKE box ${box.vertices.length}v/${box.faces.length}f '
      'vol=${MeshEdit.signedVolume(box).abs()}');

  final sw = Stopwatch()..start();
  final out = MeshFillet.fillet(
    box,
    [(Vector3(0, 0, 0), Vector3(0, 0, 20))],
    radius: 3,
    inverse: false,
  );
  print('SMOKE edge ${sw.elapsedMilliseconds}ms '
      '${out.vertices.length}v/${out.faces.length}f '
      'closed=${MeshSeal.isClosedSolid(out)} '
      'open=${MeshSeal.openBoundaryEdgeCount(out)} '
      'vol=${MeshEdit.signedVolume(out).abs().toStringAsFixed(2)}');

  final top = [
    (Vector3(0, 0, 20), Vector3(20, 0, 20)),
    (Vector3(20, 0, 20), Vector3(20, 20, 20)),
    (Vector3(20, 20, 20), Vector3(0, 20, 20)),
    (Vector3(0, 20, 20), Vector3(0, 0, 20)),
  ];
  final sw2 = Stopwatch()..start();
  final face = MeshFillet.fillet(box, top, radius: 3, inverse: false);
  print('SMOKE face ${sw2.elapsedMilliseconds}ms '
      'complete=${MeshFillet.completeCornerEdges(box, top).length} '
      '${face.vertices.length}v/${face.faces.length}f '
      'closed=${MeshSeal.isClosedSolid(face)} '
      'open=${MeshSeal.openBoundaryEdgeCount(face)} '
      'vol=${MeshEdit.signedVolume(face).abs().toStringAsFixed(2)}');

  void prism(String label, double alpha) {
    const side = 50.0;
    const height = 30.0;
    const radius = 6.0;
    final ring = [
      Vector3.zero(),
      Vector3(side, 0, 0),
      Vector3(side * math.cos(alpha), side * math.sin(alpha), 0),
    ];
    final body = MeshBuild.sweepPolygon(ring, Vector3(0, 0, height));
    final sw = Stopwatch()..start();
    try {
      final out = MeshFillet.fillet(
        body,
        [(Vector3.zero(), Vector3(0, 0, height))],
        radius: radius,
        inverse: false,
      );
      final removed = MeshEdit.signedVolume(body).abs() -
          MeshEdit.signedVolume(out).abs();
      print(
        'SMOKE $label ${sw.elapsedMilliseconds}ms '
        'closed=${MeshSeal.isClosedSolid(out)} '
        'open=${MeshSeal.openBoundaryEdgeCount(out)} '
        'removed=${removed.toStringAsFixed(1)} '
        '${out.vertices.length}v/${out.faces.length}f',
      );
    } catch (e) {
      print('SMOKE $label FAIL ${sw.elapsedMilliseconds}ms $e');
    }
  }

  prism('acute60', math.pi / 3);
  prism('obtuse120', 2 * math.pi / 3);

  void tryFillet(String label, MeshData mesh, List<(Vector3, Vector3)> edges,
      double r) {
    final sw = Stopwatch()..start();
    try {
      final complete = MeshFillet.completeCornerEdges(mesh, edges);
      final out = MeshFillet.fillet(mesh, edges, radius: r, inverse: false);
      print(
        'SMOKE $label ${sw.elapsedMilliseconds}ms '
        'picked=${edges.length} complete=${complete.length} '
        'closed=${MeshSeal.isClosedSolid(out)} '
        'open=${MeshSeal.openBoundaryEdgeCount(out)} '
        'vol=${MeshEdit.signedVolume(out).abs().toStringAsFixed(1)} '
        '${out.vertices.length}v/${out.faces.length}f',
      );
    } catch (e) {
      print('SMOKE $label FAIL ${sw.elapsedMilliseconds}ms $e');
    }
  }

  // Regular hex prism, same topology as the 12v/20f app body.
  final hexRing = [
    for (var i = 0; i < 6; i++)
      Vector3(10 * math.cos(i * math.pi / 3), 10 * math.sin(i * math.pi / 3), 0),
  ];
  final hex = MeshBuild.sweepPolygon(hexRing, Vector3(0, 0, 20));
  print('SMOKE hex ${hex.vertices.length}v/${hex.faces.length}f');
  final hexTop = [
    for (var i = 0; i < 6; i++)
      (hexRing[i] + Vector3(0, 0, 20), hexRing[(i + 1) % 6] + Vector3(0, 0, 20)),
  ];
  tryFillet('hex-face', hex, hexTop, 1.5);
  tryFillet('hex-upright', hex, [(hexRing[0], hexRing[0] + Vector3(0, 0, 20))], 1.5);

  // Cube ∪ sphere sitting on +X, fillet the opposite face (x=0).
  final cube = MeshPrimitives.cube(20, 20, 20);
  final sph = MeshPrimitives.sphere(
    center: Vector3(20, 10, 10),
    radius: 10,
  );
  final u = MeshBoolean.run(cube, sph, MeshBoolean.unionOp);
  print(
    'SMOKE union ${u.vertices.length}v/${u.faces.length}f '
    'closed=${MeshSeal.isClosedSolid(u)}',
  );
  final opposite = [
    (Vector3(0, 0, 0), Vector3(0, 20, 0)),
    (Vector3(0, 20, 0), Vector3(0, 20, 20)),
    (Vector3(0, 20, 20), Vector3(0, 0, 20)),
    (Vector3(0, 0, 20), Vector3(0, 0, 0)),
  ];
  tryFillet('union-face', u, opposite, 3);
  tryFillet('union-edge', u, [(Vector3(0, 0, 0), Vector3(0, 0, 20))], 3);

  // Smaller blob that does not touch the other four cube faces.
  final sph2 = MeshPrimitives.sphere(
    center: Vector3(20, 10, 10),
    radius: 6,
  );
  final u2 = MeshBoolean.run(cube, sph2, MeshBoolean.unionOp);
  print(
    'SMOKE union-small ${u2.vertices.length}v/${u2.faces.length}f '
    'closed=${MeshSeal.isClosedSolid(u2)}',
  );
  tryFillet('union-small-face', u2, opposite, 3);
  tryFillet('union-small-edge', u2, [(Vector3(0, 0, 0), Vector3(0, 0, 20))], 3);
}
