import 'package:test/test.dart';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('square is not treated as a circular rim', () {
    final a = Vector3(0, 0, 10);
    final b = Vector3(10, 0, 10);
    final c = Vector3(10, 10, 10);
    final d = Vector3(0, 10, 10);
    final edges = <(Vector3, Vector3)>[(a, b), (b, c), (c, d), (d, a)];
    expect(CircularRim.detect(edges), isNull);
  });

  test('16-gon cylinder rim fits a circle', () {
    const segs = 16;
    final cyl = MeshKit.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 10,
      height: 20,
      segments: segs,
    );
    final edges = <(Vector3, Vector3)>[
      for (var i = 0; i < segs; i++)
        (cyl.vertices[segs + i], cyl.vertices[segs + ((i + 1) % segs)]),
    ];
    final rim = CircularRim.detect(edges);
    expect(rim, isNotNull);
    expect(rim!.loop.length, segs);
    expect(rim.radius, closeTo(10, 0.05));
  });
}
