import 'dart:convert';
import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid filletedCylinder() {
  final s = BrepPrimitives.cylinder(
    origin: Vector3.zero(),
    axis: Vector3(0, 0, 1),
    radius: 20,
    height: 50,
  );
  return IjoOps.filletSolid(
    s,
    [
      for (var i = 0; i < s.edges.length; i++)
        if (s.edges[i].curve is BrepCircle) i,
    ],
    radius: 5,
    inverse: false,
  );
}

void main() {
  final timings = <int>[];
  tearDownAll(() {
    timings.sort();
    print(
      'Cap sequence boolean timings (${timings.length} operations): '
      'median ${timings[timings.length ~/ 2]} us; max ${timings.last} us.',
    );
  });
  for (final scale in [.1, 1.0, 10.0]) {
    for (final rotated in [false, true]) {
      for (final reverse in [false, true]) {
        for (final offset in [0.0, 1.0]) {
          test(
            'repeated caps scale=$scale rotated=$rotated reverse=$reverse offset=$offset',
            () {
              final q = Quaternion.axisAngle(
                Vector3(1, 2, 3).normalized(),
                .63,
              );
              Vector3 place(Vector3 p) =>
                  (rotated ? q.rotated(p) + Vector3(123, -90, 42) : p) * scale;
              var a = BrepXform.map(filletedCylinder(), place);
              var expected =
                  math.pi *
                  (400 * 40 +
                      2 * (225 * 5 + 30 * 25 * math.pi / 4 + 2 * 125 / 3));
              var z = 50.0;
              final steps = [
                (15.0, 0.0, 5.0),
                (17.5, offset, 8.0),
                (12.0, offset, 8.0),
                (12.0, offset, 8.0),
                (19.0, offset, 8.0),
                (8.0, offset, 8.0),
              ];
              for (final (radius, x, height) in steps) {
                final b = BrepXform.map(
                  BrepPrimitives.cylinder(
                    origin: Vector3(x, 0, z),
                    axis: Vector3(0, 0, 1),
                    radius: radius,
                    height: height,
                  ),
                  place,
                );
                final before = jsonEncode([a.toJson(), b.toJson()]);
                final watch = Stopwatch()..start();
                final result = IjoOps.booleanSolid(
                  reverse ? b : a,
                  reverse ? a : b,
                  IjoOps.unionOp,
                );
                timings.add(watch.elapsedMicroseconds);
                expect(jsonEncode([a.toJson(), b.toJson()]), before);
                expected += math.pi * radius * radius * height;
                final volume = expected * scale * scale * scale;
                expect(BrepHeal.topologyIsClosed(result), isTrue);
                expect(IjoOps.splitBrep(result), hasLength(1));
                // Test the raw tessellation, so mesh hole repair cannot hide a bad BREP.
                final mesh = BrepTessellate.run(result);
                expect(MeshKit.isClosedSolid(mesh), isTrue);
                expect(
                  MeshKit.signedVolume(mesh),
                  closeTo(volume, volume * .008),
                );
                expect(
                  IjoMeasure.volumeSolid(result),
                  closeTo(volume, math.max(1e-7, volume * 1e-9)),
                );
                expect(
                  result.faces.where((f) => f.surface is BrepTorus),
                  hasLength(2),
                );
                expect(
                  BrepClassify.isInside(
                    result,
                    place(Vector3(x + radius * .8, 0, z + height / 2)),
                  ),
                  isTrue,
                );
                expect(
                  BrepClassify.isInside(
                    result,
                    place(Vector3(x + radius + 1, 0, z + height / 2)),
                  ),
                  isFalse,
                );
                // Persist/reload between steps: no reliance on transient shape caches.
                a = BrepSolid.fromJson(jsonDecode(jsonEncode(result.toJson())));
                z += height;
              }
            },
          );
        }
      }
    }
  }
  test('partly overlapping round caps have a shared analytic boundary', () {
    final a = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 15,
      height: 20,
    );
    final b = BrepPrimitives.cylinder(
      origin: Vector3(8, 0, 20),
      axis: Vector3(0, 0, 1),
      radius: 17.5,
      height: 8,
    );
    final out = IjoOps.booleanSolid(a, b, IjoOps.unionOp);
    expect(BrepHeal.topologyIsClosed(out), isTrue);
    expect(MeshKit.isClosedSolid(BrepTessellate.run(out)), isTrue);
    expect(
      IjoMeasure.volumeSolid(out),
      closeTo(math.pi * (225 * 20 + 17.5 * 17.5 * 8), 1e-5),
    );
  });
}
