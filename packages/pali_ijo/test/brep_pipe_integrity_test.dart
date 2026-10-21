import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final path in <List<Vector3>>[
    [Vector3.zero(), Vector3(30, 0, 0), Vector3(30, 30, 0)],
    [Vector3.zero(), Vector3(40, 0, 0), Vector3(40, 40, 0), Vector3(0, 40, 0)],
    [
      Vector3.zero(),
      Vector3(30, 0, 0),
      Vector3(30, 30, 0),
      Vector3(30, 30, 30),
    ],
  ]) {
    final closed = path.length == 4 && path.last.z == 0;
    test(
      'pipe BREP and display agree for ${path.length} points closed=$closed',
      () {
        final out = IjoOps.pipePreferSolid(
          path,
          outerRadius: 3,
          closed: closed,
        );
        expect(out.solid, isNotNull);
        final solid = out.solid!;
        expect(BrepHeal.topologyIsClosed(solid), isTrue);
        final actual = BrepTessellate.run(solid);
        expect(MeshKit.isClosedSolid(actual), isTrue);
        expect(out.mesh.toJson(), actual.toJson());
        for (final f in solid.faces) {
          for (final loop in f.loops) {
            for (final co in loop.coedges) {
              final c = solid.edges[co.edge].curve;
              for (var i = 0; i <= 8; i++) {
                final p = c.evaluate(c.t0 + (c.t1 - c.t0) * i / 8);
                expect((f.surface.closestPoint(p) - p).length, lessThan(1e-6));
              }
            }
          }
        }
      },
    );
  }
  test('an unevenly sampled circular arc remains an exact torus sector', () {
    final path = [
      for (var i = 0; i <= 16; i++)
        () {
          final t = 1.7 * math.pi * math.pow(i / 16, 1.4);
          return Vector3(20 * math.cos(t), 20 * math.sin(t), 0);
        }(),
    ];
    final out = IjoOps.pipePreferSolid(path, outerRadius: 2);
    expect(out.solid, isNotNull);
    final mesh = BrepTessellate.run(out.solid!);
    expect(MeshKit.isClosedSolid(mesh), isTrue);
    final exact = math.pi * 4 * 20 * 1.7 * math.pi;
    expect(MeshKit.signedVolume(mesh), closeTo(exact, exact * .04));
  });
  test('hollow elbow retains an analytic inner and outer wall', () {
    final out = IjoOps.pipePreferSolid(
      [Vector3.zero(), Vector3(30, 0, 0), Vector3(30, 30, 0)],
      outerRadius: 4,
      innerRadius: 3,
    );
    expect(out.solid, isNotNull);
    expect(
      out.solid!.faces.any((f) => f.reversed && f.surface is BrepTorus),
      isTrue,
    );
    expect(MeshKit.isClosedSolid(BrepTessellate.run(out.solid!)), isTrue);
  });
  test('pipe rejects reversals, nonfinite inputs and impossible bends', () {
    for (final path in <List<Vector3>>[
      [Vector3.zero(), Vector3(10, 0, 0), Vector3.zero()],
      [Vector3.zero(), Vector3(1, 0, 0), Vector3(1, 1, 0)],
      [Vector3.zero(), Vector3(double.nan, 0, 0)],
    ]) {
      expect(
        () => IjoOps.pipePreferSolid(path, outerRadius: 3),
        throwsA(isA<BrepUnsupported>()),
      );
    }
  });
  test('closed pipe checks the closing corner for reversal', () {
    expect(
      () => BrepPipe.run(
        [
          Vector3.zero(),
          Vector3(30, 0, 0),
          Vector3(30, 30, 0),
          Vector3(60, 30, 0),
          Vector3(60, 0, 0),
        ],
        radius: 2,
        closed: true,
      ),
      throwsA(isA<BrepUnsupported>()),
    );
  });
}
