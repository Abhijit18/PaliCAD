import 'package:test/test.dart';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('invalid thread requests reject before altering the source', () {
    final source = BrepPrimitives.box(10, 10, 10), before = source.toJson();
    for (final starts in [0, 1000000000]) {
      expect(
        () => BrepThreads.apply(
          source,
          origin: Vector3.zero(),
          axis: Vector3(0, 0, 1),
          radius: 3,
          pitch: 1.5,
          axisZMin: 0,
          axisZMax: 10,
          length: 3,
          starts: starts,
        ),
        throwsA(anyOf(isA<ArgumentError>(), isA<BrepUnsupported>())),
      );
    }
    expect(source.toJson(), before);
  });
  test('pitch and runout inputs are finite and physically bounded', () {
    for (final pitch in [0.0, -1.0, double.nan, double.infinity]) {
      expect(
        () => ThreadForm(pitch: pitch, internal: false),
        throwsArgumentError,
      );
    }
    expect(
      () => BrepThreads.cutter(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 3,
        pitch: 1.5,
        length: 3,
        runoutLength: 2,
      ),
      throwsArgumentError,
    );
  });
}
