import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('decodeAll preserves every analytic STEP solid and its name', () {
    final bytes = IjoStep.writeAll([
      StepPart.solid(BrepPrimitives.box(10, 12, 14), name: 'Housing'),
      StepPart.solid(
        BrepPrimitives.cylinder(
          origin: Vector3(20, 0, 0),
          axis: Vector3(0, 0, 1),
          radius: 3,
          height: 8,
        ),
        name: 'Pin',
      ),
    ]);

    final parts = IjoIo.decodeAll(bytes, name: 'assembly.step');

    expect(parts, hasLength(2));
    expect(parts.map((part) => part.name), ['Housing', 'Pin']);
    expect(parts.every((part) => part.kind == IjoDocKind.solid), isTrue);
    expect(IjoMeasure.volumeSolid(parts[0].solid), closeTo(1680, 1e-6));
    expect(IjoMeasure.volumeSolid(parts[1].solid), closeTo(72 * 3.141592653589793, 1e-5));
  });

  test('decodeAll imports every faceted STEP body as a solid', () {
    final bytes = IjoStep.writeAll([
      StepPart(MeshPrimitives.cube(10, 10, 10), name: 'Block A'),
      StepPart(MeshPrimitives.cube(4, 5, 6), name: 'Block B'),
    ]);

    final parts = IjoIo.decodeAll(bytes, name: 'parts.step');

    expect(parts, hasLength(2));
    expect(parts.map((part) => part.name), ['Block A', 'Block B']);
    expect(parts.every((part) => part.kind == IjoDocKind.solid), isTrue);
    expect(IjoMeasure.volumeSolid(parts[0].solid), closeTo(1000, 1e-6));
    expect(IjoMeasure.volumeSolid(parts[1].solid), closeTo(120, 1e-6));
  });
}
