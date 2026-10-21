import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'thin_tray_boolean_test.dart' as tray;
import 'thin_tray_slice_test.dart' as slice;

BrepSolid fixture(String name) => BrepSolid.fromJson(
  jsonDecode(File('test/fixtures/cone_complex/$name.json').readAsStringSync())
      as Map<String, dynamic>,
);

BrepSolid cutter({double dz = 0}) => BrepXform.map(
  fixture('cone'),
  (p) =>
      Quaternion(1, 0, 0, 6.123233995736766e-17).rotated(p) +
      Vector3(-10, 9.999999999999986, 53.5875486381323 + dz),
);

void expectCountersink(BrepSolid body, BrepSolid cone, double dz) {
  final before = [jsonEncode(body.toJson()), jsonEncode(cone.toJson())];
  final out = IjoOps.booleanSolid(body, cone, IjoOps.subtractOp);
  tray.expectGeometry(out);
  expect(out.faces.where((f) => f.surface is BrepCone), hasLength(2));
  final mesh = IjoOps.tessellateClosed(out);
  expect(MeshKit.isClosedSolid(mesh), isTrue);
  expect(MeshKit.signedVolume(mesh), greaterThan(0));
  // The pre-existing bore removes the inner cylinder of this frustum.
  final h = 51 - (53.5875486381323 + dz - 5);
  const r0 = 7.5;
  final r1 = r0 + h / 2;
  final removed =
      math.pi * h / 3 * (r0 * r0 + r0 * r1 + r1 * r1) - math.pi * r0 * r0 * h;
  expect(
    IjoMeasure.volumeSolid(body) - IjoMeasure.volumeSolid(out),
    closeTo(removed, .02),
  );
  expect([jsonEncode(body.toJson()), jsonEncode(cone.toJson())], before);
}

void main() {
  for (final dz in [0.0, -.5, .5]) {
    test('captured complex cone subtraction at offset $dz', () {
      expectCountersink(fixture('body'), cutter(dz: dz), dz);
    });
  }
  test('captured cone subtraction after rigid transform', () {
    final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .71);
    Vector3 map(Vector3 p) => q.rotated(p) + Vector3(23, -17, 31);
    expectCountersink(
      BrepXform.map(fixture('body'), map),
      BrepXform.map(cutter(), map),
      0,
    );
  });
  for (final normal in [Vector3(0, -1, 0), Vector3(0, 1, 0)]) {
    test('captured complex body split normal $normal', () {
      slice.expectSlice(fixture('body'), Vector3.zero(), normal);
    });
  }
  test('captured split after rigid transform', () {
    final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .71);
    Vector3 map(Vector3 p) => q.rotated(p) + Vector3(23, -17, 31);
    slice.expectSlice(
      BrepXform.map(fixture('body'), map),
      map(Vector3.zero()),
      q.rotated(Vector3(0, -1, 0)),
    );
  });
}
