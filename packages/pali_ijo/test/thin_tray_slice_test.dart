import 'dart:convert';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';
import 'thin_tray_boolean_test.dart' as tray;

void expectSlice(BrepSolid body, Vector3 origin, Vector3 normal) {
  final before = jsonEncode(body.toJson());
  final parts = BrepSlice.parts(body, origin, normal);
  expect(parts, hasLength(2));
  var volume = 0.0;
  for (final p in parts) {
    tray.expectGeometry(p);
    final mesh = IjoOps.tessellateClosed(p);
    expect(MeshKit.isClosedSolid(mesh), isTrue);
    final directions = <(int, int), int>{};
    for (final face in mesh.faces) {
      for (var i = 0; i < face.length; i++) {
        final a = face[i], b = face[(i + 1) % face.length];
        final key = a < b ? (a, b) : (b, a);
        directions.update(
          key,
          (n) => n + (a < b ? 1 : -1),
          ifAbsent: () => a < b ? 1 : -1,
        );
      }
    }
    expect(
      directions.values.every((n) => n == 0),
      isTrue,
      reason:
          'Adjacent display triangles must use each shared edge in opposite directions',
    );
    expect(MeshKit.signedVolume(mesh), closeTo(IjoMeasure.volumeSolid(p), 35));
    expect(MeshKit.signedVolume(mesh), greaterThan(0));
    volume += IjoMeasure.volumeSolid(p);
  }
  expect(volume, closeTo(IjoMeasure.volumeSolid(body), .02));
  expect(jsonEncode(body.toJson()), before);
}

void main() {
  test('slice semicircle-cut joined tray at floor height', () {
    final joined = IjoOps.booleanSolid(
      tray.fixture('base'),
      tray.rebuiltWall(),
      IjoOps.unionOp,
    );
    final cut = IjoOps.booleanSolid(joined, tray.cutter(), IjoOps.subtractOp);
    final halves = IjoOps.splitBrep(cut);
    expect(halves, hasLength(2));
    for (final half in halves) {
      expectSlice(
        half,
        Vector3(-45.33196239011206, -27.442099090175855, 1),
        Vector3(-3.999234235647749e-17, 0, 1),
      );
    }
  });
  for (final height in [.5, .999, 1.0, 1.001, 5.0]) {
    test(
      'captured 139-face tray slice at z=$height retains volume and trims',
      () {
        final body = tray.fixture('slice_body');
        expect(body.faces, hasLength(139));
        expectSlice(
          body,
          Vector3(-45.33196239011206, -27.442099090175855, height),
          Vector3(-3.999234235647749e-17, 0, 1),
        );
      },
    );
  }
  test('captured tray coplanar slice with reversed normal', () {
    expectSlice(
      tray.fixture('slice_body'),
      Vector3(0, 0, 1),
      Vector3(0, 0, -1),
    );
  });
  test('captured tray coplanar slice after rigid rotation and translation', () {
    final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .71);
    Vector3 map(Vector3 p) => q.rotated(p) + Vector3(23, -17, 31);
    expectSlice(
      BrepXform.map(tray.fixture('slice_body'), map),
      map(Vector3(0, 0, 1)),
      q.rotated(Vector3(0, 0, 1)),
    );
  });
}
