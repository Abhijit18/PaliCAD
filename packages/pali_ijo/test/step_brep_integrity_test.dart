import 'dart:convert';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  test('cone STEP preserves its degenerate apex and closed shell', () {
    final source = BrepPrimitives.cone(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 3,
      height: 7,
    );
    final back = IjoStep.readSolid(IjoStep.writeSolid(source));
    expect(BrepHeal.topologyIsClosed(back), isTrue);
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(back)), isTrue);
    expect(IjoMeasure.volumeSolid(back), closeTo(21 * math.pi, 1e-5));
  });
  test('STEP refuses a solid containing an unsupported surface', () {
    final bytes = IjoStep.writeSolid(
      BrepPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 3,
        height: 7,
      ),
    );
    final damaged = utf8.encode(
      utf8
          .decode(bytes)
          .replaceAll('CYLINDRICAL_SURFACE(', 'UNSUPPORTED_SURFACE('),
    );
    expect(
      () => IjoStep.readSolid(damaged),
      throwsA(isA<StepBrepUnsupportedException>()),
    );
  });
  test('ellipsoid STEP stores rational surfaces and imports a closed BREP', () {
    final source = BrepXform.scaleAbout(
      BrepPrimitives.sphere(center: Vector3(3, 2, 1), radius: 4),
      Vector3.zero(),
      Vector3(2, 1, .5),
    );
    final bytes = IjoStep.writeSolid(source);
    expect(utf8.decode(bytes), contains('RATIONAL_B_SPLINE_SURFACE'));
    final back = IjoStep.readSolid(bytes);
    expect(BrepHeal.topologyIsClosed(back), isTrue);
    expect(back.faces.single.surface, isA<BrepAffineSurface>());
    expect(
      IjoOps.sliceBrep(back, Vector3(6, 2, .5), Vector3(1, 0, 0)).length,
      2,
    );
    final mesh = IjoOps.tessellateClosed(back);
    expect(MeshKit.signedVolume(mesh), closeTo(4 * math.pi * 64 / 3, 8));
    expect(
      (IjoMeasure.ofSolid(back).centroid - Vector3(6, 2, .5)).length,
      lessThan(1e-5),
    );
  });
  test('STEP refuses unreadable bounds and missing cavity shells', () {
    final cavity = IjoOps.hollowSolid(
      BrepPrimitives.box(10, 10, 10),
      [],
      Vector3(0, 0, 1),
      wallThicknessMm: 1,
    );
    final text = utf8.decode(IjoStep.writeSolid(cavity));
    final voidShell = RegExp(
      r"ORIENTED_CLOSED_SHELL\('',\*,#(\d+),\.F\.\)",
    ).firstMatch(text)!.group(1)!;
    for (final damaged in [
      text.replaceFirst('FACE_OUTER_BOUND(', 'UNSUPPORTED_BOUND('),
      text.replaceFirst(
        '#$voidShell = CLOSED_SHELL(',
        '#$voidShell = MISSING_SHELL(',
      ),
    ]) {
      expect(damaged, isNot(text));
      expect(
        () => IjoStep.readSolid(utf8.encode(damaged)),
        throwsA(isA<StepBrepUnsupportedException>()),
      );
    }
  });
  test('faceted STEP imports shared planar BREP edges', () {
    final bytes = IjoStep.write(MeshPrimitives.cube(10, 10, 10));
    final back = IjoStep.readSolid(bytes);
    expect(back.faces.every((f) => f.surface is BrepPlane), isTrue);
    expect(BrepHeal.topologyIsClosed(back), isTrue);
    expect(IjoMeasure.volumeSolid(back), closeTo(1000, 1e-6));
  });
  test('ruled loft STEP preserves bounded curved surfaces', () {
    final a = [
      Vector3(-5, -5, 0),
      Vector3(5, -5, 0),
      Vector3(5, 5, 0),
      Vector3(-5, 5, 0),
    ];
    final b = [
      for (var i = 0; i < 32; i++)
        Vector3(
          3 * math.cos(2 * math.pi * i / 32),
          3 * math.sin(2 * math.pi * i / 32),
          10,
        ),
    ];
    final solid = BrepLoft.sections([a, b]);
    final bytes = IjoStep.writeSolid(solid), back = IjoStep.readSolid(bytes);
    expect(back.faces.any((f) => f.surface is BrepSplineSurface), isTrue);
    expect(BrepHeal.topologyIsClosed(back), isTrue);
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(back)), isTrue);
    final pieces = IjoOps.sliceBrep(back, Vector3(0, 0, 5), Vector3(0, 0, 1));
    expect(pieces.length, 2);
    for (final piece in pieces)
      expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(piece)), isTrue);
  });

  test('sealed cavity STEP keeps its void and orientation', () {
    final source = IjoOps.hollowSolid(
      BrepPrimitives.box(10, 10, 10),
      [],
      Vector3(0, 0, 1),
      wallThicknessMm: 1,
    );
    final bytes = IjoStep.writeSolid(source);
    expect(utf8.decode(bytes), contains('BREP_WITH_VOIDS'));
    final back = IjoStep.readSolid(bytes);
    expect(BrepHeal.topologyIsClosed(back), isTrue);
    expect(IjoMeasure.volumeSolid(back), closeTo(488, 1e-6));
    expect(BrepClassify.side(back, Vector3(5, 5, 5)), BrepSide.outside);
  });
  test('stretched cylinder exports curved walls and imports closed', () {
    final c = BrepPrimitives.cylinder(
      origin: Vector3.zero(),
      axis: Vector3(0, 0, 1),
      radius: 3,
      height: 10,
    );
    final source = BrepXform.scaleAbout(c, Vector3.zero(), Vector3(2, 1, 1));
    final back = IjoStep.readSolid(IjoStep.writeSolid(source));
    expect(BrepHeal.topologyIsClosed(back), isTrue);
    expect(MeshKit.isClosedSolid(IjoOps.tessellateClosed(back)), isTrue);
  });
}
