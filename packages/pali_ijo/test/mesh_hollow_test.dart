import 'dart:math' as math;

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  group('MeshHollow', () {
    test('box with one opening stays closed', () {
      final box = MeshPrimitives.cube(10, 10, 10);
      final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(box);
      expect(top, isNotEmpty);
      final shell = MeshHollow.run(
        box,
        openingFaces: top,
        openingNormal: Vector3(0, 0, 1),
        wallThicknessMm: 1,
      );
      expect(MeshKit.isClosedSolid(shell), isTrue);
      final vol = MeshEdit.signedVolume(shell).abs();
      expect(vol, greaterThan(100));
      expect(vol, lessThan(1000));
    });

    test('hexagonal extrude hollows', () {
      final ring = [
        for (var i = 0; i < 6; i++)
          Vector3(
            10 * math.cos(i * math.pi / 3),
            10 * math.sin(i * math.pi / 3),
            0,
          ),
      ];
      final hex = MeshBuild.sweepPolygon(ring, Vector3(0, 0, 20));
      expect(MeshKit.isClosedSolid(hex), isTrue);
      final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(hex);
      expect(top, isNotEmpty);
      final shell = MeshHollow.run(
        hex,
        openingFaces: top,
        openingNormal: Vector3(0, 0, 1),
        wallThicknessMm: 1.5,
      );
      expect(MeshKit.isClosedSolid(shell), isTrue);
      final outer = MeshEdit.signedVolume(hex).abs();
      final vol = MeshEdit.signedVolume(shell).abs();
      expect(vol, lessThan(outer * 0.9));
      expect(vol, greaterThan(outer * 0.2));
    });

    test('cylinder hollows', () {
      final cyl = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 10,
        height: 20,
        segments: 24,
      );
      final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(cyl);
      expect(top, isNotEmpty);
      final shell = MeshHollow.run(
        cyl,
        openingFaces: top,
        openingNormal: Vector3(0, 0, 1),
        wallThicknessMm: 2,
      );
      expect(MeshKit.isClosedSolid(shell), isTrue);
      final outer = MeshEdit.signedVolume(cyl).abs();
      final vol = MeshEdit.signedVolume(shell).abs();
      expect(vol, lessThan(outer * 0.9));
      expect(vol, greaterThan(outer * 0.2));
    });

    test('rotated box hollows', () {
      final box = MeshPrimitives.cube(10, 10, 10);
      const a = math.pi / 6;
      final ca = math.cos(a);
      final sa = math.sin(a);
      final rot = MeshXform.transformVertices(box, (v) {
        final d = v - Vector3(5, 5, 5);
        return Vector3(5 + d.x * ca - d.y * sa, 5 + d.x * sa + d.y * ca, v.z);
      });
      final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(rot);
      expect(top, isNotEmpty);
      final shell = MeshHollow.run(
        rot,
        openingFaces: top,
        openingNormal: Vector3(0, 0, 1),
        wallThicknessMm: 1,
      );
      expect(MeshKit.isClosedSolid(shell), isTrue);
      final vol = MeshEdit.signedVolume(shell).abs();
      expect(vol, lessThan(900));
      expect(vol, greaterThan(200));
    });

    test('collinear-point rectangle extrude uses the AABB inset', () {
      final ring = [
        Vector3(0, 0, 0),
        Vector3(20, 0, 0),
        Vector3(40, 0, 0),
        Vector3(40, 15, 0),
        Vector3(40, 30, 0),
        Vector3(20, 30, 0),
        Vector3(0, 30, 0),
        Vector3(0, 15, 0),
      ];
      final body = MeshBuild.sweepPolygon(ring, Vector3(0, 0, 20));
      final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(body);
      final shell = MeshHollow.run(
        body,
        openingFaces: top,
        openingNormal: Vector3(0, 0, 1),
        wallThicknessMm: 2,
      );
      expect(MeshKit.isClosedSolid(shell), isTrue);
      final outer = MeshEdit.signedVolume(body).abs();
      final vol = MeshEdit.signedVolume(shell).abs();
      expect(vol, lessThan(outer * 0.85));
      expect(vol, greaterThan(outer * 0.2));
    });

    test('L-shaped extrude hollows', () {
      final ring = [
        Vector3(0, 0, 0),
        Vector3(30, 0, 0),
        Vector3(30, 10, 0),
        Vector3(10, 10, 0),
        Vector3(10, 30, 0),
        Vector3(0, 30, 0),
      ];
      final body = MeshBuild.sweepPolygon(ring, Vector3(0, 0, 16));
      final top = FaceSelect.extreme(Vector3(0, 0, 1)).resolve(body);
      final shell = MeshHollow.run(
        body,
        openingFaces: top,
        openingNormal: Vector3(0, 0, 1),
        wallThicknessMm: 2,
      );
      expect(MeshKit.isClosedSolid(shell), isTrue);
      final outer = MeshEdit.signedVolume(body).abs();
      final vol = MeshEdit.signedVolume(shell).abs();
      expect(vol, lessThan(outer * 0.85));
      expect(vol, greaterThan(outer * 0.15));
    });
  });
}

