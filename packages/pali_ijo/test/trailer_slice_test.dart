import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid fixture() => BrepSolid.fromJson(
  jsonDecode(
        File(
          'test/fixtures/trailer_union/extended_frame.json',
        ).readAsStringSync(),
      )
      as Map<String, dynamic>,
);

void checkParts(List<BrepSolid> parts, double total) {
  expect(parts.length, greaterThanOrEqualTo(2));
  var volume = 0.0;
  for (final part in parts) {
    expect(BrepHeal.topologyIsClosed(part), isTrue);
    final mesh = IjoOps.tessellateKeep(part);
    expect(MeshKit.isClosedSolid(mesh), isTrue);
    expect(MeshKit.signedVolume(mesh), greaterThan(0));
    expect(
      IjoMeasure.volumeSolid(part),
      closeTo(MeshKit.signedVolume(mesh), .01),
    );
    volume += IjoMeasure.volumeSolid(part);
  }
  expect(volume, closeTo(total, .01));
}

// Clip the original tongue's bottom outline in XY, independently of BREP
// Boolean classification. The protruding tip is a constant 50.8 mm extrusion.
double tipVolume(BrepSolid s, double x, double side) {
  final bottom = s.faces.firstWhere(
    (f) =>
        f.surface is BrepPlane &&
        f.outwardNormal(0, 0).z < -.9 &&
        ((f.surface as BrepPlane).origin.z + 50.8).abs() < 1e-6,
  );
  final pts = [
    for (final co in bottom.loops.single.coedges)
      s.vertices[co.reversed ? s.edges[co.edge].v1 : s.edges[co.edge].v0],
  ];
  final clipped = <Vector3>[];
  for (var i = 0; i < pts.length; i++) {
    final a = pts[i], b = pts[(i + 1) % pts.length];
    final da = (a.x - x) * side, db = (b.x - x) * side;
    if (da >= 0) clipped.add(a);
    if ((da < 0) != (db < 0)) clipped.add(a + (b - a) * (da / (da - db)));
  }
  var area = 0.0;
  for (var i = 0; i < clipped.length; i++) {
    final a = clipped[i], b = clipped[(i + 1) % clipped.length];
    area += a.x * b.y - b.x * a.y;
  }
  return area.abs() / 2 * 50.8;
}

void main() {
  for (final side in [-1.0, 1.0]) {
    for (final reversed in [false, true]) {
      for (final rotated in [false, true]) {
        test(
          'extended trailer slice side=$side reversed=$reversed rotated=$rotated',
          () {
            final source = fixture();
            final q = Quaternion.axisAngle(
              Vector3(1, 2, 3).normalized(),
              rotated ? .73 : 0,
            );
            Vector3 place(Vector3 p) => q.rotated(p) + Vector3(132, -47, 23);
            final input = BrepXform.map(source, place);
            final before = jsonEncode(input.toJson());
            final origin = place(Vector3(side * 1066.8, 0, 25.4));
            final normal = q.rotated(Vector3(reversed ? -1 : 1, 0, 0));
            final parts = BrepSlice.parts(input, origin, normal);
            expect(parts, hasLength(2));
            checkParts(parts, IjoMeasure.volumeSolid(source));
            final volumes = parts.map(IjoMeasure.volumeSolid).toList()..sort();
            expect(
              volumes.first,
              closeTo(tipVolume(source, side * 1066.8, side), .01),
            );
            for (final x in [
              -1100.0,
              -1030.0,
              -500.0,
              0.0,
              500.0,
              1030.0,
              1100.0,
            ]) {
              for (final y in [800.0, 1000.0, 1600.0, 2110.0, 2600.0]) {
                for (final z in [-25.0, 25.0]) {
                  final p = place(Vector3(x, y, z));
                  expect(
                    parts.any((s) => BrepClassify.isInside(s, p)),
                    BrepClassify.isInside(input, p),
                    reason: 'material $x,$y,$z',
                  );
                }
              }
            }
            for (final part in parts) {
              final saved = BrepSolid.fromJson(
                jsonDecode(jsonEncode(part.toJson())) as Map<String, dynamic>,
              );
              expect(
                MeshKit.isClosedSolid(IjoOps.tessellateKeep(saved)),
                isTrue,
              );
            }
            expect(jsonEncode(input.toJson()), before);
          },
        );
      }
    }
  }
  test('slice both trailer sides and continue cutting saved main body', () {
    final source = fixture();
    final right = BrepSlice.parts(
      source,
      Vector3(1066.8, 0, 0),
      Vector3(1, 0, 0),
    );
    right.sort(
      (a, b) => IjoMeasure.volumeSolid(b).compareTo(IjoMeasure.volumeSolid(a)),
    );
    final left = BrepSlice.parts(
      right.first,
      Vector3(-1066.8, 0, 0),
      Vector3(1, 0, 0),
    );
    checkParts([...left, ...right.skip(1)], IjoMeasure.volumeSolid(source));
    left.sort(
      (a, b) => IjoMeasure.volumeSolid(b).compareTo(IjoMeasure.volumeSolid(a)),
    );
    final saved = BrepSolid.fromJson(
      jsonDecode(jsonEncode(left.first.toJson())) as Map<String, dynamic>,
    );
    final cut = BrepSlice.parts(saved, Vector3(0, 0, 0), Vector3(0, 1, 0));
    checkParts(cut, IjoMeasure.volumeSolid(saved));
  });
}
