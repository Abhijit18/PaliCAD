import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';

BrepSolid patch(String name) {
  return IjoStep.readSolids(
    Uint8List.fromList(
      gzip.decode(
        File(
          'test/fixtures/step_import/rounded_${name}_patch.step.gz',
        ).readAsBytesSync(),
      ),
    ),
    allowOpenShells: true,
  ).solids.single;
}

void main() {
  for (final tess in [
    MeshTessellation.standard,
    MeshTessellation.importPreview,
  ]) {
    test(
      'rounded spherical STEP trim preserves its complete boundary (${tess.quality})',
      () {
        final solid = patch('sphere'), before = jsonEncode(solid.toJson());
        final mesh = BrepTessellate.run(solid, tessellation: tess);
        expect(mesh.faces, isNotEmpty);
        expect(mesh.faceIds!.toSet(), {0});
        final sphere = solid.faces.single.surface as BrepSphere;
        final uses = <String, int>{};
        for (final f in mesh.faces) {
          final a = mesh.vertices[f[0]],
              b = mesh.vertices[f[1]],
              c = mesh.vertices[f[2]];
          expect(
            (b - a).cross(c - a).dot((a + b + c) / 3 - sphere.center),
            greaterThan(1e-16),
          );
          for (var i = 0; i < 3; i++) {
            final key = MeshData.segmentKey(
              mesh.vertices[f[i]],
              mesh.vertices[f[(i + 1) % 3]],
            );
            uses[key] = (uses[key] ?? 0) + 1;
          }
        }
        final boundary = BrepTessellate.sampleLoop(
          solid,
          solid.faces.single.loops.single,
          tess.cylinderSegments,
          tess: tess,
        );
        final expected = {
          for (var i = 0; i < boundary.length; i++)
            MeshData.segmentKey(
              boundary[i],
              boundary[(i + 1) % boundary.length],
            ),
        };
        expect(
          uses.entries.where((e) => e.value == 1).map((e) => e.key).toSet(),
          expected,
        );
        expect(uses.values.every((n) => n == 1 || n == 2), isTrue);
        expect(mesh.vertices.length - uses.length + mesh.faces.length, 1);
        for (final p in mesh.vertices) {
          expect((p - sphere.center).length, closeTo(sphere.radius, 1e-9));
        }
        expect(jsonEncode(solid.toJson()), before);
      },
    );
  }
  for (final tess in [
    MeshTessellation.importPreview,
    MeshTessellation.standard,
    MeshTessellation.fine,
  ]) {
    test(
      'rounded cylindrical trim survives welded slivers (${tess.quality})',
      () {
        final solid = BrepSolid.fromJson(
          jsonDecode(
                File(
                  'test/fixtures/step_import/rounded_cylinder_patch.json',
                ).readAsStringSync(),
              )
              as Map<String, dynamic>,
        );
        final before = jsonEncode(solid.toJson());
        final mesh = BrepTessellate.run(solid, tessellation: tess);
        expect(mesh.faces, isNotEmpty);
        expect(mesh.faceIds!.toSet(), {0});
        final face = solid.faces.single;
        final cylinder = face.surface as BrepCylinder;
        for (final p in mesh.vertices) {
          expect(p.x.isFinite && p.y.isFinite && p.z.isFinite, isTrue);
          final uv = cylinder.parameters(p)!;
          expect((p - cylinder.evaluate(uv.$1, uv.$2)).length, lessThan(1e-7));
        }
        for (final f in mesh.faces) {
          final a = mesh.vertices[f[0]],
              b = mesh.vertices[f[1]],
              c = mesh.vertices[f[2]];
          final normal = (b - a).cross(c - a);
          expect(normal.length2, greaterThan(1e-28));
          final uv = cylinder.parameters((a + b + c) / 3)!;
          expect(normal.dot(face.outwardNormal(uv.$1, uv.$2)), greaterThan(0));
        }
        final uses = <(int, int), int>{};
        final usedVertices = <int>{};
        for (final triangle in mesh.faces) {
          usedVertices.addAll(triangle);
          for (var i = 0; i < 3; i++) {
            final a = triangle[i], b = triangle[(i + 1) % 3];
            final key = a < b ? (a, b) : (b, a);
            uses[key] = (uses[key] ?? 0) + 1;
          }
        }
        expect(uses.values.every((count) => count == 1 || count == 2), isTrue);
        expect(
          usedVertices.length - uses.length + mesh.faces.length,
          1,
          reason: 'The single trimmed patch must remain a disk without cracks',
        );
        // Welding may merge sub-resolution samples, but must retain the trim.
        for (final loop in face.loops) {
          for (final p in BrepTessellate.sampleLoop(
            solid,
            loop,
            tess.cylinderSegments,
            tess: tess,
          )) {
            expect(
              mesh.vertices.any((v) => (v - p).length < 1e-7),
              isTrue,
              reason: 'Every trim sample must remain represented',
            );
          }
        }
        expect(jsonEncode(solid.toJson()), before);
      },
    );
  }

  test('STEP spline patch survives parameter welding and edge refinement', () {
    final solid = patch('spline'), before = jsonEncode(solid.toJson());
    final mesh = BrepTessellate.run(
      solid,
      tessellation: MeshTessellation.importPreview,
    );
    expect(mesh.faces, isNotEmpty);
    expect(mesh.faceIds!.toSet(), {0});
    for (final p in mesh.vertices) {
      expect(p.x.isFinite && p.y.isFinite && p.z.isFinite, isTrue);
    }
    for (final f in mesh.faces) {
      expect(f.toSet(), hasLength(3));
      final a = mesh.vertices[f[0]],
          b = mesh.vertices[f[1]],
          c = mesh.vertices[f[2]];
      expect((b - a).cross(c - a).length2, greaterThan(1e-28));
    }
    expect(jsonEncode(solid.toJson()), before);
  });
}
