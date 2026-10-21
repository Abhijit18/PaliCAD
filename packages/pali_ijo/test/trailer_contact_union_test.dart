import 'dart:convert';
import 'dart:io';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid fixture(String name) => BrepSolid.fromJson(
  jsonDecode(File('test/fixtures/trailer_union/$name.json').readAsStringSync())
      as Map<String, dynamic>,
);

void expectClosedContact(BrepSolid s, double volume) {
  expect(BrepHeal.topologyIsClosed(s), isTrue);
  expect(IjoOps.splitBrep(s), hasLength(1));
  final mesh = IjoOps.tessellateKeep(s);
  expect(MeshKit.openBoundaryEdgeCount(mesh), 0);
  expect(MeshKit.signedVolume(mesh), closeTo(volume, volume * 1e-8));
  expect(IjoMeasure.volumeSolid(s), closeTo(volume, volume * 1e-8));
  expect(mesh.isMeshSolid, isFalse);
  for (final f in s.faces) {
    for (final l in f.loops) {
      for (var i = 0; i < l.coedges.length; i++) {
        final co = l.coedges[i], next = l.coedges[(i + 1) % l.coedges.length];
        final e = s.edges[co.edge], en = s.edges[next.edge];
        expect(
          (s.vertices[co.reversed ? e.v0 : e.v1] -
                  s.vertices[next.reversed ? en.v1 : en.v0])
              .length,
          lessThan(1e-6),
        );
      }
    }
  }
}

void main() {
  for (final reverse in [false, true]) {
    for (final scale in [.01, 1.0]) {
      for (final rotated in [false, true]) {
        test(
          'saved trailer union reverse=$reverse scale=$scale rotated=$rotated',
          () {
            final q = Quaternion.axisAngle(Vector3(1, 2, 3).normalized(), .63);
            Vector3 place(Vector3 p) => rotated
                ? q.rotated(p * scale) + Vector3(123, -90, 42)
                : p * scale;
            final a = BrepXform.map(fixture('frame'), place);
            final b = BrepXform.map(fixture('tongue'), place);
            final before = jsonEncode([a.toJson(), b.toJson()]);
            final out = IjoOps.booleanSolid(
              reverse ? b : a,
              reverse ? a : b,
              IjoOps.unionOp,
            );
            // The inputs touch at z=0; their volumes do not overlap.
            final volume = 60675420.924767895 * scale * scale * scale;
            expectClosedContact(out, volume);
            final restored = BrepSolid.fromJson(
              jsonDecode(jsonEncode(out.toJson())) as Map<String, dynamic>,
            );
            expectClosedContact(restored, volume);
            for (final x in [
              -1030.0,
              -985.0,
              -500.0,
              0.0,
              500.0,
              985.0,
              1030.0,
            ]) {
              for (final y in [
                -2000.0,
                1100.0,
                1600.0,
                2110.0,
                2600.0,
                3300.0,
              ]) {
                for (final z in [-25.0, 25.0]) {
                  final p = place(Vector3(x, y, z));
                  expect(
                    BrepClassify.isInside(out, p),
                    BrepClassify.isInside(a, p) || BrepClassify.isInside(b, p),
                    reason: 'material at $x,$y,$z',
                  );
                }
              }
            }
            expect(jsonEncode([a.toJson(), b.toJson()]), before);
          },
        );
      }
    }
  }
  test('joined trailer remains editable after saving', () {
    final joined = IjoOps.booleanSolid(
      fixture('frame'),
      fixture('tongue'),
      IjoOps.unionOp,
    );
    final restored = BrepSolid.fromJson(
      jsonDecode(jsonEncode(joined.toJson())) as Map<String, dynamic>,
    );
    // Trim the tongue tip without removing the contacts at the frame.
    final cutter = BrepXform.map(
      BrepPrimitives.box(300, 200, 200),
      (p) => p + Vector3(-150, 3250, -100),
    );
    final cut = IjoOps.booleanSolid(restored, cutter, IjoOps.subtractOp);
    expect(
      IjoMeasure.volumeSolid(cut),
      lessThan(IjoMeasure.volumeSolid(restored)),
    );
    expectClosedContact(cut, IjoMeasure.volumeSolid(cut));
    expect(BrepClassify.isInside(cut, Vector3(0, 3320, -25)), isFalse);
  });
  test('planar edge contact can rejoin through a face elsewhere', () {
    final frame = BrepExtrude.profile([
      Vector3(0, 0, 0),
      Vector3(6, 0, 0),
      Vector3(6, 10, 0),
      Vector3(4, 10, 0),
      Vector3(4, 2, 0),
      Vector3(2, 2, 0),
      Vector3(2, 10, 0),
      Vector3(0, 10, 0),
    ], Vector3(0, 0, 2));
    final bar = BrepXform.map(
      BrepPrimitives.box(6, 2, 2),
      (p) => p + Vector3(0, 2, -2),
    );
    final joined = IjoOps.booleanSolid(frame, bar, IjoOps.unionOp);
    expectClosedContact(joined, 112);
    expect(BrepClassify.isInside(joined, Vector3(3, 8, 1)), isFalse);
    expect(BrepClassify.isInside(joined, Vector3(3, 3, -1)), isTrue);
    expect(BrepClassify.isInside(joined, Vector3(1, 3, 1)), isTrue);
    // The STL-style position-welded mesh genuinely contains a line contact.
    // Its stricter manifold check must not be weakened to accept this case.
    expect(MeshKit.isClosedSolid(BrepTessellate.run(joined)), isFalse);
  });
  test('contact handling still rejects a missing or flipped face', () {
    final joined = IjoOps.booleanSolid(
      fixture('frame'),
      fixture('tongue'),
      IjoOps.unionOp,
    );
    final missing = joined.clone()..faces.removeLast();
    expect(
      () => IjoOps.tessellateKeep(missing),
      throwsA(isA<BrepUnsupported>()),
    );
    final flipped = joined.clone();
    // Keep its closed edge graph, but invert one of the contact faces.
    final contactFace = flipped.faces.firstWhere(
      (f) => f.surface is BrepPlane && f.outwardNormal(0, 0).y > .9,
    );
    contactFace.reversed = !contactFace.reversed;
    expect(BrepHeal.topologyIsClosed(flipped), isTrue);
    expect(
      () => IjoOps.tessellateKeep(flipped),
      throwsA(isA<BrepUnsupported>()),
    );
    // Swapping one face from each coincident edge keeps the edge-use count
    // at two but connects the wrong material sectors. It must not pass.
    final edgeFaces = <int, List<int>>{};
    for (var fi = 0; fi < joined.faces.length; fi++) {
      for (final l in joined.faces[fi].loops) {
        for (final co in l.coedges) (edgeFaces[co.edge] ??= []).add(fi);
      }
    }
    final groups = <(int, int), List<int>>{};
    for (final id in edgeFaces.keys) {
      final e = joined.edges[id];
      final key = e.v0 < e.v1 ? (e.v0, e.v1) : (e.v1, e.v0);
      (groups[key] ??= []).add(id);
    }
    final contact = groups.values.firstWhere((ids) => ids.length == 2);
    final first = contact[0], second = contact[1];
    final swap = {
      edgeFaces[first]!.first: (first, second),
      edgeFaces[second]!.first: (second, first),
    };
    final wrongPairs = BrepSolid(
      vertices: joined.vertices,
      edges: joined.edges,
      faces: [
        for (var fi = 0; fi < joined.faces.length; fi++)
          BrepFace(
            surface: joined.faces[fi].surface,
            reversed: joined.faces[fi].reversed,
            loops: [
              for (final l in joined.faces[fi].loops)
                BrepLoop([
                  for (final co in l.coedges)
                    BrepCoedge(
                      swap[fi]?.$1 == co.edge ? swap[fi]!.$2 : co.edge,
                      reversed: co.reversed,
                    ),
                ]),
            ],
          ),
      ],
    );
    expect(BrepHeal.topologyIsClosed(wrongPairs), isTrue);
    expect(
      () => IjoOps.tessellateKeep(wrongPairs),
      throwsA(isA<BrepUnsupported>()),
    );
  });
}
