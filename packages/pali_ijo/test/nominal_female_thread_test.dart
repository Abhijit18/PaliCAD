import 'package:test/test.dart';
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  for (final blind in [false, true]) {
    test('nominal M5 rotated ${blind ? "blind bore" : "tube"}', () {
      final axis = Vector3(1, 2, 3).normalized(), origin = Vector3(10, -5, 8);
      final body = BrepBoolean.run(
        BrepPrimitives.cylinder(
          origin: origin,
          axis: axis,
          radius: 5,
          height: 8,
        ),
        BrepPrimitives.cylinder(
          origin: origin - axis,
          axis: axis,
          radius: 2.5,
          height: blind ? 7 : 10,
        ),
        BrepBoolean.subtractOp,
      );
      final result = BrepThreads.apply(
        body,
        origin: origin,
        axis: axis,
        radius: 2.5 - ThreadForm(pitch: .8, internal: true).depth,
        pitch: .8,
        axisZMin: 0,
        axisZMax: blind ? 6 : 8,
        length: 2,
        internal: true,
        fromEndA: true,
        fromEndB: true,
      );
      expect(BrepHeal.topologyIsClosed(result), isTrue);
      final mesh = IjoOps.tessellateKeep(result);
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(
        IjoMeasure.volume(mesh),
        greaterThan(IjoMeasure.volume(IjoOps.tessellateKeep(body))),
      );
      if (blind)
        expect(BrepClassify.side(result, origin + axis * 7), BrepSide.inside);
    });
  }
  test(
    'unsupported nominal thread does not silently subtract an empty groove',
    () {
      final axis = Vector3(0, 0, 1), origin = Vector3.zero();
      final body = BrepBoolean.run(
        BrepPrimitives.cylinder(
          origin: origin,
          axis: axis,
          radius: 5,
          height: 8,
        ),
        BrepPrimitives.cylinder(
          origin: origin - axis,
          axis: axis,
          radius: 2.5,
          height: 10,
        ),
        BrepBoolean.subtractOp,
      );
      expect(
        () => BrepThreads.apply(
          body,
          origin: origin,
          axis: axis,
          radius: 2.5 - ThreadForm(pitch: .8, internal: true).depth,
          pitch: .8,
          axisZMin: 0,
          axisZMax: 8,
          length: 8,
          internal: true,
          runoutLength: .5,
        ),
        throwsA(isA<BrepUnsupported>()),
      );
    },
  );
  for (final (diameter, pitch) in [(3.0, .5), (5.0, .8), (10.0, 1.5)]) {
    for (final ends in ['full', 'a', 'b', 'both']) {
      for (final right in [true, false]) {
        test('nominal M$diameter ends=$ends right=$right', () {
          final r = diameter / 2, height = 8 * pitch;
          final axis = Vector3(0, 0, 1),
              origin = Vector3(diameter, diameter, 0);
          final body = BrepBoolean.run(
            BrepPrimitives.box(2 * diameter, 2 * diameter, height),
            BrepPrimitives.cylinder(
              origin: origin - axis,
              axis: axis,
              radius: r,
              height: height + 2,
            ),
            BrepBoolean.subtractOp,
          );
          final before = body.toJson();
          final crest = r - ThreadForm(pitch: pitch, internal: true).depth;
          final result = BrepThreads.apply(
            body,
            origin: origin,
            axis: axis,
            radius: crest,
            pitch: pitch,
            axisZMin: 0,
            axisZMax: height,
            length: ends == 'full' ? height : 2 * pitch,
            internal: true,
            rightHand: right,
            starts: right ? 1 : 2,
            fromEndA: ends != 'b',
            fromEndB: ends == 'b' || ends == 'both',
          );
          expect(body.toJson(), before);
          expect(BrepHeal.topologyIsClosed(result), isTrue);
          final copy = BrepSolid.fromJson(result.toJson());
          for (final quality in [
            MeshTessellation.draft,
            MeshTessellation.fine,
          ]) {
            final mesh = BrepTessellate.run(copy, tessellation: quality);
            expect(MeshKit.isClosedSolid(mesh), isTrue);
            expect(MeshKit.signedVolume(mesh), greaterThan(0));
            final originalVolume = IjoMeasure.volume(
              BrepTessellate.run(body, tessellation: quality),
            );
            expect(IjoMeasure.volume(mesh), greaterThan(originalVolume));
            expect(
              IjoMeasure.volume(mesh),
              lessThan(4 * diameter * diameter * height),
            );
          }
          // Exact helical radii must span the requested minor/major diameters.
          final radii =
              result.edges
                  .map((e) => e.curve)
                  .whereType<BrepHelix>()
                  .map((c) => c.radius)
                  .toList()
                ..sort();
          expect(radii.first, closeTo(crest, 1e-7));
          expect(radii.last, closeTo(r, 1e-7));
          if (ends != 'full') {
            // Partial threads must leave the middle at the original bore size.
            expect(
              BrepClassify.side(
                copy,
                origin + axis * height / 2 + Vector3(r - .01, 0, 0),
              ),
              BrepSide.outside,
            );
          }
        });
      }
    }
  }
}
