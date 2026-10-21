import 'dart:convert';
import 'dart:typed_data';

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  group('solid JSON', () {
    test('a solid survives the round trip as the same mesh', () {
      final solid = MeshPrimitives.cube(10, 20, 30);
      final back = IjoDocument.decode(
        IjoDocument.mesh(solid, name: 'plate').encode(),
      );

      expect(back.kind, IjoDocKind.mesh);
      expect(back.name, 'plate');

      final a = back.mesh;
      expect(a.vertices.length, solid.vertices.length);
      expect(a.faces.length, solid.faces.length);
      expect(a.faceIds, solid.faceIds);
      expect(IjoMeasure.volume(a), closeTo(IjoMeasure.volume(solid), 1e-9));
    });

    test('a faceted cylinder comes back with the same volume', () {
      final solid = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 4,
        height: 12,
      );
      final back = IjoDocument.decode(IjoDocument.mesh(solid).encode()).mesh;
      expect(back.faces.length, solid.faces.length);
      expect(IjoMeasure.volume(back), closeTo(IjoMeasure.volume(solid), 1e-9));
    });

    test('face ids survive the round trip', () {
      final solid = MeshPrimitives.cube(6, 6, 6);
      final back = IjoDocument.decode(IjoDocument.mesh(solid).encode()).mesh;
      expect(back.faceIds, solid.faceIds);
      expect(FaceSelect.all.resolve(back), hasLength(6));
    });

    test('legacy origin watermarks are ignored', () {
      final solid = MeshPrimitives.cube(2, 2, 2);
      final json = IjoDocument.mesh(solid).toJson();
      json['originWatermark'] = 900000;
      final back = IjoDocument.fromJson(json);
      expect(back.mesh.faces, hasLength(12));
    });

    test('kind is always mesh', () {
      final mesh = MeshPrimitives.cube(1, 1, 1);
      final back = IjoDocument.decode(IjoDocument.mesh(mesh).encode());
      expect(back.kind, IjoDocKind.mesh);
      expect(back.solidOrNull, isNull);
      expect(back.mesh.vertices, hasLength(mesh.vertices.length));
      expect(back.mesh.faces, hasLength(12));
    });

    test('a solid document already holds its mesh', () {
      final doc = IjoDocument.mesh(MeshPrimitives.cube(4, 4, 4));
      expect(doc.meshOrNull, isNotNull);
      expect(doc.mesh.faces, hasLength(12));
    });

    test('a future format version is refused, not guessed at', () {
      final json = IjoDocument.mesh(MeshPrimitives.cube(1, 1, 1)).toJson();
      json['pali_ijo'] = IjoDocument.formatVersion + 1;
      expect(
        () => IjoDocument.fromJson(json),
        throwsA(
          isA<FormatException>().having(
            (e) => e.message,
            'message',
            contains('Upgrade pali ijo'),
          ),
        ),
      );
    });

    test('arbitrary JSON is refused', () {
      expect(
        () => IjoDocument.decode('{"hello": "world"}'),
        throwsA(
          isA<FormatException>().having(
            (e) => e.message,
            'message',
            contains('Not a pali ijo document'),
          ),
        ),
      );
      expect(() => IjoDocument.decode('not json'), throwsFormatException);
      expect(() => IjoDocument.decode('[1, 2]'), throwsFormatException);
    });
  });

  group('STL', () {
    test('binary round trips the geometry', () {
      final mesh = MeshPrimitives.cube(10, 10, 10);
      final back = IjoStl.read(IjoStl.write(mesh));

      // The box tessellates to 12 triangles and, after welding, back to its
      // 8 corners — which is the point of welding on read.
      expect(back.faces, hasLength(12));
      expect(back.vertices, hasLength(8));
      expect(MeshKit.signedVolume(back), closeTo(1000, 1e-3));
    });

    test('ASCII round trips the geometry', () {
      final mesh = MeshPrimitives.cube(3, 4, 5);
      final text = IjoStl.writeAscii(mesh, name: 'brick');
      expect(text, startsWith('solid brick'));
      expect(text.trimRight(), endsWith('endsolid brick'));

      final back = IjoStl.read(Uint8List.fromList(utf8.encode(text)));
      expect(back.vertices, hasLength(8));
      expect(MeshKit.signedVolume(back), closeTo(60, 1e-3));
    });

    test('the binary header does not start with solid', () {
      // A binary file whose first five bytes spell "solid" is the classic way
      // to make an STL that half the world's readers get wrong.
      final bytes = IjoStl.write(MeshPrimitives.cube(1, 1, 1));
      expect(String.fromCharCodes(bytes.take(5)), isNot('solid'));
    });

    test('the flavour is sniffed, not trusted to the extension', () {
      final mesh = MeshPrimitives.cube(2, 2, 2);
      final ascii = Uint8List.fromList(utf8.encode(IjoStl.writeAscii(mesh)));

      // Declaring the format wrong should not matter: read() looks.
      final viaBinaryName = IjoIo.decode(ascii, format: IjoFormat.stl).mesh;
      expect(MeshKit.signedVolume(viaBinaryName), closeTo(8, 1e-3));
    });

    test('welding can be turned off', () {
      final mesh = MeshPrimitives.cube(1, 1, 1);
      final loose = IjoStl.read(IjoStl.write(mesh), weldTolerance: 0);
      // Three unshared corners per triangle is what the file literally says.
      expect(loose.vertices, hasLength(loose.faces.length * 3));
    });

    test('welding merges only what is within tolerance', () {
      final mesh = MeshData(
        vertices: [
          Vector3(0, 0, 0),
          Vector3(1, 0, 0),
          Vector3(0, 1, 0),
          // A hair away from the first vertex, and a clear millimetre away.
          Vector3(1e-9, 0, 0),
          Vector3(0, 0, 1),
          Vector3(1, 0, 1),
        ],
        faces: [
          [0, 1, 2],
          [3, 4, 5],
        ],
      );
      final back = IjoStl.read(IjoStl.write(mesh));
      expect(back.vertices, hasLength(5), reason: 'the near pair merged');
    });

    test('a truncated file is refused with a readable reason', () {
      final bytes = IjoStl.write(MeshPrimitives.cube(1, 1, 1));
      final cut = Uint8List.sublistView(bytes, 0, bytes.length - 60);
      expect(
        () => IjoStl.read(cut),
        throwsA(
          isA<FormatException>().having(
            (e) => e.message,
            'message',
            contains('only'),
          ),
        ),
      );
      expect(() => IjoStl.read(Uint8List(4)), throwsFormatException);
    });

    test('binary files whose header says solid still read as binary', () {
      final mesh = MeshPrimitives.cube(2, 2, 2);
      final raw = IjoStl.write(mesh);
      final bytes = Uint8List.fromList(raw);
      bytes.setRange(0, 5, 'solid'.codeUnits);
      // Trailing junk used to flip the sniff to ASCII and OOM on big files.
      final padded = Uint8List(bytes.length + 80);
      padded.setRange(0, bytes.length, bytes);
      final back = IjoStl.read(padded);
      expect(MeshKit.signedVolume(back), closeTo(8, 1e-3));
    });
  });

  group('STEP', () {
    test('a box round-trips as faceted triangles', () {
      final solid = MeshPrimitives.cube(10, 20, 30);
      final back = IjoStep.read(IjoStep.write(solid, name: 'block'));
      expect(
        utf8.decode(IjoStep.write(solid, name: 'block')),
        contains('FACETED_BREP'),
      );
      expect(MeshKit.isClosedSolid(back), isTrue);
      expect(IjoMeasure.volume(back), closeTo(6000, 1e-3));
    });

    test('an analytic cylinder writes CYLINDRICAL_SURFACE and reads back', () {
      final solid = BrepPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 5,
        height: 20,
      );
      final text = utf8.decode(IjoStep.writeSolid(solid, name: 'shaft'));
      expect(text, contains('CYLINDRICAL_SURFACE'));
      expect(text, isNot(contains('FACETED_BREP')));
      final back = IjoStep.readSolid(IjoStep.writeSolid(solid));
      expect(back.faces.any((f) => f.surface is BrepCylinder), isTrue);
    });

    test('a cylinder writes faceted STEP, not analytic surfaces', () {
      final solid = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 5,
        height: 20,
      );
      final text = utf8.decode(IjoStep.write(solid, name: 'shaft'));
      expect(text, contains('FACETED_BREP'));
      expect(text, isNot(contains('CYLINDRICAL_SURFACE')));
      final back = IjoStep.read(IjoStep.write(solid));
      expect(MeshKit.isClosedSolid(back), isTrue);
      expect(IjoMeasure.volume(back), closeTo(IjoMeasure.volume(solid), 1e-3));
    });

    test('a triangle mesh writes as faceted STEP and reads back closed', () {
      final mesh = MeshPrimitives.cube(4, 4, 4);
      final bytes = IjoStep.writeMesh(mesh, name: 'facets');
      expect(utf8.decode(bytes), contains('ISO-10303-21'));
      final back = IjoStep.read(bytes);
      final out = back;
      expect(MeshKit.isClosedSolid(out), isTrue);
      expect(MeshKit.signedVolume(out).abs(), closeTo(64, 1e-2));
    });

    test('a mesh document encodes as STEP', () {
      final mesh = MeshPrimitives.cube(5, 5, 5);
      final bytes = IjoIo.encodeMesh(mesh, IjoFormat.step, name: 'block');
      final back = IjoIo.decode(
        bytes,
        format: IjoFormat.step,
        name: 'block.step',
      );
      expect(back.kind, IjoDocKind.solid);
      expect(BrepHeal.topologyIsClosed(back.solid), isTrue);
      expect(MeshKit.signedVolume(back.mesh).abs(), closeTo(125, 1e-2));
    });

    test('a booleaned solid round-trips closed', () {
      final cut = MeshBoolean.run(
        MeshPrimitives.cube(20, 20, 20),
        MeshXform.transformVertices(
          MeshPrimitives.cube(10, 10, 10),
          (p) => p + Vector3(5, 5, 5),
        ),
        MeshBoolean.subtractOp,
      );
      final back = IjoStep.read(IjoStep.write(cut, name: 'cut'));
      final mesh = back;
      expect(MeshKit.isClosedSolid(mesh), isTrue);
      expect(MeshKit.signedVolume(mesh).abs(), closeTo(7000, 50));
    });
  });

  group('OBJ', () {
    test('round trips shared vertices and polygons', () {
      final mesh = MeshData(
        vertices: [
          Vector3(0, 0, 0),
          Vector3(1, 0, 0),
          Vector3(1, 1, 0),
          Vector3(0, 1, 0),
        ],
        // A quad, not two triangles: OBJ keeps it, STL could not.
        faces: [
          [0, 1, 2, 3],
        ],
      );
      final back = IjoObj.read(IjoObj.write(mesh));
      expect(back.vertices, hasLength(4));
      expect(back.faces.single, [0, 1, 2, 3]);
    });

    test('face groups carry CAD face ids', () {
      final mesh = MeshPrimitives.cylinder(
        origin: Vector3.zero(),
        axis: Vector3(0, 0, 1),
        radius: 3,
        height: 5,
      );
      final text = IjoObj.write(mesh);
      expect(text, contains('g face_0'));

      final back = IjoObj.read(text);
      // The grouping survives even though the names do not, and it is dense
      // from zero regardless of the leading `o` line.
      final groups = back.faceIds!.toSet();
      expect(groups, hasLength(mesh.faceIds!.toSet().length));
      expect(groups.reduce((a, b) => a > b ? a : b), groups.length - 1);
    });

    test('negative and slashed indices are read', () {
      const text = '''
        v 0 0 0
        v 1 0 0
        v 0 1 0
        f -3/1/1 -2/2/2 -1//3
      ''';
      final mesh = IjoObj.read(text);
      expect(mesh.faces.single, [0, 1, 2]);
    });

    test('comments and unknown records are ignored', () {
      const text = '''
        # a comment
        mtllib nothing.mtl
        v 0 0 0 # trailing comment
        v 1 0 0
        v 0 1 0
        vn 0 0 1
        vt 0 0
        usemtl steel
        s off
        f 1 2 3
      ''';
      final mesh = IjoObj.read(text);
      expect(mesh.vertices, hasLength(3));
      expect(mesh.faces, hasLength(1));
    });

    test('a face pointing past the end of the vertices is refused', () {
      expect(
        () => IjoObj.read('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n'),
        throwsA(
          isA<FormatException>().having(
            (e) => e.message,
            'message',
            contains('only 3 are defined'),
          ),
        ),
      );
      expect(() => IjoObj.read('# nothing\n'), throwsFormatException);
    });
  });

  group('3MF', () {
    test('round trips through a real ZIP', () {
      final mesh = MeshPrimitives.cube(8, 8, 8);
      final bytes = IjoThreeMf.write(mesh, name: 'cube');

      // "PK" and the end-of-directory signature: this is an actual archive,
      // not XML with a suggestive extension.
      expect(bytes.take(2), [0x50, 0x4b]);

      final archive = ZipArchive.fromBytes(bytes);
      expect(
        archive.names,
        containsAll(['[Content_Types].xml', '_rels/.rels', '3D/3dmodel.model']),
      );
      expect(archive.readText('_rels/.rels'), contains('3dmodel.model'));
      expect(archive.readText('3D/3dmodel.model'), contains('cube'));

      final back = IjoThreeMf.read(bytes);
      expect(back.vertices, hasLength(mesh.vertices.length));
      expect(MeshKit.signedVolume(back), closeTo(512, 1e-6));
    });

    test('writeAll keeps two objects as two objects', () {
      final a = MeshPrimitives.cube(8, 8, 8);
      final b = MeshPrimitives.cube(4, 4, 4);
      final bytes = IjoThreeMf.writeAll([
        ThreeMfPart(a, name: 'Block'),
        ThreeMfPart(b, name: 'Small'),
      ]);
      final parts = IjoThreeMf.readAll(bytes);
      expect(parts, hasLength(2));
      expect(parts[0].name, 'Block');
      expect(parts[1].name, 'Small');
      expect(MeshKit.signedVolume(parts[0].mesh).abs(), closeTo(512, 1e-6));
      expect(MeshKit.signedVolume(parts[1].mesh).abs(), closeTo(64, 1e-6));
      final firstOnly = IjoThreeMf.read(bytes);
      expect(MeshKit.signedVolume(firstOnly).abs(), closeTo(512, 1e-6));
    });

    test('coordinates keep six decimals, unlike float32 STL', () {
      // A value that float32 cannot hold: STL would return 0.1000000014...
      final mesh = MeshData(
        vertices: [Vector3(0.123457, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0)],
        faces: [
          [0, 1, 2],
        ],
      );
      final viaThreeMf = IjoThreeMf.read(IjoThreeMf.write(mesh));
      expect(viaThreeMf.vertices.first.x, 0.123457);

      final viaStl = IjoStl.read(IjoStl.write(mesh), weldTolerance: 0);
      expect(viaStl.vertices.first.x, isNot(0.123457));
    });

    test('units are converted on the way in', () {
      final mesh = MeshData(
        vertices: [Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)],
        faces: [
          [0, 1, 2],
        ],
      );
      final archive = ZipArchive.fromBytes(
        IjoThreeMf.write(mesh, unit: 'inch'),
      );
      final model = archive.readText('3D/3dmodel.model')!;
      expect(model, contains('unit="inch"'));

      final rebuilt = ZipArchive()..addText('3D/3dmodel.model', model);
      final back = IjoThreeMf.read(rebuilt.toBytes());
      expect(back.vertices.first.x, closeTo(25.4, 1e-9));
    });

    test('an unsupported unit is refused on write', () {
      final mesh = MeshPrimitives.cube(1, 1, 1);
      expect(
        () => IjoThreeMf.write(mesh, unit: 'furlong'),
        throwsArgumentError,
      );
    });

    test('the beam lattice extension is refused, not silently dropped', () {
      final archive = ZipArchive()
        ..addText(
          '3D/3dmodel.model',
          '<model unit="millimeter"><mesh><vertices>'
              '<vertex x="0" y="0" z="0" /></vertices><triangles />'
              '<beamlattice minlength="1" radius="1" />'
              '</mesh></model>',
        );
      expect(
        () => IjoThreeMf.read(archive.toBytes()),
        throwsA(
          isA<UnsupportedError>().having(
            (e) => e.message,
            'message',
            contains('beam lattice'),
          ),
        ),
      );
    });

    test('a model part somewhere unconventional is still found', () {
      final mesh = MeshPrimitives.cube(2, 2, 2);
      final model = ZipArchive.fromBytes(
        IjoThreeMf.write(mesh),
      ).readText('3D/3dmodel.model')!;
      final odd = ZipArchive()..addText('elsewhere/model.model', model);
      expect(IjoThreeMf.read(odd.toBytes()).vertices, isNotEmpty);
    });

    test('something that is not an archive is refused', () {
      expect(
        () => IjoThreeMf.read(Uint8List.fromList(utf8.encode('<model/>'))),
        throwsFormatException,
      );
    });
  });

  group('ZIP', () {
    test('round trips several entries', () {
      final zip = ZipArchive()
        ..addText('a.txt', 'hello')
        ..addText('dir/b.txt', 'world')
        ..addBytes('c.bin', [0, 1, 2, 255]);

      final back = ZipArchive.fromBytes(zip.toBytes());
      expect(back.length, 3);
      expect(back.readText('a.txt'), 'hello');
      expect(back.readText('dir/b.txt'), 'world');
      expect(back['c.bin'], [0, 1, 2, 255]);
      expect(back['missing'], isNull);
    });

    test('round trips text that is not ASCII', () {
      final back = ZipArchive.fromBytes(
        (ZipArchive()..addText('ijo.txt', 'jan pona — ale li pona')).toBytes(),
      );
      expect(back.readText('ijo.txt'), 'jan pona — ale li pona');
    });

    test('an empty archive is still a valid archive', () {
      final back = ZipArchive.fromBytes(ZipArchive().toBytes());
      expect(back.length, 0);
    });

    test('a compressed entry is reported, not misread', () {
      // Flip the stored-method field in both the local and central headers.
      final bytes = (ZipArchive()..addText('x.txt', 'body')).toBytes();
      final data = ByteData.sublistView(bytes);
      data.setUint16(8, 8, Endian.little);
      for (var i = 0; i + 4 < bytes.length; i++) {
        if (data.getUint32(i, Endian.little) == 0x02014b50) {
          data.setUint16(i + 10, 8, Endian.little);
          break;
        }
      }
      expect(
        () => ZipArchive.fromBytes(bytes),
        throwsA(
          isA<UnsupportedError>().having(
            (e) => e.message,
            'message',
            contains('compressed'),
          ),
        ),
      );
    });

    test('non-ZIP bytes are refused', () {
      expect(() => ZipArchive.fromBytes(Uint8List(10)), throwsFormatException);
    });
  });

  group('format dispatch', () {
    test('the extension picks the format', () {
      expect(IjoFormat.forPath('part.json'), IjoFormat.json);
      expect(IjoFormat.forPath('/tmp/Part.3MF'), IjoFormat.threeMf);
      expect(IjoFormat.forPath(r'C:\models\part.obj'), IjoFormat.obj);
      expect(IjoFormat.forPath('part.stl'), IjoFormat.stl);
      expect(IjoFormat.forPath('part.step'), IjoFormat.step);
      expect(IjoFormat.forPath('part.stp'), IjoFormat.step);
      expect(IjoFormat.forPath('part.weird'), isNull);
      expect(() => IjoFormat.parse('part.weird'), throwsFormatException);
    });

    test('every format writes bytes something can read back', () {
      final doc = IjoDocument.mesh(MeshPrimitives.cube(5, 5, 5), name: 'block');
      for (final format in IjoFormat.values) {
        final bytes = IjoIo.encode(doc, format);
        expect(bytes, isNotEmpty, reason: '${format.name} wrote nothing');

        final back = IjoIo.decode(bytes, format: format, name: 'block.x');
        expect(
          MeshKit.signedVolume(back.mesh),
          closeTo(125, 1e-3),
          reason: '${format.name} lost the shape',
        );
        expect(
          back.kind,
          format == IjoFormat.step ? IjoDocKind.solid : IjoDocKind.mesh,
          reason: '${format.name} must retain its representation',
        );
      }
    });

    test('every format round-trips a closed mesh', () {
      final doc = IjoDocument.mesh(
        MeshPrimitives.cylinder(
          origin: Vector3.zero(),
          axis: Vector3(0, 0, 1),
          radius: 2,
          height: 4,
        ),
      );
      for (final format in IjoFormat.values) {
        final back = IjoIo.decode(IjoIo.encode(doc, format), format: format);
        expect(
          back.kind,
          format == IjoFormat.step ? IjoDocKind.solid : IjoDocKind.mesh,
        );
        expect(MeshKit.isClosedSolid(back.mesh), isTrue);
      }
    });

    test('the document name comes from the filename', () {
      final bytes = IjoIo.encodeMesh(
        MeshPrimitives.cube(1, 1, 1),
        IjoFormat.obj,
      );
      expect(
        IjoIo.decode(bytes, name: '/models/bracket-v2.obj').name,
        'bracket-v2',
      );
      expect(IjoIo.decode(bytes, name: r'C:\a\b.obj').name, 'b');
    });

    test('bytes with no format and no name are refused', () {
      expect(
        () => IjoIo.decode(Uint8List(4)),
        throwsA(
          isA<FormatException>().having(
            (e) => e.message,
            'message',
            contains('Cannot tell what format'),
          ),
        ),
      );
    });

    test('text formats are text, binary formats are not', () {
      expect(IjoFormat.json.isText, isTrue);
      expect(IjoFormat.obj.isText, isTrue);
      expect(IjoFormat.stlAscii.isText, isTrue);
      expect(IjoFormat.stl.isText, isFalse);
      expect(IjoFormat.threeMf.isText, isFalse);
    });
  });
}
