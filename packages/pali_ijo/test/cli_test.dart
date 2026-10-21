import 'dart:convert';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/cli/cli.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

/// One command, run in-process. Returns the host so a test can read stdout,
/// stderr and the files it wrote.
Future<MemoryHost> run(
  List<String> args, {
  String? stdin,
  Map<String, List<int>>? files,
  int expect = 0,
}) async {
  final host = MemoryHost(
    stdin: stdin == null ? null : utf8.encode(stdin),
    files: files,
  );
  final code = await IjoCli(host).run(args);
  expectExit(code, expect, args, host);
  return host;
}

void expectExit(int actual, int wanted, List<String> args, MemoryHost host) {
  if (actual != wanted) {
    fail(
      'pali_ijo ${args.join(' ')}\n'
      'exited $actual, expected $wanted\n'
      'stderr: ${host.stderr}\n'
      'stdout: ${host.stdout.length > 400 ? '${host.stdout.substring(0, 400)}…' : host.stdout}',
    );
  }
}

/// Runs a chain of commands, feeding each one's stdout to the next.
Future<MemoryHost> pipeline(List<List<String>> stages) async {
  String? carried;
  MemoryHost host = MemoryHost();
  for (final stage in stages) {
    host = MemoryHost(
      stdin: carried == null ? null : utf8.encode(carried),
      files: host.files,
    );
    final code = await IjoCli(host).run(stage);
    expectExit(code, 0, stage, host);
    carried = host.stdout;
  }
  return host;
}

MeshData solidOf(MemoryHost host) => IjoDocument.decode(host.stdout).mesh;

void main() {
  test('rotated partial threads stay BREP with measured axial limits', () async {
    final axis=Vector3(1,2,3).normalized(), origin=Vector3(2,4,6);
    final source=BrepPrimitives.cylinder(origin: origin,axis:axis,radius:3,height:3);
    final host=await run(['threads','--pitch','1.5','--diameter','6',
      '--from','b','--length','1.5','--axis','1,2,3','--origin','2,4,6'],
      stdin:IjoDocument.solid(source).encode());
    final result=IjoDocument.decode(host.stdout);
    expect(result.solidOrNull,isNotNull);
    expect(result.solid.edges.any((e)=>e.curve is BrepHelix),isTrue);
    expect(MeshKit.isClosedSolid(result.mesh),isTrue);
    final planes=result.solid.faces.where((f)=>f.surface is BrepPlane)
        .map((f)=>((f.surface as BrepPlane).origin-origin).dot(axis)).toList();
    expect(planes.any((z)=>(z-3).abs()<1e-7),isTrue);
  });
  group('primitives', () {
    test('a box is a box', () async {
      final host = await run(['box', '--size', '10,20,30']);
      final solid = solidOf(host);
      expect(solid.faces, hasLength(12));
      expect(IjoMeasure.volume(solid), closeTo(6000, 1e-9));
      expect(host.stderr, isEmpty, reason: 'nothing to warn about');
    });

    test('a cylinder is a closed solid of about pi r^2 h', () async {
      final solid = solidOf(
        await run(['cylinder', '-r', '5', '--height', '10']),
      );
      expect(MeshKit.isClosedSolid(solid), isTrue);
      expect(IjoMeasure.volume(solid), closeTo(math.pi * 25 * 10, 40));
    });

    test('every primitive produces a solid of the right volume', () async {
      final cases = <List<String>, double>{
        ['sphere', '-r', '3']: 4 / 3 * math.pi * 27,
        ['cone', '-r', '4', '--height', '9']: math.pi * 16 * 9 / 3,
        ['torus', '--major', '10', '--minor', '2']:
            2 * math.pi * math.pi * 10 * 4,
        ['prism', '-d', '10', '--height', '5', '-n', '6']:
            // Regular hexagon of circumradius 5.
            6 * (math.sqrt(3) / 4) * 25 * 5,
      };
      for (final entry in cases.entries) {
        final solid = solidOf(await run(entry.key));
        expect(
          IjoMeasure.volume(solid),
          closeTo(entry.value, entry.value.abs() * 0.08 + 1),
          reason: entry.key.first,
        );
      }
    });

    test('an axis can be named instead of spelled out', () async {
      final byName = solidOf(
        await run(['cylinder', '-r', '2', '--height', '8', '--axis', 'x']),
      );
      final byVector = solidOf(
        await run(['cylinder', '-r', '2', '--height', '8', '--axis', '1,0,0']),
      );
      final a = IjoMeasure.boundingBox(byName)!;
      final b = IjoMeasure.boundingBox(byVector)!;
      expect(a.size.x, closeTo(b.size.x, 1e-12));
      expect(a.size.x, closeTo(8, 1e-9), reason: 'the axis is x');
    });
  });

  group('operations compose over stdin', () {
    test('the pipeline from the help text works', () async {
      final host = await pipeline([
        ['box', '--size', '40,40,20'],
        ['export', '--out', 'part.3mf'],
      ]);

      final bytes = host.files['part.3mf']!;
      expect(bytes.take(2), [0x50, 0x4b], reason: 'a real 3MF archive');
      expect(IjoThreeMf.read(bytes).faces, isNotEmpty);

      final measured = await run(
        ['measure', '--json'],
        stdin: (await pipeline([
          ['box', '--size', '40,40,20'],
        ])).stdout,
      );
      final volume = jsonDecode(measured.stdout)['volume'] as double;
      expect(volume, closeTo(32000, 1e-6));
    });

    test('a boolean takes its second solid from a file', () async {
      final block = await run(['box', '--size', '20,20,20']);
      final hole = await run([
        'cylinder',
        '-r',
        '4',
        '--height',
        '30',
        '--origin',
        '10,10,-5',
      ]);

      final host = await run(
        ['boolean', '--op', 'subtract', '--with', 'tool.json'],
        stdin: block.stdout,
        files: {'tool.json': utf8.encode(hole.stdout)},
      );

      final volume = IjoMeasure.volume(solidOf(host));
      expect(volume, closeTo(8000 - math.pi * 16 * 20, 50));
    });

    test('verbose subtract prints csg breadcrumbs on stderr', () async {
      final block = await run(['box', '--size', '20,20,20']);
      final hole = await run([
        'cylinder',
        '-r',
        '4',
        '--height',
        '30',
        '--origin',
        '10,10,-5',
      ]);

      final host = await run(
        ['-v', 'boolean', '--op', 'subtract', '--with', 'tool.json'],
        stdin: block.stdout,
        files: {'tool.json': utf8.encode(hole.stdout)},
      );

      expect(host.stderr, contains('[ijo:'));
      expect(host.stderr, contains('brep boolean subtract'));
      expect(host.stderr, contains('brep boolean done'));
      expect(
        host.stdout,
        isNot(contains('[ijo:')),
        reason: 'breadcrumbs must not corrupt the solid on stdout',
      );
    });

    test('translate then union puts two boxes side by side', () async {
      final moved = await run([
        'translate',
        '--by',
        '10,0,0',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      final host = await run(
        ['boolean', '--op', 'union', '--with', 'b.json'],
        stdin: (await run(['box', '--size', '10,10,10'])).stdout,
        files: {'b.json': utf8.encode(moved.stdout)},
      );
      final bounds = IjoMeasure.boundingBox(solidOf(host))!;
      expect(bounds.size.x, closeTo(20, 1e-9));
      expect(IjoMeasure.volume(solidOf(host)), closeTo(2000, 1e-6));
    });

    test('minkowski sum of two cubes is a 20-cube', () async {
      final box = await run(['box', '--size', '10,10,10']);
      final host = await run(
        ['minkowski', '--op', 'sum', '--with', 'k.json'],
        stdin: box.stdout,
        files: {'k.json': utf8.encode(box.stdout)},
      );
      expect(IjoMeasure.volume(solidOf(host)), closeTo(8000, 1.0));
      expect(MeshKit.isClosedSolid(solidOf(host)), isTrue);
    });

    test('offset grows a box', () async {
      final host = await run([
        'offset',
        '--radius',
        '1',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      expect(IjoMeasure.volume(solidOf(host)), greaterThan(1000));
      expect(MeshKit.isClosedSolid(solidOf(host)), isTrue);
    });

    test('hollow leaves the selected face open', () async {
      final host = await run([
        'hollow',
        '--thickness',
        '2',
        '--faces',
        'top',
      ], stdin: (await run(['box', '--size', '20,20,20'])).stdout);
      // A 20mm cube shelled to 2mm with an open top: 8000 minus the cavity,
      // which is 16×16×18.
      expect(
        IjoMeasure.volume(solidOf(host)),
        closeTo(8000 - 16 * 16 * 18, 400),
      );
    });

    test('mirror reflects without turning the solid inside out', () async {
      final host = await run([
        'mirror',
        '--normal',
        'x',
        '--origin',
        '0,0,0',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      final solid = solidOf(host);
      // Positive volume is the assertion: a reflection that forgets to
      // reverse the loops gives an inside-out solid of negative volume.
      expect(IjoMeasure.volume(solid), closeTo(1000, 1e-9));
      expect(IjoMeasure.boundingBox(solid)!.max.x, closeTo(0, 1e-9));
    });

    test('scale is uniform by default and per-axis on request', () async {
      final box = await run(['box', '--size', '10,10,10']);
      final uniform = await run(['scale', '-f', '2'], stdin: box.stdout);
      expect(IjoMeasure.volume(solidOf(uniform)), closeTo(8000, 1e-6));

      final stretched = await run([
        'scale',
        '--by',
        '1,1,3',
      ], stdin: box.stdout);
      expect(IjoMeasure.volume(solidOf(stretched)), closeTo(3000, 1e-6));
      expect(
        IjoMeasure.boundingBox(solidOf(stretched))!.size.z,
        closeTo(30, 1e-9),
      );
    });

    test('slice keeps the largest piece and says so', () async {
      final host = await run([
        'slice',
        '--normal',
        'z',
        '--origin',
        '0,0,4',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      expect(IjoMeasure.volume(solidOf(host)), closeTo(600, 1e-6));
      expect(host.stderr, contains('largest'));
    });

    test('slice can be told which piece to keep', () async {
      final box = (await run(['box', '--size', '10,10,10'])).stdout;
      final volumes = <double>[];
      for (final part in ['0', '1']) {
        final host = await run([
          'slice',
          '--normal',
          'z',
          '--origin',
          '0,0,4',
          '--part',
          part,
        ], stdin: box);
        volumes.add(IjoMeasure.volume(solidOf(host)));
      }
      expect(volumes.reduce((a, b) => a + b), closeTo(1000, 1e-6));
      expect(volumes, containsAll([closeTo(400, 1e-6), closeTo(600, 1e-6)]));

      await run(
        ['slice', '--normal', 'z', '--origin', '0,0,4', '--part', '5'],
        stdin: box,
        expect: 64,
      );
    });

    test('chamfer cuts less material than a fillet of the same size', () async {
      final box = (await run(['box', '--size', '20,20,20'])).stdout;
      final chamfered = await run([
        'chamfer',
        '-d',
        '2',
        '--edges',
        '0',
      ], stdin: box);
      final filleted = await run([
        'fillet',
        '-r',
        '2',
        '--edges',
        '0',
      ], stdin: box);
      expect(
        IjoMeasure.volume(solidOf(chamfered)),
        lessThan(IjoMeasure.volume(solidOf(filleted))),
      );
    });

    test('extrude pushes a face and finds it again by provenance', () async {
      final host = await run([
        'extrude',
        '--faces',
        'top',
        '--distance',
        '5',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      final solid = solidOf(host);
      expect(IjoMeasure.volume(solid), closeTo(1500, 1e-6));
      expect(IjoMeasure.boundingBox(solid)!.size.z, closeTo(15, 1e-9));
    });

    test('heal reports what it merged', () async {
      final host = await run([
        'heal',
      ], stdin: (await run(['box', '--size', '5,5,5'])).stdout);
      expect(host.stderr, contains('faces'));
      expect(IjoMeasure.volume(solidOf(host)), closeTo(125, 1e-9));
    });
  });

  group('sweeps', () {
    test('a lathe turns a shouldered shaft', () async {
      final host = await run(['lathe', '-l', '0,0;10,0;10,5;5,5;5,20;0,20']);
      // A Ø20×5 disc under a Ø10×15 shaft.
      expect(
        IjoMeasure.volume(solidOf(host)),
        closeTo(
          math.pi * 100 * 5 + math.pi * 25 * 15,
          (math.pi * 100 * 5 + math.pi * 25 * 15) * 0.05,
        ),
      );
    });

    test('a partial revolve is a wedge', () async {
      final full = solidOf(await run(['lathe', '-l', '5,0;10,0;10,10;5,10']));
      final quarter = solidOf(
        await run(['lathe', '-l', '5,0;10,0;10,10;5,10', '--angle', '90']),
      );
      expect(
        IjoMeasure.volume(quarter),
        closeTo(IjoMeasure.volume(full) / 4, IjoMeasure.volume(full) * 0.05),
      );
    });

    test('a straight pipe is a cylinder, and a tube is hollow', () async {
      final pipe = solidOf(
        await run(['pipe', '--path', '0,0,0;0,0,20', '-r', '5']),
      );
      expect(
        IjoMeasure.volume(pipe),
        closeTo(math.pi * 25 * 20, math.pi * 25 * 20 * 0.05),
      );

      final tube = solidOf(
        await run([
          'pipe',
          '--path',
          '0,0,0;0,0,20',
          '-r',
          '5',
          '--inner',
          '3',
        ]),
      );
      expect(
        IjoMeasure.volume(tube),
        closeTo(math.pi * (25 - 9) * 20, math.pi * 16 * 20 * 0.05),
      );
    });

    test('a swept square prism has the volume of a prism', () async {
      final solid = solidOf(
        await run([
          'sweep',
          '--profile',
          '-2,-2,0;2,-2,0;2,2,0;-2,2,0',
          '--path',
          '0,0,0;0,0,10',
        ]),
      );
      expect(IjoMeasure.volume(solid), closeTo(16 * 10, 1e-6));
    });

    test('a loft between two squares is a frustum', () async {
      final solid = solidOf(
        await run([
          'loft',
          '-s',
          '-4,-4,0;4,-4,0;4,4,0;-4,4,0',
          '-s',
          '-2,-2,10;2,-2,10;2,2,10;-2,2,10',
        ]),
      );
      // A square frustum: h/3 × (A₁ + A₂ + √(A₁A₂)).
      const a1 = 64.0;
      const a2 = 16.0;
      expect(
        IjoMeasure.volume(solid),
        closeTo(10 / 3 * (a1 + a2 + math.sqrt(a1 * a2)), 1e-6),
      );
    });

    test('threading one end leaves the other plain', () async {
      final shaft = (await run([
        'cylinder',
        '-r',
        '5',
        '--height',
        '20',
      ])).stdout;
      final threaded = solidOf(
        await run([
          'threads',
          '-p',
          '2',
          '--from',
          'a',
          '-l',
          '8',
          '-d',
          '10',
        ], stdin: shaft),
      );

      // Material came off. How *much* is not asserted: subtracting a helical
      // cutter from a faceted shaft over-removes, which is recorded under
      // Known Limitations, and sweep_test measures the groove against the
      // cutters where the boolean cannot skew it.
      final before = IjoMeasure.volumeSolid(IjoDocument.decode(shaft).solid);
      expect(IjoMeasure.volume(threaded), lessThan(before));
      expect(IjoMeasure.volume(threaded), greaterThan(0));
    });

    test(
      '--length is ignored when the whole axis is threaded, with a warning',
      () async {
        final host = await run(
          ['threads', '-p', '2', '-l', '5', '-d', '10'],
          stdin: (await run(['cylinder', '-r', '5', '--height', '20'])).stdout,
        );
        expect(host.stderr, contains('--length is ignored'));
      },
    );
  });

  group('selectors', () {
    test('the top face is one face', () async {
      final host = await run([
        'select',
        '--faces',
        'top',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      expect(host.stdout, contains('faces (1/6)'));
    });

    test('+ intersects and / unions', () async {
      final box = (await run(['box', '--size', '10,10,10'])).stdout;

      final both = await run([
        'select',
        '--faces',
        'planar+top',
        '--json',
      ], stdin: box);
      expect(jsonDecode(both.stdout)['faces'], hasLength(1));

      final either = await run([
        'select',
        '--faces',
        'top/bottom',
        '--json',
      ], stdin: box);
      expect(jsonDecode(either.stdout)['faces'], hasLength(2));

      final negated = await run([
        'select',
        '--faces',
        '!top',
        '--json',
      ], stdin: box);
      expect(jsonDecode(negated.stdout)['faces'], hasLength(5));
    });

    test('+ binds tighter than /', () async {
      final box = (await run(['box', '--size', '10,10,10'])).stdout;
      // "(planar and top) or bottom" is 2 faces; if / bound tighter it would
      // be "planar and (top or bottom)", which is also 2 — so a case where
      // they differ: "(cylinder and top) or bottom" is just bottom.
      final host = await run([
        'select',
        '--faces',
        'cylinder+top/bottom',
        '--json',
      ], stdin: box);
      expect(jsonDecode(host.stdout)['faces'], hasLength(1));
    });

    test('edges of a face are nested inside the edge grammar', () async {
      final host = await run([
        'select',
        '--edges',
        'of-face:top',
        '--json',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      expect(jsonDecode(host.stdout)['edges'], hasLength(4));
    });

    test('a selector still finds the top after a one-edge fillet', () async {
      final host = await run(
        ['select', '--faces', 'top', '--json'],
        stdin: (await pipeline([
          ['box', '--size', '20,20,20'],
          ['fillet', '-r', '2', '--edges', '0'],
        ])).stdout,
      );
      expect(jsonDecode(host.stdout)['faces'], hasLength(1));
    });

    test('fillet of one edge reduces volume', () async {
      final host = await run([
        'fillet',
        '-r',
        '1',
        '--edges',
        '0',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      expect(IjoMeasure.volume(solidOf(host)), lessThan(1000));
    });

    test('a selector that matches nothing is an error, not a no-op', () async {
      final host = await run(
        ['fillet', '-r', '1', '--edges', 'circular'],
        stdin: (await run(['box', '--size', '10,10,10'])).stdout,
        expect: 64,
      );
      expect(host.stderr, contains('No edges matched'));
    });

    test('an unknown selector term names what was expected', () async {
      final host = await run(
        ['select', '--faces', 'wibble'],
        stdin: (await run(['box', '--size', '10,10,10'])).stdout,
        expect: 64,
      );
      expect(host.stderr, contains('Unknown --faces term "wibble"'));
      expect(host.stderr, contains('planar'));
    });
  });

  group('queries', () {
    test('measure reports mesh volume', () async {
      final host = await run([
        'measure',
      ], stdin: (await run(['cylinder', '-r', '10', '--height', '10'])).stdout);
      expect(host.stdout, contains('volume'));
      expect(host.stdout, contains('surface area'));
    });

    test('measure --json is machine-readable', () async {
      final host = await run([
        'measure',
        '--json',
      ], stdin: (await run(['box', '--size', '2,3,4'])).stdout);
      final data = jsonDecode(host.stdout) as Map<String, dynamic>;
      expect(data['volume'], closeTo(24, 1e-9));
      expect(data['exact'], isTrue);
      expect((data['centroid'] as List)[2], closeTo(2, 1e-9));
    });

    test('info reports a closed mesh', () async {
      final host = await run([
        'info',
        '--json',
      ], stdin: (await run(['cylinder', '-r', '3', '--height', '6'])).stdout);
      final data = jsonDecode(host.stdout) as Map<String, dynamic>;
      expect(data['kind'], 'mesh');
      expect(data['closed'], isTrue);
      expect(data['faces'], greaterThan(0));
    });

    test('info on a tessellated document reports a mesh', () async {
      final host = await run(
        ['info', '--json'],
        stdin: (await pipeline([
          ['box', '--size', '4,4,4'],
          ['tessellate'],
        ])).stdout,
      );
      expect(jsonDecode(host.stdout)['kind'], 'mesh');
    });

    test('tessellate of a mesh is still a mesh', () async {
      final host = await run(
        ['info', '--json'],
        stdin: (await pipeline([
          ['box', '--size', '10,10,10'],
          ['tessellate'],
        ])).stdout,
      );
      expect(jsonDecode(host.stdout)['kind'], 'mesh');
    });
  });

  group('export', () {
    test('the extension picks the format', () async {
      final solid = (await run(['box', '--size', '10,10,10'])).stdout;
      for (final ext in ['json', '3mf', 'obj', 'stl', 'stla']) {
        final host = await run(['export', '--out', 'part.$ext'], stdin: solid);
        final bytes = host.files['part.$ext'];
        expect(bytes, isNotNull, reason: ext);
        expect(bytes, isNotEmpty, reason: ext);

        final back = IjoIo.decode(bytes!, name: 'part.$ext');
        expect(
          MeshKit.signedVolume(back.mesh),
          closeTo(1000, 1e-3),
          reason: '$ext lost the shape',
        );
      }
    });

    test('a text format can go to stdout', () async {
      final host = await run([
        'export',
        '--format',
        'obj',
      ], stdin: (await run(['box', '--size', '1,1,1'])).stdout);
      expect(host.stdout, contains('\nv '));
      expect(host.stdout, contains('\nf '));
    });

    test('export needs somewhere to write', () async {
      final host = await run(
        ['export'],
        stdin: (await run(['box', '--size', '1,1,1'])).stdout,
        expect: 64,
      );
      expect(host.stderr, contains('--out'));
    });

    test('a written document keeps its name', () async {
      final host = await run([
        'export',
        '--out',
        'part.json',
        '--name',
        'bracket',
      ], stdin: (await run(['box', '--size', '1,1,1'])).stdout);
      expect(
        IjoIo.decode(host.files['part.json']!, name: 'part.json').name,
        'bracket',
      );
    });

    test('a solid read back from JSON keeps its volume', () async {
      final written = await run([
        'export',
        '--out',
        'part.json',
      ], stdin: (await run(['cylinder', '-r', '4', '--height', '4'])).stdout);
      final host = await run([
        'measure',
        '--in',
        'part.json',
        '--json',
      ], files: written.files);
      expect(
        jsonDecode(host.stdout)['volume'],
        closeTo(math.pi * 16 * 4, math.pi * 16 * 4 * 0.05),
      );
    });
  });

  group('input handling', () {
    test('the format of piped bytes is sniffed', () async {
      // No filename to go on, and the stages did not agree in advance what
      // they were passing.
      for (final format in ['3mf', 'obj', 'stl']) {
        final written = await run([
          'export',
          '--out',
          'x.$format',
        ], stdin: (await run(['box', '--size', '6,6,6'])).stdout);
        final host = MemoryHost(stdin: written.files['x.$format']);
        expect(
          await IjoCli(host).run(['info', '--json']),
          0,
          reason: '$format was not recognised: ${host.stderr}',
        );
        expect(jsonDecode(host.stdout)['kind'], 'mesh');
      }
    });

    test('no input at all says what to do about it', () async {
      final host = await run(['measure'], expect: 64);
      expect(host.stderr, contains('Pipe a solid in'));
    });

    test('a missing file is reported as a missing file', () async {
      final host = await run(['measure', '--in', 'nope.json'], expect: 66);
      expect(host.stderr, contains('No such file'));
    });

    test('bytes that are not any known format are refused', () async {
      final host = MemoryHost(stdin: [1, 2, 3, 4, 5, 6, 7, 8]);
      expect(await IjoCli(host).run(['info']), 65);
      expect(host.stderr, contains('does not look like'));
    });
  });

  group('argument errors', () {
    test('an unknown command shows the command list', () async {
      final host = await run(['wibble'], expect: 64);
      expect(host.stderr, contains('Could not find a command named "wibble"'));
      expect(host.stderr, contains('box'));
      expect(host.stderr, contains('fillet'));
    });

    test('an unknown flag shows the usage', () async {
      final host = await run(['box', '--sixe', '1,1,1'], expect: 64);
      expect(host.stderr, contains('sixe'));
      expect(host.stderr, contains('Usage'));
    });

    test('a malformed vector says what a vector looks like', () async {
      final host = await run(['box', '--size', '10,20'], expect: 64);
      expect(host.stderr, contains('x,y,z'));
    });

    test('a non-numeric value is caught', () async {
      final host = await run([
        'cylinder',
        '-r',
        'wide',
        '--height',
        '10',
      ], expect: 64);
      expect(host.stderr, contains('expects a number'));
    });

    test('a missing required value is named', () async {
      final host = await run(['cylinder', '--height', '10'], expect: 64);
      expect(host.stderr, contains('--radius'));
    });

    test('a zero or negative size is refused', () async {
      expect(
        (await run(['box', '--size', '0,10,10'], expect: 64)).stderr,
        contains('positive'),
      );
      expect(
        (await run([
          'cylinder',
          '-r',
          '-3',
          '--height',
          '1',
        ], expect: 64)).stderr,
        contains('greater than zero'),
      );
    });

    test('a torus whose tube swallows its hole is refused', () async {
      final host = await run([
        'torus',
        '--major',
        '2',
        '--minor',
        '5',
      ], expect: 64);
      expect(host.stderr, contains('smaller than'));
    });

    test('a prism with two sides is refused', () async {
      final host = await run([
        'prism',
        '-d',
        '10',
        '--height',
        '5',
        '-n',
        '2',
      ], expect: 64);
      expect(host.stderr, contains('at least 3'));
    });

    test('a zero direction is refused rather than producing NaN', () async {
      final host = await run(
        ['mirror', '--normal', '0,0,0'],
        stdin: (await run(['box', '--size', '1,1,1'])).stdout,
        expect: 64,
      );
      expect(host.stderr, contains('zero vector'));
    });

    test('a loft of one section is refused', () async {
      final host = await run(['loft', '-s', '0,0,0;1,0,0;1,1,0'], expect: 64);
      expect(host.stderr, contains('at least two'));
    });

    test('a degenerate profile is refused with a geometric reason', () async {
      final host = await run([
        'sweep',
        '--profile',
        '0,0,0;1,0,0',
        '--path',
        '0,0,0;0,0,5',
      ], expect: 64);
      expect(host.stderr, contains('at least 3 points'));
    });
  });

  group('the runner itself', () {
    test('help lists every command, grouped by what it does', () {
      // Asserted on the usage text rather than by running --help, because
      // `args` prints that itself and it never reaches the host.
      final usage = IjoCli(MemoryHost()).runner.usage;
      for (final command in [
        'box',
        'cylinder',
        'sphere',
        'cone',
        'torus',
        'prism',
        'pyramid',
        'lathe',
        'revolve',
        'sweep',
        'pipe',
        'loft',
        'boolean',
        'minkowski',
        'offset',
        'fillet',
        'chamfer',
        'hollow',
        'extrude',
        'threads',
        'scale',
        'mirror',
        'translate',
        'slice',
        'heal',
        'tessellate',
        'measure',
        'select',
        'info',
        'export',
      ]) {
        expect(usage, contains(command), reason: '$command is missing');
      }
    });

    test('every command documents itself', () {
      for (final command in IjoCli(MemoryHost()).runner.commands.values) {
        expect(command.description, isNotEmpty, reason: command.name);
        // Every option carries help text, since the usage screen is the only
        // documentation a CLI user has to hand.
        for (final option in command.argParser.options.values) {
          expect(
            option.help,
            isNotNull,
            reason: '${command.name} --${option.name}',
          );
        }
      }
    });

    test('nothing but the document goes to stdout', () async {
      // The contract that makes the pipeline work. slice warns on stderr, so
      // it is the case that would break it.
      final host = await run([
        'slice',
        '--normal',
        'z',
        '--origin',
        '0,0,5',
      ], stdin: (await run(['box', '--size', '10,10,10'])).stdout);
      expect(host.stderr, isNotEmpty);
      expect(
        () => IjoDocument.decode(host.stdout),
        returnsNormally,
        reason: 'stdout must be nothing but the document',
      );
    });

    test('stdout is compact, a written file is readable', () async {
      final piped = await run(['box', '--size', '1,1,1']);
      expect(piped.stdout, isNot(contains('\n  ')));

      final written = await run([
        'export',
        '--out',
        'part.json',
      ], stdin: piped.stdout);
      expect(
        utf8.decode(written.files['part.json']!),
        contains('\n  '),
        reason: 'a file on disk is meant to be read and diffed',
      );
    });
  });

  group('the host boundary', () {
    test('a memory host records what was written', () async {
      final host = MemoryHost();
      await IjoCli(host).run(['box', '--size', '1,2,3', '--out', 'a.json']);
      expect(host.files.keys, contains('a.json'));
      expect(host.stdout, isEmpty, reason: '--out means not stdout');
    });

    test('binary bytes on stdout are not mangled by text encoding', () async {
      final host = await run([
        'export',
        '--format',
        'stl',
      ], stdin: (await run(['box', '--size', '1,1,1'])).stdout);
      final bytes = host.stdoutBytes;
      // 12 triangles at 50 bytes each, plus the 84-byte header.
      expect(bytes, hasLength(84 + 12 * 50));
      expect(ByteData.sublistView(bytes).getUint32(80, Endian.little), 12);
    });
  });
}
