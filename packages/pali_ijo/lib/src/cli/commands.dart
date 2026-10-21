import 'dart:convert';
import 'dart:math' as math;

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:vector_math/vector_math_64.dart';

import '../brep/brep.dart';
import '../io/io.dart';
import '../measure/measure.dart';
import '../mesh/mesh_build.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_regions.dart';
import '../mesh/mesh_kit.dart';
import '../mesh/mesh_seal.dart';
import '../mesh/mesh_xform.dart';
import '../ops/edge_geometry.dart';
import '../ops/ops.dart';
import '../thread/form.dart';
import '../select/select.dart';
import '../sweep/sweep.dart';
import 'io_host.dart';
import 'values.dart';

/// Base for every subcommand: handles reading the input document and writing
/// the output one, so a command body is only its geometry.
///
/// The contract that makes commands composable is here rather than in each of
/// them: read solid JSON from `--in` or stdin, write solid JSON to `--out` or
/// stdout, and put nothing else on stdout. That is what lets
/// `pali_ijo box ... | pali_ijo fillet ... | pali_ijo export ...` work.
abstract class IjoCommand extends Command<int> {
  IjoCommand(this.host) {
    argParser.addFlag(
      'verbose',
      abbr: 'v',
      help:
          'Kernel breadcrumbs on stderr (also IJO_DEBUG=1). The last line '
          'is the step that was running if the kernel crashes.',
      negatable: false,
    );
    if (takesInput) {
      argParser.addOption(
        'in',
        abbr: 'i',
        help: 'Input solid JSON. Defaults to stdin.',
        valueHelp: 'FILE',
      );
    }
    if (writesOutput) {
      argParser
        ..addOption(
          'out',
          abbr: 'o',
          help:
              'Where to write. Defaults to stdout. The extension picks the '
              'format: .json keeps the analytic solid, .3mf/.obj/.stl write a '
              'mesh.',
          valueHelp: 'FILE',
        )
        ..addOption(
          'name',
          help: 'Name recorded in the document.',
          valueHelp: 'TEXT',
        );
    }
  }

  final IjoHost host;

  /// Whether this command consumes a document. False for primitives, which
  /// make one from nothing.
  bool get takesInput => true;

  /// Whether this command produces a document. False for queries, which
  /// report rather than transform.
  bool get writesOutput => true;

  ArgResultsLike get opts => ArgResultsLike(argResults!);

  /// The command's geometry. Return the document to write, or null when the
  /// command has already reported its own result.
  Future<IjoDocument?> build(IjoDocument? input);

  @override
  Future<int> run() async {
    final input = takesInput ? await _read() : null;
    final output = await build(input);
    if (output != null && writesOutput) await _write(output);
    return 0;
  }

  Future<IjoDocument> _read() async {
    final path = opts.string('in');
    final bytes = path == null
        ? await host.readStdin()
        : await host.readFile(path);
    if (bytes.isEmpty) {
      throw CliValueException(
        path == null
            ? 'No input on stdin. Pipe a solid in, or pass --in FILE.'
            : 'File "$path" is empty.',
      );
    }
    return IjoIo.decode(bytes, name: path);
  }

  Future<void> _write(IjoDocument document) async {
    final path = opts.string('out');
    final named = opts.string('name');
    final doc = named == null || named == document.name
        ? document
        : _renamed(document, named);

    if (path == null) {
      // stdout is the pipeline, and a pipeline carries solids. A mesh format
      // on stdout would silently turn the analytic solid into triangles for
      // every later stage, so it has to be asked for by filename.
      host.writeStdout(doc.encode(pretty: false));
      return;
    }
    await host.writeFile(path, IjoIo.encode(doc, IjoFormat.parse(path)));
  }

  static IjoDocument _renamed(IjoDocument doc, String name) {
    final solid = doc.solidOrNull;
    return solid != null
        ? IjoDocument.solid(solid, name: name)
        : IjoDocument.mesh(doc.mesh, name: name);
  }

  /// Reports a query result, as text or as JSON.
  void report(Map<String, dynamic> data, List<String> lines) {
    if (opts.flag('json')) {
      host.writeStdout(const JsonEncoder.withIndent('  ').convert(data));
    } else {
      host.writeStdout(lines.join('\n'));
    }
  }
}

/// Thin wrapper over `ArgResults` that fails with a readable message instead
/// of a cast error when an option is missing.
class ArgResultsLike {
  ArgResultsLike(this._results);
  final ArgResults _results;

  String? string(String name) {
    if (!_results.options.contains(name)) return null;
    final v = _results[name];
    return v is String && v.isNotEmpty ? v : null;
  }

  bool flag(String name) =>
      _results.options.contains(name) && _results[name] == true;

  List<String> multi(String name) =>
      _results.options.contains(name) ? _results.multiOption(name) : const [];

  String require(String name, String help) =>
      string(name) ?? (throw CliValueException('--$name is required. $help'));

  double number(String name, {double? or}) {
    final raw = string(name);
    if (raw == null) {
      return or ??
          (throw CliValueException('--$name is required and takes a number.'));
    }
    return CliValues.number(raw, name);
  }

  double positive(String name, {double? or}) {
    final raw = string(name);
    if (raw == null) {
      return or ??
          (throw CliValueException(
            '--$name is required and takes a number '
            'greater than zero.',
          ));
    }
    return CliValues.positive(raw, name);
  }

  int integer(String name, {int? or}) {
    final raw = string(name);
    if (raw == null) {
      return or ??
          (throw CliValueException(
            '--$name is required and takes a whole '
            'number.',
          ));
    }
    return CliValues.integer(raw, name);
  }

  Vector3 vector(String name, {Vector3? or}) {
    final raw = string(name);
    if (raw == null) {
      return or ??
          (throw CliValueException('--$name is required and takes "x,y,z".'));
    }
    return CliValues.vector(raw, name);
  }

  Vector3 direction(String name, {Vector3? or}) {
    final raw = string(name);
    if (raw == null) {
      return or ??
          (throw CliValueException(
            '--$name is required and takes a '
            'direction, as "x,y,z" or an axis name.',
          ));
    }
    return CliValues.direction(raw, name);
  }
}

// ── primitives ───────────────────────────────────────────────────────────────

/// Shared plumbing for the commands that make a solid from nothing.
abstract class _PrimitiveCommand extends IjoCommand {
  _PrimitiveCommand(super.host);

  @override
  bool get takesInput => false;

  IjoDocument make();

  @override
  Future<IjoDocument?> build(IjoDocument? input) async => make();
}

class BoxCommand extends _PrimitiveCommand {
  BoxCommand(super.host) {
    argParser.addOption(
      'size',
      abbr: 's',
      help: 'Extent along each axis.',
      valueHelp: 'X,Y,Z',
    );
  }

  @override
  String get name => 'box';
  @override
  String get description => 'A rectangular box with a corner at the origin.';

  @override
  IjoDocument make() {
    final s = opts.vector('size');
    if (s.x <= 0 || s.y <= 0 || s.z <= 0) {
      throw CliValueException('--size must be positive in every axis, got $s');
    }
    return IjoDocument.solid(
      BrepPrimitives.box(s.x, s.y, s.z),
      name: opts.string('name') ?? name,
    );
  }
}

class CylinderCommand extends _PrimitiveCommand {
  CylinderCommand(super.host) {
    argParser
      ..addOption('radius', abbr: 'r', help: 'Radius.', valueHelp: 'MM')
      ..addOption('height', help: 'Height along the axis.', valueHelp: 'MM')
      ..addOption('origin', help: 'Centre of the base.', valueHelp: 'X,Y,Z')
      ..addOption('axis', help: 'Direction of the height.', valueHelp: 'DIR');
  }

  @override
  String get name => 'cylinder';
  @override
  String get description =>
      'A cylinder. One exact cylindrical face, not a '
      'many-sided prism.';

  @override
  IjoDocument make() => IjoDocument.solid(
    BrepPrimitives.cylinder(
      origin: opts.vector('origin', or: Vector3.zero()),
      axis: opts.direction('axis', or: Vector3(0, 0, 1)),
      radius: opts.positive('radius'),
      height: opts.positive('height'),
    ),
    name: opts.string('name') ?? name,
  );
}

class SphereCommand extends _PrimitiveCommand {
  SphereCommand(super.host) {
    argParser
      ..addOption('radius', abbr: 'r', help: 'Radius.', valueHelp: 'MM')
      ..addOption('center', help: 'Centre point.', valueHelp: 'X,Y,Z');
  }

  @override
  String get name => 'sphere';
  @override
  String get description => 'A sphere: one exact spherical face.';

  @override
  IjoDocument make() => IjoDocument.solid(
    BrepPrimitives.sphere(
      center: opts.vector('center', or: Vector3.zero()),
      radius: opts.positive('radius'),
    ),
    name: opts.string('name') ?? name,
  );
}

class ConeCommand extends _PrimitiveCommand {
  ConeCommand(super.host) {
    argParser
      ..addOption('radius', abbr: 'r', help: 'Radius.', valueHelp: 'MM')
      ..addOption('height', help: 'Height along the axis.', valueHelp: 'MM')
      ..addOption('origin', help: 'Reference point.', valueHelp: 'X,Y,Z')
      ..addOption('axis', help: 'Axis direction.', valueHelp: 'DIR');
  }

  @override
  String get name => 'cone';
  @override
  String get description => 'A cone, apex along the axis.';

  @override
  IjoDocument make() => IjoDocument.solid(
    BrepPrimitives.cone(
      origin: opts.vector('origin', or: Vector3.zero()),
      axis: opts.direction('axis', or: Vector3(0, 0, 1)),
      radius: opts.positive('radius'),
      height: opts.positive('height'),
    ),
    name: opts.string('name') ?? name,
  );
}

class TorusCommand extends _PrimitiveCommand {
  TorusCommand(super.host) {
    argParser
      ..addOption('major', help: 'Ring radius.', valueHelp: 'MM')
      ..addOption('minor', help: 'Tube radius.', valueHelp: 'MM')
      ..addOption('center', help: 'Centre point.', valueHelp: 'X,Y,Z')
      ..addOption('axis', help: 'Axis direction.', valueHelp: 'DIR');
  }

  @override
  String get name => 'torus';
  @override
  String get description => 'A torus: one exact toroidal face.';

  @override
  IjoDocument make() {
    final major = opts.positive('major');
    final minor = opts.positive('minor');
    if (minor >= major) {
      throw CliValueException(
        '--minor ($minor) must be smaller than --major ($major), or the '
        'tube swallows its own hole.',
      );
    }
    return IjoDocument.solid(
      BrepPrimitives.torus(
        center: opts.vector('center', or: Vector3.zero()),
        axis: opts.direction('axis', or: Vector3(0, 0, 1)),
        majorRadius: major,
        minorRadius: minor,
      ),
      name: opts.string('name') ?? name,
    );
  }
}

class PrismCommand extends _PrimitiveCommand {
  PrismCommand(super.host) {
    argParser
      ..addOption('diameter', abbr: 'd', help: 'Across the corners.')
      ..addOption('height', help: 'Height along +Z.', valueHelp: 'MM')
      ..addOption('sides', abbr: 'n', help: 'Number of sides.', valueHelp: 'N');
  }

  @override
  String get name => 'prism';
  @override
  String get description => 'A regular n-sided prism.';

  @override
  IjoDocument make() => IjoDocument.solid(
    BrepPrimitives.prism(
      diameter: opts.positive('diameter'),
      height: opts.positive('height'),
      sides: _sides(opts.integer('sides')),
    ),
    name: opts.string('name') ?? name,
  );
}

class PyramidCommand extends _PrimitiveCommand {
  PyramidCommand(super.host) {
    argParser
      ..addOption('diameter', abbr: 'd', help: 'Across the corners.')
      ..addOption('height', help: 'Height along +Z.', valueHelp: 'MM')
      ..addOption('sides', abbr: 'n', help: 'Number of sides.', valueHelp: 'N');
  }

  @override
  String get name => 'pyramid';
  @override
  String get description => 'A regular n-sided pyramid.';

  @override
  IjoDocument make() => IjoDocument.solid(
    BrepPrimitives.pyramid(
      diameter: opts.positive('diameter'),
      height: opts.positive('height'),
      sides: _sides(opts.integer('sides')),
    ),
    name: opts.string('name') ?? name,
  );
}

int _sides(int n) =>
    n >= 3 ? n : throw CliValueException('--sides must be at least 3, got $n');

// ── operations ───────────────────────────────────────────────────────────────

class BooleanCommand extends IjoCommand {
  BooleanCommand(super.host) {
    argParser
      ..addOption(
        'op',
        help: 'Which boolean.',
        allowed: ['union', 'subtract', 'intersect'],
        defaultsTo: 'union',
      )
      ..addOption(
        'with',
        abbr: 'w',
        help: 'The other solid.',
        valueHelp: 'FILE',
      );
  }

  @override
  String get name => 'boolean';
  @override
  String get description =>
      'Union, subtract or intersect this solid with another.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final other = IjoIo.decode(
      await host.readFile(opts.require('with', 'It names the second solid.')),
      name: opts.string('with'),
    );
    final op = CliValues.booleanOp(opts.string('op') ?? 'union');
    final a = input!.solidOrNull;
    final b = other.solidOrNull;
    if (a != null && b != null) {
      try {
        return IjoDocument.solid(
          IjoOps.booleanSolid(a, b, op),
          name: input.name,
        );
      } on BrepUnsupported {
        rethrow;
      }
    }
    if (a != null || b != null) {
      throw BrepUnsupported(
        'Boolean of a B-Rep with a mesh is not an implicit conversion. '
        'Convert the B-Rep body to a mesh first.',
      );
    }
    return IjoDocument.mesh(
      IjoOps.boolean(input.mesh, other.mesh, op),
      name: input.name,
    );
  }
}

class MinkowskiCommand extends IjoCommand {
  MinkowskiCommand(super.host) {
    argParser
      ..addOption(
        'op',
        help: 'Sum (dilate) or difference (erode).',
        allowed: ['sum', 'difference', 'diff'],
        defaultsTo: 'sum',
      )
      ..addOption(
        'with',
        abbr: 'w',
        help: 'The kernel solid. A sphere at the origin is a round offset.',
        valueHelp: 'FILE',
      );
  }

  @override
  String get name => 'minkowski';
  @override
  String get description =>
      'Minkowski sum or difference with another solid (mesh offset).';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final other = IjoIo.decode(
      await host.readFile(opts.require('with', 'It names the kernel solid.')),
      name: opts.string('with'),
    );
    final op = opts.string('op') ?? 'sum';
    final solid = op == 'sum'
        ? IjoOps.minkowskiSum(input!.mesh, other.mesh)
        : IjoOps.minkowskiDifference(input!.mesh, other.mesh);
    return IjoDocument.mesh(solid, name: input.name);
  }
}

class OffsetCommand extends IjoCommand {
  OffsetCommand(super.host) {
    argParser.addOption(
      'radius',
      abbr: 'r',
      help:
          'Offset distance. Positive grows; negative shrinks. '
          'A sphere at the origin is the kernel.',
      valueHelp: 'MM',
    );
  }

  @override
  String get name => 'offset';
  @override
  String get description =>
      'Round offset of a solid by Minkowski sum with a sphere.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    return IjoDocument.mesh(
      IjoOps.offsetMesh(input!.mesh, opts.number('radius')),
      name: input.name,
    );
  }
}

class FilletCommand extends IjoCommand {
  FilletCommand(super.host) {
    argParser
      ..addOption('radius', abbr: 'r', help: 'Radius.', valueHelp: 'MM')
      ..addOption(
        'edges',
        abbr: 'e',
        help: 'Which edges. ${CliSelectors.edgeHelp}',
        defaultsTo: 'convex',
        valueHelp: 'SPEC',
      )
      ..addFlag(
        'inverse',
        help: 'Fill the corner rather than round it.',
        negatable: false,
      );
  }

  @override
  String get name => 'fillet';
  @override
  String get description => 'Round edges to a radius.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      final ids = CliSelectors.brepEdges(
        opts.string('edges') ?? 'convex',
      ).resolve(brep);
      if (ids.isEmpty) {
        throw CliValueException('No edges matched "${opts.string('edges')}".');
      }
      return IjoDocument.solid(
        IjoOps.filletSolid(
          brep,
          ids,
          radius: opts.positive('radius'),
          inverse: opts.flag('inverse'),
        ),
        name: input.name,
      );
    }
    final solid = input.mesh;
    final points = CliSelectors.edges(
      opts.string('edges') ?? 'convex',
    ).resolvePoints(solid);
    if (points.isEmpty) {
      throw CliValueException(
        'No edges matched "${opts.string('edges')}". Run '
        '"pali_ijo select --edges all" to see what is there.',
      );
    }
    final radius = opts.positive('radius');
    final inverse = opts.flag('inverse');
    final out = IjoOps.filletEdges(
      solid,
      points,
      radius: radius,
      inverse: inverse,
    );
    return IjoDocument.mesh(out, name: input.name);
  }
}

class ChamferCommand extends IjoCommand {
  ChamferCommand(super.host) {
    argParser
      ..addOption(
        'distance',
        abbr: 'd',
        help: 'How far the cut reaches back along each face.',
        valueHelp: 'MM',
      )
      ..addOption(
        'edges',
        abbr: 'e',
        help: 'Which edges. ${CliSelectors.edgeHelp}',
        defaultsTo: 'convex',
        valueHelp: 'SPEC',
      );
  }

  @override
  String get name => 'chamfer';
  @override
  String get description => 'Cut edges back at 45°.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      final ids = CliSelectors.brepEdges(
        opts.string('edges') ?? 'convex',
      ).resolve(brep);
      if (ids.isEmpty) {
        throw CliValueException('No edges matched "${opts.string('edges')}".');
      }
      return IjoDocument.solid(
        IjoOps.chamferSolid(brep, ids, opts.positive('distance')),
        name: input.name,
      );
    }
    final solid = input.mesh;
    final points = CliSelectors.edges(
      opts.string('edges') ?? 'convex',
    ).resolvePoints(solid);
    if (points.isEmpty) {
      throw CliValueException('No edges matched "${opts.string('edges')}".');
    }
    final distance = opts.positive('distance');
    final out = IjoOps.chamferEdges(solid, points, distance);
    return IjoDocument.mesh(out, name: input.name);
  }
}

class HollowCommand extends IjoCommand {
  HollowCommand(super.host) {
    argParser
      ..addOption('thickness', abbr: 't', help: 'Wall thickness.')
      ..addOption(
        'faces',
        abbr: 'f',
        help: 'Faces to leave open. ${CliSelectors.faceHelp}',
        defaultsTo: 'top',
        valueHelp: 'SPEC',
      )
      ..addFlag(
        'closed',
        help: 'Shell with no opening at all.',
        negatable: false,
      );
  }

  @override
  String get name => 'hollow';
  @override
  String get description => 'Shell the solid, optionally leaving faces open.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      final openings = opts.flag('closed')
          ? <int>[]
          : CliSelectors.brepFaces(opts.string('faces') ?? 'top').resolve(brep);
      if (openings.isEmpty && !opts.flag('closed'))
        throw CliValueException('No opening faces matched');
      final n = openings.isEmpty
          ? Vector3(0, 0, 1)
          : brep.faces[openings.first].outwardNormal(0, 0);
      final out = IjoOps.hollowSolid(
        brep,
        openings,
        n,
        wallThicknessMm: opts.positive('thickness'),
      );
      IjoOps.tessellateClosed(out, op: 'Hollow');
      return IjoDocument.solid(out, name: input.name);
    }
    final solid = input.mesh;
    final openings = opts.flag('closed')
        ? const <int>[]
        : CliSelectors.faces(opts.string('faces') ?? 'top').resolve(solid);
    if (openings.isEmpty && !opts.flag('closed')) {
      throw CliValueException(
        'No faces matched "${opts.string('faces')}". Pass --closed for a '
        'sealed shell.',
      );
    }

    // The opening normal is the outward normal of the first face left open,
    // which is what the caller means by "open this face" and saves them
    // stating it twice.
    final n =
        (openings.isEmpty
            ? null
            : EdgeGeometry.faceNormal(solid, openings.first)) ??
        Vector3(0, 0, 1);
    final normal = openings.isEmpty ? Vector3(0, 0, 1) : n;

    return IjoDocument.mesh(
      IjoOps.hollow(
        solid,
        openings,
        normal,
        wallThicknessMm: opts.positive('thickness'),
      ),
      name: input.name,
    );
  }
}

class ScaleCommand extends IjoCommand {
  ScaleCommand(super.host) {
    argParser
      ..addOption('factor', abbr: 'f', help: 'Uniform scale.')
      ..addOption('by', help: 'Per-axis scale.', valueHelp: 'X,Y,Z')
      ..addOption(
        'about',
        help: 'Fixed point. Defaults to the centroid.',
        valueHelp: 'X,Y,Z',
      );
  }

  @override
  String get name => 'scale';
  @override
  String get description => 'Scale the solid about a point.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      final Vector3 factor;
      if (opts.string('by') != null) {
        factor = opts.vector('by');
      } else {
        final f = opts.positive('factor');
        factor = Vector3(f, f, f);
      }
      if (factor.x == 0 || factor.y == 0 || factor.z == 0) {
        throw CliValueException('A zero scale factor collapses the solid.');
      }
      return IjoDocument.solid(
        BrepXform.scaleAbout(
          brep,
          opts.vector('about', or: IjoMeasure.centroidSolid(brep)),
          factor,
        ),
        name: input.name,
      );
    }
    final solid = input.mesh;
    final Vector3 factor;
    if (opts.string('by') != null) {
      factor = opts.vector('by');
    } else {
      final f = opts.positive('factor');
      factor = Vector3(f, f, f);
    }
    if (factor.x == 0 || factor.y == 0 || factor.z == 0) {
      throw CliValueException('A zero scale factor collapses the solid.');
    }
    return IjoDocument.mesh(
      MeshXform.scaleAbout(
        solid,
        opts.vector('about', or: IjoMeasure.centroid(solid)),
        factor,
      ),
      name: input.name,
    );
  }
}

class MirrorCommand extends IjoCommand {
  MirrorCommand(super.host) {
    argParser
      ..addOption('normal', abbr: 'n', help: 'Plane normal.', valueHelp: 'DIR')
      ..addOption('origin', help: 'Point on the plane.', valueHelp: 'X,Y,Z');
  }

  @override
  String get name => 'mirror';
  @override
  String get description => 'Reflect the solid through a plane.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      return IjoDocument.solid(
        BrepXform.mirror(
          brep,
          opts.vector('origin', or: Vector3.zero()),
          opts.direction('normal'),
        ),
        name: input.name,
      );
    }
    return IjoDocument.mesh(
      MeshXform.mirror(
        input.mesh,
        opts.vector('origin', or: Vector3.zero()),
        opts.direction('normal'),
      ),
      name: input.name,
    );
  }
}

class TranslateCommand extends IjoCommand {
  TranslateCommand(super.host) {
    argParser.addOption(
      'by',
      abbr: 'b',
      help: 'Displacement.',
      valueHelp: 'X,Y,Z',
    );
  }

  @override
  String get name => 'translate';
  @override
  String get description =>
      'Move the solid. Useful for positioning one '
      'solid before a boolean; a host application would normally keep '
      'placement outside the kernel.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      return IjoDocument.solid(
        BrepXform.translate(brep, opts.vector('by')),
        name: input.name,
      );
    }
    return IjoDocument.mesh(
      MeshXform.transformVertices(input.mesh, (p) => p + opts.vector('by')),
      name: input.name,
    );
  }
}

class SliceCommand extends IjoCommand {
  SliceCommand(super.host) {
    argParser
      ..addOption('normal', abbr: 'n', help: 'Plane normal.', valueHelp: 'DIR')
      ..addOption('origin', help: 'Point on the plane.', valueHelp: 'X,Y,Z')
      ..addOption(
        'part',
        abbr: 'p',
        help: 'Which piece to keep, by index. Defaults to the largest.',
        valueHelp: 'N',
      );
  }

  @override
  String get name => 'slice';
  @override
  String get description => 'Cut the solid with a plane and keep one piece.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    final solids = brep == null
        ? null
        : IjoOps.sliceBrep(
            brep,
            opts.vector('origin', or: Vector3.zero()),
            opts.direction('normal'),
          );
    final parts = solids == null
        ? IjoOps.sliceParts(
            input.mesh,
            opts.vector('origin', or: Vector3.zero()),
            opts.direction('normal'),
          )
        : [for (final s in solids) IjoOps.tessellateClosed(s, op: 'Slice')];
    IjoDocument document(int i) => solids == null
        ? IjoDocument.mesh(parts[i], name: input.name)
        : IjoDocument.solid(solids[i], name: input.name);
    if (parts.isEmpty) {
      throw CliValueException('The plane missed the solid; nothing was cut.');
    }

    final wanted = opts.string('part');
    if (wanted == null) {
      // The largest piece, because a slice that shaves a sliver off usually
      // means to keep the body, and one document can hold only one solid.
      var best = 0;
      var bestVolume = double.negativeInfinity;
      for (var i = 0; i < parts.length; i++) {
        final v = IjoMeasure.volume(parts[i]).abs();
        if (v > bestVolume) {
          bestVolume = v;
          best = i;
        }
      }
      host.writeStderr(
        'slice: ${parts.length} piece(s); kept #$best, the largest. '
        'Use --part to choose another.',
      );
      return document(best);
    }

    final index = CliValues.integer(wanted, 'part');
    if (index < 0 || index >= parts.length) {
      throw CliValueException(
        '--part $index is out of range: the cut made ${parts.length} piece(s).',
      );
    }
    return document(index);
  }
}

class LatheCommand extends _PrimitiveCommand {
  LatheCommand(super.host) {
    argParser
      ..addOption(
        'outline',
        abbr: 'l',
        help: 'The turned profile as (radius, height) pairs.',
        valueHelp: 'R,H;R,H;...',
      )
      ..addOption('origin', help: 'Reference point.', valueHelp: 'X,Y,Z')
      ..addOption('axis', help: 'Axis direction.', valueHelp: 'DIR')
      ..addOption('angle', help: 'Degrees of revolution.', defaultsTo: '360');
  }

  @override
  String get name => 'lathe';
  @override
  String get description =>
      'Revolve a (radius, height) outline. Exact: '
      'straight runs become cylinders, cones or discs.';

  @override
  IjoDocument make() => IjoDocument.mesh(
    IjoRevolve.lathe(
      CliValues.outline(
        opts.require('outline', 'It is the shape being turned.'),
        'outline',
      ),
      origin: opts.vector('origin', or: Vector3.zero()),
      axis: opts.direction('axis', or: Vector3(0, 0, 1)),
      angle: _radians(opts.number('angle', or: 360)),
    ),
    name: opts.string('name') ?? name,
  );
}

class RevolveCommand extends _PrimitiveCommand {
  RevolveCommand(super.host) {
    argParser
      ..addOption(
        'profile',
        abbr: 'p',
        help: 'Closed ring of points, coplanar with the axis.',
        valueHelp: 'X,Y,Z;...',
      )
      ..addOption('origin', help: 'Reference point.', valueHelp: 'X,Y,Z')
      ..addOption('axis', help: 'Axis direction.', valueHelp: 'DIR')
      ..addOption('angle', help: 'Degrees of revolution.', defaultsTo: '360');
  }

  @override
  String get name => 'revolve';
  @override
  String get description =>
      'Revolve a closed profile about an axis. Exact, like lathe, but takes '
      'the profile in 3D.';

  @override
  IjoDocument make() => IjoDocument.mesh(
    IjoRevolve.run(
      CliValues.profile(
        opts.require('profile', 'It is the section being revolved.'),
        'profile',
      ),
      opts.vector('origin', or: Vector3.zero()),
      opts.direction('axis', or: Vector3(0, 0, 1)),
      angle: _radians(opts.number('angle', or: 360)),
    ),
    name: opts.string('name') ?? name,
  );
}

class SweepCommand extends _PrimitiveCommand {
  SweepCommand(super.host) {
    argParser
      ..addOption(
        'profile',
        abbr: 'p',
        help: 'Closed ring of points to carry along the path.',
        valueHelp: 'X,Y,Z;...',
      )
      ..addOption('path', help: 'Polyline to follow.', valueHelp: 'X,Y,Z;...')
      ..addOption('twist', help: 'Total twist in degrees.', defaultsTo: '0')
      ..addFlag('closed', help: 'Join the path end to end.', negatable: false);
  }

  @override
  String get name => 'sweep';
  @override
  String get description => 'Sweep a profile along a polyline. Faceted.';

  @override
  IjoDocument make() => IjoDocument.mesh(
    IjoSweep.path(
      CliValues.profile(
        opts.require('profile', 'It is the section being swept.'),
        'profile',
      ),
      CliValues.points(
        opts.require('path', 'It is the trajectory to follow.'),
        'path',
      ),
      closed: opts.flag('closed'),
      twistRadians: _radians(opts.number('twist', or: 0)),
    ),
    name: opts.string('name') ?? name,
  );
}

class PipeCommand extends _PrimitiveCommand {
  PipeCommand(super.host) {
    argParser
      ..addOption('path', help: 'Polyline to follow.', valueHelp: 'X,Y,Z;...')
      ..addOption('radius', abbr: 'r', help: 'Outer radius.')
      ..addOption('inner', help: 'Bore radius, for a tube.')
      ..addOption('sides', abbr: 'n', help: 'Facets around.', defaultsTo: '32')
      ..addFlag('closed', help: 'Join the path end to end.', negatable: false);
  }

  @override
  String get name => 'pipe';
  @override
  String get description => 'A round pipe or tube along a polyline.';

  @override
  IjoDocument make() {
    final path = CliValues.points(
      opts.require('path', 'It is the route the pipe takes.'),
      'path',
    );
    final outer = opts.positive('radius');
    final inner = opts.string('inner') == null ? null : opts.positive('inner');
    if (inner != null && inner >= outer) {
      throw CliValueException(
        '--inner ($inner) must be smaller than --radius ($outer).',
      );
    }
    _sides(opts.integer('sides', or: 32));
    final out = IjoOps.pipePreferSolid(
      path,
      outerRadius: outer,
      innerRadius: inner,
      closed: opts.flag('closed'),
    );
    return IjoDocument.solid(out.solid!, name: opts.string('name') ?? name);
  }
}

class LoftCommand extends _PrimitiveCommand {
  LoftCommand(super.host) {
    argParser
      ..addMultiOption(
        'section',
        abbr: 's',
        help: 'A closed ring of points. Repeat for each section, in order.',
        valueHelp: 'X,Y,Z;...',
        // A section's own syntax uses commas, so args must not treat them as
        // separators between repeated values.
        splitCommas: false,
      )
      ..addFlag(
        'closed',
        help: 'Loop back from the last section to the first.',
        negatable: false,
      );
  }

  @override
  String get name => 'loft';
  @override
  String get description =>
      'Loft through parallel planar sections as a BREP solid.';

  @override
  IjoDocument make() {
    final raw = opts.multi('section');
    if (raw.length < 2) {
      throw CliValueException(
        'loft needs at least two --section values, got ${raw.length}.',
      );
    }
    return IjoDocument.solid(
      IjoOps.loftPreferSolid([
        for (var i = 0; i < raw.length; i++)
          CliValues.profile(raw[i], 'section'),
      ], closed: opts.flag('closed')).solid!,
      name: opts.string('name') ?? name,
    );
  }
}

class ExtrudeCommand extends IjoCommand {
  ExtrudeCommand(super.host) {
    argParser
      ..addOption(
        'faces',
        abbr: 'f',
        help: 'Planar faces to push. ${CliSelectors.faceHelp}',
        defaultsTo: 'top',
        valueHelp: 'SPEC',
      )
      ..addOption('distance', abbr: 'd', help: 'How far, along the normal.');
  }

  @override
  String get name => 'extrude';
  @override
  String get description =>
      'Push planar faces along their own normal, adding or removing material.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      final faces = CliSelectors.brepFaces(
        opts.string('faces') ?? 'top',
      ).resolve(brep);
      if (faces.isEmpty) throw CliValueException('No faces matched');
      final distance = opts.number('distance');
      if (distance == 0)
        throw CliValueException('--distance 0 would change nothing.');
      var out = brep;
      // Each step resolves the original face by provenance after topology changes.
      final origins = [for (final i in faces) brep.faces[i].origin];
      if (faces.length > 1 && origins.any((origin) => origin == null)) {
        throw BrepUnsupported(
          'Extruding multiple faces requires stable face provenance',
        );
      }
      for (var i = 0; i < faces.length; i++) {
        final origin = origins[i];
        final matches = [
          for (var j = 0; j < out.faces.length; j++)
            if (out.faces[j].origin?.toJson().toString() ==
                origin?.toJson().toString())
              j,
        ];
        final index = i == 0
            ? faces[i]
            : (matches.length == 1 ? matches.single : -1);
        if (index < 0)
          throw BrepUnsupported('Extrude could not track the selected face');
        out =
            BrepExtrude.tryMovePlanarFace(out, index, distance) ??
            BrepExtrude.attachFromFace(
              out,
              index,
              out.faces[index].outwardNormal(0, 0) * distance,
            );
      }
      IjoOps.tessellateClosed(out, op: 'Extrude');
      return IjoDocument.solid(out, name: input.name);
    }
    final solid = input.mesh;
    final faces = CliSelectors.faces(
      opts.string('faces') ?? 'top',
    ).resolve(solid);
    if (faces.isEmpty) {
      throw CliValueException('No faces matched "${opts.string('faces')}".');
    }
    final distance = opts.number('distance');
    if (distance == 0) {
      throw CliValueException('--distance 0 would change nothing.');
    }

    var out = solid;
    for (final index in faces) {
      out = _push(out, index, distance);
    }
    return IjoDocument.mesh(out, name: input.name);
  }

  static MeshData _push(MeshData solid, int index, double distance) {
    final n = EdgeGeometry.faceNormal(solid, index);
    if (n == null) return solid;
    final region = MeshFaceRegions.selectionRegion(solid, index);
    final tool = MeshBuild.extrudeRegion(
      mesh: solid,
      region: region,
      direction: n * distance,
    );
    if (tool.faces.isEmpty) return solid;
    return IjoOps.boolean(
      solid,
      tool,
      distance > 0 ? IjoOps.unionOp : IjoOps.subtractOp,
    );
  }
}

class ThreadsCommand extends IjoCommand {
  ThreadsCommand(super.host) {
    argParser
      ..addOption(
        'pitch',
        abbr: 'p',
        help: 'Axial spacing between thread crests in millimetres.',
      )
      ..addOption(
        'taper',
        help: 'Radial change per axial millimetre (BREP only).',
        defaultsTo: '0',
      )
      ..addOption(
        'runout',
        help: 'Length over which the groove fades at each end (BREP only).',
        defaultsTo: '0',
      )
      ..addOption('diameter', abbr: 'd', help: 'Major diameter.')
      ..addOption(
        'from',
        help: 'Which part of the axis to thread. "whole" ignores --length.',
        allowed: ['whole', 'a', 'b', 'both'],
        defaultsTo: 'whole',
      )
      ..addOption(
        'length',
        abbr: 'l',
        help: 'Threaded length, measured from the end named by --from.',
      )
      ..addOption('origin', help: 'A point on the axis.', valueHelp: 'X,Y,Z')
      ..addOption('axis', help: 'Axis direction.', valueHelp: 'DIR')
      ..addOption('starts', help: 'Thread starts.', defaultsTo: '1')
      ..addFlag('internal', help: 'Thread a bore.', negatable: false)
      ..addFlag('left-hand', help: 'Left-hand helix.', negatable: false);
  }

  @override
  String get name => 'threads';
  @override
  String get description => 'Cut a helical thread into a shaft or a bore.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final axis = opts.direction('axis', or: Vector3(0, 0, 1));
    final origin = opts.vector('origin', or: Vector3.zero());
    final brep = input!.solidOrNull;
    final x = brepFrameU(axis), y = axis.cross(x);
    Vector3 local(Vector3 p) {
      final d = p - origin;
      return Vector3(d.dot(x), d.dot(y), d.dot(axis));
    }

    // Measure in the thread frame. Projecting the world AABB would enlarge
    // the axial interval on a rotated shaft and misplace partial threads.
    final bounds = brep != null
        ? solidBounds(BrepXform.map(brep, local))
        : IjoBounds.fromPoints(input.mesh.vertices.map(local));
    if (bounds == null) throw CliValueException('The solid is empty.');
    final lo = bounds.min.z, hi = bounds.max.z;

    final diameter = opts.positive('diameter');
    final from = opts.string('from') ?? 'whole';
    final length = opts.positive('length', or: hi - lo);
    if (from == 'whole' && opts.string('length') != null) {
      host.writeStderr(
        'threads: --length is ignored with --from whole, which threads the '
        'full ${(hi - lo).toStringAsFixed(2)}mm along the axis.',
      );
    }

    final taper = opts.number('taper'),
        runout = opts.number('runout'),
        pitch = opts.positive('pitch');
    if (brep != null) {
      final radius =
          diameter / 2 -
          (opts.flag('internal')
              ? ThreadForm(pitch: pitch, internal: true).depth
              : 0);
      return IjoDocument.solid(
        BrepThreads.apply(
          brep,
          origin: origin,
          axis: axis,
          radius: radius,
          pitch: pitch,
          axisZMin: lo,
          axisZMax: hi,
          taper: taper,
          runoutLength: runout,
          length: length,
          starts: opts.integer('starts', or: 1),
          rightHand: !opts.flag('left-hand'),
          internal: opts.flag('internal'),
          fromEndA: from == 'a' || from == 'both',
          fromEndB: from == 'b' || from == 'both',
        ),
        name: input.name,
      );
    }
    if (taper != 0 || runout != 0)
      throw CliValueException('Taper and runout require a BREP input');
    return IjoDocument.mesh(
      IjoOps.threads(
        base: input.mesh,
        origin: origin,
        axis: axis,
        majorRadius: diameter / 2,
        holeRadius: diameter / 2,
        axisZMin: lo,
        axisZMax: hi,
        pitchMm: opts.positive('pitch'),
        lengthMm: length,
        starts: opts.integer('starts', or: 1),
        rightHand: !opts.flag('left-hand'),
        internal: opts.flag('internal'),
        fromEndA: from == 'a' || from == 'both',
        fromEndB: from == 'b' || from == 'both',
      ),
      name: input.name,
    );
  }
}
double _radians(double degrees) => degrees * math.pi / 180;

// ── queries ──────────────────────────────────────────────────────────────────

/// Shared plumbing for the commands that report rather than transform.
abstract class _QueryCommand extends IjoCommand {
  _QueryCommand(super.host) {
    argParser.addFlag(
      'json',
      help: 'Report as JSON rather than as text.',
      negatable: false,
    );
  }

  @override
  bool get writesOutput => false;
}

class MeasureCommand extends _QueryCommand {
  MeasureCommand(super.host);

  @override
  String get name => 'measure';
  @override
  String get description =>
      'Volume, surface area, centroid and bounds of the triangle mesh.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    final props = brep != null
        ? IjoMeasure.ofSolid(brep)
        : IjoMeasure.of(input.mesh);
    final b = props.bounds;
    final faceCount = brep?.faces.length ?? input.mesh.faces.length;

    String v(Vector3 p) => '${_f(p.x)}, ${_f(p.y)}, ${_f(p.z)}';
    report(props.toJson(), [
      'volume        ${_f(props.volume)} mm³',
      'surface area  ${_f(props.surfaceArea)} mm²',
      'centroid      ${v(props.centroid)}',
      if (b != null) 'bounds min    ${v(b.min)}',
      if (b != null) 'bounds max    ${v(b.max)}',
      if (b != null) 'size          ${v(b.size)}',
      'sphericity    ${_f(props.sphericity)}',
      'faces         $faceCount',
      if (!props.isExact)
        'note          triangle-mesh properties, not closed-form surfaces',
    ]);
    return null;
  }

  static String _f(double x) => x.abs() < 1e-12 ? '0' : x.toStringAsFixed(4);
}

class SelectCommand extends _QueryCommand {
  SelectCommand(super.host) {
    argParser
      ..addOption(
        'faces',
        abbr: 'f',
        help: 'Face selector. ${CliSelectors.faceHelp}',
        valueHelp: 'SPEC',
      )
      ..addOption(
        'edges',
        abbr: 'e',
        help: 'Edge selector. ${CliSelectors.edgeHelp}',
        valueHelp: 'SPEC',
      );
  }

  @override
  String get name => 'select';
  @override
  String get description =>
      'Resolve a selector to indices. Use it to check '
      'a selector before feeding it to fillet or hollow.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final solid = input!.mesh;
    final faceSpec = opts.string('faces');
    final edgeSpec = opts.string('edges');
    if (faceSpec == null && edgeSpec == null) {
      throw CliValueException('Pass --faces or --edges, or both.');
    }

    final faces = faceSpec == null
        ? null
        : CliSelectors.faces(faceSpec).resolve(solid);
    final edges = edgeSpec == null
        ? null
        : CliSelectors.edges(edgeSpec).resolve(solid);

    report(
      {
        if (faces != null) 'faces': faces,
        if (edges != null)
          'edges': [
            for (final e in edges) [e.$1, e.$2],
          ],
      },
      [
        if (faces != null)
          'faces (${faces.length}/${meshCadFaces(solid).length})  ${faces.join(',')}',
        if (edges != null) 'edges (${edges.length})  ${edges.join(',')}',
      ],
    );
    return null;
  }
}

class InfoCommand extends _QueryCommand {
  InfoCommand(super.host);

  @override
  String get name => 'info';
  @override
  String get description => 'Topology and surface breakdown.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final mesh = input!.mesh;
    report(
      {
        'kind': 'mesh',
        'vertices': mesh.vertices.length,
        'faces': mesh.faces.length,
        'closed': MeshKit.isClosedSolid(mesh),
        'volume': IjoMeasure.volume(mesh),
      },
      [
        'kind      mesh',
        'vertices  ${mesh.vertices.length}',
        'faces     ${mesh.faces.length}',
        'closed    ${MeshKit.isClosedSolid(mesh)}',
        'volume    ${IjoMeasure.volume(mesh)}',
      ],
    );
    return null;
  }
}

class HealCommand extends IjoCommand {
  HealCommand(super.host);

  @override
  String get name => 'heal';
  @override
  String get description =>
      'Merge coplanar faces, collapse collinear edges, orient outward.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final brep = input!.solidOrNull;
    if (brep != null) {
      final after = BrepHeal.run(brep);
      host.writeStderr(
        'heal: ${brep.faces.length} faces -> ${after.faces.length}',
      );
      return IjoDocument.solid(after, name: input.name);
    }
    final before = input.mesh;
    final after = MeshSeal.unifyCoplanarFaces(before);
    host.writeStderr(
      'heal: ${before.faces.length} faces -> ${after.faces.length}',
    );
    return IjoDocument.mesh(after, name: input.name);
  }
}

class TessellateCommand extends IjoCommand {
  TessellateCommand(super.host) {
    argParser.addOption(
      'quality',
      abbr: 'q',
      help: 'Facet density on curved surfaces.',
      allowed: ['draft', 'standard', 'fine'],
      defaultsTo: 'standard',
    );
  }

  @override
  String get name => 'tessellate';
  @override
  String get description =>
      'Turn the analytic solid into triangles. One-way: the exact surfaces '
      'do not come back.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async =>
      IjoDocument.mesh(input!.mesh, name: input.name);
}

class ExportCommand extends IjoCommand {
  ExportCommand(super.host) {
    argParser.addOption(
      'format',
      abbr: 'f',
      help: 'Override the format implied by --out.',
      allowed: [for (final f in IjoFormat.values) f.extension],
    );
  }

  @override
  String get name => 'export';
  @override
  String get description =>
      'Write the solid in another format. 3MF is the best mesh format here; '
      'STL is the most widely read and the most lossy.';

  @override
  Future<IjoDocument?> build(IjoDocument? input) async {
    final explicit = opts.string('format');
    final path = opts.string('out');
    if (explicit == null && path == null) {
      throw CliValueException(
        'export needs --out FILE, or --format to write to stdout.',
      );
    }

    final format = explicit != null
        ? IjoFormat.parse(explicit)
        : IjoFormat.parse(path!);

    final bytes = IjoIo.encode(
      opts.string('name') == null
          ? input!
          : IjoCommand._renamed(input!, opts.string('name')!),
      format,
    );
    if (path != null) {
      await host.writeFile(path, bytes);
    } else if (format.isText) {
      host.writeStdout(utf8.decode(bytes));
    } else {
      host.writeStdoutBytes(bytes);
    }
    // Written here rather than by the base class, since export is the one
    // command whose output format is its whole purpose.
    return null;
  }
}
