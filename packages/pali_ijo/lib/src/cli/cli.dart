/// The `pali_ijo` command line, minus its `dart:io` entry point.
library;

import 'package:args/args.dart';
import 'package:args/command_runner.dart';

import '../ijo_log.dart';
import '../mesh/mesh_op.dart';
import 'commands.dart';
import 'io_host.dart';
import 'values.dart';

export 'commands.dart';
export 'io_host.dart';
export 'values.dart';

/// The command line, as a library.
///
/// Two reasons this is not simply `main`. It is testable — see [MemoryHost] —
/// and it has no `dart:io`, so the same commands compile for a WasmGC target
/// where a host would be backed by browser APIs instead of files.
class IjoCli {
  IjoCli(this.host) : runner = _buildRunner(host);

  final IjoHost host;
  final CommandRunner<int> runner;

  static const description = 'pali ijo — an analytic B-Rep CAD kernel.\n\n'
      'Commands read a solid from stdin and write one to stdout, so they '
      'compose:\n\n'
      '  pali_ijo box --size 40,40,20 \\\n'
      '    | pali_ijo fillet --radius 4 --edges convex \\\n'
      '    | pali_ijo hollow --thickness 2 --faces top \\\n'
      '    | pali_ijo export --out part.3mf\n\n'
      'Solid JSON keeps the exact surfaces; a mesh format does not. Keep '
      'JSON between stages and export once at the end.\n\n'
      'Debug with -v (or IJO_DEBUG=1). Breadcrumbs go to stderr so they '
      'cannot corrupt the pipeline. The last line is the step that was '
      'running if the kernel crashes.';

  static CommandRunner<int> _buildRunner(IjoHost host) {
    final runner = CommandRunner<int>('pali_ijo', description);
    runner.argParser.addFlag(
      'verbose',
      abbr: 'v',
      help: 'Kernel breadcrumbs on stderr (also IJO_DEBUG=1).',
      negatable: false,
    );

    for (final command in [
      // Primitives: make a solid from nothing.
      BoxCommand(host),
      CylinderCommand(host),
      SphereCommand(host),
      ConeCommand(host),
      TorusCommand(host),
      PrismCommand(host),
      PyramidCommand(host),
      // Sweeps: also make a solid from nothing, from a profile and a path.
      LatheCommand(host),
      RevolveCommand(host),
      SweepCommand(host),
      PipeCommand(host),
      LoftCommand(host),
      // Operations: take a solid, give a solid.
      BooleanCommand(host),
      MinkowskiCommand(host),
      OffsetCommand(host),
      FilletCommand(host),
      ChamferCommand(host),
      HollowCommand(host),
      ExtrudeCommand(host),
      ThreadsCommand(host),
      ScaleCommand(host),
      MirrorCommand(host),
      TranslateCommand(host),
      SliceCommand(host),
      HealCommand(host),
      TessellateCommand(host),
      // Queries: take a solid, report.
      MeasureCommand(host),
      SelectCommand(host),
      InfoCommand(host),
      ExportCommand(host),
    ]) {
      runner.addCommand(command);
    }
    return runner;
  }

  /// Runs [args] and returns the process exit code.
  ///
  /// Nothing is allowed to escape as an exception, because a CLI's job on
  /// failure is a message and a non-zero code, not a stack trace. The
  /// exception types are separated so each gets the treatment it deserves:
  ///
  /// - An unknown flag gets the usage text, since the user needs the list.
  /// - A malformed value gets the message alone, since the usage text would
  ///   bury it.
  /// - Geometry the kernel cannot do gets [MeshOpException]'s own wording,
  ///   which is written for exactly this.
  Future<int> run(List<String> args) async {
    final verbose = _wantsVerbose(args);
    final previousSink = IjoLog.sink;
    IjoLog.last = null;
    IjoLog.sink = host.writeStderr;
    if (verbose) {
      IjoLog.enabled = true;
      IjoLog.verbose = true;
    }
    try {
      return await runner.run(args) ?? 0;
    } on UsageException catch (e) {
      host.writeStderr(e.toString());
      return 64; // EX_USAGE
    } on CliValueException catch (e) {
      host.writeStderr('pali_ijo: ${e.message}');
      return 64;
    } on CliFileException catch (e) {
      host.writeStderr('pali_ijo: ${e.message}');
      return 66; // EX_NOINPUT
    } on ArgParserException catch (e) {
      host.writeStderr('pali_ijo: ${e.message}');
      return 64;
    } on FormatException catch (e) {
      host.writeStderr('pali_ijo: ${e.message}');
      return 65; // EX_DATAERR
    } on MeshOpException catch (e) {
      IjoLog.fail(_opName(args), e);
      host.writeStderr('pali_ijo: ${e.message}');
      return 70; // EX_SOFTWARE
    } on UnsupportedError catch (e, st) {
      IjoLog.fail(_opName(args), e, st);
      host.writeStderr('pali_ijo: ${e.message}');
      return 70;
    } on ArgumentError catch (e) {
      host.writeStderr('pali_ijo: ${e.message}');
      return 64;
    } on Object catch (e, st) {
      IjoLog.fail(_opName(args), e, st);
      host.writeStderr('pali_ijo: $e');
      return 70;
    } finally {
      IjoLog.sink = previousSink;
      if (verbose) {
        IjoLog.enabled = null;
        IjoLog.verbose = null;
      }
    }
  }

  static String _opName(List<String> args) {
    for (final a in args) {
      if (a == '--') break;
      if (!a.startsWith('-')) return a;
    }
    return 'cli';
  }

  static bool _wantsVerbose(List<String> args) {
    for (final a in args) {
      if (a == '--') return false;
      if (a == '-v' || a == '--verbose') return true;
    }
    return false;
  }
}
