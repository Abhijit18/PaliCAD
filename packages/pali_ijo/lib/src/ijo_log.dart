import 'dart:io';
import 'dart:isolate';

/// Diagnostic breadcrumbs for the pali ijo kernel.
///
/// Writes to **stderr**, tagged with the isolate name, so the last line before
/// a hang or a crash names the operation that was running. stderr matters: the
/// CLI writes solid JSON to stdout, and a stray log line would corrupt it.
///
/// Off by default, because a library has no business writing to a host
/// application's console uninvited. Enable with `IJO_DEBUG=1`, `pali_ijo -v`,
/// or from Dart with [enabled]. `IJO_DEBUG=2` (or [verbose]) prints every CSG
/// clip step; otherwise the console only gets begin/done/fail. Set [logFile]
/// (or `IJO_DEBUG_FILE`) to also append the same lines to a session file.
///
/// [line] always records [last] and [recent], even when printing is off, so a
/// crash handler can still name the step that was running.
class IjoLog {
  IjoLog._();

  static const _ringMax = 400;
  static const _fileMaxBytes = 2 * 1024 * 1024;

  static bool? _override;
  static bool? _verboseOverride;
  static String? _logFile;
  static int _fileWrites = 0;
  static final List<String> _ring = [];

  /// Last [line] recorded, whether or not it was printed.
  static String? last;

  /// Recent lines (timestamped), newest last. Always recorded.
  static List<String> get recent => List.unmodifiable(_ring);

  /// Redirect output. The CLI sets this to the host's stderr so tests and
  /// Wasm see the same breadcrumbs as a terminal. Null writes `dart:io`
  /// stderr, which is what the app isolate wants.
  static void Function(String line)? sink;

  /// Session log path. Isolates do not inherit statics — pass this into the
  /// isolate and call [attachFile] there. `IJO_DEBUG_FILE` is the fallback.
  static String? get logFile => _logFile ?? _envFile;

  static String? get _envFile {
    final env = Platform.environment['IJO_DEBUG_FILE'];
    if (env == null || env.isEmpty) return null;
    return env;
  }

  /// Truncate [path] and start a new session. Host isolate only.
  static void beginFile(String path) {
    _logFile = path;
    enabled = true;
    try {
      final f = File(path);
      f.parent.createSync(recursive: true);
      f.writeAsStringSync('');
    } catch (_) {}
  }

  /// Point this isolate at an existing session file (append, do not truncate).
  static void attachFile(String path) {
    if (path.isEmpty) return;
    _logFile = path;
    enabled = true;
  }

  /// Stop teeing to disk. Used by tests.
  static void detachFile() {
    _logFile = null;
  }

  /// Force logging on or off, ignoring the environment. Pass null to go back to
  /// reading `IJO_DEBUG`.
  static set enabled(bool? value) => _override = value;

  /// Per-step CSG / heal breadcrumbs on the console. Summary lines still print
  /// when [enabled] is true. `IJO_DEBUG=2` or `pali_ijo -v` turns this on.
  static set verbose(bool? value) => _verboseOverride = value;

  static bool get enabled {
    final forced = _override;
    if (forced != null) return forced;
    final fromEnv = Platform.environment['IJO_DEBUG'] ??
        // Retained for PaliCAD run configurations that predate the split.
        Platform.environment['PALICAD_IJO_DEBUG'];
    if (fromEnv == null || fromEnv.isEmpty) return false;
    return fromEnv != '0';
  }

  static bool get verbose {
    final forced = _verboseOverride;
    if (forced != null) return forced;
    final fromEnv = Platform.environment['IJO_DEBUG'] ??
        Platform.environment['PALICAD_IJO_DEBUG'];
    return fromEnv == '2' || fromEnv == 'verbose';
  }

  static String get where {
    final name = Isolate.current.debugName;
    if (name == null || name.isEmpty) return 'main';
    return name;
  }

  static void line(String message) {
    last = message;
    _emit('[ijo:$where] $message', toConsole: enabled);
  }

  /// Same ring/file trail as [line], but the console only prints it when
  /// [verbose] is on. Use for BSP clip steps, T-junction walks, preview ticks.
  static void trace(String message) {
    last = message;
    _emit('[ijo:$where] $message', toConsole: enabled && verbose);
  }

  static void banner(String title) {
    line('======== $title ========');
  }

  /// Failures are always reported, since losing the reason a solid broke costs
  /// far more than a line of stderr.
  static void fail(String op, Object error, [StackTrace? stack]) {
    final prior = last;
    if (prior != null && prior.isNotEmpty) {
      _emit('[ijo:$where] last: $prior', toConsole: true);
    }
    _emit('[ijo:$where] ERROR $op: $error', toConsole: true);
    if (stack != null) {
      _emit('[ijo:$where] $stack', toConsole: true);
    }
  }

  static void _emit(String text, {required bool toConsole}) {
    final stamped = '${DateTime.now().toIso8601String()} $text';
    _ring.add(stamped);
    if (_ring.length > _ringMax) {
      _ring.removeRange(0, _ring.length - _ringMax);
    }
    if (toConsole) {
      final out = sink;
      if (out != null) {
        out(text);
      } else {
        stderr.writeln(stamped);
      }
    }
    _appendFile(stamped);
  }

  static void _appendFile(String stamped) {
    final path = logFile;
    if (path == null) return;
    try {
      final f = File(path);
      if (!f.parent.existsSync()) f.parent.createSync(recursive: true);
      _fileWrites++;
      if (_fileWrites % 64 == 0 &&
          f.existsSync() &&
          f.lengthSync() > _fileMaxBytes) {
        final all = f.readAsStringSync();
        final keepFrom = all.length * 3 ~/ 4;
        final cut = all.lastIndexOf('\n', keepFrom);
        f.writeAsStringSync(
          '… truncated older kernel log …\n'
          '${cut >= 0 ? all.substring(cut + 1) : all.substring(keepFrom)}',
        );
      }
      f.writeAsStringSync('$stamped\n', mode: FileMode.append);
    } catch (_) {}
  }

  static String size(int verts, int faces) => '${verts}v/${faces}f';

  static String ms(Stopwatch sw) => '${sw.elapsedMilliseconds}ms';
}
