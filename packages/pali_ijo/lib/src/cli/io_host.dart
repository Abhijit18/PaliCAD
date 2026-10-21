import 'dart:convert';
import 'dart:typed_data';

/// Everything the CLI does that is not geometry: files, stdin, stdout, stderr.
///
/// This exists so the commands can be tested without spawning a process. A
/// test drives them through [MemoryHost] and reads the bytes back out, which
/// takes a CLI test from seconds of subprocess launch to microseconds, and
/// means the tests can assert on exact output rather than scraping a
/// terminal.
///
/// It is also the only reason `dart:io` appears in exactly one file of this
/// package. A WasmGC build has no `dart:io` at all, so keeping the boundary
/// this narrow is what lets the same command code compile for the web.
abstract class IjoHost {
  Future<Uint8List> readFile(String path);
  Future<void> writeFile(String path, List<int> bytes);
  Future<Uint8List> readStdin();

  /// Writes to stdout with a trailing newline.
  void writeStdout(String text);

  /// Writes raw bytes to stdout, for a binary export with no `--out`.
  void writeStdoutBytes(List<int> bytes);

  /// Progress and warnings. Never stdout, which belongs to the pipeline.
  void writeStderr(String text);
}

/// An [IjoHost] backed by maps and buffers.
class MemoryHost implements IjoHost {
  MemoryHost({Map<String, List<int>>? files, List<int>? stdin})
      : files = {
          for (final e in (files ?? const {}).entries)
            e.key: Uint8List.fromList(e.value),
        },
        _stdin = stdin == null ? null : Uint8List.fromList(stdin);

  /// The filesystem, before and after. Writes land here.
  final Map<String, Uint8List> files;

  final Uint8List? _stdin;

  final _stdout = BytesBuilder();
  final _stderr = StringBuffer();

  /// Everything written to stdout, as bytes.
  Uint8List get stdoutBytes => _stdout.toBytes();

  /// Everything written to stdout, decoded. Malformed sequences are allowed
  /// through so a test that accidentally captures a binary export sees
  /// mojibake rather than an exception from the assertion itself.
  String get stdout => utf8.decode(stdoutBytes, allowMalformed: true);

  String get stderr => _stderr.toString();

  /// Puts text where a following [readStdin] will find it, for chaining one
  /// command's output into the next.
  MemoryHost then(String input) => MemoryHost(
        files: files,
        stdin: utf8.encode(input),
      );

  @override
  Future<Uint8List> readFile(String path) async =>
      files[path] ??
      (throw CliFileException('No such file: $path'));

  @override
  Future<void> writeFile(String path, List<int> bytes) async {
    files[path] = Uint8List.fromList(bytes);
  }

  @override
  Future<Uint8List> readStdin() async => _stdin ?? Uint8List(0);

  @override
  void writeStdout(String text) => _stdout.add(utf8.encode('$text\n'));

  @override
  void writeStdoutBytes(List<int> bytes) => _stdout.add(bytes);

  @override
  void writeStderr(String text) => _stderr.writeln(text);
}

/// A file could not be read or written. Held separately from
/// `CliValueException` because the argument was fine and the filesystem was
/// not, which is a different thing to tell the user.
class CliFileException implements Exception {
  CliFileException(this.message);
  final String message;
  @override
  String toString() => message;
}
