/// The WasmGC entry point: the kernel, callable from JavaScript.
///
/// Built by `tool/build_wasm.sh`. This is the third of the three ways to
/// consume pali ijo, alongside importing the Dart package and running the
/// native executable — and the one a browser-based designer would use.
///
/// What it exposes is deliberately the *same* surface as the command line
/// rather than a hand-written binding per operation. One function that takes
/// an argument list and a document and returns a document means there is one
/// API to document, one to test, and nothing to fall out of step when an
/// operation gains a flag.
///
/// This file lives outside `lib/` because it is the only part of the package
/// that imports `dart:js_interop`, which has no meaning on the VM. Everything
/// it calls is plain Dart.
library;

import 'dart:convert';
import 'dart:js_interop';
import 'dart:js_interop_unsafe';
import 'dart:typed_data';

import 'package:pali_ijo/pali_ijo.dart';
import 'package:pali_ijo/src/cli/cli.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  final api = JSObject()
    ..setProperty('run'.toJS, _run.toJS)
    ..setProperty('runBytes'.toJS, _runBytes.toJS)
    ..setProperty('importAll'.toJS, _importAll.toJS)
    ..setProperty('extrudeProfile'.toJS, _extrudeProfile.toJS)
    ..setProperty('formats'.toJS, _formats.toJS);
  globalContext.setProperty('paliIjo'.toJS, api);
}

/// Runs one command.
///
/// From JavaScript:
///
/// ```js
/// const box = await paliIjo.run(['box', '--size', '40,40,20'], null);
/// const tool = await paliIjo.run(['cylinder', '--radius', '4', '--height', '12'], null);
/// const part = await paliIjo.run(
///   ['boolean', '--op', 'subtract', '--with', 'tool.json'],
///   box.stdout,
///   tool.stdout,
/// );
/// const info = await paliIjo.run(['measure', '--json'], part.stdout);
/// console.log(JSON.parse(info.stdout).volume);
/// ```
///
/// Returns `{code, stdout, stderr, files}`. A non-zero `code` means the
/// command failed and `stderr` says why; nothing throws, matching how the
/// same command behaves in a shell.
///
/// `files` carries anything written with `--out`, since a browser has no
/// filesystem to write it to. Its values are `Uint8Array`, so a binary export
/// survives — `paliIjo.run(['export', '--out', 'part.3mf'], solid)` then
/// `result.files['part.3mf']` is bytes ready for a `Blob`.
///
/// `runBytes(argv, bytes)` runs the same command with binary stdin, for
/// importing STEP, STL, 3MF and OBJ data without converting it through text.
JSPromise<JSObject> _run(
  JSArray<JSString> argv,
  JSString? stdin, [
  JSString? withDocument,
]) {
  final args = [
    for (final a in argv.toDart) a.toDart,
  ];
  return _execute(
    args,
    stdin == null ? null : utf8.encode(stdin.toDart),
    withDocument?.toDart,
  ).toJS;
}

/// Runs a command with binary stdin, for example an imported STEP or STL file.
JSPromise<JSObject> _runBytes(
  JSArray<JSString> argv,
  JSUint8Array stdin, [
  JSString? withDocument,
]) {
  final args = [
    for (final a in argv.toDart) a.toDart,
  ];
  return _execute(args, stdin.toDart, withDocument?.toDart).toJS;
}

JSPromise<JSString> _importAll(JSUint8Array bytes, JSString name) =>
    _importAllAsync(bytes, name).toJS;

Future<JSString> _importAllAsync(JSUint8Array bytes, JSString name) async {
  final documents = IjoIo.decodeAll(bytes.toDart, name: name.toDart);
  return jsonEncode(documents.map((document) => document.toJson()).toList()).toJS;
}

JSString _extrudeProfile(JSString profileJson, JSNumber height, JSString name) {
  final decoded = jsonDecode(profileJson.toDart);
  if (decoded is! List || decoded.length < 3) {
    throw FormatException('An extrusion profile needs at least three points.');
  }
  final ring = <Vector3>[];
  for (final point in decoded) {
    if (point is! List ||
        point.length != 2 ||
        point.any((value) => value is! num || !value.toDouble().isFinite)) {
      throw FormatException('Extrusion profile points must contain finite X and Y coordinates.');
    }
    ring.add(Vector3((point[0] as num).toDouble(), (point[1] as num).toDouble(), 0));
  }
  final depth = height.toDartDouble;
  if (!depth.isFinite || depth <= 0) {
    throw FormatException('Extrusion height must be a positive finite number.');
  }
  final solid = BrepExtrude.polygon(ring, Vector3(0, 0, depth));
  IjoOps.tessellateClosed(solid, op: 'Sketch extrusion');
  return IjoDocument.solid(solid, name: name.toDart).encode().toJS;
}

Future<JSObject> _execute(
  List<String> args,
  Uint8List? stdin,
  String? withDocument,
) async {
  final host = MemoryHost(
    files: withDocument == null ? null : {'tool.json': utf8.encode(withDocument)},
    stdin: stdin,
  );
  final code = await IjoCli(host).run(args);
  final files = JSObject();
  for (final entry in host.files.entries) {
    files.setProperty(entry.key.toJS, entry.value.toJS);
  }
  return JSObject()
    ..setProperty('code'.toJS, code.toJS)
    ..setProperty('stdout'.toJS, host.stdout.toJS)
    ..setProperty('stderr'.toJS, host.stderr.toJS)
    ..setProperty('files'.toJS, files);
}

/// The file extensions `--out` and `export --format` understand.
JSArray<JSString> _formats() => [
      for (final f in IjoFormat.values) f.extension.toJS,
    ].toJS;

extension on Uint8List {
  JSUint8Array get toJS => JSUint8Array.withLength(length)
    ..toDart.setAll(0, this);
}
