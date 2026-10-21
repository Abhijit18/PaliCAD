import 'dart:io';

import 'package:test/test.dart';
import 'package:yaml/yaml.dart';

/// pali ijo has to keep standing on its own. Nothing shipped in the package may
/// reach for Flutter or back into the PaliCAD application, because the kernel
/// also has to run as a CLI, in a plain Dart isolate, and compiled to WasmGC.
///
/// This is the check that stops the coupling creeping back the next time
/// someone reaches for a convenient app type.
void main() {
  final shipped = [
    for (final dir in ['lib', 'bin'])
      ...Directory(dir)
          .listSync(recursive: true)
          .whereType<File>()
          .where((f) => f.path.endsWith('.dart')),
  ];

  test('there is source to check', () {
    expect(shipped, isNotEmpty);
  });

  group('shipped source stays independent', () {
    const forbidden = {
      'package:flutter/': 'Flutter is not available to a CLI or a plain isolate',
      'package:palicad/': 'the kernel must not depend on the host application',
      'package:flutter_test/': 'tests here run on package:test',
      'dart:ui': 'dart:ui only exists inside a Flutter engine',
      'dart:html': 'the kernel must stay platform-agnostic',
      'dart:js_interop': 'the kernel must stay platform-agnostic',
    };

    for (final file in shipped) {
      test(file.path, () {
        final source = file.readAsStringSync();
        for (final entry in forbidden.entries) {
          expect(
            source,
            isNot(contains("'${entry.key}")),
            reason: '${file.path} imports ${entry.key}: ${entry.value}',
          );
        }
      });
    }
  });

  test('pubspec declares no Flutter dependency', () {
    final pubspec = loadYaml(File('pubspec.yaml').readAsStringSync()) as Map;

    for (final section in ['dependencies', 'dev_dependencies']) {
      final deps = pubspec[section] as Map?;
      if (deps == null) continue;
      expect(
        deps.keys,
        isNot(anyElement(anyOf('flutter', 'flutter_test', 'flutter_lints'))),
        reason: '$section must stay Flutter-free',
      );
    }

    expect(
      pubspec['environment'],
      isNot(contains('flutter')),
      reason: 'the SDK constraint must not require Flutter',
    );
  });

  test('app types that used to live in PaliCAD are exported', () {
    // MeshData and MeshFaceRegions moved in from the app. If either slipped
    // back out of the public barrel the app would compile against two
    // different definitions.
    final barrel = File('lib/pali_ijo.dart').readAsStringSync();
    expect(barrel, contains('mesh_data.dart'));
    expect(barrel, contains('mesh_regions.dart'));
    expect(barrel, contains('brep.dart'));
  });
}
