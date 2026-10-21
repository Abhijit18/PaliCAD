import 'dart:io';

import 'package:pali_ijo/pali_ijo.dart';
import 'package:test/test.dart';

void main() {
  late Directory tmp;

  setUp(() {
    tmp = Directory.systemTemp.createTempSync('ijo-log-');
    IjoLog.sink = null;
    IjoLog.detachFile();
    IjoLog.enabled = false;
    IjoLog.verbose = false;
  });

  tearDown(() {
    IjoLog.detachFile();
    IjoLog.enabled = null;
    IjoLog.verbose = null;
    IjoLog.sink = null;
    tmp.deleteSync(recursive: true);
  });

  test('beginFile records lines even when the console sink is quiet', () {
    final path = '${tmp.path}/session.log';
    IjoLog.beginFile(path);
    IjoLog.line('boolean subtract tessellate');
    IjoLog.line('csg subtract clip polys A=12 B=8');
    final text = File(path).readAsStringSync();
    expect(text, contains('boolean subtract tessellate'));
    expect(text, contains('csg subtract clip polys'));
    expect(IjoLog.recent, isNotEmpty);
    expect(IjoLog.last, 'csg subtract clip polys A=12 B=8');
  });

  test('trace stays off the console unless verbose', () {
    final lines = <String>[];
    IjoLog.sink = lines.add;
    IjoLog.enabled = true;
    IjoLog.verbose = false;
    IjoLog.trace('csg subtract clip A to B');
    IjoLog.line('boolean subtract done');
    expect(lines.any((l) => l.contains('clip A to B')), isFalse);
    expect(lines.any((l) => l.contains('boolean subtract done')), isTrue);
    IjoLog.verbose = true;
    IjoLog.trace('csg subtract clip A to B');
    expect(lines.any((l) => l.contains('clip A to B')), isTrue);
  });
}
