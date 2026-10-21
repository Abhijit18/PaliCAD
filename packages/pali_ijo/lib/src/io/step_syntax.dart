/// The STEP exchange structure: ISO 10303-21 syntax, with no geometry in it.
///
/// A STEP file is a flat, numbered graph of records:
///
/// ```
/// #14 = CARTESIAN_POINT('',(0.,0.,0.));
/// #15 = DIRECTION('',(0.,0.,1.));
/// #16 = AXIS2_PLACEMENT_3D('',#14,#15,$);
/// ```
///
/// Splitting the syntax out from the geometry earns two things. The reader gets
/// to resolve references and read fields without also thinking about surfaces,
/// and the writer gets to emit records without also thinking about text
/// escaping. It also means the awkward corners of the format — strings that
/// contain semicolons, entities built from several types at once — are handled
/// in one place instead of being rediscovered.
library;

import 'dart:convert';

/// One field of a record.
///
/// STEP's value space is small but not uniform: a field can be a number, a
/// quoted string, an enumeration, a reference to another record, a list, a
/// typed wrapper like `LENGTH_MEASURE(0.01)`, or one of two kinds of absent.
sealed class StepValue {
  const StepValue();
}

class StepNumber extends StepValue {
  const StepNumber(this.value);
  final double value;
  @override
  String toString() => '$value';
}

class StepText extends StepValue {
  const StepText(this.value);
  final String value;
  @override
  String toString() => "'$value'";
}

/// An enumeration such as `.T.` or `.MILLI.`, without its dots.
class StepEnum extends StepValue {
  const StepEnum(this.name);
  final String name;

  bool get isTrue => name == 'T' || name == 'TRUE';
  bool get isFalse => name == 'F' || name == 'FALSE';

  @override
  String toString() => '.$name.';
}

class StepRef extends StepValue {
  const StepRef(this.id);
  final int id;
  @override
  String toString() => '#$id';
}

class StepList extends StepValue {
  const StepList(this.items);
  final List<StepValue> items;
  @override
  String toString() => '(${items.join(',')})';
}

/// A value carrying its own type, as select types and measures do.
class StepTyped extends StepValue {
  const StepTyped(this.type, this.args);
  final String type;
  final List<StepValue> args;

  /// The single argument, for the common `MEASURE(value)` shape.
  StepValue? get single => args.length == 1 ? args.first : null;

  @override
  String toString() => '$type(${args.join(',')})';
}

/// `$` — a field that is present in the schema and absent in this instance.
class StepUnset extends StepValue {
  const StepUnset();
  @override
  String toString() => r'$';
}

/// `*` — a field whose value is derived by the schema, not stated.
class StepDerived extends StepValue {
  const StepDerived();
  @override
  String toString() => '*';
}

/// One typed record: `ADVANCED_FACE('',(#31),#30,.T.)`.
///
/// A single entity instance may be built from several of these when the schema
/// uses multiple inheritance, which is why an id maps to a list of them:
///
/// ```
/// #5 = ( NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) LENGTH_UNIT() );
/// ```
class StepRecord {
  StepRecord(this.type, this.args);

  final String type;
  final List<StepValue> args;

  StepValue? arg(int i) => i < args.length ? args[i] : null;

  @override
  String toString() => '$type(${args.join(',')})';
}

/// A parsed STEP file: the header records, and the data records by id.
class StepFile {
  StepFile({
    required this.header,
    required this.entities,
    required this.schema,
  });

  /// `FILE_DESCRIPTION`, `FILE_NAME`, `FILE_SCHEMA`.
  final List<StepRecord> header;

  /// Data records by instance id. Ids are not dense and not ordered.
  final Map<int, List<StepRecord>> entities;

  /// The schema names from `FILE_SCHEMA`, upper-cased, e.g.
  /// `AUTOMOTIVE_DESIGN`. Informational: the reader works from the entity
  /// types it finds rather than from a claimed schema, since files routinely
  /// claim AP214 and contain AP203 entities.
  final List<String> schema;

  /// The records for [id], or empty when nothing is there.
  List<StepRecord> at(int id) => entities[id] ?? const [];

  /// The record of type [type] at [id], or null.
  ///
  /// Types are matched exactly. Complex entities are searched part by part, so
  /// asking a unit instance for `SI_UNIT` finds it among its siblings.
  StepRecord? typed(int id, String type) {
    for (final r in at(id)) {
      if (r.type == type) return r;
    }
    return null;
  }

  /// The first record at [id] whose type is in [types], or null. Order follows
  /// [types], so a caller states its own preference.
  StepRecord? anyOf(int id, List<String> types) {
    for (final type in types) {
      final found = typed(id, type);
      if (found != null) return found;
    }
    return null;
  }

  /// The type name at [id], for an error message. Complex entities are joined.
  String typeName(int id) {
    final records = at(id);
    if (records.isEmpty) return 'nothing';
    if (records.length == 1) return records.first.type;
    return records.map((r) => r.type).join('+');
  }

  /// Every id whose instance includes a record of type [type].
  Iterable<int> idsOf(String type) => entities.entries
      .where((e) => e.value.any((r) => r.type == type))
      .map((e) => e.key);

  /// Follows [value] as a reference and returns the record of [type] there.
  StepRecord? follow(StepValue? value, String type) =>
      value is StepRef ? typed(value.id, type) : null;

  static StepFile parse(String text) => StepParser(text).parse();
}

/// Raised when a file is not STEP, or is STEP that does not parse.
///
/// Separate from the reader's own complaint about geometry it cannot
/// represent: a malformed file and a file full of NURBS are different news.
class StepSyntaxException implements Exception {
  StepSyntaxException(this.message, {this.offset});
  final String message;

  /// Character offset, when the fault is at a known place.
  final int? offset;

  @override
  String toString() =>
      offset == null ? message : '$message (at character $offset)';
}

/// A character-by-character reader for the ISO 10303-21 exchange structure.
///
/// Character-by-character rather than by line or regular expression because a
/// quoted string may contain `;`, `)` and newlines, so anything that splits
/// the text up front will eventually cut one in half. Real files also arrive
/// with `/* comments */` between any two tokens.
class StepParser {
  StepParser(this._text);

  final String _text;
  int _i = 0;

  StepFile parse() {
    _expectKeyword('ISO-10303-21');
    _expect(';');

    final header = <StepRecord>[];
    var schema = <String>[];
    if (_tryKeyword('HEADER')) {
      _expect(';');
      while (!_tryKeyword('ENDSEC')) {
        if (_atEnd) {
          throw StepSyntaxException('Header section is not terminated');
        }
        final record = _record();
        _expect(';');
        header.add(record);
        if (record.type == 'FILE_SCHEMA') {
          schema = _schemaNames(record);
        }
      }
      _expect(';');
    }

    final entities = <int, List<StepRecord>>{};
    if (!_tryKeyword('DATA')) {
      throw StepSyntaxException('No DATA section');
    }
    // AP242 allows DATA('name'); — a population qualifier nobody writes, but
    // skipping it is cheaper than failing on it.
    _skipTrivia();
    if (_peek == '(') {
      _valueList();
    }
    _expect(';');

    while (!_tryKeyword('ENDSEC')) {
      if (_atEnd) {
        throw StepSyntaxException('Data section is not terminated');
      }
      final id = _instanceId();
      _expect('=');
      _skipTrivia();
      final records = _peek == '(' ? _complex() : [_record()];
      _expect(';');
      entities[id] = records;
    }
    _expect(';');

    return StepFile(header: header, entities: entities, schema: schema);
  }

  static List<String> _schemaNames(StepRecord record) {
    final first = record.arg(0);
    if (first is! StepList) return const [];
    return [
      for (final item in first.items)
        if (item is StepText)
          // 'AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }' -> the name.
          item.value.split(RegExp(r'[\s{]')).first.toUpperCase(),
    ];
  }

  int _instanceId() {
    _skipTrivia();
    if (_peek != '#') {
      throw StepSyntaxException(
        'Expected an instance name like #12, found ${_here()}',
        offset: _i,
      );
    }
    _i++;
    return _integer();
  }

  int _integer() {
    final start = _i;
    while (!_atEnd && _isDigit(_text.codeUnitAt(_i))) {
      _i++;
    }
    if (start == _i) {
      throw StepSyntaxException('Expected a number', offset: _i);
    }
    return int.parse(_text.substring(start, _i));
  }

  /// `( A(...) B(...) )` — one instance assembled from several types.
  List<StepRecord> _complex() {
    _expect('(');
    final parts = <StepRecord>[];
    while (true) {
      _skipTrivia();
      if (_peek == ')') {
        _i++;
        return parts;
      }
      if (_atEnd) {
        throw StepSyntaxException('Unterminated complex entity', offset: _i);
      }
      parts.add(_record());
    }
  }

  StepRecord _record() {
    _skipTrivia();
    final type = _typeName();
    final args = _valueList();
    return StepRecord(type, args);
  }

  String _typeName() {
    final start = _i;
    while (!_atEnd) {
      final c = _text.codeUnitAt(_i);
      final ok = _isLetter(c) || _isDigit(c) || c == 0x5f; // _
      if (!ok) break;
      _i++;
    }
    if (start == _i) {
      throw StepSyntaxException(
        'Expected an entity type, found ${_here()}',
        offset: _i,
      );
    }
    return _text.substring(start, _i).toUpperCase();
  }

  List<StepValue> _valueList() {
    _expect('(');
    final values = <StepValue>[];
    _skipTrivia();
    if (_peek == ')') {
      _i++;
      return values;
    }
    while (true) {
      values.add(_value());
      _skipTrivia();
      final c = _peek;
      if (c == ',') {
        _i++;
        continue;
      }
      if (c == ')') {
        _i++;
        return values;
      }
      throw StepSyntaxException(
        'Expected , or ) in a parameter list, found ${_here()}',
        offset: _i,
      );
    }
  }

  StepValue _value() {
    _skipTrivia();
    if (_atEnd) {
      throw StepSyntaxException('File ends inside a parameter list');
    }
    final c = _text[_i];
    switch (c) {
      case '#':
        _i++;
        return StepRef(_integer());
      case "'":
        return StepText(_string());
      case '(':
        return StepList(_valueList());
      case r'$':
        _i++;
        return const StepUnset();
      case '*':
        _i++;
        return const StepDerived();
      case '.':
        return _enumeration();
      case '"':
        return StepText(_binary());
    }
    final code = _text.codeUnitAt(_i);
    if (_isDigit(code) || c == '-' || c == '+') return _number();
    if (_isLetter(code)) {
      final type = _typeName();
      return StepTyped(type, _valueList());
    }
    throw StepSyntaxException('Unexpected ${_here()}', offset: _i);
  }

  StepValue _enumeration() {
    _i++; // opening dot
    final start = _i;
    while (!_atEnd && _text[_i] != '.') {
      _i++;
    }
    if (_atEnd) {
      throw StepSyntaxException('Unterminated enumeration', offset: start);
    }
    final name = _text.substring(start, _i);
    _i++; // closing dot
    return StepEnum(name.toUpperCase());
  }

  StepValue _number() {
    final start = _i;
    if (_peek == '-' || _peek == '+') _i++;
    while (!_atEnd) {
      final c = _text[_i];
      final code = _text.codeUnitAt(_i);
      if (_isDigit(code) || c == '.') {
        _i++;
        continue;
      }
      if (c == 'e' || c == 'E') {
        _i++;
        if (!_atEnd && (_peek == '-' || _peek == '+')) _i++;
        continue;
      }
      break;
    }
    final text = _text.substring(start, _i);
    final value = double.tryParse(text);
    if (value == null) {
      throw StepSyntaxException('Malformed number "$text"', offset: start);
    }
    return StepNumber(value);
  }

  /// A quoted string. `''` is an escaped quote; `\X2\....\X0\` and friends
  /// encode characters outside the base alphabet.
  String _string() {
    _i++; // opening quote
    final out = StringBuffer();
    while (true) {
      if (_atEnd) {
        throw StepSyntaxException('Unterminated string');
      }
      final c = _text[_i];
      if (c == "'") {
        _i++;
        if (!_atEnd && _text[_i] == "'") {
          out.write("'");
          _i++;
          continue;
        }
        return _decodeControlDirectives(out.toString());
      }
      out.write(c);
      _i++;
    }
  }

  /// `"0A1B"` — a binary literal. Kept as text: nothing in the geometry this
  /// kernel reads uses one, and dropping it would lose a field position.
  String _binary() {
    _i++;
    final start = _i;
    while (!_atEnd && _text[_i] != '"') {
      _i++;
    }
    if (_atEnd) throw StepSyntaxException('Unterminated binary literal');
    final text = _text.substring(start, _i);
    _i++;
    return text;
  }

  /// Turns ISO 10303-21 control directives into characters.
  ///
  /// Part names routinely carry them, since the base alphabet has no accents:
  /// `\X2\00E9\X0\` is é. `\S\E` is a shorter form for one Latin-1 character,
  /// and `\X\E9` an older one. Anything unrecognised is left as written rather
  /// than dropped, so a name is never silently mangled.
  static String _decodeControlDirectives(String raw) {
    if (!raw.contains(r'\')) return raw;
    final out = StringBuffer();
    var i = 0;
    while (i < raw.length) {
      if (raw[i] != r'\') {
        out.write(raw[i++]);
        continue;
      }
      final x2 = _matchAt(raw, i, RegExp(r'^\\X2\\((?:[0-9A-Fa-f]{4})+)\\X0\\'));
      if (x2 != null) {
        final hex = x2.group(1)!;
        for (var k = 0; k < hex.length; k += 4) {
          out.writeCharCode(int.parse(hex.substring(k, k + 4), radix: 16));
        }
        i += x2.group(0)!.length;
        continue;
      }
      final x4 = _matchAt(raw, i, RegExp(r'^\\X4\\((?:[0-9A-Fa-f]{8})+)\\X0\\'));
      if (x4 != null) {
        final hex = x4.group(1)!;
        for (var k = 0; k < hex.length; k += 8) {
          out.writeCharCode(int.parse(hex.substring(k, k + 8), radix: 16));
        }
        i += x4.group(0)!.length;
        continue;
      }
      final s = _matchAt(raw, i, RegExp(r'^\\S\\(.)'));
      if (s != null) {
        out.writeCharCode(s.group(1)!.codeUnitAt(0) + 128);
        i += s.group(0)!.length;
        continue;
      }
      final x1 = _matchAt(raw, i, RegExp(r'^\\X\\([0-9A-Fa-f]{2})'));
      if (x1 != null) {
        out.writeCharCode(int.parse(x1.group(1)!, radix: 16));
        i += x1.group(0)!.length;
        continue;
      }
      out.write(raw[i++]);
    }
    return out.toString();
  }

  static RegExpMatch? _matchAt(String text, int index, RegExp pattern) =>
      pattern.firstMatch(text.substring(index));

  void _skipTrivia() {
    while (!_atEnd) {
      final c = _text.codeUnitAt(_i);
      if (c == 0x20 || c == 0x09 || c == 0x0a || c == 0x0d) {
        _i++;
        continue;
      }
      if (c == 0x2f && _i + 1 < _text.length && _text[_i + 1] == '*') {
        final end = _text.indexOf('*/', _i + 2);
        if (end < 0) throw StepSyntaxException('Unterminated comment');
        _i = end + 2;
        continue;
      }
      return;
    }
  }

  void _expect(String char) {
    _skipTrivia();
    if (_atEnd || _text[_i] != char) {
      throw StepSyntaxException(
        'Expected "$char", found ${_here()}',
        offset: _i,
      );
    }
    _i++;
  }

  void _expectKeyword(String word) {
    if (!_tryKeyword(word)) {
      throw StepSyntaxException(
        'Expected "$word", found ${_here()}. This does not look like a STEP '
        'file.',
        offset: _i,
      );
    }
  }

  bool _tryKeyword(String word) {
    _skipTrivia();
    if (!_text.startsWith(word, _i)) return false;
    _i += word.length;
    return true;
  }

  bool get _atEnd => _i >= _text.length;
  String? get _peek => _atEnd ? null : _text[_i];

  String _here() {
    if (_atEnd) return 'end of file';
    final end = _i + 12 < _text.length ? _i + 12 : _text.length;
    return '"${_text.substring(_i, end).replaceAll('\n', r'\n')}"';
  }

  static bool _isDigit(int c) => c >= 0x30 && c <= 0x39;
  static bool _isLetter(int c) =>
      (c >= 0x41 && c <= 0x5a) || (c >= 0x61 && c <= 0x7a);
}

/// Builds the text of a STEP file.
///
/// Records are added in dependency order and referred to by the id returned.
/// [dedupe] is what keeps files a sane size: a box has 8 distinct points but
/// its faces, edges and placements between them ask for the same directions
/// over and over, and emitting one `DIRECTION` per request instead of per
/// distinct value roughly triples the file.
class StepWriter {
  StepWriter();

  final _lines = <String>[];
  final _byKey = <String, int>{};
  var _nextId = 1;

  /// Adds a record and returns its id.
  int add(String type, String args) {
    final id = _nextId++;
    _lines.add('#$id = $type($args);');
    return id;
  }

  /// Adds a record unless an identical one was already added, in which case
  /// the earlier id comes back.
  int dedupe(String type, String args) {
    final key = '$type($args)';
    final existing = _byKey[key];
    if (existing != null) return existing;
    final id = add(type, args);
    _byKey[key] = id;
    return id;
  }

  /// Adds one instance assembled from several types, as unit declarations are.
  int addComplex(List<String> parts) {
    final id = _nextId++;
    _lines.add('#$id = ( ${parts.join(' ')} );');
    return id;
  }

  /// A reference list: `(#3,#7,#9)`.
  static String refs(Iterable<int> ids) => '(${ids.map((i) => '#$i').join(',')})';

  /// A number in STEP's notation, where a real always carries a decimal point.
  ///
  /// The point is not cosmetic: `1` is an integer in this format and `1.` is a
  /// real, and a strict reader given an integer where the schema wants a real
  /// is within its rights to reject the file. Values are written with enough
  /// digits to survive the round trip exactly.
  static String num(double value) {
    if (!value.isFinite) {
      throw ArgumentError.value(value, 'value', 'not representable in STEP');
    }
    if (value == value.roundToDouble() && value.abs() < 1e15) {
      return '${value.toInt()}.';
    }
    var text = value.toString();
    if (text.contains('e')) {
      // Dart writes 1e-7; STEP wants 1.E-07.
      final parts = text.split('e');
      var mantissa = parts[0];
      if (!mantissa.contains('.')) mantissa = '$mantissa.';
      final exponent = int.parse(parts[1]);
      final sign = exponent < 0 ? '-' : '+';
      final digits = exponent.abs().toString().padLeft(2, '0');
      return '${mantissa.toUpperCase()}E$sign$digits';
    }
    return text;
  }

  /// A quoted string with the escaping STEP requires.
  static String text(String value) {
    final out = StringBuffer("'");
    for (final rune in value.runes) {
      if (rune == 0x27) {
        out.write("''"); // a quote doubles
        continue;
      }
      if (rune == 0x5c) {
        out.write(r'\\'); // as does a backslash
        continue;
      }
      if (rune >= 0x20 && rune < 0x7f) {
        out.writeCharCode(rune);
        continue;
      }
      // Outside the base alphabet: encode, rather than write bytes a reader
      // is not required to understand.
      final units = rune > 0xffff
          ? String.fromCharCode(rune).codeUnits
          : [rune];
      out.write(r'\X2\');
      for (final u in units) {
        out.write(u.toRadixString(16).toUpperCase().padLeft(4, '0'));
      }
      out.write(r'\X0\');
    }
    out.write("'");
    return out.toString();
  }

  /// The finished file, with the header wrapped around the records.
  String build({
    required String description,
    required String fileName,
    required String originator,
    required String schema,
    DateTime? timestamp,
  }) {
    final when = (timestamp ?? DateTime.now().toUtc());
    final stamp = '${when.year.toString().padLeft(4, '0')}-'
        '${when.month.toString().padLeft(2, '0')}-'
        '${when.day.toString().padLeft(2, '0')}T'
        '${when.hour.toString().padLeft(2, '0')}:'
        '${when.minute.toString().padLeft(2, '0')}:'
        '${when.second.toString().padLeft(2, '0')}';
    return [
      'ISO-10303-21;',
      'HEADER;',
      'FILE_DESCRIPTION((${text(description)}),${text('2;1')});',
      'FILE_NAME(${text(fileName)},${text(stamp)},(${text('')}),'
          '(${text('')},${text('')}),${text(originator)},'
          '${text(originator)},${text('')});',
      'FILE_SCHEMA((${text(schema)}));',
      'ENDSEC;',
      'DATA;',
      ..._lines,
      'ENDSEC;',
      'END-ISO-10303-21;',
      '',
    ].join('\n');
  }

  /// The finished file as bytes. STEP is Latin-1 by tradition and the writer
  /// escapes everything outside the base alphabet, so this is pure ASCII.
  List<int> bytes({
    required String description,
    required String fileName,
    required String originator,
    required String schema,
    DateTime? timestamp,
  }) =>
      ascii.encode(
        build(
          description: description,
          fileName: fileName,
          originator: originator,
          schema: schema,
          timestamp: timestamp,
        ),
      );
}
