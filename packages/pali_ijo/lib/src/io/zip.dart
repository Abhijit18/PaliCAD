import 'dart:convert';
import 'dart:typed_data';

/// A minimal ZIP reader and writer, because 3MF is a ZIP archive.
///
/// Entries are **stored**, not deflated. That is a legal ZIP — method 0 is in
/// the specification and every reader supports it — and it is what lets this
/// package stay on `vector_math` and `args` alone rather than pulling in a
/// compression library for one file format. A 3MF of a CAD model is XML that
/// would compress well, so the cost is real file size; the benefit is that a
/// consumer of this kernel inherits no dependency they did not ask for.
///
/// The reader accepts deflated entries' presence but cannot decode them, and
/// says so rather than returning wrong bytes.
class ZipArchive {
  ZipArchive();

  final Map<String, Uint8List> _entries = {};

  Iterable<String> get names => _entries.keys;

  int get length => _entries.length;

  Uint8List? operator [](String name) => _entries[name];

  void addBytes(String name, List<int> bytes) {
    _entries[name] = Uint8List.fromList(bytes);
  }

  void addText(String name, String text) =>
      addBytes(name, utf8.encode(text));

  String? readText(String name) {
    final bytes = _entries[name];
    return bytes == null ? null : utf8.decode(bytes);
  }

  /// The archive as ZIP bytes.
  Uint8List toBytes() {
    final out = BytesBuilder();
    final directory = BytesBuilder();
    var count = 0;

    for (final entry in _entries.entries) {
      final name = utf8.encode(entry.key);
      final data = entry.value;
      final crc = _crc32(data);
      final offset = out.length;

      final local = ByteData(30);
      local.setUint32(0, 0x04034b50, Endian.little); // local file header
      local.setUint16(4, 20, Endian.little); // version needed
      local.setUint16(6, 0, Endian.little); // flags
      local.setUint16(8, 0, Endian.little); // stored
      local.setUint16(10, 0, Endian.little); // time
      local.setUint16(12, 0x21, Endian.little); // date: 1980-01-01
      local.setUint32(14, crc, Endian.little);
      local.setUint32(18, data.length, Endian.little);
      local.setUint32(22, data.length, Endian.little);
      local.setUint16(26, name.length, Endian.little);
      local.setUint16(28, 0, Endian.little); // extra
      out.add(local.buffer.asUint8List());
      out.add(name);
      out.add(data);

      final central = ByteData(46);
      central.setUint32(0, 0x02014b50, Endian.little); // central directory
      central.setUint16(4, 20, Endian.little); // version made by
      central.setUint16(6, 20, Endian.little); // version needed
      central.setUint16(8, 0, Endian.little); // flags
      central.setUint16(10, 0, Endian.little); // stored
      central.setUint16(12, 0, Endian.little); // time
      central.setUint16(14, 0x21, Endian.little); // date
      central.setUint32(16, crc, Endian.little);
      central.setUint32(20, data.length, Endian.little);
      central.setUint32(24, data.length, Endian.little);
      central.setUint16(28, name.length, Endian.little);
      central.setUint16(30, 0, Endian.little); // extra
      central.setUint16(32, 0, Endian.little); // comment
      central.setUint16(34, 0, Endian.little); // disk
      central.setUint16(36, 0, Endian.little); // internal attrs
      central.setUint32(38, 0, Endian.little); // external attrs
      central.setUint32(42, offset, Endian.little);
      directory.add(central.buffer.asUint8List());
      directory.add(name);
      count++;
    }

    final directoryOffset = out.length;
    final directoryBytes = directory.toBytes();
    out.add(directoryBytes);

    final end = ByteData(22);
    end.setUint32(0, 0x06054b50, Endian.little); // end of central directory
    end.setUint16(4, 0, Endian.little); // this disk
    end.setUint16(6, 0, Endian.little); // disk with directory
    end.setUint16(8, count, Endian.little);
    end.setUint16(10, count, Endian.little);
    end.setUint32(12, directoryBytes.length, Endian.little);
    end.setUint32(16, directoryOffset, Endian.little);
    end.setUint16(20, 0, Endian.little); // comment
    out.add(end.buffer.asUint8List());

    return out.toBytes();
  }

  /// Reads a ZIP, walking the central directory rather than scanning for local
  /// headers, so an entry's own name and size come from the authoritative
  /// record.
  static ZipArchive fromBytes(Uint8List bytes) {
    final data = ByteData.sublistView(bytes);
    final eocd = _findEndOfCentralDirectory(bytes);
    if (eocd < 0) {
      throw FormatException('Not a ZIP archive: no end-of-directory record');
    }

    final count = data.getUint16(eocd + 10, Endian.little);
    var p = data.getUint32(eocd + 16, Endian.little);
    final archive = ZipArchive();

    for (var i = 0; i < count; i++) {
      if (p + 46 > bytes.length ||
          data.getUint32(p, Endian.little) != 0x02014b50) {
        throw FormatException('ZIP directory entry $i is malformed');
      }
      final method = data.getUint16(p + 10, Endian.little);
      final size = data.getUint32(p + 24, Endian.little);
      final nameLen = data.getUint16(p + 28, Endian.little);
      final extraLen = data.getUint16(p + 30, Endian.little);
      final commentLen = data.getUint16(p + 32, Endian.little);
      final offset = data.getUint32(p + 42, Endian.little);
      final name = utf8.decode(
        bytes.sublist(p + 46, p + 46 + nameLen),
        allowMalformed: true,
      );

      if (method != 0) {
        throw UnsupportedError(
          'ZIP entry "$name" is compressed (method $method). This reader '
          'handles stored entries only; see ZipArchive.',
        );
      }

      final localNameLen = data.getUint16(offset + 26, Endian.little);
      final localExtraLen = data.getUint16(offset + 28, Endian.little);
      final start = offset + 30 + localNameLen + localExtraLen;
      archive.addBytes(name, bytes.sublist(start, start + size));

      p += 46 + nameLen + extraLen + commentLen;
    }
    return archive;
  }

  /// The end-of-central-directory record sits at the end, but a trailing
  /// comment can push it back by up to 64 KiB, so it is searched for.
  static int _findEndOfCentralDirectory(Uint8List bytes) {
    if (bytes.length < 22) return -1;
    final earliest = bytes.length - 22 - 65535;
    final lowest = earliest < 0 ? 0 : earliest;
    for (var i = bytes.length - 22; i >= lowest; i--) {
      if (bytes[i] == 0x50 &&
          bytes[i + 1] == 0x4b &&
          bytes[i + 2] == 0x05 &&
          bytes[i + 3] == 0x06) {
        return i;
      }
    }
    return -1;
  }

  static Uint32List? _crcTable;

  static int _crc32(List<int> bytes) {
    final table = _crcTable ??= _buildCrcTable();
    var crc = 0xffffffff;
    for (final b in bytes) {
      crc = table[(crc ^ b) & 0xff] ^ (crc >> 8);
    }
    return (crc ^ 0xffffffff) & 0xffffffff;
  }

  static Uint32List _buildCrcTable() {
    final table = Uint32List(256);
    for (var i = 0; i < 256; i++) {
      var c = i;
      for (var k = 0; k < 8; k++) {
        c = (c & 1) != 0 ? 0xedb88320 ^ (c >> 1) : c >> 1;
      }
      table[i] = c;
    }
    return table;
  }
}
