import 'dart:convert';
import 'dart:typed_data';

import 'solid.dart';

/// Isolate payload: UTF-8 JSON of a [BrepSolid] (typed data, sendable).
class PackedBrep {
  PackedBrep(this.bytes);

  final Uint8List bytes;

  factory PackedBrep.fromSolid(BrepSolid solid) => PackedBrep(
        Uint8List.fromList(utf8.encode(jsonEncode(solid.toJson()))),
      );

  BrepSolid toSolid() => BrepSolid.fromJson(
        jsonDecode(utf8.decode(bytes)) as Map<String, dynamic>,
      );
}
