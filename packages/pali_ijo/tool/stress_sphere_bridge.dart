// Reproduce the user's two cubes connected only by a spherical bridge.
// dart run tool/stress_sphere_bridge.dart /absolute/output/directory
// Writes inputs/results even for invalid operations, and exits 1 on failure.
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

bool sameJson(Object? a, Object? b) {
  if (a is num && b is num) {
    return (a - b).abs() <= 1e-10 * math.max(1.0, math.max(a.abs(), b.abs()));
  }
  if (a is List && b is List) {
    return a.length == b.length &&
        List.generate(a.length, (i) => i).every((i) => sameJson(a[i], b[i]));
  }
  if (a is Map && b is Map) {
    return a.length == b.length &&
        a.keys.every((k) => b.containsKey(k) && sameJson(a[k], b[k]));
  }
  return a == b;
}

final records = <Map<String, Object?>>[];
late Directory output;
final accepted = <String, BrepSolid>{};
void save(String name, BrepSolid s) => File(
  '${output.path}/$name.brep.json',
).writeAsStringSync(jsonEncode(s.toJson()));

Map<String, Object?> inspect(BrepSolid s) {
  final mesh = IjoOps.tessellate(s);
  var maxSurfaceResidual = 0.0;
  for (final f in s.faces) {
    for (final loop in f.loops) {
      for (final c in loop.coedges) {
        final curve = s.edges[c.edge].curve;
        // Skip the kernel's virtual cylinder seam (used twice on one face).
        if (loop.coedges.where((x) => x.edge == c.edge).length > 1) continue;
        for (final fraction in [0.0, 0.25, 0.5, 0.75, 1.0]) {
          final p = curve.evaluate(curve.t0 + (curve.t1 - curve.t0) * fraction);
          maxSurfaceResidual = math.max(
            maxSurfaceResidual,
            (p - f.surface.closestPoint(p)).length,
          );
        }
      }
    }
  }
  final v = MeshKit.signedVolume(mesh);
  MeshData? strictMesh;
  String? tessellationError;
  try {
    strictMesh = IjoOps.tessellateClosed(s);
  } catch (e) {
    tessellationError = e.toString();
  }

  return {
    'faces': s.faces.length,
    'surfaces': {
      for (final type
          in s.faces.map((f) => f.surface.runtimeType.toString()).toSet())
        type: s.faces
            .where((f) => f.surface.runtimeType.toString() == type)
            .length,
    },
    'topologyClosed': BrepHeal.topologyIsClosed(s),
    'rawMeshClosed': MeshKit.isClosedSolid(mesh),
    'openBoundaryEdges': MeshSeal.openBoundaryEdgeCount(mesh),
    'signedVolume': v,
    'components': IjoOps.splitBrep(s).length,
    'maxSampledEdgeSurfaceResidual': maxSurfaceResidual,
    'roundTripEqual': sameJson(
      PackedBrep.fromSolid(s).toSolid().toJson(),
      s.toJson(),
    ),
    'strictMeshClosed': strictMesh != null && MeshKit.isClosedSolid(strictMesh),
    'strictSignedVolume': strictMesh == null
        ? null
        : MeshKit.signedVolume(strictMesh),
    if (tessellationError != null) 'tessellationError': tessellationError,
  };
}

void stage(
  String name,
  String parent,
  BrepSolid Function(BrepSolid) operation, {
  BrepSolid? tool,
  String kind = 'subtract',
  double? expectedVolume,
}) {
  final record = <String, Object?>{
    'name': name,
    'parent': parent,
    'kind': kind,
  };
  final source = accepted[parent];
  if (source == null) {
    record['status'] = 'blocked';
    records.add(record);
    print('$name: blocked by $parent');
    return;
  }
  save('$name-input', source);
  if (tool != null) save('$name-tool', tool);
  final before = jsonEncode(source.toJson());
  final watch = Stopwatch()..start();
  try {
    final result = operation(source);
    save('$name-result', result);
    final metrics = inspect(result);
    record.addAll(metrics);
    final failures = <String>[];
    if (metrics['topologyClosed'] != true) failures.add('open BREP topology');
    if (metrics['strictMeshClosed'] != true) {
      failures.add('open/nonmanifold display mesh');
    }
    final volume =
        (metrics['strictSignedVolume'] ?? metrics['signedVolume']) as double;
    if (!volume.isFinite || volume <= 0) failures.add('invalid signed volume');
    if ((metrics['maxSampledEdgeSurfaceResidual'] as double) > 1e-5) {
      failures.add('edge leaves incident surface');
    }
    if (metrics['roundTripEqual'] != true) {
      failures.add('BREP serialization changed');
    }
    if (before != jsonEncode(source.toJson())) failures.add('input mutated');
    final oldVolume = MeshKit.signedVolume(IjoOps.tessellate(source));
    if (kind == 'subtract' && volume >= oldVolume - 0.01) {
      failures.add('cut did not remove volume');
    }
    if (kind == 'union' && volume <= oldVolume + 0.01) {
      failures.add('union did not add volume');
    }
    if (expectedVolume != null && (volume - expectedVolume).abs() > 25) {
      failures.add('incorrect reference volume');
    }
    if ((kind == 'fillet' || kind == 'chamfer') && volume < oldVolume * 0.95) {
      failures.add('small edge feature removed more than 5% of the body');
    }
    if (!result.faces.any((f) {
      final surface = f.surface;
      return surface is BrepSphere &&
          (surface.center - Vector3(22, 10, 10)).length < 1e-6 &&
          (surface.radius - 8).abs() < 1e-6;
    })) {
      failures.add('spherical bridge surface was lost');
    }
    // These small cutters are deliberately not large enough to sever the bridge.
    if (metrics['components'] != 1) {
      failures.add('unexpected disconnected components');
    }
    record['status'] = failures.isEmpty ? 'pass' : 'invalid';
    record['failures'] = failures;
    if (failures.isEmpty) accepted[name] = result;
  } catch (e, st) {
    record['status'] = 'exception';
    record['error'] = e.toString();
    record['stack'] = st.toString();
  }
  record['elapsedMs'] = watch.elapsedMilliseconds;
  records.add(record);
  print(
    '$name: ${record['status']} ${record['failures'] ?? record['error'] ?? ''}',
  );
  File(
    '${output.path}/report.json',
  ).writeAsStringSync(const JsonEncoder.withIndent('  ').convert(records));
}

int edgeAt(BrepSolid s, Vector3 a, Vector3 b) {
  final ids = [
    for (var i = 0; i < s.edges.length; i++)
      if (s.edges[i].curve is BrepLine &&
          (((s.vertices[s.edges[i].v0] - a).length < 1e-6 &&
                  (s.vertices[s.edges[i].v1] - b).length < 1e-6) ||
              ((s.vertices[s.edges[i].v1] - a).length < 1e-6 &&
                  (s.vertices[s.edges[i].v0] - b).length < 1e-6)))
        i,
  ];
  if (ids.length != 1) {
    throw StateError('Expected one geometric edge $a to $b; found $ids');
  }
  return ids.single;
}

BrepSolid circle(Vector3 center, Vector3 direction, double radius) {
  final n = direction.normalized();
  return BrepPrimitives.cylinder(
    origin: center - n * 70,
    axis: n,
    radius: radius,
    height: 140,
  );
}

BrepSolid square(Vector3 center, Vector3 direction, double width) {
  final n = direction.normalized(), u = brepFrameU(direction.normalized());
  final v = n.cross(u), o = center - n * 70;
  return BrepExtrude.polygon([
    o - u * width / 2 - v * width / 2,
    o + u * width / 2 - v * width / 2,
    o + u * width / 2 + v * width / 2,
    o - u * width / 2 + v * width / 2,
  ], n * 140);
}

void main(List<String> args) {
  if (args.length != 1) throw ArgumentError('Provide an output directory');
  output = Directory(args.single)..createSync(recursive: true);
  accepted['cube-a'] = BrepPrimitives.box(20, 20, 20);
  save('cube-a', accepted['cube-a']!);
  final sphere = BrepPrimitives.sphere(center: Vector3(22, 10, 10), radius: 8);
  final second = BrepXform.translate(
    BrepPrimitives.box(20, 20, 20),
    Vector3(24, 0, 0),
  );
  stage(
    'sphere-union',
    'cube-a',
    (s) => IjoOps.booleanSolid(s, sphere, IjoOps.unionOp),
    tool: sphere,
    kind: 'union',
  );
  stage(
    'bridge',
    'sphere-union',
    (s) => IjoOps.booleanSolid(s, second, IjoOps.unionOp),
    tool: second,
    kind: 'union',
    expectedVolume: 16000 + math.pi * (64 * 4 - 16 / 3),
  );
  stage(
    'fillet',
    'bridge',
    (s) => IjoOps.filletSolid(
      s,
      [edgeAt(s, Vector3(0, 0, 0), Vector3(0, 0, 20))],
      radius: 2,
      inverse: false,
    ),
    kind: 'fillet',
  );
  stage(
    'chamfer',
    'bridge',
    (s) => IjoOps.chamferSolid(s, [
      edgeAt(s, Vector3(44, 20, 0), Vector3(44, 20, 20)),
    ], 1),
    kind: 'chamfer',
  );
  stage(
    'fillet-then-chamfer',
    'fillet',
    (s) => IjoOps.chamferSolid(s, [
      edgeAt(s, Vector3(44, 20, 0), Vector3(44, 20, 20)),
    ], 1),
    kind: 'chamfer',
  );
  stage(
    'face-loop-fillet',
    'bridge',
    (s) => IjoOps.filletSolid(
      s,
      BrepEdgeSelect.ofFace(
        BrepFaceSelect.extreme(Vector3(-1, 0, 0)),
      ).resolve(s),
      radius: 1,
      inverse: false,
    ),
    kind: 'fillet',
  );
  final cuts = <String, BrepSolid>{
    'circle-left': circle(Vector3(8, 8, 10), Vector3(0, 0, 1), 2),
    'circle-overlap': circle(Vector3(10, 8, 10), Vector3(0, 0, 1), 2),
    'square-right': square(Vector3(34, 10, 10), Vector3(0, 0, 1), 4),
    'circle-bridge': circle(Vector3(22, 10, 10), Vector3(0, 0, 1), 1.5),
    'square-bridge': square(Vector3(22, 10, 10), Vector3(0, 1, 0), 3),
    'square-both-cubes': square(Vector3(22, 10, 10), Vector3(1, 0, 0), 3),
    'angled-square': square(Vector3(12, 12, 10), Vector3(0.4, 0.2, 1), 3),
    'angled-circle': circle(Vector3(32, 12, 10), Vector3(-0.3, 0.2, 1), 1.5),
    'angled-bridge-circle': circle(
      Vector3(22, 10, 10),
      Vector3(0.3, 0.2, 1),
      1.5,
    ),
  };
  for (final entry in cuts.entries) {
    stage(
      'independent-${entry.key}',
      'bridge',
      (s) => IjoOps.booleanSolid(s, entry.value, IjoOps.subtractOp),
      tool: entry.value,
    );
    stage(
      'featured-${entry.key}',
      'fillet-then-chamfer',
      (s) => IjoOps.booleanSolid(s, entry.value, IjoOps.subtractOp),
      tool: entry.value,
    );
  }
  var parent = 'bridge';
  for (final entry in cuts.entries) {
    final name = 'sequence-${entry.key}';
    stage(
      name,
      parent,
      (s) => IjoOps.booleanSolid(s, entry.value, IjoOps.subtractOp),
      tool: entry.value,
    );
    parent = name;
  }
  File(
    '${output.path}/report.json',
  ).writeAsStringSync(const JsonEncoder.withIndent('  ').convert(records));
  print('Results: ${output.path}/report.json');
  if (records.any((r) => r['status'] != 'pass')) exitCode = 1;
}
