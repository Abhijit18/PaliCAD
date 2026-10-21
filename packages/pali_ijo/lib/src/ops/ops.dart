import 'dart:math' as math;
import '../thread/form.dart';

import 'package:vector_math/vector_math_64.dart';

import '../brep/brep.dart';
import '../csg/boolean.dart';
import '../csg/minkowski.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_frame.dart';
import '../mesh/mesh_kit.dart';
import '../mesh/mesh_op.dart';
import '../mesh/mesh_regions.dart';
import '../mesh/mesh_seal.dart';
import '../ijo_log.dart';
import '../sweep/sweep.dart';
import '../tessellation.dart';

/// Direct-modelling operations for BREP solids and explicitly mesh-only input.
/// Prefer the solid APIs when topology is available; MeshData overloads cannot
/// preserve geometry that callers discarded before entering the kernel.
class IjoOps {
  IjoOps._();

  static const unionOp = BrepBoolean.unionOp;
  static const subtractOp = BrepBoolean.subtractOp;
  static const intersectOp = BrepBoolean.intersectOp;

  static MeshData tessellate(
    BrepSolid solid, {
    MeshTessellation tessellation = MeshTessellation.standard,
  }) => BrepTessellate.run(solid, tessellation: tessellation);

  static MeshData tessellateClosed(
    BrepSolid solid, {
    MeshTessellation tessellation = MeshTessellation.standard,
    String op = 'This change',
  }) => BrepHeal.tessellateClosed(solid, tessellation: tessellation, op: op);

  /// Validates a closed display mesh while retaining the analytic solid.
  /// A coarser retry can repair sampling seams; an open result is never
  /// presented as a successful modelling operation.
  static MeshData tessellateKeep(
    BrepSolid solid, {
    MeshTessellation tessellation = MeshTessellation.standard,
    String op = 'This change',
  }) {
    try {
      return tessellateClosed(solid, tessellation: tessellation, op: op);
    } on BrepUnsupported {
      return tessellateClosed(
        solid,
        tessellation: MeshTessellation.draft,
        op: op,
      );
    }
  }

  static BrepSolid booleanSolid(BrepSolid a, BrepSolid b, int op) =>
      BrepBoolean.run(a, b, op);

  /// Analytic boolean when both sides are B-Rep. Mesh CSG only when neither
  /// side is a [BrepSolid] — never a silent conversion of analytic solids.
  static ({BrepSolid? solid, MeshData mesh}) booleanPreferSolid(
    BrepSolid? a,
    BrepSolid? b,
    MeshData ma,
    MeshData mb,
    int op,
  ) {
    if (a != null && b != null) {
      final solid = booleanSolid(a, b, op);
      return (solid: solid, mesh: tessellateKeep(solid, op: 'Boolean'));
    }
    if (a != null || b != null) {
      throw BrepUnsupported(
        'Boolean of a B-Rep with a mesh is not an implicit conversion. '
        'Convert the B-Rep body to a mesh first.',
      );
    }
    return (
      solid: null,
      mesh: MeshBoolean.run(
        ma,
        mb,
        op,
      ).asMeshSolid('Imported or converted mesh solid'),
    );
  }

  /// Mesh CSG for mesh-only solids. Hosts that hold a [BrepSolid] should call
  /// [booleanSolid] or [booleanPreferSolid] instead of reconstructing
  /// from triangles.
  static MeshData boolean(MeshData a, MeshData b, int op) {
    final sa = BrepPrimitives.tryAabbBox(a);
    final sb = BrepPrimitives.tryAabbBox(b);
    if (sa != null && sb != null) {
      try {
        return tessellateClosed(booleanSolid(sa, sb, op), op: 'Boolean');
      } on BrepUnsupported {
        // Fall through to mesh CSG.
      } catch (_) {
        // Fall through to mesh CSG.
      }
    }
    return MeshBoolean.run(
      a,
      b,
      op,
    ).asMeshSolid('Imported or converted mesh solid');
  }

  static MeshData minkowskiSum(MeshData a, MeshData b) =>
      MeshMinkowski.sum(a, b);

  static MeshData minkowskiDifference(MeshData a, MeshData b) =>
      MeshMinkowski.difference(a, b);

  /// Grow ([radius] > 0) or shrink ([radius] < 0). Boxes use a face inset;
  /// planar solids miter along incident face planes.
  static MeshData offsetMesh(
    MeshData mesh,
    double radius, {
    MeshTessellation tessellation = MeshTessellation.draft,
  }) => MeshOffset.run(mesh, radius, tessellation: tessellation);

  static List<MeshData> splitSolids(MeshData mesh) =>
      MeshTopology.decompose(mesh);

  static List<MeshData> sliceParts(
    MeshData mesh,
    Vector3 origin,
    Vector3 normal,
  ) => MeshBuild.sliceParts(mesh, origin, normal);

  static MeshData slice(MeshData mesh, Vector3 origin, Vector3 normal) {
    final parts = sliceParts(mesh, origin, normal);
    if (parts.isEmpty) {
      throw MeshOpException('Slice produced no solid');
    }
    var out = parts.first;
    for (var i = 1; i < parts.length; i++) {
      out = MeshBoolean.run(out, parts[i], unionOp);
    }
    return out;
  }

  static List<BrepSolid> splitBrep(BrepSolid solid) => brepSplitSolids(solid);

  static List<BrepSolid> sliceBrep(
    BrepSolid solid,
    Vector3 origin,
    Vector3 normal,
  ) => BrepSlice.parts(solid, origin, normal);

  static BrepSolid chamferSolid(
    BrepSolid solid,
    List<int> edgeIds,
    double distance, {
    ChamferOptions options = const ChamferOptions(),
  }) => BrepFillet.chamfer(solid, edgeIds, distance, options: options);

  static MeshData chamferEdges(
    MeshData mesh,
    List<(Vector3, Vector3)> edges,
    double distance, {
    MeshTessellation tessellation = MeshTessellation.standard,
  }) {
    final s = BrepPrimitives.tryAabbBox(mesh);
    if (s != null) {
      try {
        final ids = BrepFillet.resolveEdges(s, edges);
        return tessellateClosed(
          chamferSolid(s, ids, distance),
          tessellation: tessellation,
          op: 'Chamfer',
        );
      } on BrepUnsupported {
        // MeshSolid fallback.
      }
    }
    return MeshChamfer.chamfer(
      mesh,
      edges,
      distance: distance,
      tessellation: tessellation,
    );
  }

  static BrepSolid filletSolid(
    BrepSolid solid,
    List<int> edgeIds, {
    required double radius,
    required bool inverse,
  }) => BrepFillet.fillet(solid, edgeIds, radius: radius, inverse: inverse);

  static MeshData filletEdges(
    MeshData mesh,
    List<(Vector3, Vector3)> edges, {
    required double radius,
    required bool inverse,
    MeshTessellation tessellation = MeshTessellation.standard,
  }) {
    final s = BrepPrimitives.tryAabbBox(mesh);
    if (s != null) {
      try {
        final ids = BrepFillet.resolveEdges(s, edges);
        return tessellateClosed(
          filletSolid(s, ids, radius: radius, inverse: inverse),
          tessellation: tessellation,
          op: 'Fillet',
        );
      } on BrepUnsupported {
        // MeshSolid fallback.
      }
    }
    return MeshFillet.fillet(
      mesh,
      edges,
      radius: radius,
      inverse: inverse,
      tessellation: tessellation,
    );
  }

  static BrepSolid hollowSolid(
    BrepSolid solid,
    List<int> openingFaces,
    Vector3 openingNormal, {
    required double wallThicknessMm,
  }) => BrepHollow.run(
    solid,
    openingFaces,
    openingNormal,
    wallThicknessMm: wallThicknessMm,
  );

  static MeshData hollow(
    MeshData mesh,
    List<int> openingFaces,
    Vector3 openingNormal, {
    required double wallThicknessMm,
    MeshTessellation tessellation = MeshTessellation.draft,
    BrepSolid? solid,
  }) => hollowPreferSolid(
    mesh,
    openingFaces,
    openingNormal,
    wallThicknessMm: wallThicknessMm,
    tessellation: tessellation,
    solid: solid,
  ).mesh;

  /// Analytic shell when a [BrepSolid] is available; otherwise mesh offset.
  static ({BrepSolid? solid, MeshData mesh}) hollowPreferSolid(
    MeshData mesh,
    List<int> openingFaces,
    Vector3 openingNormal, {
    required double wallThicknessMm,
    MeshTessellation tessellation = MeshTessellation.draft,
    BrepSolid? solid,
  }) {
    final brep =
        solid ??
        BrepPrimitives.tryAabbBox(mesh) ??
        BrepPrimitives.tryPlanarSolid(mesh);
    if (brep != null) {
      try {
        final out = BrepHollow.fromMeshOpening(
          brep,
          mesh,
          openingFaces,
          openingNormal,
          wallThicknessMm: wallThicknessMm,
        );
        return (
          solid: out,
          mesh: tessellateKeep(out, tessellation: tessellation, op: 'Hollow'),
        );
      } on BrepUnsupported catch (e) {
        // Mesh offset of a filleted body punches an AABB pocket. Refuse
        // rather than silently turning rounded walls into a box cavity.
        if (solid != null || e.code == BrepUnsupported.filletOffset) rethrow;
      }
    }
    return (
      solid: null,
      mesh: MeshHollow.run(
        mesh,
        openingFaces: openingFaces,
        openingNormal: openingNormal,
        wallThicknessMm: wallThicknessMm,
        tessellation: tessellation,
      ),
    );
  }

  // ── sweeps ─────────────────────────────────────────────────────────────────

  static MeshData revolve(
    SweepProfile profile,
    Vector3 origin,
    Vector3 axis, {
    double angle = math.pi * 2,
  }) => IjoRevolve.run(profile, origin, axis, angle: angle);

  static MeshData lathe(
    List<(double radius, double height)> outline, {
    Vector3? origin,
    Vector3? axis,
    double angle = math.pi * 2,
  }) => IjoRevolve.lathe(outline, origin: origin, axis: axis, angle: angle);

  static MeshData sweep(
    SweepProfile profile,
    List<Vector3> path, {
    bool closed = false,
    double twistRadians = 0,
  }) =>
      IjoSweep.path(profile, path, closed: closed, twistRadians: twistRadians);

  static MeshData loftSolid(
    List<SweepProfile> sections, {
    bool closed = false,
  }) => loft(sections, closed: closed);

  /// Builds a BREP loft and returns its closed display tessellation.
  ///
  /// Native cylinder, frustum, or prism when the profiles allow it;
  /// otherwise exact ruled surfaces. Unsupported profiles throw.
  ///
  /// [holes] are paired inner loops for two-section lofts.
  static MeshData loft(
    List<SweepProfile> sections, {
    bool closed = false,
    List<(SweepProfile, SweepProfile)> holes = const [],
  }) => loftPreferSolid(sections, closed: closed, holes: holes).mesh;

  /// A BREP loft with native or ruled surfaces and its display tessellation.
  /// Unsupported section correspondence throws without producing a mesh solid.
  static ({BrepSolid? solid, MeshData mesh}) loftPreferSolid(
    List<SweepProfile> sections, {
    bool closed = false,
    List<(SweepProfile, SweepProfile)> holes = const [],
  }) {
    if (closed)
      throw BrepUnsupported(
        'A closed loft needs a periodic section correspondence',
      );
    final solid = BrepLoft.sections(
      [for (final section in sections) section.ring()],
      holes: [for (final (a, b) in holes) (a.ring(), b.ring())],
    );
    return (solid: solid, mesh: tessellateClosed(solid, op: 'Loft'));
  }

  static MeshData pipeSolid(
    List<Vector3> path, {
    required double radius,
    bool closed = false,
    int sides = 32,
    MeshTessellation tessellation = MeshTessellation.standard,
  }) => pipePreferSolid(
    path,
    outerRadius: radius,
    closed: closed,
    sides: sides,
    tessellation: tessellation,
  ).mesh;

  /// A round pipe along [path]. Pass [innerRadius] for a hollow tube.
  static MeshData pipe(
    List<Vector3> path, {
    required double outerRadius,
    double? innerRadius,
    bool closed = false,
    int sides = 32,
    MeshTessellation tessellation = MeshTessellation.standard,
  }) => pipePreferSolid(
    path,
    outerRadius: outerRadius,
    innerRadius: innerRadius,
    closed: closed,
    sides: sides,
    tessellation: tessellation,
  ).mesh;

  /// A BREP pipe with cylindrical runs and toroidal bends. Unsupported or
  /// singular spines throw without producing a mesh solid.
  static ({BrepSolid? solid, MeshData mesh}) pipePreferSolid(
    List<Vector3> path, {
    required double outerRadius,
    double? innerRadius,
    bool closed = false,
    int sides = 32,
    MeshTessellation tessellation = MeshTessellation.standard,
  }) {
    final solid = BrepPipe.run(
      path,
      radius: outerRadius,
      innerRadius: innerRadius,
      closed: closed,
    );
    final mesh = tessellateClosed(
      solid,
      tessellation: tessellation,
      op: 'Pipe',
    );
    return (solid: solid, mesh: mesh);
  }

  // ── threads ────────────────────────────────────────────────────────────────

  /// Cuts a helical thread into [base].
  ///
  /// The ISO 60° groove is stamped onto the existing cylindrical wall —
  /// remeshed and pushed inward on a shaft, outward on a bore. Subtracting a
  /// faceted helix only closes for a few shaft sizes; moving the wall is
  /// what actually stays closed.
  ///
  /// [internal] cuts outward from a bore of [holeRadius] rather than inward
  /// from a shaft of [majorRadius]. [fromEndA] and [fromEndB] thread only that
  /// end for [lengthMm]; with neither, the whole span between [axisZMin] and
  /// [axisZMax] is threaded.
  static MeshData threads({
    required MeshData base,
    required Vector3 origin,
    required Vector3 axis,
    required double majorRadius,
    required double holeRadius,
    required double axisZMin,
    required double axisZMax,
    required double pitchMm,
    required double lengthMm,
    required int starts,
    required bool rightHand,
    required bool internal,
    required bool fromEndA,
    required bool fromEndB,
    int segmentsPerTurn = 12,
  }) {
    // Stay on the mesh. Recovering a B-Rep from a helical cut spent tens of
    // seconds healing slivers and then rejected the open result.
    final radius = internal ? holeRadius : majorRadius;
    if (internal && radius <= 0) {
      throw MeshOpException('An internal thread needs a positive hole radius');
    }
    if (!internal && radius <= 0) {
      throw MeshOpException('An external thread needs a positive major radius');
    }
    final wallCut = _tryThreadWall(
      base: base,
      origin: origin,
      axis: axis,
      radius: radius,
      outward: internal,
      axisZMin: axisZMin,
      axisZMax: axisZMax,
      pitchMm: pitchMm,
      lengthMm: lengthMm,
      starts: starts,
      rightHand: rightHand,
      fromEndA: fromEndA,
      fromEndB: fromEndB,
      segmentsPerTurn: segmentsPerTurn,
    );
    if (wallCut != null) return wallCut;
    if (internal) {
      throw MeshOpException(
        'An internal thread needs a cylindrical hole to cut into',
      );
    }

    // Featured shafts with no clean cylindrical wall still try the helix
    // subtract and refuse if the shell stays open.
    var result = base;
    for (final cutter in threadCutterMeshes(
      origin: origin,
      axis: axis,
      majorRadius: majorRadius,
      holeRadius: holeRadius,
      axisZMin: axisZMin,
      axisZMax: axisZMax,
      pitchMm: pitchMm,
      lengthMm: lengthMm,
      starts: starts,
      rightHand: rightHand,
      internal: internal,
      fromEndA: fromEndA,
      fromEndB: fromEndB,
      segmentsPerTurn: segmentsPerTurn,
    )) {
      result = MeshBoolean.run(result, cutter, subtractOp);
    }
    if (!MeshKit.isClosedSolid(result)) {
      result = MeshSeal.sealSolid(result);
    }
    if (!MeshKit.isClosedSolid(result)) {
      throw MeshOpException('Threads produced a broken solid');
    }
    return result;
  }

  /// Stamp the ISO groove onto a cylindrical wall, or `null` if that wall
  /// cannot be isolated as two rims.
  static MeshData? _tryThreadWall({
    required MeshData base,
    required Vector3 origin,
    required Vector3 axis,
    required double radius,
    required bool outward,
    required double axisZMin,
    required double axisZMax,
    required double pitchMm,
    required double lengthMm,
    required int starts,
    required bool rightHand,
    required bool fromEndA,
    required bool fromEndB,
    required int segmentsPerTurn,
  }) {
    if (pitchMm <= 0) {
      throw MeshOpException('Threads need a positive pitch');
    }
    final depth = ThreadForm(pitch: pitchMm, internal: outward).depth;
    if (depth >= radius) {
      throw MeshOpException(
        'A ${pitchMm}mm pitch cuts ${depth.toStringAsFixed(3)}mm deep, which '
        'is past the ${radius.toStringAsFixed(3)}mm radius being threaded',
      );
    }
    final regions = _threadRegions(
      axisZMin: axisZMin,
      axisZMax: axisZMax,
      lengthMm: lengthMm,
      fromEndA: fromEndA,
      fromEndB: fromEndB,
    );
    if (regions.isEmpty) {
      throw MeshOpException('Threads have no length to occupy');
    }

    final mesh = MeshEdit.weld(MeshEdit.triangulate(base), epsilon: 1e-4);
    final wall = _cylinderWallFaces(mesh, origin, axis, radius);
    if (wall.length < 6) return null;
    final loops = MeshFaceRegions.orderedBoundaryLoops(mesh, wall);
    final rims = [
      for (final loop in loops)
        if (loop.length >= 8) loop,
    ];
    if (rims.length < 2) return null;

    final a = axis.normalized();
    final xDir = meshFrameU(a);
    final yDir = a.cross(xDir).normalized();
    double axial(int i) => (mesh.vertices[i] - origin).dot(a);
    rims.sort((p, q) {
      final za = p.fold<double>(0, (s, i) => s + axial(i)) / p.length;
      final zb = q.fold<double>(0, (s, i) => s + axial(i)) / q.length;
      return za.compareTo(zb);
    });
    // Extreme openings, not the two lowest loops — CSG/hollow walls often
    // have extra fragments, and using rims[1] left a huge hole that seal
    // then tried to cap.
    final low = _orderByAngle(mesh, rims.first, origin, a, xDir, yDir);
    final high = _orderByAngle(mesh, rims.last, origin, a, xDir, yDir);

    final extra = _threadGrooveDepth(
      pitchMm: pitchMm,
      depth: depth,
      internal: outward,
      starts: starts,
      rightHand: rightHand,
      regions: regions,
    );
    final sign = outward ? 1.0 : -1.0;

    final verts = [for (final v in mesh.vertices) v.clone()];
    final zLow = low.fold<double>(0, (s, i) => s + axial(i)) / low.length;
    final zHigh = high.fold<double>(0, (s, i) => s + axial(i)) / high.length;
    final samples = math.max(8, segmentsPerTurn);
    final span = (zHigh - zLow).abs();
    var belts = math.max(2, (span / pitchMm * samples).round());
    if (belts > 80) belts = 80;

    void stamp(int i) {
      final p = verts[i];
      final d = p - origin;
      final z = d.dot(a);
      final radial = d - a * z;
      final r0 = radial.length;
      if (r0 < 1e-12) return;
      final theta = math.atan2(radial.dot(yDir), radial.dot(xDir));
      final r1 = radius + sign * extra(theta, z);
      verts[i] = origin + a * z + radial * (r1 / r0);
    }

    final wallSet = wall.toSet();
    final faces = <List<int>>[
      for (var i = 0; i < mesh.faces.length; i++)
        if (!wallSet.contains(i)) List<int>.from(mesh.faces[i]),
    ];

    if (low.length == high.length) {
      for (final i in low) {
        stamp(i);
      }
      for (final i in high) {
        stamp(i);
      }
      final n = low.length;
      final rings = <List<int>>[low];
      for (var s = 1; s < belts; s++) {
        final t = s / belts;
        final z = zLow + (zHigh - zLow) * t;
        final ring = <int>[];
        for (var i = 0; i < n; i++) {
          final pa = verts[low[i]];
          final da = pa - origin;
          final ra = da - a * da.dot(a);
          final theta = math.atan2(ra.dot(yDir), ra.dot(xDir));
          final r = radius + sign * extra(theta, z);
          ring.add(verts.length);
          verts.add(
            origin +
                a * z +
                xDir * (r * math.cos(theta)) +
                yDir * (r * math.sin(theta)),
          );
        }
        rings.add(ring);
      }
      rings.add(high);
      for (var s = 0; s < rings.length - 1; s++) {
        _addThreadBelt(faces, rings[s], rings[s + 1], outward: outward);
      }
    } else {
      // Keep the original opening rims (sphere–cylinder curves stay on the
      // outer shell). Remesh the bore at the longer rim's count so the
      // helical belts are circular, not a 24-gon zipper onto a 128-gon hole.
      final n = math.max(low.length, high.length);
      final stations = _threadWallStations(
        zLow: zLow,
        zHigh: zHigh,
        regions: regions,
        pitchMm: pitchMm,
        samples: samples,
      );
      List<int> ringAt(double z) {
        final ring = <int>[];
        for (var i = 0; i < n; i++) {
          final theta = -math.pi + (i / n) * math.pi * 2;
          final r = radius + sign * extra(theta, z);
          ring.add(verts.length);
          verts.add(
            origin +
                a * z +
                xDir * (r * math.cos(theta)) +
                yDir * (r * math.sin(theta)),
          );
        }
        return ring;
      }

      final rings = <List<int>>[low, for (final z in stations) ringAt(z), high];
      for (var s = 0; s < rings.length - 1; s++) {
        _bridgeThreadRims(faces, rings[s], rings[s + 1], outward: outward);
      }
    }

    var result = MeshData(vertices: verts, faces: faces);
    result = MeshEdit.weld(result, epsilon: 1e-4);
    if (!MeshKit.isClosedSolid(result)) {
      final open = MeshSeal.openBoundaryEdgeCount(result);
      // A leftover T-junction or two is worth sealing. A missing wall is
      // not — cap-loop ear clipping on that hole freezes the isolate.
      if (open > 0 && open <= 24) {
        result = MeshSeal.sealSolid(result);
      } else {
        IjoLog.line(
          'thread wall open=$open rims ${low.length}/${high.length} '
          'equal=${low.length == high.length}',
        );
      }
    }
    if (!MeshKit.isClosedSolid(result)) return null;
    IjoLog.line(
      '${outward ? "female" : "male"} wall ${wall.length}f '
      '${low.length}/${high.length} rims ${belts} belts '
      '${IjoLog.size(result.vertices.length, result.faces.length)}',
    );
    return result;
  }

  static List<double> _threadWallStations({
    required double zLow,
    required double zHigh,
    required List<(double, double)> regions,
    required double pitchMm,
    required int samples,
  }) {
    final loZ = math.min(zLow, zHigh);
    final hiZ = math.max(zLow, zHigh);
    if (hiZ - loZ < 1e-9) return const [];
    final inset = math.min(pitchMm * 0.08, (hiZ - loZ) * 0.05);
    final zs = <double>{};
    for (final (lo, hi) in regions) {
      final a = math.max(loZ + inset, math.min(lo, hi));
      final b = math.min(hiZ - inset, math.max(lo, hi));
      if (b - a < 1e-9) continue;
      final nDiv = ((b - a) / pitchMm * samples).round().clamp(2, 32);
      for (var i = 0; i <= nDiv; i++) {
        zs.add(a + (b - a) * i / nDiv);
      }
    }
    var sorted = zs.where((z) => z > loZ + 1e-9 && z < hiZ - 1e-9).toList()
      ..sort();
    if (sorted.isEmpty) {
      sorted = [(loZ + hiZ) * 0.5];
    }
    if (sorted.length > 78) {
      final kept = <double>{sorted.first, sorted.last};
      final step = (sorted.length - 1) / 76;
      for (var i = 1; i < 76; i++) {
        kept.add(sorted[(i * step).round().clamp(0, sorted.length - 1)]);
      }
      sorted = kept.toList()..sort();
    }
    return sorted;
  }

  static void _addThreadBelt(
    List<List<int>> faces,
    List<int> aR,
    List<int> bR, {
    required bool outward,
  }) {
    final n = aR.length;
    if (n < 3 || bR.length != n) return;
    for (var i = 0; i < n; i++) {
      final j = (i + 1) % n;
      if (outward) {
        faces.add([aR[i], bR[i], bR[j]]);
        faces.add([aR[i], bR[j], aR[j]]);
      } else {
        faces.add([aR[i], aR[j], bR[j]]);
        faces.add([aR[i], bR[j], bR[i]]);
      }
    }
  }

  /// Zipper two circular rims that may have different vertex counts.
  ///
  /// Each polygon is traversed exactly once. Wrapping an index back to 0 and
  /// continuing used to walk a second lap on the shorter rim and leave the
  /// longer one open — female bores (CSG/hollow, 32 vs 64) then either threw
  /// or handed a huge hole to seal.
  static void _bridgeThreadRims(
    List<List<int>> faces,
    List<int> a,
    List<int> b, {
    required bool outward,
  }) {
    final na = a.length;
    final nb = b.length;
    if (na < 3 || nb < 3) return;
    if (na == nb) {
      _addThreadBelt(faces, a, b, outward: outward);
      return;
    }
    var i = 0;
    var j = 0;
    var takenA = 0;
    var takenB = 0;
    while (takenA < na || takenB < nb) {
      final aNext = (i + 1) % na;
      final bNext = (j + 1) % nb;
      final takeA =
          takenA < na &&
          (takenB >= nb || (takenA + 1) / na <= (takenB + 1) / nb);
      if (takeA) {
        if (outward) {
          faces.add([a[i], b[j], a[aNext]]);
        } else {
          faces.add([a[i], a[aNext], b[j]]);
        }
        i = aNext;
        takenA++;
      } else {
        if (outward) {
          faces.add([a[i], b[j], b[bNext]]);
        } else {
          faces.add([a[i], b[bNext], b[j]]);
        }
        j = bNext;
        takenB++;
      }
    }
  }

  static List<int> _cylinderWallFaces(
    MeshData mesh,
    Vector3 origin,
    Vector3 axis,
    double holeRadius,
  ) {
    final a = axis.normalized();
    final tol = math.max(0.35, holeRadius * 0.08);
    bool onBore(Vector3 p) {
      final d = p - origin;
      final radial = (d - a * d.dot(a)).length;
      return (radial - holeRadius).abs() <= tol;
    }

    final out = <int>[];
    for (var i = 0; i < mesh.faces.length; i++) {
      final face = mesh.faces[i];
      if (face.length < 3) continue;
      if (!face.every((v) => onBore(mesh.vertices[v]))) continue;
      var c = Vector3.zero();
      for (final v in face) {
        c += mesh.vertices[v];
      }
      c /= face.length.toDouble();
      if (!onBore(c)) continue;
      final n = (mesh.vertices[face[1]] - mesh.vertices[face[0]]).cross(
        mesh.vertices[face[2]] - mesh.vertices[face[0]],
      );
      if (n.length2 < 1e-16) continue;
      n.normalize();
      if (n.dot(a).abs() > 0.55) continue;
      out.add(i);
    }
    return out;
  }

  static List<int> _orderByAngle(
    MeshData mesh,
    List<int> loop,
    Vector3 origin,
    Vector3 axis,
    Vector3 xDir,
    Vector3 yDir,
  ) {
    double theta(int i) {
      final d = mesh.vertices[i] - origin;
      final radial = d - axis * d.dot(axis);
      return math.atan2(radial.dot(yDir), radial.dot(xDir));
    }

    final items = [for (final i in loop) (theta(i), i)]
      ..sort((a, b) => a.$1.compareTo(b.$1));
    return [for (final item in items) item.$2];
  }

  /// Depth of the ISO groove into the wall at a cylindrical point.
  ///
  /// This is the thread *form*, not the CSG cutter. The cutter overshoots the
  /// wall so a boolean can close; using that intersection here left a crest
  /// land about half a pitch wide. ISO 68-1 is a 60° V with a P/8 crest flat.
  static double Function(double theta, double z) _threadGrooveDepth({
    required double pitchMm,
    required double depth,
    required bool internal,
    required int starts,
    required bool rightHand,
    required List<(double, double)> regions,
  }) {
    final startCount = starts < 1 ? 1 : starts;
    final lead = pitchMm * startCount;
    final crestStart = ThreadForm(
      pitch: pitchMm,
      internal: internal,
    ).grooveHalfWidth;
    final flank = math.sqrt(3.0);
    final sign = rightHand ? 1.0 : -1.0;
    final twoPi = math.pi * 2;

    bool inCutter(double z) {
      for (final (lo, hi) in regions) {
        if (z >= lo - pitchMm && z <= hi + pitchMm) return true;
      }
      return false;
    }

    double profile(double v) {
      final av = v.abs();
      if (av >= crestStart) return 0;
      final e = (crestStart - av) * flank;
      return e > depth ? depth : e;
    }

    return (theta, z) {
      if (!inCutter(z)) return 0;
      var best = 0.0;
      for (final (lo, _) in regions) {
        for (var s = 0; s < startCount; s++) {
          var zCrest = lo - pitchMm + s * pitchMm;
          zCrest += sign * lead * theta / twoPi;
          var v = z - zCrest;
          v -= lead * (v / lead).roundToDouble();
          final e = profile(v);
          if (e > best) best = e;
        }
      }
      return best;
    };
  }

  static List<MeshData> threadCutters({
    required Vector3 origin,
    required Vector3 axis,
    required double majorRadius,
    required double holeRadius,
    required double axisZMin,
    required double axisZMax,
    required double pitchMm,
    required double lengthMm,
    required int starts,
    required bool rightHand,
    required bool internal,
    required bool fromEndA,
    required bool fromEndB,
    int segmentsPerTurn = 16,
  }) => threadCutterMeshes(
    origin: origin,
    axis: axis,
    majorRadius: majorRadius,
    holeRadius: holeRadius,
    axisZMin: axisZMin,
    axisZMax: axisZMax,
    pitchMm: pitchMm,
    lengthMm: lengthMm,
    starts: starts,
    rightHand: rightHand,
    internal: internal,
    fromEndA: fromEndA,
    fromEndB: fromEndB,
    segmentsPerTurn: segmentsPerTurn,
  );

  /// [threadCutters] as triangle meshes.
  static List<MeshData> threadCutterMeshes({
    required Vector3 origin,
    required Vector3 axis,
    required double majorRadius,
    required double holeRadius,
    required double axisZMin,
    required double axisZMax,
    required double pitchMm,
    required double lengthMm,
    required int starts,
    required bool rightHand,
    required bool internal,
    required bool fromEndA,
    required bool fromEndB,
    int segmentsPerTurn = 12,
  }) {
    if (pitchMm <= 0) {
      throw MeshOpException('Threads need a positive pitch');
    }
    final radius = internal ? holeRadius : majorRadius;
    if (radius <= 0) {
      throw MeshOpException(
        internal
            ? 'An internal thread needs a positive hole radius'
            : 'An external thread needs a positive major radius',
      );
    }
    final startCount = starts < 1 ? 1 : starts;
    final depth = ThreadForm(pitch: pitchMm, internal: internal).depth;
    if (depth >= radius) {
      throw MeshOpException(
        'A ${pitchMm}mm pitch cuts ${depth.toStringAsFixed(3)}mm deep, which '
        'is past the ${radius.toStringAsFixed(3)}mm radius being threaded',
      );
    }

    final a = axis.normalized();
    final regions = _threadRegions(
      axisZMin: axisZMin,
      axisZMax: axisZMax,
      lengthMm: lengthMm,
      fromEndA: fromEndA,
      fromEndB: fromEndB,
    );
    if (regions.isEmpty) {
      throw MeshOpException('Threads have no length to occupy');
    }

    final lead = pitchMm * startCount;
    final profile = _threadCutterProfile(
      pitch: pitchMm,
      depth: depth,
      internal: internal,
    );

    final out = <MeshData>[];
    for (final (zLow, zHigh) in regions) {
      for (var s = 0; s < startCount; s++) {
        final z0 = zLow - pitchMm + s * pitchMm;
        final turns = (zHigh + pitchMm - z0) / lead;
        if (turns <= 0) continue;
        final cutter = IjoSweep.helixMesh(
          profile,
          origin: origin + a * z0,
          direction: a,
          radius: radius,
          pitch: lead,
          turns: turns,
          rightHand: rightHand,
          segmentsPerTurn: segmentsPerTurn,
        );
        IjoLog.line(
          'thread cutter mesh start ${s + 1}/$startCount '
          'z=${z0.toStringAsFixed(2)} turns=${turns.toStringAsFixed(2)} '
          '${IjoLog.size(cutter.vertices.length, cutter.faces.length)}',
        );
        out.add(cutter);
      }
    }
    return out;
  }

  /// The groove's cross-section, in the plane of the axis.
  ///
  /// A truncated V with 30° half-flanks, as the ISO form specifies. Both
  /// truncations matter: the crest flat keeps adjacent turns from meeting, and
  /// the root flat keeps the cutter from ending in a knife edge, which is the
  /// kind of near-degenerate geometry that makes booleans fail.
  ///
  /// `u` runs radially outward from the threaded surface and `v` along the
  /// axis, so the profile is built in the plane the sweep will carry it in.
  static SweepProfile _threadCutterProfile({
    required double pitch,
    required double depth,
    required bool internal,
  }) {
    final form = ThreadForm(pitch: pitch, internal: internal);
    final rootHalf = form.rootHalfWidth;
    // The cutter has to clearly exit the faceted wall, not graze it. A 5%
    // of-pitch overshoot is smaller than the inset of a typical cylinder
    // mesh, and the subtract then leaves an open sliver. Half a millimetre
    // (or half a pitch) sits in air for an external thread and in the bore
    // for an internal one, and is what actually closes the cut.
    final over = math.max(pitch * 0.5, 0.8);

    // Inward for an external thread, outward for an internal one.
    final dir = internal ? 1.0 : -1.0;
    final ring = <Vector3>[
      Vector3(-dir * over, -form.grooveHalfWidth - over / math.sqrt(3), 0),
      Vector3(-dir * over, form.grooveHalfWidth + over / math.sqrt(3), 0),
      Vector3(dir * depth, rootHalf, 0),
      Vector3(dir * depth, -rootHalf, 0),
    ];
    // Two things are deliberate here. The basis: the sweep carries x to the
    // radial direction and y along the axis, so the groove cuts inward rather
    // than sideways. And the anchor at the origin: it puts u = 0 — the crest —
    // on the helix, so the groove hangs off the major diameter. Left to the
    // default the section's centre would ride the helix instead, and the
    // thread would end up half a depth too large.
    return SweepProfile.closed(
      ring,
      normal: Vector3(0, 0, 1),
      xDir: Vector3(1, 0, 0),
      anchor: Vector3.zero(),
    );
  }

  /// The axial spans to thread.
  static List<(double, double)> _threadRegions({
    required double axisZMin,
    required double axisZMax,
    required double lengthMm,
    required bool fromEndA,
    required bool fromEndB,
  }) {
    return ThreadForm.regions(
      axisZMin: axisZMin,
      axisZMax: axisZMax,
      length: lengthMm <= 0 ? (axisZMax - axisZMin).abs() : lengthMm,
      fromEndA: fromEndA,
      fromEndB: fromEndB,
    );
  }
}
