import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../ijo_log.dart';
import '../tessellation.dart';
import 'mesh_data.dart';
import 'mesh_edit.dart';
import 'mesh_op.dart';
import 'mesh_polygon.dart';
import 'mesh_regions.dart';
import 'mesh_seal.dart';

/// Constant-radius edge/face fillet by local mesh rebuild.
///
/// One path, no boolean. A rolling ball of radius [r] sits tangent to the
/// two planes of each selected crease. That gives an inset
/// `setback = r / tan(α/2)` along each face (α = interior angle), a
/// cylindrical canal between the tangent lines, and a spherical patch
/// where three selected creases meet.
///
/// Face picks expand at every 3-crease vertex: two selected edges pull in
/// the third, so a cube face becomes eight creases (loop + uprights).
/// Acute, right, and obtuse edges use the same setback formula — a 90°
/// quarter-cylinder is just the special case α = π/2.
class MeshFillet {
  MeshFillet._();

  static MeshData fillet(
    MeshData mesh,
    List<(Vector3, Vector3)> edges, {
    required double radius,
    required bool inverse,
    MeshTessellation tessellation = MeshTessellation.standard,
  }) {
    if (radius.abs() < 1e-9 || edges.isEmpty) return mesh;
    final sw = Stopwatch()..start();
    final wanted = completeCornerEdges(mesh, edges);
    _dbg(
      'begin r=${radius.abs().toStringAsFixed(3)} inverse=$inverse '
      'picked=${edges.length} creases=${wanted.length} '
      '${IjoLog.size(mesh.vertices.length, mesh.faces.length)}',
    );
    final out = _run(
      mesh,
      wanted,
      radius.abs(),
      inverse: inverse,
      tessellation: tessellation,
    );
    _dbg(
      'done ${IjoLog.ms(sw)} '
      '${IjoLog.size(out.vertices.length, out.faces.length)} '
      'closed=${MeshSeal.isClosedSolid(out)} '
      'vol=${MeshEdit.signedVolume(out).abs().toStringAsFixed(1)}',
    );
    return out;
  }

  /// Adds the missing crease at any vertex where two of three feature
  /// edges are already selected. Four top edges of a cube therefore pick
  /// up the four uprights (eight creases total) and leave the opposite
  /// face sharp.
  ///
  /// Each requested segment also selects every feature edge that lies on
  /// it, so a split rim (one long chord, many shorts) is one crease.
  static List<(Vector3, Vector3)> completeCornerEdges(
    MeshData mesh,
    List<(Vector3, Vector3)> edges,
  ) {
    if (edges.isEmpty) return edges;
    final features = MeshFaceRegions.featureEdges(mesh);
    if (features.isEmpty) return edges;

    String vKey(int i) => MeshFaceRegions.quantizePosition(mesh.vertices[i]);
    String eKey(int a, int b) =>
        MeshFaceRegions.edgeKey(mesh.vertices[a], mesh.vertices[b]);

    final atVertex = <String, List<(int, int)>>{};
    for (final e in features) {
      (atVertex[vKey(e.$1)] ??= []).add(e);
      (atVertex[vKey(e.$2)] ??= []).add(e);
    }

    final selected = <String>{};
    for (final (a, b) in edges) {
      var any = false;
      for (final e in features) {
        if (!_featureLiesOn(mesh, e.$1, e.$2, a, b)) continue;
        selected.add(eKey(e.$1, e.$2));
        any = true;
      }
      if (!any) {
        final hit = _matchFeature(mesh, a, b);
        if (hit == null) {
          selected.add(MeshFaceRegions.edgeKey(a, b));
        } else {
          selected.add(eKey(hit.$1, hit.$2));
        }
      }
    }

    var changed = true;
    while (changed) {
      changed = false;
      for (final incident in atVertex.values) {
        if (incident.length != 3) continue;
        final keys = [for (final e in incident) eKey(e.$1, e.$2)];
        var nSel = 0;
        var missing = 0;
        for (var i = 0; i < 3; i++) {
          if (selected.contains(keys[i])) {
            nSel++;
          } else {
            missing = i;
          }
        }
        if (nSel != 2) continue;
        selected.add(keys[missing]);
        changed = true;
      }
    }

    final out = <(Vector3, Vector3)>[];
    final seen = <String>{};
    void add(int ia, int ib) {
      final key = eKey(ia, ib);
      if (!seen.add(key)) return;
      if (!selected.contains(key)) return;
      out.add((mesh.vertices[ia].clone(), mesh.vertices[ib].clone()));
    }

    for (final e in features) {
      add(e.$1, e.$2);
    }
    return out.isEmpty ? edges : out;
  }

  static MeshData _run(
    MeshData mesh,
    List<(Vector3, Vector3)> wanted,
    double r, {
    required bool inverse,
    required MeshTessellation tessellation,
  }) {
    final src = MeshEdit.triangulate(mesh);
    final chains = _chains(src, wanted, r, inverse, tessellation);
    if (chains.isEmpty) {
      throw MeshOpException('Fillet: no creases with two supporting planes');
    }
    final corners = inverse ? const <_Corner>[] : _corners(src, wanted, r);
    final cornerAt = {
      for (final c in corners) MeshFaceRegions.quantizePosition(c.vertex),
    };
    for (final chain in chains) {
      chain.trimStart =
          cornerAt.contains(MeshFaceRegions.quantizePosition(chain.first));
      chain.trimEnd =
          cornerAt.contains(MeshFaceRegions.quantizePosition(chain.last));
    }
    _dbg('chains=${chains.length} corners=${corners.length} inverse=$inverse');
    for (var i = 0; i < chains.length; i++) {
      final c = chains[i];
      _dbg(
        '  chain[$i] ${_fmt(c.first)}->${_fmt(c.last)} '
        'len=${c.length.toStringAsFixed(2)} '
        'alpha=${(c.alpha * 180 / math.pi).toStringAsFixed(1)}deg '
        'setback=${c.setback.toStringAsFixed(3)} '
        'phi=${(c.phi * 180 / math.pi).toStringAsFixed(1)}deg '
        'nSeg=${c.nSeg} trim=${c.trimStart}/${c.trimEnd}',
      );
    }

    final out = _Builder();
    final consumed = <int>{};
    final seenRegion = <String>{};
    final rebuiltPlanes = <({Vector3 n, Vector3 p})>[];

    // Support faces use the whole CAD face (boolean T-junctions included)
    // and rebuild as one polygon with holes. End caps stay on the local
    // edge-connected island so a square-with-circle leftover is not
    // rebuilt as five independent filled loops.
    final seeds = <({int fi, bool end})>{};
    for (var fi = 0; fi < src.faces.length; fi++) {
      if (_faceTouchesAnyChain(src, fi, chains)) {
        seeds.add((fi: fi, end: false));
      }
      if (_faceIsEndCap(src, fi, chains, cornerAt)) {
        seeds.add((fi: fi, end: true));
      }
    }
    final islands = <({List<int> region, bool preferEnd})>[];
    for (final seed in seeds) {
      final region = seed.end
          ? _planarIsland(src, seed.fi)
          : _cadFace(src, seed.fi);
      if (region.isEmpty) continue;
      final key = '${seed.end ? "e" : "s"}:${([...region]..sort()).join(',')}';
      if (!seenRegion.add(key)) continue;
      islands.add((region: region, preferEnd: seed.end));
    }
    islands.sort((a, b) => b.region.length.compareTo(a.region.length));
    final cappedEnds = <String>{};

    for (final island in islands) {
      final region = island.region;
      if (region.any(consumed.contains)) continue;

      final nHat = _regionOutward(src, region);
      if (nHat == null) continue;
      var kind = _regionKind(src, region, nHat, chains, cornerAt);
      if (kind == _RegionKind.plain) continue;
      if (!island.preferEnd && kind == _RegionKind.end) {
        // CAD-face flood of a plane that only meets the crease at a
        // terminal — the local end island handles the profile.
        continue;
      }
      if (kind == _RegionKind.end) {
        final ends = _openEndsInRegion(src, region, chains, cornerAt);
        if (ends.isNotEmpty &&
            ends.every((e) => cappedEnds.contains(_endKey(e, nHat)))) {
          if (region.length <= 2) {
            consumed.addAll(region);
            _dbg('  drop already-capped sliver faces=${region.length}');
          } else {
            _dbg('  keep already-capped faces=${region.length}');
          }
          continue;
        }
      }

      final work = kind == _RegionKind.end
          ? _facesWithOpenEnd(src, region, nHat, chains, cornerAt)
          : region;
      if (work.isEmpty) continue;
      final loops = MeshFaceRegions.orderedBoundaryLoops(src, work);
      if (loops.isEmpty) continue;
      _dbg(
        'rebuild ${kind.name} faces=${work.length}/${region.length} '
        'loops=${loops.length} n=${_fmt(nHat)}',
      );
      final rebuiltLoops = <List<Vector3>>[];
      for (final loop in loops) {
        final pts = [for (final i in loop) src.vertices[i].clone()];
        final hasCrease = _loopHasCrease(pts, chains, nHat);
        final hasEnd = _loopHasOpenEnd(pts, chains, cornerAt);
        final List<Vector3> rebuilt;
        if (kind == _RegionKind.end && hasEnd) {
          rebuilt = _endLoop(pts, nHat, chains);
        } else if (kind == _RegionKind.support && hasCrease) {
          rebuilt = _supportLoop(pts, nHat, chains, corners);
        } else {
          rebuilt = _cleanLoop(pts, nHat, dropOn: const []);
        }
        if (rebuilt.length < 3) {
          _dbg('  loop dropped n=${rebuilt.length}');
          continue;
        }
        _dbg('  loop ${pts.length}->${rebuilt.length}');
        rebuiltLoops.add(rebuilt);
      }
      if (rebuiltLoops.isEmpty) continue;
      out.emitLoops(rebuiltLoops, nHat, _regionFaceId(src, work));
      consumed.addAll(work);
      rebuiltPlanes.add((n: nHat.clone(), p: src.vertices[src.faces[work.first][0]].clone()));
      if (kind == _RegionKind.end) {
        for (final e in _openEndsInRegion(src, work, chains, cornerAt)) {
          cappedEnds.add(_endKey(e, nHat));
        }
      }
    }

    // A T-junction sliver that still uses a crease vertex would keep the
    // sharp corner we just replaced. Drop it when it lies on a rebuilt plane.
    for (var fi = 0; fi < src.faces.length; fi++) {
      if (consumed.contains(fi)) continue;
      if (!_faceTouchesAnyChain(src, fi, chains) &&
          !_faceSharesChainVertex(src, fi, chains)) {
        continue;
      }
      if (_planarIsland(src, fi).length > 2) continue;
      if (!_coplanarWithAny(src, fi, rebuiltPlanes)) continue;
      _dbg('  drop sliver face=$fi');
      consumed.add(fi);
    }

    for (final chain in chains) {
      _emitCanal(out, chain, corners);
    }
    for (final corner in corners) {
      _emitSphere(out, corner, chains);
    }

    final srcIds = src.faceIds;
    final keptFaces = <List<int>>[];
    final keptIds = <int>[];
    for (var fi = 0; fi < src.faces.length; fi++) {
      if (consumed.contains(fi)) continue;
      final f = src.faces[fi];
      if (f.length < 3) continue;
      keptFaces.add(List<int>.from(f));
      keptIds.add(srcIds == null || fi >= srcIds.length ? -1 : srcIds[fi]);
    }
    final kept = MeshData(
      vertices: [for (final v in src.vertices) v.clone()],
      faces: keptFaces,
      faceIds: keptIds.every((id) => id < 0) ? null : keptIds,
    );
    final added = MeshData(
      vertices: out.verts,
      faces: out.faces,
      faceIds: out.ids.every((id) => id < 0) ? null : out.ids,
    );
    final splitKept = _splitFacesOnSeams(kept, added.vertices);
    var result = MeshEdit.combine([splitKept, added]);
    result = MeshEdit.weld(result, epsilon: 1e-4);
    if (MeshEdit.signedVolume(result) < 0) {
      result = MeshEdit.invert(result);
    }
    final open = MeshSeal.openBoundaryEdgeCount(result);
    _dbg(
      'weld ${IjoLog.size(result.vertices.length, result.faces.length)} '
      'open=$open vol=${MeshEdit.signedVolume(result).abs().toStringAsFixed(1)}',
    );
    if (open != 0) {
      throw MeshOpException(
        'Fillet left $open open edges — the rebuild did not close',
      );
    }
    return result;
  }

  // ── rolling-ball crease ──────────────────────────────────────────────

  static List<_Chain> _chains(
    MeshData mesh,
    List<(Vector3, Vector3)> wanted,
    double r,
    bool inverse,
    MeshTessellation tessellation,
  ) {
    final features = MeshFaceRegions.featureEdges(mesh);

    String eKey(int a, int b) =>
        MeshFaceRegions.edgeKey(mesh.vertices[a], mesh.vertices[b]);
    String vKey(int i) => MeshFaceRegions.quantizePosition(mesh.vertices[i]);

    final segs = <(int, int)>[];
    final seen = <String>{};
    for (final e in features) {
      var hit = false;
      for (final (a, b) in wanted) {
        if (_featureLiesOn(mesh, e.$1, e.$2, a, b) ||
            MeshFaceRegions.edgeKey(a, b) == eKey(e.$1, e.$2)) {
          hit = true;
          break;
        }
      }
      if (!hit) continue;
      if (!seen.add(eKey(e.$1, e.$2))) continue;
      segs.add(e);
    }
    if (segs.isEmpty) return const [];

    final atVertex = <String, List<(int, int)>>{};
    for (final e in segs) {
      (atVertex[vKey(e.$1)] ??= []).add(e);
      (atVertex[vKey(e.$2)] ??= []).add(e);
    }

    bool collinear((int, int) a, (int, int) b) {
      final da = mesh.vertices[a.$2] - mesh.vertices[a.$1];
      final db = mesh.vertices[b.$2] - mesh.vertices[b.$1];
      if (da.length2 < 1e-20 || db.length2 < 1e-20) return false;
      return da.normalized().dot(db.normalized()).abs() > 0.995;
    }

    final used = <String>{};
    final chains = <_Chain>[];
    for (final start in segs) {
      final sk = eKey(start.$1, start.$2);
      if (used.contains(sk)) continue;

      // Walk only along collinear selected edges so a cube corner is a
      // chain break, not a bent pipe.
      final points = <int>[start.$1, start.$2];
      used.add(sk);

      void extend(bool forward) {
        while (true) {
          final end = forward ? points.last : points.first;
          final prev = forward ? points[points.length - 2] : points[1];
          final lastSeg = forward ? (prev, end) : (end, prev);
          (int, int)? next;
          for (final e in atVertex[vKey(end)] ?? const <(int, int)>[]) {
            final k = eKey(e.$1, e.$2);
            if (used.contains(k)) continue;
            if (!collinear(lastSeg, e)) continue;
            next = e;
            break;
          }
          if (next == null) return;
          used.add(eKey(next.$1, next.$2));
          final add = vKey(next.$1) == vKey(end) ? next.$2 : next.$1;
          if (forward) {
            points.add(add);
          } else {
            points.insert(0, add);
          }
        }
      }

      extend(true);
      extend(false);

      final chain = _chainFrom(
        mesh,
        [for (final i in points) mesh.vertices[i].clone()],
        r,
        inverse,
        tessellation,
      );
      if (chain != null) chains.add(chain);
    }
    return chains;
  }

  static _Chain? _chainFrom(
    MeshData mesh,
    List<Vector3> points,
    double r,
    bool inverse,
    MeshTessellation tessellation,
  ) {
    if (points.length < 2) return null;
    var len = 0.0;
    for (var i = 1; i < points.length; i++) {
      len += (points[i] - points[i - 1]).length;
    }
    if (len < 1e-6) return null;

    var t = points.last - points.first;
    if (t.length < 1e-9) t = points[1] - points[0];
    if (t.length < 1e-9) return null;
    t.normalize();

    final pair = _twoNormals(mesh, points.first, points.last, t);
    if (pair == null) {
      _dbg('  skip no-normals ${_fmt(points.first)}->${_fmt(points.last)}');
      return null;
    }
    final n0 = pair.$1;
    final n1 = pair.$2;
    final nd = n0.dot(n1).clamp(-1.0, 1.0);
    if (nd.abs() > 0.999) {
      _dbg('  skip flat dihedral ${nd.toStringAsFixed(3)}');
      return null;
    }

    // φ = angle between outward normals. Interior material angle α = π − φ.
    // Cube edge: φ = 90°, α = 90°, setback = r.
    // Acute solid (α < 90°): larger setback. Obtuse: smaller setback.
    final phi = math.acos(nd);
    final alpha = math.pi - phi;
    if (alpha < 1e-3 || alpha > math.pi - 1e-3) return null;
    final setback = r / math.tan((alpha / 2).clamp(1e-3, math.pi / 2 - 1e-3));

    // Spine offset along the outward-normal bisector. Distance to each
    // plane is r when the center is `edge − offset`.
    final denom = 1.0 + nd;
    if (denom.abs() < 1e-8) return null;
    final offset = (n0 + n1) * (r / denom);

    Vector3 dirOnFace(Vector3 n) {
      var d = n.cross(t);
      if (d.length < 1e-9) return Vector3.zero();
      d.normalize();
      final inward = -(n0 + n1);
      if (d.dot(inward) < 0) d = -d;
      return d;
    }

    final d0 = dirOnFace(n0);
    final d1 = dirOnFace(n1);
    if (d0.length2 < 1e-16 || d1.length2 < 1e-16) return null;

    var axis = n0.cross(n1);
    if (axis.length2 < 1e-16) return null;
    axis.normalize();
    // Keep n0 × n1 so a positive rotation carries n0 onto n1. Do not
    // flip to match the crease tangent — that sends the canal the
    // wrong way around the ball.

    // Two radius-r circles pass through the same tangent points. The
    // inner center is the rolling-ball spine (convex round). The other
    // center is the inverse / groove profile.
    final inner0 = points.first - offset;
    final tan0 = inner0 + n0 * r;
    final tan1 = inner0 + n1 * r;
    final other0 = _otherCenter(tan0, tan1, t, r, inner0);
    final delta = points.last - points.first;
    final spine0 = inverse ? other0 : inner0;
    final spine1 = inverse ? other0 + delta : points.last - offset;

    // Cap the facet count so a medium body still live-previews.
    final arc = inverse ? (math.pi * 2 - phi) : phi;
    final nSeg = tessellation.filletArcSegments(r, arc).clamp(4, 8);

    return _Chain(
      points: points,
      t: t,
      n0: n0,
      n1: n1,
      d0: d0,
      d1: d1,
      axis: axis,
      phi: phi,
      alpha: alpha,
      setback: setback,
      r: r,
      inverse: inverse,
      inner0: inner0,
      spine0: spine0,
      spine1: spine1,
      nSeg: nSeg,
    );
  }

  /// The second radius-[r] center through [t0] and [t1], in the plane
  /// perpendicular to [tangent]. [prefer] is the rolling-ball (inner) center.
  static Vector3 _otherCenter(
    Vector3 t0,
    Vector3 t1,
    Vector3 tangent,
    double r,
    Vector3 prefer,
  ) {
    final mid = (t0 + t1) * 0.5;
    final chord = t1 - t0;
    final half = chord.length * 0.5;
    final h2 = r * r - half * half;
    if (h2 <= 1e-16 || chord.length2 < 1e-20) return prefer;
    var perp = tangent.cross(chord);
    if (perp.length2 < 1e-20) perp = t0 - prefer;
    if (perp.length2 < 1e-20) return prefer;
    perp.normalize();
    final h = math.sqrt(h2);
    final a = mid + perp * h;
    final b = mid - perp * h;
    return (a - prefer).length2 <= (b - prefer).length2 ? b : a;
  }

  static (Vector3, Vector3)? _twoNormals(
    MeshData mesh,
    Vector3 a,
    Vector3 b,
    Vector3 tangent,
  ) {
    final planar = MeshFaceRegions.planarFaceNormals(mesh);
    final clusters = <({Vector3 n, double area})>[];
    void add(Vector3 n, double area) {
      for (var i = 0; i < clusters.length; i++) {
        if (clusters[i].n.dot(n) > 0.98) {
          clusters[i] = (n: clusters[i].n, area: clusters[i].area + area);
          return;
        }
      }
      clusters.add((n: n.clone(), area: area));
    }

    var t = tangent.clone();
    if (t.length2 > 1e-20) t.normalize();

    for (var f = 0; f < mesh.faces.length; f++) {
      final face = mesh.faces[f];
      if (face.length < 3) continue;
      var on = false;
      for (var i = 0; i < face.length; i++) {
        final pa = mesh.vertices[face[i]];
        final pb = mesh.vertices[face[(i + 1) % face.length]];
        if (_onSeg(pa, a, b) && _onSeg(pb, a, b)) {
          on = true;
          break;
        }
      }
      if (!on) continue;
      final cr = (mesh.vertices[face[1]] - mesh.vertices[face[0]])
          .cross(mesh.vertices[face[2]] - mesh.vertices[face[0]]);
      final area = cr.length * 0.5;
      if (area < 1e-12) continue;
      final n = (f < planar.length ? planar[f] : null) ?? _triOutward(mesh, f);
      if (n == null) continue;
      if (t.length2 > 1e-20 && n.dot(t).abs() > 0.35) continue;
      add(n, area);
    }
    if (clusters.length < 2) return null;
    clusters.sort((x, y) => y.area.compareTo(x.area));
    return (clusters[0].n.normalized(), clusters[1].n.normalized());
  }

  // ── triple corners ───────────────────────────────────────────────────

  static List<_Corner> _corners(
    MeshData mesh,
    List<(Vector3, Vector3)> wanted,
    double r,
  ) {
    final features = MeshFaceRegions.featureEdges(mesh);
    if (features.isEmpty) return const [];
    String vKey(int i) => MeshFaceRegions.quantizePosition(mesh.vertices[i]);
    String eKey(int a, int b) =>
        MeshFaceRegions.edgeKey(mesh.vertices[a], mesh.vertices[b]);

    final atVertex = <String, List<(int, int)>>{};
    for (final e in features) {
      var isSel = false;
      for (final (a, b) in wanted) {
        if (_featureLiesOn(mesh, e.$1, e.$2, a, b) ||
            MeshFaceRegions.edgeKey(a, b) == eKey(e.$1, e.$2)) {
          isSel = true;
          break;
        }
      }
      if (!isSel) continue;
      (atVertex[vKey(e.$1)] ??= []).add(e);
      (atVertex[vKey(e.$2)] ??= []).add(e);
    }

    final out = <_Corner>[];
    final seen = <String>{};
    for (final entry in atVertex.entries) {
      if (entry.value.length != 3) continue;
      if (!seen.add(entry.key)) continue;
      var vi = entry.value[0].$1;
      if (vKey(vi) != entry.key) vi = entry.value[0].$2;
      final vAt = mesh.vertices[vi];

      final norms = <Vector3>[];
      void addNorm(Vector3 n) {
        for (final e in norms) {
          if (e.dot(n) > 0.995) return;
        }
        norms.add(n);
      }

      for (final e in entry.value) {
        for (final fi in MeshFaceRegions.facesForEdge(mesh, e.$1, e.$2)) {
          final n = _triOutward(mesh, fi);
          if (n != null) addNorm(n);
        }
        if (norms.length >= 3) continue;
        final pair = _twoNormals(
          mesh,
          mesh.vertices[e.$1],
          mesh.vertices[e.$2],
          mesh.vertices[e.$2] - mesh.vertices[e.$1],
        );
        if (pair != null) {
          addNorm(pair.$1);
          addNorm(pair.$2);
        }
      }
      if (norms.length != 3) continue;
      // Intersection of the three planes offset inward by r:
      // n_i · (center − vertex) = −r  ⇒  center = vertex − solve(N, r).
      final offset = _solve3(norms[0], norms[1], norms[2], r);
      if (offset == null) continue;
      out.add(
        _Corner(
          vertex: vAt.clone(),
          center: vAt - offset,
          normals: norms,
          radius: r,
        ),
      );
    }
    _dbg('corners found=${out.length}');
    return out;
  }

  /// Solve n_i · x = radius for i = 0,1,2.
  static Vector3? _solve3(Vector3 n0, Vector3 n1, Vector3 n2, double radius) {
    final det = n0.dot(n1.cross(n2));
    if (det.abs() < 1e-10) return null;
    return (n1.cross(n2) + n2.cross(n0) + n0.cross(n1)) * (radius / det);
  }

  // ── face classification & polygon rebuild ────────────────────────────

  static _RegionKind _regionKind(
    MeshData mesh,
    List<int> region,
    Vector3 nHat,
    List<_Chain> chains,
    Set<String> cornerAt,
  ) {
    var support = false;
    var end = false;
    for (final chain in chains) {
      if (nHat.dot(chain.n0) > 0.94 || nHat.dot(chain.n1) > 0.94) {
        if (_regionTouchesChain(mesh, region, chain)) support = true;
      }
      if (nHat.dot(chain.t).abs() > 0.92) {
        final cap0 = !chain.trimStart &&
            !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.first));
        final cap1 = !chain.trimEnd &&
            !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.last));
        if ((cap0 || cap1) &&
            _regionHasEnd(mesh, region, chain, cap0, cap1)) {
          end = true;
        }
      }
    }
    if (support) return _RegionKind.support;
    if (end) return _RegionKind.end;
    return _RegionKind.plain;
  }

  static bool _regionTouchesChain(
    MeshData mesh,
    List<int> region,
    _Chain chain,
  ) {
    for (final fi in region) {
      final f = mesh.faces[fi];
      for (var i = 0; i < f.length; i++) {
        final p = mesh.vertices[f[i]];
        final q = mesh.vertices[f[(i + 1) % f.length]];
        if (_onSeg(p, chain.first, chain.last) &&
            _onSeg(q, chain.first, chain.last)) {
          return true;
        }
      }
    }
    return false;
  }

  static bool _regionHasEnd(
    MeshData mesh,
    List<int> region,
    _Chain chain,
    bool wantA,
    bool wantB,
  ) {
    for (final fi in region) {
      for (final i in mesh.faces[fi]) {
        final p = mesh.vertices[i];
        if (wantA && (p - chain.first).length < 0.4) return true;
        if (wantB && (p - chain.last).length < 0.4) return true;
      }
    }
    return false;
  }

  /// Inset every selected boundary edge in the plane, then intersect the
  /// offset lines at each vertex. Two selected edges at a 3-crease corner
  /// snap to the sphere/face tangent so the canal and the sphere share verts.
  static List<Vector3> _supportLoop(
    List<Vector3> raw,
    Vector3 nHat,
    List<_Chain> chains,
    List<_Corner> corners,
  ) {
    final pts = _cleanLoop(raw, nHat, dropOn: chains);
    if (pts.length < 3) return pts;
    final n = pts.length;
    var centroid = Vector3.zero();
    for (final p in pts) {
      centroid += p;
    }
    centroid /= n.toDouble();

    final offsets = List<double>.filled(n, 0);
    for (var i = 0; i < n; i++) {
      final chain = _chainCovering(chains, pts[i], pts[(i + 1) % n]);
      if (chain == null) continue;
      if (nHat.dot(chain.n0) < 0.94 && nHat.dot(chain.n1) < 0.94) continue;
      offsets[i] = chain.setback;
    }

    final out = <Vector3>[];
    for (var i = 0; i < n; i++) {
      final prev = (i - 1 + n) % n;
      final corner = _cornerAt(corners, pts[i]);
      if (corner != null) {
        // Sphere is tangent to this plane at center + r · n.
        var p = corner.center + nHat * corner.radius;
        p -= nHat * (p - pts[i]).dot(nHat);
        out.add(p);
        continue;
      }
      if (offsets[prev] == 0 && offsets[i] == 0) {
        // Boolean faces keep their Steiner verts so neighbors still match.
        out.add(pts[i].clone());
        continue;
      }
      final hit = _offsetIntersect(
        pts[prev],
        pts[i],
        offsets[prev],
        pts[i],
        pts[(i + 1) % n],
        offsets[i],
        nHat,
        centroid,
      );
      out.add(hit ?? pts[i].clone());
    }
    return _cleanLoop(out, nHat, dropOn: chains);
  }

  /// Replace a 1-crease terminal vertex with the canal profile, which lies
  /// in this end face (the face whose normal follows the crease).
  static List<Vector3> _endLoop(
    List<Vector3> raw,
    Vector3 nHat,
    List<_Chain> chains,
  ) {
    final pts = _cleanLoop(raw, nHat, dropOn: chains);
    if (pts.length < 3) return pts;
    final out = <Vector3>[];
    for (var i = 0; i < pts.length; i++) {
      final chain = _chainEndingAt(chains, pts[i]);
      if (chain == null || nHat.dot(chain.t).abs() < 0.92) {
        out.add(pts[i].clone());
        continue;
      }
      if ((chain.trimStart && _near(pts[i], chain.first)) ||
          (chain.trimEnd && _near(pts[i], chain.last))) {
        out.add(pts[i].clone());
        continue;
      }
      final prev = pts[(i - 1 + pts.length) % pts.length];
      final next = pts[(i + 1) % pts.length];
      var uPrev = prev - pts[i];
      var uNext = next - pts[i];
      if (uPrev.length < 1e-9 || uNext.length < 1e-9) {
        out.add(pts[i].clone());
        continue;
      }
      uPrev.normalize();
      uNext.normalize();
      final alongPrev = pts[i] + uPrev * chain.setback;
      final alongNext = pts[i] + uNext * chain.setback;
      final spine = _near(pts[i], chain.first) ? chain.spine0 : chain.spine1;
      final arc = _profileArc(chain, spine);
      out.add(alongPrev);
      if (arc.length >= 2) {
        final dStart = (alongPrev - arc.first).length +
            (alongNext - arc.last).length;
        final dFlip = (alongPrev - arc.last).length +
            (alongNext - arc.first).length;
        if (dStart <= dFlip) {
          for (var k = 1; k < arc.length - 1; k++) {
            out.add(arc[k].clone());
          }
        } else {
          for (var k = arc.length - 2; k >= 1; k--) {
            out.add(arc[k].clone());
          }
        }
      }
      out.add(alongNext);
      _dbg(
        '  end-arc at ${_fmt(pts[i])} prev=${_fmt(alongPrev)} '
        'next=${_fmt(alongNext)} samples=${arc.length} '
        'arc0=${arc.isEmpty ? "-" : _fmt(arc.first)} '
        'arcN=${arc.length < 2 ? "-" : _fmt(arc.last)}',
      );
    }
    return _cleanLoop(out, nHat, dropOn: chains);
  }

  static _Chain? _chainCovering(List<_Chain> chains, Vector3 p, Vector3 q) {
    for (final c in chains) {
      if (_onSeg(p, c.first, c.last) && _onSeg(q, c.first, c.last)) return c;
    }
    return null;
  }

  static _Chain? _chainEndingAt(List<_Chain> chains, Vector3 p) {
    for (final c in chains) {
      if (_near(p, c.first) || _near(p, c.last)) return c;
    }
    return null;
  }

  static _Corner? _cornerAt(List<_Corner> corners, Vector3 p) {
    for (final c in corners) {
      if ((c.vertex - p).length < 0.45) return c;
    }
    return null;
  }

  /// Intersection of two in-plane offset lines. Offset 0 keeps the original
  /// line; a positive offset shifts it into the face by that distance.
  static Vector3? _offsetIntersect(
    Vector3 a0,
    Vector3 a1,
    double offA,
    Vector3 b0,
    Vector3 b1,
    double offB,
    Vector3 nHat,
    Vector3 centroid,
  ) {
    var da = a1 - a0;
    var db = b1 - b0;
    if (da.length < 1e-9 || db.length < 1e-9) return null;
    da.normalize();
    db.normalize();

    Vector3 inward(Vector3 p0, Vector3 p1, Vector3 dir) {
      var d = nHat.cross(dir);
      if (d.length < 1e-9) return Vector3.zero();
      d.normalize();
      final mid = (p0 + p1) * 0.5;
      if (d.dot(centroid - mid) < 0) d = -d;
      return d;
    }

    final p0 = a0 + inward(a0, a1, da) * offA;
    final p1 = b0 + inward(b0, b1, db) * offB;
    return _lineIntersect(p0, da, p1, db, nHat);
  }

  static Vector3? _lineIntersect(
    Vector3 p0,
    Vector3 d0,
    Vector3 p1,
    Vector3 d1,
    Vector3 nHat,
  ) {
    final cross = d0.cross(d1);
    final denom = cross.dot(nHat);
    if (denom.abs() < 1e-10) {
      // Collinear: keep the shared vertex (caller already applied offset).
      return ((p0 + p1) * 0.5);
    }
    final w = p1 - p0;
    final t = w.cross(d1).dot(nHat) / denom;
    return p0 + d0 * t;
  }

  // ── canal + sphere ───────────────────────────────────────────────────

  /// Cylindrical strip from one tangent line to the other. Trimmed by [r]
  /// at a 3-crease so the sphere takes the corner.
  static void _emitCanal(_Builder out, _Chain chain, List<_Corner> corners) {
    var s0 = chain.spine0.clone();
    var s1 = chain.spine1.clone();
    final dir = s1 - s0;
    if (dir.length < 1e-9) return;
    final t = dir.normalized();

    if (chain.trimStart) {
      final c = _cornerAt(corners, chain.first);
      if (c != null) {
        // Stop the canal at the plane through the sphere center.
        final s = (c.center - s0).dot(t);
        if (s > 1e-6 && s < dir.length - 1e-6) s0 = s0 + t * s;
      }
    }
    if (chain.trimEnd) {
      final c = _cornerAt(corners, chain.last);
      if (c != null) {
        final s = (c.center - s0).dot(t);
        final span = (s1 - s0).dot(t);
        if (s > 1e-6 && s < span - 1e-6) s1 = s0 + t * s;
      }
    }
    if ((s1 - s0).dot(t) < 1e-6) return;

    // One strip along a straight crease. Extra stations along the spring
    // would T-junction against the rebuilt support face.
    final rows = [
      _profileArc(chain, s0),
      _profileArc(chain, s1),
    ];
    for (var i = 0; i + 1 < rows.length; i++) {
      final a = rows[i];
      final b = rows[i + 1];
      final n = math.min(a.length, b.length);
      if (n < 2) continue;
      final mid = (s0 + s1) * 0.5;
      for (var k = 0; k + 1 < n; k++) {
        // Outward (away from remaining solid) is from spine through the
        // surface: for a convex round that points toward the original edge.
        final rad = (a[k] + a[k + 1] + b[k]) / 3 - mid;
        out.emitQuad(a[k], a[k + 1], b[k + 1], b[k], rad, -2);
      }
    }
  }

  /// Spherical triangle on the rolling-ball, one fan per pair of incident
  /// planes. The midpoint is the unit-sum of the three face normals.
  static void _emitSphere(_Builder out, _Corner corner, List<_Chain> chains) {
    final ns = corner.normals;
    if (ns.length != 3) return;
    var midN = ns[0] + ns[1] + ns[2];
    if (midN.length2 < 1e-16) return;
    midN.normalize();
    final mid = corner.center + midN * corner.radius;
    final nSeg = chains.isEmpty
        ? 6
        : chains.fold<int>(4, (m, c) => math.max(m, c.nSeg));
    for (var i = 0; i < 3; i++) {
      final na = ns[i];
      final nb = ns[(i + 1) % 3];
      var axis = na.cross(nb);
      if (axis.length2 < 1e-16) continue;
      axis.normalize();
      final phi = math.acos(na.dot(nb).clamp(-1.0, 1.0));
      if (phi < 1e-4) continue;
      var prev = corner.center + na * corner.radius;
      for (var k = 1; k <= nSeg; k++) {
        final n = _rodrigues(na, axis, phi * k / nSeg)..normalize();
        final next = corner.center + n * corner.radius;
        final rad = prev - corner.center;
        out.emitTri(mid, prev, next, -3, prefer: rad);
        prev = next;
      }
    }
  }

  /// Profile samples from tangent on plane 0 to tangent on plane 1.
  /// Inverse uses the other radius-r arc through those same tangents.
  static List<Vector3> _profileArc(_Chain chain, Vector3 spine) {
    final nSeg = chain.nSeg;
    final out = <Vector3>[];
    if (!chain.inverse) {
      for (var k = 0; k <= nSeg; k++) {
        final n = _rodrigues(chain.n0, chain.axis, chain.phi * k / nSeg)
          ..normalize();
        out.add(spine + n * chain.r);
      }
      return out;
    }
    // Station along the crease, then the two face-tangents at that station.
    final param = (spine - chain.spine0).dot(chain.t);
    final inner = chain.inner0 + chain.t * param;
    final tan0 = inner + chain.n0 * chain.r;
    final tan1 = inner + chain.n1 * chain.r;
    var u = tan0 - spine;
    var v = tan1 - spine;
    final ru = u.length;
    if (ru < 1e-9 || v.length < 1e-9) return [tan0, tan1];
    u.normalize();
    v.normalize();
    var axis = u.cross(v);
    if (axis.length2 < 1e-16) axis = chain.axis;
    axis.normalize();
    final span = math.acos(u.dot(v).clamp(-1.0, 1.0));
    for (var k = 0; k <= nSeg; k++) {
      final d = _rodrigues(u, axis, span * k / nSeg)..normalize();
      out.add(spine + d * ru);
    }
    return out;
  }

  static List<Vector3> _cleanLoop(
    List<Vector3> pts,
    Vector3 nHat, {
    List<_Chain>? dropOn,
  }) {
    if (pts.length < 3) return pts;
    final out = <Vector3>[];
    for (final p in pts) {
      if (out.isEmpty || (out.last - p).length > 1e-4) out.add(p.clone());
    }
    if (out.length > 1 && (out.first - out.last).length < 1e-4) {
      out.removeLast();
    }
    // Drop collinear Steiner points on selected creases only. Points on
    // other edges stay so a boolean-split neighbor still shares them.
    var i = 0;
    while (i < out.length && out.length >= 3) {
      final a = out[(i - 1 + out.length) % out.length];
      final b = out[i];
      final c = out[(i + 1) % out.length];
      final cr = (b - a).cross(c - b);
      if (cr.length < 1e-8 || cr.normalized().dot(nHat).abs() < 0.02) {
        if ((c - a).normalized().dot((b - a).normalized()) > 0.995) {
          if (dropOn == null || _onAnyChain(b, dropOn)) {
            out.removeAt(i);
            continue;
          }
        }
      }
      i++;
    }
    return out;
  }

  /// Split kept triangles on new seam points that land in the interior of
  /// an existing edge (setback points on a boolean-split neighbor).
  static MeshData _splitFacesOnSeams(MeshData mesh, List<Vector3> seams) {
    if (seams.isEmpty || mesh.faces.isEmpty) return mesh;
    final verts = [for (final v in mesh.vertices) v.clone()];
    final faces = <List<int>>[];
    final ids = <int>[];
    final srcIds = mesh.faceIds;

    int addVert(Vector3 p) {
      for (var i = 0; i < verts.length; i++) {
        if ((verts[i] - p).length < 1e-4) return i;
      }
      verts.add(p.clone());
      return verts.length - 1;
    }

    Vector3? onEdge(Vector3 a, Vector3 b) {
      final ab = b - a;
      final len = ab.length;
      if (len < 1e-9) return null;
      Vector3? best;
      var bestDist = 1e9;
      for (final s in seams) {
        final t = ((s - a).dot(ab) / (len * len));
        if (t < 0.02 || t > 0.98) continue;
        final dist = (a + ab * t - s).length;
        if (dist > 1e-3 || dist >= bestDist) continue;
        bestDist = dist;
        best = s;
      }
      return best;
    }

    void emit(List<int> tri, int id) {
      if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) return;
      faces.add(tri);
      ids.add(id);
    }

    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final f = mesh.faces[fi];
      if (f.length < 3) continue;
      final id = srcIds == null || fi >= srcIds.length ? -1 : srcIds[fi];
      final queue = [List<int>.from(f)];
      var guard = 0;
      while (queue.isNotEmpty && guard++ < 8) {
        final t = queue.removeLast();
        if (t.length < 3) continue;
        final pa = verts[t[0]];
        final pb = verts[t[1]];
        final pc = verts[t[2]];
        final ab = onEdge(pa, pb);
        if (ab != null) {
          final i = addVert(ab);
          queue.add([t[0], i, t[2]]);
          queue.add([i, t[1], t[2]]);
          continue;
        }
        final bc = onEdge(pb, pc);
        if (bc != null) {
          final i = addVert(bc);
          queue.add([t[0], t[1], i]);
          queue.add([t[0], i, t[2]]);
          continue;
        }
        final ca = onEdge(pc, pa);
        if (ca != null) {
          final i = addVert(ca);
          queue.add([t[0], t[1], i]);
          queue.add([t[1], t[2], i]);
          continue;
        }
        emit(t, id);
      }
    }
    return MeshData(
      vertices: verts,
      faces: faces,
      faceIds: ids.every((e) => e < 0) ? null : ids,
    );
  }

  static List<int> _planarIsland(MeshData mesh, int fi) {
    final region = MeshFaceRegions.regionForFace(mesh, fi);
    return region.isEmpty ? [fi] : region;
  }

  static List<int> _cadFace(MeshData mesh, int fi) {
    final region = MeshFaceRegions.selectionRegion(mesh, fi);
    return region.isEmpty ? _planarIsland(mesh, fi) : region;
  }

  static bool _loopHasCrease(
    List<Vector3> pts,
    List<_Chain> chains,
    Vector3 nHat,
  ) {
    for (var i = 0; i < pts.length; i++) {
      final chain = _chainCovering(chains, pts[i], pts[(i + 1) % pts.length]);
      if (chain == null) continue;
      if (nHat.dot(chain.n0) > 0.94 || nHat.dot(chain.n1) > 0.94) return true;
    }
    return false;
  }

  static bool _loopHasOpenEnd(
    List<Vector3> pts,
    List<_Chain> chains,
    Set<String> cornerAt,
  ) {
    for (final p in pts) {
      for (final chain in chains) {
        final cap0 = !chain.trimStart &&
            !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.first));
        final cap1 = !chain.trimEnd &&
            !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.last));
        if (cap0 && (p - chain.first).length < 0.4) return true;
        if (cap1 && (p - chain.last).length < 0.4) return true;
      }
    }
    return false;
  }

  static bool _faceTouchesAnyChain(
    MeshData mesh,
    int fi,
    List<_Chain> chains,
  ) {
    final f = mesh.faces[fi];
    if (f.length < 3) return false;
    for (var i = 0; i < f.length; i++) {
      final p = mesh.vertices[f[i]];
      final q = mesh.vertices[f[(i + 1) % f.length]];
      for (final chain in chains) {
        if (!_onSeg(p, chain.first, chain.last) ||
            !_onSeg(q, chain.first, chain.last)) {
          continue;
        }
        final e = q - p;
        if (e.length2 < 1e-16) continue;
        if (e.normalized().dot(chain.t).abs() >= 0.92) return true;
      }
    }
    return false;
  }

  static bool _faceIsEndCap(
    MeshData mesh,
    int fi,
    List<_Chain> chains,
    Set<String> cornerAt,
  ) {
    final n = _triOutward(mesh, fi);
    if (n == null) return false;
    final f = mesh.faces[fi];
    for (final chain in chains) {
      if (n.dot(chain.t).abs() < 0.92) continue;
      final cap0 = !chain.trimStart &&
          !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.first));
      final cap1 = !chain.trimEnd &&
          !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.last));
      if (!cap0 && !cap1) continue;
      for (final i in f) {
        final p = mesh.vertices[i];
        if (cap0 && (p - chain.first).length < 0.4) return true;
        if (cap1 && (p - chain.last).length < 0.4) return true;
      }
    }
    return false;
  }

  static List<int> _facesWithOpenEnd(
    MeshData mesh,
    List<int> region,
    Vector3 nHat,
    List<_Chain> chains,
    Set<String> cornerAt,
  ) {
    final ends = _openEndsInRegion(mesh, region, chains, cornerAt);
    if (ends.isEmpty) return const [];
    var planeP = mesh.vertices[mesh.faces[region.first][0]];
    final out = <int>[];
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final n = _triOutward(mesh, fi);
      if (n == null || n.dot(nHat) < 0.94) continue;
      final f = mesh.faces[fi];
      if (f.isEmpty) continue;
      if ((mesh.vertices[f[0]] - planeP).dot(nHat).abs() > 1e-3) continue;
      var hit = false;
      for (final i in f) {
        final p = mesh.vertices[i];
        for (final e in ends) {
          if ((p - e).length < 0.4) {
            hit = true;
            break;
          }
        }
        if (hit) break;
      }
      if (hit) out.add(fi);
    }
    return out;
  }

  static List<Vector3> _openEndsInRegion(
    MeshData mesh,
    List<int> region,
    List<_Chain> chains,
    Set<String> cornerAt,
  ) {
    final out = <Vector3>[];
    final seen = <String>{};
    void add(Vector3 p) {
      final k = MeshFaceRegions.quantizePosition(p);
      if (seen.add(k)) out.add(p);
    }

    for (final fi in region) {
      for (final i in mesh.faces[fi]) {
        final p = mesh.vertices[i];
        for (final chain in chains) {
          final cap0 = !chain.trimStart &&
              !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.first));
          final cap1 = !chain.trimEnd &&
              !cornerAt.contains(MeshFaceRegions.quantizePosition(chain.last));
          if (cap0 && (p - chain.first).length < 0.4) add(chain.first);
          if (cap1 && (p - chain.last).length < 0.4) add(chain.last);
        }
      }
    }
    return out;
  }

  static String _endKey(Vector3 end, Vector3 nHat) {
    final n = nHat.normalized();
    return '${MeshFaceRegions.quantizePosition(end)}|'
        '${n.x.toStringAsFixed(2)},${n.y.toStringAsFixed(2)},${n.z.toStringAsFixed(2)}';
  }

  static bool _faceSharesChainVertex(
    MeshData mesh,
    int fi,
    List<_Chain> chains,
  ) {
    for (final i in mesh.faces[fi]) {
      if (_onAnyChain(mesh.vertices[i], chains)) return true;
    }
    return false;
  }

  static bool _onAnyChain(Vector3 p, List<_Chain> chains) {
    for (final c in chains) {
      if (_onSeg(p, c.first, c.last)) return true;
    }
    return false;
  }

  static bool _coplanarWithAny(
    MeshData mesh,
    int fi,
    List<({Vector3 n, Vector3 p})> planes,
  ) {
    final n = _triOutward(mesh, fi);
    if (n == null || planes.isEmpty) return false;
    final f = mesh.faces[fi];
    if (f.isEmpty) return false;
    final p = mesh.vertices[f[0]];
    for (final plane in planes) {
      if (n.dot(plane.n).abs() < 0.98) continue;
      if ((p - plane.p).dot(plane.n).abs() < 1e-3) return true;
    }
    return false;
  }

  static Vector3? _regionOutward(MeshData mesh, List<int> region) {
    // Trust source winding. Vertex-average "inside" tests flip the wrong
    // way when one side of a boolean is densely tessellated.
    var n = Vector3.zero();
    for (final fi in region) {
      final f = mesh.faces[fi];
      if (f.length < 3) continue;
      n += (mesh.vertices[f[1]] - mesh.vertices[f[0]])
          .cross(mesh.vertices[f[2]] - mesh.vertices[f[0]]);
    }
    if (n.length2 < 1e-20) return null;
    return n.normalized();
  }

  static int _regionFaceId(MeshData mesh, List<int> region) {
    final ids = mesh.faceIds;
    if (ids == null || region.isEmpty) return -1;
    final first = region.first;
    if (first < 0 || first >= ids.length) return -1;
    return ids[first];
  }

  static Vector3? _triOutward(MeshData mesh, int fi) {
    final f = mesh.faces[fi];
    if (f.length < 3) return null;
    final n = (mesh.vertices[f[1]] - mesh.vertices[f[0]])
        .cross(mesh.vertices[f[2]] - mesh.vertices[f[0]]);
    if (n.length2 < 1e-20) return null;
    return n.normalized();
  }

  static Vector3 _rodrigues(Vector3 v, Vector3 axis, double angle) {
    final c = math.cos(angle);
    final s = math.sin(angle);
    final k = axis.normalized();
    return v * c + k.cross(v) * s + k * (k.dot(v) * (1 - c));
  }

  static bool _near(Vector3 a, Vector3 b) => (a - b).length < 0.45;

  static bool _onSeg(Vector3 p, Vector3 a, Vector3 b, {double tol = 0.08}) {
    final ab = b - a;
    final len = ab.length;
    if (len < 1e-9) return (p - a).length < tol;
    final t = ((p - a).dot(ab) / (len * len)).clamp(0.0, 1.0);
    return (a + ab * t - p).length < tol;
  }

  static bool _featureLiesOn(
    MeshData mesh,
    int ia,
    int ib,
    Vector3 a,
    Vector3 b,
  ) {
    final pa = mesh.vertices[ia];
    final pb = mesh.vertices[ib];
    var want = b - a;
    final wantLen = want.length;
    if (wantLen < 1e-9) return false;
    want = want / wantLen;
    bool on(Vector3 p) {
      final along = ((p - a).dot(want)).clamp(0.0, wantLen);
      return (a + want * along - p).length < 1.0;
    }

    if (!on(pa) || !on(pb)) return false;
    final e = pb - pa;
    final eLen = e.length;
    if (eLen < 1e-9) return false;
    return (e / eLen).dot(want).abs() >= 0.92;
  }

  static (int, int)? _matchFeature(MeshData mesh, Vector3 a, Vector3 b) {
    final want = b - a;
    final wantLen = want.length;
    if (wantLen < 1e-9) return null;
    final wantDir = want / wantLen;
    (int, int)? best;
    var bestScore = double.infinity;
    for (final e in MeshFaceRegions.featureEdges(mesh)) {
      final pa = mesh.vertices[e.$1];
      final pb = mesh.vertices[e.$2];
      final dEnd = math.min(
        (pa - a).length + (pb - b).length,
        (pa - b).length + (pb - a).length,
      );
      var score = dEnd;
      final eVec = pb - pa;
      final eLen = eVec.length;
      if (eLen > 1e-9 && (eVec / eLen).dot(wantDir).abs() >= 0.92) {
        final mid = (pa + pb) * 0.5;
        final along = ((mid - a).dot(wantDir)).clamp(0.0, wantLen);
        final on = a + wantDir * along;
        final midDist = (mid - on).length;
        if (along >= -1e-6 && along <= wantLen + 1e-6 && midDist < 1.0) {
          score = math.min(score, midDist);
        }
      }
      if (score < bestScore) {
        bestScore = score;
        best = e;
      }
    }
    if (best == null || bestScore > 2.0) return null;
    return best;
  }

  static void _dbg(String msg) {
    final line = '[fillet] $msg';
    print(line);
    IjoLog.line(msg);
  }

  static String _fmt(Vector3 p) =>
      '(${p.x.toStringAsFixed(2)},${p.y.toStringAsFixed(2)},${p.z.toStringAsFixed(2)})';
}

enum _RegionKind { plain, support, end }

class _Chain {
  _Chain({
    required this.points,
    required this.t,
    required this.n0,
    required this.n1,
    required this.d0,
    required this.d1,
    required this.axis,
    required this.phi,
    required this.alpha,
    required this.setback,
    required this.r,
    required this.inverse,
    required this.inner0,
    required this.spine0,
    required this.spine1,
    required this.nSeg,
  });

  final List<Vector3> points;
  final Vector3 t;
  final Vector3 n0;
  final Vector3 n1;
  final Vector3 d0;
  final Vector3 d1;
  final Vector3 axis;
  final double phi;
  final double alpha;
  final double setback;
  final double r;
  final bool inverse;
  final Vector3 inner0;
  final Vector3 spine0;
  final Vector3 spine1;
  final int nSeg;
  bool trimStart = false;
  bool trimEnd = false;

  Vector3 get first => points.first;
  Vector3 get last => points.last;
  double get length {
    var s = 0.0;
    for (var i = 1; i < points.length; i++) {
      s += (points[i] - points[i - 1]).length;
    }
    return s;
  }
}

class _Corner {
  const _Corner({
    required this.vertex,
    required this.center,
    required this.normals,
    required this.radius,
  });

  final Vector3 vertex;
  final Vector3 center;
  final List<Vector3> normals;
  final double radius;
}

/// Welded triangle accumulator. Vertices that land on the same 0.01 mm
/// grid share an index so canals, spheres, and rebuilt faces meet.
class _Builder {
  final verts = <Vector3>[];
  final faces = <List<int>>[];
  final ids = <int>[];
  final idx = <String, int>{};

  int add(Vector3 p) {
    final key =
        '${(p.x * 1e4).round()}_${(p.y * 1e4).round()}_${(p.z * 1e4).round()}';
    return idx.putIfAbsent(key, () {
      verts.add(p.clone());
      return verts.length - 1;
    });
  }

  void emitTri(Vector3 a, Vector3 b, Vector3 c, int id, {Vector3? prefer}) {
    final ia = add(a);
    final ib = add(b);
    final ic = add(c);
    if (ia == ib || ib == ic || ia == ic) return;
    final cr = (verts[ib] - verts[ia]).cross(verts[ic] - verts[ia]);
    if (cr.length2 < 1e-20) return;
    if (prefer != null && cr.dot(prefer) < 0) {
      faces.add([ia, ic, ib]);
    } else {
      faces.add([ia, ib, ic]);
    }
    ids.add(id);
  }

  void emitQuad(
    Vector3 a,
    Vector3 b,
    Vector3 c,
    Vector3 d,
    Vector3 prefer,
    int id,
  ) {
    emitTri(a, b, c, id, prefer: prefer);
    emitTri(a, c, d, id, prefer: prefer);
  }

  void emitPoly(List<Vector3> poly, Vector3 nHat, int id) {
    emitLoops([poly], nHat, id);
  }

  void emitLoops(List<List<Vector3>> loops, Vector3 nHat, int id) {
    if (loops.isEmpty) return;
    final scored = [
      for (final loop in loops)
        if (loop.length >= 3) (area: _absLoopArea(loop, nHat), pts: loop),
    ];
    if (scored.isEmpty) return;
    scored.sort((a, b) => b.area.compareTo(a.area));
    final verts = <Vector3>[for (final p in scored.first.pts) p.clone()];
    final rings = <List<int>>[
      [for (var i = 0; i < scored.first.pts.length; i++) i],
    ];
    for (var i = 1; i < scored.length; i++) {
      final start = verts.length;
      verts.addAll([for (final p in scored[i].pts) p.clone()]);
      rings.add([for (var k = 0; k < scored[i].pts.length; k++) start + k]);
    }
    final tris = MeshPolygon.triangulateRings(verts, rings, nHat);
    if (tris.isEmpty) {
      // Hole bridge failed — fill the outer only. Extra loops stay holes.
      for (var i = 1; i + 1 < scored.first.pts.length; i++) {
        emitTri(
          scored.first.pts[0],
          scored.first.pts[i],
          scored.first.pts[i + 1],
          id,
          prefer: nHat,
        );
      }
      return;
    }
    for (final t in tris) {
      if (t.length < 3) continue;
      emitTri(verts[t[0]], verts[t[1]], verts[t[2]], id, prefer: nHat);
    }
  }
}

double _absLoopArea(List<Vector3> loop, Vector3 nHat) {
  if (loop.length < 3) return 0;
  var n = nHat.length2 < 1e-20 ? Vector3(0, 0, 1) : nHat.normalized();
  var u = n.cross(Vector3(1, 0, 0));
  if (u.length2 < 1e-12) u = n.cross(Vector3(0, 1, 0));
  u.normalize();
  final v = n.cross(u);
  var a = 0.0;
  for (var i = 0; i < loop.length; i++) {
    final p = loop[i] - loop[0];
    final q = loop[(i + 1) % loop.length] - loop[0];
    final pu = Vector2(p.dot(u), p.dot(v));
    final qu = Vector2(q.dot(u), q.dot(v));
    a += pu.x * qu.y - qu.x * pu.y;
  }
  return a.abs() * 0.5;
}
