import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import '../ijo_log.dart';
import '../csg/boolean.dart';
import '../tessellation.dart';
import 'mesh_build.dart';
import 'mesh_data.dart';
import 'mesh_edit.dart';
import 'mesh_hull.dart';
import 'mesh_op.dart';
import 'mesh_polygon.dart';
import 'mesh_regions.dart';
import 'mesh_seal.dart';
import 'mesh_topology.dart';

/// Chamfer for triangle meshes.
///
/// Kept as its own file so fillet can be rebuilt without taking chamfer
/// with it. This is the previous blend implementation with the fillet
/// entry point removed; chamfer is rewritten next.
class MeshChamfer {
  MeshChamfer._();

  static MeshData chamfer(
    MeshData mesh,
    List<(Vector3, Vector3)> edges, {
    required double distance,
    MeshTessellation tessellation = MeshTessellation.standard,
  }) {
    if (distance.abs() < 1e-9 || edges.isEmpty) return mesh;
    final wanted = completeCornerEdges(mesh, edges);
    IjoLog.trace(
      'mesh-chamfer d=$distance picked=${edges.length} creases=${wanted.length}',
    );
    return _run(
      mesh,
      wanted,
      distance.abs(),
      inverse: false,
      chamfer: true,
      tessellation: tessellation,
      name: 'Chamfer',
      requested: edges,
    );
  }

  /// Adds the missing crease at any vertex where two of three feature edges
  /// are already selected. On a cube, four top edges therefore pick up the
  /// four uprights and leave the opposite face sharp.
  ///
  /// Each requested segment also selects every feature edge that lies on it,
  /// so a CSG-split rim (one long chord, many Steiner shorts) is filleted as
  /// one crease rather than a single lucky fragment.
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
    required bool chamfer,
    required MeshTessellation tessellation,
    required String name,
    List<(Vector3, Vector3)> requested = const [],
  }) {
    final chains = _chains(mesh, wanted);
    final corners = inverse
        ? const <_CornerBlend>[]
        : _tripleCorners(mesh, wanted, r);
    final cornerAt = {
      for (final c in corners) MeshFaceRegions.quantizePosition(c.vertex),
    };
    IjoLog.line(
      'mesh-fillet chains=${chains.length} corners=${corners.length}',
    );
    if (!inverse) {
      final surfWatch = Stopwatch()..start();
      final surf = _surfaceFillet(
        mesh,
        chains,
        corners,
        r,
        chamfer: chamfer,
        tessellation: tessellation,
      );
      if (surf != null) {
        IjoLog.line(
          'mesh-fillet surface ${IjoLog.ms(surfWatch)} '
          '${IjoLog.size(surf.vertices.length, surf.faces.length)}',
        );
        return _finish(surf, name);
      }
      IjoLog.line('mesh-fillet surface miss ${IjoLog.ms(surfWatch)}');
    }
    final tools = <MeshData>[];
    final toolEnds = <(Vector3, Vector3)>[];
    for (final chain in chains) {
      final tool = chamfer
          ? _chamferTool(
              mesh,
              chain,
              r,
              trimStart: cornerAt.contains(
                MeshFaceRegions.quantizePosition(chain.points.first),
              ),
              trimEnd: cornerAt.contains(
                MeshFaceRegions.quantizePosition(chain.points.last),
              ),
            )
          : _filletTool(
              mesh,
              chain,
              r,
              inverse: inverse,
              tessellation: tessellation,
              trimStart: cornerAt.contains(
                MeshFaceRegions.quantizePosition(chain.points.first),
              ),
              trimEnd: cornerAt.contains(
                MeshFaceRegions.quantizePosition(chain.points.last),
              ),
            );
      if (tool == null || tool.faces.isEmpty) {
        IjoLog.line('mesh-fillet skip no cutter');
        continue;
      }
      if (MeshEdit.signedVolume(tool).abs() < 1e-9) continue;
      tools.add(tool);
      toolEnds.add((chain.points.first.clone(), chain.points.last.clone()));
    }

    final op = inverse ? MeshBoolean.unionOp : MeshBoolean.subtractOp;
    final total = Stopwatch()..start();
    IjoLog.line('mesh-fillet tools=${tools.length} sequential');
    var out = mesh;
    var lastGood = mesh;
    final denseInput = mesh.faces.length > 200;
    final order = [for (var i = 0; i < tools.length; i++) i];
    order.sort((a, b) {
      if (denseInput) {
        final pa = _segmentOnRequested(toolEnds[a], requested) ? 0 : 1;
        final pb = _segmentOnRequested(toolEnds[b], requested) ? 0 : 1;
        if (pa != pb) return pa.compareTo(pb);
        final sa = _shatteredScore(mesh, toolEnds[a].$1, toolEnds[a].$2);
        final sb = _shatteredScore(mesh, toolEnds[b].$1, toolEnds[b].$2);
        if (sa != sb) return sb.compareTo(sa);
      }
      final la = (toolEnds[a].$1 - toolEnds[a].$2).length;
      final lb = (toolEnds[b].$1 - toolEnds[b].$2).length;
      return lb.compareTo(la);
    });
    for (final i in order) {
      final next = _clip(
        out,
        tools[i],
        op,
        edge: toolEnds[i],
        radius: r,
        inverse: inverse,
        chamfer: chamfer,
        denseInput: denseInput,
      );
      if (next == null) {
        IjoLog.line('mesh-fillet skip tool $i/${tools.length}');
        continue;
      }
      final before = MeshEdit.signedVolume(out).abs();
      final after = MeshEdit.signedVolume(next).abs();
      final better = inverse ? after > before + 1e-4 : after + 1e-4 < before;
      if (!better) {
        IjoLog.line('mesh-fillet skip no-volume $i/${tools.length}');
        continue;
      }
      out = next;
      lastGood = out;
    }
    if (chamfer) {
      out = _punchChamferCorners(out, corners);
      if (MeshSeal.isClosedSolid(out)) lastGood = out;
    } else if (!inverse &&
        (mesh.faces.length <= 200 || !identical(out, mesh))) {
      out = _punchCorners(out, corners, denseInput: denseInput);
      if (MeshSeal.isClosedSolid(out)) lastGood = out;
    }
    if (!MeshSeal.isClosedSolid(out) && MeshSeal.isClosedSolid(lastGood)) {
      out = lastGood;
    }
    IjoLog.line(
      'mesh-fillet done ${IjoLog.ms(total)} '
      '${IjoLog.size(out.vertices.length, out.faces.length)} '
      'closed=${MeshSeal.isClosedSolid(out)}',
    );
    return _finish(out, name);
  }

  /// Offset planar n-gons, insert a cylinder / chamfer strip, and fill
  /// triple corners. Returns null when the result is not a closed solid
  /// with less volume than [mesh].
  static MeshData? _surfaceFillet(
    MeshData mesh,
    List<_CreaseChain> chains,
    List<_CornerBlend> corners,
    double r, {
    required bool chamfer,
    required MeshTessellation tessellation,
  }) {
    final draft = <_SurfBevel>[];
    for (final chain in chains) {
      final bevel = _surfBevel(
        mesh,
        chain,
        r,
        chamfer: chamfer,
        tessellation: tessellation,
        trimStart: false,
        trimEnd: false,
      );
      if (bevel != null) draft.add(bevel);
    }
    bool covered(_CornerBlend c) {
      var n = 0;
      for (final b in draft) {
        if ((b.rawFirst - c.vertex).length < 0.45 ||
            (b.rawLast - c.vertex).length < 0.45) {
          n++;
        }
      }
      return n >= 3;
    }

    final cornerAt = {
      for (final c in corners)
        if (covered(c)) MeshFaceRegions.quantizePosition(c.vertex),
    };
    final bevels = <_SurfBevel>[];
    for (final chain in chains) {
      final bevel = _surfBevel(
        mesh,
        chain,
        r,
        chamfer: chamfer,
        tessellation: tessellation,
        trimStart: cornerAt.contains(
          MeshFaceRegions.quantizePosition(chain.points.first),
        ),
        trimEnd: cornerAt.contains(
          MeshFaceRegions.quantizePosition(chain.points.last),
        ),
      );
      if (bevel != null) {
        bevels.add(bevel);
      } else {
        IjoLog.trace(
          'mesh-fillet surface skip chain '
          'n=${chain.points.length} '
          'len=${(chain.points.last - chain.points.first).length.toStringAsFixed(2)}',
        );
      }
    }
    if (bevels.isEmpty) return null;
    IjoLog.line(
      'mesh-fillet surface bevels=${bevels.length}/${chains.length} '
      'corners=${cornerAt.length}',
    );
    for (final b in bevels) {
      IjoLog.line(
        'mesh-fillet surface bevel '
        'p=${_fmt(b.p0)}->${_fmt(b.p1)} '
        'tan0=${_fmt(b.tan0)} tan1=${_fmt(b.tan1)} '
        'trim=${b.trimStart}/${b.trimEnd} setback=${b.setback.toStringAsFixed(2)}',
      );
    }

    final nSegMax = bevels.fold<int>(4, (m, b) => math.max(m, b.nSeg));
    final verts = <Vector3>[];
    final idx = <String, int>{};
    final faces = <List<int>>[];

    int add(Vector3 p) {
      final key =
          '${(p.x * 1e4).round()}_${(p.y * 1e4).round()}_${(p.z * 1e4).round()}';
      return idx.putIfAbsent(key, () {
        verts.add(p.clone());
        return verts.length - 1;
      });
    }

    void emitTri(int a, int b, int c) {
      if (a == b || b == c || a == c) return;
      if ((verts[b] - verts[a]).cross(verts[c] - verts[a]).length2 < 1e-20) {
        return;
      }
      faces.add([a, b, c]);
    }

    void emitPoly(List<Vector3> poly, Vector3 nHat) {
      if (poly.length < 3) return;
      final tmp = [for (final p in poly) p.clone()];
      final ring = [for (var i = 0; i < tmp.length; i++) i];
      final tris = MeshPolygon.triangulateRings(tmp, [ring], nHat);
      if (tris.isEmpty) {
        final ids = [for (final p in poly) add(p)];
        for (var i = 1; i + 1 < ids.length; i++) {
          emitTri(ids[0], ids[i], ids[i + 1]);
        }
        return;
      }
      for (final t in tris) {
        if (t.length < 3) continue;
        emitTri(add(tmp[t[0]]), add(tmp[t[1]]), add(tmp[t[2]]));
      }
    }

    final consumed = <int>{};
    final rebuiltEnds = <String>{};
    final seenRegion = <String>{};

    void rebuildRegion(List<int> faces, Vector3 nHat) {
      if (faces.isEmpty) return;
      if (faces.every(consumed.contains)) return;
      final key = ([...faces]..sort()).join(',');
      if (!seenRegion.add(key)) return;
      if (faces.length > 16) {
        final through = mesh.vertices[mesh.faces[faces.first][0]];
        final planar = _coplanarFaces(mesh, nHat, through);
        final target = planar.length > 24 ? planar : faces;
        final clipped = _clipFilletRemnants(mesh, target, nHat, bevels);
        if (clipped != null) {
          IjoLog.line(
            'mesh-fillet surface remnant-clip tris=${clipped.tris.length} '
            'faces=${clipped.consumed.length}',
          );
          for (final tri in clipped.tris) {
            emitTri(add(tri[0]), add(tri[1]), add(tri[2]));
          }
          consumed.addAll(clipped.consumed);
          return;
        }
        IjoLog.line(
          'mesh-fillet surface skip dense-region faces=${faces.length}',
        );
        return;
      }
      final loops = _cadLoops(mesh, faces);
      if (loops.isEmpty) return;
      final kinds = [
        for (final loop in loops) _loopKind(loop, nHat, bevels),
      ];
      if (kinds.every((k) => k == _LoopKind.none)) return;
      if (loops.any((loop) => _cleanLoop(loop).length > 14)) {
        IjoLog.line(
          'mesh-fillet surface skip curved-loop faces=${faces.length}',
        );
        return;
      }
      IjoLog.line(
        'mesh-fillet surface region loops=${loops.length} '
        'kinds=${[for (final k in kinds) k.name]} faces=${faces.length}',
      );
      for (var li = 0; li < loops.length; li++) {
        final pts = loops[li];
        final rebuilt = switch (kinds[li]) {
          _LoopKind.support => _insertMeshVertsOnLoop(
            mesh,
            _offsetLoop(pts, nHat, bevels),
          ),
          _LoopKind.end => _endLoop(
            pts,
            nHat,
            bevels,
            rebuiltEnds,
            coversDisk:
                faces.length > 4 && _cleanLoop(pts).length <= 4,
          ),
          _LoopKind.none => _cleanLoop(pts),
        };
        if (rebuilt.length < 3) continue;
        IjoLog.line(
          'mesh-fillet surface loop ${kinds[li].name} n=${rebuilt.length} '
          '${[for (final p in rebuilt) _fmt(p)]}',
        );
        emitPoly(rebuilt, nHat);
      }
      consumed.addAll(faces);
    }

    for (final b in bevels) {
      for (var fi = 0; fi < mesh.faces.length; fi++) {
        final fn = _triOutward(mesh, fi);
        if (fn == null) continue;
        final support = fn.dot(b.n0) > 0.94 || fn.dot(b.n1) > 0.94;
        final end = !support && fn.dot(b.t).abs() > 0.92;
        if (!support && !end) continue;
        if (support) {
          if (!_triTouchesBevel(mesh, fi, b)) continue;
          if (!_allVertsOnSupportPlane(mesh, fi, b)) continue;
        } else {
          if (!_allVertsOnEndPlane(mesh, fi, b)) continue;
        }
        List<int> region;
        if (end) {
          final sel = MeshFaceRegions.selectionRegion(mesh, fi);
          final loops = _cadLoops(mesh, sel);
          final simple = sel.length <= 16 &&
              loops.length == 1 &&
              _cleanLoop(loops.first).length <= 8;
          region = simple ? sel : _endIsland(mesh, fi, fn, b);
        } else {
          region = MeshFaceRegions.selectionRegion(mesh, fi);
        }
        rebuildRegion(region, fn);
      }
    }

    for (var fi = 0; fi < mesh.faces.length; fi++) {
      if (consumed.contains(fi)) continue;
      final f = mesh.faces[fi];
      if (f.length < 3) continue;
      emitTri(
        add(mesh.vertices[f[0]]),
        add(mesh.vertices[f[1]]),
        add(mesh.vertices[f[2]]),
      );
    }

    for (final b in bevels) {
      if (chamfer) {
        final a0 = add(b.p0 + b.d0 * b.setback);
        final a1 = add(b.p0 + b.d1 * b.setback);
        final c0 = add(b.p1 + b.d0 * b.setback);
        final c1 = add(b.p1 + b.d1 * b.setback);
        final outward = b.d0 + b.d1;
        final cr = (verts[a1] - verts[a0]).cross(verts[c0] - verts[a0]);
        if (cr.dot(outward) >= 0) {
          emitTri(a0, a1, c1);
          emitTri(a0, c1, c0);
        } else {
          emitTri(a0, c1, a1);
          emitTri(a0, c0, c1);
        }
        continue;
      }
      final height = (b.p1 - b.p0).length;
      if (height < 1e-9) continue;
      final stations = <double>[0.0];
      if (height > 2) stations.add(height * 0.5);
      stations.add(height);
      final rows = <List<int>>[];
      for (final s in stations) {
        final c = b.c0 + b.t * s;
        final row = <int>[];
        for (var k = 0; k <= b.nSeg; k++) {
          final n = _rodrigues(b.n0, b.axis, b.phi * k / b.nSeg)..normalize();
          row.add(add(c + n * b.r));
        }
        rows.add(row);
      }
      for (var i = 0; i + 1 < rows.length; i++) {
        final midC = b.c0 + b.t * ((stations[i] + stations[i + 1]) * 0.5);
        for (var k = 0; k < b.nSeg; k++) {
          final a0 = rows[i][k];
          final a1 = rows[i][k + 1];
          final c0 = rows[i + 1][k];
          final c1 = rows[i + 1][k + 1];
          final rad = (verts[a0] + verts[a1] + verts[c0]) / 3 - midC;
          final cr = (verts[c0] - verts[a0]).cross(verts[a1] - verts[a0]);
          if (cr.dot(rad) >= 0) {
            emitTri(a0, c0, c1);
            emitTri(a0, c1, a1);
          } else {
            emitTri(a0, c1, c0);
            emitTri(a0, a1, c1);
          }
        }
      }
      void cap(bool start) {
        if (start ? b.trimStart : b.trimEnd) return;
        final p = start ? b.rawFirst : b.rawLast;
        if (rebuiltEnds.contains(MeshFaceRegions.quantizePosition(p))) {
          return;
        }
        final c = start ? b.c0 : b.c1;
        final outward = start ? -b.t : b.t;
        final cid = add(c);
        final arc = <int>[];
        for (var k = 0; k <= b.nSeg; k++) {
          final n = _rodrigues(b.n0, b.axis, b.phi * k / b.nSeg)..normalize();
          arc.add(add(c + n * b.r));
        }
        for (var k = 0; k < b.nSeg; k++) {
          final cr =
              (verts[arc[k]] - verts[cid]).cross(verts[arc[k + 1]] - verts[cid]);
          if (cr.dot(outward) >= 0) {
            emitTri(cid, arc[k], arc[k + 1]);
          } else {
            emitTri(cid, arc[k + 1], arc[k]);
          }
        }
      }

      cap(true);
      cap(false);
    }

    if (chamfer) {
      for (final c in corners) {
        if (!cornerAt.contains(MeshFaceRegions.quantizePosition(c.vertex))) {
          continue;
        }
        if (c.along.length != 3) continue;
        final pts = [for (final a in c.along) c.vertex + a * r];
        var n = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
        if (n.length2 < 1e-20) continue;
        if (n.dot(c.vertex - pts[0]) < 0) {
          emitPoly([pts[0], pts[2], pts[1]], n.normalized());
        } else {
          emitPoly(pts, n.normalized());
        }
      }
    } else {
      for (final c in corners) {
        if (!cornerAt.contains(MeshFaceRegions.quantizePosition(c.vertex))) {
          continue;
        }
        if (c.normals.length != 3) continue;
        final ns = c.normals;
        var midN = ns[0] + ns[1] + ns[2];
        if (midN.length2 < 1e-16) continue;
        midN.normalize();
        final mid = add(c.center + midN * c.radius);
        for (var i = 0; i < 3; i++) {
          final na = ns[i];
          final nb = ns[(i + 1) % 3];
          var axis = na.cross(nb);
          if (axis.length2 < 1e-16) continue;
          axis.normalize();
          final phi = math.acos(na.dot(nb).clamp(-1.0, 1.0));
          if (phi < 1e-4) continue;
          var prev = add(c.center + na * c.radius);
          for (var k = 1; k <= nSegMax; k++) {
            final n = _rodrigues(na, axis, phi * k / nSegMax)..normalize();
            final next = add(c.center + n * c.radius);
            final cr =
                (verts[prev] - verts[mid]).cross(verts[next] - verts[mid]);
            final rad = verts[prev] - c.center;
            if (cr.dot(rad) >= 0) {
              emitTri(mid, prev, next);
            } else {
              emitTri(mid, next, prev);
            }
            prev = next;
          }
        }
      }
    }

    final built = MeshData(vertices: verts, faces: faces);
    final closed = _closeSurface(built);
    if (closed == null) return null;
    final v0 = MeshEdit.signedVolume(mesh).abs();
    final v1 = MeshEdit.signedVolume(closed).abs();
    if (v1 + 1e-4 >= v0) {
      IjoLog.line(
        'mesh-fillet surface volume ${v1.toStringAsFixed(1)} !< '
        '${v0.toStringAsFixed(1)}',
      );
      return null;
    }
    return closed;
  }

  static _LoopKind _loopKind(
    List<Vector3> pts,
    Vector3 nHat,
    List<_SurfBevel> bevels,
  ) {
    final loop = _cleanLoop(pts);
    if (loop.length < 3) return _LoopKind.none;
    for (var i = 0; i < loop.length; i++) {
      final a = loop[i];
      final b = loop[(i + 1) % loop.length];
      if (_bevelForEdge(a, b, bevels) != null) return _LoopKind.support;
    }
    for (final p in loop) {
      for (final b in bevels) {
        if (_isEndVertex(p, b, nHat)) return _LoopKind.end;
      }
    }
    return _LoopKind.none;
  }

  static List<Vector3> _offsetLoop(
    List<Vector3> pts,
    Vector3 nHat,
    List<_SurfBevel> bevels,
  ) {
    final loop = _cleanLoop(pts);
    final m = loop.length;
    if (m < 3) return loop;

    ({Vector3 origin, Vector3 dir})? tanLine(_SurfBevel b) {
      Vector3 n;
      if (nHat.dot(b.n0) > 0.94) {
        n = b.n0;
      } else if (nHat.dot(b.n1) > 0.94) {
        n = b.n1;
      } else {
        return null;
      }
      return (origin: b.c0 + n * b.r, dir: b.t);
    }

    final out = <Vector3>[];
    for (var i = 0; i < m; i++) {
      final prev = loop[(i + m - 1) % m];
      final cur = loop[i];
      final next = loop[(i + 1) % m];
      final b0 = _bevelForEdge(prev, cur, bevels);
      final b1 = _bevelForEdge(cur, next, bevels);
      final e0 = cur - prev;
      final e1 = next - cur;
      final l0 = b0 == null ? null : tanLine(b0);
      final l1 = b1 == null ? null : tanLine(b1);
      if (l0 != null && l1 != null) {
        out.add(
          _intersectLines(l0.origin, l0.dir, l1.origin, l1.dir) ??
              (l0.origin + l1.origin) * 0.5,
        );
        continue;
      }
      if (l0 != null && l1 == null) {
        final hit = _intersectLines(cur, e1, l0.origin, l0.dir);
        if (hit != null) {
          out.add(hit);
        } else {
          out.add(l0.origin + l0.dir * (cur - l0.origin).dot(l0.dir));
          out.add(cur.clone());
        }
        continue;
      }
      if (l0 == null && l1 != null) {
        final hit = _intersectLines(cur, e0, l1.origin, l1.dir);
        if (hit != null) {
          out.add(hit);
        } else {
          out.add(cur.clone());
          out.add(l1.origin + l1.dir * (cur - l1.origin).dot(l1.dir));
        }
        continue;
      }
      out.add(cur.clone());
    }
    return _cleanLoop(out, keepCollinear: true);
  }

  static List<Vector3> _insertMeshVertsOnLoop(MeshData mesh, List<Vector3> loop) {
    if (loop.length < 2) return loop;
    final out = <Vector3>[];
    for (var i = 0; i < loop.length; i++) {
      final a = loop[i];
      final b = loop[(i + 1) % loop.length];
      out.add(a.clone());
      final ab = b - a;
      final len2 = ab.length2;
      if (len2 < 1e-8) continue;
      final mids = <({double t, Vector3 p})>[];
      final seen = <String>{};
      for (final v in mesh.vertices) {
        if (!_onSeg(v, a, b, 0.08)) continue;
        final t = (v - a).dot(ab) / len2;
        if (t <= 0.02 || t >= 0.98) continue;
        final key = MeshFaceRegions.quantizePosition(v);
        if (!seen.add(key)) continue;
        mids.add((t: t, p: v.clone()));
      }
      mids.sort((x, y) => x.t.compareTo(y.t));
      for (final m in mids) {
        out.add(m.p);
      }
    }
    return out;
  }

  static List<List<Vector3>> _cadLoops(MeshData mesh, List<int> faces) {
    if (faces.length <= 4) {
      final raw = MeshFaceRegions.orderedBoundaryLoops(mesh, faces);
      if (raw.isNotEmpty) {
        return [
          for (final loop in raw)
            [for (final i in loop) mesh.vertices[i].clone()],
        ];
      }
    }
    final outline = MeshFaceRegions.selectionOutline(mesh, faces);
    final walked = _walkEdges(
      mesh,
      outline.isNotEmpty
          ? outline
          : MeshFaceRegions.boundaryEdges(mesh, faces),
    );
    if (walked.isNotEmpty) return walked;
    return [
      for (final loop in MeshFaceRegions.orderedBoundaryLoops(mesh, faces))
        [for (final i in loop) mesh.vertices[i].clone()],
    ];
  }

  static List<List<Vector3>> _walkEdges(
    MeshData mesh,
    List<(int, int)> edges,
  ) {
    if (edges.length < 3) return const [];
    final pos = <String, Vector3>{};
    final nbr = <String, List<String>>{};
    void link(Vector3 a, Vector3 b) {
      final ka = MeshFaceRegions.quantizePosition(a);
      final kb = MeshFaceRegions.quantizePosition(b);
      pos[ka] = a.clone();
      pos[kb] = b.clone();
      (nbr[ka] ??= []).add(kb);
      (nbr[kb] ??= []).add(ka);
    }

    for (final e in edges) {
      link(mesh.vertices[e.$1], mesh.vertices[e.$2]);
    }
    for (final n in nbr.values) {
      if (n.length != 2) return const [];
    }
    final unused = {...nbr.keys};
    final loops = <List<Vector3>>[];
    while (unused.isNotEmpty) {
      final start = unused.first;
      var prev = '';
      var cur = start;
      final loop = <Vector3>[];
      for (var i = 0; i < edges.length + 2; i++) {
        unused.remove(cur);
        loop.add(pos[cur]!.clone());
        final opts = nbr[cur]!;
        String? next;
        for (final o in opts) {
          if (o != prev) {
            next = o;
            break;
          }
        }
        if (next == null) break;
        if (next == start) {
          if (loop.length >= 3) loops.add(loop);
          break;
        }
        prev = cur;
        cur = next;
      }
    }
    return loops;
  }

  static List<Vector3> _endLoop(
    List<Vector3> pts,
    Vector3 nHat,
    List<_SurfBevel> bevels,
    Set<String> rebuiltEnds, {
    bool coversDisk = true,
  }) {
    final loop = _cleanLoop(pts);
    final out = <Vector3>[];
    for (var i = 0; i < loop.length; i++) {
      final prev = loop[(i + loop.length - 1) % loop.length];
      final cur = loop[i];
      final next = loop[(i + 1) % loop.length];
      _SurfBevel? hit;
      for (final b in bevels) {
        if (_isEndVertex(cur, b, nHat)) {
          hit = b;
          break;
        }
      }
      if (hit == null) {
        out.add(cur.clone());
        continue;
      }
      if (coversDisk) {
        rebuiltEnds.add(MeshFaceRegions.quantizePosition(cur));
      }
      var arc = _endArc(hit, cur);
      if (arc.length >= 2) {
        final dFirstPrev = (arc.first - prev).length;
        final dLastPrev = (arc.last - prev).length;
        if (dLastPrev + 1e-4 < dFirstPrev) {
          arc = arc.reversed.toList();
        } else if ((dFirstPrev - dLastPrev).abs() <= 1e-4 &&
            (arc.last - next).length > (arc.first - next).length) {
          arc = arc.reversed.toList();
        }
      }
      out.addAll(arc);
    }
    return _dedupLoop(out);
  }

  static List<Vector3> _endArc(_SurfBevel b, Vector3 at) {
    final start = (at - b.rawFirst).length <= (at - b.rawLast).length;
    final c = start ? b.c0 : b.c1;
    return [
      for (var k = 0; k <= b.nSeg; k++)
        c + (_rodrigues(b.n0, b.axis, b.phi * k / b.nSeg)..normalize()) * b.r,
    ];
  }

  static bool _isEndVertex(Vector3 p, _SurfBevel b, Vector3 nHat) {
    if (nHat.dot(b.t).abs() < 0.92) return false;
    if (!b.trimStart && (p - b.rawFirst).length < 0.45) return true;
    if (!b.trimEnd && (p - b.rawLast).length < 0.45) return true;
    return false;
  }

  static _SurfBevel? _bevelForEdge(
    Vector3 a,
    Vector3 b,
    List<_SurfBevel> bevels,
  ) {
    for (final bev in bevels) {
      if (_onSeg(a, bev.rawFirst, bev.rawLast) &&
          _onSeg(b, bev.rawFirst, bev.rawLast)) {
        return bev;
      }
    }
    return null;
  }

  static List<Vector3> _dedupLoop(List<Vector3> pts) {
    if (pts.length < 3) return [for (final p in pts) p.clone()];
    final out = <Vector3>[];
    for (final p in pts) {
      if (out.isEmpty || (out.last - p).length > 0.05) {
        out.add(p.clone());
      }
    }
    if (out.length > 1 && (out.first - out.last).length < 0.05) {
      out.removeLast();
    }
    return out;
  }

  static List<Vector3> _cleanLoop(
    List<Vector3> pts, {
    bool keepCollinear = false,
  }) {
    if (pts.length < 3) return [for (final p in pts) p.clone()];
    final dedup = <Vector3>[];
    for (final p in pts) {
      if (dedup.isEmpty || (dedup.last - p).length > 0.15) {
        dedup.add(p.clone());
      }
    }
    if (dedup.length > 1 && (dedup.first - dedup.last).length < 0.15) {
      dedup.removeLast();
    }
    if (dedup.length < 3) return dedup;
    if (keepCollinear) return dedup;
    final out = <Vector3>[];
    for (var i = 0; i < dedup.length; i++) {
      final prev = dedup[(i + dedup.length - 1) % dedup.length];
      final cur = dedup[i];
      final next = dedup[(i + 1) % dedup.length];
      final u = cur - prev;
      final v = next - cur;
      if (u.length < 0.15 || v.length < 0.15) continue;
      if (_unit(u).dot(_unit(v)) > 0.995) continue;
      out.add(cur);
    }
    return out.length >= 3 ? out : dedup;
  }

  static Vector3 _unit(Vector3 v) {
    final n = v.length;
    if (n < 1e-12) return Vector3.zero();
    return v / n;
  }

  static Vector3? _intersectLines(Vector3 p, Vector3 u, Vector3 q, Vector3 v) {
    if (u.length2 < 1e-20 || v.length2 < 1e-20) return null;
    final cr = u.cross(v);
    final cr2 = cr.length2;
    if (cr2 < 1e-20) return null;
    final t = (q - p).cross(v).dot(cr) / cr2;
    return p + u * t;
  }

  static String _fmt(Vector3 p) =>
      '${p.x.toStringAsFixed(2)},${p.y.toStringAsFixed(2)},${p.z.toStringAsFixed(2)}';

  static bool _onSeg(Vector3 p, Vector3 a, Vector3 b, [double eps = 0.45]) {
    final ab = b - a;
    final len = ab.length;
    if (len < 1e-9) return (p - a).length < eps;
    final t = ((p - a).dot(ab) / (len * len)).clamp(0.0, 1.0);
    return (a + ab * t - p).length < eps;
  }

  static bool _triTouchesBevel(MeshData mesh, int fi, _SurfBevel b) {
    final f = mesh.faces[fi];
    if (f.length < 3) return false;
    for (var i = 0; i < f.length; i++) {
      final a = mesh.vertices[f[i]];
      final c = mesh.vertices[f[(i + 1) % f.length]];
      if (_onSeg(a, b.rawFirst, b.rawLast) &&
          _onSeg(c, b.rawFirst, b.rawLast)) {
        return true;
      }
    }
    return false;
  }

  static bool _allVertsOnSupportPlane(
    MeshData mesh,
    int fi,
    _SurfBevel b,
  ) {
    final f = mesh.faces[fi];
    var on0 = true;
    var on1 = true;
    for (final i in f) {
      final p = mesh.vertices[i];
      if ((p - b.p0).dot(b.n0).abs() >= 0.08) on0 = false;
      if ((p - b.p0).dot(b.n1).abs() >= 0.08) on1 = false;
    }
    return on0 || on1;
  }

  static bool _allVertsOnEndPlane(MeshData mesh, int fi, _SurfBevel b) {
    final fn = _triOutward(mesh, fi);
    if (fn == null || fn.dot(b.t).abs() < 0.92) return false;
    final f = mesh.faces[fi];
    if (f.length < 3) return false;
    var hitEnd = false;
    for (final i in f) {
      final p = mesh.vertices[i];
      if ((p - mesh.vertices[f[0]]).dot(fn).abs() > 0.12) return false;
      if (!b.trimStart && (p - b.rawFirst).length < 0.5) hitEnd = true;
      if (!b.trimEnd && (p - b.rawLast).length < 0.5) hitEnd = true;
    }
    return hitEnd;
  }

  static List<int> _endIsland(
    MeshData mesh,
    int seed,
    Vector3 nHat,
    _SurfBevel b,
  ) {
    final through = mesh.vertices[mesh.faces[seed][0]];
    final planar = _coplanarFaces(mesh, nHat, through);
    if (planar.length > 24) {
      final quad = _cornerQuadrant(mesh, planar, nHat, b, through);
      if (quad.length >= 3) return quad;
    }
    final out = <int>[];
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final fn = _triOutward(mesh, fi);
      if (fn == null || fn.dot(nHat) < 0.94) continue;
      if (!_allVertsOnEndPlane(mesh, fi, b)) continue;
      out.add(fi);
    }
    if (out.isEmpty) return [seed];
    if (out.length > 8) return MeshFaceRegions.regionForFace(mesh, seed);
    return out..sort();
  }

  static List<int> _coplanarFaces(
    MeshData mesh,
    Vector3 nHat,
    Vector3 through,
  ) {
    final out = <int>[];
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final fn = _triOutward(mesh, fi);
      if (fn == null || fn.dot(nHat) < 0.94) continue;
      final f = mesh.faces[fi];
      if (f.length < 3) continue;
      var on = true;
      for (final i in f) {
        if ((mesh.vertices[i] - through).dot(nHat).abs() > 0.12) {
          on = false;
          break;
        }
      }
      if (on) out.add(fi);
    }
    return out;
  }

  static List<int> _cornerQuadrant(
    MeshData mesh,
    List<int> planar,
    Vector3 nHat,
    _SurfBevel b,
    Vector3 through,
  ) {
    final corner = _endCorner(b, nHat, through);
    var min0 = 1e300;
    var max0 = -1e300;
    var min1 = 1e300;
    var max1 = -1e300;
    for (final fi in planar) {
      for (final i in mesh.faces[fi]) {
        final d = mesh.vertices[i] - through;
        final s0 = d.dot(b.d0);
        final s1 = d.dot(b.d1);
        if (s0 < min0) min0 = s0;
        if (s0 > max0) max0 = s0;
        if (s1 < min1) min1 = s1;
        if (s1 > max1) max1 = s1;
      }
    }
    final mid0 = (min0 + max0) * 0.5;
    final mid1 = (min1 + max1) * 0.5;
    final dc = corner - through;
    final c0 = dc.dot(b.d0);
    final c1 = dc.dot(b.d1);
    final out = <int>[];
    for (final fi in planar) {
      final f = mesh.faces[fi];
      var g0 = 0.0;
      var g1 = 0.0;
      for (final i in f) {
        final d = mesh.vertices[i] - through;
        g0 += d.dot(b.d0);
        g1 += d.dot(b.d1);
      }
      g0 /= f.length;
      g1 /= f.length;
      if ((g0 - mid0) * (c0 - mid0) < -1e-6) continue;
      if ((g1 - mid1) * (c1 - mid1) < -1e-6) continue;
      out.add(fi);
    }
    return out..sort();
  }

  static Vector3 _endCorner(_SurfBevel b, Vector3 nHat, Vector3 through) {
    final sOn = (b.rawFirst - through).dot(nHat).abs() < 0.25;
    final eOn = (b.rawLast - through).dot(nHat).abs() < 0.25;
    if (sOn && !b.trimStart && !(eOn && !b.trimEnd)) return b.rawFirst;
    if (eOn && !b.trimEnd && !(sOn && !b.trimStart)) return b.rawLast;
    if (sOn && !b.trimStart) return b.rawFirst;
    if (eOn && !b.trimEnd) return b.rawLast;
    return (b.rawFirst - through).length <= (b.rawLast - through).length
        ? b.rawFirst
        : b.rawLast;
  }

  /// Keep the CSG leftover triangles that still belong after a plane–plane
  /// fillet, clipped to the L outside the cylinder end. Original sphere
  /// weld edges stay; only the cube-corner pocket is removed (the cap
  /// fills the pie).
  static ({List<List<Vector3>> tris, List<int> consumed})? _clipFilletRemnants(
    MeshData mesh,
    List<int> planar,
    Vector3 nHat,
    List<_SurfBevel> bevels,
  ) {
    if (planar.length < 8) return null;
    final through = mesh.vertices[mesh.faces[planar.first][0]];
    final consumed = <int>{};
    final tris = <List<Vector3>>[];
    var any = false;
    for (final b in bevels) {
      if (!_isEndVertex(b.rawFirst, b, nHat) &&
          !_isEndVertex(b.rawLast, b, nHat)) {
        continue;
      }
      final corner = _endCorner(b, nHat, through);
      if ((corner - through).dot(nHat).abs() > 0.25) continue;
      final island = _cornerQuadrant(mesh, planar, nHat, b, through);
      if (island.length < 2) continue;
      any = true;
      final r = b.setback;
      final d0 = b.d0;
      final d1 = b.d1;
      final tan0 = corner + d0 * r;
      final tan1 = corner + d1 * r;
      Vector3 snap(Vector3 p) {
        if ((p - tan0).length < 0.08) return tan0.clone();
        if ((p - tan1).length < 0.08) return tan1.clone();
        return p;
      }

      for (final fi in island) {
        consumed.add(fi);
        final f = mesh.faces[fi];
        if (f.length < 3) continue;
        final tri = [
          mesh.vertices[f[0]].clone(),
          mesh.vertices[f[1]].clone(),
          mesh.vertices[f[2]].clone(),
        ];
        bool keep(Vector3 p) {
          final a0 = (p - corner).dot(d0);
          final a1 = (p - corner).dot(d1);
          return a0 >= r - 1e-6 || a1 >= r - 1e-6;
        }

        final flags = [for (final p in tri) keep(p)];
        if (flags.every((v) => v)) {
          final kept = [snap(tri[0]), snap(tri[1]), snap(tri[2])];
          if (_triNormal(kept).dot(nHat) < 0) {
            tris.add([kept[0], kept[2], kept[1]]);
          } else {
            tris.add(kept);
          }
          continue;
        }
        if (flags.every((v) => !v)) continue;
        for (final raw in _clipTriangleToL(tri, corner, d0, d1, r)) {
          final kept = [snap(raw[0]), snap(raw[1]), snap(raw[2])];
          if (_triNormal(kept).dot(nHat) < 0) {
            tris.add([kept[0], kept[2], kept[1]]);
          } else {
            tris.add(kept);
          }
        }
      }
    }
    if (!any) return null;
    return (tris: tris, consumed: consumed.toList());
  }

  static Vector3 _triNormal(List<Vector3> t) =>
      (t[1] - t[0]).cross(t[2] - t[0]);

  static List<List<Vector3>> _clipTriangleToL(
    List<Vector3> tri,
    Vector3 corner,
    Vector3 d0,
    Vector3 d1,
    double r,
  ) {
    final a0 = _clipPolyHalf(tri, corner, d0, r);
    final a1 = _clipPolyHalf(tri, corner, d1, r);
    final only0 = _clipPolyHalf(a0, corner, -d1, -r);
    final out = <List<Vector3>>[];
    void fan(List<Vector3> poly) {
      if (poly.length < 3) return;
      for (var i = 1; i + 1 < poly.length; i++) {
        final n = (poly[i] - poly[0]).cross(poly[i + 1] - poly[0]);
        if (n.length2 < 1e-20) continue;
        out.add([poly[0].clone(), poly[i].clone(), poly[i + 1].clone()]);
      }
    }

    fan(only0);
    fan(a1);
    return out;
  }

  static List<Vector3> _clipPolyHalf(
    List<Vector3> poly,
    Vector3 origin,
    Vector3 n,
    double keepMin,
  ) {
    if (poly.length < 3) return const [];
    final out = <Vector3>[];
    for (var i = 0; i < poly.length; i++) {
      final a = poly[i];
      final b = poly[(i + 1) % poly.length];
      final da = (a - origin).dot(n);
      final db = (b - origin).dot(n);
      final aIn = da >= keepMin - 1e-9;
      final bIn = db >= keepMin - 1e-9;
      if (aIn && bIn) {
        out.add(b.clone());
      } else if (aIn && !bIn) {
        final t = ((keepMin - da) / (db - da)).clamp(0.0, 1.0);
        out.add(a + (b - a) * t);
      } else if (!aIn && bIn) {
        final t = ((keepMin - da) / (db - da)).clamp(0.0, 1.0);
        out.add(a + (b - a) * t);
        out.add(b.clone());
      }
    }
    return out;
  }

  static bool _cadSupports(
    MeshData mesh,
    _CreaseChain chain,
    Vector3 n,
  ) {
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final fn = _triOutward(mesh, fi);
      if (fn == null || fn.dot(n) < 0.94) continue;
      if (!_triTouchesChain(mesh, fi, chain)) continue;
      final region = MeshFaceRegions.selectionRegion(mesh, fi);
      if (region.isEmpty || region.length > 16) continue;
      final loops = MeshFaceRegions.orderedBoundaryLoops(mesh, region);
      for (final loop in loops) {
        final pts = [for (final i in loop) mesh.vertices[i]];
        final clean = _cleanLoop(pts);
        if (clean.length >= 3 && clean.length <= 8) return true;
      }
    }
    return false;
  }

  static bool _triTouchesChain(MeshData mesh, int fi, _CreaseChain chain) {
    final f = mesh.faces[fi];
    if (f.length < 3) return false;
    final a = chain.points.first;
    final b = chain.points.last;
    for (var i = 0; i < f.length; i++) {
      final p = mesh.vertices[f[i]];
      final q = mesh.vertices[f[(i + 1) % f.length]];
      if (_onSeg(p, a, b) && _onSeg(q, a, b)) return true;
    }
    return false;
  }

  static _SurfBevel? _surfBevel(
    MeshData mesh,
    _CreaseChain chain,
    double r, {
    required bool chamfer,
    required MeshTessellation tessellation,
    required bool trimStart,
    required bool trimEnd,
  }) {
    var chainLen = 0.0;
    for (var i = 1; i < chain.points.length; i++) {
      chainLen += (chain.points[i] - chain.points[i - 1]).length;
    }
    if (chainLen < 1.2) {
      IjoLog.trace(
        'mesh-fillet surface skip short ${chainLen.toStringAsFixed(2)}',
      );
      return null;
    }
    final geo = _creaseForChain(mesh, chain);
    if (geo == null) {
      IjoLog.trace('mesh-fillet surface skip no-crease');
      return null;
    }
    final (_, _, tEdge, n0, n1) = geo;
    if (n0.dot(n1) > 0.98 || n0.dot(n1) < -0.98) {
      IjoLog.trace(
        'mesh-fillet surface skip dihedral ${n0.dot(n1).toStringAsFixed(3)}',
      );
      return null;
    }
    if (!_chainPlanar(mesh, chain, n0)) {
      IjoLog.trace('mesh-fillet surface skip curved n0=$n0');
      return null;
    }
    if (!_chainPlanar(mesh, chain, n1)) {
      IjoLog.trace('mesh-fillet surface skip curved n1=$n1');
      return null;
    }
    if (!_cadSupports(mesh, chain, n0) || !_cadSupports(mesh, chain, n1)) {
      IjoLog.trace('mesh-fillet surface skip non-cad-support');
      return null;
    }
    Vector3 dirOnFace(Vector3 n) {
      var d = n.cross(tEdge);
      if (d.length < 1e-9) return Vector3.zero();
      d.normalize();
      final inward = -(n0 + n1);
      if (d.dot(inward) < 0) d = -d;
      return d;
    }

    final d0 = dirOnFace(n0);
    final d1 = dirOnFace(n1);
    if (d0.length2 < 1e-16 || d1.length2 < 1e-16) return null;
    final phi = math.acos(n0.dot(n1).clamp(-1.0, 1.0));
    final alpha = math.pi - phi;
    final setback = chamfer
        ? r
        : (alpha.abs() < 1e-6
            ? r
            : r / math.tan((alpha / 2).clamp(1e-3, math.pi / 2 - 1e-3)));
    final path = _endPath(
      chain.points,
      trimStart: trimStart,
      trimEnd: trimEnd,
      trim: setback,
      pad: 0,
    );
    if (path.length < 2) {
      IjoLog.trace(
        'mesh-fillet surface skip short-path trim=$setback '
        'trimS=$trimStart trimE=$trimEnd n=${chain.points.length}',
      );
      return null;
    }
    final p0 = path.first;
    final p1 = path.last;
    var t = p1 - p0;
    if (t.length < 1e-9) t = tEdge.clone();
    t.normalize();
    if ((p1 - p0).dot(t) < 1e-6) return null;
    final denom = 1.0 + n0.dot(n1);
    if (denom.abs() < 1e-8) return null;
    final offset = (n0 + n1) * (r / denom);
    final c0 = p0 - offset;
    final c1 = p1 - offset;
    var axis = n0.cross(n1);
    if (axis.length2 < 1e-16) return null;
    axis.normalize();
    final nSeg = chamfer
        ? 1
        : tessellation.filletArcSegments(r, phi).clamp(4, 8);
    return _SurfBevel(
      t: t,
      n0: n0,
      n1: n1,
      d0: d0,
      d1: d1,
      p0: p0,
      p1: p1,
      c0: c0,
      c1: c1,
      tan0: c0 + n0 * r,
      tan1: c0 + n1 * r,
      rawFirst: chain.points.first.clone(),
      rawLast: chain.points.last.clone(),
      r: r,
      phi: phi,
      setback: setback,
      trimStart: trimStart,
      trimEnd: trimEnd,
      axis: axis,
      nSeg: nSeg,
    );
  }

  static bool _chainPlanar(MeshData mesh, _CreaseChain chain, Vector3 n) {
    if (_planarSide(mesh, chain.repA, chain.repB, n)) return true;
    final hit = _matchFeature(mesh, chain.points.first, chain.points.last);
    if (hit != null && _planarSide(mesh, hit.$1, hit.$2, n)) return true;
    var t = chain.points.last - chain.points.first;
    if (t.length < 1e-9) return false;
    t.normalize();
    var area = 0.0;
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final f = mesh.faces[fi];
      if (f.length < 3) continue;
      final fn = _triOutward(mesh, fi);
      if (fn == null || fn.dot(n) < 0.95) continue;
      final a = mesh.vertices[f[0]];
      final b = mesh.vertices[f[1]];
      final c = mesh.vertices[f[2]];
      if (!_pointOnChain(a, chain) &&
          !_pointOnChain(b, chain) &&
          !_pointOnChain(c, chain)) {
        continue;
      }
      area += (b - a).cross(c - a).length * 0.5;
    }
    return area > 1e-6;
  }

  static bool _pointOnChain(Vector3 p, _CreaseChain chain) {
    for (var i = 1; i < chain.points.length; i++) {
      final a = chain.points[i - 1];
      final b = chain.points[i];
      var d = b - a;
      final len = d.length;
      if (len < 1e-9) continue;
      d = d / len;
      final along = ((p - a).dot(d)).clamp(0.0, len);
      if ((a + d * along - p).length < 0.4) return true;
    }
    return false;
  }

  static (Vector3, Vector3, Vector3, Vector3, Vector3)? _creaseForChain(
    MeshData mesh,
    _CreaseChain chain,
  ) {
    var geo = _crease(mesh, chain.repA, chain.repB);
    if (geo != null) return geo;
    for (var i = 1; i < chain.points.length; i++) {
      final hit = _matchFeature(mesh, chain.points[i - 1], chain.points[i]);
      if (hit == null) continue;
      geo = _crease(mesh, hit.$1, hit.$2);
      if (geo != null) return geo;
    }
    final a = chain.points.first;
    final b = chain.points.last;
    var t = b - a;
    if (t.length < 1e-9) return null;
    t.normalize();
    int? ia;
    int? ib;
    var da = 1e9;
    var db = 1e9;
    for (var i = 0; i < mesh.vertices.length; i++) {
      final d0 = (mesh.vertices[i] - a).length;
      final d1 = (mesh.vertices[i] - b).length;
      if (d0 < da) {
        da = d0;
        ia = i;
      }
      if (d1 < db) {
        db = d1;
        ib = i;
      }
    }
    if (ia == null || ib == null || da > 0.5 || db > 0.5) return null;
    var pair = _twoCreaseNormals(mesh, ia, ib);
    pair ??= _normalsNearSegment(mesh, a, b);
    if (pair == null) return null;
    return (a.clone(), b.clone(), t, pair.$1, pair.$2);
  }

  static (Vector3, Vector3)? _normalsNearSegment(
    MeshData mesh,
    Vector3 a,
    Vector3 b,
  ) {
    var t = b - a;
    final len = t.length;
    if (len < 1e-9) return null;
    t = t / len;
    final clusters = <({Vector3 n, double area})>[];
    for (var fi = 0; fi < mesh.faces.length; fi++) {
      final f = mesh.faces[fi];
      if (f.length < 3) continue;
      var near = false;
      for (final i in f) {
        final p = mesh.vertices[i];
        final along = ((p - a).dot(t)).clamp(0.0, len);
        if ((a + t * along - p).length < 0.45) {
          near = true;
          break;
        }
      }
      if (!near) continue;
      final fn = _triOutward(mesh, fi);
      if (fn == null || fn.dot(t).abs() > 0.25) continue;
      final cr = (mesh.vertices[f[1]] - mesh.vertices[f[0]])
          .cross(mesh.vertices[f[2]] - mesh.vertices[f[0]]);
      final area = cr.length * 0.5;
      if (area < 1e-10) continue;
      var merged = false;
      for (var i = 0; i < clusters.length; i++) {
        if (clusters[i].n.dot(fn) > 0.92) {
          clusters[i] = (n: clusters[i].n, area: clusters[i].area + area);
          merged = true;
          break;
        }
      }
      if (!merged) clusters.add((n: fn, area: area));
    }
    if (clusters.length < 2) return null;
    clusters.sort((x, y) => y.area.compareTo(x.area));
    if (clusters[0].n.dot(clusters[1].n) > 0.98) return null;
    return (clusters[0].n, clusters[1].n);
  }

  static bool _planarSide(MeshData mesh, int ia, int ib, Vector3 n) {
    final faces = MeshFaceRegions.facesAlongCrease(mesh, ia, ib);
    var hit = 0;
    for (final fi in faces) {
      final fn = _triOutward(mesh, fi);
      if (fn == null) continue;
      final d = fn.dot(n);
      if (d > 0.95) {
        hit++;
        continue;
      }
      if (d > 0.3 && d < 0.85) return false;
    }
    return hit > 0;
  }

  static Vector3 _rodrigues(Vector3 v, Vector3 axis, double ang) {
    final c = math.cos(ang);
    final s = math.sin(ang);
    return v * c + axis.cross(v) * s + axis * (axis.dot(v)) * (1 - c);
  }

  static MeshData? _closeSurface(MeshData m) {
    if (m.faces.isEmpty) return null;
    var s = MeshEdit.weld(m, epsilon: 1e-4);
    s = _solid(s);
    IjoLog.line(
      'mesh-fillet surface weld '
      '${IjoLog.size(s.vertices.length, s.faces.length)} '
      'open=${MeshSeal.openBoundaryEdgeCount(s)}',
    );
    if (MeshSeal.isClosedSolid(s)) return s;
    final all = [for (var i = 0; i < s.faces.length; i++) i];
    final open = MeshFaceRegions.boundaryEdges(s, all);
    var shown = 0;
    for (final e in open) {
      if (shown++ >= 18) break;
      final a = s.vertices[e.$1];
      final b = s.vertices[e.$2];
      IjoLog.line(
        'mesh-fillet surface open '
        '${a.x.toStringAsFixed(2)},${a.y.toStringAsFixed(2)},${a.z.toStringAsFixed(2)} -> '
        '${b.x.toStringAsFixed(2)},${b.y.toStringAsFixed(2)},${b.z.toStringAsFixed(2)} '
        'len=${(b - a).length.toStringAsFixed(2)}',
      );
    }
    s = MeshTopology.splitOpenEdgeTJunctions(s, epsilon: 1e-3);
    s = MeshEdit.weld(s, epsilon: 1e-5);
    s = _solid(s);
    IjoLog.line(
      'mesh-fillet surface tsplit1e-3 open=${MeshSeal.openBoundaryEdgeCount(s)}',
    );
    if (MeshSeal.isClosedSolid(s)) return s;
    s = MeshTopology.splitOpenEdgeTJunctions(s, epsilon: 1e-2);
    s = MeshEdit.weld(s, epsilon: 1e-5);
    s = _solid(s);
    IjoLog.line(
      'mesh-fillet surface tsplit1e-2 open=${MeshSeal.openBoundaryEdgeCount(s)}',
    );
    if (MeshSeal.isClosedSolid(s)) return s;
    s = MeshTopology.splitOpenEdgeTJunctions(s, epsilon: 5e-2);
    s = MeshEdit.weld(s, epsilon: 1e-5);
    s = _solid(s);
    IjoLog.line(
      'mesh-fillet surface tsplit5e-2 open=${MeshSeal.openBoundaryEdgeCount(s)}',
    );
    if (MeshSeal.isClosedSolid(s)) return s;
    s = MeshSeal.stitchTJunctions(s);
    IjoLog.line(
      'mesh-fillet surface stitch open=${MeshSeal.openBoundaryEdgeCount(s)}',
    );
    if (MeshSeal.isClosedSolid(s)) return _solid(s);
    s = MeshSeal.dropDegenerateFaces(s);
    s = MeshEdit.weld(s, epsilon: 1e-3);
    s = _solid(s);
    IjoLog.line(
      'mesh-fillet surface weld1e-3 open=${MeshSeal.openBoundaryEdgeCount(s)}',
    );
    if (MeshSeal.isClosedSolid(s)) return s;
    s = MeshSeal.sealSolid(s);
    IjoLog.line(
      'mesh-fillet surface seal open=${MeshSeal.openBoundaryEdgeCount(s)}',
    );
    if (!MeshSeal.isClosedSolid(s)) {
      s = _fillLeftoverOpens(s);
      IjoLog.line(
        'mesh-fillet surface fill-open open=${MeshSeal.openBoundaryEdgeCount(s)}',
      );
      if (!MeshSeal.isClosedSolid(s)) {
        s = _collapseShortOpens(s, 0.4);
        IjoLog.line(
          'mesh-fillet surface collapse-open open=${MeshSeal.openBoundaryEdgeCount(s)}',
        );
      }
    }
    if (!MeshSeal.isClosedSolid(s)) {
      final allLeft = [for (var i = 0; i < s.faces.length; i++) i];
      for (final e in MeshFaceRegions.boundaryEdges(s, allLeft).take(12)) {
        final a = s.vertices[e.$1];
        final b = s.vertices[e.$2];
        IjoLog.line(
          'mesh-fillet surface left ${_fmt(a)} -> ${_fmt(b)} '
          'len=${(b - a).length.toStringAsFixed(2)}',
        );
      }
      return null;
    }
    return _solid(s);
  }

  static MeshData _fillLeftoverOpens(MeshData mesh) {
    final all = [for (var i = 0; i < mesh.faces.length; i++) i];
    final open = MeshFaceRegions.boundaryEdges(mesh, all);
    if (open.length < 3 || open.length > 6) return mesh;
    final idx = <int>{};
    for (final e in open) {
      idx.add(e.$1);
      idx.add(e.$2);
    }
    if (idx.length < 3 || idx.length > 8) return mesh;
    final pts = [for (final i in idx) mesh.vertices[i].clone()];
    var n = Vector3.zero();
    for (var i = 0; i < pts.length; i++) {
      n += pts[i].cross(pts[(i + 1) % pts.length]);
    }
    if (n.length2 < 1e-16) {
      n = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
    }
    if (n.length2 < 1e-16) return mesh;
    n.normalize();
    var c = Vector3.zero();
    for (final p in pts) {
      c += p;
    }
    c = c / pts.length.toDouble();
    var u = n.cross(Vector3(1, 0, 0));
    if (u.length2 < 1e-12) u = n.cross(Vector3(0, 1, 0));
    u.normalize();
    final v = n.cross(u).normalized();
    pts.sort((a, b) {
      final aa = math.atan2((a - c).dot(v), (a - c).dot(u));
      final bb = math.atan2((b - c).dot(v), (b - c).dot(u));
      return aa.compareTo(bb);
    });
    final tmp = [for (final p in pts) p.clone()];
    final ring = [for (var i = 0; i < tmp.length; i++) i];
    final tris = MeshPolygon.triangulateRings(tmp, [ring], n);
    if (tris.isEmpty) return mesh;
    final verts = [for (final p in mesh.vertices) p.clone()];
    final faces = [for (final f in mesh.faces) List<int>.from(f)];
    final start = verts.length;
    verts.addAll(tmp);
    for (final t in tris) {
      if (t.length < 3) continue;
      faces.add([start + t[0], start + t[1], start + t[2]]);
    }
    var out = MeshData(vertices: verts, faces: faces);
    out = MeshEdit.weld(out, epsilon: 1e-4);
    return _solid(out);
  }

  static MeshData _collapseShortOpens(MeshData mesh, double maxLen) {
    final all = [for (var i = 0; i < mesh.faces.length; i++) i];
    final open = MeshFaceRegions.boundaryEdges(mesh, all);
    if (open.isEmpty) return mesh;
    final verts = [for (final v in mesh.vertices) v.clone()];
    var any = false;
    for (final e in open) {
      final a = verts[e.$1];
      final b = verts[e.$2];
      if ((b - a).length > maxLen) return mesh;
      final mid = (a + b) * 0.5;
      verts[e.$1] = mid.clone();
      verts[e.$2] = mid.clone();
      any = true;
    }
    if (!any) return mesh;
    return _solid(MeshEdit.weld(MeshData(vertices: verts, faces: mesh.faces)));
  }

  /// Convex wedge (crease + tangents + axis) minus the rolling-ball tube.
  static MeshData? _filletTool(
    MeshData mesh,
    _CreaseChain chain,
    double r, {
    required bool inverse,
    required MeshTessellation tessellation,
    required bool trimStart,
    required bool trimEnd,
  }) {
    final geo = _crease(mesh, chain.repA, chain.repB);
    if (geo == null) return null;
    final (_, _, tEdge, n0, n1) = geo;
    Vector3 dirOnFace(Vector3 n) {
      var d = n.cross(tEdge);
      if (d.length < 1e-9) return Vector3.zero();
      d.normalize();
      final inward = -(n0 + n1);
      if (d.dot(inward) < 0) d = -d;
      return d;
    }

    final d0 = dirOnFace(n0);
    final d1 = dirOnFace(n1);
    if (d0.length2 < 1e-16 || d1.length2 < 1e-16) return null;
    final phi = math.acos(n0.dot(n1).clamp(-1.0, 1.0));
    final alpha = math.pi - phi;
    final setback = alpha.abs() < 1e-6
        ? r
        : r / math.tan((alpha / 2).clamp(1e-3, math.pi / 2 - 1e-3));
    final path = _endPath(
      chain.points,
      trimStart: trimStart,
      trimEnd: trimEnd,
      trim: r,
      pad: r * 0.1,
    );
    if (path.length < 2) return null;
    final p0 = path.first;
    final p1 = path.last;
    var t = p1 - p0;
    if (t.length < 1e-9) t = tEdge;
    t.normalize();
    if ((p1 - p0).dot(t) < 1e-6) return null;

    final denom = 1.0 + n0.dot(n1);
    if (denom.abs() < 1e-8) return null;
    final offset = (n0 + n1) * (r / denom);
    final c0 = inverse ? p0 + offset : p0 - offset;
    final c1 = inverse ? p1 + offset : p1 - offset;
    final depth = r * 1.1;
    final v0 = p0;
    final v1 = p1;
    var wedge = _solid(
      MeshHull.ofPoints(
        setback > r * 1.2
            ? [
                v0,
                p0 + d0 * setback,
                p0 + d1 * setback,
                c0,
                v1,
                p1 + d0 * setback,
                p1 + d1 * setback,
                c1,
              ]
            : [
                v0,
                p0 - n0 * depth,
                p0 - n1 * depth,
                v1,
                p1 - n0 * depth,
                p1 - n1 * depth,
              ],
      ),
    );
    if (wedge.faces.isEmpty) {
      wedge = MeshBuild.sweepPolygon([
        p0,
        p0 + d0 * setback,
        p0 + d1 * setback,
      ], p1 - p0);
      if (wedge.faces.isNotEmpty) wedge = _solid(wedge);
    }
    if (wedge.faces.isEmpty) return null;

    final nSeg = tessellation.quality == MeshQuality.draft ? 12 : 16;
    final height = (c1 - c0).dot(t);
    if (height.abs() < 1e-6) return null;
    final tube = _solid(
      MeshBuild.cylinder(
        origin: height > 0 ? c0 : c1,
        axis: t,
        radius: r,
        height: height.abs(),
        segments: nSeg,
      ),
    );
    if (tube.faces.isEmpty) return null;

    final op = inverse
        ? MeshBoolean.intersectOp
        : MeshBoolean.subtractOp;
    var cutter = inverse
        ? _boolean(
            _solid(
              MeshHull.ofPoints([
                p0,
                p0 + n0 * depth,
                p0 + n1 * depth,
                p1,
                p1 + n0 * depth,
                p1 + n1 * depth,
              ]),
            ),
            tube,
            MeshBoolean.intersectOp,
          )
        : _boolean(wedge, tube, op);
    if (cutter == null && !inverse) {
      final cyl = _solid(
        MeshBuild.cylinder(
          origin: height > 0 ? c0 : c1,
          axis: t,
          radius: r,
          height: math.max(height.abs(), 1e-3),
          segments: nSeg,
        ),
      );
      cutter = _boolean(wedge, cyl, MeshBoolean.subtractOp);
    }
    return cutter;
  }

  static MeshData? _chamferTool(
    MeshData mesh,
    _CreaseChain chain,
    double r, {
    required bool trimStart,
    required bool trimEnd,
  }) {
    final geo = _crease(mesh, chain.repA, chain.repB);
    if (geo == null) return null;
    final (_, _, t, n0, n1) = geo;
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
    final path = _endPath(
      chain.points,
      trimStart: trimStart,
      trimEnd: trimEnd,
      trim: r,
      pad: 0.05,
    );
    if (path.length < 2) return null;
    final a0 = path.first;
    return _solid(
      MeshBuild.sweepPolygon([a0, a0 + d0 * r, a0 + d1 * r], path.last - a0),
    );
  }

  static List<Vector3> _endPath(
    List<Vector3> points, {
    required bool trimStart,
    required bool trimEnd,
    required double trim,
    required double pad,
  }) {
    if (points.length < 2) return points;
    var t0 = points[1] - points[0];
    var t1 = points.last - points[points.length - 2];
    if (t0.length < 1e-9 || t1.length < 1e-9) return points;
    t0.normalize();
    t1.normalize();
    final a = trimStart ? points.first + t0 * trim : points.first - t0 * pad;
    final b = trimEnd ? points.last - t1 * trim : points.last + t1 * pad;
    if ((b - a).dot(t0) < 1e-6) return const [];
    return [a, for (var i = 1; i < points.length - 1; i++) points[i], b];
  }

  static MeshData _punchChamferCorners(
    MeshData out,
    List<_CornerBlend> corners,
  ) {
    if (corners.isEmpty) return out;
    var body = out;
    var lastGood = out;
    for (final c in corners) {
      if (!body.vertices.any((v) => (v - c.vertex).length < 0.5)) continue;
      final tet = _solid(
        MeshHull.ofPoints([
          c.vertex,
          c.vertex + c.along[0] * c.setback,
          c.vertex + c.along[1] * c.setback,
          c.vertex + c.along[2] * c.setback,
        ]),
      );
      if (tet.faces.isEmpty) continue;
      final next = _clip(
        body,
        tet,
        MeshBoolean.subtractOp,
        allowLocal: false,
      );
      if (next == null) continue;
      if (MeshEdit.signedVolume(next).abs() + 1e-4 >=
          MeshEdit.signedVolume(body).abs()) {
        continue;
      }
      body = next;
      lastGood = body;
    }
    return MeshSeal.isClosedSolid(body) ? body : lastGood;
  }

  static MeshData _punchCorners(
    MeshData out,
    List<_CornerBlend> corners, {
    bool denseInput = false,
  }) {
    if (corners.isEmpty) return out;
    var body = out;
    var lastGood = out;
    for (final c in corners) {
      if (!body.vertices.any((v) => (v - c.vertex).length < 0.5)) continue;
      final tet = _solid(
        MeshHull.ofPoints([
          c.vertex,
          c.vertex + c.along[0] * c.setback,
          c.vertex + c.along[1] * c.setback,
          c.vertex + c.along[2] * c.setback,
        ]),
      );
      if (tet.faces.isEmpty) continue;
      final ball = MeshBuild.icosphere(
        center: c.center,
        radius: c.radius,
        subdivisions: 0,
      );
      final tool = _boolean(tet, ball, MeshBoolean.subtractOp);
      if (tool == null) continue;
      var inward = c.along[0] + c.along[1] + c.along[2];
      (Vector3, Vector3)? plane;
      if (inward.length > 1e-9) {
        inward.normalize();
        plane = (c.vertex + inward * (c.setback * 1.1), -inward);
      }
      final next = _clip(
        body,
        tool,
        MeshBoolean.subtractOp,
        allowLocal: denseInput,
        preferLocal: denseInput,
        denseInput: denseInput,
        cutPlane: plane,
        hint: c.vertex,
      );
      if (next == null) continue;
      if (MeshEdit.signedVolume(next).abs() + 1e-4 >=
          MeshEdit.signedVolume(body).abs()) {
        continue;
      }
      body = next;
      lastGood = body;
    }
    return MeshSeal.isClosedSolid(body) ? body : lastGood;
  }

  static List<_CreaseChain> _chains(
    MeshData mesh,
    List<(Vector3, Vector3)> wanted, {
    double maxLen = 1e9,
  }) {
    final features = MeshFaceRegions.featureEdges(mesh);
    final selected = <String, (int, int)>{};
    for (final (a, b) in wanted) {
      var any = false;
      for (final e in features) {
        if (!_featureLiesOn(mesh, e.$1, e.$2, a, b)) continue;
        selected[MeshFaceRegions.edgeKey(
              mesh.vertices[e.$1],
              mesh.vertices[e.$2],
            )] =
            e;
        any = true;
      }
      if (!any) {
        final hit = _matchFeature(mesh, a, b);
        if (hit != null) {
          selected[MeshFaceRegions.edgeKey(
                mesh.vertices[hit.$1],
                mesh.vertices[hit.$2],
              )] =
              hit;
        }
      }
    }
    if (selected.isEmpty) return const [];

    String vKey(int i) => MeshFaceRegions.quantizePosition(mesh.vertices[i]);
    final at = <String, List<(int, int)>>{};
    for (final e in selected.values) {
      (at[vKey(e.$1)] ??= []).add(e);
      (at[vKey(e.$2)] ??= []).add(e);
    }

    final unused = Map<String, (int, int)>.from(selected);
    final chains = <_CreaseChain>[];

    (int, int)? take(String key) {
      final e = unused.remove(key);
      if (e == null) return null;
      unused.remove(
        MeshFaceRegions.edgeKey(mesh.vertices[e.$1], mesh.vertices[e.$2]),
      );
      return e;
    }

    List<int> extend(int prev, int cur) {
      final extra = <int>[];
      var p = prev;
      var c = cur;
      while (true) {
        final incident = at[vKey(c)];
        if (incident == null) break;
        (int, int)? next;
        int? other;
        for (final e in incident) {
          final key = MeshFaceRegions.edgeKey(
            mesh.vertices[e.$1],
            mesh.vertices[e.$2],
          );
          if (!unused.containsKey(key)) continue;
          final o = vKey(e.$1) == vKey(c) ? e.$2 : e.$1;
          if (vKey(o) == vKey(p)) continue;
          if (!_sameCrease(mesh, p, c, o)) continue;
          next = e;
          other = o;
          break;
        }
        if (next == null || other == null) break;
        take(
          MeshFaceRegions.edgeKey(
            mesh.vertices[next.$1],
            mesh.vertices[next.$2],
          ),
        );
        extra.add(other);
        p = c;
        c = other;
      }
      return extra;
    }

    while (unused.isNotEmpty) {
      (int, int)? seed;
      for (final e in unused.values) {
        final da = at[vKey(e.$1)]?.length ?? 0;
        final db = at[vKey(e.$2)]?.length ?? 0;
        if (da == 1 || db == 1) {
          seed = e;
          break;
        }
      }
      seed ??= unused.values.first;
      take(
        MeshFaceRegions.edgeKey(mesh.vertices[seed.$1], mesh.vertices[seed.$2]),
      );
      final left = extend(seed.$2, seed.$1);
      final right = extend(seed.$1, seed.$2);
      final verts = [...left.reversed, seed.$1, seed.$2, ...right];
      if (verts.length < 2) continue;
      final pts = [for (final i in verts) mesh.vertices[i].clone()];
      final mid = verts.length ~/ 2;
      final repA = verts[mid > 0 ? mid - 1 : 0];
      final repB = verts[mid > 0 ? mid : 1];
      for (final piece in _splitPath(pts, maxLen)) {
        chains.add(_CreaseChain(repA: repA, repB: repB, points: piece));
      }
    }
    return chains;
  }

  static List<List<Vector3>> _splitPath(List<Vector3> pts, double maxLen) {
    if (pts.length < 2) return const [];
    if (maxLen >= 1e8) return [pts];
    var total = 0.0;
    for (var i = 1; i < pts.length; i++) {
      total += (pts[i] - pts[i - 1]).length;
    }
    if (total <= maxLen) return [pts];

    final out = <List<Vector3>>[];
    var cur = <Vector3>[pts.first.clone()];
    var acc = 0.0;
    for (var i = 1; i < pts.length; i++) {
      var remaining = pts[i] - cur.last;
      var remLen = remaining.length;
      while (acc + remLen > maxLen + 1e-9) {
        final need = maxLen - acc;
        if (need < 1e-9 || remLen < 1e-9) break;
        final cut = cur.last + remaining * (need / remLen);
        cur.add(cut.clone());
        out.add([for (final p in cur) p.clone()]);
        cur = [cut.clone()];
        acc = 0.0;
        remaining = pts[i] - cut;
        remLen = remaining.length;
      }
      cur.add(pts[i].clone());
      acc += remLen;
    }
    if (cur.length >= 2) out.add(cur);
    return out;
  }

  static bool _sameCrease(MeshData mesh, int a, int b, int c) {
    final p = mesh.vertices[a];
    final q = mesh.vertices[b];
    final r = mesh.vertices[c];
    var u = q - p;
    var v = r - q;
    if (u.length < 1e-9 || v.length < 1e-9) return false;
    u.normalize();
    v.normalize();
    if (u.dot(v) < 0.92) return false;
    final ga = _crease(mesh, a, b);
    final gb = _crease(mesh, b, c);
    if (ga == null || gb == null) return u.dot(v) > 0.995;
    bool same(Vector3 n, Vector3 m) => n.dot(m) > 0.98;
    return (same(ga.$4, gb.$4) && same(ga.$5, gb.$5)) ||
        (same(ga.$4, gb.$5) && same(ga.$5, gb.$4));
  }

  static MeshData? _boolean(MeshData a, MeshData b, int op) {
    if (a.faces.isEmpty || b.faces.isEmpty) return null;
    final raw = MeshBoolean.run(a, b, op, seal: false);
    final out = _close(raw);
    if (out == null || out.faces.isEmpty) return null;
    return _solid(out);
  }

  static MeshData? _clip(
    MeshData body,
    MeshData tool,
    int op, {
    (Vector3, Vector3)? edge,
    double? radius,
    bool inverse = false,
    bool chamfer = false,
    bool allowLocal = true,
    bool preferLocal = false,
    bool denseInput = false,
    (Vector3, Vector3)? cutPlane,
    Vector3? hint,
  }) {
    if (tool.faces.isEmpty) return body;
    final sw = Stopwatch()..start();
    final shattered =
        edge != null &&
        radius != null &&
        _shatteredCrease(body, edge.$1, edge.$2);
    if (allowLocal && denseInput && (shattered || preferLocal)) {
      final local = _clipLocal(
        body,
        tool,
        op,
        edge: edge,
        radius: radius,
        inverse: inverse,
        chamfer: chamfer,
        cutPlane: cutPlane,
        hint: hint,
      );
      if (local != null) {
        IjoLog.line(
          'mesh-fillet clip local '
          '${IjoLog.size(local.vertices.length, local.faces.length)} '
          '${IjoLog.ms(sw)}',
        );
        return local;
      }
      IjoLog.line('mesh-fillet clip local failed ${IjoLog.ms(sw)}');
      return null;
    }
    final next = MeshBoolean.run(body, tool, op, seal: false);
    final cand = _close(next);
    if (cand != null) {
      IjoLog.line(
        'mesh-fillet clip ok '
        '${IjoLog.size(cand.vertices.length, cand.faces.length)} '
        '${IjoLog.ms(sw)}',
      );
      return cand;
    }
    IjoLog.line(
      'mesh-fillet clip open='
      '${MeshSeal.openBoundaryEdgeCount(next)} ${IjoLog.ms(sw)}',
    );
    if (!allowLocal) return null;
    final local = _clipLocal(
      body,
      tool,
      op,
      edge: edge,
      radius: radius,
      inverse: inverse,
      chamfer: chamfer,
      cutPlane: cutPlane,
      hint: hint,
    );
    if (local != null) {
      IjoLog.line(
        'mesh-fillet clip local '
        '${IjoLog.size(local.vertices.length, local.faces.length)} '
        '${IjoLog.ms(sw)}',
      );
      return local;
    }
    IjoLog.line('mesh-fillet clip local failed ${IjoLog.ms(sw)}');
    return null;
  }

  /// Leftover walls lie on tessellated CSG faces, so a full-body clip opens.
  /// Slice off a simple slab, clip the leftover there, CSG-union the slab
  /// back together, then weld that onto the remaining (possibly dense) solid.
  static MeshData? _clipLocal(
    MeshData body,
    MeshData tool,
    int op, {
    (Vector3, Vector3)? edge,
    double? radius,
    bool inverse = false,
    bool chamfer = false,
    (Vector3, Vector3)? cutPlane,
    Vector3? hint,
  }) {
    if (body.vertices.isEmpty || tool.vertices.isEmpty) return null;
    final (tMin, tMax) = _bounds(tool);
    final boxMin = Vector3(
      (tMin.x - 0.5).floorToDouble(),
      (tMin.y - 0.5).floorToDouble(),
      (tMin.z - 0.5).floorToDouble(),
    );
    final boxMax = Vector3(
      (tMax.x + 0.5).ceilToDouble(),
      (tMax.y + 0.5).ceilToDouble(),
      (tMax.z + 0.5).ceilToDouble(),
    );
    var planes = <(Vector3, Vector3)>[];
    if (cutPlane != null && _planeCuts(body, cutPlane.$1, cutPlane.$2)) {
      planes = [cutPlane];
    } else if (!inverse && edge != null && radius != null) {
      final hyp = _leftoverPlane(body, edge, radius);
      if (hyp != null) planes = [hyp];
    }
    if (planes.isEmpty) {
      planes = _cutPlanes(body, boxMin, boxMax);
      if (planes.isEmpty) {
        IjoLog.line('mesh-fillet local no-plane');
        return null;
      }
      if (planes.length > 4) {
        IjoLog.line('mesh-fillet local skip planes=${planes.length}');
        return null;
      }
      planes.sort((a, b) {
        final na = _nearVertCount(body, a.$1, a.$2);
        final nb = _nearVertCount(body, b.$1, b.$2);
        return na.compareTo(nb);
      });
    }
    final first = planes.first;
    IjoLog.line(
      'mesh-fillet local box=$boxMin..$boxMax planes=${planes.length} '
      'first=${first.$1} n=${first.$2}',
    );

    final parts = MeshBuild.sliceParts(body, first.$1, first.$2);
    if (parts.length < 2) {
      IjoLog.line('mesh-fillet local first-slice n=${parts.length}');
      return null;
    }
    parts.sort(
      (a, b) => MeshEdit.signedVolume(
        b,
      ).abs().compareTo(MeshEdit.signedVolume(a).abs()),
    );
    var far = parts.first;
    var near = parts.last;
    final focus = hint ?? (edge != null ? (edge.$1 + edge.$2) * 0.5 : null);
    if (focus != null) {
      double minD(MeshData m) {
        var d = 1e300;
        for (final v in m.vertices) {
          d = math.min(d, (v - focus).length);
        }
        return d;
      }

      if (minD(far) < minD(near)) {
        final swap = near;
        near = far;
        far = swap;
      }
    } else if (!_boxHits(near, boxMin, boxMax)) {
      if (!_boxHits(far, boxMin, boxMax)) {
        IjoLog.line('mesh-fillet local no-near');
        return null;
      }
      final swap = near;
      near = far;
      far = swap;
    }
    if (near.faces.length > 800) {
      IjoLog.line('mesh-fillet local near-dense ${near.faces.length}f');
      return null;
    }

    var simple = <MeshData>[near];
    for (final plane in planes.skip(1)) {
      final next = <MeshData>[];
      for (final piece in simple) {
        if (!_planeCuts(piece, plane.$1, plane.$2)) {
          next.add(piece);
          continue;
        }
        final sub = MeshBuild.sliceParts(piece, plane.$1, plane.$2);
        next.addAll(sub.length >= 2 ? sub : [piece]);
      }
      simple = next;
    }

    MeshData? chunk;
    var bestVol = 1e300;
    var bestD = 1e300;
    final mid = edge != null ? (edge.$1 + edge.$2) * 0.5 : null;
    for (final p in simple) {
      if (!_boxHits(p, boxMin, boxMax)) continue;
      final vol = MeshEdit.signedVolume(p).abs();
      if (vol <= 1e-8) continue;
      if (mid != null) {
        var d = 1e300;
        for (final v in p.vertices) {
          d = math.min(d, (v - mid).length);
        }
        if (d < bestD - 1e-6 || (d <= bestD + 1e-6 && vol < bestVol)) {
          bestD = d;
          bestVol = vol;
          chunk = p;
        }
      } else if (vol < bestVol) {
        bestVol = vol;
        chunk = p;
      }
    }
    if (chunk == null) {
      IjoLog.line('mesh-fillet local no-chunk simple=${simple.length}');
      return null;
    }

    MeshData? clipped = _boolean(chunk, tool, op);
    if (clipped == null && !chamfer && edge != null && radius != null) {
      clipped = _clipRebuilt(chunk, edge, radius, inverse, op);
    }
    if (clipped == null) {
      IjoLog.line('mesh-fillet local tool-open');
      return null;
    }

    var g1 = clipped;
    for (final p in simple) {
      if (identical(p, chunk)) continue;
      final joined = _boolean(g1, p, MeshBoolean.unionOp);
      if (joined == null) {
        IjoLog.line('mesh-fillet local simple-union-open');
        return null;
      }
      g1 = joined;
    }

    IjoLog.line(
      'mesh-fillet local g1='
      '${IjoLog.size(g1.vertices.length, g1.faces.length)} '
      'vol=${MeshEdit.signedVolume(g1).abs().toStringAsFixed(1)} '
      'far=${IjoLog.size(far.vertices.length, far.faces.length)}',
    );

    var glued = _weldPieces([
      _stripPlanes(g1, [first]),
      _stripPlanes(far, [first]),
    ]);
    if (glued == null) {
      IjoLog.line(
        'mesh-fillet local glue-open g1='
        '${MeshSeal.openBoundaryEdgeCount(_stripPlanes(g1, [first]))} far='
        '${MeshSeal.openBoundaryEdgeCount(_stripPlanes(far, [first]))}',
      );
    }
    return glued;
  }

  static MeshData? _clipRebuilt(
    MeshData chunk,
    (Vector3, Vector3) edge,
    double radius,
    bool inverse,
    int op,
  ) {
    final chains = _chains(chunk, [edge]);
    IjoLog.line('mesh-fillet local rebuild chains=${chains.length}');
    for (final chain in chains) {
      final localTool = _filletTool(
        chunk,
        chain,
        radius,
        inverse: inverse,
        tessellation: MeshTessellation.standard,
        trimStart: false,
        trimEnd: false,
      );
      if (localTool == null) continue;
      final hit = _boolean(chunk, localTool, op);
      if (hit != null) return hit;
    }
    return null;
  }

  static int _nearVertCount(MeshData mesh, Vector3 origin, Vector3 n) {
    var pos = 0;
    var neg = 0;
    for (final v in mesh.vertices) {
      final h = (v - origin).dot(n);
      if (h > 1e-3) pos++;
      if (h < -1e-3) neg++;
    }
    return math.min(pos, neg);
  }

  static bool _boxHits(MeshData mesh, Vector3 min, Vector3 max) {
    const inset = 0.2;
    for (final v in mesh.vertices) {
      if (v.x > min.x + inset &&
          v.x < max.x - inset &&
          v.y > min.y + inset &&
          v.y < max.y - inset &&
          v.z > min.z + inset &&
          v.z < max.z - inset) {
        return true;
      }
    }
    return false;
  }

  static (Vector3, Vector3) _bounds(MeshData mesh) {
    final min = mesh.vertices.first.clone();
    final max = mesh.vertices.first.clone();
    for (final v in mesh.vertices) {
      min.x = math.min(min.x, v.x);
      min.y = math.min(min.y, v.y);
      min.z = math.min(min.z, v.z);
      max.x = math.max(max.x, v.x);
      max.y = math.max(max.y, v.y);
      max.z = math.max(max.z, v.z);
    }
    return (min, max);
  }

  /// Plane through the leftover's open face (the two tangency offsets).
  /// For a 90° cube edge this is the hypotenuse of the leftover prism, which
  /// peels a simple slab off a CSG-shattered wall without taking the sphere.
  static (Vector3, Vector3)? _leftoverPlane(
    MeshData mesh,
    (Vector3, Vector3) edge,
    double r,
  ) {
    final hit = _matchFeature(mesh, edge.$1, edge.$2);
    if (hit == null) return null;
    final geo = _crease(mesh, hit.$1, hit.$2);
    if (geo == null) return null;
    final (_, _, _, n0, n1) = geo;
    bool axisAligned(Vector3 n) {
      final m = math.max(n.x.abs(), math.max(n.y.abs(), n.z.abs()));
      return m > 0.98;
    }

    if (!axisAligned(n0) || !axisAligned(n1)) return null;
    var n = n0 + n1;
    if (n.length < 1e-9) return null;
    n.normalize();
    final phi = math.acos(n0.dot(n1).clamp(-1.0, 1.0));
    final alpha = math.pi - phi;
    final setback = alpha.abs() < 1e-6
        ? r
        : r / math.tan((alpha / 2).clamp(1e-3, math.pi / 2 - 1e-3));
    final depth = math.max(r * 1.25, setback * 1.05);
    final mid = (edge.$1 + edge.$2) * 0.5;
    final origin = (mid - n0 * depth + mid - n1 * depth) * 0.5;
    if ((mid - origin).dot(n) < 0) n = -n;
    if (!_planeCuts(mesh, origin, n)) return null;
    return (origin, n);
  }

  static bool _shatteredCrease(MeshData mesh, Vector3 a, Vector3 b) {
    return _shatteredScore(mesh, a, b) > 0;
  }

  static int _shatteredScore(MeshData mesh, Vector3 a, Vector3 b) {
    final hit = _matchFeature(mesh, a, b);
    if (hit == null) return 0;
    final adj = MeshFaceRegions.facesAlongCrease(mesh, hit.$1, hit.$2);
    var n = 0;
    for (final fi in adj) {
      if (MeshFaceRegions.growPlanarRegion(mesh, fi).length > 4 ||
          MeshFaceRegions.selectionRegion(mesh, fi).length > 4) {
        n++;
      }
    }
    return n;
  }

  static List<(Vector3, Vector3)> _cutPlanes(
    MeshData body,
    Vector3 boxMin,
    Vector3 boxMax,
  ) {
    final dx = boxMax.x - boxMin.x;
    final dy = boxMax.y - boxMin.y;
    final dz = boxMax.z - boxMin.z;
    final longest = math.max(dx, math.max(dy, dz));
    final shortest = math.min(dx, math.min(dy, dz));
    final elongated = longest > shortest * 2.5;
    final planes = <(Vector3, Vector3)>[];

    void addAxis(
      double extent,
      Vector3 lo,
      Vector3 hi,
      Vector3 nLo,
      Vector3 nHi,
    ) {
      if (elongated && extent > longest * 0.6) return;
      if (_planeCuts(body, lo, nLo)) planes.add((lo, nLo));
      if (_planeCuts(body, hi, nHi)) planes.add((hi, nHi));
    }

    addAxis(
      dx,
      Vector3(boxMin.x, 0, 0),
      Vector3(boxMax.x, 0, 0),
      Vector3(-1, 0, 0),
      Vector3(1, 0, 0),
    );
    addAxis(
      dy,
      Vector3(0, boxMin.y, 0),
      Vector3(0, boxMax.y, 0),
      Vector3(0, -1, 0),
      Vector3(0, 1, 0),
    );
    addAxis(
      dz,
      Vector3(0, 0, boxMin.z),
      Vector3(0, 0, boxMax.z),
      Vector3(0, 0, -1),
      Vector3(0, 0, 1),
    );
    return planes;
  }

  static bool _planeCuts(MeshData mesh, Vector3 origin, Vector3 n) {
    var pos = false;
    var neg = false;
    for (final v in mesh.vertices) {
      final h = (v - origin).dot(n);
      if (h > 1e-3) pos = true;
      if (h < -1e-3) neg = true;
      if (pos && neg) return true;
    }
    return false;
  }

  static MeshData _stripPlanes(
    MeshData mesh,
    List<(Vector3, Vector3)> planes, {
    double eps = 0.08,
  }) {
    final faces = <List<int>>[];
    for (final f in mesh.faces) {
      var drop = false;
      for (final (o, n) in planes) {
        final nn = n.normalized();
        if (f.every((i) => (mesh.vertices[i] - o).dot(nn).abs() < eps)) {
          drop = true;
          break;
        }
      }
      if (!drop) faces.add(List<int>.from(f));
    }
    return MeshData(
      vertices: [for (final v in mesh.vertices) v.clone()],
      faces: faces,
    );
  }

  static MeshData? _weldPieces(List<MeshData> pieces) {
    final nonempty = [
      for (final p in pieces)
        if (p.faces.isNotEmpty) p,
    ];
    if (nonempty.isEmpty) return null;
    if (nonempty.length == 1) {
      return MeshSeal.isClosedSolid(nonempty.first) ? nonempty.first : null;
    }
    var out = MeshEdit.combine(nonempty);
    out = MeshEdit.weld(out, epsilon: 1e-4);
    if (MeshSeal.isClosedSolid(out)) return out;
    out = MeshTopology.splitOpenEdgeTJunctions(out, epsilon: 1e-3);
    out = MeshEdit.weld(out, epsilon: 1e-5);
    if (MeshSeal.isClosedSolid(out)) return out;
    out = MeshSeal.stitchTJunctions(out);
    return MeshSeal.isClosedSolid(out) ? out : null;
  }

  static MeshData? _close(MeshData m) {
    if (m.faces.isEmpty) return null;
    if (MeshSeal.isClosedSolid(m)) return m;
    var s = MeshTopology.splitOpenEdgeTJunctions(m, epsilon: 1e-3);
    s = MeshEdit.weld(s, epsilon: 1e-5);
    if (MeshSeal.isClosedSolid(s)) return s;
    s = MeshTopology.splitOpenEdgeTJunctions(s, epsilon: 1e-2);
    s = MeshEdit.weld(s, epsilon: 1e-5);
    if (MeshSeal.isClosedSolid(s)) return s;
    s = MeshSeal.stitchTJunctions(s);
    if (!MeshSeal.isClosedSolid(s)) return null;
    return s;
  }

  static MeshData _solid(MeshData mesh) {
    if (mesh.faces.isEmpty) return mesh;
    if (MeshEdit.signedVolume(mesh) < 0) return MeshEdit.invert(mesh);
    return mesh;
  }

  static MeshData _finish(MeshData mesh, String op) {
    var out = mesh;
    if (!MeshSeal.isClosedSolid(out)) {
      out = MeshSeal.stitchTJunctions(out);
    }
    if (out.faces.isEmpty || !MeshSeal.isClosedSolid(out)) {
      throw MeshOpException('$op produced a broken solid');
    }
    return _solid(out);
  }

  static (Vector3, Vector3, Vector3, Vector3, Vector3)? _crease(
    MeshData mesh,
    int ia,
    int ib,
  ) {
    final a = mesh.vertices[ia];
    final b = mesh.vertices[ib];
    final adj = MeshFaceRegions.facesForEdge(mesh, ia, ib);
    Vector3? n0;
    Vector3? n1;
    if (adj.length >= 2) {
      n0 = _triOutward(mesh, adj[0]);
      n1 = _triOutward(mesh, adj[1]);
    }
    if (n0 == null || n1 == null || n0.dot(n1) > 0.995) {
      final pair = _twoCreaseNormals(mesh, ia, ib);
      if (pair == null) return null;
      n0 = pair.$1;
      n1 = pair.$2;
    }
    var t = b - a;
    if (t.length < 1e-9) return null;
    t.normalize();
    return (a.clone(), b.clone(), t, n0, n1);
  }

  static bool _segmentOnRequested(
    (Vector3, Vector3) segment,
    List<(Vector3, Vector3)> requested,
  ) {
    if (requested.isEmpty) return false;
    final mid = (segment.$1 + segment.$2) * 0.5;
    for (final (a, b) in requested) {
      var dir = b - a;
      final len = dir.length;
      if (len < 1e-9) continue;
      dir = dir / len;
      final along = ((mid - a).dot(dir)).clamp(0.0, len);
      if ((a + dir * along - mid).length < 1.0) return true;
    }
    return false;
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

  static (Vector3, Vector3)? _twoCreaseNormals(MeshData mesh, int ia, int ib) {
    var t = mesh.vertices[ib] - mesh.vertices[ia];
    if (t.length < 1e-9) return null;
    t.normalize();

    (Vector3, Vector3)? fromFaces(Iterable<int> faces) {
      final clusters = <({Vector3 n, double area})>[];
      for (final fi in faces) {
        if (MeshFaceRegions.isSliverFace(mesh, fi)) continue;
        final f = mesh.faces[fi];
        if (f.length < 3) continue;
        final cr = (mesh.vertices[f[1]] - mesh.vertices[f[0]]).cross(
          mesh.vertices[f[2]] - mesh.vertices[f[0]],
        );
        final area = cr.length * 0.5;
        if (area < 1e-10) continue;
        final n = cr.normalized();
        if (n.dot(t).abs() > 0.25) continue;
        var merged = false;
        for (var i = 0; i < clusters.length; i++) {
          if (clusters[i].n.dot(n) > 0.995) {
            clusters[i] = (n: clusters[i].n, area: clusters[i].area + area);
            merged = true;
            break;
          }
        }
        if (!merged) clusters.add((n: n, area: area));
      }
      if (clusters.length < 2) return null;
      clusters.sort((a, b) => b.area.compareTo(a.area));
      return (clusters[0].n, clusters[1].n);
    }

    final direct = MeshFaceRegions.facesForEdge(mesh, ia, ib);
    final pair = fromFaces(direct);
    if (pair != null) return pair;
    return fromFaces(MeshFaceRegions.facesAlongCrease(mesh, ia, ib));
  }

  static Vector3? _triOutward(MeshData mesh, int fi) {
    final f = mesh.faces[fi];
    if (f.length < 3) return null;
    final n = (mesh.vertices[f[1]] - mesh.vertices[f[0]]).cross(
      mesh.vertices[f[2]] - mesh.vertices[f[0]],
    );
    if (n.length2 < 1e-20) return null;
    return n.normalized();
  }

  static List<_CornerBlend> _tripleCorners(
    MeshData mesh,
    List<(Vector3, Vector3)> wanted,
    double r,
  ) {
    if (wanted.isEmpty || r < 1e-9) return const [];
    final features = MeshFaceRegions.featureEdges(mesh);
    if (features.isEmpty) return const [];

    String vKey(int i) => MeshFaceRegions.quantizePosition(mesh.vertices[i]);
    String eKey(int a, int b) =>
        MeshFaceRegions.edgeKey(mesh.vertices[a], mesh.vertices[b]);

    final selected = <String>{
      for (final (a, b) in wanted) MeshFaceRegions.edgeKey(a, b),
    };

    final atVertex = <String, List<(int, int)>>{};
    for (final e in features) {
      final key = eKey(e.$1, e.$2);
      if (!selected.contains(key)) continue;
      (atVertex[vKey(e.$1)] ??= []).add(e);
      (atVertex[vKey(e.$2)] ??= []).add(e);
    }

    final out = <_CornerBlend>[];
    final seen = <String>{};
    for (final entry in atVertex.entries) {
      if (entry.value.length != 3) continue;
      if (!seen.add(entry.key)) continue;
      final edges = entry.value;
      var vi = edges[0].$1;
      if (vKey(vi) != entry.key) vi = edges[0].$2;
      final vAt = mesh.vertices[vi];

      final norms = <Vector3>[];
      void addNorm(Vector3 n) {
        for (final e in norms) {
          if (e.dot(n) > 0.995) return;
        }
        norms.add(n);
      }

      final along = <Vector3>[];
      for (final e in edges) {
        var got = false;
        for (final fi in MeshFaceRegions.facesForEdge(mesh, e.$1, e.$2)) {
          final n = _triOutward(mesh, fi);
          if (n == null) continue;
          addNorm(n);
          got = true;
        }
        if (!got) {
          final pair = _twoCreaseNormals(mesh, e.$1, e.$2);
          if (pair != null) {
            addNorm(pair.$1);
            addNorm(pair.$2);
          }
        }
        final pa = mesh.vertices[e.$1];
        final pb = mesh.vertices[e.$2];
        var dir = MeshFaceRegions.quantizePosition(pa) == entry.key
            ? pb - pa
            : pa - pb;
        if (dir.length < 1e-9) continue;
        along.add(dir.normalized());
      }
      if (norms.length != 3 || along.length != 3) continue;
      final offset = _solve3(norms[0], norms[1], norms[2], r);
      if (offset == null) continue;
      out.add(
        _CornerBlend(
          vertex: vAt.clone(),
          center: vAt - offset,
          along: along,
          normals: norms,
          radius: r,
          setback: r,
        ),
      );
    }
    return out;
  }

  static Vector3? _solve3(Vector3 n0, Vector3 n1, Vector3 n2, double radius) {
    final det = n0.dot(n1.cross(n2));
    if (det.abs() < 1e-10) return null;
    return (n1.cross(n2) + n2.cross(n0) + n0.cross(n1)) * (radius / det);
  }
}

enum _LoopKind { none, support, end }

class _CreaseChain {
  const _CreaseChain({
    required this.repA,
    required this.repB,
    required this.points,
  });

  final int repA;
  final int repB;
  final List<Vector3> points;
}

class _SurfBevel {
  const _SurfBevel({
    required this.t,
    required this.n0,
    required this.n1,
    required this.d0,
    required this.d1,
    required this.p0,
    required this.p1,
    required this.c0,
    required this.c1,
    required this.tan0,
    required this.tan1,
    required this.rawFirst,
    required this.rawLast,
    required this.r,
    required this.phi,
    required this.setback,
    required this.trimStart,
    required this.trimEnd,
    required this.axis,
    required this.nSeg,
  });

  final Vector3 t;
  final Vector3 n0;
  final Vector3 n1;
  final Vector3 d0;
  final Vector3 d1;
  final Vector3 p0;
  final Vector3 p1;
  final Vector3 c0;
  final Vector3 c1;
  final Vector3 tan0;
  final Vector3 tan1;
  final Vector3 rawFirst;
  final Vector3 rawLast;
  final double r;
  final double phi;
  final double setback;
  final bool trimStart;
  final bool trimEnd;
  final Vector3 axis;
  final int nSeg;
}

class _CornerBlend {
  const _CornerBlend({
    required this.vertex,
    required this.center,
    required this.along,
    required this.normals,
    required this.radius,
    required this.setback,
  });

  final Vector3 vertex;
  final Vector3 center;
  final List<Vector3> along;
  final List<Vector3> normals;
  final double radius;
  final double setback;
}
