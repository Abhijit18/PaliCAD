import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import 'mesh_data.dart';
import '../ijo_log.dart';
import 'mesh_edit.dart';
import 'mesh_regions.dart';
import 'mesh_polygon.dart';

/// T-junction splits and connected-component walks.
class MeshTopology {
  MeshTopology._();

  /// Split triangles that have other vertices lying on their edges (T-junctions).
  ///
  /// BSP CSG emits these constantly; without the split, edge-adjacency walks
  /// treat one solid as several shells.
  static MeshData splitTJunctions(MeshData mesh, {double epsilon = 1e-4}) {
    // Do not weld first: welding would merge distinct solids that only kiss
    // on a cut plane (slice). T-junction vertices are still found by position.
    final tri = MeshEdit.triangulate(mesh);
    if (tri.faces.isEmpty) return tri;
    final verts = tri.vertices;
    final eps2 = epsilon * epsilon;
    final cell = math.max(epsilon * 50, 0.05);
    final inv = 1.0 / cell;
    int bin(double x) => (x * inv).floor();
    int hash(int ix, int iy, int iz) => Object.hash(ix, iy, iz);
    final grid = <int, List<int>>{};
    for (var vi = 0; vi < verts.length; vi++) {
      final v = verts[vi];
      (grid[hash(bin(v.x), bin(v.y), bin(v.z))] ??= []).add(vi);
    }

    var longEdgeWalks = 0;
    void collectCell(int ix, int iy, int iz, Set<int> seen, List<int> out) {
      final list = grid[hash(ix, iy, iz)];
      if (list == null) return;
      for (final i in list) {
        if (seen.add(i)) out.add(i);
      }
    }

    List<int> walkEdgeBins(Vector3 a, Vector3 b) {
      final out = <int>[];
      final seen = <int>{};
      void halo(int ix, int iy, int iz) {
        for (var ox = -1; ox <= 1; ox++) {
          for (var oy = -1; oy <= 1; oy++) {
            for (var oz = -1; oz <= 1; oz++) {
              collectCell(ix + ox, iy + oy, iz + oz, seen, out);
            }
          }
        }
      }

      final dx = b.x - a.x;
      final dy = b.y - a.y;
      final dz = b.z - a.z;
      final len = math.sqrt(dx * dx + dy * dy + dz * dz);
      if (len < 1e-18) {
        halo(bin(a.x), bin(a.y), bin(a.z));
        return out;
      }
      var ix = bin(a.x);
      var iy = bin(a.y);
      var iz = bin(a.z);
      final ix1 = bin(b.x);
      final iy1 = bin(b.y);
      final iz1 = bin(b.z);
      final stepX = dx >= 0 ? 1 : -1;
      final stepY = dy >= 0 ? 1 : -1;
      final stepZ = dz >= 0 ? 1 : -1;
      double tMax(double origin, double dir, int i, int step) {
        if (dir.abs() < 1e-18) return double.infinity;
        final next = (step > 0 ? (i + 1) : i) * cell;
        return (next - origin) / dir;
      }

      var tMaxX = tMax(a.x, dx, ix, stepX);
      var tMaxY = tMax(a.y, dy, iy, stepY);
      var tMaxZ = tMax(a.z, dz, iz, stepZ);
      final tDeltaX = dx.abs() < 1e-18 ? double.infinity : cell / dx.abs();
      final tDeltaY = dy.abs() < 1e-18 ? double.infinity : cell / dy.abs();
      final tDeltaZ = dz.abs() < 1e-18 ? double.infinity : cell / dz.abs();

      halo(ix, iy, iz);
      var guard = 0;
      while ((ix != ix1 || iy != iy1 || iz != iz1) && guard++ < 100000) {
        final tNext = tMaxX <= tMaxY
            ? (tMaxX <= tMaxZ ? tMaxX : tMaxZ)
            : (tMaxY <= tMaxZ ? tMaxY : tMaxZ);
        if (tNext > 1.0 + 1e-12) break;
        if (tMaxX <= tMaxY && tMaxX <= tMaxZ) {
          ix += stepX;
          tMaxX += tDeltaX;
        } else if (tMaxY <= tMaxZ) {
          iy += stepY;
          tMaxY += tDeltaY;
        } else {
          iz += stepZ;
          tMaxZ += tDeltaZ;
        }
        halo(ix, iy, iz);
      }
      halo(ix1, iy1, iz1);
      return out;
    }

    List<int> candidates(Vector3 a, Vector3 b) {
      final minX = bin(math.min(a.x, b.x) - epsilon);
      final maxX = bin(math.max(a.x, b.x) + epsilon);
      final minY = bin(math.min(a.y, b.y) - epsilon);
      final maxY = bin(math.max(a.y, b.y) + epsilon);
      final minZ = bin(math.min(a.z, b.z) - epsilon);
      final maxZ = bin(math.max(a.z, b.z) + epsilon);
      final cells = (maxX - minX + 1) * (maxY - minY + 1) * (maxZ - minZ + 1);
      // A sparse architectural model can have only a few dozen vertices but
      // millions of empty 0.05 mm bins along its edges. Scan its vertices when
      // that costs less than either grid traversal. This also avoids the DDA
      // walk's guard truncating very long edges before reaching a T-junction.
      final walkCells =
          (bin(b.x) - bin(a.x)).abs() +
          (bin(b.y) - bin(a.y)).abs() +
          (bin(b.z) - bin(a.z)).abs() +
          1;
      if (math.min(cells, walkCells * 27) > verts.length ||
          walkCells > 100000) {
        return [
          for (var i = 0; i < verts.length; i++)
            if (verts[i].x >= math.min(a.x, b.x) - epsilon &&
                verts[i].x <= math.max(a.x, b.x) + epsilon &&
                verts[i].y >= math.min(a.y, b.y) - epsilon &&
                verts[i].y <= math.max(a.y, b.y) + epsilon &&
                verts[i].z >= math.min(a.z, b.z) - epsilon &&
                verts[i].z <= math.max(a.z, b.z) + epsilon)
              i,
        ];
      }
      if (cells > 2048) {
        longEdgeWalks++;
        if (longEdgeWalks == 1 &&
            (verts.length > 256 || tri.faces.length > 1000)) {
          IjoLog.trace(
            'splitT long-edge walk verts=${verts.length} '
            'faces=${tri.faces.length}',
          );
        }
        return walkEdgeBins(a, b);
      }
      final out = <int>[];
      final seen = <int>{};
      for (var ix = minX; ix <= maxX; ix++) {
        for (var iy = minY; iy <= maxY; iy++) {
          for (var iz = minZ; iz <= maxZ; iz++) {
            collectCell(ix, iy, iz, seen, out);
          }
        }
      }
      return out;
    }

    final srcIds = tri.faceIds;
    final keepIds = srcIds != null && srcIds.length == tri.faces.length;
    final faces = <List<int>>[];
    final outIds = keepIds ? <int>[] : null;
    for (var fi = 0; fi < tri.faces.length; fi++) {
      final face = tri.faces[fi];
      final ring = <int>[];
      for (var e = 0; e < 3; e++) {
        final i0 = face[e];
        final i1 = face[(e + 1) % 3];
        ring.add(i0);
        final a = verts[i0];
        final b = verts[i1];
        final ab = b - a;
        final len2 = ab.length2;
        if (len2 < eps2) continue;
        final mids = <({int i, double t})>[];
        for (final vi in candidates(a, b)) {
          if (vi == i0 ||
              vi == i1 ||
              vi == face[0] ||
              vi == face[1] ||
              vi == face[2]) {
            continue;
          }
          final ap = verts[vi] - a;
          final t = ap.dot(ab) / len2;
          if (t <= epsilon / math.sqrt(len2) ||
              t >= 1 - epsilon / math.sqrt(len2))
            continue;
          final proj = a + ab * t;
          if ((verts[vi] - proj).length2 > eps2) continue;
          mids.add((i: vi, t: t));
        }
        mids.sort((x, y) => x.t.compareTo(y.t));
        var last = i0;
        for (final m in mids) {
          if (m.i == last) continue;
          ring.add(m.i);
          last = m.i;
        }
      }
      if (ring.length < 3) continue;
      // A fan from a boundary vertex emits zero-area triangles when that
      // vertex's own edges were split. Ear clipping retains every shared
      // boundary vertex without creating those degenerate bridge triangles.
      final normal = (verts[face[1]] - verts[face[0]]).cross(
        verts[face[2]] - verts[face[0]],
      );
      for (final triangle in MeshPolygon.triangulateRings(
        verts,
        [ring],
        normal,
        keepCollinear: true,
      )) {
        faces.add(triangle);
        outIds?.add(srcIds![fi]);
      }
    }
    if (longEdgeWalks > 0 && (verts.length > 256 || tri.faces.length > 1000)) {
      IjoLog.trace(
        'splitT long-edge walks=$longEdgeWalks verts=${verts.length} '
        'faces=${tri.faces.length}',
      );
    }
    return MeshData(
      vertices: [for (final v in verts) v.clone()],
      faces: faces,
      faceIds: outIds,
    );
  }

  /// Fan only 1-use edges that have another vertex on them.
  ///
  /// Unlike [splitTJunctions] this does not walk every long interior wall
  /// (that stacked overlapping slivers on a 132-gon C minus a through
  /// cylinder, then CSG rejected the result as open). A leftover collinear
  /// hole after clip or boolean is usually a 1-use T-junction.
  static MeshData splitOpenEdgeTJunctions(
    MeshData mesh, {
    double epsilon = 1e-3,
  }) {
    var m = MeshEdit.triangulate(mesh);
    for (var pass = 0; pass < 4; pass++) {
      final next = _splitOpenEdgeTJunctionsOnce(m, eps: epsilon);
      if (identical(next, m) || next.faces.length == m.faces.length) {
        return next;
      }
      m = next;
    }
    return m;
  }

  static MeshData _splitOpenEdgeTJunctionsOnce(
    MeshData mesh, {
    required double eps,
  }) {
    final tri = mesh;
    if (tri.faces.isEmpty) return tri;
    final verts = tri.vertices;
    final uses = <String, int>{};
    for (final face in tri.faces) {
      if (face.length < 3) continue;
      for (var i = 0; i < 3; i++) {
        final a = face[i];
        final b = face[(i + 1) % 3];
        final key = MeshFaceRegions.edgeKey(verts[a], verts[b]);
        uses[key] = (uses[key] ?? 0) + 1;
      }
    }
    final openKeys = <String>{
      for (final e in uses.entries)
        if (e.value == 1) e.key,
    };
    if (openKeys.isEmpty) return tri;

    final eps2 = eps * eps;
    List<({int i, double t})> midsOn(int i0, int i1) {
      final a = verts[i0];
      final b = verts[i1];
      final ab = b - a;
      final len2 = ab.length2;
      if (len2 < eps2) return const [];
      final tol2 = math.max(eps2, len2 * 1e-16);
      final mids = <({int i, double t})>[];
      for (var vi = 0; vi < verts.length; vi++) {
        if (vi == i0 || vi == i1) continue;
        final ap = verts[vi] - a;
        final t = ap.dot(ab) / len2;
        if (t <= eps / math.sqrt(len2) || t >= 1 - eps / math.sqrt(len2))
          continue;
        if ((a + ab * t - verts[vi]).length2 > tol2) continue;
        mids.add((i: vi, t: t));
      }
      mids.sort((x, y) => x.t.compareTo(y.t));
      return mids;
    }

    final srcIds = tri.faceIds;
    final keepIds = srcIds != null && srcIds.length == tri.faces.length;
    final faces = <List<int>>[];
    final outIds = keepIds ? <int>[] : null;

    void emit(int a, int b, int c, int fi) {
      if (a == b || b == c || a == c) return;
      final cr = (verts[b] - verts[a]).cross(verts[c] - verts[a]);
      if (cr.length2 < 1e-20) return;
      faces.add([a, b, c]);
      outIds?.add(srcIds![fi]);
    }

    var fans = 0;
    for (var fi = 0; fi < tri.faces.length; fi++) {
      final face = tri.faces[fi];
      if (face.length < 3) continue;
      final i0 = face[0];
      final i1 = face[1];
      final i2 = face[2];
      final idx = [i0, i1, i2];
      var splitCount = 0;
      var apex = i0;
      var a = i1;
      var b = i2;
      var mids = <({int i, double t})>[];
      for (var e = 0; e < 3; e++) {
        final ea = idx[e];
        final eb = idx[(e + 1) % 3];
        final key = MeshFaceRegions.edgeKey(verts[ea], verts[eb]);
        if (!openKeys.contains(key)) continue;
        final found = midsOn(ea, eb);
        if (found.isEmpty) continue;
        splitCount++;
        a = ea;
        b = eb;
        apex = idx[(e + 2) % 3];
        mids = found;
      }
      if (splitCount == 1) {
        fans++;
        var prev = a;
        for (final m in mids) {
          emit(apex, prev, m.i, fi);
          prev = m.i;
        }
        emit(apex, prev, b, fi);
        continue;
      }
      emit(i0, i1, i2, fi);
    }
    if (fans == 0) return tri;
    IjoLog.trace('splitT open-edge fans=$fans');
    return MeshData(
      vertices: [for (final v in verts) v.clone()],
      faces: faces,
      faceIds: outIds,
    );
  }

  /// Connected components, with one CAD extra for zero-thickness walls.
  ///
  /// Uniting every halfedge endpoint keeps a cube∪sphere as one
  /// body even when CSG leaves 4-use edges on the cut face. A face walk
  /// that only follows valence-2 edges treats those overlays as cuts and
  /// shatters the union into hundreds of scraps.
  ///
  /// Walk:
  /// - valence 2: join (manifold)
  /// - valence ≥ 3, all incident triangles coplanar: join (CSG overlay)
  /// - otherwise do not join (inscribed-cylinder knife: 4 faces, 2 planes)
  static List<MeshData> decompose(MeshData mesh) {
    // Tangent walls can use one long edge on one side and two short edges
    // on the other. Match their subdivisions before counting incident faces.
    final tri = splitTJunctions(mesh, epsilon: 1e-7);
    if (tri.faces.isEmpty) return const [];
    final groups = _overlayEdgeGroups(tri);
    return [
      for (final faces in groups)
        if (faces.isNotEmpty) _extractComponent(tri, faces),
    ].where((p) => MeshEdit.signedVolume(p).abs() > 1e-8).toList();
  }

  /// Connected triangle components.
  ///
  /// Closed meshes walk manifold edges (exactly two triangles) so a
  /// through-cut that only kisses at a knife-edge — four faces around a
  /// zero-thickness wall — splits into separate solids. Vertex-star walks
  /// would keep those pieces glued.
  ///
  /// Open meshes (hollow openings, leftover T-junctions) keep a vertex-star
  /// walk after T-junction splits; a manifold-edge walk would shatter them
  /// along every valence-1 rim.
  ///
  /// After mesh CSG, use [decompose]. This knife-edge
  /// walk is for an explicit split of zero-thickness walls.
  static List<MeshData> connectedComponents(MeshData mesh) {
    var tri = MeshEdit.triangulate(mesh);
    if (tri.faces.isEmpty) return const [];
    final open = MeshFaceRegions.openBoundaryEdgeCount(tri) > 0;
    if (open) tri = splitTJunctions(tri);
    final groups = open ? _vertexStarGroups(tri) : _manifoldEdgeGroups(tri);
    return [
      for (final faces in groups)
        if (faces.isNotEmpty) _extractComponent(tri, faces),
    ].where((p) => MeshEdit.signedVolume(p).abs() > 1e-8).toList();
  }

  static List<List<int>> _manifoldEdgeGroups(MeshData tri) {
    final edgeFaces = <String, List<int>>{};
    for (var f = 0; f < tri.faces.length; f++) {
      final face = tri.faces[f];
      for (var i = 0; i < face.length; i++) {
        final key = MeshFaceRegions.edgeKey(
          tri.vertices[face[i]],
          tri.vertices[face[(i + 1) % face.length]],
        );
        (edgeFaces[key] ??= []).add(f);
      }
    }
    final seen = List<bool>.filled(tri.faces.length, false);
    final groups = <List<int>>[];
    for (var seed = 0; seed < tri.faces.length; seed++) {
      if (seen[seed]) continue;
      final stack = [seed];
      seen[seed] = true;
      final faces = <int>[];
      while (stack.isNotEmpty) {
        final f = stack.removeLast();
        faces.add(f);
        final face = tri.faces[f];
        for (var i = 0; i < face.length; i++) {
          final key = MeshFaceRegions.edgeKey(
            tri.vertices[face[i]],
            tri.vertices[face[(i + 1) % face.length]],
          );
          final nbrs = edgeFaces[key];
          // Non-manifold (pinch / knife) edges do not join solids.
          if (nbrs == null || nbrs.length != 2) continue;
          for (final n in nbrs) {
            if (seen[n]) continue;
            seen[n] = true;
            stack.add(n);
          }
        }
      }
      groups.add(faces);
    }
    return groups;
  }

  static List<List<int>> _overlayEdgeGroups(MeshData tri) {
    final edgeFaces = <String, List<int>>{};
    for (var f = 0; f < tri.faces.length; f++) {
      final face = tri.faces[f];
      for (var i = 0; i < face.length; i++) {
        final key = MeshFaceRegions.edgeKey(
          tri.vertices[face[i]],
          tri.vertices[face[(i + 1) % face.length]],
        );
        (edgeFaces[key] ??= []).add(f);
      }
    }

    bool joinable(List<int> nbrs) {
      if (nbrs.length == 2) return true;
      if (nbrs.length < 2) return false;
      return _incidentFacesCoplanar(tri, nbrs);
    }

    final seen = List<bool>.filled(tri.faces.length, false);
    final groups = <List<int>>[];
    for (var seed = 0; seed < tri.faces.length; seed++) {
      if (seen[seed]) continue;
      final stack = [seed];
      seen[seed] = true;
      final faces = <int>[];
      while (stack.isNotEmpty) {
        final f = stack.removeLast();
        faces.add(f);
        final face = tri.faces[f];
        for (var i = 0; i < face.length; i++) {
          final key = MeshFaceRegions.edgeKey(
            tri.vertices[face[i]],
            tri.vertices[face[(i + 1) % face.length]],
          );
          final nbrs = edgeFaces[key];
          if (nbrs == null || !joinable(nbrs)) continue;
          for (final n in nbrs) {
            if (seen[n]) continue;
            seen[n] = true;
            stack.add(n);
          }
        }
      }
      groups.add(faces);
    }
    return groups;
  }

  static bool _incidentFacesCoplanar(MeshData tri, List<int> faces) {
    Vector3? n0;
    Vector3? p0;
    for (final f in faces) {
      final face = tri.faces[f];
      if (face.length < 3) continue;
      final n = (tri.vertices[face[1]] - tri.vertices[face[0]]).cross(
        tri.vertices[face[2]] - tri.vertices[face[0]],
      );
      if (n.length2 < 1e-20) continue;
      n.normalize();
      n0 ??= n;
      p0 ??= tri.vertices[face[0]];
      if (n.dot(n0).abs() < MeshFaceRegions.smoothCreaseMinDot) {
        return false;
      }
      if ((tri.vertices[face[0]] - p0).dot(n0).abs() >=
          MeshFaceRegions.planarGrowOffsetEpsilon) {
        return false;
      }
    }
    return n0 != null;
  }

  static List<List<int>> _vertexStarGroups(MeshData tri) {
    final vertFaces = <int, List<int>>{};
    for (var f = 0; f < tri.faces.length; f++) {
      for (final v in tri.faces[f]) {
        (vertFaces[v] ??= []).add(f);
      }
    }
    final seen = List<bool>.filled(tri.faces.length, false);
    final groups = <List<int>>[];
    for (var seed = 0; seed < tri.faces.length; seed++) {
      if (seen[seed]) continue;
      final stack = [seed];
      seen[seed] = true;
      final faces = <int>[];
      while (stack.isNotEmpty) {
        final f = stack.removeLast();
        faces.add(f);
        for (final v in tri.faces[f]) {
          final nbrs = vertFaces[v];
          if (nbrs == null) continue;
          for (final n in nbrs) {
            if (seen[n]) continue;
            seen[n] = true;
            stack.add(n);
          }
        }
      }
      groups.add(faces);
    }
    return groups;
  }

  static MeshData _extractComponent(MeshData tri, List<int> faces) {
    final used = <int>{};
    for (final f in faces) {
      used.addAll(tri.faces[f]);
    }
    final remap = <int, int>{};
    final verts = <Vector3>[];
    for (final i in used) {
      remap[i] = verts.length;
      verts.add(tri.vertices[i].clone());
    }
    return MeshData(
      vertices: verts,
      faces: [
        for (final f in faces) [for (final i in tri.faces[f]) remap[i]!],
      ],
      faceIds: tri.faceIds != null && tri.faceIds!.length == tri.faces.length
          ? [for (final f in faces) tri.faceIds![f]]
          : null,
    );
  }
}
