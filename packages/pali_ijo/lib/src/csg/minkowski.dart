// Minkowski sum / difference: convex-convex is the hull of vertex sums; mixed
// convexity unions per-triangle hulls; both non-convex unions face-pair hulls.

import 'package:vector_math/vector_math_64.dart';

import '../mesh/mesh_data.dart';
import '../mesh/mesh_edit.dart';
import '../mesh/mesh_hull.dart';
import 'boolean.dart';

/// Minkowski sum (dilate) and difference (erode).
///
/// A ⊕ B is every point of A plus every point of B. A ⊖ B is the inverse
/// erosion: A minus the boundary of A swept by B. A sphere (or cube) kernel
/// containing the origin is the usual round offset.
class MeshMinkowski {
  MeshMinkowski._();

  static const _batch = 32;

  static MeshData sum(MeshData a, MeshData b) => _run(a, b, inset: false);

  static MeshData difference(MeshData a, MeshData b) =>
      _run(a, b, inset: true);

  static MeshData _run(MeshData aIn, MeshData bIn, {required bool inset}) {
    var a = MeshEdit.triangulate(aIn);
    var b = MeshEdit.triangulate(bIn);
    if (b.faces.isEmpty || b.vertices.isEmpty) return MeshEdit.clone(a);
    if (a.faces.isEmpty || a.vertices.isEmpty) {
      return inset
          ? MeshData(vertices: const [], faces: const [])
          : MeshEdit.clone(b);
    }

    var aConvex = MeshHull.isConvex(a);
    var bConvex = MeshHull.isConvex(b);
    // Sum is commutative; keep the convex operand as B so the fast paths apply.
    if (!inset && aConvex && !bConvex) {
      final t = a;
      a = b;
      b = t;
      aConvex = false;
      bConvex = true;
    }

    // Convex ⊕ convex is the hull of every vertex pair. Returning that hull
    // is the true Minkowski sum, including when B does not contain the origin.
    if (!inset && aConvex && bConvex) {
      return MeshHull.ofPoints(_vertexSums(a.vertices, b.vertices));
    }

    final extras = <MeshData>[];
    if ((inset || !aConvex) && bConvex) {
      extras.add(_unionTriHulls(a, b.vertices));
    } else if (!aConvex && !bConvex) {
      extras.add(_unionFacePairHulls(a, b));
    }

    final grown = _unionAll(extras);
    if (grown.faces.isEmpty) return MeshEdit.clone(a);
    return MeshBoolean.run(
      a,
      grown,
      inset ? MeshBoolean.subtractOp : MeshBoolean.unionOp,
    );
  }

  static List<Vector3> _vertexSums(List<Vector3> a, List<Vector3> b) {
    final out = <Vector3>[];
    for (final p in a) {
      for (final q in b) {
        out.add(p + q);
      }
    }
    return out;
  }

  static MeshData _unionTriHulls(MeshData a, List<Vector3> bVerts) {
    final hulls = <MeshData>[];
    for (final f in a.faces) {
      if (f.length < 3) continue;
      final pts = <Vector3>[];
      for (var i = 0; i < 3; i++) {
        final p = a.vertices[f[i]];
        for (final q in bVerts) {
          pts.add(p + q);
        }
      }
      final h = MeshHull.ofPoints(pts);
      if (h.faces.isNotEmpty) hulls.add(h);
      if (hulls.length >= _batch) {
        final u = _unionAll(hulls);
        hulls
          ..clear()
          ..add(u);
      }
    }
    return _unionAll(hulls);
  }

  static MeshData _unionFacePairHulls(MeshData a, MeshData b) {
    const coplanarTol = 1e-12;
    final nA = [for (final f in a.faces) _triNormal(a, f)];
    final nB = [for (final f in b.faces) _triNormal(b, f)];
    final accumulated = <MeshData>[];
    for (var ia = 0; ia < a.faces.length; ia++) {
      final fa = a.faces[ia];
      if (fa.length < 3) continue;
      final na = nA[ia];
      final a1 = a.vertices[fa[0]];
      final a2 = a.vertices[fa[1]];
      final a3 = a.vertices[fa[2]];
      final faceHulls = <MeshData>[];
      for (var ib = 0; ib < b.faces.length; ib++) {
        final fb = b.faces[ib];
        if (fb.length < 3) continue;
        final nb = nB[ib];
        final same = na.dot(nb);
        final opp = na.dot(-nb);
        if ((same - 1.0).abs() < coplanarTol ||
            (opp - 1.0).abs() < coplanarTol) {
          continue;
        }
        final b1 = b.vertices[fb[0]];
        final b2 = b.vertices[fb[1]];
        final b3 = b.vertices[fb[2]];
        final h = MeshHull.ofPoints([
          a1 + b1,
          a1 + b2,
          a1 + b3,
          a2 + b1,
          a2 + b2,
          a2 + b3,
          a3 + b1,
          a3 + b2,
          a3 + b3,
        ]);
        if (h.faces.isNotEmpty) faceHulls.add(h);
      }
      if (faceHulls.isNotEmpty) accumulated.add(_unionAll(faceHulls));
      if (accumulated.length >= 16) {
        final u = _unionAll(accumulated);
        accumulated
          ..clear()
          ..add(u);
      }
    }
    return _unionAll(accumulated);
  }

  static Vector3 _triNormal(MeshData mesh, List<int> f) {
    final n = (mesh.vertices[f[1]] - mesh.vertices[f[0]])
        .cross(mesh.vertices[f[2]] - mesh.vertices[f[0]]);
    final len = n.length;
    return len < 1e-18 ? Vector3(0, 0, 1) : n / len;
  }

  static MeshData _unionAll(List<MeshData> meshes) {
    MeshData? acc;
    for (final m in meshes) {
      if (m.faces.isEmpty) continue;
      acc = acc == null
          ? m
          : MeshBoolean.run(acc, m, MeshBoolean.unionOp);
    }
    return acc ?? MeshData(vertices: const [], faces: const []);
  }
}
