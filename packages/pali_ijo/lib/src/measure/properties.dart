import 'dart:math' as math;
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import '../brep/brep.dart';
import '../mesh/mesh_data.dart';
import '../mesh/mesh_edit.dart';
import 'bounds.dart';
import 'face_domain.dart';
import 'quadrature.dart';

/// Mass properties of a solid.
class IjoProperties {
  const IjoProperties({
    required this.volume,
    required this.surfaceArea,
    required this.centroid,
    required this.bounds,
    required this.isExact,
  });

  /// Enclosed volume. Negative means the solid's faces are inside-out.
  final double volume;

  final double surfaceArea;

  /// Center of mass, assuming uniform density.
  final Vector3 centroid;

  final IjoBounds? bounds;

  /// True only when every B-Rep face had a closed-form domain.
  /// Always false for triangle meshes.
  final bool isExact;

  double get sphericity {
    if (surfaceArea <= 0) return 0;
    return math.pow(math.pi, 1 / 3) * math.pow(6 * volume, 2 / 3) / surfaceArea;
  }

  Map<String, dynamic> toJson() => {
    'volume': volume,
    'surfaceArea': surfaceArea,
    'centroid': [centroid.x, centroid.y, centroid.z],
    if (bounds != null) 'bounds': bounds!.toJson(),
    'exact': isExact,
  };

  @override
  String toString() =>
      'IjoProperties(volume: $volume, area: $surfaceArea, '
      'centroid: $centroid, exact: $isExact)';
}

/// Triangle-mesh mass properties.
class IjoMeasure {
  IjoMeasure._();

  static IjoProperties ofMesh(MeshData mesh) {
    final volume = MeshEdit.signedVolume(mesh);
    var area = 0.0;
    var cx = 0.0, cy = 0.0, cz = 0.0;
    for (final f in mesh.faces) {
      if (f.length < 3) continue;
      final a = mesh.vertices[f[0]];
      final b = mesh.vertices[f[1]];
      final c = mesh.vertices[f[2]];
      final cr = (b - a).cross(c - a);
      area += cr.length * 0.5;
      final tet = a.dot(b.cross(c));
      cx += tet * (a.x + b.x + c.x);
      cy += tet * (a.y + b.y + c.y);
      cz += tet * (a.z + b.z + c.z);
    }
    final centroid = volume.abs() < 1e-15
        ? Vector3.zero()
        : Vector3(cx, cy, cz) / (volume * 24);
    return IjoProperties(
      volume: volume.abs(),
      surfaceArea: area,
      centroid: centroid,
      bounds: meshBounds(mesh),
      isExact: false,
    );
  }

  static IjoProperties of(MeshData mesh) => ofMesh(mesh);

  static double volume(MeshData mesh) => ofMesh(mesh).volume;

  static double surfaceArea(MeshData mesh) => ofMesh(mesh).surfaceArea;

  static Vector3 centroid(MeshData mesh) => ofMesh(mesh).centroid;

  static IjoBounds? boundingBox(MeshData mesh) => meshBounds(mesh);

  /// Analytic properties of a [BrepSolid]. Prefer this over tessellating first.
  static IjoProperties ofSolid(BrepSolid solid) {
    final totals = Float64List(_terms);
    var exact = true;

    for (final face in solid.faces) {
      final FaceDomain domain;
      try {
        final cap = _circularSphereCap(solid, face) ?? _spherePatch(solid, face);
        if (cap != null) {
          if (!cap.$2.isExact) exact = false;
          final values = _integrateFace(cap.$1, cap.$2);
          for (var i = 0; i < _terms; i++) {
            totals[i] += values[i];
          }
          continue;
        }
        domain = faceDomain(solid, face);
      } on BrepUnsupported {
        // Omitting a face gives a plausible but wrong mass. If a trim domain
        // is unavailable, measure the complete display mesh and mark it inexact.
        return ofMesh(BrepTessellate.run(solid));
      }
      if (!domain.isExact) exact = false;

      final face0 = _integrateFace(face, domain);
      for (var i = 0; i < _terms; i++) {
        totals[i] += face0[i];
      }
    }

    final area = totals[0];
    final volume = totals[1] / 3;
    final centroid = volume.abs() < 1e-15
        ? Vector3.zero()
        : Vector3(totals[2], totals[3], totals[4]) / (2 * volume);

    return IjoProperties(
      volume: volume,
      surfaceArea: area,
      centroid: centroid,
      bounds: solidBounds(solid),
      isExact: exact,
    );
  }

  static (BrepFace, FaceDomainRect)? _circularSphereCap(
    BrepSolid solid,
    BrepFace face,
  ) {
    final transformed = face.surface is BrepAffineSurface
        ? BrepXform.unwrapAffine(solid)
        : null;
    final localSolid = transformed?.solid ?? solid;
    final localFace = transformed == null
        ? face
        : localSolid.faces[solid.faces.indexOf(face)];
    final sphere = localFace.surface;
    if (sphere is! BrepSphere ||
        localFace.loops.length != 1 ||
        localFace.loops.single.coedges.length != 1)
      return null;
    final co = localFace.loops.single.coedges.single;
    final circle = localSolid.edges[co.edge].curve;
    if (circle is! BrepCircle || !circle.isClosed) return null;
    final pole = sphere.circularCapPole(circle, coedgeReversed: co.reversed);
    final axis = (pole - sphere.center).normalized(), x = brepFrameU(axis);
    var map = BrepAffineTransform(x, axis.cross(x), axis, sphere.center);
    if (transformed != null) map = map.then(transformed.transform);
    final theta = math.acos(
      ((circle.center - sphere.center).dot(axis) / sphere.radius).clamp(
        -1.0,
        1.0,
      ),
    );
    return (
      BrepFace(
        surface: BrepAffineSurface(
          BrepSphere(center: Vector3.zero(), radius: sphere.radius),
          map,
        ),
        loops: [],
        reversed: face.reversed,
      ),
      FaceDomainRect(0, 2 * math.pi, 0, theta),
    );
  }

  static (BrepFace, FaceDomain)? _spherePatch(BrepSolid solid, BrepFace face) {
    final sphere = face.surface;
    if (sphere is! BrepSphere) return null;
    final frame = BrepTessellate.sphereFaceFrame(solid, face);
    if (frame == null) return null;
    // Put this finite patch around the local +X equator. Its entire boundary
    // stays away from the spherical poles and cannot wind around the longitude
    // seam, which would otherwise be mistaken for a full revolution domain.
    final map = BrepAffineTransform(
      frame.normal,
      frame.xDir,
      frame.yDir,
      sphere.center,
    );
    final patch = BrepSolid(
      vertices: solid.vertices,
      edges: solid.edges,
      faces: [face],
    );
    final local = BrepXform.map(patch, map.inversePoint);
    final domain = faceDomain(local, local.faces.single);
    return (
      BrepFace(
        surface: BrepAffineSurface(
          BrepSphere(center: Vector3.zero(), radius: sphere.radius),
          map,
        ),
        loops: [],
        reversed: face.reversed,
      ),
      domain,
    );
  }

  static double volumeSolid(BrepSolid solid) => ofSolid(solid).volume;

  static double surfaceAreaSolid(BrepSolid solid) => ofSolid(solid).surfaceArea;

  static Vector3 centroidSolid(BrepSolid solid) => ofSolid(solid).centroid;

  static IjoBounds? boundingBoxSolid(BrepSolid solid) => solidBounds(solid);

  static double faceArea(BrepSolid solid, int faceIndex) {
    final face = solid.faces[faceIndex];
    final cap = _circularSphereCap(solid, face) ?? _spherePatch(solid, face);
    return cap == null
        ? _integrateFace(face, faceDomain(solid, face))[0]
        : _integrateFace(cap.$1, cap.$2)[0];
  }

  static const _terms = 5;

  static Float64List _integrateFace(BrepFace face, FaceDomain domain) {
    final surface = face.surface;

    void integrand(double u, double v, Float64List out) {
      final j = _jacobian(surface, u, v);
      if (j == 0) {
        out.fillRange(0, _terms, 0);
        return;
      }
      final p = surface.evaluate(u, v);
      final n = face.outwardNormal(u, v);
      out[0] = j;
      out[1] = p.dot(n) * j;
      out[2] = p.x * p.x * n.x * j;
      out[3] = p.y * p.y * n.y * j;
      out[4] = p.z * p.z * n.z * j;
    }

    if (surface is BrepSplineSurface && domain is FaceDomainRect) {
      final us =
          surface.knotsU
              .toSet()
              .where((u) => u >= surface.u0 && u <= surface.u1)
              .toList()
            ..sort();
      final vs =
          surface.knotsV
              .toSet()
              .where((v) => v >= surface.v0 && v <= surface.v1)
              .toList()
            ..sort();
      final total = Float64List(_terms);
      for (var i = 1; i < us.length; i++) {
        for (var j = 1; j < vs.length; j++) {
          final part = _overRect(
            FaceDomainRect(us[i - 1], us[i], vs[j - 1], vs[j]),
            integrand,
          );
          for (var k = 0; k < _terms; k++) total[k] += part[k];
        }
      }
      return total;
    }
    final raw = switch (domain) {
      FaceDomainRect() => _overRect(domain, integrand),
      FaceDomainLoops() => _overLoops(domain, integrand),
    };

    if (raw[0] < 0) {
      for (var i = 0; i < _terms; i++) {
        raw[i] = -raw[i];
      }
    }
    return raw;
  }

  static Float64List _overRect(
    FaceDomainRect r,
    void Function(double, double, Float64List) f,
  ) {
    final gl = GaussLegendre.standard;
    final out = Float64List(_terms);
    final tmp = Float64List(_terms);
    final hu = (r.u1 - r.u0) / 2;
    final mu = (r.u1 + r.u0) / 2;
    final hv = (r.v1 - r.v0) / 2;
    final mv = (r.v1 + r.v0) / 2;
    for (var i = 0; i < gl.order; i++) {
      final v = mv + hv * gl.nodes[i];
      for (var j = 0; j < gl.order; j++) {
        f(mu + hu * gl.nodes[j], v, tmp);
        final w = gl.weights[i] * gl.weights[j];
        for (var k = 0; k < _terms; k++) {
          out[k] += w * tmp[k];
        }
      }
    }
    for (var k = 0; k < _terms; k++) {
      out[k] *= hu * hv;
    }
    return out;
  }

  static Float64List _overLoops(
    FaceDomainLoops dom,
    void Function(double, double, Float64List) f,
  ) {
    final gl = GaussLegendre.standard;
    final out = Float64List(_terms);
    final inner = Float64List(_terms);
    final tmp = Float64List(_terms);

    var uRef = double.infinity;
    for (final loop in dom.loops) {
      for (final seg in loop) {
        for (var i = 0; i <= 8; i++) {
          uRef = math.min(uRef, seg.at(i / 8).x);
        }
      }
    }
    if (!uRef.isFinite) return out;

    for (final loop in dom.loops) {
      for (final seg in loop) {
        for (var i = 0; i < gl.order; i++) {
          final t = 0.5 + 0.5 * gl.nodes[i];
          final dv = seg.dvdt(t);
          if (dv == 0) continue;
          final p = seg.at(t);

          inner.fillRange(0, _terms, 0);
          final half = (p.x - uRef) / 2;
          final mid = (p.x + uRef) / 2;
          for (var j = 0; j < gl.order; j++) {
            f(mid + half * gl.nodes[j], p.y, tmp);
            for (var k = 0; k < _terms; k++) {
              inner[k] += gl.weights[j] * tmp[k];
            }
          }

          final w = gl.weights[i] * half * dv * 0.5;
          for (var k = 0; k < _terms; k++) {
            out[k] += w * inner[k];
          }
        }
      }
    }
    return out;
  }

  static double _jacobian(BrepSurface s, double u, double v) {
    switch (s) {
      case BrepOffsetSurface():
        return s.du(u, v).cross(s.dv(u, v)).length;
      case BrepAffineSurface():
        return s.du(u, v).cross(s.dv(u, v)).length;
      case BrepSplineSurface():
        return s.du(u, v).cross(s.dv(u, v)).length;
      case BrepRuledSurface():
        return s.du(u, v).cross(s.dv(u, v)).length;
      case BrepPlane():
        return 1;
      case BrepCylinder():
        return s.radius;
      case BrepCone():
        final slope = s.height.abs() < 1e-18
            ? 0.0
            : (s.topRadius - s.radius) / s.height;
        return s.radiusAt(v).abs() * math.sqrt(1 + slope * slope);
      case BrepSphere():
        return s.radius * s.radius * math.sin(v).abs();
      case BrepTorus():
        return s.minorRadius.abs() *
            (s.majorRadius + s.minorRadius * math.cos(v)).abs();
    }
  }
}
