import 'dart:math' as math;

/// Tessellation density for curved mesh generation.
///
/// Viewport shading can look round without extra triangles; this setting
/// controls how many faces are actually built (and therefore exported).
enum MeshQuality {
  /// Fast previews. Visible facets in STL/export; viewport still smooth-shades.
  draft,

  /// Default. Matches historical cylinder/sphere counts.
  standard,

  /// Denser fillets, spheres, cylinders, and pipes. Heavier CSG.
  fine,
}

class MeshTessellation {
  const MeshTessellation(
    this.quality, {
    this.surfaceChordToleranceMm,
    this.maxSurfaceRefinement = 8,
  }) : assert(maxSurfaceRefinement >= 1 && maxSurfaceRefinement <= 12),
       assert(surfaceChordToleranceMm == null || surfaceChordToleranceMm > 0);

  /// Optional display-only limits. Exact CAD geometry is unaffected.
  final double? surfaceChordToleranceMm;
  final int maxSurfaceRefinement;

  /// Bounded initial display cost for large imported assemblies.
  static const importPreview = MeshTessellation(
    MeshQuality.draft,
    surfaceChordToleranceMm: .1,
    maxSurfaceRefinement: 4,
  );

  final MeshQuality quality;

  static const draft = MeshTessellation(MeshQuality.draft);
  static const standard = MeshTessellation(MeshQuality.standard);
  static const fine = MeshTessellation(MeshQuality.fine);

  static MeshTessellation fromName(String? name) {
    switch (name) {
      case 'draft':
        return draft;
      case 'fine':
        return fine;
      default:
        return standard;
    }
  }

  int get index => quality.index;

  static MeshTessellation fromIndex(int index) {
    switch (index.clamp(0, 2)) {
      case 0:
        return draft;
      case 2:
        return fine;
      default:
        return standard;
    }
  }

  int get cylinderSegments => switch (quality) {
        MeshQuality.draft => 16,
        MeshQuality.standard => 32,
        MeshQuality.fine => 64,
      };

  int get sphereLatSegments => switch (quality) {
        MeshQuality.draft => 12,
        MeshQuality.standard => 24,
        MeshQuality.fine => 36,
      };

  int get sphereLonSegments => switch (quality) {
        MeshQuality.draft => 16,
        MeshQuality.standard => 32,
        MeshQuality.fine => 48,
      };

  /// Geodesic icosphere depth. UV spheres put a high-valence pole on every
  /// meridian; those slivers break mesh CSG. 2/3/4 → 320 / 1280 / 5120 tris.
  int get sphereIcoSubdiv => switch (quality) {
        MeshQuality.draft => 2,
        MeshQuality.standard => 3,
        MeshQuality.fine => 4,
      };

  /// Map a lat-segment count (older UV API) onto [sphereIcoSubdiv].
  static int sphereIcoSubdivForLat(int latSegments) {
    if (latSegments <= 16) return 2;
    if (latSegments <= 30) return 3;
    return 4;
  }

  int get pipeSegments => switch (quality) {
        MeshQuality.draft => 12,
        MeshQuality.standard => 16,
        MeshQuality.fine => 24,
      };

  /// Helical stations per turn for a thread cutter.
  ///
  /// Kept modest: finer helices graze the faceted wall and the subtract
  /// comes back open. Eight stations is enough to read as a thread and is
  /// the count that reliably closes on a cylinder.
  int get threadSegments => switch (quality) {
        MeshQuality.draft => 8,
        MeshQuality.standard => 8,
        MeshQuality.fine => 10,
      };

  int get sketchCircleSegments => switch (quality) {
        MeshQuality.draft => 24,
        MeshQuality.standard => 32,
        MeshQuality.fine => 64,
      };

  /// Full-circle facet count for a fillet of [radius] mm.
  int filletSegments(double radius) {
    final circ = 2 * math.pi * radius.abs();
    final (chord, lo, hi) = switch (quality) {
      MeshQuality.draft => (0.35, 12, 32),
      MeshQuality.standard => (0.15, 24, 80),
      MeshQuality.fine => (0.08, 48, 128),
    };
    return (circ / chord).round().clamp(lo, hi);
  }

  int filletArcSegments(double radius, double angleRad) {
    final full = filletSegments(radius);
    final n = (full * angleRad.abs() / (2 * math.pi)).round();
    return n.clamp(4, full);
  }
}
