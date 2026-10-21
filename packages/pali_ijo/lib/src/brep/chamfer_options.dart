import 'dart:math' as math;

import 'solid.dart';

/// Distances are setbacks on the two incident faces. Angle is measured from
/// the first face toward the chamfer in the edge's normal cross-section.
enum ChamferMode { defaultMode, twoDistance, setAngle }

class ChamferOptions {
  const ChamferOptions({
    this.mode = ChamferMode.defaultMode,
    this.distance2 = 1,
    this.angleDegrees = 45,
  });
  final ChamferMode mode;
  final double distance2;
  final double angleDegrees;

  double secondDistance(double first, double interiorAngle) {
    if (!first.isFinite || first < 0) {
      throw BrepUnsupported('Chamfer distance must be finite and nonnegative');
    }
    if (mode == ChamferMode.defaultMode) return first;
    if (mode == ChamferMode.twoDistance) {
      if (!distance2.isFinite || distance2 <= 0) {
        throw BrepUnsupported('Both chamfer distances must be positive');
      }
      return distance2;
    }
    final angle = angleDegrees * math.pi / 180;
    if (!angle.isFinite ||
        angle <= 0 ||
        angle + interiorAngle >= math.pi - 1e-6) {
      throw BrepUnsupported('Chamfer angle does not fit between these faces');
    }
    return first * math.sin(angle) / math.sin(interiorAngle + angle);
  }

  @override
  bool operator ==(Object other) =>
      other is ChamferOptions &&
      mode == other.mode &&
      distance2 == other.distance2 &&
      angleDegrees == other.angleDegrees;
  @override
  int get hashCode => Object.hash(mode, distance2, angleDegrees);

  ChamferOptions copyWith({
    ChamferMode? mode,
    double? distance2,
    double? angleDegrees,
  }) => ChamferOptions(
    mode: mode ?? this.mode,
    distance2: distance2 ?? this.distance2,
    angleDegrees: angleDegrees ?? this.angleDegrees,
  );
}
