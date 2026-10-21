import 'dart:math' as math;

/// Dimensions of the truncated 60-degree thread form, in model units.
/// Pitch is axial crest spacing; lead is pitch multiplied by the start count.
/// Fit allowances are separate from these basic profile dimensions.
class ThreadForm {
  ThreadForm({required this.pitch, required this.internal}) {
    if (!pitch.isFinite || pitch <= 0)
      throw ArgumentError('Thread pitch must be positive and finite');
  }
  final double pitch;
  final bool internal;
  double get depth => pitch * math.sqrt(3) * (internal ? 5 / 16 : 17 / 48);
  double get crestWidth => pitch * (internal ? 1 / 4 : 1 / 8);
  double get grooveHalfWidth => (pitch - crestWidth) / 2;
  double get rootHalfWidth => grooveHalfWidth - depth / math.sqrt(3);
  static List<(double, double)> regions({
    required double axisZMin,
    required double axisZMax,
    required double length,
    required bool fromEndA,
    required bool fromEndB,
  }) {
    if (![axisZMin, axisZMax, length].every((x) => x.isFinite) || length <= 0) {
      throw ArgumentError(
        'Thread length and axial limits must be finite and length positive',
      );
    }
    final lo = math.min(axisZMin, axisZMax), hi = math.max(axisZMin, axisZMax);
    if (hi - lo <= 1e-9) throw ArgumentError('Thread wall has no axial length');
    final len = math.min(length, hi - lo);
    if (!fromEndA && !fromEndB || fromEndA && fromEndB && 2 * len >= hi - lo)
      return [(lo, hi)];
    return [if (fromEndA) (lo, lo + len), if (fromEndB) (hi - len, hi)];
  }

  static double basicInternalMinorDiameter(
    double majorDiameter,
    double pitch,
  ) => majorDiameter - 2 * ThreadForm(pitch: pitch, internal: true).depth;
}
