import 'dart:math' as math;
import 'package:pali_ijo/pali_ijo.dart';
import 'package:vector_math/vector_math_64.dart';

BrepSolid hookedExtrusion() {
  final p = [
    Vector3(0, 0, 0),
    Vector3(80, 0, 0),
    Vector3(80, 20, 0),
    Vector3(68, 20, 0),
    Vector3(68, 15, 0),
    Vector3(70, 15, 0),
    Vector3(70, 5, 0),
    Vector3(0, 5, 0),
  ];
  final curves = <BrepCurve>[
    for (var i = 0; i < p.length; i++)
      if (i == 5)
        BrepCircle(
          center: Vector3(70, 10, 0),
          normal: Vector3(0, 0, 1),
          radius: 5,
          xDir: Vector3(1, 0, 0),
          t0: math.pi / 2,
          t1: -math.pi / 2,
        )
      else
        BrepLine(p[i], p[(i + 1) % p.length]),
  ];
  return BrepExtrude.fromCurves(p, curves, Vector3(0, 0, 10));
}
