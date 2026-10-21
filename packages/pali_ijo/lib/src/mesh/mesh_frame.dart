import 'package:vector_math/vector_math_64.dart';

/// A stable in-plane axis perpendicular to [axis].
Vector3 meshFrameU(Vector3 axis) {
  var u = axis.cross(Vector3(0, 0, 1));
  if (u.length < 1e-8) u = axis.cross(Vector3(0, 1, 0));
  return u.normalized();
}
