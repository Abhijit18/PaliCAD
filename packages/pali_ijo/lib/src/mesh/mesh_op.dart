/// Thrown when a mesh modelling op cannot produce a closed solid.
class MeshOpException implements Exception {
  MeshOpException(this.message);
  final String message;
  @override
  String toString() => message;
}
