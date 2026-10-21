/// Where a face or edge came from.
///
/// Face and edge indices are not names. A boolean renumbers everything, a
/// fillet inserts faces in the middle, and a hollow doubles the count — so
/// "face 7" means nothing across an operation. That is the topological naming
/// problem, and no kernel has solved it in general.
///
/// This kernel uses an **ancestry tag**, not a stable global name. A
/// per-triangle face id points back at the original it came from. That
/// does not promise that face 7 stays face 7.
///
/// What that buys, concretely:
///
/// - A face colour, or a selection, can follow a face through an operation
///   instead of being thrown away.
/// - When a boolean splits one face into three, all three still point at the
///   one face they came from.
/// - The faces a fillet *invents* are distinguishable from the faces it
///   inherited, which is what makes "select the fillet" expressible.
class BrepOrigin {
  const BrepOrigin({
    required this.original,
    required this.entity,
    this.createdBy,
  });

  /// Identifies the solid this entity was born in. See
  /// [BrepOriginIds.next] for how these are handed out.
  final int original;

  /// Index of the face or edge within that original.
  final int entity;

  /// The operation that invented this entity, when it has no ancestor — a
  /// fillet's blend surface, or a boolean's cut face. Null when inherited.
  final String? createdBy;

  bool get isCreated => createdBy != null;
  bool get isInherited => createdBy == null;

  Map<String, dynamic> toJson() => {
        'o': original,
        'e': entity,
        if (createdBy != null) 'c': createdBy,
      };

  /// Reads a tag, and reserves the id it names.
  ///
  /// The reservation is the point. An id read from a file was handed out in
  /// some earlier process, where the counter has since been forgotten — so
  /// without this, loading a document whose solids carry ids 1, 2, 3 and then
  /// drawing a new box hands that box id 1 as well. Two unrelated solids would
  /// then claim one ancestry, and a provenance selector run after a boolean
  /// between them would match the wrong faces.
  ///
  /// Doing it here rather than in each loader means every route in is covered:
  /// [BrepSolid.fromJson], [IjoDocument], a host's own document format. Reading
  /// a tag this process issued costs nothing, since [BrepOriginIds.observe]
  /// only moves a counter that is behind — which is what keeps `clone()`, a
  /// JSON round trip, from burning ids.
  static BrepOrigin fromJson(Map<String, dynamic> json) {
    final original = (json['o'] as num).toInt();
    BrepOriginIds.observe(original);
    return BrepOrigin(
      original: original,
      entity: (json['e'] as num).toInt(),
      createdBy: json['c'] as String?,
    );
  }

  @override
  bool operator ==(Object other) =>
      other is BrepOrigin &&
      other.original == original &&
      other.entity == entity &&
      other.createdBy == createdBy;

  @override
  int get hashCode => Object.hash(original, entity, createdBy);

  @override
  String toString() => createdBy == null
      ? 'BrepOrigin($original:$entity)'
      : 'BrepOrigin($original:$entity, created by $createdBy)';
}

/// Hands out the ids that [BrepOrigin.original] refers to.
///
/// Ids are process-local and monotonic. They are meaningful inside one
/// modelling session and across a save/load of the same document, not between
/// unrelated documents.
class BrepOriginIds {
  BrepOriginIds._();

  static int _next = 1;

  /// The next unused id.
  static int next() => _next++;

  /// Reserves [count] consecutive ids and returns the first, mirroring
  /// Useful when building several solids that need to
  /// agree on ids before any of them exists.
  static int reserve(int count) {
    if (count < 0) throw ArgumentError.value(count, 'count', 'must be >= 0');
    final first = _next;
    _next += count;
    return first;
  }

  /// Highest id handed out so far. Persist this alongside a document so a
  /// reload does not reissue ids that its solids already use.
  static int get watermark => _next - 1;

  /// Moves the counter past [id] so no future id collides with a loaded
  /// document's tags.
  static void observe(int id) {
    if (id >= _next) _next = id + 1;
  }
}
