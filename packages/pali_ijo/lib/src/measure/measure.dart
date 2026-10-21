/// Calculation and evaluation: volume, surface area, bounding box, centroid.
///
/// Properties are summed from the triangle mesh. See [IjoMeasure].
library;

export 'bounds.dart'
    show IjoBounds, meshBounds, solidBounds, conservativeSolidBounds;
export 'properties.dart' show IjoMeasure, IjoProperties;
export 'quadrature.dart' show GaussLegendre;
