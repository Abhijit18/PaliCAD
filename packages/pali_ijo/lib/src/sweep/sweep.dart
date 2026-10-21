/// Advanced sweeps: revolve, sweep along a path, loft, pipe and helix.
///
/// Pulling or turning a 2D profile along a trajectory, which is how the
/// features that no combination of primitives and booleans can reach get made.
///
/// The module divides on a real line rather than an arbitrary one:
///
/// - [IjoRevolve] is **exact**. Revolving lines and arcs about an axis lands
///   precisely on planes, cylinders, cones, spheres and tori, so a turned part
///   comes out as true analytic geometry.
/// - [IjoSweep] is **faceted**. A profile carried along a curved path sweeps a
///   surface with no exact name in this kernel, so it is approximated by a
///   controllable number of stations.
///
/// Prefer a revolve whenever the shape admits one.
library;

export 'profile.dart'
    show ProfileArc, ProfileLine, ProfileSegment, SweepProfile;
export 'revolve.dart' show IjoRevolve;
export 'skin.dart' show IjoSkin, PathFrames;
export 'sweeps.dart' show IjoSweep;
