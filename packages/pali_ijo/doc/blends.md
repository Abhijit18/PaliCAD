# Fillet and chamfer

`IjoOps.filletSolid` and `IjoOps.chamferSolid` return BREP. Tessellation is derived display data. Invalid geometry throws `BrepUnsupported`; these APIs never replace the source solid with a mesh.

## Supported constructions

- Straight edges between planar faces, with the existing prism/box corner constructions.
- Full circular cap rims on cylinders and cones. Cylinder fillets use tori; chamfers use cones. Bore chamfers enlarge the opening and retain inward wall orientation, including conical cavities.
- Smooth, closed rims of perpendicular extrusions. A rim can contain lines, partial circular arcs, ellipses, or spline curves. Selecting a portion propagates around its tangent-continuous rim. Sharp rim junctions are not silently smoothed by this path.
- Whole extrusion rims mixing smooth curves with sharp line-line corners, including a rectangular prism with one previously rounded upright edge. Sharp corners use shared miter sections between the cap blends; they do not require rounding the remaining uprights. This path supports Default, Two Distance, and Set Angle chamfers. Sharp curve-curve or line-curve joins still require additional endpoint constructions.
- Rigid placement, reflection, and uniform scale of supported bodies. Distances remain measured in the transformed body's units.

The extrusion path offsets the source profile and constructs common contact boundaries for the cap, wall, and blend. Fillet cross-sections are rational circular arcs. Longitudinal offsets use adaptive cubic fitting, checked at interior stations against a `1e-5` mm target. This is numerical BREP geometry, not an exact symbolic offset or a certified global error bound. Adaptive spans share spline patches to limit topology size. The source wall remains trimmed to the contact boundary.

At a sharp line-line corner, the common section is an elliptic miter between the neighbouring blends. Descending curve parameter ranges are distinguished from reversed coedges. Display tessellation preserves shared boundary samples even when opposite patch boundaries need different sample counts; polynomial cubic spans use derivative bounds and aligned knots to avoid excessive subdivision.

## Chamfer conventions

```dart
final equal = IjoOps.chamferSolid(solid, edges, 1.0);
final unequal = IjoOps.chamferSolid(solid, edges, 1.0,
  options: const ChamferOptions(
    mode: ChamferMode.twoDistance,
    distance2: 2.0,
  ));
final angled = IjoOps.chamferSolid(solid, edges, 1.0,
  options: const ChamferOptions(
    mode: ChamferMode.setAngle,
    angleDegrees: 30.0,
  ));
```

Default uses equal setbacks, giving 45 degrees for perpendicular faces. Two Distance measures a setback on each incident face. Set Angle measures from the first face toward the chamfer in the normal section. With section angle `beta`, the second setback is `d1*sin(angle)/sin(beta+angle)`. Values that do not fit that section are rejected.

For planar pairs, first and second follow the incident-face order. Circular cap rims and smooth extrusion rims use the cap first, wall second. `BrepFillet.chamferFrame` supplies consistent setback directions for host controls.

PaliCAD exposes these options in its Chamfer dropdown. Two Distance has two numbered arrows; Set Angle has an arrow and a rotating dial. Numeric fields update the same preview parameters. Jobs stay in a cancellable background isolate. A mode change invalidates the prior preview; malformed input or a failed preview cannot cause Apply to commit an older successful result.

## Validation and remaining limits

The new paths check finite inputs, cap clearance, opposite wall boundaries, local offset reversal, and contact continuity. The host also requires a closed display tessellation. These checks do not constitute a general self-intersection proof.

Arbitrary spatial spline intersection edges, variable rolling-ball contact on two freeform surfaces, isolated partial rims requiring endpoint patches, and general multi-surface vertex blends still need dedicated constructions. Nonuniformly scaled blends are not automatically equivalent to a constant-radius blend. Available distance alone does not establish that a valid blend exists: offset curvature, tangencies, and topology also matter. Unsupported configurations must preserve the original BREP and report an error.

## Tests

The Dart-only `test/curved_blend_modes_test.dart` covers plane, cylinder, bore, cone, ellipse, cubic-spline and mixed line/arc constructions; transforms; sequential operations; invalid dimensions; serialization; fillet radius/tangency; triangle budgets; and exactly two opposite uses of each display edge. Existing fillet, radius, concave-corner, loft, and affine tests remain applicable.

PaliCAD's `test/chamfer_modes_flow_test.dart` and `test/chamfer_mode_widget_test.dart` cover mode changes at a fixed first distance, stale jobs, preview reuse, invalid second distances, Apply, dropdown fields, and angle-dial math. Existing cancellation/reopen and Enter tests cover shared tool behavior.

`test/mixed_corner_cap_blend_test.dart` covers top/bottom cap blends after an upright fillet, all chamfer modes, source preservation, excessive-radius rejection, analytic volume, surface tangency, edge parameter orientation, older reversed line records, opposite-cap sequencing, and display triangle limits. The app's `test/mixed_corner_cap_flow_test.dart` starts from the actual primitive tool and performs both operations through preview, cancellation, and Apply, checking that the body and cache retain BREP.
