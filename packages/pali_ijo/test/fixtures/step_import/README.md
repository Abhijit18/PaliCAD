# STEP trim regressions

These gzip-compressed STEP fragments retain only the referenced geometry and unit context for two faces from the private `7x14_trailer.STEP` stress file:

- `rounded_sphere_patch.step.gz`: body 4, face 226. Great-circle boundary samples become nearly collinear in the spherical chart; STEP rounding previously caused triangulation to return no triangles.
- `rounded_spline_patch.step.gz`: body 6, face 3226. Parameter welding collapses a tiny triangle to two distinct vertices, which previously threw during refinement.

`rounded_cylinder_patch.json` contains only the BREP geometry of body 1, face 124 from the same file. Tiny parameter-space slivers collapse during vertex welding or midpoint refinement; the fixture checks that tessellation retains the trim and produces valid triangles at preview, standard and fine quality. It contains no source file metadata.

Face and body numbers above are one-based. Each fragment deliberately contains an open, single-face shell; import must retain the face and report the open boundary. Headers contain no original file metadata. The full assembly is not included.

Kernel tests check boundary preservation and valid display triangles. PaliCAD tests exercise the asynchronous STEP import worker. To additionally run the complete private assembly test from the Main directory:

```sh
PALICAD_TRAILER_STEP=/path/to/7x14_trailer.STEP flutter test test/step_trailer_import_test.dart --concurrency=1
```
