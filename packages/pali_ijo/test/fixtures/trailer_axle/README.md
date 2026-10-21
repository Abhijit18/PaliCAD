# Mirrored trailer axle

These compressed BREP fixtures are the two saved axle halves from `TrailerTesting011Version.pali`. They contain geometry only. The app places both at `(0, 0, 330.2)` with identity body rotations.

Each half has 42 faces: four planes, six cylinders and 32 spline-based ruled surfaces, wrapped in an affine transform. One transform is identity; the other reflects X. The opposing center caps have matching circular trims. Union must remove those caps and sew the shared rim, retaining 82 exact faces in one closed body. It must not require trimming the distant wheel surfaces or converting them to mesh.
