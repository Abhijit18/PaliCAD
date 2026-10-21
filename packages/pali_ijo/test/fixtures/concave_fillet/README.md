# Concave fillet regression

Captured read-only from the original BREP in the live fillet session on 2026-09-12. The source is a 100 × 100 × 20 mm frame with a 60 × 60 mm through-hole. Edge 17 is an inside upright corner; edge 2 is an outside upright corner. The reported preview radius was 17.971043513278715 mm.

A normal inside fillet must add material to the hole corner. For radius r on one 90-degree upright corner, the added volume is `(1 - pi/4) * r² * 20`. An outside fillet removes the same volume. The new cylindrical face is reversed at an inside corner, and each shared edge must have opposite coedge uses.

The regression covers both kinds of corner, mirrored geometry, reversed edge and surface representations, and four inside corners in either selection order. App tests also exercise this fixture through the cancellable fillet preview worker.
