# Body Sweep — SDF coarse method (last-resort tier): notes

Status: prototype (BodySweepProto), not yet wired into the add-on.

## Pipeline
1. Signed distance field (SDF) of the tool swept along the path (RMF frames; exact B-rep tool distance; min over path copies).
2. Local closing (fills crevices between copies, rounds concave corners).
3. Marching tetrahedra -> reference triangle mesh.
4. QuadWild (QRemeshify libs) -> low-resolution quad cage (scaleFact sets the cage size).
5. Catmull-Clark x2 (volume shrinks with coarse cages: cola -8 %, torus -6 %).
6. "Shrinkwrap": Newton projection of the CC mesh onto the zero set of the smoothed SDF, only in a repair zone (distance > 0.6 mm + 4 vertex rings).
7. Quad surface -> triangles -> Part solid -> boolean cut.

## Results (shrinkwrap, QuadWild s2 + CC x2)
| Tool | mean dist. to SDF before -> after | max dist. to SDF before -> after | volume vs SDF before -> after |
|---|---|---|---|
| hammer | 0.19 -> 0.14 mm | 8.5 -> 1.3 mm | -2.6 % -> -1.4 % |
| cola | 1.38 -> 0.07 mm | 4.13 -> 0.74 mm | -8.2 % -> -0.6 % |
| torus | 0.96 -> 0.12 mm | 3.87 -> 0.59 mm | -6.1 % -> +0.02 % |

Cost: hammer dihedral-angle p99 19 deg -> 42 deg (rougher; cola/torus not measured).
Hammer solid from the shrinkwrapped CC x2 surface: valid, 1 solid, 138,773 faces, box cut valid in 25.9 s.

## Timing vs. the other Body Sweep methods (same tool + path, desktop PC)
| Method | cola | torus | hammer |
|---|---|---|---|
| ray-traced "fast" | 51 s (fell back) | 100 s (time budget, fell back) | FAIL |
| "standard" (fuse cap) | 15 s | 51 s | FAIL |
| "coarse" | 3 s (vol -4.8 %) | FAIL | FAIL |
| SDF fast (SF1, hdiv 30) | 11 s | 11 s | 7 s |
| SDF quality (SF2, hdiv 60) | 248 s | 196 s | 144 s |

- SDF fast works on all three (also the hammer, which every other method rejects), but it is not faster than "standard"/"coarse" where those work.
- SDF quality is slow (minutes): the field computation dominates (Python + KD-tree). QuadWild/CC/shrinkwrap take seconds (QuadWild not timed separately).
- The torus' through-hole is filled by the other methods (volume 3.53 M mm3 vs 2.55 M mm3 for SDF); SDF keeps the hole.

## Known limits / open items
- Waviness on the hammer's top face and bottom edge; teeth near the path end come from large cage quads at the nose + CC.
- Weaving the SubD net from path-perpendicular slices works on the body (mean |D| 0.16 mm) but fails at curved noses/ends.
- Star-shaped tool (sharp edges along the path) not yet tested; no crease-aware shrinkwrap.
- Not yet integrated into the add-on; QuadWild libs are Linux-only here.
