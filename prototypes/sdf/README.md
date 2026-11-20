# SDF coarse Body Sweep: prototype scripts

Research prototype for the last-resort tier of Body Sweep. Not part of the add-on and not loaded by it.
Results, timings and open items are in [`docs/SDF_NOTES.md`](../../docs/SDF_NOTES.md).

Scripts expect to run inside FreeCAD (for the test tools) with this folder copied to `~/BodySweepProto/`
(they read `~/BodySweepProto/sdf_proto.py` and `~/BodySweepProto/remesh_out/...`).

| File | Purpose |
|---|---|
| `sdf_proto.py` | SDF of a tool swept along a path, marching tetrahedra, test tools (cola, torus, hammer) |
| `sdf_close_test.py`, `sdf_v2_test.py` | gap closing (`local_closing`), fewer path copies (`lib_sf`), exact tool distance |
| `qr_run2.py` | QuadWild (QRemeshify Linux libs) quad cage |
| `loft_net.py` | path-perpendicular slice net, corner tracking, polygon Catmull-Clark with creases |
| `job_field.py`, `job_wrap.py` | background jobs: field + ridge correction; Catmull-Clark x2 + shrinkwrap |
| `job_bench.py` | timing against the ray-traced methods |

Run heavy jobs in a background thread: they block FreeCAD's GUI thread otherwise.
