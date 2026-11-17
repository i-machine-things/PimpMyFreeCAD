# Worker for the experimental ray-traced sweep: run with FreeCAD's own freecadcmd so a long OCC call cannot freeze the GUI.
# usage: freecadcmd pda_bodysweep_rt_worker.py <workdir>   (reads in.json, tool.brep, path.brep; writes out.brep / out.json)
import json, os, sys, traceback
import FreeCAD as App
import Part

work = os.environ.get("PDA_RT_WORK") or sys.argv[-1]
res = {"ok": False, "error": "", "warnings": []}
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import pda_bodysweep_raytraced as rbs
    prm = json.load(open(os.path.join(work, "in.json")))
    if not rbs.HAVE_SHAPELY:
        raise RuntimeError("shapely is required for the Raytraced method")
    tool = Part.read(os.path.join(work, "tool.brep"))
    path = Part.read(os.path.join(work, "path.brep"))
    sol, info = rbs.build(tool, path, twist_deg=prm["twist_deg"], mode=prm["mode"], method="fast")
    sol.exportBrep(os.path.join(work, "out.brep"))
    res.update(ok=True, warnings=info.get("warnings", []), method=info.get("method"))
except Exception as ex:
    if ex.__class__.__name__ == "BodySweepError":
        res["error"] = str(ex)
        res["code"] = "outside_limits"
    else:
        res["error"] = "Body Sweep (experimental): %s" % ex
    res["trace"] = traceback.format_exc()[-1500:]
json.dump(res, open(os.path.join(work, "out.json"), "w"))
