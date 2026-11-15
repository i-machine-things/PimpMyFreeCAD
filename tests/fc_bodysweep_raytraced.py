# -*- coding: utf-8 -*-
"""Ray-Traced Body Sweep test bench (runs inside FreeCAD, needs shapely).

From the FreeCAD Python console (repo root on sys.path):

    exec(open("/path/to/PimpMyFreeCAD/tests/fc_bodysweep_raytraced.py").read())

Builds 8 tool bodies on one 3D B-spline path (138.5 mm, rising 15 mm), sweeps each one,
cuts a block and measures how far tool surface samples stick out of the sweep.
Results on 2026-11-16 (Windows laptop, FreeCAD + shapely 2.1):

    box_fillet   0 deg   4 faces  max out 0.33 mm   9 s
    cola_lying   0 deg  46 faces  max out 0.35 mm   3 s
    cola_twist 180 deg  50 faces  max out 0.45 mm   3 s
    L          180 deg   8 faces  max out 0.26 mm   3 s
    D          180 deg   4 faces  max out 0.13 mm   3 s
    T          180 deg  10 faces  max out 0.42 mm   4 s
    hammer     180 deg  15 faces  max out 0.25 mm   3 s
    hourglass  180 deg  47 faces  max out 0.33 mm   6 s
All: 0 folds, 0 ray misses, valid solid, valid cut.
"""
import os
import sys
import time
import importlib

import FreeCAD as App
import Part
from FreeCAD import Vector as V

_here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
_root = os.path.abspath(os.path.join(_here, ".."))
if _root not in sys.path:
    sys.path.insert(0, _root)
import pda_bodysweep_raytraced as rbs
importlib.reload(rbs)


def make_path():
    bs = Part.BSplineCurve()
    bs.interpolate([V(0, 0, 0), V(40, 8, 0), V(80, 35, 8), V(105, 75, 15)])
    return bs.toShape()


def make_cases():
    """Tool bodies in path-local coordinates: x along the path (tool behind its
    reference point, x <= 0), z up."""
    C = {}
    b = Part.makeBox(20, 10, 10)
    e = [x for x in b.Edges if abs((x.Vertexes[1].Point - x.Vertexes[0].Point).x) > 1
         and abs(x.Vertexes[0].Point.y - 10) < 1e-6 and abs(x.Vertexes[0].Point.z - 10) < 1e-6][0]
    b = b.makeFillet(4, [e])
    e = [x for x in b.Edges if x.Length > 15 and abs(x.Vertexes[0].Point.y) < 1e-6
         and abs(x.Vertexes[0].Point.z) < 1e-6][0]
    C["box_fillet"] = (b.makeFillet(2, [e]).transformGeometry(
        App.Matrix(1, 0, 0, -20, 0, 1, 0, -5, 0, 0, 1, -5, 0, 0, 0, 1)), 0)
    C["cola_lying"] = (Part.makeCylinder(5, 16, V(-5, -8, 0), V(0, 1, 0)), 0)
    C["cola_twist"] = (Part.makeCylinder(5, 16, V(-5, 0, -8), V(0, 0, 1)), 180)
    C["L"] = (Part.Face(Part.makePolygon([V(-10, -5, -6), V(-10, 5, -6), V(-10, 5, -2), V(-10, -1, -2),
                                           V(-10, -1, 6), V(-10, -5, 6), V(-10, -5, -6)])).extrude(V(10, 0, 0)), 180)
    x0 = -10
    arc = Part.Arc(V(x0, 0, 6), V(x0, 6, 0), V(x0, 0, -6)).toShape()
    C["D"] = (Part.Face(Part.Wire([Part.makeLine(V(x0, -2, -6), V(x0, -2, 6)), Part.makeLine(V(x0, -2, 6), V(x0, 0, 6)),
                                   arc, Part.makeLine(V(x0, 0, -6), V(x0, -2, -6))])).extrude(V(10, 0, 0)), 180)
    C["T"] = (Part.Face(Part.makePolygon([V(x0, -2, -6), V(x0, 2, -6), V(x0, 2, 2), V(x0, 6, 2), V(x0, 6, 6),
                                           V(x0, -6, 6), V(x0, -6, 2), V(x0, -2, 2), V(x0, -2, -6)])).extrude(V(10, 0, 0)), 180)
    C["hammer"] = (Part.makeBox(14, 3, 3, V(-20, -1.5, -1.5)).fuse(Part.makeBox(6, 12, 6, V(-6, -6, -3))).removeSplitter(), 180)
    cA = Part.makeCone(2.5, 6, 8, V(-6, 0, 0), V(0, 0, 1))
    cB = Part.makeCone(2.5, 6, 8, V(-6, 0, 0), V(0, 0, -1))
    C["hourglass"] = (cA.fuse(cB).removeSplitter(), 180)
    return C


def run(names=None, show=True):
    path = make_path()
    cases = make_cases()
    pf0 = rbs.PathFrames(path, path.Vertexes[0].Point)
    doc = App.newDocument("RayTracedSweepTests") if show else None
    if doc:
        doc.addObject("Part::Feature", "Path").Shape = path
    results = {}
    for name in (names or list(cases)):
        local, twist = cases[name]
        tool = local.copy()
        tool.Placement = pf0.placement(0).multiply(local.Placement)
        t0 = time.time()
        sol, info = rbs.build(tool, path, twist_deg=twist)
        cut = Part.makeBox(70, 60, 14, V(25, -10, -6)).cut(sol, 1e-3)
        out = rbs.coverage(tool, path, sol, info, ncopies=8, every=6)
        r = dict(twist=twist, folds=info["folds"], misses=info["misses"], valid=sol.isValid(),
                 faces=len(sol.Faces), cut_valid=cut.isValid(), max_out_mm=round(out, 2),
                 seconds=round(time.time() - t0, 1), warnings=info["warnings"])
        results[name] = r
        print(name, r)
        if doc:
            doc.addObject("Part::Feature", name + "_tool").Shape = tool
            o = doc.addObject("Part::Feature", name + "_sweep"); o.Shape = sol
            o.ViewObject.Visibility = False
    if doc:
        doc.recompute()
    bad = [n for n, r in results.items()
           if r["folds"] or r["misses"] or not r["valid"] or not r["cut_valid"] or r["max_out_mm"] > 0.6]
    print("FAILED:" if bad else "OK", bad if bad else "")
    return results


if __name__ == "__main__":
    run()
