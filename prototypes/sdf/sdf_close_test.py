
# SDF gap-closing test: smooth + closing on the swept field, consistent orientation, drop floaters.
import os, sys, time
import numpy as np
from scipy import ndimage
import FreeCAD as App, Mesh

def _lib():
    src = open(os.path.expanduser("~/BodySweepProto/sdf_proto.py")).read()
    ns = {"__name__": "sdflib"}
    exec(compile(src.split("\nif True:")[0], "sdf_lib", "exec"), ns)
    return ns

def edge_stats(v, t):
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    n = np.cross(b-a, c-a); l = np.linalg.norm(n, axis=1); l[l == 0] = 1; n = n/l[:, None]
    e = np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]]); f = np.tile(np.arange(len(t)), 3)
    k = np.sort(e, 1); key = k[:, 0].astype(np.int64)*len(v)+k[:, 1]
    o = np.argsort(key, kind="stable"); key = key[o]; f = f[o]
    same = np.where(key[1:] == key[:-1])[0]
    ang = np.degrees(np.arccos(np.clip((n[f[same]]*n[f[same+1]]).sum(1), -1, 1)))
    return ang.mean(), np.percentile(ang, 99), 100.0*(ang > 60).mean()

def closing_field(D, h, c_vox, sigma_vox):
    ins = D < 0
    d_out = ndimage.distance_transform_edt(~ins, sampling=h)
    dil = d_out <= c_vox*h
    d_in = ndimage.distance_transform_edt(dil, sampling=h)
    clo = dil & (d_in > c_vox*h)
    clo |= ins                              # closing never removes original material
    eo = ndimage.distance_transform_edt(~clo, sampling=h); ei = ndimage.distance_transform_edt(clo, sampling=h)
    S = np.where(clo, -(ei-0.5*h), eo-0.5*h)
    S = np.clip(S, -20*h, 20*h)
    if sigma_vox > 0: S = ndimage.gaussian_filter(S, sigma_vox)
    return S

def to_mesh_clean(v, t):
    m = Mesh.Mesh(v[t].tolist())
    m.harmonizeNormals()
    if m.Volume < 0: m.flipNormals()
    comps = m.getSeparateComponents()
    keep = [x for x in comps if x.CountFacets >= 0.02*m.CountFacets]
    if len(keep) != len(comps):
        mm = Mesh.Mesh()
        for x in keep: mm.addMesh(x)
        m = mm
    return m, len(comps)-len(keep)

def report(tag, m, extra=""):
    from collections import Counter
    c = Counter()
    for f in m.Facets:
        i = f.PointIndices
        for a, b in ((i[0], i[1]), (i[1], i[2]), (i[2], i[0])): c[(min(a, b), max(a, b))] += 1
    return "%s facets=%d vol=%d open=%d nonman=%d nonuniform=%d solid=%s comps=%d %s" % (
        tag, m.CountFacets, round(m.Volume), sum(1 for x in c.values() if x == 1), sum(1 for x in c.values() if x > 2),
        m.countNonUniformOrientedFacets(), m.isSolid(), len(m.getSeparateComponents()), extra)

def run(name, hdiv=30, c_vox=1.5, sigma_vox=0.8):
    L = _lib()
    tool, path, ref = L["shapes"](name)
    bb = tool.BoundBox; size = max(bb.XLength, bb.YLength, bb.ZLength); h = size/hdiv
    t0 = time.time()
    D, lo, h = L["sweep_sdf"](tool, path, h, lambda m: None)
    t1 = time.time()
    out = {}
    v0, t0_ = L["marching_tets"](D, lo, h)
    out["raw"] = (v0, t0_, edge_stats(v0, t0_))
    S = closing_field(D, h, c_vox, sigma_vox)
    v1, t1_ = L["marching_tets"](S, lo, h)
    out["closed"] = (v1, t1_, edge_stats(v1, t1_))
    t2 = time.time()
    res = {}
    for k, (v, t, es) in out.items():
        m, dropped = to_mesh_clean(v, t)
        res[k] = m
        print(report("%s/%s" % (name, k), m, "dihedral mean=%.1f p99=%.1f sharp>60deg=%.2f%% dropped_floaters=%d" % (es[0], es[1], es[2], dropped)))
    print("%s ref=%s sweep %.1fs closing+mesh %.1fs" % (name, ref, t1-t0, time.time()-t1))
    return res
