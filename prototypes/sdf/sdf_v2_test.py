
# SDF v2: tool distance from the true B-rep surface (sub-voxel), then optional closing/smoothing.
import os, time
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
import FreeCAD as App
import sdf_close_test as T
V = App.Vector

def surface_samples(tl, h):
    pts, tris = tl.tessellate(0.03*h)
    P = np.array([[p.x, p.y, p.z] for p in pts]); out = [P]
    for a, b, c in tris:
        A, B, C = P[a], P[b], P[c]
        e = max(np.linalg.norm(B-A), np.linalg.norm(C-B), np.linalg.norm(A-C))
        n = int(np.ceil(e/(0.3*h)))
        if n < 2: continue
        i, j = np.meshgrid(np.arange(n+1), np.arange(n+1), indexing="ij"); m = (i+j) <= n
        u = i[m]/n; v = j[m]/n
        out.append(A+np.outer(u, B-A)+np.outer(v, C-A))
    return np.concatenate(out)

def make_tool_sdf_exact(shapely_mod, Polygon):
    def tool_sdf_exact(tl, ht):
        bb = tl.BoundBox; pad = 3*ht; jit = 0.37*ht
        x0, y0, z0 = bb.XMin-pad+jit, bb.YMin-pad+jit, bb.ZMin-pad+jit
        nx = int(np.ceil((bb.XLength+2*pad)/ht))+1; ny = int(np.ceil((bb.YLength+2*pad)/ht))+1; nz = int(np.ceil((bb.ZLength+2*pad)/ht))+1
        ys = y0+ht*np.arange(ny); zs = z0+ht*np.arange(nz)
        Y, Z = np.meshgrid(ys, zs, indexing="ij")
        ins = np.zeros((nx, ny, nz), bool)
        for i in range(nx):
            x = x0+ht*i
            if x < bb.XMin-1e-9 or x > bb.XMax+1e-9: continue
            try: wires = tl.slice(V(1, 0, 0), x)
            except Exception: continue
            cnt = np.zeros((ny, nz), int)
            for w in wires:
                if not w.isClosed(): continue
                p = w.discretize(Deflection=0.05*ht)
                if len(p) < 3: continue
                pg = Polygon([(q.y, q.z) for q in p])
                if not pg.is_valid: pg = pg.buffer(0)
                if pg.is_empty: continue
                cnt += shapely_mod.contains_xy(pg, Y, Z).astype(int)
            ins[i] = (cnt % 2) == 1
        S = surface_samples(tl, ht); tree = cKDTree(S)
        X = x0+ht*np.arange(nx)
        G = np.stack(np.meshgrid(X, ys, zs, indexing="ij"), -1).reshape(-1, 3)
        dist, _ = tree.query(G, workers=-1)
        dist = dist.reshape(nx, ny, nz)
        d = np.where(ins, -dist, dist)
        return d, np.array([x0, y0, z0]), int(ins.sum())
    return tool_sdf_exact

def run2(name, hdiv=30, c_vox=1.0, sigma_vox=0.4):
    L = T._lib()
    import shapely
    from shapely.geometry import Polygon
    L["tool_sdf_old"] = L["tool_sdf"]
    tool, path, ref = L["shapes"](name)
    bb = tool.BoundBox; size = max(bb.XLength, bb.YLength, bb.ZLength); h = size/hdiv
    res = {}
    for tag, exact in (("old", False), ("exact", True)):
        if exact: L["tool_sdf"] = make_tool_sdf_exact(shapely, Polygon)
        else: L["tool_sdf"] = L["tool_sdf_old"]
        t0 = time.time()
        D, lo, hh = L["sweep_sdf"](tool, path, h, lambda m: None)
        v, t = L["marching_tets"](D, lo, hh); es = T.edge_stats(v, t)
        m, dr = T.to_mesh_clean(v, t); res[tag+"_raw"] = m
        print(T.report("%s/%s_raw" % (name, tag), m, "dihedral mean=%.1f p99=%.1f sharp=%.2f%% t=%.1fs" % (es[0], es[1], es[2], time.time()-t0)))
        if exact:
            S = T.closing_field(D, hh, c_vox, sigma_vox)
            v, t = L["marching_tets"](S, lo, hh); es = T.edge_stats(v, t)
            m, dr = T.to_mesh_clean(v, t); res["exact_closed"] = m
            print(T.report("%s/exact_closed" % name, m, "dihedral mean=%.1f p99=%.1f sharp=%.2f%%" % (es[0], es[1], es[2])))
    return res


def lib_sf(sf):
    src = open(os.path.expanduser("~/BodySweepProto/sdf_proto.py")).read().split("\nif True:")[0]
    assert "ns = int(np.ceil(L/(0.5*h)))+1" in src
    src = src.replace("ns = int(np.ceil(L/(0.5*h)))+1", "ns = int(np.ceil(L/(0.5*h/SDF_SF)))+1")
    ns = {"__name__": "sdflib2", "SDF_SF": sf}
    exec(compile(src, "sdf_lib_sf", "exec"), ns)
    import shapely
    from shapely.geometry import Polygon
    ns["tool_sdf"] = make_tool_sdf_exact(shapely, Polygon)
    return ns

def local_closing(D, h, c_vox=1.0, sigma_vox=0.5):
    """Fill crevices without re-quantising the field: only voxels added by a binary closing get filled, then smooth locally."""
    ins = D < 0
    dil = ndimage.distance_transform_edt(~ins, sampling=h) <= c_vox*h
    clo = dil & (ndimage.distance_transform_edt(dil, sampling=h) > c_vox*h)
    added = clo & ~ins
    S = D.copy()
    S[added] = np.minimum(S[added], -0.25*h)
    near = ndimage.binary_dilation(added, iterations=2)
    Sg = ndimage.gaussian_filter(np.clip(S, -20*h, 20*h), sigma_vox)
    S[near] = Sg[near]
    return S, int(added.sum())

def run3(name, sf, hdiv=30, closing=False):
    L = lib_sf(sf)
    tool, path, ref = L["shapes"](name)
    bb = tool.BoundBox; size = max(bb.XLength, bb.YLength, bb.ZLength); h = size/hdiv
    t0 = time.time()
    D, lo, hh = L["sweep_sdf"](tool, path, h, lambda m: None)
    tag = "%s/SF%d" % (name, sf)
    extra_txt = ""
    if closing:
        D, nadd = local_closing(D, hh); tag += "+localclose"; extra_txt = "added_vox=%d " % nadd
    v, t = L["marching_tets"](D, lo, hh); es = T.edge_stats(v, t)
    m, dr = T.to_mesh_clean(v, t)
    print(T.report(tag, m, extra_txt + "dihedral mean=%.1f p99=%.1f sharp=%.2f%% t=%.1fs" % (es[0], es[1], es[2], time.time()-t0)))
    return m
