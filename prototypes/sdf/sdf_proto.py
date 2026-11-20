
# SDF prototype of the sweep: swept SDF = min over path samples of the tool SDF, surface by marching tetrahedra.
import sys, os, time
sys.path=[p for p in sys.path if "BodySweepProto" not in p]
import numpy as np
import FreeCAD as App, Part
V=App.Vector
sys.path.insert(0, os.path.expanduser("~/.local/share/FreeCAD/v26-3/Mod/PimpMyFreeCAD"))
import pda_bodysweep_raytraced as rbs
from scipy import ndimage
from scipy.spatial import cKDTree
import shapely
from shapely.geometry import Polygon

def tool_sdf(tl, ht):
    bb = tl.BoundBox; pad = 3*ht
    jit = 0.37*ht            # keep grid points off flat faces (a point exactly on the boundary counts as outside)
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
            pts = w.discretize(Deflection=0.05*ht)
            if len(pts) < 3: continue
            pg = Polygon([(p.y, p.z) for p in pts])
            if not pg.is_valid: pg = pg.buffer(0)
            if pg.is_empty: continue
            cnt += shapely.contains_xy(pg, Y, Z).astype(int)
        ins[i] = (cnt % 2) == 1
    eo = ndimage.distance_transform_edt(~ins, sampling=ht); ei = ndimage.distance_transform_edt(ins, sampling=ht)
    d = np.where(ins, -(ei-0.5*ht), eo-0.5*ht)       # zero level half a voxel from the inside centres
    return d, np.array([x0, y0, z0]), int(ins.sum())

def sweep_sdf(tool, path, h, log):
    t0 = time.time()
    bbw = tool.BoundBox; size = max(bbw.XLength, bbw.YLength, bbw.ZLength)
    pf = rbs.PathFrames(path, tool.BoundBox.Center, step=min(0.5, size/40), mode="rmf", twist_deg=0.0)
    L = pf.L
    pl0inv = pf.placement(0).inverse()
    tl = tool.copy(); tl.Placement = pl0inv.multiply(tool.Placement)
    ht = h
    sdf, org, nin = tool_sdf(tl, ht)
    log("tool sdf %s inside %d  %.1fs" % (sdf.shape, nin, time.time()-t0))
    ns = int(np.ceil(L/(0.5*h)))+1
    S = np.linspace(0, L, ns)
    Ms = []
    for s in S:
        m = pf.placement(s).Matrix
        M = np.array([[m.A11, m.A12, m.A13, m.A14],[m.A21, m.A22, m.A23, m.A24],[m.A31, m.A32, m.A33, m.A34],[0,0,0,1]], float)
        Ms.append(M)
    Ms = np.array(Ms)
    # world grid = bbox of all tool corners over the path
    bb = tl.BoundBox
    corners = np.array([[x, y, z, 1.0] for x in (bb.XMin, bb.XMax) for y in (bb.YMin, bb.YMax) for z in (bb.ZMin, bb.ZMax)])
    allc = np.concatenate([(M @ corners.T).T[:, :3] for M in Ms[::max(1, ns//40)]] + [(Ms[-1] @ corners.T).T[:, :3]])
    lo = allc.min(0)-2*h; hi = allc.max(0)+2*h
    shp = np.ceil((hi-lo)/h).astype(int)+1
    gx, gy, gz = [lo[k]+h*np.arange(shp[k]) for k in range(3)]
    G = np.stack(np.meshgrid(gx, gy, gz, indexing="ij"), -1).reshape(-1, 3)
    R = np.max(np.linalg.norm(corners[:, :3], axis=1))
    origins = Ms[:, :3, 3]
    tree = cKDTree(origins)
    dist_o, _ = tree.query(G)
    band = np.where(dist_o < R+2*h)[0]
    log("grid %s points %d band %d samples %d  %.1fs" % (tuple(shp), len(G), len(band), ns, time.time()-t0))
    P = G[band]
    D = np.full(len(band), 1e9)
    ptree = cKDTree(P)
    for k in range(ns):
        idx = np.array(ptree.query_ball_point(origins[k], R+2*h), dtype=int)
        if len(idx) == 0: continue
        M = Ms[k]; Rm = M[:3, :3]; t = M[:3, 3]
        Q = (P[idx]-t) @ Rm                          # = Rm^T (p - t)
        c = ((Q-org)/ht).T
        v = ndimage.map_coordinates(sdf, c, order=1, mode="nearest")
        D[idx] = np.minimum(D[idx], v)
    Dg = np.full(len(G), 1e3)
    Dg[band] = np.minimum(D, 1e3)
    log("min over path done %.1fs" % (time.time()-t0))
    return Dg.reshape(tuple(shp)), lo, h

TETS = [(0,1,2,6),(0,2,3,6),(0,3,7,6),(0,7,4,6),(0,4,5,6),(0,5,1,6)]
CORN = [(0,0,0),(1,0,0),(1,1,0),(0,1,0),(0,0,1),(1,0,1),(1,1,1),(0,1,1)]
EDG = [(0,1),(0,2),(0,3),(1,2),(1,3),(2,3)]
TRI = {1:[(0,1,2)],2:[(0,3,4)],4:[(1,3,5)],8:[(2,4,5)],14:[(0,1,2)],13:[(0,3,4)],11:[(1,3,5)],7:[(2,4,5)],
       3:[(1,3,4),(1,4,2)],12:[(1,3,4),(1,4,2)],5:[(0,3,5),(0,5,2)],10:[(0,3,5),(0,5,2)],6:[(0,4,5),(0,5,1)],9:[(0,4,5),(0,5,1)]}

def marching_tets(D, lo, h):
    nx, ny, nz = D.shape
    neg = (D < 0)
    cnt = np.zeros((nx-1, ny-1, nz-1), int)
    for dx, dy, dz in CORN:
        cnt += neg[dx:nx-1+dx, dy:ny-1+dy, dz:nz-1+dz]
    ci = np.argwhere((cnt > 0) & (cnt < 8))
    lin = np.arange(D.size).reshape(D.shape)
    Df = D.ravel()
    ia_all = []; ib_all = []
    for tet in TETS:
        ids = np.stack([lin[ci[:, 0]+CORN[c][0], ci[:, 1]+CORN[c][1], ci[:, 2]+CORN[c][2]] for c in tet], 1)   # (m,4)
        vals = Df[ids]
        mask = ((vals[:, 0] < 0)*1 + (vals[:, 1] < 0)*2 + (vals[:, 2] < 0)*4 + (vals[:, 3] < 0)*8)
        for mk, tris in TRI.items():
            sel = np.where(mask == mk)[0]
            if len(sel) == 0: continue
            for tr in tris:
                a = []; b = []
                for e in tr:
                    p, q = EDG[e]
                    a.append(ids[sel, p]); b.append(ids[sel, q])
                ia_all.append(np.stack(a, 1)); ib_all.append(np.stack(b, 1))
    if not ia_all: return np.zeros((0, 3)), np.zeros((0, 3), int)
    ia = np.concatenate(ia_all); ib = np.concatenate(ib_all)       # (T,3)
    lo_id = np.minimum(ia, ib); hi_id = np.maximum(ia, ib)
    key = lo_id.astype(np.int64)*D.size + hi_id
    uk, first, inv = np.unique(key.ravel(), return_index=True, return_inverse=True)
    a = lo_id.ravel()[first]; b = hi_id.ravel()[first]
    def pos(i): return np.stack(np.unravel_index(i, D.shape), 1)*h + lo
    va = Df[a]; vb = Df[b]
    t = (va/(va-vb))[:, None]
    verts = pos(a) + (pos(b)-pos(a))*t
    tris = inv.reshape(-1, 3)
    # orient by the gradient of D (outward = D increasing)
    gr = np.gradient(np.clip(D, -1e3, 1e3), h)
    cen = verts[tris].mean(1)
    gi = np.clip(np.rint((cen-lo)/h).astype(int), 0, np.array(D.shape)-1)
    g = np.stack([gr[k][gi[:, 0], gi[:, 1], gi[:, 2]] for k in range(3)], 1)
    n = np.cross(verts[tris[:, 1]]-verts[tris[:, 0]], verts[tris[:, 2]]-verts[tris[:, 0]])
    flip = (n*g).sum(1) < 0
    tris[flip] = tris[flip][:, ::-1]
    return verts, tris

def mesh_volume(v, t):
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()/6.0)

def write_stl(path, v, t):
    with open(path, "w") as f:
        f.write("solid s\n")
        for tr in t:
            a, b, c = v[tr]
            n = np.cross(b-a, c-a); l = np.linalg.norm(n); n = n/l if l > 0 else n
            f.write("facet normal %g %g %g\nouter loop\n" % tuple(n))
            for p in (a, b, c): f.write("vertex %g %g %g\n" % tuple(p))
            f.write("endloop\nendfacet\n")
        f.write("endsolid s\n")

def shapes(name):
    base = os.path.expanduser("~/BodySweepProto")
    path = Part.read(base+"/path.brep")
    if name == "cyl_line":
        return Part.makeCylinder(32, 120, V(0, 0, 0), V(0, 0, 1)), Part.makeLine(V(0, 0, 60), V(0, 492.93, 60)), 4172109.0
    if name == "hammer":
        return Part.read(base+"/tool_hammer.brep").Solids[0], path, None
    if name == "torus":
        return Part.makeTorus(40, 12, V(0, -15, 0), V(0, 1, 0)), path, 3526404.0
    if name == "hourglass":
        prof = [(0,0),(30,0),(8,-40),(8,-45),(30,-85),(0,-85)]
        pts = [V(r, y, 0) for r, y in prof]
        return Part.Face(Part.makePolygon(pts+[pts[0]])).revolve(V(0,0,0), V(0,1,0), 360), path, 1526087.0
    if name == "cola":
        prof = [(0,0),(25,0),(33,-6),(33,-108),(27,-116),(27,-120),(0,-120)]
        pts = [V(r, y, 0) for r, y in prof]
        return Part.Face(Part.makePolygon(pts+[pts[0]])).revolve(V(0,0,0), V(0,1,0), 360), path, 2205440.0
    raise KeyError(name)

if True:
    name = os.environ["SHAPE"]; hdiv = float(os.environ.get("HDIV", "30"))
    tool, path, ref = shapes(name)
    bb = tool.BoundBox; size = max(bb.XLength, bb.YLength, bb.ZLength)
    h = size/hdiv
    t0 = time.time()
    D, lo, h = sweep_sdf(tool, path, h, lambda m: print("LOG", m, flush=True))
    v, t = marching_tets(D, lo, h)
    vol = mesh_volume(v, t)
    print("BBOX", v.min(0).round(1), v.max(0).round(1), "tool", tool.BoundBox, flush=True)
    out = os.path.expanduser("~/BodySweepProto/out_sdf_%s.stl" % name)
    write_stl(out, v, t)
    print("RES %s h=%.2f tris=%d volume=%d ref=%s diff=%s time=%.1fs" % (name, h, len(t), vol, ref,
          ("%+.2f%%" % (100*(vol-ref)/ref)) if ref else "-", time.time()-t0), flush=True)
