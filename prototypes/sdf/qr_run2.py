
# Standalone QRemeshify (QuadWild) runner without Blender. Usage: python3 qr_run.py <name> [sharpAngle] [timeLimit] [scaleFact]
import sys, os, shutil, time, importlib
import numpy as np
BASE = os.path.expanduser("~/BodySweepProto/")
sys.path.insert(0, BASE + "QRemeshify")
import lib as qlib                      # lib/__init__.py (ctypes wrapper); avoids QRemeshify/__init__.py which imports bpy

def read_obj(p):
    V = []; F = []
    for ln in open(p):
        if ln.startswith("v "): V.append([float(x) for x in ln.split()[1:4]])
        elif ln.startswith("f "):
            idx = [int(t.split("/")[0])-1 for t in ln.split()[1:]]
            for k in range(1, len(idx)-1): F.append([idx[0], idx[k], idx[k+1]])
    return np.array(V), np.array(F)

def write_obj(p, V, F, N):
    with open(p, "w") as f:
        f.write("# OBJ file\n")
        f.write("\n".join("v %.6f %.6f %.6f" % tuple(v) for v in V) + "\n")
        f.write("\n".join("vn %.4f %.4f %.4f" % tuple(n) for n in N) + "\n")
        f.write("\n".join("f %d//%d %d//%d %d//%d" % (a+1, i+1, b+1, i+1, c+1, i+1) for i, (a, b, c) in enumerate(F)) + "\n")

def sharp_features(V, F, N, angle):
    m = len(F); n = len(V)
    E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    fid = np.tile(np.arange(m), 3); loc = np.repeat(np.arange(3), m)
    k = np.sort(E, 1); key = k[:, 0].astype(np.int64)*n + k[:, 1]
    o = np.argsort(key, kind="stable"); key = key[o]
    same = np.where(key[1:] == key[:-1])[0]
    out = []
    for s in same:
        a, b = o[s], o[s+1]; fa, fb = fid[a], fid[b]
        ang = np.degrees(np.arccos(np.clip(np.dot(N[fa], N[fb]), -1, 1)))
        if ang > angle:
            # convex if the far vertex of fb lies behind fa's plane
            ev = set(E[a]); opp = [v for v in F[fb] if v not in ev]
            convex = 1 if np.dot(N[fa], V[opp[0]] - V[E[a][0]]) < 0 else 0
            out.append("%d,%d,%d" % (convex, fa, loc[a]))
    return out

def sharp_features_masked(V, F, N, angle, C, tol):
    from scipy.spatial import cKDTree
    tr = cKDTree(C)
    m = len(F); n = len(V)
    E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    fid = np.tile(np.arange(m), 3); loc = np.repeat(np.arange(3), m)
    k = np.sort(E, 1); key = k[:, 0].astype(np.int64)*n + k[:, 1]
    o = np.argsort(key, kind="stable"); key = key[o]
    same = np.where(key[1:] == key[:-1])[0]
    out = []
    for s_ in same:
        a, b = o[s_], o[s_+1]; fa, fb = fid[a], fid[b]
        ang = np.degrees(np.arccos(np.clip(np.dot(N[fa], N[fb]), -1, 1)))
        if ang > angle:
            mid = 0.5*(V[E[a][0]] + V[E[a][1]])
            if tr.query(mid)[0] <= tol:
                ev = set(E[a]); opp = [v for v in F[fb] if v not in ev]
                convex = 1 if np.dot(N[fa], V[opp[0]] - V[E[a][0]]) < 0 else 0
                out.append("%d,%d,%d" % (convex, fa, loc[a]))
    return out

def count_faces(p):
    q = t = 0
    for ln in open(p):
        if ln.startswith("f "):
            k = len(ln.split())-1
            if k == 4: q += 1
            elif k == 3: t += 1
    return q, t

name = sys.argv[1]
sharp_angle = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
time_limit = int(sys.argv[3]) if len(sys.argv) > 3 else 60
scale = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
featfile = sys.argv[5] if len(sys.argv) > 5 else None
feat_tol = float(sys.argv[6]) if len(sys.argv) > 6 else 5.0
tag = (name if scale == 1.0 else "%s_s%g" % (name, scale)) + ("_feat" if featfile else "")
wd = BASE + "remesh_out/%s/" % tag; os.makedirs(wd, exist_ok=True)
V, F = read_obj(BASE + "remesh_in/%s.obj" % name)
a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
N = np.cross(b-a, c-a); N /= np.maximum(np.linalg.norm(N, axis=1), 1e-12)[:, None]
mesh_path = wd + tag + ".obj"
write_obj(mesh_path, V, F, N)
qw = qlib.Quadwild(mesh_path)
t0 = time.time()
use_sharp = sharp_angle > 0
if use_sharp:
    sf = sharp_features_masked(V, F, N, sharp_angle, np.load(featfile), feat_tol) if featfile else sharp_features(V, F, N, sharp_angle)
    with open(qw.sharp_path, "w") as f: f.write("%d\n" % len(sf)); f.write("\n".join(sf) + ("\n" if sf else ""))
    print("sharp edges:", len(sf), flush=True)
qw.remeshAndField(remesh=True, enableSharp=use_sharp, sharpAngle=sharp_angle)
print("remeshAndField done %.1fs" % (time.time()-t0), flush=True)
print("trace:", qw.trace(), "%.1fs" % (time.time()-t0), flush=True)
r = qw.quadrangulate(True, scale, 0, 0.005, "LEASTSQUARES", time_limit, 0.0, 0.4, True, True, True, 0.9, True, 0.1, True, False, False, True, True,
                     "SIMPLE", "DEFAULT", [3.0, 5.0, 10.0, 20.0, 30.0, 60.0, 90.0, 120.0], [0.005, 0.02, 0.05, 0.10, 0.15, 0.20, 0.25, 0.3])
print("quadrangulate returned", r, "%.1fs" % (time.time()-t0), flush=True)
for p in (qw.output_path, qw.output_smoothed_path):
    if os.path.exists(p):
        q, t = count_faces(p)
        print("OUT", os.path.basename(p), "quads", q, "tris", t, "quad%%=%.1f" % (100.0*q/max(q+t, 1)), flush=True)
    else:
        print("MISSING", os.path.basename(p), flush=True)
