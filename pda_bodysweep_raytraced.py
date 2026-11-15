# -*- coding: utf-8 -*-
"""
Ray-Traced Body Sweep  (Işın İzlemeli Body Sweep)
=================================================

Sweeps a solid tool body along a path (follow-path RMF or Z-up, optional twist)
and returns the swept volume as a clean BRep solid, ready to cut other solids.

Method
------
1. Tool silhouette (seen along the path) -> template loop with fixed point count.
   Sharp corners become creases; each corner gets two extra columns at +/- eps.
2. Section planes along the path (dense at both ends, ramped linearly):
   every plane is cut by the tool copies that cross it, the 2D union of those cuts
   (shapely) is the true section of the swept volume = the outermost line.
3. Ray tracing: each template point is mapped into every section by a ray along
   the template normal, smoothed over +/- R (corners act as if rounded by R), so rays
   never cross before the surface shift caused by rotation/twist.  The outermost hit
   is taken.  Prediction uses the frame of the body whose centre sits on the plane.
4. Quad cage (rows x template points) -> one B-spline surface per strip between
   creases: interpolating across the section, SubD-like control net along the path.
5. Planar caps close the cage; the tool bodies at the start and end are fused on,
   which brings back the real start / end caps (the 'shadow' faces) by themselves.

Usage in FreeCAD
----------------
Select: tool solid, path (edges / wire / sketch), then any solids to cut. Then:

    import pda_bodysweep_raytraced as rbs, importlib; importlib.reload(rbs)
    rbs.run_selection(twist_deg=0, mode="rmf", clearance=0.0)

The tool must sit at the start of the path as it should start the sweep.
"""
import math
import time

import FreeCAD as App
import Part
from FreeCAD import Vector as V

try:
    from shapely.geometry import Polygon, LineString, Point
    from shapely.ops import unary_union
    HAVE_SHAPELY = True
except ImportError:  # pragma: no cover
    HAVE_SHAPELY = False

__version__ = "0.1.0"


# --------------------------------------------------------------------- path frames
class PathFrames:
    """Equal-arc-length samples of the path with RMF (or Z-up) frames and twist.
    s < 0 and s > L are straight extensions along the end tangents."""

    def __init__(self, path_shape, tool_center, step=0.5, mode="rmf",
                 twist_deg=0.0, up=V(0, 0, 1)):
        edges = Part.__sortEdges__(path_shape.Edges)
        wire = Part.Wire(edges)
        self.L = wire.Length
        n = max(60, int(math.ceil(self.L / step)))
        pts = wire.discretize(Number=n + 1)
        if (pts[-1] - tool_center).Length < (pts[0] - tool_center).Length:
            pts = pts[::-1]                      # path starts at the tool
        self.n = n
        self.ds = self.L / n
        self.P = pts
        T = []
        for i in range(n + 1):
            a = pts[max(0, i - 1)]
            b = pts[min(n, i + 1)]
            t = b - a
            t.normalize()
            T.append(t)
        self.T = T
        R = []
        if mode == "zup":
            prev = None
            for t in T:
                r = up - t * up.dot(t)
                if r.Length < 1e-6:
                    r = prev if prev is not None else V(1, 0, 0)
                r.normalize()
                R.append(r)
                prev = r
        else:                                   # rotation-minimizing (double reflection)
            r0 = up - T[0] * up.dot(T[0])
            if r0.Length < 1e-6:
                r0 = V(1, 0, 0) - T[0] * T[0].x
            r0.normalize()
            R = [r0]
            for i in range(n):
                v1 = pts[i + 1] - pts[i]
                c1 = v1.dot(v1)
                if c1 < 1e-18:
                    R.append(R[-1]); continue
                rL = R[i] - v1 * (2 / c1 * v1.dot(R[i]))
                tL = T[i] - v1 * (2 / c1 * v1.dot(T[i]))
                v2 = T[i + 1] - tL
                c2 = v2.dot(v2)
                r = rL - v2 * (2 / c2 * v2.dot(rL)) if c2 > 1e-12 else rL
                r = r - T[i + 1] * r.dot(T[i + 1])
                r.normalize()
                R.append(r)
        self.R = R
        self.twist = math.radians(twist_deg)

    def frame(self, s):
        """(origin, t, r, b) at arc length s; right-handed (t, b, r)."""
        if s <= 0:
            i, ext = 0, s
        elif s >= self.L:
            i, ext = self.n, s - self.L
        else:
            i, ext = int(round(s / self.ds)), 0.0
        t = self.T[i]
        r = self.R[i]
        b = r.cross(t)
        th = self.twist * min(max(s, 0.0), self.L) / self.L
        if th:
            r = r * math.cos(th) + b * math.sin(th)
            b = r.cross(t)
        if ext:
            o = self.P[i] + t * ext
        elif 0 < s < self.L:
            f = s / self.ds
            i0 = int(math.floor(f)); i1 = min(self.n, i0 + 1); w = f - i0
            o = self.P[i0] * (1 - w) + self.P[i1] * w
        else:
            o = self.P[i]
        return o, t, r, b

    def placement(self, s):
        o, t, r, b = self.frame(s)
        return App.Placement(App.Matrix(t.x, b.x, r.x, o.x,
                                        t.y, b.y, r.y, o.y,
                                        t.z, b.z, r.z, o.z,
                                        0, 0, 0, 1))

    def max_curvature(self):
        k = 0.0
        for i in range(1, self.n):
            k = max(k, (self.T[i + 1] - self.T[i - 1]).Length / (2 * self.ds))
        return k


# --------------------------------------------------------------- 2D helpers
def _poly_from_wire(w, to2d, defl):
    pts = w.discretize(Deflection=defl)
    pg = Polygon([to2d(p) for p in pts])
    if not pg.is_valid:
        pg = pg.buffer(0)
    return pg


def _largest(geom):
    if geom.geom_type == "MultiPolygon":
        return max(geom.geoms, key=lambda g: g.area)
    return geom


def _ccw(coords):
    a = 0.0
    for k in range(len(coords)):
        y0, z0 = coords[k]; y1, z1 = coords[(k + 1) % len(coords)]
        a += y0 * z1 - y1 * z0
    return a > 0


def _resample(coords, step):
    """Closed polyline -> dense equal-step points + cumulative length."""
    pts = list(coords)
    if pts[0] == pts[-1]:
        pts = pts[:-1]
    out = []; acc = []
    s = 0.0
    n = len(pts)
    for k in range(n):
        y0, z0 = pts[k]; y1, z1 = pts[(k + 1) % n]
        seg = math.hypot(y1 - y0, z1 - z0)
        m = max(1, int(math.ceil(seg / step)))
        for q in range(m):
            f = q / m
            out.append((y0 + (y1 - y0) * f, z0 + (z1 - z0) * f))
            acc.append(s + seg * f)
        s += seg
    return out, acc, s


def build_template(coords, spacing, eps, corner_deg=20.0):
    """Template points on a closed 2D outline: fixed count, corners (creases) kept,
    two extra columns at +/- eps around each corner.  Returns (pts2d, corner_idx)."""
    dense, acc, Lt = _resample(coords, min(0.05, spacing / 10))
    n = len(dense)
    win = max(2, int(round(0.3 / (Lt / n))))

    def ang(k):
        a = dense[(k - win) % n]; b = dense[k]; c = dense[(k + win) % n]
        d1 = (b[0] - a[0], b[1] - a[1]); d2 = (c[0] - b[0], c[1] - b[1])
        l1 = math.hypot(*d1); l2 = math.hypot(*d2)
        if l1 < 1e-9 or l2 < 1e-9:
            return 0.0
        cs = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (l1 * l2)))
        return math.degrees(math.acos(cs))

    cand = sorted(((ang(k), k) for k in range(n)), reverse=True)
    corners = []
    for a, k in cand:
        if a < corner_deg:
            break
        if all(min(abs(acc[k] - acc[c]), Lt - abs(acc[k] - acc[c])) > 4 * eps + 0.5
               for c in corners):
            corners.append(k)
    corners.sort(key=lambda k: acc[k])

    def at(sv):
        sv %= Lt
        lo, hi = 0, n - 1
        while lo < hi:                         # binary search on acc
            mid = (lo + hi + 1) // 2
            if acc[mid] <= sv: lo = mid
            else: hi = mid - 1
        return dense[lo]

    if not corners:                            # fully smooth outline
        m = max(8, int(round(Lt / spacing)))
        return [at(Lt * q / m) for q in range(m)], []
    svals = []; cidx = []
    for ci, c in enumerate(corners):
        s0 = acc[c]; s1 = acc[corners[(ci + 1) % len(corners)]]
        if s1 <= s0:
            s1 += Lt
        seg = s1 - s0
        cidx.append(len(svals))
        svals.append(s0)
        if seg > 3 * eps:
            m = max(2, int(round((seg - 2 * eps) / spacing)))
            svals.append(s0 + eps)
            for q in range(1, m):
                svals.append(s0 + eps + (seg - 2 * eps) * q / m)
            svals.append(s1 - eps)
        else:
            svals.append(s0 + seg / 2)
    return [at(x) for x in svals], cidx


def smooth_rays(pts2d, R):
    """Ray direction per template point: length-weighted mean of the outward edge
    normals within +/- R of arc length (as if corners were rounded by R)."""
    n = len(pts2d)
    sgn = 1.0 if _ccw(pts2d) else -1.0
    segs = []; acc = [0.0]
    for k in range(n):
        a = pts2d[k]; b = pts2d[(k + 1) % n]
        dy, dz = b[0] - a[0], b[1] - a[1]
        l = math.hypot(dy, dz)
        nn = (dz * sgn / l, -dy * sgn / l) if l > 1e-12 else (0.0, 0.0)
        segs.append((acc[-1] + l / 2, l, nn))
        acc.append(acc[-1] + l)
    Lt = acc[-1]
    out = []
    for k in range(n):
        sk = acc[k]; ay = az = 0.0
        for sm, l, nn in segs:
            d = abs(sm - sk); d = min(d, Lt - d)
            if d < R:
                w = l * (1 - d / R); ay += nn[0] * w; az += nn[1] * w
        ln = math.hypot(ay, az) or 1.0
        out.append((ay / ln, az / ln))
    return out


def _clamped(n, p):
    return [float(x) for x in range(n - p + 1)], [p + 1] + [1] * (n - p - 1) + [p + 1]


# ------------------------------------------------------------------ main builder
def build(tool, path, twist_deg=0.0, mode="rmf", clearance=0.0, fuse_ends=True,
          copy_step=None, path_step=None, log=print):
    """tool: Part.Shape (solid, placed at the path start); path: Part.Shape with edges.
    Returns (solid, info)."""
    if not HAVE_SHAPELY:
        raise RuntimeError("shapely is required (pip install shapely in FreeCAD's Python)")
    t0 = time.time()
    info = {"warnings": []}
    tool = tool.copy()
    bbw = tool.BoundBox
    size = max(bbw.XLength, bbw.YLength, bbw.ZLength)
    pf = PathFrames(path, tool.BoundBox.Center, step=path_step or min(0.5, size / 40),
                    mode=mode, twist_deg=twist_deg)
    L = pf.L
    pl0inv = pf.placement(0).inverse()
    tl = tool.copy()
    tl.Placement = pl0inv.multiply(tool.Placement)      # tool in path-local coords
    bb = tl.BoundBox
    blen = bb.XLength

    # ---- silhouette along the path direction (local x)
    defl = max(0.005, size / 2000)
    slabs = []; polys = []
    nsl = 80
    for q in range(nsl + 1):
        x = bb.XMin + 1e-3 + (blen - 2e-3) * q / nsl
        ps = [_poly_from_wire(w, lambda p: (p.y, p.z), defl)
              for w in tl.slice(V(1, 0, 0), x) if w.isClosed()]
        u = unary_union(ps) if ps else None
        slabs.append((x, u.area if u is not None else 0.0))
        if u is not None and not u.is_empty:
            polys.append(u)
    sil = _largest(unary_union(polys))
    amax = max(a for x, a in slabs)
    big = [x for x, a in slabs if a > 0.9 * amax]
    xc, xf, xb = sum(big) / len(big), max(big), min(big)
    xmid = (bb.XMin + bb.XMax) / 2
    if clearance > 0:
        sil = sil.buffer(clearance, join_style=2, mitre_limit=4.0)
    sil = sil.simplify(defl / 4)
    coords = list(sil.exterior.coords)
    perim = sil.exterior.length
    spacing = perim / 48.0
    eps = min(0.04 * size, spacing * 0.5)
    tmpl2, sharp = build_template(coords, spacing, eps)
    nT = len(tmpl2)
    rmax = max(math.hypot(y, z) for y, z in tmpl2)

    # ---- automatic parameters
    copy_step = copy_step or max(0.05, min(2.0, blen / 12))
    kmax = pf.max_curvature()
    hmax = 0.46 * blen
    if pf.twist:
        hmax = min(hmax, math.radians(6) * L / abs(pf.twist))
    if kmax > 1e-9:
        hmax = min(hmax, math.radians(5) / kmax)
    hmin = min(hmax, blen / 5.2)
    ramp = 2 * blen
    rot_body = abs(pf.twist) / L * blen + kmax * blen
    shift = rmax * math.sin(min(rot_body, math.pi / 2) / 2)
    Rsm = max(2.0 * spacing, 3.0 * shift, 0.15 * size)
    back = max(1.5 * spacing, 2.0 * shift) + clearance
    fwd = max(3.0 * spacing, 4.0 * shift) + clearance
    info.update(L=L, template=nT, creases=len(sharp), copy_step=copy_step,
                hmin=hmin, hmax=hmax, smooth_R=Rsm, shift=shift)

    # ---- tool copies (shape + sample points), section planes
    pts_local = [V(*p) for p in tl.tessellate(defl * 20)[0]]
    ncp = max(2, int(math.ceil(L / copy_step)))
    copies = []
    for q in range(ncp + 1):
        s = L * q / ncp
        pl = pf.placement(s)
        sh = tl.copy(); sh.Placement = pl.multiply(tl.Placement)
        copies.append((s, sh, [pl.multVec(p) for p in pts_local]))

    # cage runs from the back of the start body's widest slab (so the backs of the
    # following copies are inside the cage) to just inside the front of the end body
    s_lo, s_hi = xb, L + xf
    pos = [s_lo]; s = s_lo
    while True:
        h = hmin + (hmax - hmin) * min(1.0, min(s - s_lo, s_hi - s) / ramp)
        s += h
        if s >= s_hi - hmin * 0.5:
            break
        pos.append(s)
    pos.append(s_hi)

    def section(s):
        o, t, r, b = pf.frame(s)
        d = o.dot(t)
        polys = []
        for sr, sh, sp in copies:
            if abs((sr + xmid) - s) > blen / 2 + 2 * copy_step + 0.5 * size:   # coarse prefilter
                continue
            ds = [p.dot(t) - d for p in sp]
            if min(ds) < -1e-6 and max(ds) > 1e-6:
                for w in sh.slice(t, d):
                    if w.isClosed():
                        polys.append(_poly_from_wire(
                            w, lambda p: ((p - o).dot(b), (p - o).dot(r)), defl))
        if not polys:
            return None
        u = _largest(unary_union(polys))
        if clearance > 0:
            u = u.buffer(clearance, join_style=2, mitre_limit=4.0)
        return o, t, r, b, u

    secs = []; secs_s = []
    for s in pos:
        x = section(s)
        if x is not None:
            secs.append(x); secs_s.append(s)
    t_sec = time.time() - t0

    # ---- ray tracing: template -> every section
    def trace(sec, s, rays):
        o, t, r, b, poly = sec
        _, _, rp, bp = pf.frame(s - xc)                      # body centred on plane
        rp = rp - t * rp.dot(t); rp.normalize(); bp = rp.cross(t)
        # prediction basis expressed in plane 2D coords
        by = (bp.dot(b), bp.dot(r)); ry = (rp.dot(b), rp.dot(r))
        ext = poly.exterior
        row = []; miss = 0
        for (py, pz), (dy, dz) in zip(tmpl2, rays):
            oy = by[0] * py + ry[0] * pz; oz = by[1] * py + ry[1] * pz
            ny = by[0] * dy + ry[0] * dz; nz = by[1] * dy + ry[1] * dz
            seg = LineString([(oy - ny * back, oz - nz * back), (oy + ny * fwd, oz + nz * fwd)])
            hit = seg.intersection(ext)
            cand = []
            if not hit.is_empty:
                geoms = getattr(hit, "geoms", [hit])
                for gm in geoms:
                    cand += list(gm.coords)
            if cand:
                hy, hz = max(cand, key=lambda c: (c[0] - oy) * ny + (c[1] - oz) * nz)
            else:
                miss += 1
                pnt = ext.interpolate(ext.project(Point(oy, oz)))
                hy, hz = pnt.x, pnt.y
            row.append(o + b * hy + r * hz)
        return row, miss

    def folds(rows):
        f = 0
        for row in rows:
            for k in range(nT):
                d1 = row[k] - row[k - 1]; d2 = row[(k + 1) % nT] - row[k]
                if d1.Length > 1e-7 and d2.Length > 1e-7 and d1.getAngle(d2) > math.radians(120):
                    f += 1
        return f

    for attempt in range(4):
        rays = smooth_rays(tmpl2, Rsm)
        res = [trace(sec, s, rays) for sec, s in zip(secs, secs_s)]
        rows = [r for r, m in res]
        nmiss = sum(m for r, m in res)
        nfold = folds(rows)
        if nfold == 0 and nmiss == 0:
            break
        Rsm *= 1.5; back *= 1.3; fwd *= 1.3
    info.update(smooth_R=Rsm, folds=nfold, misses=nmiss, sections=len(rows))
    if nfold or nmiss:
        info["warnings"].append("cage has %d folds / %d ray misses" % (nfold, nmiss))

    # ---- surfaces: interpolate across, SubD-like control net along the path
    if sharp:
        chains = []
        for k in range(len(sharp)):
            a = sharp[k]; b_ = sharp[(k + 1) % len(sharp)]
            if b_ <= a:
                b_ += nT
            chains.append([c % nT for c in range(a, b_ + 1)])
    else:
        chains = None
    faces = []
    nu = len(rows); du = min(3, nu - 1)
    uk, um = _clamped(nu, du)
    if chains:
        for cols in chains:
            prm = [0.0]
            for q in range(1, len(cols)):
                A = tmpl2[cols[q]]; B = tmpl2[cols[q - 1]]
                prm.append(prm[-1] + math.hypot(A[0] - B[0], A[1] - B[1]))
            prm = [x / prm[-1] for x in prm]
            curves = []
            for row in rows:
                c = Part.BSplineCurve()
                c.interpolate(Points=[row[q] for q in cols], Parameters=prm)
                curves.append(c)
            srf = Part.BSplineSurface()
            srf.buildFromPolesMultsKnots([c.getPoles() for c in curves], um,
                                         curves[0].getMultiplicities(), uk,
                                         curves[0].getKnots(), False, False, du,
                                         curves[0].Degree)
            faces.append(srf)
    else:                                       # smooth closed outline: periodic across
        curves = []
        for row in rows:
            c = Part.BSplineCurve(); c.interpolate(Points=row, PeriodicFlag=True); curves.append(c)
        srf = Part.BSplineSurface()
        srf.buildFromPolesMultsKnots([c.getPoles() for c in curves], um,
                                     curves[0].getMultiplicities(), uk,
                                     curves[0].getKnots(), False, True, du,
                                     curves[0].Degree)
        faces.append(srf)

    def capface(first):
        eds = []
        for sf in faces:
            u0, u1, v0, v1 = sf.bounds()
            eds.append(sf.uIso(u0 if first else u1).toShape())
        return Part.Face(Part.Wire(Part.__sortEdges__(eds)))

    shell = Part.Shell([Part.Face(sf) for sf in faces] + [capface(True), capface(False)])
    shell.sewShape(0.05 * size / 20)
    if not shell.isClosed():
        info["warnings"].append("cage shell not closed")
    cage = Part.Solid(Part.Shell(shell.Faces))
    if cage.Volume < 0:
        cage.reverse()
    cage.fix(1e-4, 1e-5, 1e-3)
    result = cage
    if fuse_ends:
        # only the parts of the start / end bodies that lie outside the cage are added:
        # behind the first section plane and beyond the last one.  They touch the cage
        # only on its planar caps, which keeps the boolean robust (no near-coincident
        # B-spline / body faces).
        ends = []
        H = 4 * size + 10
        for s_body, s_plane, behind in ((0.0, secs_s[0], True), (L, secs_s[-1], False)):
            pl = pf.placement(s_body)
            e = tl.copy(); e.Placement = pl.multiply(tl.Placement)
            if clearance > 0:
                try:
                    e = e.makeOffsetShape(clearance, 1e-3, join=2)
                except Exception:
                    info["warnings"].append("offset of end body failed; end body used as is")
            o, t, r, b = pf.frame(s_plane)
            hp = Part.makeBox(H, H, H, V(-H if behind else 0, -H / 2, -H / 2))
            hp.Placement = App.Placement(App.Matrix(t.x, b.x, r.x, o.x, t.y, b.y, r.y, o.y,
                                                    t.z, b.z, r.z, o.z, 0, 0, 0, 1)
                                         ).multiply(hp.Placement)
            piece = e.common(hp)
            if piece.Solids:
                reach = max(abs((v.Point - o).dot(t)) for v in piece.Vertexes)
                reach = max(reach, max(abs((V(*p) - o).dot(t))
                                       for p in piece.tessellate(defl * 20)[0]))
                if reach > 0.05 * blen:          # thinner slivers are covered by the cage
                    ends.append(piece)
        if not ends:
            ends = None
        # exact fuse first: the fuzzy one was seen to drop parts of the end bodies
        fu = None if ends else cage
        for fz in ((0.0, 1e-3, 1e-2) if ends else ()):
            try:
                f = cage.fuse(ends, fz) if fz else cage.fuse(ends)
            except Exception:
                continue
            if f.isValid() and len(f.Solids) == 1:
                fu = f
                break
            fu = fu or f
        try:
            rs = fu.removeSplitter()
            if rs.isValid() and abs(rs.Volume - fu.Volume) < 1e-4 * fu.Volume:
                fu = rs
        except Exception:
            pass
        if len(fu.Solids) == 1:
            result = fu.Solids[0]
        else:
            info["warnings"].append("fusing the end pieces failed (%d solids); "
                                    "returning cage + pieces as a compound" % len(fu.Solids))
            result = Part.makeCompound([cage] + (ends or []))
    info.update(valid=result.isValid(), faces=len(result.Faces), cage=(len(rows), nT),
                t_sections=round(t_sec, 2), t_total=round(time.time() - t0, 2))
    info["_rows"] = rows
    info["_frames"] = pf
    return result, info


# ------------------------------------------------------------------ helpers
def cage_lines(info):
    rows = info["_rows"]; nT = len(rows[0])
    return Part.makeCompound([Part.makePolygon(r + [r[0]]) for r in rows] +
                             [Part.makePolygon([r[k] for r in rows]) for k in range(nT)])


def coverage(tool, path, solid, info, ncopies=10, every=5):
    """Max distance (mm) of tool surface samples that fall outside the sweep."""
    pf = info["_frames"]
    pl0inv = pf.placement(0).inverse()
    tl = tool.copy(); tl.Placement = pl0inv.multiply(tool.Placement)
    pts = [V(*p) for p in tl.tessellate(0.5)[0]][::every]
    worst = 0.0
    for q in range(ncopies + 1):
        pl = pf.placement(pf.L * q / ncopies)
        for p in pts:
            w = pl.multVec(p)
            if not solid.isInside(w, 1e-3, True):
                worst = max(worst, solid.distToShape(Part.Vertex(w))[0])
    return worst


def run_selection(twist_deg=0.0, mode="rmf", clearance=0.0, fuse_ends=True, show_cage=False):
    """Selection: tool solid, path (edges/wire/sketch), [solids to cut ...]."""
    import FreeCADGui as Gui
    sel = Gui.Selection.getSelectionEx()
    if len(sel) < 2:
        raise RuntimeError("Select the tool solid, then the path, then (optional) solids to cut")
    tool = sel[0].Object.Shape
    pobj = sel[1]
    path = Part.Compound([e for e in pobj.SubObjects if isinstance(e, Part.Edge)]) \
        if pobj.SubObjects else pobj.Object.Shape
    targets = [s.Object for s in sel[2:]]
    doc = App.ActiveDocument
    solid, info = build(tool, path, twist_deg=twist_deg, mode=mode,
                        clearance=clearance, fuse_ends=fuse_ends)
    o = doc.addObject("Part::Feature", "RayTracedSweep")
    o.Shape = solid
    if show_cage:
        c = doc.addObject("Part::Feature", "RayTracedSweep_Cage"); c.Shape = cage_lines(info)
    for t in targets:
        cut = t.Shape.cut(solid, 1e-3)
        r = doc.addObject("Part::Feature", t.Label + "_cut"); r.Shape = cut
        t.ViewObject.Visibility = False
    o.ViewObject.Visibility = not targets
    doc.recompute()
    App.Console.PrintMessage("RayTracedSweep: %s\n" % {k: v for k, v in info.items()
                                                       if not k.startswith("_")})
    return o, info
