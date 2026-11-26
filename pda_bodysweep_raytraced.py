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
        # kapali yol: bas ve son ayni nokta; yol aletin oldugu yerden baslar, frame(s) L ile periyodik
        self.closed = wire.isClosed() or (pts[0] - pts[-1]).Length < 1e-6 * max(self.L, 1.0)
        if self.closed:
            k = min(range(n), key=lambda i: (pts[i] - tool_center).Length)
            pts = pts[k:n] + pts[:k] + [pts[k]]
        elif (pts[-1] - tool_center).Length < (pts[0] - tool_center).Length:
            pts = pts[::-1]                      # path starts at the tool
        self.n = n
        self.ds = self.L / n
        self.P = pts
        T = []
        for i in range(n + 1):
            if self.closed:
                a = pts[(i - 1) % n]
                b = pts[(i + 1) % n]
            else:
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
            if self.closed:
                # RMF bir turda kapanmaz (holonomi): son cerceve ilkine gore phi donmus. Fark yol boyunca dagitilir,
                # yoksa kafesin son satiri ilkine burulmus baglanir.
                t0 = T[0]
                rn = R[n] - t0 * R[n].dot(t0)
                if rn.Length > 1e-9:
                    rn.normalize()
                    phi = math.atan2(rn.cross(R[0]).dot(t0), rn.dot(R[0]))
                    for i in range(1, n + 1):
                        a = phi * i / n
                        t = T[i]; b = t.cross(R[i])
                        r = R[i] * math.cos(a) + b * math.sin(a)
                        r = r - t * r.dot(t); r.normalize()
                        R[i] = r
        self.R = R
        self.twist = math.radians(twist_deg)

    def frame(self, s):
        """(origin, t, r, b) at arc length s; right-handed (t, b, r)."""
        if self.closed:
            s = s % self.L
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
        for i in range(0 if self.closed else 1, self.n):
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
def _build_impl(tool, path, twist_deg=0.0, mode="rmf", clearance=0.0, fuse_ends=True,
          copy_step=None, path_step=None, log=print, caps="fuse", ncols=None):
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

    # round tools: sample arcs finer, so the cage and the exact cap edges (arcs) agree closely
    if ncols is None:
        ncols = 96 if (caps == "shadow" and any(f.Surface.TypeId != "Part::GeomPlane" for f in tl.Faces)) else 48
    defl, xc, xf, xb, xmid, spacing, eps, tmpl2, sharp = _tool_template(tl, bb, blen, size, clearance, ncols)
    nT = len(tmpl2)
    rmax = max(math.hypot(y, z) for y, z in tmpl2)

    # ---- shadow caps (caps="shadow"): faces not seen from the first / last section plane
    shadow = None
    closed = pf.closed
    if closed:                  # kapali yol: kapak yok, ara bolgenin islemi butun tur boyunca
        caps = "none"
        fuse_ends = False
        info["closed_path"] = True
    if caps == "shadow":
        try:
            tlc = tl
            if clearance > 0:
                tlc = tl.makeOffsetShape(clearance, 1e-3, join=2)
            fs, ps = shadow_cap(tlc, False, size, log)
            fe, pe = shadow_cap(tlc, True, size, log)
            if fs is None or fe is None:
                raise RuntimeError("no usable shadow cap")
            nmax = max(8, int(0.7 * nT))
            shadow = (fs, _decimate_loop(ps, nmax), fe, _decimate_loop(pe, nmax), ps, pe)
        except Exception as ex:
            info["warnings"].append("shadow caps failed (%s); using fused end bodies" % ex)
            shadow = None

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
    for q in range(ncp if closed else ncp + 1):     # kapali: s = L, s = 0 ile ayni kopya
        s = L * q / ncp
        pl = pf.placement(s)
        sh = tl.copy(); sh.Placement = pl.multiply(tl.Placement)
        copies.append((s, sh, [pl.multVec(p) for p in pts_local]))

    # cage runs from the back of the start body's widest slab (so the backs of the
    # following copies are inside the cage) to just inside the front of the end body
    s_lo, s_hi = xb, L + xf
    if shadow:                      # the cage runs between the two silhouette loops
        s_lo = max(p.x for p in shadow[1]) + 1e-3 * size
        s_hi = L + min(p.x for p in shadow[3]) - 1e-3 * size
        if s_hi - s_lo < 3 * hmin:
            info["warnings"].append("path too short for shadow caps; using fused end bodies")
            shadow = None
            s_lo, s_hi = xb, L + xf
    pos = [s_lo]; s = s_lo
    while True:
        h = hmin + (hmax - hmin) * min(1.0, min(s - s_lo, s_hi - s) / ramp)
        s += h
        if s >= s_hi - hmin * 0.5:
            break
        pos.append(s)
    pos.append(s_hi)
    if closed:                  # esit aralikli kesitler, ara bolgenin en buyuk adimiyla, uc sikilastirmasi yok
        nsec = max(8, int(math.ceil(L / hmax)))
        pos = [L * q / nsec for q in range(nsec)]

    def section(s):
        o, t, r, b = pf.frame(s)
        d = o.dot(t)
        polys = []
        for sr, sh, sp in copies:
            dsr = (sr + xmid) - s
            if closed:
                dsr = (dsr + L / 2) % L - L / 2
            if abs(dsr) > blen / 2 + 2 * copy_step + 0.5 * size:   # coarse prefilter
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
        return _trace(sec, s, rays, pf, xc, tmpl2, back, fwd)

    def folds(rows):
        return _folds(rows, nT)

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

    cap_faces = None
    if shadow:
        pl0 = pf.placement(0); plL = pf.placement(L)
        cols_s, pts_s = _assign_vertices(tmpl2, shadow[1])
        cols_e, pts_e = _assign_vertices(tmpl2, shadow[3])
        sharp = sorted(set(sharp) | set(cols_s) | set(cols_e))
        r0 = _row_from_loop(tmpl2, pts_s, cols_s, shadow[4])
        rN = _row_from_loop(tmpl2, pts_e, cols_e, shadow[5])
        try:        # spread the cap-loop points so the longitudinal curves do not bunch up
            r0 = _spread_row(r0, [pl0.inverse().multVec(q) for q in rows[0]], sharp, shadow[4], cols_s)
            rN = _spread_row(rN, [plL.inverse().multVec(q) for q in rows[-1]], sharp, shadow[5], cols_e)
        except Exception as ex:
            info["warnings"].append("cap row spreading skipped (%s)" % ex)
        row0 = [pl0.multVec(q) for q in r0]
        rowN = [plL.multVec(q) for q in rN]
        mid = []
        try:    # rows between the last traced section and the end cap loop follow the path's frames
            sA = secs_s[-1]
            if L - sA > hmin:
                plA_inv = pf.placement(sA).inverse(); plL_inv = plL.inverse()
                A_loc = [plA_inv.multVec(q) for q in rows[-1]]
                B_loc = [plL_inv.multVec(q) for q in rowN]
                nmid = max(2, min(6, int(round((L - sA) / max(hmax, 1e-6)))))
                for k in range(1, nmid + 1):
                    f = k / (nmid + 1.0)
                    plk = pf.placement(sA + f * (L - sA))
                    mid.append([plk.multVec(A_loc[i] * (1.0 - f) + B_loc[i] * f) for i in range(len(A_loc))])
        except Exception as ex:
            info["warnings"].append("end span rows skipped (%s)" % ex)
            mid = []
        rows = [row0] + rows + mid + [rowN]
        cap_faces = []
        for fcs, pl in ((shadow[0], pl0), (shadow[2], plL)):
            for f in fcs:
                g = f.copy()
                g.Placement = pl.multiply(f.Placement)
                cap_faces.append(g)
        info["shadow_caps"] = (len(shadow[0]), len(shadow[2]))
    if closed and len(rows) != len(pos):
        info["warnings"].append("closed path: %d of %d sections are empty; cage not closed" % (len(pos) - len(rows), len(pos)))
    cage = _cage_solid(rows, tmpl2, sharp, nT, size, info, cap_faces, periodic=closed)
    result = cage
    if fuse_ends and not shadow:
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
            except Exception as ex_:
                info["warnings"].append("fusing the end pieces (fuzz %g) raised: %s" % (fz, str(ex_)[:80]))
                continue
            if f.isValid() and len(f.Solids) == 1:
                fu = f
                break
            fu = fu or f
        if fu is None and ends:             # every all-at-once fuse raised: try piece by piece
            try:
                f = cage
                for pc in ends:
                    f = f.fuse(pc, 1e-3)
                fu = f
            except Exception as ex_:
                info["warnings"].append("fusing the end pieces one by one raised: %s" % str(ex_)[:80])
        if fu is None:
            raise RuntimeError("the end pieces could not be fused onto the cage (%s)" % "; ".join(info["warnings"][-3:]))
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


def run_selection(twist_deg=0.0, mode="rmf", clearance=0.0, fuse_ends=True, show_cage=False,
                  method="standard"):
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
    try:
        solid, info = build(tool, path, twist_deg=twist_deg, mode=mode,
                            clearance=clearance, fuse_ends=fuse_ends, method=method)
    except BodySweepError as ex:
        App.Console.PrintError(str(ex) + "\n")
        try:
            from PySide import QtGui
            QtGui.QMessageBox.warning(None, "Body Sweep (deneysel)", FAIL_MESSAGE)
        except Exception:
            pass
        return None
    for w in info["warnings"]:
        App.Console.PrintWarning("Body Sweep: " + w + "\n")
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


# ------------------------------------------------ helpers lifted out of build()
def _tool_template(tl, bb, blen, size, clearance, ncols=48):
    """Tool silhouette along the path (local x) -> (defl, xc, xf, xb, xmid, spacing, eps, tmpl2, sharp)."""
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
    spacing = perim / float(ncols)
    eps = min(0.04 * size, spacing * 0.5)
    tmpl2, sharp = build_template(coords, spacing, eps)
    return defl, xc, xf, xb, xmid, spacing, eps, tmpl2, sharp


def _trace(sec, s, rays, pf, xc, tmpl2, back, fwd):
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


def _folds(rows, nT):
    f = 0
    for row in rows:
        for k in range(nT):
            d1 = row[k] - row[k - 1]; d2 = row[(k + 1) % nT] - row[k]
            if d1.Length > 1e-7 and d2.Length > 1e-7 and d1.getAngle(d2) > math.radians(120):
                f += 1
    return f


def _cage_solid(rows, tmpl2, sharp, nT, size, info, cap_faces=None, periodic=False):
    """Quad cage (rows x template) -> closed BRep solid. periodic: kapali yol, son satir ilkine baglanir (halka),
    kapak yuzu yok."""
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
    if periodic:                # nu kutup, nu + 1 dugum, katlilik 1: u yonunde periyodik
        uk, um = [float(x) for x in range(nu + 1)], [1] * (nu + 1)
    if chains:
        for cols in chains:
            prm = [0.0]
            for q in range(1, len(cols)):
                A = tmpl2[cols[q]]; B = tmpl2[cols[q - 1]]
                prm.append(prm[-1] + math.hypot(A[0] - B[0], A[1] - B[1]))
            prm = [x / prm[-1] for x in prm]
            use_rows = rows
            if cap_faces is not None and len(rows) >= 5:
                use_rows = _free_exit_rows(rows)
            curves = []
            for row in use_rows:
                c = Part.BSplineCurve()
                c.interpolate(Points=_nudge([row[q] for q in cols], size), Parameters=prm)
                curves.append(c)
            srf = Part.BSplineSurface()
            srf.buildFromPolesMultsKnots([c.getPoles() for c in curves], um,
                                         curves[0].getMultiplicities(), uk,
                                         curves[0].getKnots(), periodic, False, du,
                                         curves[0].Degree)
            faces.append(srf)
    else:                                       # smooth closed outline: periodic across
        curves = []
        for row in rows:
            c = Part.BSplineCurve(); c.interpolate(Points=row, PeriodicFlag=True); curves.append(c)
        srf = Part.BSplineSurface()
        srf.buildFromPolesMultsKnots([c.getPoles() for c in curves], um,
                                     curves[0].getMultiplicities(), uk,
                                     curves[0].getKnots(), periodic, True, du,
                                     curves[0].Degree)
        faces.append(srf)

    def capface(first):
        eds = []
        for sf in faces:
            u0, u1, v0, v1 = sf.bounds()
            eds.append(sf.uIso(u0 if first else u1).toShape())
        return Part.Face(Part.Wire(Part.__sortEdges__(eds)))

    if periodic:
        shell = Part.Shell([Part.Face(sf) for sf in faces])
    elif cap_faces is not None:
        allf = [Part.Face(sf) for sf in faces] + list(cap_faces)
        try:
            from BOPTools import JoinAPI
            allf = JoinAPI.connect(allf, tolerance=2e-3 * size).Faces
        except Exception as ex:
            info["warnings"].append("face connect failed: %s" % ex)
        shell = Part.Shell(allf)
    else:
        shell = Part.Shell([Part.Face(sf) for sf in faces] + [capface(True), capface(False)])
    shell.sewShape(0.05 * size / 20)
    if not shell.isClosed() and cap_faces is not None:
        base = list(allf)
        for mul in (4, 20, 100):
            sh2 = Part.Shell(base)
            sh2.sewShape(mul * 0.05 * size / 20)
            if sh2.isClosed():
                shell = sh2
                break
    if not shell.isClosed():
        info["warnings"].append("cage shell not closed")
        try:
            fe = [e for e in shell.Edges if len(shell.ancestorsOfType(e, Part.Face)) < 2]
            info["free_edges"] = [(round(e.Length, 2), tuple(round(x, 1) for x in e.CenterOfMass)) for e in fe]
            info["free_detail"] = []
            for e in fe:
                a, b = e.Vertexes[0].Point, e.Vertexes[-1].Point
                near = []
                for e2 in shell.Edges:
                    if e2.isSame(e):
                        continue
                    da = e2.distToShape(Part.Vertex(a))[0] if hasattr(e2, "distToShape") else -1
                    db = e2.distToShape(Part.Vertex(b))[0]
                    if da < 0.5 or db < 0.5:
                        near.append((round(e2.Length, 2), round(da, 3), round(db, 3), len(shell.ancestorsOfType(e2, Part.Face))))
                info["free_detail"].append((round(e.Length, 2), [tuple(round(x, 2) for x in a), tuple(round(x, 2) for x in b)], near[:8]))
        except Exception as ex:
            info["free_edges"] = str(ex)
    cage = Part.Solid(Part.Shell(shell.Faces))
    if cage.Volume < 0:
        cage.reverse()
    try:
        cage.fix(1e-4, 1e-5, 1e-3)
    except Exception as ex:
        info["warnings"].append("cage not closed: solid repair failed (%s)" % ex)
        try:
            cage = Part.Solid(shell)
        except Exception:
            pass
    return cage



# ================================================================== slab silhouette method
# "Slab siluet" yöntemi: no caps / no rays.  The swept range is cut into slabs between
# neighbouring section planes (dense at both ends, smooth ramp).  For every slab the
# part of all tool copies inside it is projected onto the slab mid plane; the outer
# silhouette (holes ignored) is that row of the cage.  The first / last rows sit on the
# object's own start / end extreme, so the cage covers the whole object from its
# beginning.  Where the object starts behind / ends beyond the path, the path frames
# continue as straight tangent extensions (PathFrames.frame with s < 0 or s > L).
def _smoothstep(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def _np3(v):
    import numpy as np
    return np.array([v.x, v.y, v.z])


def _clip_poly(poly, n, c, ge):
    sg = 1.0 if ge else -1.0
    out = []
    m = len(poly)
    for i in range(m):
        p = poly[i]; q = poly[(i + 1) % m]
        dp = sg * (p.dot(n) - c); dq = sg * (q.dot(n) - c)
        if dp >= 0:
            out.append(p)
        if (dp >= 0) != (dq >= 0):
            out.append(p + (q - p) * (dp / (dp - dq)))
    return out


def _tool_copies_mesh(tl, pf, s_list, tol):
    """Tessellated tool placed at every s: W (ncopies, nverts, 3), triangle idx T, centres, radii."""
    import numpy as np
    verts, tris = tl.tessellate(tol)
    P = np.array([[v.x, v.y, v.z] for v in verts])
    T = np.array(tris, dtype=int)
    W = np.empty((len(s_list), len(P), 3))
    for i, s in enumerate(s_list):
        m = pf.placement(s).Matrix
        A = np.array([[m.A11, m.A12, m.A13], [m.A21, m.A22, m.A23], [m.A31, m.A32, m.A33]])
        W[i] = P @ A.T + np.array([m.A14, m.A24, m.A34])
    cen = (W.min(1) + W.max(1)) / 2
    rad = np.linalg.norm(W - cen[:, None, :], axis=2).max(1)
    return W, T, cen, rad


def _slab_silhouette(W, T, cen, rad, f0, f1, fm, tol):
    """Outer silhouette (shapely Polygon, coords (b, r) of frame fm) of everything lying
    between the planes of frames f0 and f1."""
    import numpy as np
    import shapely
    t0 = _np3(f0[1]); t1 = _np3(f1[1])
    c0 = _np3(f0[0]) @ t0; c1 = _np3(f1[0]) @ t1
    om = _np3(fm[0]); rm = _np3(fm[2]); bm = _np3(fm[3])
    sel = np.nonzero((cen @ t0 - c0 > -rad) & (cen @ t1 - c1 < rad))[0]
    if not len(sel):
        return None
    Ws = W[sel]
    d0t = (Ws @ t0 - c0)[:, T]
    d1t = (Ws @ t1 - c1)[:, T]
    keep = ~((d0t < 0).all(2) | (d1t > 0).all(2))
    ci, ti = np.nonzero(keep)
    if not len(ci):
        return None
    tri = Ws[ci[:, None], T[ti]]
    inside = (d0t[ci, ti] >= 0).all(1) & (d1t[ci, ti] <= 0).all(1)

    def to2(X):
        Y = X - om
        return np.stack([Y @ bm, Y @ rm], -1)

    geoms = []
    ins = to2(tri[inside])
    if len(ins):
        ar = 0.5 * np.abs((ins[:, 1, 0] - ins[:, 0, 0]) * (ins[:, 2, 1] - ins[:, 0, 1])
                          - (ins[:, 2, 0] - ins[:, 0, 0]) * (ins[:, 1, 1] - ins[:, 0, 1]))
        ins = ins[ar > 1e-10]
        if len(ins):
            geoms += list(shapely.polygons(np.concatenate([ins, ins[:, :1]], 1)))
    for k in np.nonzero(~inside)[0]:
        poly = [tri[k, 0], tri[k, 1], tri[k, 2]]
        poly = _clip_poly(poly, t0, c0, True)
        if len(poly) >= 3:
            poly = _clip_poly(poly, t1, c1, False)
        if len(poly) >= 3:
            pg = Polygon([tuple(q) for q in to2(np.array(poly))])
            if pg.is_valid and pg.area > 1e-10:
                geoms.append(pg)
    if not geoms:
        return None
    u = _largest(shapely.union_all(geoms))
    u = Polygon(u.exterior)                      # holes ignored
    return u.buffer(tol, join_style=2, mitre_limit=4.0) if tol > 0 else u


def _radial_row(sec, s, pf, xc, tmpl2, ccw, ref=None):
    """Fallback row for slabs whose silhouette is too different from the template (tilted
    tip: rays miss or collapse): each template point keeps its angle around the template
    centre and is cast from the silhouette centre to the outermost outline hit."""
    o, t, r, b, poly = sec
    _, _, rp, bp = pf.frame(s - xc)
    rp = rp - t * rp.dot(t); rp.normalize(); bp = rp.cross(t)
    by = (bp.dot(b), bp.dot(r)); ry = (rp.dot(b), rp.dot(r))
    if ref is not None:                      # angles taken from the neighbouring row seen in this plane
        dirs = [((q - o).dot(b), (q - o).dot(r)) for q in ref]
        cy = sum(p[0] for p in dirs) / len(dirs); cz = sum(p[1] for p in dirs) / len(dirs)
        dirs = [(py - cy, pz - cz) for py, pz in dirs]
    else:
        cy = sum(p[0] for p in tmpl2) / len(tmpl2); cz = sum(p[1] for p in tmpl2) / len(tmpl2)
        dirs = None
    ext = poly.exterior
    c = poly.representative_point() if not poly.contains(poly.centroid) else poly.centroid
    big = 4.0 * max(poly.bounds[2] - poly.bounds[0], poly.bounds[3] - poly.bounds[1], 1e-6)
    row = []
    for k, (py, pz) in enumerate(tmpl2):
        if dirs is not None:
            ny, nz = dirs[k]
        else:
            dy, dz = py - cy, pz - cz
            ny = by[0] * dy + ry[0] * dz; nz = by[1] * dy + ry[1] * dz
        ln = math.hypot(ny, nz) or 1.0
        ny /= ln; nz /= ln
        hit = LineString([(c.x, c.y), (c.x + ny * big, c.y + nz * big)]).intersection(ext)
        cand = []
        if not hit.is_empty:
            for gm in getattr(hit, "geoms", [hit]):
                cand += list(gm.coords)
        if cand:
            hy, hz = max(cand, key=lambda q: (q[0] - c.x) ** 2 + (q[1] - c.y) ** 2)
        else:
            pnt = ext.interpolate(ext.project(c))
            hy, hz = pnt.x, pnt.y
        row.append(o + b * hy + r * hz)
    return row


def build_slab(tool, path, twist_deg=0.0, mode="rmf", clearance=0.0, copy_step=None,
               path_step=None, end_factor=3.0, log=print):
    """Slab-silhouette body sweep.  tool: Part.Shape (solid at the path start);
    path: Part.Shape with edges.  end_factor: slab density at the two ends
    (x times more slabs than the middle), ramped smoothly.  Returns (solid, info)."""
    if not HAVE_SHAPELY:
        raise RuntimeError("shapely is required (pip install shapely in FreeCAD's Python)")
    t0 = time.time()
    info = {"warnings": [], "method": "slab"}
    tool = tool.copy()
    bbw = tool.BoundBox
    size = max(bbw.XLength, bbw.YLength, bbw.ZLength)
    pf = PathFrames(path, tool.BoundBox.Center, step=path_step or min(0.5, size / 40),
                    mode=mode, twist_deg=twist_deg)
    L = pf.L
    tl = tool.copy()
    tl.Placement = pf.placement(0).inverse().multiply(tool.Placement)
    bb = tl.BoundBox
    blen = bb.XLength
    defl, xc, xf, xb, xmid, spacing, eps, tmpl2, sharp = _tool_template(tl, bb, blen, size, clearance)
    nT = len(tmpl2)
    rmax = max(math.hypot(y, z) for y, z in tmpl2)

    # ---- automatic parameters (same prediction / smoothing rules as the ray method)
    copy_step = copy_step or max(0.05, min(2.0, blen / 12))
    kmax = pf.max_curvature()
    hn = 0.46 * blen
    if pf.twist:
        hn = min(hn, math.radians(6) * L / abs(pf.twist))
    if kmax > 1e-9:
        hn = min(hn, math.radians(5) / kmax)
    he = hn / max(1.0, end_factor)
    ramp = 2 * blen
    rot_body = abs(pf.twist) / L * blen + kmax * blen
    shift = rmax * math.sin(min(rot_body, math.pi / 2) / 2)
    Rsm = max(2.0 * spacing, 3.0 * shift, 0.15 * size)
    back = max(1.5 * spacing, 2.0 * shift) + clearance
    fwd = max(3.0 * spacing, 4.0 * shift) + clearance

    # ---- swept range: the whole object, from its own back at the start to its own
    # front at the end (straight tangent extensions outside the path), slabs dense at both ends
    s_lo, s_hi = bb.XMin, L + bb.XMax
    edges = [s_lo]
    s = s_lo
    while True:
        d = min(s - s_lo, s_hi - s)
        s += he + (hn - he) * _smoothstep((d - blen) / ramp)
        if s >= s_hi - he * 0.5:
            break
        edges.append(s)
    edges.append(s_hi)
    info.update(L=L, template=nT, creases=len(sharp), copy_step=copy_step, h_mid=hn,
                h_end=he, smooth_R=Rsm, shift=shift, ext_start=max(0.0, -s_lo),
                ext_end=max(0.0, s_hi - L - 0.0))

    # ---- tool copies as meshes
    ncp = max(2, int(math.ceil(L / copy_step)))
    s_list = [L * q / ncp for q in range(ncp + 1)]
    W, T, cen, rad = _tool_copies_mesh(tl, pf, s_list, defl * 5)

    # ---- one silhouette per slab, row placed at the mid plane (first / last row moved
    # onto the object's own start / end)
    secs, secs_s, dts = [], [], []
    for i in range(len(edges) - 1):
        a, b_ = edges[i], edges[i + 1]
        mid = 0.5 * (a + b_)
        fm = pf.frame(mid)
        poly = _slab_silhouette(W, T, cen, rad, pf.frame(a), pf.frame(b_), fm, defl * 5)
        if poly is None:
            continue
        if clearance > 0:
            poly = poly.buffer(clearance, join_style=2, mitre_limit=4.0)
        secs.append((fm[0], fm[1], fm[2], fm[3], poly))
        secs_s.append(mid)
        dts.append(a - mid if i == 0 else (b_ - mid if i == len(edges) - 2 else 0.0))
    if len(secs) < 3:
        raise RuntimeError("slab sweep: too few slabs (%d)" % len(secs))
    t_sec = time.time() - t0

    def trace(sec, s, rays):
        return _trace(sec, s, rays, pf, xc, tmpl2, back, fwd)

    for attempt in range(4):
        rays = smooth_rays(tmpl2, Rsm)
        res = [trace(sec, s, rays) for sec, s in zip(secs, secs_s)]
        rows = [r for r, m in res]
        tiny = 1e-4 * size
        nfix = 0
        bad = {i for i, (row, m) in enumerate(res)
               if m > 0 or min((row[k] - row[k - 1]).Length for k in range(nT)) < tiny}
        todo = set(bad)
        while todo:                          # grow outwards from the good rows (keeps continuity)
            prog = False
            for i in sorted(todo):
                nb = [j for j in (i - 1, i + 1) if 0 <= j < len(rows) and j not in todo]
                if nb or len(todo) == len(rows):
                    ref = rows[nb[0]] if nb else None
                    rows[i] = _radial_row(secs[i], secs_s[i], pf, xc, tmpl2, True, ref)
                    todo.discard(i); nfix += 1; prog = True
                    break
            if not prog:
                break
        nmiss = 0
        nfold = _folds(rows, nT)
        if nfold == 0 and nmiss == 0:
            break
        Rsm *= 1.5; back *= 1.3; fwd *= 1.3
    for row, sec, dt in zip(rows, secs, dts):
        if dt:
            tv = sec[1] * dt
            for k in range(len(row)):
                row[k] = row[k] + tv
    info.update(smooth_R=Rsm, folds=nfold, misses=nmiss, sections=len(rows))
    if nfold or nmiss:
        info["warnings"].append("cage has %d folds / %d ray misses" % (nfold, nmiss))

    info["respread_rows"] = nfix
    cage = _cage_solid(rows, tmpl2, sharp, nT, size, info)
    info.update(valid=cage.isValid(), faces=len(cage.Faces), cage=(len(rows), nT),
                t_sections=round(t_sec, 2), t_total=round(time.time() - t0, 2))
    info["_rows"] = rows
    info["_frames"] = pf
    info["_edges"] = edges
    return cage, info


def run_selection_slab(twist_deg=0.0, mode="rmf", clearance=0.0, end_factor=3.0, show_cage=False):
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
    solid, info = build_slab(tool, path, twist_deg=twist_deg, mode=mode,
                             clearance=clearance, end_factor=end_factor)
    o = doc.addObject("Part::Feature", "SlabSweep")
    o.Shape = solid
    if show_cage:
        c = doc.addObject("Part::Feature", "SlabSweep_Cage"); c.Shape = cage_lines(info)
    for t in targets:
        cut = t.Shape.cut(solid, 1e-3)
        r = doc.addObject("Part::Feature", t.Label + "_cut"); r.Shape = cut
        t.ViewObject.Visibility = False
    o.ViewObject.Visibility = not targets
    doc.recompute()
    App.Console.PrintMessage("SlabSweep: %s\n" % {k: v for k, v in info.items()
                                                  if not k.startswith("_")})
    return o, info


# ============================================================ shadow-cap method (kapak: gölge yüzler)
# Start cap = the tool faces that are NOT seen from the first section plane (outward normal
# against the path direction); faces seen from the plane and faces parallel to the view
# direction are deleted.  The silhouette loop of the kept faces is the first row of the cage.
# End cap: mirrored (faces whose normal points along the path direction).
def _cap_faces(tl, sg, size):
    """sg=-1: faces facing against +x (start cap); sg=+1: facing along +x (end cap).
    Planar faces are taken whole, curved ones are split at the shadow line by a tiny shift."""
    D = 2e-4 * size
    eps = 1e-6
    # a planar face that is (nearly) parallel to the view direction projects to a line: both of its
    # edges would land on top of each other in the cap loop and the sweep lines would cross there.
    # Faces within this angle of the view direction count as parallel and are not part of the cap.
    eps_plane = math.sin(math.radians(1.0))
    kept, curved = [], []
    for f in tl.Faces:
        u0, u1, v0, v1 = f.ParameterRange
        if f.Surface.TypeId == "Part::GeomPlane":
            if sg * f.normalAt((u0 + u1) / 2, (v0 + v1) / 2).x > eps_plane:
                kept.append(f)
        elif f.Surface.TypeId == "Part::GeomSphere":
            # the silhouette of a sphere is its great circle perpendicular to the view: cut exactly
            # through the centre and keep the half that faces away from the viewer
            cx = f.Surface.Center.x
            big = 10.0 * size
            if sg < 0:      # keep x <= cx
                halfspace = Part.makeBox(big, 2 * big, 2 * big, V(cx, -big, -big))
            else:           # keep x >= cx
                halfspace = Part.makeBox(big, 2 * big, 2 * big, V(cx - big, -big, -big))
            try:
                kept.extend(f.cut(halfspace).Faces)
            except Exception:
                curved.append(f)
        elif f.Surface.TypeId == "Part::GeomCone":
            pieces = _split_cone_at_silhouette(f, size)
            if pieces is None:
                curved.append(f)
            else:
                for g in pieces:
                    try:
                        uu, vv = g.Surface.parameter(g.CenterOfMass)
                        nx = g.normalAt(uu, vv).x
                    except Exception:
                        continue
                    if sg * nx > eps:
                        kept.append(g)
        elif f.Surface.TypeId == "Part::GeomCylinder":
            # silhouette generators of a cylinder lie in the plane through its axis that is
            # perpendicular to the view direction's component across the axis
            a = V(f.Surface.Axis); a.normalize()
            w = V(1, 0, 0) - a * a.x
            if w.Length < 1e-6:                 # axis along the view: no lateral face is kept
                continue
            w.normalize()
            c = V(f.Surface.Center)
            away = w * (-sg)                    # the removed half lies on this side of the plane
            big = 10.0 * size
            box = Part.makeBox(big, 2 * big, 2 * big, V(0, -big, -big))
            box.Placement = App.Placement(c, App.Rotation(V(1, 0, 0), away))
            try:
                kept.extend(f.cut(box).Faces)
            except Exception:
                curved.append(f)
        else:
            curved.append(f)
    if curved:
        moved = tl.copy()
        moved.translate(V(-sg * D, 0, 0))
        for f in tl.Shells[0].cut(moved).Faces:
            if f.Surface.TypeId == "Part::GeomPlane":
                continue
            # a split piece can span the whole parameter range, so its parametric middle may lie in
            # the other half: read the normal where the piece itself is (projected centre of mass)
            try:
                uu, vv = f.Surface.parameter(f.CenterOfMass)
                nx = f.normalAt(uu, vv).x
            except Exception:
                u0, u1, v0, v1 = f.ParameterRange
                nx = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2).x
            if sg * nx > eps:
                kept.append(f)
    return kept


def _loop_polyline(faces, tolsew):
    """Sew the faces; return (shell, pts3d closed list without repeat) or (shell, None) if the
    free edges do not form exactly one closed loop."""
    fs_ = faces
    if len(faces) > 1 and any(f.Surface.TypeId != "Part::GeomPlane" for f in faces):
        # edges shared along only part of their length (stacked round faces whose silhouette
        # lines differ) must be split first, or they never pair up
        try:
            from BOPTools import JoinAPI
            fs_ = JoinAPI.connect(list(faces), tolerance=max(tolsew, 1e-6)).Faces
        except Exception:
            fs_ = faces
    sh = Part.Shell(fs_)
    sh.sewShape(tolsew)
    free = [e for e in sh.Edges if len(sh.ancestorsOfType(e, Part.Face)) == 1]
    # slivers left where the shifted copy split a curved face are dangling spurs, not loop edges
    free = [e for e in free if e.Length > 8 * tolsew]

    def _seam(e):               # a periodic face's seam edge is shared with itself, not a loop edge
        try:
            return any(e.isSeam(f) for f in sh.ancestorsOfType(e, Part.Face))
        except Exception:
            return False
    free = [e for e in free if not _seam(e)]
    if not free:
        return sh, None
    wires = Part.sortEdges(free)
    if len(wires) != 1:
        return sh, None
    w = Part.Wire(wires[0])
    if not w.isClosed():
        return sh, None
    qs = []
    for e in w.OrderedEdges:
        qs.append(e.discretize(Number=2) if e.Curve.TypeId == "Part::GeomLine" else e.discretize(Deflection=0.02))
    if len(qs) > 1:
        # orient the first edge so that its end touches the second edge (its own direction is arbitrary)
        nxt = (qs[1][0], qs[1][-1])
        if min((qs[0][0] - a).Length for a in nxt) < min((qs[0][-1] - a).Length for a in nxt):
            qs[0] = qs[0][::-1]
    pts = []
    end = None
    for q in qs:
        if end is not None and (end - q[0]).Length > (end - q[-1]).Length:
            q = q[::-1]
        pts += q[:-1]
        end = q[-1]
    # (every edge contributes its start; the closing vertex equals the first point)
    clean = []
    for p_ in pts:                      # drop repeated vertices (zero-length edges / seams)
        if not clean or (p_ - clean[-1]).Length > 1e-6 * max(1.0, tolsew * 1e4):
            clean.append(p_)
    while len(clean) > 1 and (clean[0] - clean[-1]).Length <= 1e-6 * max(1.0, tolsew * 1e4):
        clean.pop()
    return sh, clean


def _crossings(pts):
    """number of self-intersections of the (y, z) projection of a closed 3D polyline."""
    from shapely.geometry import LineString
    n = len(pts)
    segs = [LineString([(pts[k].y, pts[k].z), (pts[(k + 1) % n].y, pts[(k + 1) % n].z)]) for k in range(n)]
    c = 0
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if segs[i].crosses(segs[j]):
                c += 1
    return c


def shadow_cap(tl, end, size, log=print):
    """Cap shell + silhouette loop (3D points in tool-local coords) for the start or end cap.
    Rear faces that make the loop cross itself in the view (concave corners) are dropped
    one by one (the cage reproduces them).  Returns (faces, pts) or (None, None)."""
    faces = _cap_faces(tl, +1.0 if end else -1.0, size)
    if not faces:
        return None, None
    tolsew = 1e-4 * size
    for it in range(12):
        sh, pts = _loop_polyline(faces, tolsew)
        if pts is None:        # the tiny shift used to split curved faces leaves ~D gaps: sew coarser
            for mul in (4, 16, 64):
                sh, pts = _loop_polyline(faces, mul * tolsew)
                if pts is not None:
                    tolsew = mul * tolsew
                    break
        if pts is None:
            log("shadow cap: loop not a single closed curve")
            return None, None
        nc = _crossings(pts)
        if nc == 0:
            return faces, pts
        big = max(range(len(faces)), key=lambda i: faces[i].Area)
        best = None
        for i in sorted(range(len(faces)), key=lambda i: faces[i].Area):
            if i == big:
                continue
            rest = faces[:i] + faces[i + 1:]
            sh2, p2 = _loop_polyline(rest, tolsew)
            if p2 is None:
                continue
            c2 = _crossings(p2)
            if c2 < nc and (best is None or c2 < best[0]):
                best = (c2, i)
        if best is None:
            log("shadow cap: cannot remove loop crossings")
            return None, None
        faces = faces[:best[1]] + faces[best[1] + 1:]
    return None, None


def _lift_to_loop(tmpl2, pts):
    """Template (y, z) points -> nearest point on the closed 3D loop (seen in projection)."""
    n = len(pts)
    out = []
    for (py, pz) in tmpl2:
        best = None
        for k in range(n):
            a = pts[k]; b = pts[(k + 1) % n]
            dy, dz = b.y - a.y, b.z - a.z
            l2 = dy * dy + dz * dz
            u = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((py - a.y) * dy + (pz - a.z) * dz) / l2))
            d2 = (a.y + u * dy - py) ** 2 + (a.z + u * dz - pz) ** 2
            if best is None or d2 < best[0]:
                best = (d2, a + (b - a) * u)
        out.append(best[1])
    return out


def _augment_template(tmpl2, sharp, verts2d, tol):
    """Insert a crease column at every loop vertex (projected) so that cage row 0 / last row
    can follow the cap loop edge by edge.  Returns (tmpl2, sharp)."""
    pts = [tuple(p) for p in tmpl2]
    cre = set(sharp)
    for (vy, vz) in verts2d:
        dmin, kmin = min((math.hypot(vy - p[0], vz - p[1]), k) for k, p in enumerate(pts))
        if dmin < tol:
            cre.add(kmin)
            continue
        n = len(pts)
        best = None
        for k in range(n):
            a = pts[k]; b = pts[(k + 1) % n]
            dy, dz = b[0] - a[0], b[1] - a[1]
            l2 = dy * dy + dz * dz
            u = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((vy - a[0]) * dy + (vz - a[1]) * dz) / l2))
            d2 = (a[0] + u * dy - vy) ** 2 + (a[1] + u * dz - vz) ** 2
            if best is None or d2 < best[0]:
                best = (d2, k)
        k = best[1]
        pts.insert(k + 1, (vy, vz))
        cre = {c + 1 if c > k else c for c in cre}
        cre.add(k + 1)
    return pts, sorted(cre)


def _lift_to_loop_mono(tmpl2, pts, verts_idx=None):
    """Lift template (y, z) points to the 3D loop: nearest loop point in projection, walking
    monotonically along the loop so that neighbouring points use neighbouring loop edges."""
    n = len(pts)
    proj = []
    for k in range(n):
        a = pts[k]; b = pts[(k + 1) % n]
        proj.append((a.y, a.z, b.y - a.y, b.z - a.z, a, b))

    def nearest_on(k, py, pz):
        ay, az, dy, dz, a, b = proj[k]
        l2 = dy * dy + dz * dz
        u = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((py - ay) * dy + (pz - az) * dz) / l2))
        d2 = (ay + u * dy - py) ** 2 + (az + u * dz - pz) ** 2
        return d2, u
    # start the walk at the globally best segment of the first point
    best = min(((nearest_on(k, *tmpl2[0])[0], k) for k in range(n)))
    cur = best[1]
    out = []
    for (py, pz) in tmpl2:
        cand = []
        for step in range(0, 4):
            k = (cur + step) % n
            d2, u = nearest_on(k, py, pz)
            cand.append((d2 + 1e-9 * step, k, u))
        d2, k, u = min(cand)
        # fall back to the global best if the local window is clearly worse
        g = min(((nearest_on(kk, py, pz)[0], kk) for kk in range(n)))
        if g[0] < 0.25 * d2 - 1e-9:
            k = g[1]; d2, u = nearest_on(k, py, pz)
        cur = k
        a, b = proj[k][4], proj[k][5]
        out.append(a + (b - a) * u)
    return out


def _assign_vertices(tmpl2, pts):
    """Monotone (cyclic) assignment of the closed loop's vertices to template columns, nearest
    in projection.  Returns (cols, pts_ordered) - the loop may be reversed to match the template
    direction."""
    n = len(tmpl2)
    best = None
    for direction in (1, -1):
        P = pts if direction == 1 else pts[::-1]
        m = len(P)
        if m > n - 1:
            continue
        V2 = [(p.y, p.z) for p in P]
        cost = [[(V2[j][0] - tmpl2[c][0]) ** 2 + (V2[j][1] - tmpl2[c][1]) ** 2 for c in range(n)]
                for j in range(m)]
        for c0 in sorted(range(n), key=lambda c: cost[0][c])[:6]:
            INF = 1e30
            dp = [[INF] * n for _ in range(m)]
            par = [[-1] * n for _ in range(m)]
            dp[0][0] = cost[0][c0]
            for j in range(1, m):
                run, arg = INF, -1
                for t in range(1, n):
                    if dp[j - 1][t - 1] < run:
                        run, arg = dp[j - 1][t - 1], t - 1
                    if run < INF:
                        dp[j][t] = run + cost[j][(c0 + t) % n]
                        par[j][t] = arg
            tb = min(range(1, n), key=lambda t: dp[m - 1][t])
            tot = dp[m - 1][tb]
            if tot < INF and (best is None or tot < best[0]):
                ts = [tb]
                for j in range(m - 1, 0, -1):
                    ts.append(par[j][ts[-1]])
                ts = ts[::-1]
                best = (tot, [(c0 + t) % n for t in ts], P)
    if best is None:
        raise RuntimeError("cannot assign loop vertices to the template")
    return best[1], best[2]


def _row_from_loop(tmpl2, pts, cols, dense=None):
    """Row of 3D points for all template columns lying on the loop: assigned columns sit on the
    loop vertices, the columns between two of them are spread along that loop edge by template
    arclength (so every cage chain on this row is exactly a loop edge)."""
    n = len(tmpl2); m = len(pts)
    row = [None] * n
    for j in range(m):
        ca = cols[j]; cb = cols[(j + 1) % m]
        A = pts[j]; B = pts[(j + 1) % m]
        idx = [ca]
        c = ca
        while c != cb:
            c = (c + 1) % n
            idx.append(c)
        cum = [0.0]
        for k in range(1, len(idx)):
            a = tmpl2[idx[k - 1]]; b = tmpl2[idx[k]]
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        tot = cum[-1] or 1.0
        path = _dense_path(dense, A, B) if dense else None
        for k, c in enumerate(idx):
            if path is None:
                row[c] = A + (B - A) * (cum[k] / tot)
            else:
                row[c] = _along(path, cum[k] / tot)
    return row


METHODS = ("fast", "standard", "coarse")      # selectable methods, each one falls back to the next
METHOD_LABELS = {"fast": "Hizli (golge kapak) - deneysel",
                 "standard": "Standart (fuse kapak)",
                 "coarse": "Kaba (az ornek, fuse kapak)"}
FAIL_MESSAGE = ("Body Sweep (experimental): this shape / path is outside the tool's limits, "
                "the operation could not be completed.")


class BodySweepError(RuntimeError):
    pass


class _Timeout(Exception):
    pass


def _run_limited(fn, seconds):
    """Run fn() and abort it with _Timeout after `seconds` (checked between Python lines, so a
    single long OCC call cannot be interrupted)."""
    import sys
    if not seconds:
        return fn()
    end = time.time() + seconds

    def tracer(frame, event, arg):
        if time.time() > end:
            raise _Timeout("time budget (%d s) exceeded" % seconds)
        return tracer
    old = sys.gettrace()
    sys.settrace(tracer)
    try:
        return fn()
    finally:
        sys.settrace(old)


def _good(sol, info, strict):
    bad = [w for w in info["warnings"] if "not closed" in w or "shadow caps failed" in w or "sections are empty" in w]
    return sol.isValid() and len(sol.Solids) == 1 and not (strict and bad)


def build(tool, path, *args, method="standard", budgets=None, **kw):
    """Selectable method with fall-back to the next one on failure:
    fast (shadow caps) -> standard (fused end bodies) -> coarse (fused, fewer samples).
    Raises BodySweepError (FAIL_MESSAGE) if even the last one fails.
    Returns (solid, info); info["method"] is the one that produced the result."""
    budgets = budgets or {"fast": 25, "standard": 180, "coarse": 120, "closed": 300}
    try:
        closed = Part.Wire(Part.__sortEdges__(path.Edges)).isClosed()
    except Exception:
        closed = False
    if closed:
        # kapali yol: bas ve son yok, kapak hesaplanmaz; ara bolgenin kesit + isin izleme kafesi butun tur boyunca
        # (periyodik). Hizli / standart / kaba ayrimi yalniz kapaklarda: tek deneme.
        try:
            curved = any(f.Surface.TypeId != "Part::GeomPlane" for f in tool.Faces)
            f = lambda: _build_impl(tool, path, *args, caps="none", ncols=96 if curved else 48, **kw)
            sol, info = _run_limited(f, budgets.get("closed"))
            if _good(sol, info, strict=True):
                info["method"] = "closed"
                return sol, info
            note = "; ".join(info["warnings"]) or "invalid result"
        except Exception as ex:
            note = str(ex)
        raise BodySweepError(FAIL_MESSAGE + " (closed path: " + note + ")")
    chain = METHODS[METHODS.index(method):]
    notes = []
    for m in chain:
        tries = ((None, 48) if m == "fast" else (None,))
        for nc in tries:
            try:
                if m == "fast":
                    f = lambda: _build_impl(tool, path, *args, caps="shadow", ncols=nc, **kw)
                elif m == "standard":
                    f = lambda: _build_impl(tool, path, *args, caps="fuse", **kw)
                else:
                    k2 = dict(kw)
                    size = max(tool.BoundBox.XLength, tool.BoundBox.YLength, tool.BoundBox.ZLength)
                    k2["copy_step"] = (kw.get("copy_step") or size / 10) * 2.0
                    k2["path_step"] = (kw.get("path_step") or min(0.5, size / 40)) * 2.0
                    f = lambda: _build_impl(tool, path, *args, caps="fuse", **k2)
                sol, info = _run_limited(f, budgets.get(m))
                if _good(sol, info, strict=(m == "fast")):
                    info["method"] = m
                    if notes:
                        info["warnings"].append("%s failed (%s); result made with: %s"
                                                % (METHOD_LABELS[method], "; ".join(notes),
                                                   METHOD_LABELS[m]))
                    return sol, info
                notes.append("%s: %s" % (m, "; ".join(info["warnings"]) or "invalid result"))
            except Exception as ex:
                notes.append("%s: %s" % (m, ex))
    raise BodySweepError(FAIL_MESSAGE + " (" + " | ".join(notes) + ")")


def _free_end_surface(rows, cols, prm):
    """Chain surface that passes through every cage point (rows x cols), with free (natural)
    ends along the path: the longitudinal curves leave the cap loop in their own direction
    instead of being forced along the chord to the next row."""
    nu = len(rows)
    acc = [0.0]
    for i in range(1, nu):
        acc.append(acc[-1] + sum((rows[i][c] - rows[i - 1][c]).Length for c in range(len(rows[i]))) / len(rows[i]))
    uprm = [a / acc[-1] for a in acc]
    ucur = []
    for c in cols:
        cu = Part.BSplineCurve()
        cu.interpolate(Points=[rows[i][c] for i in range(nu)], Parameters=uprm)
        ucur.append(cu)
    upoles = [cu.getPoles() for cu in ucur]
    vcur = []
    for i in range(len(upoles[0])):
        cv = Part.BSplineCurve()
        cv.interpolate(Points=[up[i] for up in upoles], Parameters=prm)
        vcur.append(cv)
    srf = Part.BSplineSurface()
    srf.buildFromPolesMultsKnots([cv.getPoles() for cv in vcur], ucur[0].getMultiplicities(),
                                 vcur[0].getMultiplicities(), ucur[0].getKnots(),
                                 vcur[0].getKnots(), False, False, ucur[0].Degree, vcur[0].Degree)
    return srf


def _free_exit_rows(rows):
    """Control rows for the cap-bounded cage: the second and second-to-last control row are moved
    onto the chord to the next row (keeping their distance), so the longitudinal curves leave the
    cap loop in the direction the sections continue instead of perpendicular to the cap (hook).
    The boundary rows themselves (cap loops) are untouched, so the shell still closes."""
    out = [list(r) for r in rows]
    n = len(rows)
    for a, b, c in ((0, 1, 2), (n - 1, n - 2, n - 3)):
        for q in range(len(rows[a])):
            P0, P1, P2 = rows[a][q], rows[b][q], rows[c][q]
            chord = P2 - P0
            if chord.Length < 1e-9:
                continue
            d = min((P1 - P0).Length, 0.5 * chord.Length)
            out[b][q] = P0 + chord * (d / chord.Length)
    return out


def _nudge(pts, size):
    """A cap-loop vertex shared by two template columns makes a zero-length chain span (coincident
    points), which B-spline interpolation refuses. Separate such points by a negligible amount."""
    eps = 1e-5 * size
    out = [pts[0]]
    for i in range(1, len(pts)):
        p = pts[i]
        if (p - out[-1]).Length < eps:
            ref = pts[i + 1] - p if i + 1 < len(pts) else None
            if ref is None or ref.Length < eps:
                ref = pts[0] - p if (pts[0] - p).Length >= eps else App.Vector(1, 0, 0)
            ref = App.Vector(ref)
            ref.normalize()
            p = out[-1] + ref * eps
        out.append(p)
    return out


def _decimate_loop(pts, nmax):
    """Thin a densely sampled closed loop (arcs) to at most ~nmax vertices.  Corners (turn > 20 deg)
    are always kept; the rest is spread evenly by arclength between them."""
    n = len(pts)
    if n <= nmax:
        return pts
    def turn(i):
        a = pts[i] - pts[i - 1]; b = pts[(i + 1) % n] - pts[i]
        if a.Length < 1e-12 or b.Length < 1e-12:
            return 0.0
        c = max(-1.0, min(1.0, a.dot(b) / (a.Length * b.Length)))
        return math.degrees(math.acos(c))
    keep = [i for i in range(n) if turn(i) > 20.0]
    if len(keep) < 2:
        keep = [0, n // 2]
    seg_len = []
    for k in range(len(keep)):
        i0, i1 = keep[k], keep[(k + 1) % len(keep)]
        idx = [i0]; j = i0
        while j != i1:
            j = (j + 1) % n; idx.append(j)
        seg_len.append((idx, sum((pts[idx[t + 1]] - pts[idx[t]]).Length for t in range(len(idx) - 1))))
    total = sum(l for _, l in seg_len) or 1.0
    budget = max(0, nmax - len(keep))
    out = []
    for idx, ln in seg_len:
        out.append(pts[idx[0]])
        cnt = min(len(idx) - 2, int(round(budget * ln / total)))
        if cnt > 0:
            cum = [0.0]
            for t in range(len(idx) - 1):
                cum.append(cum[-1] + (pts[idx[t + 1]] - pts[idx[t]]).Length)
            for q in range(1, cnt + 1):
                target = ln * q / (cnt + 1)
                t = min(range(1, len(idx) - 1), key=lambda t: abs(cum[t] - target))
                if pts[idx[t]] is not out[-1]:
                    out.append(pts[idx[t]])
    return out


def _dense_path(dense, A, B):
    """The stretch of the dense closed loop between its vertices A and B (the shorter way round)."""
    n = len(dense)
    ia = min(range(n), key=lambda i: (dense[i] - A).Length)
    ib = min(range(n), key=lambda i: (dense[i] - B).Length)
    fw = (ib - ia) % n
    bw = (ia - ib) % n
    if fw <= bw:
        return [dense[(ia + t) % n] for t in range(fw + 1)]
    return [dense[(ia - t) % n] for t in range(bw + 1)]


def _along(path, f):
    """Point at fraction f of the polyline's arclength."""
    if len(path) < 2:
        return path[0]
    cum = [0.0]
    for t in range(1, len(path)):
        cum.append(cum[-1] + (path[t] - path[t - 1]).Length)
    target = f * cum[-1]
    for t in range(1, len(path)):
        if cum[t] >= target - 1e-12:
            seg = cum[t] - cum[t - 1]
            u = 0.0 if seg < 1e-12 else (target - cum[t - 1]) / seg
            return path[t - 1] + (path[t] - path[t - 1]) * u
    return path[-1]


def _loop_param(dense, cum, P):
    """Arclength position along the closed dense loop of the point nearest to P."""
    best = None
    n = len(dense)
    for i in range(n):
        a = dense[i]; b = dense[(i + 1) % n]
        ab = b - a
        l2 = ab.dot(ab)
        u = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, (P - a).dot(ab) / l2))
        d = (a + ab * u - P).Length
        if best is None or d < best[0]:
            best = (d, cum[i] + u * math.sqrt(l2))
    return best[1]


def _spread_row(row, next_row, sharp, dense, fixed=()):
    """Redistribute the cap-loop row inside every crease-to-crease chain: the chain's two end
    columns stay, the columns between them are placed along the cap loop by the spacing of the
    next row (blended with uniform spacing).  Stops the longitudinal curves bunching into a fan."""
    n = len(row)
    m = len(dense)
    cum = [0.0]
    for i in range(m):
        cum.append(cum[-1] + (dense[(i + 1) % m] - dense[i]).Length)
    tot = cum[-1]
    if tot < 1e-9:
        return row
    def point_at(t):
        t %= tot
        lo, hi = 0, m
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if cum[mid] <= t:
                lo = mid
            else:
                hi = mid
        seg = cum[lo + 1] - cum[lo]
        u = 0.0 if seg < 1e-12 else (t - cum[lo]) / seg
        return dense[lo] + (dense[(lo + 1) % m] - dense[lo]) * u
    out = list(row)
    sh = sorted(set(sharp))
    for k in range(len(sh)):
        a, b = sh[k], sh[(k + 1) % len(sh)]
        idx = [a]; c = a
        while c != b:
            c = (c + 1) % n
            idx.append(c)
        if len(idx) < 3:
            continue
        ta = _loop_param(dense, cum, row[a]); tb = _loop_param(dense, cum, row[b])
        fwd = (tb - ta) % tot
        tmid = [_loop_param(dense, cum, row[c]) for c in idx[1:-1]]
        nfw = sum(1 for t in tmid if 0 < (t - ta) % tot < fwd + 1e-6)
        if nfw >= len(tmid) / 2.0:
            span, sgn = fwd, 1.0
        else:
            span, sgn = (ta - tb) % tot, -1.0
        if span < 1e-9:
            continue
        ch = [0.0]
        for q in range(1, len(idx)):
            ch.append(ch[-1] + (next_row[idx[q]] - next_row[idx[q - 1]]).Length)
        for q in range(1, len(idx) - 1):
            fu = q / (len(idx) - 1.0)
            fr = ch[q] / ch[-1] if ch[-1] > 1e-9 else fu
            f = 0.5 * fu + 0.5 * fr
            out[idx[q]] = point_at(ta + sgn * f * span)
    # two columns that land on the same loop point make their longitudinal curves meet there and
    # pinch a sliver between them: pull such columns apart along the loop
    eps = 1e-4 * tot
    fixed = set(fixed)
    i = 0
    while i < n:
        j = i
        while j + 1 < n and (out[j + 1] - out[i]).Length < eps:
            j += 1
        if j > i and not (i == 0 and j == n - 1):
            run = list(range(i, j + 1))
            anchor = next((c for c in run if c in fixed), run[0])
            nx = (j + 1) % n
            pm = (i - 1) % n
            ta = _loop_param(dense, cum, out[anchor])
            tn = _loop_param(dense, cum, out[nx])
            tp = _loop_param(dense, cum, out[pm])
            def toward(t0, t1):
                d = (t1 - t0) % tot
                return (d, 1.0) if d <= tot - d else (tot - d, -1.0)
            after = [c for c in run if c > anchor]
            before = [c for c in run if c < anchor]
            if after:
                d, sg_ = toward(ta, tn)
                for q, c in enumerate(after, 1):
                    out[c] = point_at(ta + sg_ * d * q / (len(after) + 1.0))
            if before:
                d, sg_ = toward(ta, tp)
                for q, c in enumerate(reversed(before), 1):
                    out[c] = point_at(ta + sg_ * d * q / (len(before) + 1.0))
        i = j + 1
    return out


def _split_cone_at_silhouette(f, size):
    """Split a conical face along its two silhouette generators (where the normal is perpendicular
    to the view direction x).  Returns the pieces, [f] if there is no silhouette, None on failure."""
    try:
        from BOPTools import SplitAPI
        surf = f.Surface
        sgn = -1.0 if f.Orientation == "Reversed" else 1.0
        u0, u1, v0, v1 = f.ParameterRange
        vm = 0.5 * (v0 + v1)

        def g(u):
            return sgn * surf.normal(u, vm).x
        N = 360
        us = [2 * math.pi * k / N for k in range(N + 1)]
        gs = [g(u) for u in us]
        roots = []
        for k in range(N):
            if gs[k] == 0.0 or gs[k] * gs[k + 1] < 0:
                a, b, ga = us[k], us[k + 1], gs[k]
                for _ in range(40):
                    m = 0.5 * (a + b); gm = g(m)
                    if ga * gm <= 0:
                        b = m
                    else:
                        a, ga = m, gm
                roots.append(0.5 * (a + b))
        if not roots:
            return [f]
        dv = 0.05 * (v1 - v0) + 1e-6
        edges = [Part.makeLine(surf.value(u, v0 - dv), surf.value(u, v1 + dv)) for u in roots]
        res = SplitAPI.slice(f, edges, "Standard", 1e-6 * size)
        pieces = [x for x in res.Faces if x.Surface.TypeId == "Part::GeomCone"]
        return pieces or None
    except Exception:
        return None
