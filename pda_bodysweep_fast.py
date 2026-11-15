# -*- coding: utf-8 -*-
# Pimp My FreeCAD - Body Sweep hizli yontem: mesh kopyalari + duzlem kesitleri + dis sinir + loft.
# Copyright (c) 2026 Sedat Kuran - MIT License
#
# 1. Alet mesh'e cevrilir ve yol boyunca (yolu takip et / dunya Z / destek yuzeyi, burulma) sik kopyalanir; kopyalar
#    tek mesh'te toplanir.
# 2. Yolun iki ucu aletin boyu kadar teget yonunde uzatilir; yola dik duzlemler aletin gercekten basladigi ve bittigi
#    yerden konur. Siklik doguusal: ucta 0.5 mm araliktan baslar, her kesitte sabit miktar buyur (ilk 30 mm'de 20 kesit),
#    ortadaki araliga varinca sabit kalir; sonda ayna.
# 3. Tek mesh butun duzlemlerle bir kerede kesilir. Her duzlemde, kesit noktalarinin ortasindan 128 yone isin gonderilir,
#    yalniz en distaki kesisim tutulur: kopyalarin icte kalan cizgileri kendiliginden duser (kesit o noktaya gore yildiz
#    bicimli olmali; ice girintili kesitte girinti dolu cikar).
# 4. Halkalar u yonunde periyodik kubik interpolasyonla, v yonunde veri noktalari kontrol noktasi olarak (tasmasiz
#    yumusak yaklasim: dondurulen aletin kopyalari kesitlerde ince dalga birakiyor, tam interpolasyon aralarinda
#    tasiyordu) tek B-spline yuzeye, iki duz kapakla katiya cevrilir.
# Olculen (FreeCAD 27.1, mesh hacmiyle): kola 180 derece burulma 200 kopya kesin yonteme -%0.19 (29 s, kesin 118 s),
# pahli kutu 3B yol 90 derece burulma -%0.47 (8 s, kesin 19 s); sinirdaki en buyuk sapma koseli aletin koselerinde
# ~0.3-0.6 mm. Not: bu yogun B-spline katida OCC'nin Volume degeri %5'e kadar yanlis cikabiliyor; hacmi agdan olcun.
import math
import time

import numpy as np

import FreeCAD as App
import Part

N_RAYS = 128
MAX_FACETS = 4.0e6          # butun kopyalarin toplam ucgen sayisi ust siniri


class Cancelled(Exception):
    pass


def _chain_points(edges, step=0.25):
    """Kenar zinciri -> sirali noktalar (bas -> son)."""
    w = Part.Wire(Part.__sortEdges__(list(edges)))
    return np.array([tuple(p) for p in w.discretize(Distance=step)])


def frames_for(pts, orient="path", ups=None, twist_deg=0.0):
    """Yol noktalari icin cerceveler: s, teget, yukari (burulmasiz), burulmali yukari."""
    import pda_bodysweep as bs
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    tg = np.gradient(pts, s, axis=0)
    tg /= np.linalg.norm(tg, axis=1, keepdims=True)
    if ups is None:
        if orient == "path":
            ups = bs.rmf_ups(pts, tg)
        else:
            ups = np.array([0, 0, 1.0]) - tg * tg[:, 2:3]
            if np.linalg.norm(ups, axis=1).min() < 1e-3:
                raise ValueError("the path is vertical somewhere: World Z cannot be the up direction, use Follow path")
    ups = ups - tg * np.sum(ups * tg, axis=1, keepdims=True)
    ups /= np.linalg.norm(ups, axis=1, keepdims=True)
    return s, tg, ups


class Motion(object):
    """Uzatilmis yol uzerinde konum / cerceve. s=0 asil yolun basi, s=L sonu; uzantilarda teget sabit."""

    def __init__(self, edges, orient, twist_deg, ext, support=None, start_hint=None):
        import pda_bodysweep as bs
        mid = _chain_points(edges)
        if start_hint is not None and np.linalg.norm(mid[-1] - start_hint) < np.linalg.norm(mid[0] - start_hint):
            mid = mid[::-1].copy()                         # yol aletin yanindaki ucundan baslar (kesin yontem gibi)
        self.L = float(np.sum(np.linalg.norm(np.diff(mid, axis=0), axis=1)))
        t0 = mid[1] - mid[0]
        t0 /= np.linalg.norm(t0)
        t1 = mid[-1] - mid[-2]
        t1 /= np.linalg.norm(t1)
        ne = max(2, int(ext / 0.25))
        pre = np.array([mid[0] - t0 * ext * (1 - f) for f in np.linspace(0, 1, ne, endpoint=False)])
        post = np.array([mid[-1] + t1 * ext * f for f in np.linspace(0, 1, ne + 1)[1:]])
        pts = np.r_[pre, mid, post]
        ups = None
        if support is not None:
            ups = bs.surface_ups(support, pts)
        self.s, self.tg, self.ups = frames_for(pts, orient, ups)
        self.s -= ext
        self.pts = pts
        self.twist = math.radians(twist_deg)

    def frame(self, sv, roll=True):
        s, pts, tg, ups = self.s, self.pts, self.tg, self.ups
        i = int(np.clip(np.searchsorted(s, sv), 1, len(s) - 1))
        f = (sv - s[i - 1]) / max(s[i] - s[i - 1], 1e-12)
        p = pts[i - 1] + (pts[i] - pts[i - 1]) * f
        t = tg[i - 1] + (tg[i] - tg[i - 1]) * f
        t /= np.linalg.norm(t)
        z = ups[i - 1] + (ups[i] - ups[i - 1]) * f
        z -= t * (t @ z)
        z /= np.linalg.norm(z)
        if roll and self.twist:
            th = self.twist * min(max(sv, 0.0), self.L) / self.L
            z = z * math.cos(th) + np.cross(t, z) * math.sin(th)
        return p, t, np.cross(z, t), z

    def matrix(self, sv, roll=True):
        p, t, y, z = self.frame(sv, roll)
        return App.Matrix(t[0], y[0], z[0], p[0], t[1], y[1], z[1], p[1], t[2], y[2], z[2], p[2], 0, 0, 0, 1)


def _ramp(start, sign, ds_mid, first_len=30.0, first_n=20, ds0=0.5):
    """Uctan iceri dogrusal buyuyen aralik: ilk first_len mm'de first_n kesit, ds_mid'e varinca durur."""
    dlt = (first_len - first_n * ds0) / (first_n * (first_n - 1) / 2.0)
    out = [start + sign * 0.05]
    k = 0
    while True:
        ds = min(ds0 + k * dlt, ds_mid)
        if ds >= ds_mid:
            break
        out.append(out[-1] + sign * ds)
        k += 1
    return np.array(out)


def _outer_ring(polys, p, u, v, rmax, n=N_RAYS):
    """Duzlemdeki butun cizgilerden yalniz en distakiler: noktalarin ortasindan n yone isin, en uzak kesisim."""
    segs = []
    for pl in polys:
        q = np.array([(a.x, a.y, a.z) for a in pl])
        if len(q) < 2:
            continue
        near = np.linalg.norm(q - p, axis=1) < rmax       # yolun baska yerinden gelen cizgiler atilir
        if not near.any():
            continue
        q2 = np.c_[(q - p) @ u, (q - p) @ v]
        segs.append(np.c_[q2[:-1], q2[1:]][near[:-1] | near[1:]])
    if not segs:
        return None
    S = np.vstack(segs)
    if not len(S):
        return None
    cc = np.r_[S[:, :2], S[:, 2:]].mean(0)
    A = S[:, :2] - cc
    D = S[:, 2:] - S[:, :2]
    ring = np.zeros((n, 2))
    for k, ang in enumerate(np.pi / 2 + np.linspace(0, 2 * np.pi, n, endpoint=False)):
        r = np.array([math.cos(ang), math.sin(ang)])
        den = r[0] * D[:, 1] - r[1] * D[:, 0]
        ok = np.abs(den) > 1e-12
        dd = np.where(ok, den, 1)
        t = (A[:, 0] * D[:, 1] - A[:, 1] * D[:, 0]) / dd
        w = (A[:, 0] * r[1] - A[:, 1] * r[0]) / dd
        m = ok & (t >= 0) & (w >= -1e-9) & (w <= 1 + 1e-9)
        ring[k] = cc + (r * t[m].max() if m.any() else 0.0)
    if np.abs(ring - cc).max() < 1e-6:
        return None
    return p + ring[:, :1] * u + ring[:, 1:] * v


_ubasis = {}


def _periodic_basis(nu, n):
    """Periyodik kubik B-spline (dugumler k/nu, katlik 1) tabani n esit aralikli noktada: (n, nu). OCCT'nin kendi
    egrisinden okunur (kutup siralamasi OCCT ile ayni olsun diye)."""
    key = (nu, n)
    if key not in _ubasis:
        B = np.zeros((n, nu))
        us = np.arange(n) / float(n)
        for j in range(nu):
            c = Part.BSplineCurve()
            c.buildFromPolesMultsKnots([App.Vector(1.0 if k == j else 0.0, 0, 0) for k in range(nu)], [1] * (nu + 1),
                                       [k / float(nu) for k in range(nu + 1)], True, 3)
            B[:, j] = [c.value(float(u)).x for u in us]
        _ubasis[key] = B
    return _ubasis[key]


def _vbasis(vp, mv):
    """Kenetli kubik taban (M, mv); ic dugumler kesitlerin sira esit dagilimina gore (sik kesitli uclarda sik dugum)."""
    from scipy.interpolate import BSpline
    inner = np.quantile(vp, np.linspace(0, 1, mv - 2)[1:-1]) if mv > 4 else np.array([])
    t = np.r_[[0.0] * 4, inner, [1.0] * 4]
    return BSpline.design_matrix(np.clip(vp, 0, 1 - 1e-12), t, 3).toarray(), t


def _loft(G, Vv, n=N_RAYS, tol=0.1):
    """Halkalar -> kati. u (kesit) yonunde derece 1: yuzey halka noktalarindan tam gecer, kose tasmasi yok (128 nokta
    30 mm yaricapta 0.01 mm kiris hatasi). v (yol) yonunde kubik en kucuk kareler: kontrol noktasi sayisi sapma
    tol'un altina inene dek artar. Ekranda ucgen sayisi aralik (span) sayisiyla buyudugu icin yuzey seyrek tutulur
    (128 x 330 kubik yuzey: 0.2 mm'de 3 milyon ucgen, 170 s)."""
    M = len(G)
    if M < 4:
        raise ValueError("too few sections")
    vp = (Vv - Vv[0]) / (Vv[-1] - Vv[0])
    best = None
    for mv in (M // 3, M // 2, (3 * M) // 4, M):
        mv = max(4, min(mv, M))
        Bv, t = _vbasis(vp, mv)
        P = np.linalg.lstsq(Bv, G.reshape(M, -1), rcond=None)[0].reshape(mv, n, 3)
        err = np.linalg.norm(np.einsum("mv,vua->mua", Bv, P) - G, axis=2).max()
        best = (P, t, mv, err)
        if err <= tol:
            break
    P, t, mv, err = best
    kv, km = np.unique(np.round(t, 12), return_counts=True)
    srf = Part.BSplineSurface()
    srf.buildFromPolesMultsKnots([[App.Vector(*P[j, k]) for j in range(mv)] for k in range(n)],
                                 [1] * (n + 1), km.tolist(), [k / float(n) for k in range(n + 1)], kv.tolist(),
                                 True, False, 1, 3)

    def cap(vv):
        w = Part.Wire(srf.vIso(vv).toShape())
        P_ = np.array([tuple(p) for p in w.discretize(97)[:96]])
        c = P_.mean(0)
        nn = np.linalg.svd(P_ - c)[2][2]
        return Part.Face(Part.Plane(App.Vector(*c), App.Vector(*nn)), w)
    sh = Part.Shell([srf.toShape(), cap(0.0), cap(1.0)])
    sh.sewShape()
    sol = Part.Solid(sh)
    if sol.Volume < 0:
        sol.reverse()
    return sol, (n, mv, err)


def run(tool, edges, support=None, orient="path", twist_deg=0.0, sec_tol=0.02, fit_tol=0.02, progress=None,
        cancel=None, ds_mid=10.0, max_disp=0.5, **kw):
    """Hizli supurme: kati ya da hata metni. tool: kati (dunya koordinatinda), edges: yol kenarlari."""
    import Mesh
    import MeshPart
    T0 = time.time()
    log = []

    def step(msg):
        log.append("%5.1fs %s" % (time.time() - T0, msg))
        if progress is not None:
            progress(msg)
        if cancel is not None and cancel.is_set():
            raise Cancelled()
    try:
        diag = tool.BoundBox.DiagonalLength
        step("hizli yontem: alet mesh'e cevriliyor")
        tm = MeshPart.meshFromShape(Shape=tool, LinearDeflection=max(0.02, min(0.2, 2 * sec_tol)),
                                    AngularDeflection=0.1)
        c = tool.CenterOfMass
        mo = Motion(edges, orient, twist_deg, diag, support, start_hint=np.array([c.x, c.y, c.z]))
        L = mo.L
        inv0 = mo.matrix(0.0).inverse()
        V = np.array([(q.x, q.y, q.z) for q in tm.Points])
        # kopya araligi: komsu kopyalarda aletin hicbir noktasi max_disp'ten fazla kaymasin; toplam ucgen siniri
        probe = np.linspace(0, L, 41)

        def placed(sv):
            M = mo.matrix(sv).multiply(inv0)
            return np.array([tuple(M.multVec(App.Vector(*q))) for q in V[:: max(1, len(V) // 400)]])
        disp = max(np.linalg.norm(placed(min(L, a + 1.0)) - placed(a), axis=1).max() for a in probe[:-1])
        n = int(np.ceil(L * max(disp, 1e-9) / max_disp)) + 1
        n = max(2, min(n, int(MAX_FACETS / max(tm.CountFacets, 1))))
        step("hizli yontem: %d kopya (%d ucgen)" % (n, n * tm.CountFacets))
        big = Mesh.Mesh()
        for sv in np.linspace(0, L, n):
            m = tm.copy()
            m.transform(mo.matrix(sv).multiply(inv0))
            big.addMesh(m)
        # kesit araligi: ilk / son kopyanin yol yonundeki en geri / en ileri noktasi
        p0, t0 = mo.frame(0.0)[:2]
        p1, t1 = mo.frame(L)[:2]
        sa = min(0.0, ((placed(0.0) - p0) @ t0).min())
        sb = L + max(0.0, ((placed(L) - p1) @ t1).max())
        A_ = _ramp(sa, 1, ds_mid)
        B_ = _ramp(sb, -1, ds_mid)
        st = np.unique(np.round(np.r_[A_, np.arange(A_[-1] + ds_mid, B_[-1] - ds_mid / 2, ds_mid), B_], 6))
        frames = [mo.frame(sv, roll=False) for sv in st]
        step("hizli yontem: %d duzlem kesiliyor" % len(st))
        secs = big.crossSections([(App.Vector(*F[0]), App.Vector(*F[1])) for F in frames], 1e-6, True)
        step("hizli yontem: dis sinirlar")
        G, Vv = [], []
        for sv, (p, t, u, v), polys in zip(st, frames, secs):
            ring = _outer_ring(polys, p, u, v, 1.5 * diag)
            if ring is not None:
                G.append(ring)
                Vv.append(sv)
        step("hizli yontem: kati kuruluyor (%d kesit)" % len(G))
        import builtins
        builtins._bsfastdata = (np.array(G), np.array(Vv))      # inceleme icin
        sol, (nu, mv, err) = _loft(np.array(G), np.array(Vv), tol=max(0.1, fit_tol))
        step("hizli yontem bitti: gecerli %s, yuzey %d x %d kontrol noktasi, halkalara en buyuk sapma %.3f mm"
             % (sol.isValid(), nu, mv, err))
        return sol
    except Cancelled:
        return "cancelled"
    except Exception as e:
        import traceback
        log.append("HATA " + traceback.format_exc()[-1200:])
        return str(e) or e.__class__.__name__
    finally:
        import builtins
        builtins._bsfastlog = log
