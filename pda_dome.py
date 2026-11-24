# -*- coding: utf-8 -*-
# Pimp My FreeCAD - Dome: duz yuzun uzerinde, kenarlarda komsu yuzlere teget kubbe. Copyright (c) 2026 Sedat Kuran - MIT License
#
# Yuzey OCCT dolgusu (GeomPlate) degil: yuzun duzleminde yukseklik alani h(x, y), kubik B-spline uzayinda ince plaka
# enerjisini en aza indirir (Ritz): kenarda h = 0 ve dh/dn = komsu yuzun egimi, yuk Height'i verecek kadar. Sonuc
# dogrudan B-spline, ayrica oturtma yok. Olculdu (yuvarlak dikdortgen, 80 x 50, 4 mm): teget hatasi 0.0005 derece, kenar
# boslugu 3e-6 mm, 0.3 s; GeomPlate 0.04 derece ama delikte ve ic bukeyde gecersiz kati ya da 0.03..0.6 mm bosluk verdi.
# Teget kubbe kenarda sifira inen ince bir dilimdir, boolean bozuluyordu (0 kati, gecersiz sonuc): yuz katinin icinde
# dogrudan degistirilir. Boolean yalniz kubbe katinin baska bir yerine degiyorsa denenir.
# Butun komsular dikse (Elliptical) yukseklik alani yetmez, egim sonsuza gider: kenar merkeze dogru kuculurken ceyrek elips
# boyunca yukselir (ellipse_patches). Tam NURBS, duvara teget hatasi 0; burada boolean sorunsuz.
import numpy as np

import FreeCAD as App
import Part

import pda_railpipe as rp
from pda_i18n import T

V = App.Vector
FREE_DEG = 60.0             # komsu bu acidan dik: teget olunamaz (egim sonsuza gider), kenar egimi serbest birakilir
WALL_DEG = 30.0             # butun komsular en az bu kadar egimliyse kubbe duvarlara teget cikar (pah da olur)
ROW_TOL = 2e-3              # duvar boyunca orta sira: yukseklik basina izin verilen sapma
MAX_ROW_TRIES = 6           # orta sira uydurma: en cok bu kadar deneme (her birinde dugum sayisi ikiye katlanir)
MAX_ROW_POLES = 300         # ...ve kenar basina en cok bu kadar kontrol noktasi
RIDGE_RATIO = 1.6          # uzunluk / genislik bunun ustundeyse tepe tek nokta degil, ortadan gecen bir sirt
CORNER_DEG = 10.0         # kenar birlesiminde bu acinin ustu kosedir: egim koseye dogru sifira iner
CELLS = (40, 56)            # kenar boslugu buyukse ikinci, ince izgara
GAP_OK = 1e-3               # mm
GAP_MAX = 0.02              # mm: izgara yuz icin kaba (cok dar bolge)
LAMBDA = 1e6
GHOST = 1e-5
MARGIN = 3                  # izgara yuzun disina tasan hucre: yuz duzgun dugum bolgesinde kalir
SUB = 5                     # hucre basina alt nokta (kesik hucrelerin integrali)


def cubic(s):
    """Duzgun kubik B-spline: hucre icinde s (0..1) icin N, N', N'' (birim hucre)."""
    s = np.asarray(s, float)
    r = 1 - s
    N = np.stack([r**3, 3*s**3 - 6*s**2 + 4, -3*s**3 + 3*s**2 + 3*s + 1, s**3], -1) / 6.0
    D = np.stack([-3*r**2, 9*s**2 - 12*s, -9*s**2 + 6*s + 3, 3*s**2], -1) / 6.0
    S = np.stack([6*r, 18*s - 12, -18*s + 6, 6*s], -1) / 6.0
    return N, D, S


def _segments(loops):
    return np.vstack([p[:-1] for p in loops]), np.vstack([p[1:] for p in loops])


def _inside_rows(X, Y, loops):
    """Cift-tek kurali, ayni y'li noktalar birlikte (izgara icin hizli)."""
    A, B = _segments(loops)
    out = np.zeros(len(X), bool)
    for y in np.unique(Y):
        k = np.nonzero(Y == y)[0]
        c = (A[:, 1] > y) != (B[:, 1] > y)
        if c.any():
            xs = np.sort(A[c, 0] + (y - A[c, 1]) * (B[c, 0] - A[c, 0]) / (B[c, 1] - A[c, 1]))
            out[k] = np.searchsorted(xs, X[k]) % 2 == 1
    return out


def _inside(P, loops):
    A, B = _segments(loops)
    res = np.zeros(len(P), bool)
    for i, (x, y) in enumerate(P):
        c = (A[:, 1] > y) != (B[:, 1] > y)
        res[i] = (A[c, 0] + (y - A[c, 1]) * (B[c, 0] - A[c, 0]) / (B[c, 1] - A[c, 1]) > x).sum() % 2 == 1
    return res


def neighbour(shape, edge, face):
    for f in shape.ancestorsOfType(edge, Part.Face):
        if not f.isSame(face):
            return f
    return None


def frame(face):
    """Duzlem cercevesi: x en uzun duz kenar boyunca (izgara kenarlarla hizali), n yuzun disa bakan normali."""
    u0, u1, v0, v1 = face.ParameterRange
    n = face.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
    n.normalize()
    lines = [e for e in face.Edges if e.Curve.TypeId == "Part::GeomLine"]
    if lines:
        e = max(lines, key=lambda x: x.Length)
        x = e.tangentAt(e.FirstParameter)
    else:
        P = np.array([tuple(p) for p in face.OuterWire.discretize(Number=200)])
        x = V(*(float(c) for c in np.linalg.svd(P - P.mean(axis=0))[2][0]))
    x = x - n * x.dot(n)
    x.normalize()
    return face.CenterOfGravity, x, n.cross(x), n


class Field(object):
    """Bir yuzun yukseklik alani: h = a * h1 (yuk) + b * hg (kenar egimi). Height degisince yalniz a, b degisir."""

    def __init__(self, face, shape, cells=40, tangent=True):
        self.face, self.shape, self.tangent = face, shape, tangent
        self.o, self.x, self.y, self.n = frame(face)
        self.loops = [np.array([self.to2(p) for p in w.discretize(Number=max(400, int(w.Length / 0.2)))])
                      for w in face.Wires]
        pts = np.vstack(self.loops)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        self.dx, self.dy = (hi - lo) / cells
        self.org = lo - MARGIN * np.array([self.dx, self.dy])
        self.nc = cells + 2 * MARGIN
        self.np = self.nc + 3
        self._samples()
        self._solve()

    def to2(self, p):
        d = V(p) - self.o
        return d.dot(self.x), d.dot(self.y)

    def _samples(self):
        step = min(self.dx, self.dy) / 3.0
        eps = 0.05 * min(self.dx, self.dy)
        pts, nrm, g, free = [], [], [], []
        for e in self.face.Edges:
            f = neighbour(self.shape, e, self.face)
            ts = np.linspace(e.FirstParameter, e.LastParameter, max(8, int(e.Length / step)) + 1)
            ep, ed, en2 = [], [], []
            for t in ts:
                p, d = e.valueAt(t), e.tangentAt(t)
                ep.append(self.to2(p))
                ed.append((d.dot(self.x), d.dot(self.y)))
                if f is None:
                    en2.append((0.0, 0.0, 1.0))
                else:
                    m = f.normalAt(*f.Surface.parameter(p))
                    m.normalize()
                    en2.append((m.dot(self.x), m.dot(self.y), m.dot(self.n)))
            ep, ed, en2 = np.array(ep), np.array(ed), np.array(en2)
            ed /= np.linalg.norm(ed, axis=1)[:, None]
            en = np.stack([-ed[:, 1], ed[:, 0]], 1)
            mid = np.arange(1, len(ep) - 1)
            if _inside(ep[mid] + eps * en[mid], self.loops).mean() < 0.5:   # n: yuzun icine
                en = -en
            ang = np.degrees(np.arctan2(-(en2[:, 0] * en[:, 0] + en2[:, 1] * en[:, 1]), en2[:, 2]))  # +: komsu disa dusuyor
            fr = np.abs(ang) > FREE_DEG
            gg = np.where(fr, 0.0, np.tan(np.radians(np.clip(ang, 0.0, FREE_DEG))))
            pts.append(ep), nrm.append(en), g.append(gg), free.append(fr)
        # keskin kosede iki kenarin h = 0'i egimi sifira zorlar: egim koseye dogru iner, yoksa sonuc kirisir
        cell = max(self.dx, self.dy)
        ends = [(p[k], n[k]) for p, n in zip(pts, nrm) for k in (0, -1)]
        corners = [a[0] for i, a in enumerate(ends) for b in ends[i + 1:]
                   if np.linalg.norm(a[0] - b[0]) < 1e-6 and a[1].dot(b[1]) < np.cos(np.radians(CORNER_DEG))]
        self.bp, self.bn = np.vstack(pts), np.vstack(nrm)
        self.bg, self.bfree = np.concatenate(g), np.concatenate(free)
        for c in corners:
            s = np.clip(np.linalg.norm(self.bp - c, axis=1) / (3 * cell), 0.0, 1.0)
            self.bg = self.bg * s * s * (3 - 2 * s)
        if not self.tangent:                               # oyma: komsu yukari gelmez, duz baslar
            self.bg = np.zeros_like(self.bg)

    def rows(self, P):
        """2B noktalar: kontrol noktasi indeksleri (m x 16) ve deger, d/dx, d/dy satirlari."""
        tu = (P[:, 0] - self.org[0]) / self.dx
        tv = (P[:, 1] - self.org[1]) / self.dy
        iu = np.clip(np.floor(tu).astype(int), 0, self.nc - 1)
        iv = np.clip(np.floor(tv).astype(int), 0, self.nc - 1)
        Nu, Du, _ = cubic(tu - iu)
        Nv, Dv, _ = cubic(tv - iv)
        ids = np.stack([(iu + a) * self.np + iv + b for a in range(4) for b in range(4)], 1)
        val = (Nu[:, :, None] * Nv[:, None, :]).reshape(len(P), 16)
        rx = (Du[:, :, None] * Nv[:, None, :]).reshape(len(P), 16) / self.dx
        ry = (Nu[:, :, None] * Dv[:, None, :]).reshape(len(P), 16) / self.dy
        return ids, val, rx, ry

    def _cell_ids(self, i, j):
        return np.array([(i + a) * self.np + j + b for a in range(4) for b in range(4)])

    def _solve(self):
        s = (np.arange(SUB) + 0.5) / SUB
        Ns, Ds, Ss = cubic(s)
        dx, dy = self.dx, self.dy
        area = dx * dy / SUB**2
        tab = np.zeros((SUB, SUB, 16, 16))                 # hucre ici alt noktalarin enerji tablolari: butun hucrelerde ayni
        load = np.zeros((SUB, SUB, 16))
        for a in range(SUB):
            for b in range(SUB):
                bxx = np.outer(Ss[a], Ns[b]).ravel() / dx**2
                byy = np.outer(Ns[a], Ss[b]).ravel() / dy**2
                bxy = np.outer(Ds[a], Ds[b]).ravel() / (dx * dy)
                tab[a, b] = (np.outer(bxx, bxx) + np.outer(byy, byy) + 2 * np.outer(bxy, bxy)) * area
                load[a, b] = np.outer(Ns[a], Ns[b]).ravel() * area
        cu = (np.arange(self.nc)[:, None] + s[None, :]).ravel() * dx + self.org[0]
        cv = (np.arange(self.nc)[:, None] + s[None, :]).ravel() * dy + self.org[1]
        X, Y = np.meshgrid(cu, cv, indexing="ij")
        mask = _inside_rows(X.ravel(), Y.ravel(), self.loops).reshape(X.shape)
        N = self.np**2
        A = np.zeros((N, N))
        f = np.zeros(N)
        full = tab.sum(axis=(0, 1))
        scale = dx * dy
        for i in range(self.nc):
            for j in range(self.nc):
                m = mask[i * SUB:(i + 1) * SUB, j * SUB:(j + 1) * SUB]
                if m.any():
                    ids = self._cell_ids(i, j)
                    A[np.ix_(ids, ids)] += scale * (full if m.all() else (tab * m[:, :, None, None]).sum(axis=(0, 1)))
                    f[ids] += (load * m[:, :, None]).sum(axis=(0, 1)) / scale
        ids, val, rx, ry = self.rows(self.bp)
        active = A.diagonal() > 0
        active[ids.ravel()] = True
        for i in range(self.nc):                           # hayalet ceza: kesik hucrelerde kontrol noktalari sallanmasin
            for j in range(self.nc):
                c = self._cell_ids(i, j)
                if active[c].any():
                    A[np.ix_(c, c)] += GHOST * scale * full
        np.add.at(A, (ids[:, :, None].repeat(16, 2), ids[:, None, :].repeat(16, 1)),
                  LAMBDA * val[:, :, None] * val[:, None, :])
        rn = rx * self.bn[:, 0:1] + ry * self.bn[:, 1:2]
        c = ~self.bfree
        w = LAMBDA * min(dx, dy)**2
        g = np.zeros(N)
        np.add.at(A, (ids[c][:, :, None].repeat(16, 2), ids[c][:, None, :].repeat(16, 1)),
                  w * rn[c][:, :, None] * rn[c][:, None, :])
        np.add.at(g, ids[c], w * rn[c] * self.bg[c][:, None])
        act = np.nonzero(active)[0]
        M = A[np.ix_(act, act)]
        M += 1e-12 * np.trace(M) / len(act) * np.eye(len(act))
        sol = np.linalg.solve(M, np.stack([f[act], g[act]], 1))
        h1, hg = np.zeros(N), np.zeros(N)
        h1[act], hg[act] = sol[:, 0], sol[:, 1]
        gi, gv, _, _ = self.rows(np.stack([X[mask], Y[mask]], 1))
        z1 = (h1[gi] * gv).sum(axis=1)
        h1 /= np.abs(z1).max()
        self.h1, self.hg = h1, hg
        self.z1, self.zg = z1 / np.abs(z1).max(), (hg[gi] * gv).sum(axis=1)

    def coefficients(self, height):
        """Tepe Height olacak sekilde (a, b). Komsu egimi bu yukseklige gore fazla dikse b < 1: egim tutmaz, yuzey kenarda
        sirt yapip ortada cukurlasmak yerine duz kalir."""
        top = self.zg.max() if self.zg.size else 0.0
        b = min(1.0, 0.9 * height / top) if top > 0 else 1.0
        lo, hi = 0.0, 4.0 * height + 1.0
        for _ in range(50):
            a = (lo + hi) / 2
            if (a * self.z1 + b * self.zg).max() > height:
                hi = a
            else:
                lo = a
        return (lo + hi) / 2, b

    def surface(self, height, sign=1.0):
        a, b = self.coefficients(height)
        c = (sign * (a * self.h1 + b * self.hg)).reshape(self.np, self.np)
        gx = self.org[0] + self.dx * (np.arange(self.np) - 1.0)
        gy = self.org[1] + self.dy * (np.arange(self.np) - 1.0)
        poles = [[self.o + self.x * float(gx[i]) + self.y * float(gy[j]) + self.n * float(c[i, j])
                  for j in range(self.np)] for i in range(self.np)]
        ku = [float(self.org[0] + self.dx * (k - 3)) for k in range(self.np + 4)]
        kv = [float(self.org[1] + self.dy * (k - 3)) for k in range(self.np + 4)]
        s = Part.BSplineSurface()
        s.buildFromPolesMultsKnots(poles, [1] * len(ku), [1] * len(kv), ku, kv, False, False, 3, 3)
        s.segment(ku[3], ku[self.nc + 3], kv[3], kv[self.nc + 3])    # uclari kenetli: STEP okuyuculari icin
        return s


def trim(surf, face):
    """Yuzeyi yuzun kendi kenarlarina oturt: kenarlar ayni kalir, komsu yuzlerle ortak olur."""
    d = Part.Face(surf, face.OuterWire)
    inner = [w for w in face.Wires if not w.isSame(face.OuterWire)]
    if inner:
        d.fix(1e-3, 1e-5, 1e-2)                            # pcurve'ler olmadan delik acilmiyor
        d.cutHoles(inner)
    d.fix(1e-3, 1e-5, 1e-2)
    return d


def gap_of(surf, face, fld):
    out = 0.0
    for e in face.Edges:
        for t in np.linspace(e.FirstParameter, e.LastParameter, 25):
            p = e.valueAt(float(t))
            out = max(out, (surf.value(*fld.to2(p)) - p).Length)
    return out


def tangent_error(dome, face, shape, fld):
    """Kenarlarda komsu yuze gore en buyuk aci (derece), teget olunabilen kenarlar uzerinde; dik duvar sayisi."""
    S = dome.Surface
    worst, steep = 0.0, 0
    for e in face.Edges:
        f = neighbour(shape, e, face)
        if f is None:
            continue
        err = []
        for t in np.linspace(e.FirstParameter, e.LastParameter, 11)[1:-1]:
            p = e.valueAt(float(t))
            m, m2 = S.normal(*fld.to2(p)), f.normalAt(*f.Surface.parameter(p))
            err.append((np.degrees(np.arccos(min(1.0, abs(m.dot(m2) / (m.Length * m2.Length))))),
                        abs(m2.dot(frame_normal(face))) / m2.Length))
        if min(c for _, c in err) < np.cos(np.radians(FREE_DEG)):
            steep += 1
        else:
            worst = max(worst, max(a for a, _ in err))
    return worst, steep


def frame_normal(face):
    u0, u1, v0, v1 = face.ParameterRange
    return face.normalAt((u0 + u1) / 2, (v0 + v1) / 2)


def can_ellipse(face, shape):
    """Duvarlara teget kubbe olur mu: tek dis cizgi, merkezden her kenar noktasi gorunur, butun komsular belirgin egimli
    (neredeyse yatay komsu, fillet gibi, duvar boyunca tepe duzlemine cikamaz). Merkez (yuzun agirlik merkezi) ya da None."""
    if len(face.Wires) != 1:
        return None
    n = frame_normal(face)
    n.normalize()
    c = face.CenterOfGravity
    turn = []
    for e in face.Edges:
        f = neighbour(shape, e, face)
        if f is None:
            return None
        for t in np.linspace(e.FirstParameter, e.LastParameter, 21):
            p = e.valueAt(float(t))
            m = f.normalAt(*f.Surface.parameter(p))
            if abs(m.dot(n)) / m.Length > np.cos(np.radians(WALL_DEG)):
                return None
            turn.append(((p - c).cross(e.tangentAt(float(t))).dot(n)) * (-1.0 if e.Orientation == "Reversed" else 1.0))
    turn = np.array(turn)
    # merkezden bakinca kenar hep ayni yone donmeli; duz kenar merkezden geciyorsa 0 olur, o da olmaz
    size = face.BoundBox.DiagonalLength
    return c if (turn > 1e-6 * size).all() or (turn < -1e-6 * size).all() else None


_WALL = {}                  # duvar normali onbellegi: (yuz, nokta) -> normal. Orta sira her denemede ayni noktalari
                            # yeniden soruyordu; f.Surface her cagrida yuzeyi kopyaliyor. Kubbe suresinin %85'i buydu.


def _wall_normal(f, p):
    hk = (f.hashCode(), f.Orientation)
    ent = _WALL.get(hk)
    if ent is None or not ent[0].isSame(f):
        if len(_WALL) > 64:
            _WALL.clear()
        ent = _WALL[hk] = (f, f.Surface, {})
    key = (round(p.x, 9), round(p.y, 9), round(p.z, 9))
    m = ent[2].get(key)
    if m is None:
        m = f.normalAt(*ent[1].parameter(p))
        m.normalize()
        ent[2][key] = m
    return V(m)


def _up_along(a, b, n):
    """a x b dogrultusunda, n yonunde birim yukseklik veren vektor."""
    d = a.cross(b)
    return d * (1.0 / d.dot(n))


def _corner_dirs(face, shape, n):
    """Keskin koselerde iki duvarin ortak dogrultusu: {(kenar, uc): vektor}. Sirt iki yamada da ayni olmali ve iki duvara
    da teget kalmali; bunu saglayan tek yon duvarlarin kesisimi."""
    ends = []
    for i, e in enumerate(face.Edges):
        f = neighbour(shape, e, face)
        for k, t in enumerate((e.FirstParameter, e.LastParameter)):
            p = e.valueAt(t)
            ends.append((i, k, p, _wall_normal(f, p), e.tangentAt(t)))
    out = {}
    for a, (i, k, p, m, d) in enumerate(ends):
        for j, l, q, m2, d2 in ends[a + 1:]:
            if j == i or (p - q).Length > 1e-5:
                continue
            if m.cross(m2).Length > np.sin(np.radians(1.0)):
                c = _up_along(m, m2, n)
            else:                                          # teget birlesim: iki yama ayni yonde bulussun, yoksa aralik kalir
                c = (_up_along(m, d, n) + _up_along(m2, d2, n)) * 0.5
            out[(i, k)] = out[(j, l)] = c
    return out


def _fit_params(cv, per=6):
    """En kucuk kareler icin ornek parametreleri: her dugum araligina en az `per` nokta. Esit aralikli ornekte sik
    dugumlu (mahya dugumleri eklenmis) parcada bazi araliklara hic nokta dusmuyor, kutuplar serbest kalip egri
    650 mm'lik dalgaya donuyordu."""
    k = [x for x in cv.getKnots() if cv.FirstParameter - 1e-12 <= x <= cv.LastParameter + 1e-12]
    k = sorted(set([cv.FirstParameter] + k + [cv.LastParameter]))
    ts = [cv.FirstParameter]
    for a, b in zip(k[:-1], k[1:]):
        if b - a > 1e-12:
            ts += list(np.linspace(a, b, per + 1)[1:])
    ts = np.array(ts)
    if len(ts) < max(24, 3 * cv.NbPoles):
        ts = np.union1d(ts, np.linspace(cv.FirstParameter, cv.LastParameter, max(24, 3 * cv.NbPoles)))
    return ts


def _rational_basis(cv, ts):
    """Egrinin kendi (rasyonel) tabani: ts noktalarinda satir basina kontrol noktalari."""
    knots, deg = np.array(cv.KnotSequence), cv.Degree
    w = np.array(cv.getWeights())
    N = ((knots[:-1, None] <= ts) & (ts < knots[1:, None])).astype(float).T
    N[ts >= knots[-1], :] = 0.0
    N[ts >= knots[-1], len(knots) - deg - 2] = 1.0
    for q in range(1, deg + 1):
        M = np.zeros((len(ts), len(knots) - q - 1))
        for i in range(M.shape[1]):
            a, b = knots[i + q] - knots[i], knots[i + q + 1] - knots[i + 1]
            if a > 0:
                M[:, i] += (ts - knots[i]) / a * N[:, i]
            if b > 0:
                M[:, i] += (knots[i + q + 1] - ts) / b * N[:, i + 1]
        N = M
    R = N * w
    return R / R.sum(axis=1)[:, None]


def _wall_row(cv, f, n, height, sign, span, c0, c1, joints, full=None):
    """Orta sira kontrol noktalari: kenarin her noktasi duvar boyunca tepe duzlemine cikar. Yuzeyin kenardaki tegeti bu
    yondedir. Duz duvarda iki kose yonu arasinda dogrusal kalir, bu yuzden duvar duzleminden cikmaz."""
    a, b = span
    ts = _fit_params(cv)
    Q, ends = [], {}

    def steep(t, curve=cv):
        p = curve.value(t)
        d = _wall_normal(f, p).cross(curve.tangent(t)[0])
        return p, d * (1.0 / d.dot(n))
    for key, t, c in (("a", a, c0), ("b", b, c1)):         # uc farklari kenarin tam parametresinde; parca (cv) kenarin
        # yalniz bir bolumu olabilir: ucta kenarin kendi egrisi (cv'nin disinda deger uydurma kontrol noktasini 80 mm
        # firlatiyordu, sirtli kubbe twistli elipste "wall passes the centre")
        ends[key] = V(0, 0, 0) if c is None else c - steep(t, full or cv)[1]
    for t in ts:
        p, d = steep(float(t))
        s = (float(t) - a) / (b - a)
        d = d + ends["a"] * (1.0 - s) + ends["b"] * s
        m = _wall_normal(f, p)                             # kavisli duvarda kose duzeltmesi duvardan cikmasin
        d = d - m * d.dot(m)
        h = height(p) if callable(height) else height
        Q.append(tuple(p + d * (sign * h / d.dot(n))))
    Q = np.array(Q)
    R = _rational_basis(cv, ts)
    # uclar tam: komsu yamayla sirt ayni olmali. Yaklasik uc (agirlikli en kucuk kareler) dikisi tutmuyor, kati gecersiz
    # kaliyordu (yay + duz kenarli yuzde 4 sirt acik)
    # Ayni yay parcalarinda koni normali de 6e-6 mm fark veriyordu: ortak uc bir kez hesaplanir.
    for k in (0, -1):
        Q[k] = joints.setdefault(tuple(np.round(tuple(cv.value(float(ts[k]))), 5)), Q[k])
    sol = np.zeros((R.shape[1], 3))
    sol[0], sol[-1] = Q[0], Q[-1]
    if R.shape[1] > 2:
        rhs = Q - R[:, 0:1] * sol[0] - R[:, -1:] * sol[-1]
        sol[1:-1] = np.linalg.lstsq(R[:, 1:-1], rhs, rcond=None)[0]
    return [V(*(float(x) for x in r)) for r in sol], float(np.abs(R.dot(sol) - Q).max())


CAGE_K = 0.5523             # kubik ceyrek daire: teget sira duvar boyunca h*K, tepe sirasi sirta dogru D*K
RIDGE_PTS = 25              # sirt kirik cizgisinin nokta sayisi (profilin orta cizgisini izler)
RIDGE_DENSE = 31            # mahya egrisinin ara nokta sayisi (uca dogru sik)
RIDGE_CUTS = 12             # mahya boyunca yama sayisi (her yanda): mahya artik egri, araliklari egri dolduruyor
RIDGE_EVEN = 0.5            # 0: kosinus (uclarda cok sik, ucta milimetrelik yama), 1: esit; arasi: egimde biraz sik
RIDGE_ENDS = (0.0, 0.06, 0.1, 0.15, 0.25, 0.35, 0.5)   # sirt uclari profilin ucundan bu kadar genislik iceride: once yakin (uc yamasi kisa,
                                 # katlanma az); yatik duvarda orta sira sirti gecerse daha iceriden denenir


class _Ridge(object):
    """Tepe sirti: profilin uzun ekseni u boyunca s in [sa, sb]; her s'de profilin ortasindan gecen kirik cizgi
    (kanat gibi kavisli profilde duz eksen uca dogru kenara yaklasiyordu)."""

    def __init__(self, center, u, v, ss, off, rise=None):
        self.c, self.u, self.v = center, u, v
        self.ss, self.off = list(ss), list(off)
        self.rs = list(rise) if rise is not None else [1.0] * len(self.ss)
        self.sa, self.sb = self.ss[0], self.ss[-1]
        mid, half = 0.5 * (self.sa + self.sb), 0.5 * (self.sb - self.sa)
        k = np.linspace(0.0, 1.0, RIDGE_CUTS + 1)
        self.cuts = [float(mid - half * ((1.0 - RIDGE_EVEN) * np.cos(np.pi * x) + RIDGE_EVEN * (1.0 - 2.0 * x)))
                     for x in k]
        # mahya profilin ucuna iniyorsa son dilim tek bir kapak yamasi (kullanicinin fikri): burun boyunca kenar, ilk
        # bolmedeki mahya noktasinda tek tepeye kapanir. Once mahya uca kadar iniyordu: burundaki dik inis iki yanin
        # yamalari arasinda kirisik, dikis, gobek yapiyordu. Mahya egrisi [lo, hi] arasinda.
        self.cap = self.rs[0] == 0.0 and len(self.cuts) > 3
        self.lo, self.hi = (self.cuts[1], self.cuts[-2]) if self.cap else (self.sa, self.sb)

    def rise(self, s):
        """Sirtin bu noktadaki yuksekligi / kubbe yuksekligi: yandan bakinca da kubbe (uclara dogru alcalir)."""
        if getattr(self, "curve", None) is not None:
            return (self.curve.value(min(self.hi, max(self.lo, s))) - self.c).dot(self.n) / self.hs
        return float(np.interp(s, self.ss, self.rs))

    def build(self, n, hs):
        """Mahya egrisi: sirt noktalarindan gecen kubik B-spline (parametre = s). Once kirik cizgiydi; yandan ve ustten
        bakinca her parca duz, uclarda kose gibi duruyordu (kullanici: "mahya cizgileri egri olsun")."""
        self.n, self.hs = n, hs
        # seyrek noktalardan gecen spline ucta (yarim elipsin dik indigi yer) dalgalaniyordu: yandan bakinca ucta
        # gobek. Sik (uca dogru daha sik) noktalar, yukseklik elipsin kendisinden, kayma yumusak egriden.
        oc = Part.BSplineCurve()
        oc.interpolate(Points=[V(float(s), float(o), 0.0) for s, o in zip(self.ss, self.off)],
                       Parameters=[float(x) for x in self.ss])
        mid, half = getattr(self, "rmid", None), getattr(self, "rhalf", None)
        k = np.linspace(0.0, np.pi, RIDGE_DENSE)
        ss = 0.5 * (self.lo + self.hi) - 0.5 * (self.hi - self.lo) * np.cos(k)
        pts = []
        for x in ss:
            x = float(x)
            r = (float(np.sqrt(max(1.0 - ((x - mid) / half) ** 2, 0.0))) if mid is not None
                 else float(np.interp(x, self.ss, self.rs)))
            pts.append(self.c + self.u * x + self.v * oc.value(x).y + n * (hs * r))
        if mid is not None and self.rs[0] == 0.0 and not self.cap:
            pts[0] = self.c + self.u * self.sa + self.v * self.off[0]
            pts[-1] = self.c + self.u * self.sb + self.v * self.off[-1]
        c = Part.BSplineCurve()
        c.interpolate(Points=pts, Parameters=[float(x) for x in ss])
        self.curve = c

    def top(self, s):
        """Mahya uzerindeki nokta (yukseklik dahil)."""
        return self.curve.value(min(self.hi, max(self.lo, s)))

    def slope(self, s):
        """Mahya yuksekliginin s'e gore egimi (uclarda tek yanli)."""
        e = 1e-3 * (self.hi - self.lo)
        a, b = max(self.lo, s - e), min(self.hi, s + e)
        return (self.top(b) - self.top(a)).dot(self.n) / (b - a)

    def s_of(self, p):
        return min(self.hi, max(self.lo, (p - self.c).dot(self.u)))

    def at(self, s):
        return self.c + self.u * s + self.v * float(np.interp(s, self.ss, self.off))


def _tips(face, center, u):
    """Profilin u ekseni boyunca en uc iki noktasi (tam, kenar egrisinde): [(s, nokta), (s, nokta)] (en kucuk, en buyuk)."""
    best = []
    for sgn in (1.0, -1.0):
        cand = None
        for e in face.Edges:
            a, b = e.FirstParameter, e.LastParameter
            ts = np.linspace(a, b, 600)
            vals = [sgn * (e.valueAt(float(t)) - center).dot(u) for t in ts]
            k = int(np.argmin(vals))
            lo, hi = float(ts[max(k - 1, 0)]), float(ts[min(k + 1, len(ts) - 1)])
            f = lambda t: sgn * (e.valueAt(t) - center).dot(u)
            for _ in range(60):                            # uclu arama: egrinin gercek ucu
                m1, m2 = lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0
                if f(m1) < f(m2):
                    hi = m2
                else:
                    lo = m1
            t = 0.5 * (lo + hi)
            if cand is None or f(t) < cand[0]:
                cand = (f(t), e.valueAt(t))
        best.append((sgn * cand[0], cand[1]))
    return best


def _ridge(face, center, n, end=0.5):
    """Uzun profilde tepe sirti (_Ridge) ya da None. Sirt uclardan yarim genislik iceride biter. Tek tepe noktasinda uzun
    ve sivri profil (kanat, ucta egrilik yaricapi 1.6 mm) tepeye yakin katlaniyordu: twistli duvarin egimi ucun iki
    yaninda ters donuyor, tek noktaya giden yama bunu tasiyamiyor. Yuvarlak profil (daire, kare) eskisi gibi tek tepe."""
    P = []
    for e in face.Edges:
        P += [tuple(p - center) for p in e.discretize(400)]
    P = np.array(P)
    nn = np.array(tuple(n))
    P = P - np.outer(P.dot(nn), nn)
    w, vecs = np.linalg.eigh(P.T.dot(P))
    u = vecs[:, np.argmax(w)]
    v = np.cross(nn, u)
    a, b = P.dot(u), P.dot(v)
    L, W = a.max() - a.min(), b.max() - b.min()
    if L < RIDGE_RATIO * W:
        return None
    U, Vv = V(*(float(x) for x in u)), V(*(float(x) for x in v))
    win = 0.5 * L / RIDGE_PTS
    sa, sb = a.min() + end * W, a.max() - end * W
    tips = None
    if end <= 0.0:                                         # mahya profilin en ucundan baslar (kullanicinin onerisi):
        tips = _tips(face, center, U)                      # uc yamasi kalmaz, kemer kenara kadar iner, sureklilik
        sa, sb = tips[0][0], tips[1][0]                    # kesilmez
    for _ in range(20):
        if sb - sa < 1e-3 * L:
            return None
        # uclara dogru sik (kosinus araligi): boyuna yarim elips uclarda hizla iniyor, seyrek noktada pah gibi duruyordu
        ss = 0.5 * (sa + sb) - 0.5 * (sb - sa) * np.cos(np.linspace(0.0, np.pi, RIDGE_PTS))
        off = []
        for t in ss:
            sel = np.abs(a - t) < win
            off.append(0.5 * (b[sel].min() + b[sel].max()) if sel.any() else 0.0)
        if tips is not None:
            off[0], off[-1] = (tips[0][1] - center).dot(Vv), (tips[1][1] - center).dot(Vv)
            mid, half = 0.5 * (sa + sb), 0.5 * (sb - sa)
        else:
            mid, half = 0.5 * (a.max() + a.min()), 0.5 * L
        rise = [float(np.sqrt(max(1.0 - ((t - mid) / half) ** 2, 0.0))) for t in ss]   # boyuna yarim elips
        if tips is not None:
            rise[0] = rise[-1] = 0.0
        r = _Ridge(center, U, Vv, ss, off, rise)
        r.rmid, r.rhalf = mid, half
        tol = 1e-4 * L if tips is not None else 1e-6
        if all(face.isInside(r.at(float(t)), tol, True) for t in np.linspace(sa, sb, 4 * RIDGE_PTS)):
            return r
        if tips is not None:
            return None
        m = 0.05 * (sb - sa)                               # sirt profilin icinde kalsin
        sa, sb = sa + m, sb - m
    return None


def _split_at(cv, ts):
    """Egriyi verilen parametrelerde parcalara ayirir."""
    a, b = cv.FirstParameter, cv.LastParameter
    cuts = [a] + sorted(t for t in ts if a + 1e-9 < t < b - 1e-9) + [b]
    out = []
    for p, q in zip(cuts[:-1], cuts[1:]):
        if q - p < 1e-9:
            continue
        c = cv.copy()
        c.segment(p, q)
        out.append(c)
    return out


def _ridge_spans(face, ridge):
    """Sirtli kubbe icin kenar parcalari: [(kenar, egri)]. Her parca sirtin tek bir duz [s_k, s_k+1] araligina iner; iki
    yandaki yamalarin ust kenarlari ayni araliklar olsun diye butun parcalar ayni s degerlerinde bolunur (yoksa sirtta T
    birlesimi, kabuk kapanmaz). Bolme yerleri: sirtin kirik noktalari, parca uclarinin izleri, izin dondugu yerler."""
    proj = ridge.s_of
    spans = [(e, cv) for e in face.Edges for cv in _smooth_spans(e)]
    must = {round(ridge.lo, 9), round(ridge.hi, 9)}         # zorunlu: kenar uclarinin izleri, izin dondugu yerler
    for e, cv in spans:
        x = (cv.value(cv.FirstParameter) - ridge.c).dot(ridge.u)
        if ridge.lo <= x <= ridge.hi:                      # kapak bolgesindeki kenar ucu mahyayi bolmez
            must.add(round(proj(cv.value(cv.FirstParameter)), 9))
        ts = np.linspace(cv.FirstParameter, cv.LastParameter, 400)
        raw = np.array([(cv.value(float(t)) - ridge.c).dot(ridge.u) for t in ts])
        for k in range(1, len(ts) - 1):
            if (raw[k] - raw[k - 1]) * (raw[k + 1] - raw[k]) < 0 and ridge.lo < raw[k] < ridge.hi:
                must.add(round(float(raw[k]), 9))
    S = set(must)
    cuts = ridge.cuts                                      # duzenli bolmeler: zorunlu bir bolmeye cok yakinsa atlanir
    for k in range(1, len(cuts) - 1):                      # (yan yana iki cizgi, milimetrelik ince yama olmasin)
        gap = 0.4 * min(cuts[k] - cuts[k - 1], cuts[k + 1] - cuts[k])
        if all(abs(cuts[k] - m) > gap for m in must):
            S.add(round(cuts[k], 9))
    S = sorted(S)
    out = []
    raw = lambda p: (p - ridge.c).dot(ridge.u)             # kirpilmamis iz: sirt ucunu (sa / sb) gecis de bolunur
    for e, cv in spans:
        ts = np.linspace(cv.FirstParameter, cv.LastParameter, 400)
        g = [raw(cv.value(float(t))) for t in ts]
        cut = []
        for sv in S:
            for k in range(len(ts) - 1):
                if (g[k] - sv) * (g[k + 1] - sv) < 0:
                    lo, hi = float(ts[k]), float(ts[k + 1])
                    for _ in range(50):
                        mid = 0.5 * (lo + hi)
                        if (raw(cv.value(lo)) - sv) * (raw(cv.value(mid)) - sv) <= 0:
                            hi = mid
                        else:
                            lo = mid
                    cut.append(0.5 * (lo + hi))
        span = ridge.sb - ridge.sa                         # uc noktalar (iz donuyor, sa / sb): tam orada bolunur
        for k in (range(1, len(ts) - 1) if not ridge.cap else ()):   # kapakta burun tek parca kalir
            if (g[k] - g[k - 1]) * (g[k + 1] - g[k]) < 0 and min(abs(g[k] - ridge.sa), abs(g[k] - ridge.sb)) < 0.02 * span:
                sgn = 1.0 if g[k] > g[k - 1] else -1.0     # tepe (+) ya da dip (-)
                lo, hi = float(ts[k - 1]), float(ts[k + 1])
                f = lambda t: -sgn * raw(cv.value(t))
                for _ in range(60):
                    m1, m2 = lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0
                    if f(m1) < f(m2):
                        hi = m2
                    else:
                        lo = m1
                cut.insert(0, 0.5 * (lo + hi))
        keep = []                                          # ucta iki bolme ust uste (s gecisi + uc noktasi): mikron
        for t in cut:                                      # boyunda sivri yama olusuyordu; uc noktasi kalir
            if all(cv.value(t).distanceToPoint(cv.value(k)) > 0.05 for k in keep):
                keep.append(t)
        out += [(e, c) for c in _split_at(cv, keep)]
    return out


def _top_row(cv, ridge, n, sign, height):
    """Yamanin ust sirasi: sirtin duz bir parcasi uzerinde, egri parcasinin uclarinin izleri arasinda sirali noktalar
    (ust kenar tam o aralik, iki yandaki yamalarla ayni)."""
    P = cv.getPoles()
    s0, s1 = ridge.s_of(cv.value(cv.FirstParameter)), ridge.s_of(cv.value(cv.LastParameter))
    lo, hi = min(s0, s1), max(s0, s1)
    ss = [min(hi, max(lo, ridge.s_of(p))) for p in P]
    ss[0], ss[-1] = s0, s1
    for k in range(1, len(ss)):                            # tek yonlu: ust kenar araligin disina tasmasin, geri donmesin
        ss[k] = max(ss[k], ss[k - 1]) if s1 >= s0 else min(ss[k], ss[k - 1])
    return [ridge.at(x) + n * (sign * height * ridge.rise(x)) for x in ss]


def _ridge_knots(cv, ridge):
    """Mahya egrisinin dugumleri yamanin icinde kalabilir: ayni yere tabana dugum (C2) eklenir, ust sira tam olur."""
    a, b = cv.FirstParameter, cv.LastParameter
    raw = lambda p: min(ridge.hi, max(ridge.lo, (p - ridge.c).dot(ridge.u)))
    s0, s1 = raw(cv.value(a)), raw(cv.value(b))
    if abs(s1 - s0) < 1e-9:
        return
    lo, hi = min(s0, s1), max(s0, s1)
    for kn in ridge.curve.getKnots():
        if lo + 1e-7 < kn < hi - 1e-7:
            cv.insertKnot(a + (kn - s0) / (s1 - s0) * (b - a), max(1, cv.Degree - 2))


def _top_curve_row(cv, ridge):
    """Yamanin ust sirasi tam mahya egrisi uzerinde: egri parcasinin uclarinin izleri [s0, s1] arasindaki mahya parcasi,
    egrinin kendi (rasyonel, derecesi yukseltilmis) tabaninda. Iki yandaki yamalar ayni s'lerde bolundugu icin ust
    kenarlari ayni egri parcasidir (parametreleri farkli olsa da geometri ayni): kabuk kapanir."""
    a, b = cv.FirstParameter, cv.LastParameter
    raw = lambda p: min(ridge.hi, max(ridge.lo, (p - ridge.c).dot(ridge.u)))
    s0, s1 = raw(cv.value(a)), raw(cv.value(b))
    ts = _fit_params(cv)
    Q = np.array([tuple(ridge.top(s0 + (s1 - s0) * (float(t) - a) / (b - a))) for t in ts])
    R = _rational_basis(cv, ts)
    sol = np.zeros((R.shape[1], 3))
    sol[0], sol[-1] = Q[0], Q[-1]
    if R.shape[1] > 2:
        rhs = Q - R[:, 0:1] * sol[0] - R[:, -1:] * sol[-1]
        sol[1:-1] = np.linalg.lstsq(R[:, 1:-1], rhs, rcond=None)[0]
    return [V(*(float(x) for x in r)) for r in sol]


def _flat_row(cv, top_of, K):
    """Tepeye yakin yatay sira: egrinin her noktasi icin tepe + (kenar - tepenin izdusumu) * K, egrinin kendi tabaninda en
    kucuk karelerle (uclar tam). Kontrol noktalarindan dogrudan kurmak mahya boyunca her parcada boncuk gibi kabarik
    yapiyordu: kontrol noktalari egrinin disbukey yanindadir."""
    ts = _fit_params(cv)
    Q = []
    for t in ts:
        p = cv.value(float(t))
        top, base = top_of(p)
        Q.append(tuple(top + (p - base) * K))
    Q = np.array(Q)
    R = _rational_basis(cv, ts)
    sol = np.zeros((R.shape[1], 3))
    sol[0], sol[-1] = Q[0], Q[-1]
    if R.shape[1] > 2:
        rhs = Q - R[:, 0:1] * sol[0] - R[:, -1:] * sol[-1]
        sol[1:-1] = np.linalg.lstsq(R[:, 1:-1], rhs, rcond=None)[0]
    return [V(*(float(x) for x in r)) for r in sol]


def _conic_patches(face, shape, center, height, sign):
    """Klasik kubbe (0.4.x): her kenar parcasindan tek tepe noktasina ceyrek konik (rasyonel, dik duvarda tam daire).
    Duzgun, simetrik profillerde (daire, yuvarlatilmis dikdortgen) en temiz sonuc bu: az yuz, tek parca gibi parlar.
    Duvar merkezi gecerse ValueError."""
    n = frame_normal(face)
    n.normalize()
    corners = _corner_dirs(face, shape, n)
    apex = center + n * (sign * height)
    weights = (1.0, float(np.sqrt(0.5)), 1.0)
    joints = {}
    out = []
    for i, e in enumerate(face.Edges):
        f = neighbour(shape, e, face)
        span = (e.FirstParameter, e.LastParameter)
        for cv in _smooth_spans(e):
            for it in range(MAX_ROW_TRIES):
                mid, err = _wall_row(cv, f, n, height, sign, span, corners.get((i, 0)), corners.get((i, 1)), joints,
                                     e.Curve)
                if err < ROW_TOL * max(height, 1.0) or it == MAX_ROW_TRIES - 1 or cv.NbPoles > MAX_ROW_POLES:
                    break
                k = cv.getKnots()
                for a, b in zip(k[:-1], k[1:]):
                    cv.insertKnot((a + b) / 2.0)
            P, W = cv.getPoles(), cv.getWeights()
            for p, q in zip(P, mid):
                if (q - apex).dot(p - center) <= 0:
                    raise ValueError("wall passes the centre")
            poles = [P, mid, [apex] * len(P)]
            s = Part.BSplineSurface()
            s.buildFromPolesMultsKnots(poles, [3, 3], cv.getMultiplicities(), [0.0, 1.0], cv.getKnots(), False, False,
                                       2, cv.Degree, [[w * a for w in W] for a in weights])
            out.append((e, s))
    return out


def _normal(s, u, v):
    """Yuzey normali; uca kapanan sutunda (kenar, orta sira, tepe ayni nokta) tanimsiz: sifir."""
    try:
        return tuple(s.normal(u, v))
    except Exception:
        return (0.0, 0.0, 0.0)


def _folds(patches, nu=24, nv=48):
    """Yuzeyde normalin ters dondugu komsu ornek cifti sayisi (katlanma). Tepe noktasina / uca kapanan dar bolge
    (kubbe yuksekliginin %5'inden alcak sutunlar ve tepeye cok yakin satirlar) sayilmaz: orada normal zaten tanimsiz."""
    cols = []
    for e, s in patches:
        v0, v1 = s.bounds()[2:]
        Vv = np.linspace(v0, v1, nv)
        cols.append((s, Vv, np.array([(s.value(1.0, float(v)) - s.value(0.0, float(v))).Length for v in Vv])))
    tall = max(c[2].max() for c in cols) if cols else 0.0
    U = np.linspace(0.0, 0.9, nu)
    bad = 0
    for s, Vv, H in cols:
        N = np.array([[_normal(s, float(u), float(v)) for v in Vv] for u in U])
        N = N / (np.linalg.norm(N, axis=2)[..., None] + 1e-12)
        ok = H > 0.05 * tall
        du = (N[1:] * N[:-1]).sum(2) < 0
        dv = (N[:, 1:] * N[:, :-1]).sum(2) < 0
        bad += int(du[:, ok].sum() + dv[:, ok[1:] & ok[:-1]].sum())
    return bad


def ellipse_patches(face, shape, center, height, sign):
    """Kubbe yamalari. Once klasik tek tepeli kubbe (duzgun profilde en guzeli; once sirt her uzun profilde devreye
    giriyordu, simetrik yuvarlatilmis dikdortgende de kubbeyi seritlere bolup boncuk gibi yapti). Katlaniyorsa (uzun,
    sivri, twistli profil): sirtli kafes (mahya uctan, olmazsa daha iceriden), o da olmazsa kafesli tek tepe. Hicbiri
    olmazsa ValueError (Dome serbest tumsege doner)."""
    n = frame_normal(face)
    n.normalize()
    try:
        out = _conic_patches(face, shape, center, height, sign)
        if _folds(out) == 0:
            return out
    except ValueError:
        pass
    tries = [r for r in (_ridge(face, center, n, k) for k in RIDGE_ENDS) if r is not None] + [None]
    last = None
    for ridge in tries:
        try:
            out = _patches(face, shape, center, height, sign, ridge)
        except ValueError as e:
            last = e
            continue
        if ridge is None or ellipse_tool(face, out).isValid():   # ucta mahya (end 0) duz uclu profilde kapanmayabilir
            return out
        last = ValueError("ridge dome is not a valid solid")
    raise last


def _patches(face, shape, center, height, sign, ridge):
    """Her kenar icin bir yama: kenardan duvar boyunca cikip tepeye ceyrek konik ile varir. Yay ve egri carpimi NURBS:
    sinir kenarin kendisi, dik duvarda yuzey tam. ridge None: yamalar tepede tek noktada bulusur; ridge: ust kenarlar sirtin
    duz parcalari, iki yanin yamalari sirtta bulusur (tepede normal dik, yuzey teget). Keskin kose tepeye kadar sirt olarak
    gider. Duvar o kadar yatik ki kenar merkezi / sirti gecerse ValueError."""
    n = frame_normal(face)
    n.normalize()
    corners = _corner_dirs(face, shape, n)
    apex = center + n * (sign * height)
    joints = {}
    out = []
    if ridge is not None:
        ridge.build(n, sign * height)
        rspans = _ridge_spans(face, ridge)
    edges = list(face.Edges)
    for i, e in enumerate(edges):
        f = neighbour(shape, e, face)
        span = (e.FirstParameter, e.LastParameter)
        curves = [cv for ee, cv in rspans if ee.isSame(e)] if ridge is not None else _smooth_spans(e)
        for cv in curves:
            # egimli / burulmus duvarda orta sira egrinin tabanina sigmayabilir: dugum ekle. Dongu her zaman egriyle ayni
            # sayida orta sira noktasiyla biter: once 4 denemeden sonra dugum eklenip orta sira yeniden hesaplanmiyordu,
            # yama 67 kenar / 35 orta kutupla kurulup gecersiz kaliyordu (twistli Magic Pad'in ust yuzu: hata 0.76 ->
            # 0.06 mm, hedef 0.02) ve kubbe duz kabarcik oluyordu
            # SubD kafesi gibi dort sira (kullanicinin fikri): kenar, duvar boyunca teget sira (yukseklik h*K), sirta /
            # tepeye dogru YATAY sira, tepe. Once tek kubik olmayan konik vardi (kenar, duvar yonunde orta sira, tepe):
            # duvarin egimi tepeye kadar tasiniyor, twistli duvarda egim ucun iki yaninda ters donunce tepeye yakin
            # katlaniyordu. Artik tepeye yakin sira duvarin degil sirtin yonunu izliyor; duvara teget, tepede yatay.
            # yandan da kubbe: sirtin yuksekligi boyunca yarim elips; teget sira her noktada o yuksekligin K kati
            hk = (lambda p: height * CAGE_K * ridge.rise(ridge.s_of(p))) if ridge is not None else height * CAGE_K
            for it in range(MAX_ROW_TRIES):
                mid, err = _wall_row(cv, f, n, hk, sign, span, corners.get((i, 0)), corners.get((i, 1)), joints,
                                     e.Curve)
                if err < ROW_TOL * max(height * CAGE_K, 1.0) or it == MAX_ROW_TRIES - 1 or cv.NbPoles > MAX_ROW_POLES:
                    break
                k = cv.getKnots()
                for a, b in zip(k[:-1], k[1:]):
                    cv.insertKnot((a + b) / 2.0)
            if ridge is not None:
                # mahya kubik egri: ust sira onu tam tasisin diye taban derecesi yukseltilir (rasyonel kenarda pay
                # derecesi kenar + 3), orta sira yeni tabanda yeniden kurulur
                cv = cv.copy()
                cv.increaseDegree(cv.Degree + (3 if cv.isRational() else max(0, 3 - cv.Degree)))
                _ridge_knots(cv, ridge)
                mid, err = _wall_row(cv, f, n, hk, sign, span, corners.get((i, 0)), corners.get((i, 1)), joints,
                                     e.Curve)
            P, W = cv.getPoles(), cv.getWeights()
            top = [apex] * len(P) if ridge is None else _top_curve_row(cv, ridge)
            plan = lambda x: x - n * (x - center).dot(n)       # yuz duzlemine izdusum
            for p, q, t in zip(P, mid, top):
                # sirtta: kenar noktasinin mahyadaki izi (ust sira kutbu egri parcasinda dogrusal dagilir, burunda
                # kenarin izinden kayar; onunla karsilastirmak dik duvarda bile "merkezi gecti" diyordu)
                base_t = plan(t) if ridge is None else plan(ridge.top(ridge.s_of(p)))
                r = p - base_t                             # tepeden kenara, yuz duzleminde
                if r.Length > 1e-2 * max(height, 1.0) and (plan(q) - base_t).dot(r) <= 0:
                    raise ValueError("wall passes the centre")
            if ridge is None:
                top_of = lambda p: (apex, plan(apex))
            else:
                def top_of(p):
                    # yatay sira mahyanin egimini izler (yalniz yanlara yatay): kapak tepesinde yuzey mahyanin inisini
                    # surduruyor; once tepe yatay, yandan bakinca kapakla komsu yama arasinda kirik vardi
                    x = ridge.s_of(p)
                    top = ridge.top(x)
                    base = plan(top)
                    w = (p - base) * CAGE_K
                    return top + n * (ridge.slope(x) * w.dot(ridge.u)), base
            flat = _flat_row(cv, top_of, CAGE_K)
            poles = [P, mid, flat, top]
            s = Part.BSplineSurface()
            s.buildFromPolesMultsKnots(poles, [4, 4], cv.getMultiplicities(), [0.0, 1.0], cv.getKnots(), False, False,
                                       3, cv.Degree, [list(W)] * 4)
            out.append((e, s))
    return out


def _smooth_spans(edge):
    """Kenarin NURBS'u, parametrik C0 dugumlerinde bolunmus. Daire / elips bu dugumlerle gelir: tek yamada BOP C0 hatasi
    verdi ve STEP'ten donen hacim 46..120 mm3 saptı."""
    a, b, cv = edge.FirstParameter, edge.LastParameter, edge.Curve
    if cv.TypeId in ("Part::GeomCircle", "Part::GeomEllipse"):
        cuts = list(np.linspace(a, b, int(np.ceil((b - a) / (np.pi / 2) - 1e-9)) + 1))   # parametre radyan: ceyrek basina tek aralik
    elif cv.TypeId == "Part::GeomBSplineCurve":
        inner = [k for k, m in zip(cv.getKnots(), cv.getMultiplicities()) if m >= cv.Degree and a + 1e-9 < k < b - 1e-9]
        cuts = [a] + inner + [b]
        if len(cuts) == 2 and edge.isClosed():             # kapali ama duzgun: dikis tek yamada kalmasin
            cuts.insert(1, (a + b) / 2.0)
    else:
        cuts = [a, b]
    out = []
    for p, q in zip(cuts[:-1], cuts[1:]):
        c = cv.toBSpline(float(p), float(q))
        if c.isPeriodic():
            c.setNotPeriodic()
        out.append(c)
    return out


def ellipse_tool(face, patches):
    shapes = [s.toShape() for _, s in patches]
    rims = [Part.Edge(s.uIso(0.0)) for _, s in patches]
    cap = Part.Face(Part.Wire(Part.__sortEdges__(rims)))   # kenarlar yamalarla ayni bolunmede
    sh = Part.Shell(shapes + [cap])
    sh.sewShape()
    sol = Part.Solid(sh)
    if sol.Volume < 0:
        sol.reverse()
    return sol


def ellipse_error(face, shape, patches):
    """Duvardaki en buyuk aci (derece): yama normali ile komsu yuz normali. Mahya profilin ucuna indiginde yama orada tek
    noktaya kapanir, normal tanimsiz: kubbenin yuksekligi %5'in altinda kalan uc bolgesi olculmez (yoksa 0.1 mm icinde
    38 derece gibi anlamsiz deger)."""
    cols = []
    for e, s in patches:
        v0, v1 = s.bounds()[2:]
        for t in np.linspace(0.02, 0.98, 25):
            v = v0 + (v1 - v0) * float(t)
            cols.append((e, s, v, (s.value(1.0, v) - s.value(0.0, v)).Length))
    tall = max(c[3] for c in cols) if cols else 0.0
    worst = 0.0
    for e, s, v, h in cols:
        if h < 0.05 * tall:
            continue
        f = neighbour(shape, e, face)
        q = s.value(0.0, v)
        try:
            a, b = s.normal(0.0, v), f.normalAt(*f.Surface.parameter(q))
        except Exception:
            continue
        worst = max(worst, np.degrees(np.arccos(min(1.0, abs(a.dot(b)) / (a.Length * b.Length)))))
    return worst


def read_face(link):
    """(nesne, yuz) ya da hata metni."""
    if not link or link[0] is None or not link[1]:
        return "Dome: the face is missing"
    try:
        f = link[0].Shape.getElement(link[1][0])
    except Exception:
        return "Dome: the face is missing"
    if f.ShapeType != "Face" or f.Surface.TypeId != "Part::GeomPlane":
        return "Dome: the face must be flat"
    return link[0], f


def same_face(shape, face):
    """Bir sekilde ayni geometrideki yuz: secim eski bir adimin yuzu olabilir, degistirilecek yuz son govdede."""
    c, a = face.CenterOfGravity, face.Area
    for f in shape.Faces:
        if f.isSame(face):
            return f
    for f in shape.Faces:
        if f.Surface.TypeId == "Part::GeomPlane" and abs(f.Area - a) < 1e-6 * max(a, 1.0) \
                and (f.CenterOfGravity - c).Length < 1e-5 and len(f.Edges) == len(face.Edges):
            return f
    return None


def lens(dome, face):
    """AddSubShape ve hacim denetimi icin: kubbe + duz yuz."""
    sh = Part.Shell([dome, face.copy()])
    sh.sewShape()
    sol = Part.Solid(sh)
    if sol.Volume < 0:
        sol.reverse()
    return sol


def replace_face(base, face, dome):
    """Katida yuzu kubbeyle degistir. Diger katilar oldugu gibi kalir. Gecersizse None."""
    target = next((s for s in base.Solids if any(f.isSame(face) for f in s.Faces)), None)
    if target is None:
        return None
    sh = Part.Shell([dome if f.isSame(face) else f for f in target.Faces])
    sh.sewShape()
    sol = Part.Solid(sh)
    if sol.Volume < 0:
        sol.reverse()
    if not sol.isValid():
        return None
    rest = [s for s in base.Solids if not s.isSame(target)]
    return sol if not rest else Part.makeCompound(rest + [sol])


def collides(base, dome, face, sign, fld):
    """Kubbe katinin baska bir yerine degiyor mu: eklerken yuzey katinin disinda, oyarken icinde kalmali."""
    u0, u1, v0, v1 = face.ParameterRange
    S = dome.Surface
    n = frame_normal(face)
    n.normalize()
    for i in range(1, 8):                                  # isInside kavisli govdede pahali: az nokta yeter
        for j in range(1, 8):
            q = face.Surface.value(u0 + (u1 - u0) * i / 8.0, v0 + (v1 - v0) * j / 8.0)
            if not face.isInside(q, 1e-6, False):
                continue
            p = S.value(*fld.to2(q))
            if abs((p - q).dot(n)) < 1e-2:
                continue
            if base.isInside(p, 1e-6, False) != (sign < 0):
                return True
    return False


def elliptical_prop(obj):
    if "Elliptical" not in obj.PropertiesList:
        obj.addProperty("App::PropertyBool", "Elliptical", "Dome",
                        "Next to steep walls: start tangent to the walls (bullet shape). Off: a smooth bump with a free "
                        "slope at the edges")


class Dome:
    """Dome (FeatureAdditivePython). Reversed: kubbe yuzun altina, oyma."""

    def __init__(self, obj):
        obj.addProperty("App::PropertyLinkSub", "Face", "Dome", "Flat face to raise")
        obj.addProperty("App::PropertyLength", "Height", "Dome", "Height of the dome above the face").Height = 4.0
        obj.addProperty("App::PropertyBool", "Reversed", "Dome", "Dome into the solid (a cut) instead of out of it")
        elliptical_prop(obj)
        obj.Elliptical = True
        obj.addProperty("App::PropertyAngle", "TangentError", "Dome",
                        "Largest angle to the neighbouring faces along the edges it can be tangent to")
        obj.addProperty("App::PropertyInteger", "SteepEdges", "Dome",
                        "Edges next to a wall too steep to be tangent to (the slope there is free)")
        for n in ("TangentError", "SteepEdges"):
            obj.setPropertyStatus(n, ["ReadOnly", "Output"])
        if "Refine" not in obj.PropertiesList:
            obj.addProperty("App::PropertyBool", "Refine", "Base", "Merge coplanar faces of the result")
        obj.Proxy = self

    def ellipse(self, obj, base, face, center, height, sign):
        """Dik duvara teget kubbe. Boolean olmaz, duz plaka yuzeyine donulur: False."""
        try:
            patches = ellipse_patches(face, base, center, height, sign)
            tool = ellipse_tool(face, patches)
            if not tool.isValid():
                return False
            res = None
            for fuzzy in (1e-5, 1e-3):
                r = rp._combine(base, tool, obj.Reversed, getattr(obj, "Refine", False), fuzzy)
                if r is not None and r[1]:
                    res = r[0]
                    break
        except Exception:
            return False
        if res is None:
            return False
        obj.TangentError = ellipse_error(face, base, patches)
        obj.SteepEdges = 0
        obj.AddSubShape = tool
        obj.Shape = res
        return True

    def field(self, obj, face, shape):
        """Ayni yuz icin cozum saklanir: Height ve surukleme 0.3 s yerine ~30 ms."""
        tangent = not obj.Reversed
        key = (shape.hashCode(), face.hashCode(), tangent)
        c = getattr(self, "_cache", None)
        if c is not None and c[0] == key:
            return c[1]
        fld = Field(face, shape, CELLS[0], tangent)
        if gap_of(fld.surface(1.0), face, fld) > GAP_OK:        # olcek 1 mm: bosluk h ile orantili
            fld = Field(face, shape, CELLS[1], tangent)
        self._cache = (key, fld)
        return fld

    def execute(self, obj):
        got = read_face(obj.Face)
        if isinstance(got, str):
            raise RuntimeError(T(got))
        base = rp._base(obj)
        if base is None or not base.Solids:
            raise RuntimeError(T("Dome: the Body has no solid yet"))
        face = same_face(base, got[1])
        if face is None:
            raise RuntimeError(T("Dome: the face is not on the previous solid; select it again on the last feature"))
        height = float(obj.Height)
        if height <= 0:
            raise RuntimeError(T("Dome: the height must be above zero"))
        sign = -1.0 if obj.Reversed else 1.0
        center = can_ellipse(face, base) if obj.Elliptical else None
        if center is not None:
            if self.ellipse(obj, base, face, center, height, sign):
                return
            App.Console.PrintWarning(T("Dome: can't follow the walls here, using a smooth bump with sharp edges.\n"))
        # kopya: pcurve ve tolerans kenarlara yerinde yazilir, onceki adimin sekli bozulmasin (Rail Pipe'taki gibi)
        work = base.copy()
        wface = work.Faces[next(i for i, f in enumerate(base.Faces) if f.isSame(face))]
        try:
            fld = self.field(obj, face, base)
            surf = fld.surface(height, sign)
            dome = trim(surf, wface)
            gap = gap_of(surf, face, fld)
            # kenarlar yuzeyden bu kadar uzak: duz kenarlarda BOP denetimi temiz gecer. fixTolerance degil: toleransi fix'in
            # buldugunun altina indirince delikli yuzde kati gecersiz oluyordu
            dome.limitTolerance(max(4.0 * gap, 1e-5), 1.0, Part.Edge)
        except Exception as e:
            raise RuntimeError(T("Dome: the surface failed (%s)") % e)
        if not dome.isValid():
            raise RuntimeError(T("Dome: the surface failed (%s)") % "invalid face")
        if gap > GAP_MAX * max(height, 1.0):
            raise RuntimeError(T("Dome: the outline has parts too narrow for the surface"))
        tool = lens(dome, wface)
        res = None
        if not collides(base, dome, face, sign, fld):
            res = replace_face(work, wface, dome)
        if res is None:                                    # baska bir yere degiyor: gercek boolean
            for fuzzy in (1e-3, 0.01):
                try:
                    r = rp._combine(base, tool, obj.Reversed, getattr(obj, "Refine", False), fuzzy)
                except Exception:
                    r = None
                if r is not None and r[1]:
                    res = r[0]
                    break
        if res is None:
            raise RuntimeError(T("Dome: the dome runs into another part of the solid; lower the height"))
        err, steep = tangent_error(dome, face, base, fld)
        obj.TangentError = err
        obj.SteepEdges = steep
        obj.AddSubShape = tool
        obj.Shape = res

    def onDocumentRestored(self, obj):
        elliptical_prop(obj)

    def dumps(self):
        return None

    def loads(self, state):
        return None


class ViewProviderDome:
    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.Object = vobj.Object

    def doubleClicked(self, vobj):
        import FreeCADGui as Gui
        Gui.ActiveDocument.setEdit(vobj.Object, 0)
        return True

    def setEdit(self, vobj, mode=0):
        if mode != 0:
            return None
        import pda_dome_gui
        pda_dome_gui.start(vobj.Object)
        return True

    def unsetEdit(self, vobj, mode=0):
        if mode != 0:
            return None
        import pda_dome_gui
        pda_dome_gui.stop()
        return True

    def getIcon(self):
        import os
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons", "pda_dome.svg")

    def dumps(self):
        return None

    def loads(self, state):
        return None


def make(body, target, sub):
    """Body'de yeni Dome. Yuz son govdede varsa ona baglanir. Nesne ya da hata metni (nesne olusturulmaz)."""
    got = read_face((target, [sub]))
    if isinstance(got, str):
        return got
    tip = body.Tip
    if tip is None or tip.Shape.isNull() or not tip.Shape.Solids:
        return "Dome: the Body has no solid yet"
    face = same_face(tip.Shape, got[1])
    if face is None:
        return "Dome: the face is not on the previous solid; select it again on the last feature"
    idx = next(i for i, f in enumerate(tip.Shape.Faces) if f.isSame(face))
    obj = body.newObject("PartDesign::FeatureAdditivePython", "Dome")
    Dome(obj)
    obj.Face = (tip, ["Face%d" % (idx + 1)])
    if App.GuiUp:
        ViewProviderDome(obj.ViewObject)
        rp._fine_display(obj)
    return obj


def is_dome(o):
    props = getattr(o, "PropertiesList", [])
    return "SteepEdges" in props and "TangentError" in props


def repair_proxies(doc):
    """Proxy'si kopmus Dome nesnelerini onar (FreeCAD bazen eklenti modulunu belge acilirken aktaramiyor)."""
    fixed = []
    for o in (doc.Objects if doc is not None else []):
        if is_dome(o) and not isinstance(getattr(o, "Proxy", None), Dome):
            o.Proxy = Dome.__new__(Dome)
            if App.GuiUp and not isinstance(getattr(o.ViewObject, "Proxy", None), ViewProviderDome):
                o.ViewObject.Proxy = ViewProviderDome.__new__(ViewProviderDome)
            o.touch()
            fixed.append(o)
    for o in fixed:
        o.recompute()
    return fixed
