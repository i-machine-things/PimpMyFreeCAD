# -*- coding: utf-8 -*-
# Pimp My FreeCAD - Magic Pad: tek aracta Pad ve Pocket. Copyright (c) 2026 Sedat Kuran - MIT License
#
# Uzunlugun isareti islemi belirler: pozitif (profilin disa bakan normali yonunde) malzeme ekler, negatif keser
# (push / pull). Modlar: Uzunluk, Iki uzunluk, Simetrik, Sonuna kadar (keser), Govdeye kadar (ilk temas: profilden
# cikan her nokta govdeye ilk degdigi yerde durur; ici bos silindir, dikis ve profilden dar yuz sorun degil). Egim (taper)
# ve twist (burulma). Nesne PartDesign::FeatureAddSubPython: Operation (Union / Subtraction) isarete gore degisir, desen
# ozellikleri dogru islemi gorur. Butun sekiller FreeCAD'in Python sekil islevleriyle kurulur (makeFace, extrude,
# makeLoft, fuse / cut): element map (topolojik adlandirma, FreeCAD 1.0+) korunur; olculdu, bkz. docs.
import math

import FreeCAD as App
import Part

import pda_railpipe as rp
from pda_i18n import T

V = App.Vector
PROPS = "TwistCenter"                                      # bu ozellik varsa nesne Magic Pad'dir (onarim)
MODES = ["Length", "TwoLengths", "Symmetric", "ThroughAll", "UpToBody", "UpToFirst", "UpToLast", "UpToFace",
         "UpToShape"]
AUTO = ("UpToFirst", "UpToLast", "UpToFace", "UpToShape")   # islem profilin onune gore: govdenin icine gidiyorsa keser
TARGET = ("UpToFace", "UpToShape")                         # hedef secilen modlar
CENTERS = ["Profile center", "Sketch origin"]


def _sketch(o):
    return rp._is_sketch(o)


def _centroid(shape):
    """Alan agirlikli merkez (tek yuz ya da yuz bileskesi)."""
    A, s = 0.0, V()
    for f in shape.Faces:
        A += f.Area
        s += f.CenterOfMass * f.Area
    return s * (1.0 / A) if A > 0 else shape.BoundBox.Center


def is_part(obj):
    """Part tezgahindaki Magic Pad mi: Body disinda, Part'in mantigiyla yalniz cekilen katiyi uretir (Extrude gibi;
    ekleme / kesme Part Fuse / Cut ile). Degilse PartDesign adimi."""
    return not obj.TypeId.startswith("PartDesign::")


def _to_local(obj, o):
    """o nesnesinin seklini obj'nin koordinatina getiren yerlesim (ayni Body'de birim). Part surumunde profil ve Base
    baska Body'lerde olabilir."""
    pl = App.Placement()
    g = o.getParentGeoFeatureGroup()
    if g is not None:
        pl = g.getGlobalPlacement()
    own = obj.getParentGeoFeatureGroup()
    if own is not None:
        pl = own.getGlobalPlacement().inverse().multiply(pl)
    return pl


def _moved_by(sh, pl):
    if pl.isIdentity():
        return sh
    sh = sh.copy()
    sh.transformShape(pl.toMatrix(), True)
    return sh


def _base(obj):
    """Ekleme / kesme yapilacak kati: PartDesign'da onceki adim. Part surumunde Reference: degistirilmez, yalniz
    yonu (disari / iceri) ve "kadar" modlarinin durdugu yeri belirler (yoksa None)."""
    if not is_part(obj):
        return rp._base(obj)
    b = getattr(obj, "Reference", None)
    if b is None or not hasattr(b, "Shape") or b.Shape.isNull():
        return None
    return _moved_by(b.Shape, _to_local(obj, b))


def profile(obj):
    """Profil: (yuz, merkez, dis normal, sketch orijini, govdenin uzerinde mi) ya da hata metni. Yuz, sketch'in
    kapali tellerinden (delikler dahil) ya da secilen duz yuzden; govde koordinatinda."""
    link = obj.Profile
    if not link or link[0] is None:
        return "Magic Pad: select a closed sketch or a flat face"
    o, subs = link[0], [s for s in (link[1] or []) if s]
    pl = _to_local(obj, o)
    fsubs = [s for s in subs if s.startswith("Face")]
    if fsubs:                                              # yuz(ler): her biri kendi normaliyle (bkz. _multi_tool)
        return _multi_profile(o, fsubs, pl, obj)
    try:
        if subs and subs[0].startswith("Face"):
            face = o.Shape.getElement(subs[0])
            origin = _centroid(face)
        else:
            wires = [w for w in o.Shape.Wires if w.isClosed()]
            if not wires:
                return "Magic Pad: the sketch has no closed profile"
            face = Part.makeFace(wires, "Part::FaceMakerBullseye")
            origin = o.Placement.Base if _sketch(o) else _centroid(face)
        face = _moved_by(face, pl)
        origin = pl.multVec(origin)
    except Exception as e:
        return T("Magic Pad: the profile could not be read (%s)") % e
    if face.isNull() or not face.Faces or face.Area <= 1e-12:
        return "Magic Pad: the profile has no area"
    f0 = max(face.Faces, key=lambda f: f.Area)              # birden cok ayri bolge: en buyugu yonu verir
    if any(not isinstance(f.Surface, Part.Plane) for f in face.Faces):
        return "Magic Pad: the profile must be flat"
    if subs and subs[0].startswith("Face"):
        u0, u1, v0, v1 = f0.ParameterRange
        n = f0.normalAt((u0 + u1) / 2, (v0 + v1) / 2)      # kati yuzu: yonelimi disari
        on_body = True
    else:
        n = pl.Rotation.multVec(o.Placement.Rotation.multVec(V(0, 0, 1))) if _sketch(o) else V(f0.Surface.Axis)
        on_body = False
    n.normalize()
    c = _centroid(face)
    base = _base(obj)
    if not on_body and base is not None:                   # yuze cizilmis sketch: normal govdenin disina
        q = _outward(face, c, n, base)
        if q is not None:
            n, on_body = q, True
        else:                                              # bosluktaki sketch: govde hangi yandaysa orasi "iceri"
            n = _away_from(face, c, n, base)
    return face, c, n, V(origin), on_body


def _multi_profile(o, fsubs, pl=None, obj=None):
    """Birden cok duz yuz (ornegin ardisik iki yan yuz): her biri kendi dis normaliyle. (yuzlerin bileskesi, ilk yuzun
    merkezi, ilk yuzun normali, ayni merkez, True, [(yuz, merkez, normal), ...]); gizmo ilk yuzde durur."""
    parts = []
    try:
        for sub in fsubs:
            f = _moved_by(o.Shape.getElement(sub), pl or App.Placement())
            if not isinstance(f.Surface, Part.Plane):
                return "Magic Pad: the profile must be flat"
            u0, u1, v0, v1 = f.ParameterRange
            n = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2)    # kati yuzu: yonelimi disari
            n.normalize()
            parts.append((f, _centroid(f), n))
    except Exception as e:
        return T("Magic Pad: the profile could not be read (%s)") % e
    f0, c0, n0 = parts[0]
    return Part.makeCompound([p[0] for p in parts]), c0, n0, V(c0), True, parts


DIRS = ["Normal", "Average", "Custom"]


def direction(obj, prof, n):
    """Cekme yonu: yuzun normali, secili yuzlerin ortalama normali ya da secilen kenar / yuz / eksen. Disa bakan
    normalle ayni yana cevrilir: pozitif uzunluk yine disari (ekler)."""
    mode = getattr(obj, "ExtrudeDir", "Normal")
    if mode == "Average" and len(prof) > 5:
        d = V()
        for p in prof[5]:
            d += p[2]
    elif mode == "Custom":
        d = custom_dir(obj)
        if d is None:
            return n
    else:
        return n
    if d.Length < 1e-9:
        return n
    d = V(d)
    d.normalize()
    if d.dot(n) < 0:
        d = -d
    return d


def custom_dir(obj):
    """Secilen yon referansi (duz kenar, duz yuz, datum cizgi / duzlem) govde koordinatinda birim vektor; yoksa None."""
    link = getattr(obj, "DirRef", None)
    if not link or link[0] is None:
        return None
    o, subs = link[0], [x for x in (link[1] or []) if x]
    try:
        sh = o.Shape.getElement(subs[0]) if subs else o.Shape
        pl = App.Placement()
        g = o.getParentGeoFeatureGroup()
        if g is not None:
            pl = g.getGlobalPlacement()
        body = obj.getParentGeoFeatureGroup()
        if body is not None:
            pl = body.getGlobalPlacement().inverse().multiply(pl)
        rot = pl.Rotation
        if sh.Edges and not sh.Faces:
            e = sh.Edges[0]
            if not isinstance(e.Curve, Part.Line):
                return None
            d = V(e.Curve.Direction)                       # datum ekseni sonsuz kenar: kosesi yok
        elif sh.Faces:
            f = sh.Faces[0]
            if not isinstance(f.Surface, Part.Plane):
                return None
            u0, u1, v0, v1 = f.ParameterRange
            d = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
        else:
            return None
        return rot.multVec(d)
    except Exception:
        return None


def _bent(obj, d, n):
    """Yon normalden farkliysa: egim / twist ve golgeye dayali modlar (govdeye / sekle kadar) kullanilamaz. Ozel yon
    secilmemisse ya da yuzun duzleminde kaliyorsa hata."""
    if getattr(obj, "ExtrudeDir", "Normal") == "Custom" and custom_dir(obj) is None:
        return "Magic Pad: select the direction (a straight edge, a flat face or a datum axis)"
    if abs(d.dot(n)) < 0.02:
        return "Magic Pad: the direction lies in the plane of the face"
    if (d - n).Length < 1e-7:
        return None
    if abs(float(obj.TaperAngle)) > 1e-9 or abs(float(obj.Twist)) > 1e-9:
        return "Magic Pad: taper and twist need the face normal as the direction"
    if obj.Mode in ("UpToBody", "UpToShape"):
        return "Magic Pad: Up to body / Up to shape need the face normal as the direction"
    return None


def _single(prof, part):
    f, c, n = part
    return f, c, n, V(c), True


def _corners(obj, parts, base):
    """Komsu secili yuzlerin ortak duz kenarlarinda iki prizma arasindaki kama (keskin birlesim, Fusion'daki gibi):
    kenarin bir ucunda P, P + n1 z, P + m, P + n2 z dortgeni kenar boyunca cekilir; m iki yuzun z kadar otelenmis
    duzlemlerinin kesisimi. Ic kosede prizmalar zaten ortusur, dortgen ortusmenin icinde kalir."""
    out = []
    sgn = -1.0 if obj.Reversed else 1.0
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            fi, ci, ni = parts[i]
            fj, cj, nj = parts[j]
            n1, n2 = ni * sgn, nj * sgn
            if n1.cross(n2).Length < 1e-6:                 # ayni duzlem: prizmalar zaten bitisik
                continue
            d = 1.0 + n1.dot(n2)
            if d < 0.05:                                   # neredeyse ters yuzler: sivri ucu sonsuza gider
                continue
            for e1 in fi.Edges:
                if not isinstance(e1.Curve, Part.Line):
                    continue
                if not any(e1.isSame(e2) for e2 in fj.Edges):
                    continue
                p0, p1 = e1.Vertexes[0].Point, e1.Vertexes[-1].Point
                for z in interval(obj, base, n1, ci):
                    if abs(z) < 1e-9:
                        continue
                    m = (n1 + n2) * (z / d)
                    poly = Part.makePolygon([p0, p0 + n1 * z, p0 + m, p0 + n2 * z, p0])
                    out.append(Part.Face(poly).extrude(p1 - p0))
    return out


def _follow(obj, k, parts, base, tool):
    """Yan duvar komsuyu izlesin (Fusion'daki yuz itme gibi): secili yuzun secili olmayan duz bir komsuyla ortak duz
    kenarinda, prizmanin dik yan duvari yerine komsunun duzlemi sinir olur. Komsu disa yatiksa prizmadan ucgen kesilir
    (cikinti kalmaz), ice yatiksa ucgen eklenir (kesmede ince serit kalmaz)."""
    if base is None or base.isNull() or getattr(obj, "SideFaces", "Neighbours") != "Neighbours":
        return tool
    f, c, nf, n = parts[k]                                 # yuz, merkez, yuz normali, cekme yonu
    sel = [p[0] for p in parts]
    sgn = -1.0 if obj.Reversed else 1.0
    n = n * sgn
    zs = [z for z in interval(obj, base, n, c) if abs(z) > 1e-9]
    for e in f.Edges:
        if not isinstance(e.Curve, Part.Line):
            continue
        p0, p1 = e.Vertexes[0].Point, e.Vertexes[-1].Point
        ed = p1 - p0
        if ed.Length < 1e-9:
            continue
        for g_face in base.ancestorsOfType(e, Part.Face) if _has_edge(base, e) else []:
            if g_face.isSame(f) or any(g_face.isSame(x) for x in sel):
                continue
            if not isinstance(g_face.Surface, Part.Plane):
                continue
            u0, u1, v0, v1 = g_face.ParameterRange
            g = g_face.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
            w = ed.cross(nf)                               # yuz duzleminde, kenara dik
            w.normalize()
            if w.dot(c - p0) > 0:                          # yuzun disina bakan, kenara dik yon
                w = -w
            if abs(w.dot(g)) < 1e-9:
                continue
            for z in zs:
                d = n * (1.0 if z > 0 else -1.0)
                h = abs(z)
                if abs(d.dot(g)) < 1e-9:                   # komsu prizmaya dik: zaten uyuyor
                    continue
                t = -h * d.dot(g) / w.dot(g)
                q = p0 + d * h + w * t
                tri = Part.Face(Part.makePolygon([p0, p0 + d * h, q, p0])).extrude(ed)
                tool = tool.fuse(tri) if t > 0 else tool.cut(tri)
    return tool


def _has_edge(shape, e):
    return any(x.isSame(e) for x in shape.Edges)


def _multi_tool(obj, prof, base, mode, trim):
    """Her yuz kendi normali boyunca; araclar birlestirilir, komsu yuzler arasina kama eklenir (yalniz uzunluk
    modlarinda, egim / twist yokken)."""
    tools = []
    parts = prof[5]
    for k, (f, c, n) in enumerate(prof[5]):
        sub = (f, c, n, V(c), True, prof[5])               # tek yuz; ortalama yon icin butun yuzler yaninda
        if mode in AUTO or (mode == "UpToBody" and base is not None and trim):
            t = _upto(obj, sub, base, mode)
        else:
            t = build_tool(obj, sub, base)
        if isinstance(t, str):
            return ("%s (%s %d)" % (T(t), T("face"), k + 1)) if len(prof[5]) > 1 else t
        tools.append(t)
    plain = mode in ("Length", "TwoLengths", "Symmetric") and abs(float(obj.TaperAngle)) < 1e-9 \
        and abs(float(obj.Twist)) < 1e-9
    if plain:
        moved = [(f, c, n, direction(obj, prof, n)) for f, c, n in parts]   # her yuzun gercek cekme yonu
        tools = [_follow(obj, k, moved, base, t) for k, t in enumerate(tools)]
        if all((m[3] - m[2]).Length < 1e-7 for m in moved):
            tools += _corners(obj, parts, base)            # ortak yonde yuzler arasinda bosluk olmaz
    tool = tools[0]
    for t in tools[1:]:
        tool = tool.fuse(t)
    if len(tools) > 1:
        try:
            tool = tool.removeSplitter()
        except Exception:
            pass
    return tool


def _away_from(face, c, n, base):
    """Govdeye degmeyen ya da govdenin icinden gecen sketch: normal, profilin hemen onunde govde olan yandan obur yana
    cevrilir (govdenin altindaki XY duzleminde sketch: asagi ekler, yukari keser). Iki yan da bossa / doluysa degismez."""
    try:
        e = 1e-3 * max(face.BoundBox.DiagonalLength, 1.0)
        pts = [c] + [v.Point for v in face.Vertexes][:8]
        fw = sum(1 for p in pts if face.isInside(p, 1e-6, True) and base.isInside(p + n * e, 1e-7, False))
        bw = sum(1 for p in pts if face.isInside(p, 1e-6, True) and base.isInside(p - n * e, 1e-7, False))
        if fw > bw:
            return -n
    except Exception:
        pass
    return n


def _outward(face, c, n, base):
    """Profil govdenin duz bir yuzunun uzerindeyse o yuzun dis normali, yoksa None. Yalniz profille ayni duzlemdeki
    yuzlere bakilir: Rail Pipe'in ornek nokta + mesafe denetimi 400 delikli plakada 2.5 s suruyordu."""
    bb = face.BoundBox
    tol = 1e-6 * max(bb.DiagonalLength, 1.0)
    for bf in base.Faces:
        s = bf.Surface
        if not isinstance(s, Part.Plane):
            continue
        a = V(s.Axis)
        if abs(abs(a.dot(n)) - 1.0) > 1e-7 or abs((V(s.Position) - c).dot(n)) > max(tol, 1e-7):
            continue
        if not bf.BoundBox.intersect(bb):
            continue
        try:
            # profilin ornek noktalari yuzun icinde mi (2B common delikli buyuk yuzde 0.3 s suruyordu)
            pts = [c] + [v.Point for v in face.Vertexes][:24]
            if sum(1 for p in pts if bf.isInside(p, max(tol, 1e-6), True)) < 0.5 * len(pts):
                continue
            u0, u1, v0, v1 = bf.ParameterRange
            m = bf.normalAt((u0 + u1) / 2, (v0 + v1) / 2)   # kati yuzu: yonelimi disari
        except Exception:
            continue
        m.normalize()
        return m
    return None


def operation(obj, prof=None):
    """Union ya da Subtraction: Sonuna kadar hep keser, Govdeye kadar hep ekler; obur modlarda gercek yon: profilin dis
    normali boyunca ekler, katinin icine dogru keser. Ters yon (Reversed) de yonu cevirir: Reversed + pozitif uzunluk
    iceri gider, keser. Ilk / son yuze, yuze, sekle kadar: profilin hemen onu govdenin icindeyse keser, disindaysa
    ekler (iki taraf arasindaki bosluk doldurulur)."""
    if is_part(obj):                                       # Part: boolean yok, hep cekilen kati
        return "Union"
    forced = getattr(obj, "OpMode", "Auto")               # elle secim: uc durumlar icin
    if forced == "Add":
        return "Union"
    if forced == "Cut":
        return "Subtraction"
    mode = obj.Mode
    if mode == "ThroughAll":
        return "Subtraction"
    if mode == "UpToBody":
        return "Union"
    if mode in AUTO:
        return _auto_op(obj, prof)
    s = (-1.0 if obj.Reversed else 1.0) * (-1.0 if float(obj.Length) < 0 else 1.0)
    return "Subtraction" if s < 0 else "Union"


def _auto_op(obj, prof=None):
    try:
        if prof is None:
            prof = profile(obj)
        if isinstance(prof, str):
            return "Union"
        face, c, n, origin, on_body = prof[:5]
        n = direction(obj, prof, n)
        n = -n if obj.Reversed else n
        base = _base(obj)
        if base is None or base.isNull():
            return "Union"
        e = 1e-3 * max(face.BoundBox.DiagonalLength, 1.0)
        pts = [c] + [v.Point for v in face.Vertexes][:12]
        pts = [p for p in pts if face.isInside(p, 1e-6, True)] or [c]
        inside = sum(1 for p in pts if base.isInside(p + n * e, 1e-7, False))
        return "Subtraction" if inside * 2 > len(pts) else "Union"
    except Exception:
        return "Union"


def interval(obj, base, n, c):
    """Profil duzleminden itibaren normal boyunca [z0, z1] (mm)."""
    L = float(obj.Length)
    mode = obj.Mode
    if mode == "Symmetric":
        return -abs(L) / 2, abs(L) / 2
    if mode == "TwoLengths":
        L2 = float(obj.Length2)
        a, b = L, (-L2 if L >= 0 else L2)
        return min(a, b), max(a, b)
    if mode in AUTO:
        return 0.0, _far(base, c, n)
    if mode in ("ThroughAll", "UpToBody"):
        far = _far(base, c, n)
        if mode == "ThroughAll":                           # hep katinin icine (keser); Ters yon obur yana cevirir.
            return -far, 0.0                               # Once uzunlugun isaretine bakiyordu: varsayilan 10 mm ile
                                                           # disari gidip hicbir sey kesmiyordu
        return 0.0, (abs(L) if abs(L) > 1e-9 else far)
    return min(0.0, L), max(0.0, L)


def _far(base, c, n):
    """Govdenin profil merkezinden normal boyunca (her iki yana) en uzak noktasi + pay."""
    if base is None or base.isNull():
        return 100.0
    bb = base.BoundBox
    d = 0.0
    for x in (bb.XMin, bb.XMax):
        for y in (bb.YMin, bb.YMax):
            for z in (bb.ZMin, bb.ZMax):
                d = max(d, abs((V(x, y, z) - c).dot(n)))
    return d + max(1.0, 0.05 * bb.DiagonalLength)


def _ease(t, on):
    return t * t * (3 - 2 * t) if on else t


def _section(wire, k, outer, z, t, obj, n, center):
    """Bir kesit: egim (yaricap degisimi), twist (eksen etrafinda donus), z kadar oteleme."""
    w = wire
    taper = float(obj.TaperAngle)
    if abs(taper) > 1e-9 and abs(z) > 1e-12:
        off = -math.tan(math.radians(taper)) * abs(z) * (1 if outer else -1)
        w = wire.makeOffset2D(off, 2, False, False, False)  # join 2: keskin koseler
        if len(w.Wires) != 1:
            raise ValueError(T("Magic Pad: the taper is too large for the profile"))
        w = w.Wires[0]
    else:
        w = wire.copy()
    tw = float(obj.Twist)
    if abs(tw) > 1e-9:
        w.rotate(center, n, tw * _ease(t, bool(obj.TwistEase)))
    w.translate(n * z)
    return w


def build_tool(obj, prof, base):
    """Eklenecek / kesilecek kati (govde koordinatinda, element map'li) ya da hata metni. Govdeye kadar: kirpilmadan
    once (bkz. up_to_body)."""
    face, c, n, origin, on_body = prof[:5]
    d = direction(obj, prof, n)
    err = _bent(obj, d, n)
    if err:
        return err
    n = d
    if obj.Reversed:
        n = -n
    z0, z1 = interval(obj, base, n, c)
    if z1 - z0 < 1e-7:
        return "Magic Pad: the length is zero"
    try:
        return _solid(obj, face, c, n, origin, z0, z1)
    except Exception as e:
        return T("Magic Pad: the shape failed (%s)") % e


def _solid(obj, face, c, n, origin, z0, z1, span=None):
    """Profilden [z0, z1] arasi kati: duz extrude, egimli loft ya da twist. span: twist'in dagildigi aralik (Govdeye
    kadar: ilk temas derinligi; otesinde aci sabit). Hata verirse istisna firlatir."""
    tw, taper = float(obj.Twist), float(obj.TaperAngle)
    if abs(tw) < 1e-9 and abs(taper) < 1e-9:
        f = face.copy()
        f.translate(n * z0)
        return f.extrude(n * (z1 - z0))
    center = origin if obj.TwistCenter == "Sketch origin" else c
    center = center + n * (center - c).dot(n) * -1.0     # merkez profil duzleminde
    tools = []
    for f in face.Faces:                               # her ayri bolge: dis tel eksi delikler
        wires = [f.OuterWire] + [w for w in f.Wires if not w.isSame(f.OuterWire)]
        solids = []
        for k, wire in enumerate(wires):
            if abs(tw) < 1e-9:                         # yalniz egim: iki kesitli duz loft (saglam, element map'li)
                secs = [_section(wire, k, k == 0, z, t, obj, n, center) for z, t in ((z0, 0.0), (z1, 1.0))]
                solids.append(Part.makeLoft(secs, True, True, False))
            else:
                solids.append(_twist_solid(wire, k == 0, z0, z1, obj, n, center, span))
        tool = solids[0]
        for s in solids[1:]:
            tool = tool.cut(s)
        tools.append(tool)
    tool = tools[0]
    for t in tools[1:]:
        tool = tool.fuse(t)
    return tool


def _twist_solid(wire, outer, z0, z1, obj, n, center, span=None):
    """Twistli kati: eksen boyunca duz omurga + profili istenen aciyla donduren yardimci egri (pipe shell, FreeCAD'in
    Sweep'i gibi). makeLoft burada guvenilmezdi (olculdu): tek parca kapali B-spline profilde yumusatma kapaliyken hacmin
    %0.3'u cikiyordu, 30 mm ustu bostu; pipe shell'de butun kesitler profil alaniyla %0.2 icinde. Egim: kesitler egimle
    icerilmis ve donmus olarak verilir. Egim profil duzleminden iki yana (simetrik): iki yarim ayri kurulur (uc kesit
    'uncompatible wires' veriyordu). Not: pipe shell element map uretmez."""
    Z0, Z1 = span if span else (z0, z1)
    if abs(float(obj.TaperAngle)) > 1e-9 and z0 < -1e-9 and z1 > 1e-9:
        a = _pipe_part(wire, outer, z0, 0.0, Z0, Z1, obj, n, center)
        b = _pipe_part(wire, outer, 0.0, z1, Z0, Z1, obj, n, center)
        s = a.fuse(b).removeSplitter()
        return s.Solids[0] if len(s.Solids) == 1 else s
    return _pipe_part(wire, outer, z0, z1, Z0, Z1, obj, n, center)


def _pipe_part(wire, outer, za, zb, Z0, Z1, obj, n, center, N=48):
    """[za, zb] arasinda pipe shell; donus acisi butun aralik [Z0, Z1] uzerinden (yarimlar ayni twist'i surdurur)."""
    tw = float(obj.Twist)
    ease = bool(obj.TwistEase)
    tz = lambda z: min(1.0, max(0.0, (z - Z0) / (Z1 - Z0)))
    u, _ = _frame(n)
    r = max(wire.BoundBox.DiagonalLength, 1.0)
    spine = Part.Wire(Part.LineSegment(center + n * za, center + n * zb).toShape())
    pts = []
    for i in range(N + 1):
        z = za + (zb - za) * i / float(N)
        pts.append(center + n * z + App.Rotation(n, tw * _ease(tz(z), ease)).multVec(u * r))
    aux = Part.BSplineCurve()
    aux.interpolate(pts)
    mk = Part.BRepOffsetAPI.MakePipeShell(spine)
    mk.setAuxiliarySpine(Part.Wire(aux.toShape()), False, 0)
    zs = [za, zb] if abs(float(obj.TaperAngle)) > 1e-9 else [za]
    for z in zs:
        mk.add(_section(wire, 0, outer, z, tz(z), obj, n, center), False, False)
    mk.build()
    mk.makeSolid()
    s = mk.shape()
    if not s.Solids:
        raise ValueError(T("Magic Pad: the twist gave no solid"))
    return s.Solids[0]


def _frame(n):
    u = V(1, 0, 0) if abs(n.x) < 0.9 else V(0, 1, 0)
    u = u - n * u.dot(n)
    u.normalize()
    return u, n.cross(u)


def silhouette(base, c, n, tol):
    """Govdenin profil duzleminin onunde (normal yonunde) kalan kisminin duzleme gole (shapely), ekran agindan:
    ucgenlerin izdusumlerinin birlesimi. (golge, u, w) ya da golge bossa (None, u, w)."""
    import numpy as np
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    u, w = _frame(n)
    P, tri = base.tessellate(tol)
    A = np.array([tuple(p) for p in P])
    C = np.array(tuple(c))
    z = (A - C).dot(np.array(tuple(n)))
    x = (A - C).dot(np.array(tuple(u)))
    y = (A - C).dot(np.array(tuple(w)))
    polys = []
    for a, b, d in tri:
        if (z[a] + z[b] + z[d]) / 3.0 <= tol:
            continue
        p = Polygon([(x[a], y[a]), (x[b], y[b]), (x[d], y[d])])
        if p.area > 1e-12:
            polys.append(p)
    if not polys:
        return None, u, w
    return unary_union(polys), u, w


def _face_from(geom, c, u, w):
    """shapely alani -> profil duzleminde duz yuz(ler) (bilesik)."""
    faces = []
    for p in (geom.geoms if hasattr(geom, "geoms") else [geom]):
        if p.is_empty or p.area <= 1e-12:
            continue
        rings = [p.exterior] + list(p.interiors)
        wires = [Part.makePolygon([c + u * x + w * y for x, y in r.coords]) for r in rings]
        faces.append(Part.makeFace(wires, "Part::FaceMakerBullseye"))
    return Part.makeCompound(faces) if faces else None


def up_to_body(obj, face, c, n, base, origin=None):
    """Govdeye kadar (ilk temas). Profil, govdenin duzleme dusen golgesiyle ikiye ayrilir: degen kisim uzun prizma olarak
    cikarilir, govdeyle bolunur ve yalniz profile bagli parca (profil ile ilk duvar arasi) tutulur; duvarin ici, ic
    bosluk ve otesi atilir. Golge ekran agindan ve tol kadar iceri alinarak bulunur: degmeyen bir sutun yaninda gecip
    ustteki parcayi profile baglayamaz (genis profil + silindir: olculdu). Degmeyen kisim Miss'e gore kesilir ya da
    Length'e kadar gider. (arac) ya da hata metni."""
    size = max(base.BoundBox.DiagonalLength, 1.0)
    tol = min(0.05, max(1e-3, 2e-4 * size))
    sil, u, w = silhouette(base, c, n, tol)
    if sil is None:
        return "Magic Pad: Up to body: the Body is not in front of the profile"
    sil = sil.buffer(-2 * tol)
    sf = _face_from(sil, c, u, w) if not sil.is_empty else None
    if sf is None:
        return "Magic Pad: Up to body: the profile does not reach the Body"
    hit = face.common(sf)
    miss = face.cut(sf)
    parts = []
    if origin is None:
        origin = c
    if hit.Faces:
        tool, reach = _long(obj, hit, c, n, origin, base)
        rest = tool.cut(base)
        if abs(float(obj.TaperAngle)) > 1e-9 or abs(float(obj.Twist)) > 1e-9:
            # egim / twist govdenin golgesinden tasabilir: tasan kisim govdenin yanindan asagi inmesin
            rest = rest.common(sf.extrude(n * _far(base, c, n)))
        parts += _touching(rest, face)
    L = abs(float(obj.Length))
    if getattr(obj, "Miss", "Cut") == "Length" and miss.Faces and miss.Area > 1e-9 and L > 1e-9:
        parts.append(_solid(obj, miss, c, n, origin, 0.0, L))
    if not parts:
        return "Magic Pad: Up to body: the profile does not reach the Body"
    tool = parts[0]
    for p in parts[1:]:
        tool = tool.fuse(p)
    return tool


def _touching(shape, face):
    """Profile degen parcalar (katilar)."""
    out = []
    for s in shape.Solids:
        try:
            if s.distToShape(face)[0] <= 1e-6:
                out.append(s)
        except Exception:
            out.append(s)
    return out


def _depth(solids, c, n):
    """Katilarin profil duzleminden normal boyunca en derin noktasi (egri yuz icin ag noktalari)."""
    d = 0.0
    for s in solids:
        try:
            d = max(d, max((q - c).dot(n) for q in s.tessellate(0.05)[0]))
        except Exception:
            d = max(d, max((v.Point - c).dot(n) for v in s.Vertexes))
    return d


def _long(obj, face, c, n, origin, base, far=None, split=None):
    """(kati, egimsiz temas derinligi): hedefin otesine uzanan kati; egim ve twist dahil. split(kati): hedefle
    bolunmus parcalar (varsayilan: govdeden cikarma). Daralan egimde profil uzak mesafede kapanabilir: o zaman boy,
    egimsiz ilk temasa kadar kisaltilarak denenir."""
    if far is None:
        far = _far(base, c, n)
    if split is None:
        split = lambda t: t.cut(base)
    if abs(float(obj.TaperAngle)) < 1e-9 and abs(float(obj.Twist)) < 1e-9:
        return _solid(obj, face, c, n, origin, 0.0, far), 0.0
    reach = 0.0                                        # egimsiz ilk temas derinligi (alt sinir)
    try:
        reach = _depth(_touching(split(face.extrude(n * far)), face), c, n)
    except Exception:
        pass
    z, last = far, None
    while True:
        try:
            span = (0.0, reach) if reach > 1e-6 and reach < z else None   # twist ilk temasa kadar tamamlanir
            return _solid(obj, face, c, n, origin, 0.0, z, span), reach
        except Exception as e:
            last = e
        if z <= reach * 1.02 + 1e-6:
            raise last
        z = max(reach * 1.02 + 1e-6, z * 0.7)


def target_shape(obj):
    """Secilen hedef (yuz ya da kati), bu Magic Pad'in govdesinin koordinatinda; yoksa None. Hedef baska bir govdede
    olabilir: iki govdenin yerlesimi hesaba katilir."""
    link = getattr(obj, "UpToTarget", None)
    if not link or link[0] is None:
        return None
    o, subs = link[0], [x for x in (link[1] or []) if x]
    sh = o.Shape.getElement(subs[0]) if subs else o.Shape
    sh = sh.copy()
    pl = App.Placement()
    g = o.getParentGeoFeatureGroup()
    if g is not None:
        pl = g.getGlobalPlacement()
    body = obj.getParentGeoFeatureGroup()
    if body is not None:
        pl = body.getGlobalPlacement().inverse().multiply(pl)
    if not pl.isIdentity():
        sh = _moved(sh, pl)
    return sh


def _moved(sh, pl):
    sh = sh.copy()
    sh.transformShape(pl.toMatrix(), True)
    return sh


def _facing_arc(f, s, n, v, c=None, size=1e4):
    """Silindir / koni: yuzun cekme yonune bakan tarafi ile ayni tarafta kalan u yayi (u0, u1), ya da None (tam cevre).
    Yuz hangi yandan vuruluyorsa (dis bukey: on yuz; ic bukey, ornegin icinden bakilan boru: arka duvar) o yarim
    tutulur: tam cevre, profile bakan sanal yarimi ile araci gercek yuzden once kesiyordu (ici bos obround'un icindeki
    sketch'te Up to first face: duvar 175'te, arac 69'da duruyordu; olculdu). Taraf, c'den n yonundeki isinin gercek
    yuzu ilk vurdugu noktadan; isin yuzu vurmuyorsa yuzun ortasindan (tam cevreli yuzde orta rastgele bir yandir)."""
    u0, u1, v0, v1 = f.ParameterRange
    um = (u0 + u1) / 2.0
    if c is not None:
        try:
            sec = Part.LineSegment(c, c + n * size).toShape().section(f)
            ts = [(p.Point - c).dot(n) for p in sec.Vertexes]
            ts = [t for t in ts if t > 1e-9]
            if ts:
                um = s.parameter(c + n * min(ts))[0]
        except Exception:
            pass
    m = s.normal(um, v)
    k = m.dot(n)
    if abs(k) < 1e-3:                                      # yon eksene paralel ya da yuz yana bakiyor: tam cevre
        return None
    sg = 1.0 if k > 0 else -1.0
    N = 720
    step = 2 * math.pi / N
    good = lambda u: s.normal(u, v).dot(n) * sg > 1e-9
    a = b = um
    for _ in range(N // 2):
        if not good(a - step):
            break
        a -= step
    for _ in range(N // 2):
        if not good(b + step):
            break
        b += step
    if b - a >= 2 * math.pi - 2 * step:
        return None
    return a, b


def extended(f, size, c, n=None):
    """Yuzun uzatilmis hali: duzlem sinirsiz duzlem gibi, silindir / koni ekseni boyunca, kure / tor tam yuzey; obur
    yuzeyler (B-spline) kirpilmamis tam yuzeyleri. n (cekme yonu) verilirse silindir / koni cevresi yalniz yuzun
    vuruldugu yarimla sinirlanir (_facing_arc)."""
    s = f.Surface
    try:
        if isinstance(s, Part.Plane):
            u, v = s.parameter(c)
            return s.toShape(u - size, u + size, v - size, v + size)
        if isinstance(s, (Part.Cylinder, Part.Cone)):
            u, v = s.parameter(c)
            vmin, vmax = v - size, v + size
            if isinstance(s, Part.Cone):                   # tepeyi gecmesin
                apex_v = -s.Radius / math.sin(s.SemiAngle) if abs(math.sin(s.SemiAngle)) > 1e-9 else -1e9
                if apex_v < v:
                    vmin = max(vmin, apex_v + 1e-6)
                else:
                    vmax = min(vmax, apex_v - 1e-6)
            arc = _facing_arc(f, s, n, v, c, 4 * size) if n is not None else None
            ua, ub = arc if arc else (0.0, 2 * math.pi)
            return s.toShape(ua, ub, vmin, vmax)
        if isinstance(s, (Part.Sphere, Part.Toroid)):
            return s.toShape()
        u0, u1, v0, v1 = s.bounds()
        if all(abs(x) < 1e50 for x in (u0, u1, v0, v1)):
            return s.toShape(u0, u1, v0, v1)
    except Exception:
        pass
    return f.copy()


def _slicer(sheet):
    """Katiyi bir yuzeyle bolen islev (General Fuse: yuzey katiyi tam kesmese de kesen kisimlar ayrilir)."""
    def split(t):
        import BOPTools.SplitAPI as SA
        return SA.slice(t, [sheet], "Split")
    return split


def _upto_split(obj, face, c, n, origin, far, split, what):
    """Uzun araci split ile bolup profile bagli parcayi tutar; parca hedefe varmadan uca kadar gidiyorsa hata."""
    tool, reach = _long(obj, face, c, n, origin, None, far, split)
    parts = _touching(split(tool), face)
    if not parts or _depth(parts, c, n) >= far - 1e-3:
        return T("Magic Pad: %s: the target is not in front of the profile (try Reversed)") % T(what)
    tool = parts[0]
    for p in parts[1:]:
        tool = tool.fuse(p)
    return tool


def up_to_face(obj, face, c, n, base, origin, tf, what="Up to face"):
    """Secilen (uzatilmis) yuzeye kadar: profilden cikan arac yuzeyle bolunur, profile bagli parca tutulur."""
    far = max(_far(base, c, n), _far(tf, c, n))
    sheet = extended(tf, 4 * far, c, n)
    return _upto_split(obj, face, c, n, origin, far, _slicer(sheet), what)


def first_face(face, c, n, base):
    """Profilden normal boyunca govdede ilk degilen yuz (ornek noktalardan en yakin vurus)."""
    far = _far(base, c, n)
    size = max(face.BoundBox.DiagonalLength, 1.0)
    pts = [c] + [v.Point for v in face.Vertexes][:16]
    try:
        pts += [V(*q) for q in face.tessellate(size / 8.0)[0]][:48]
    except Exception:
        pass
    best = None
    e = 1e-6 * max(far, 1.0)
    for p in pts:
        if not face.isInside(p, 1e-6, True):
            continue
        try:
            sec = Part.LineSegment(p + n * e, p + n * far).toShape().section(base)
        except Exception:
            continue
        for v in sec.Vertexes:
            d = (v.Point - p).dot(n)
            if d > 1e-6 and (best is None or d < best[0]):
                best = (d, v.Point)
    if best is None:
        return None
    q = best[1]
    return min(base.Faces, key=lambda f: f.distToShape(Part.Vertex(q))[0])


def up_to_last(obj, face, c, n, base, origin):
    """Govdenin icinden gecip her noktada govdeden son ciktigi yere kadar: uzun aractan, govdenin otesindeki (uca
    degen) bos parcalar cikarilir."""
    far = _far(base, c, n)
    split = lambda t: t.cut(base)
    tool, reach = _long(obj, face, c, n, origin, base, far, lambda t: t.common(base))
    beyond = [s for s in split(tool).Solids if _depth([s], c, n) >= far - 1e-3]
    if len(beyond) and all(abs(s.Volume - tool.Volume) < 1e-6 * max(tool.Volume, 1.0) for s in beyond):
        return "Magic Pad: Up to last: the Body is not in front of the profile (try Reversed)"
    for s in beyond:
        tool = tool.cut(s)
    if not tool.Solids:
        return "Magic Pad: Up to last: the Body is not in front of the profile (try Reversed)"
    return tool


def combine(base, tool, op, refine):
    """Govde + arac: (sonuc) ya da hata metni. Kopyalanmaz, bolunmez: element map korunur."""
    if base is None or base.isNull():
        if op == "Subtraction":
            return "Magic Pad: nothing to cut (the Body is empty)"
        return tool
    last = None
    # tam gecerlilik denetimi (isValid) yok: PartDesign Pad de yapmiyor, 400 delikli plakada 0.7 s suruyordu (Part'in
    # CheckModel ayari yalniz Part boolean'lari icin). Bos / katisiz sonuc reddedilir, hata verirse fuzzy ile denenir.
    check = False
    for fuzzy in (0.0, 1e-3, 0.01):
        try:
            if op == "Subtraction":
                r = base.cut(tool, fuzzy) if fuzzy else base.cut(tool)
            else:
                r = base.fuse(tool, fuzzy) if fuzzy else base.fuse(tool)
            if refine:
                r = r.removeSplitter()
            if r.Solids and (not check or r.isValid()):
                return r
            last = "invalid result"
        except Exception as e:
            last = str(e)
    return T("Magic Pad: the boolean with the Body failed (%s)") % last


def compute(obj, trim=True, prof=None):
    """(arac, govde, islem) ya da hata metni. trim=False: Govdeye kadar kirpilmaz (onizleme surukleme). prof: onceden
    okunmus profil (duzenleyici bir kez okur)."""
    if prof is None:
        prof = profile(obj)
    if isinstance(prof, str):
        return T(prof)
    base = _base(obj)
    op = operation(obj, prof)
    mode = obj.Mode
    if len(prof) > 5:                                      # birden cok yuz
        tool = _multi_tool(obj, prof, base, mode, trim)
    elif mode in AUTO or (mode == "UpToBody" and base is not None and trim):
        tool = _upto(obj, prof, base, mode)
    else:
        tool = build_tool(obj, prof, base)
    if isinstance(tool, str):
        return T(tool)
    return tool, base, op


def _upto(obj, prof, base, mode):
    face, c, n, origin = prof[0], prof[1], prof[2], prof[3]
    d = direction(obj, prof, n)
    err = _bent(obj, d, n)
    if err:
        return err
    n = -d if obj.Reversed else d
    names = {"UpToBody": "Up to body", "UpToFirst": "Up to first face", "UpToLast": "Up to last face",
             "UpToFace": "Up to face", "UpToShape": "Up to shape"}
    off = offset(obj)
    sh = (lambda s: _shifted(s, n, off)) if abs(off) > 1e-9 else (lambda s: s)
    try:
        if mode in TARGET:
            tgt = target_shape(obj)
            if tgt is None or tgt.isNull():
                return "Magic Pad: select the target (Up to face / Up to shape)"
            if mode == "UpToShape":
                if not tgt.Solids:
                    return "Magic Pad: Up to shape: the target has no solid; use Up to face"
                return up_to_body(obj, face, c, n, sh(tgt), origin)
            tf = tgt.Faces[0] if tgt.Faces else None
            if tf is None:
                return "Magic Pad: Up to face: the target is not a face"
            return up_to_face(obj, face, c, n, base, origin, sh(tf))
        if base is None or base.isNull():
            return T("Magic Pad: %s needs a solid in the Body") % T(names[mode])
        if mode == "UpToBody":
            return up_to_body(obj, face, c, n, sh(base), origin)
        if mode == "UpToLast":
            return up_to_last(obj, face, c, n, sh(base), origin)
        ff = first_face(face, c, n, base)
        if ff is None:
            return "Magic Pad: Up to first: the Body is not in front of the profile (try Reversed)"
        return up_to_face(obj, face, c, n, base, origin, sh(ff), "Up to first face")
    except Exception as e:
        return T("Magic Pad: %s failed (%s)") % (T(names[mode]), e)


def offset(obj):
    """Yukari kadar modlarinda hedefin kaydirilmasi (mm): arti hedefin otesine gecer, eksi hedeften once durur."""
    try:
        return float(getattr(obj, "Offset", 0.0))
    except Exception:
        return 0.0


def _shifted(sh, n, d):
    """Hedef (yuz ya da kati) cekme yonu boyunca d kadar kaydirilmis kopya: arac kaydirilmis hedefe kadar gider. Govde
    kaydirilinca bile boolean gercek govdeyle yapilir (compute'taki base degismez)."""
    sh = sh.copy()
    sh.translate(n * d)
    return sh


MISS = ["Cut", "Length"]
OPMODES = ["Auto", "Add", "Cut"]
SIDES = ["Neighbours", "Perpendicular"]


PART_MODES = [m for m in MODES if m != "ThroughAll"]     # Part: kesme yok, Sonuna kadar anlamsiz


def part_props(obj):
    """Part surumu: referans kati (degistirilmez) ve kesmeye ait olmayan mod listesi."""
    if not is_part(obj):
        return
    if "Reference" not in obj.PropertiesList:
        obj.addProperty("App::PropertyLinkGlobal", "Reference", "Magic Pad",
                        "Solid the profile belongs to (not changed): gives the outward side and where the Up to modes "
                        "stop. The result is only the pulled solid; add or cut it with Part Fuse / Cut")
    if obj.Mode == "ThroughAll":
        obj.Mode = "Length"
    if list(obj.getEnumerationsOfProperty("Mode")) != PART_MODES:
        m = obj.Mode
        obj.Mode = PART_MODES
        obj.Mode = m if m in PART_MODES else "Length"
    for prop in ("OpMode",):
        if prop in obj.PropertiesList:
            obj.setEditorMode(prop, 2)


def miss_prop(obj):
    if "ExtrudeDir" not in obj.PropertiesList:
        obj.addProperty("App::PropertyEnumeration", "ExtrudeDir", "Magic Pad",
                        "Normal: each face along its own normal; Average: all selected faces along their mean normal; "
                        "Custom: along the selected straight edge, flat face or datum axis (DirRef)")
        obj.ExtrudeDir = DIRS
        obj.addProperty("App::PropertyLinkSubGlobal", "DirRef", "Magic Pad",
                        "Custom direction: a straight edge, a flat face or a datum line / plane")
    if "SideFaces" not in obj.PropertiesList:
        obj.addProperty("App::PropertyEnumeration", "SideFaces", "Magic Pad",
                        "Selected faces: the side walls follow the neighbouring faces (like pushing a face), or stand "
                        "perpendicular to the face (like Pad)")
        obj.SideFaces = SIDES
    if "OpMode" not in obj.PropertiesList:
        obj.addProperty("App::PropertyEnumeration", "OpMode", "Magic Pad",
                        "Auto: the direction decides (into the solid cuts, out of it adds); Add / Cut: forced")
        obj.OpMode = OPMODES
    if "UpToTarget" not in obj.PropertiesList:
        obj.addProperty("App::PropertyLinkSubGlobal", "UpToTarget", "Magic Pad",
                        "Up to face: the face to stop at; Up to shape: the solid to stop at")
    if "Miss" not in obj.PropertiesList:
        obj.addProperty("App::PropertyEnumeration", "Miss", "Magic Pad",
                        "Up to body: the part of the profile that does not meet the Body is cut off, or goes to Length")
        obj.Miss = MISS
    if "Offset" not in obj.PropertiesList:
        obj.addProperty("App::PropertyDistance", "Offset", "Magic Pad",
                        "Up to modes: shifts where the feature stops; positive goes past the target, negative stops "
                        "before it")


class MagicPad:
    """Magic Pad (PartDesign::FeatureAddSubPython) Proxy'si."""

    def __init__(self, obj):
        link = "App::PropertyLinkSubGlobal" if is_part(obj) else "App::PropertyLinkSub"   # Part: baska Body'lerden de
        obj.addProperty(link, "Profile", "Magic Pad", "Closed sketch or flat face")
        obj.addProperty("App::PropertyEnumeration", "Mode", "Magic Pad",
                        "Length: positive adds, negative cuts. Through all cuts, Up to body stops where the "
                        "profile first meets the Body")
        obj.Mode = MODES
        obj.addProperty("App::PropertyDistance", "Length", "Magic Pad",
                        "Signed length along the profile's outward normal: positive adds material, negative cuts")
        obj.Length = 10.0
        obj.addProperty("App::PropertyLength", "Length2", "Magic Pad", "Second length, to the other side (Two lengths)")
        obj.addProperty("App::PropertyBool", "Reversed", "Magic Pad", "Flip the outward direction")
        obj.addProperty("App::PropertyAngle", "TaperAngle", "Magic Pad",
                        "Taper: positive narrows away from the profile, negative widens")
        obj.addProperty("App::PropertyAngle", "Twist", "Magic Pad", "Total turn of the profile along the length")
        obj.addProperty("App::PropertyEnumeration", "TwistCenter", "Magic Pad", "Axis of the twist")
        obj.TwistCenter = CENTERS
        obj.addProperty("App::PropertyBool", "TwistEase", "Magic Pad", "Slow turn at the start and the end")
        miss_prop(obj)
        part_props(obj)
        obj.Proxy = self

    def onDocumentRestored(self, obj):
        miss_prop(obj)
        part_props(obj)

    def onChanged(self, obj, prop):
        if prop in ("Length", "Mode", "Reversed", "OpMode") and "Operation" in obj.PropertiesList \
                and hasattr(obj, "Reversed") and (obj.Mode not in AUTO or getattr(obj, "OpMode", "Auto") != "Auto"):
            try:
                op = operation(obj)
            except Exception:
                return
            if obj.Operation != op:
                obj.Operation = op

    def execute(self, obj):
        res = compute(obj)
        if isinstance(res, str):
            raise RuntimeError(res)
        tool, base, op = res
        if "Operation" in obj.PropertiesList and obj.Operation != op:   # eski dosya / dogrudan degisen ozellik
            obj.Operation = op
        if is_part(obj):
            r = tool                                       # Part: yalniz cekilen kati (Extrude gibi)
        else:
            r = combine(base, tool, op, bool(getattr(obj, "Refine", False)))
        if isinstance(r, str):
            raise RuntimeError(r)
        if hasattr(obj, "AddSubShape"):
            obj.AddSubShape = tool
        obj.Shape = r

    def dumps(self):
        return None

    def loads(self, state):
        return None


class ViewProviderMagicPad:
    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.Object = vobj.Object

    def claimChildren(self):
        obj = getattr(self, "Object", None)
        if obj is None or not obj.Profile:
            return []
        return rp.claim_inputs(obj, [obj.Profile[0]])

    def doubleClicked(self, vobj):
        import FreeCADGui as Gui
        Gui.ActiveDocument.setEdit(vobj.Object, 0)
        return True

    def setEdit(self, vobj, mode=0):
        if mode != 0:
            return None
        import pda_magicpad_gui
        pda_magicpad_gui.start(vobj.Object)
        return True

    def unsetEdit(self, vobj, mode=0):
        if mode != 0:
            return None
        import pda_magicpad_gui
        pda_magicpad_gui.stop()
        return True

    def getIcon(self):
        import os
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")
        cut = getattr(self, "Object", None) is not None and getattr(self.Object, "Operation", "") == "Subtraction"
        return os.path.join(d, "pda_magicpad_cut.svg" if cut else "pda_magicpad.svg")

    def dumps(self):
        return None

    def loads(self, state):
        return None


def make(body, prof_obj, subs=None):
    """Body'de yeni Magic Pad: nesne ya da hata metni (nesne olusturulmaz)."""
    obj = body.newObject("PartDesign::FeatureAddSubPython", "MagicPad")
    MagicPad(obj)
    obj.Profile = (prof_obj, list(subs or []))
    if App.GuiUp:
        ViewProviderMagicPad(obj.ViewObject)
    if prof_obj.Shape.isNull() or "Touched" in prof_obj.State:
        prof_obj.recompute()
    p = profile(obj)
    if isinstance(p, str):
        body.removeObject(obj)
        body.Document.removeObject(obj.Name)
        return p
    obj.Refine = App.ParamGet("User parameter:BaseApp/Preferences/Mod/PartDesign").GetBool("RefineModel", True)
    return obj


def make_part(doc, prof_obj, subs=None):
    """Part tezgahinda Magic Pad: Body disinda, yalniz cekilen kati (Part'in mantigi: ekleme / kesme Part Fuse / Cut
    ile). Yuz secildiyse o yuzun katisi Reference olur (PartDesign ozelligiyse butun Body): disari yonu ve "kadar"
    modlari ona gore; degistirilmez, gizlenmez. Nesne ya da hata metni."""
    obj = doc.addObject("Part::FeaturePython", "MagicPad")
    MagicPad(obj)
    obj.Profile = (prof_obj, list(subs or []))
    if subs:
        ref = prof_obj
        g = prof_obj.getParentGeoFeatureGroup()
        if g is not None and g.isDerivedFrom("PartDesign::Body"):
            ref = g
        obj.Reference = ref
    if App.GuiUp:
        ViewProviderMagicPad(obj.ViewObject)
    if prof_obj.Shape.isNull() or "Touched" in prof_obj.State:
        prof_obj.recompute()
    p = profile(obj)
    if isinstance(p, str):
        doc.removeObject(obj.Name)
        return p
    return obj


def is_magicpad(o):
    return PROPS in getattr(o, "PropertiesList", []) and "Profile" in o.PropertiesList


def repair_proxies(doc):
    fixed = []
    for o in (doc.Objects if doc is not None else []):
        if is_magicpad(o) and not isinstance(getattr(o, "Proxy", None), MagicPad):
            o.Proxy = MagicPad.__new__(MagicPad)
            if App.GuiUp and not isinstance(getattr(o.ViewObject, "Proxy", None), ViewProviderMagicPad):
                o.ViewObject.Proxy = ViewProviderMagicPad.__new__(ViewProviderMagicPad)
            o.touch()
            fixed.append(o)
    for o in fixed:
        o.recompute()
    return fixed
