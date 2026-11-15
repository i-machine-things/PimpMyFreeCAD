# -*- coding: utf-8 -*-
# Pimp My FreeCAD - Body Sweep: tek supurme nesnesi, birden fazla kesilen govde. Copyright (c) 2026 Sedat Kuran - MIT License
#
# BodySweep (Part::FeaturePython, belgenin kokunde): bir alet katisi (herhangi bir Body / Part katisi) yol boyunca
# supurulur; sekli supurulmus zarftir (dunya koordinatinda). Zarf aletin kopyalarinin boolean'i ile degil kesin
# kesitlerden dogrudan kurulur (pda_bodysweep.run). Hesap dakikalar surebildiginden kati saklanir (SweptShape) ve
# yalniz girdiler degisince yeniden kurulur (InputKey); panel bunu arka planda yapar.
# Kesilen govdeler (Targets): her PartDesign Body'de zarfi kullanan bir BodySweepCut adimi (PartDesign
# FeatureSubtractivePython, yalniz kesme: hizli), Body olmayan katida Part Cut. Zarf bir kez hesaplanir.
import hashlib
import os

import FreeCAD as App
import Part

import pda_railpipe as rp
from pda_i18n import T

ORIENT = ["Follow path", "World Z"]
METHODS = ["Fast", "Exact"]
TWIST_MAX = 360.0
ICONS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")


# --- girdiler (dunya koordinatinda) ---
def tool_shape(obj):
    o = obj.Tool
    if o is None:
        return "Body Sweep: select the tool solid"
    try:
        sh = Part.getShape(o)
    except Exception:
        sh = None
    if sh is None or sh.isNull() or not sh.Solids:
        return "Body Sweep: the tool has no solid"
    if len(sh.Solids) != 1:
        return "Body Sweep: the tool must be one solid"
    sol = sh.Solids[0]
    off = float(getattr(obj, "Offset", 0.0))
    if abs(off) > 1e-9:
        # bosluk: aletin ofseti supurulur. Supurmenin ofseti, ofsetin supurmesidir (alet her yonde off kadar buyur):
        # kanal aletten off kadar genis, kesin. Yuvarlak: kenar / koseler off yaricapla (gercek esit uzaklik);
        # keskin: yuzler kaydirilip kesistirilir (koseler sivri kalir, koselerde bosluk off'tan biraz fazla)
        join = 0 if getattr(obj, "OffsetJoin", "Round") == "Round" else 2
        try:
            o2 = sol.makeOffsetShape(off, 1e-4 * max(sol.BoundBox.DiagonalLength, 1.0), join=join, fill=False)
            o2 = o2.Solids[0] if o2.Solids else Part.Solid(Part.Shell(o2.Faces))
        except Exception:
            o2 = None
        if o2 is None or o2.isNull() or not o2.isValid() or o2.Volume <= 0:
            return T("Body Sweep: the tool could not be offset by %.3f mm") % off
        sol = o2
    return sol


def path_edges(obj):
    """Yol kenarlari ya da hata metni. Sketch / tel: butun kenarlari; alt eleman: secilenler."""
    edges = []
    for o, subs in obj.Path:
        subs = [subs] if isinstance(subs, str) else list(subs or [])
        subs = [s for s in subs if s]
        try:
            if subs:
                edges += [Part.getShape(o, s, needSubElement=True) for s in subs if s.split(".")[-1].startswith("Edge")]
            else:
                edges += Part.getShape(o).Edges
        except Exception:
            return "Body Sweep: the path could not be read"
    if not edges:
        return "Body Sweep: select the path (edges or a sketch)"
    if len(Part.sortEdges(edges)) != 1:
        return "Body Sweep: the path edges must form one chain"
    return edges


def support_face(obj):
    if obj.SupportFace is None:
        return None
    o, subs = obj.SupportFace
    subs = [subs] if isinstance(subs, str) else list(subs or [])
    try:
        f = Part.getShape(o, subs[0], needSubElement=True) if subs and subs[0] else Part.getShape(o).Faces[0]
    except Exception:
        return None
    return f if f is not None and not f.isNull() else None


def method_prop(obj):
    """Eski dosyalar: Method ozelligi Exact ile eklenir (kayitli zarf gecerli kalir); yeni nesnede varsayilan Fast."""
    if "Method" not in obj.PropertiesList:
        obj.addProperty("App::PropertyEnumeration", "Method", "Body Sweep",
                        "Fast: copies of the tool, plane sections and their outer boundary (seconds; sharp edges of "
                        "the tool are slightly rounded, ~0.5 mm). Exact: exact sections and corner tracks (convex tools, "
                        "minutes)")
        obj.Method = METHODS
        obj.Method = "Exact"
    if "Offset" not in obj.PropertiesList:
        obj.addProperty("App::PropertyDistance", "Offset", "Body Sweep",
                        "Clearance: the swept volume grows by this much on every side (e.g. a channel wider than the "
                        "part that runs through it); negative shrinks it")
        obj.addProperty("App::PropertyEnumeration", "OffsetJoin", "Body Sweep",
                        "Round: edges and corners of the clearance are rounded (exact distance everywhere). Sharp: the "
                        "faces are moved and intersected (sharp corners)")
        obj.OffsetJoin = ["Round", "Sharp"]


def params(obj):
    return dict(method="exact" if getattr(obj, "Method", "Exact") == "Exact" else "fast",
                orient="path" if obj.Orientation == ORIENT[0] else "z",
                twist_deg=max(-TWIST_MAX, min(TWIST_MAX, float(obj.Twist))),
                sec_tol=max(float(obj.Tolerance), 1e-3), fit_tol=max(float(obj.Tolerance), 1e-3))


def _geo_key(sh):
    """Seklin geometrisinin ozeti (BREP metni degil: belge yeniden acilinca ayni sekil farkli metin veriyordu)."""
    r = lambda x: "%.5f" % x
    out = [sh.ShapeType, r(sh.Volume if sh.Solids else 0.0), r(sh.Area if sh.Faces else 0.0), r(sh.Length)]
    out += sorted(",".join(r(c) for c in v.Point) for v in sh.Vertexes)
    for e in sh.Edges:
        out.append(";".join(",".join(r(c) for c in q) for q in e.discretize(Number=17)))
    return "|".join(out)


def inputs(obj):
    """(alet, yol kenarlari, destek yuzu, parametreler, anahtar) ya da hata metni. Anahtar: girdilerin ozeti; saklanan
    supurulmus kati ancak ayni anahtarla gecerli."""
    tool = tool_shape(obj)
    if isinstance(tool, str):
        return tool
    edges = path_edges(obj)
    if isinstance(edges, str):
        return edges
    sup = support_face(obj)
    p = params(obj)
    h = hashlib.md5()
    for s in [tool] + edges + ([sup] if sup is not None else []):
        h.update(_geo_key(s).encode())
    h.update(repr(sorted(p.items())).encode())
    return tool, edges, sup, p, h.hexdigest()


def compute(tool, edges, sup, p, progress=None, cancel=None):
    """Supurulmus kati ya da hata metni (iptalde "cancelled"). Arka plan is parcaciginda da cagrilabilir."""
    p = dict(p)
    if p.pop("method", "exact") == "fast":
        import pda_bodysweep_fast as fs
        return fs.run(tool, edges, sup, progress=progress, cancel=cancel, **p)
    import pda_bodysweep as bs
    out = bs.run(None, tool, edges, None, sup, show=False, verify=False, progress=progress, cancel=cancel, **p) or {}
    if "error" in out:
        return out["error"]
    res = out.get("res")
    if res is None or res.isNull() or not res.Solids:
        return "Body Sweep: the sweep gave no solid"
    return res.Solids[0] if len(res.Solids) == 1 else res


def is_stale(obj):
    """Saklanan zarf girdilere uymuyor mu (hesap gerekir)."""
    got = inputs(obj)
    return isinstance(got, str) or got[4] != obj.InputKey or obj.SweptShape.isNull()


# --- supurme nesnesi ---
class BodySweep:
    """BodySweep (Part::FeaturePython): supurulmus zarf."""

    def __init__(self, obj):
        obj.addProperty("App::PropertyLinkGlobal", "Tool", "Body Sweep", "The solid swept along the path (any Body / Part)")
        obj.addProperty("App::PropertyLinkSubListGlobal", "Path", "Body Sweep", "Path: edges, a wire or a sketch")
        obj.addProperty("App::PropertyEnumeration", "Orientation", "Body Sweep",
                        "Follow path: the tool turns with the path (rotation-minimizing frame, also on vertical and 3D "
                        "paths). World Z: the tool's up stays toward Z")
        obj.Orientation = ORIENT
        obj.addProperty("App::PropertyLinkSubGlobal", "SupportFace", "Body Sweep",
                        "Optional face: the tool stays normal to it (overrides Orientation)")
        obj.addProperty("App::PropertyAngle", "Twist", "Body Sweep",
                        "Total turn of the tool about the path along its length, -360 to 360 degrees")
        obj.addProperty("App::PropertyLength", "Tolerance", "Body Sweep",
                        "Largest deviation of the envelope: smaller is more exact and slower (0.2 mm for a quick draft)")
        obj.Tolerance = 0.02
        method_prop(obj)
        obj.Method = "Fast"
        obj.addProperty("App::PropertyLinkListGlobal", "Targets", "Body Sweep",
                        "Bodies / solids cut by the sweep (managed by the panel)")
        obj.setPropertyStatus("Targets", "ReadOnly")
        obj.addProperty("Part::PropertyPartShape", "SweptShape", "Body Sweep", "Stored swept solid")
        obj.addProperty("App::PropertyString", "InputKey", "Body Sweep", "Inputs the stored swept solid was made from")
        obj.addProperty("App::PropertyBool", "Pending", "Body Sweep", "Not computed yet (new object, panel open)")
        for n in ("SweptShape", "InputKey", "Pending"):
            obj.setPropertyStatus(n, ["Hidden", "Output"])
        obj.Proxy = self

    def execute(self, obj):
        if obj.Pending:                                    # yeni nesne: panel arka planda hesaplar
            obj.Shape = obj.SweptShape.copy() if not obj.SweptShape.isNull() else Part.Shape()
            return
        got = inputs(obj)
        if isinstance(got, str):
            raise RuntimeError(T(got))
        tool, edges, sup, p, key = got
        if key != obj.InputKey or obj.SweptShape.isNull():
            App.Console.PrintMessage(T("Body Sweep: the inputs changed, computing the sweep (this can take minutes)...\n"))
            swept = compute(tool, edges, sup, p)
            if isinstance(swept, str):
                raise RuntimeError(T("Body Sweep: the sweep failed (%s)") % swept)
            obj.SweptShape = swept
            obj.InputKey = key
        obj.Shape = obj.SweptShape.copy()

    def onDocumentRestored(self, obj):
        method_prop(obj)

    def dumps(self):
        return None

    def loads(self, state):
        return None


class ViewProviderBodySweep:
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
        import pda_bodysweep_gui
        pda_bodysweep_gui.start(vobj.Object)
        return True

    def unsetEdit(self, vobj, mode=0):
        if mode != 0:
            return None
        import pda_bodysweep_gui
        pda_bodysweep_gui.stop()
        return True

    def getIcon(self):
        return os.path.join(ICONS, "pda_bodysweep.svg")

    def dumps(self):
        return None

    def loads(self, state):
        return None


# --- Body'deki kesme adimi ---
class BodySweepCut:
    """BodySweepCut (PartDesign FeatureSubtractivePython): zarfi kendi Body'sinden keser."""

    def __init__(self, obj):
        obj.addProperty("App::PropertyLinkGlobal", "Sweep", "Body Sweep", "The Body Sweep whose envelope is cut")
        if "Refine" not in obj.PropertiesList:
            obj.addProperty("App::PropertyBool", "Refine", "Base", "Merge coplanar faces of the result")
        obj.Refine = True
        obj.Proxy = self

    def execute(self, obj):
        base = rp._base(obj)
        sw = obj.Sweep
        if base is None:
            raise RuntimeError(T("Body Sweep: the Body has no solid to cut"))
        if sw is None or sw.Shape.isNull() or not sw.Shape.Solids:
            obj.Shape = base                               # zarf henuz hesaplanmadi: govde degismez
            return
        swept = sw.Shape.copy()
        body = obj.getParentGeoFeatureGroup()
        if body is not None:                               # zarf dunya koordinatinda: Body'nin kendi koordinatina
            swept.transformShape(body.getGlobalPlacement().inverse().toMatrix())
        res = None
        for fuzzy in (1e-3, 0.01, 0.0):
            try:
                res = rp._combine(base, swept, True, bool(obj.Refine), fuzzy)
            except Exception:
                res = None
            if res is not None:
                break
        if res is None:
            raise RuntimeError(T("Body Sweep: the cut from the Body failed"))
        obj.AddSubShape = swept
        obj.Shape = res[0]

    def dumps(self):
        return None

    def loads(self, state):
        return None


class ViewProviderBodySweepCut:
    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.Object = vobj.Object

    def doubleClicked(self, vobj):                         # duzenleme supurme nesnesinde
        import FreeCADGui as Gui
        sw = vobj.Object.Sweep
        if sw is not None:
            Gui.ActiveDocument.setEdit(sw, 0)
        return True

    def getIcon(self):
        return os.path.join(ICONS, "pda_bodysweepcut.svg")

    def dumps(self):
        return None

    def loads(self, state):
        return None


# --- hedefler ---
def body_of(o):
    """Nesnenin PartDesign Body'si (Body'nin kendisi de olabilir) ya da None."""
    if o is None:
        return None
    if o.TypeId == "PartDesign::Body":
        return o
    b = o.getParentGeoFeatureGroup() if hasattr(o, "getParentGeoFeatureGroup") else None
    return b if b is not None and b.TypeId == "PartDesign::Body" else None


def cuts_of(sweep):
    """Supurmeyi kullanan kesme nesneleri: BodySweepCut adimlari ve Part Cut'lar."""
    out = []
    for o in sweep.InList:
        if "Sweep" in o.PropertiesList and getattr(o, "Sweep", None) is sweep:
            out.append(o)
        elif o.TypeId == "Part::Cut" and o.Tool is sweep:
            out.append(o)
    return out


def target_of(cut):
    return cut.Base if cut.TypeId == "Part::Cut" else body_of(cut)


def sync_targets(sweep):
    """Targets listesine gore kesme nesnelerini kur / sil: Body'ye BodySweepCut adimi (ucuna), katiya Part Cut."""
    doc = sweep.Document
    want = []
    for t in sweep.Targets:                                # ayni govde iki kez kesilmez
        if t is not None and t not in want:
            want.append(t)
    have = {}
    for c in cuts_of(sweep):
        have.setdefault(target_of(c), []).append(c)
    for t, cs in list(have.items()):
        if t in want:
            continue
        for c in cs:
            if c.TypeId == "Part::Cut":
                if c.Base is not None and App.GuiUp:
                    c.Base.ViewObject.Visibility = True
            else:
                b = body_of(c)
                if b is not None:
                    b.removeObject(c)
            doc.removeObject(c.Name)
    made = []
    for t in want:
        if t in have:
            continue
        b = body_of(t)
        if b is not None:
            c = b.newObject("PartDesign::FeatureSubtractivePython", "BodySweepCut")
            BodySweepCut(c)
            c.Sweep = sweep
            if App.GuiUp:
                ViewProviderBodySweepCut(c.ViewObject)
                rp._fine_display(c)
        else:
            c = doc.addObject("Part::Cut", "BodySweepCut")
            c.Base, c.Tool = t, sweep
        made.append(c)
    return made


def make(doc, tool=None, path=None, targets=None):
    """Yeni Body Sweep (hesaplanmamis: panel hesaplar ve hedefleri baglar)."""
    obj = doc.addObject("Part::FeaturePython", "BodySweep")
    BodySweep(obj)
    obj.Pending = True
    if tool is not None:
        obj.Tool = tool
    if path:
        obj.Path = path
    if targets:
        obj.Targets = targets
    if App.GuiUp:
        ViewProviderBodySweep(obj.ViewObject)
        rp._fine_display(obj)
    return obj


def classify(selection):
    """Secimden alet, yol ve kesilecekler. Sira: once alet katisi, sonra yol (sketch / kenarlar), sonra kesilecek
    govdeler. (alet ya da None, yol listesi, hedef listesi)."""
    tool, path, targets = None, [], []
    for so in selection:
        o = so.Object
        edges = [s for s in so.SubElementNames if s.split(".")[-1].startswith("Edge")]
        if rp._is_sketch(o) or (hasattr(o, "Shape") and not o.Shape.Faces and o.Shape.Edges):
            path.append((o, ""))
        elif edges:
            path += [(o, s) for s in edges]
        elif hasattr(o, "Shape") and o.Shape.Solids:
            o = body_of(o) or o
            if tool is None:
                tool = o
            elif o is not tool and o not in targets:
                targets.append(o)
    return tool, path, targets


def repair_proxies(doc):
    fixed = []
    for o in (doc.Objects if doc is not None else []):
        if "SweptShape" in o.PropertiesList and "InputKey" in o.PropertiesList \
                and not isinstance(getattr(o, "Proxy", None), BodySweep):
            o.Proxy = BodySweep.__new__(BodySweep)
            if App.GuiUp:
                o.ViewObject.Proxy = ViewProviderBodySweep.__new__(ViewProviderBodySweep)
            fixed.append(o)
        elif "Sweep" in o.PropertiesList and o.TypeId == "PartDesign::FeatureSubtractivePython" \
                and not isinstance(getattr(o, "Proxy", None), BodySweepCut):
            o.Proxy = BodySweepCut.__new__(BodySweepCut)
            if App.GuiUp:
                o.ViewObject.Proxy = ViewProviderBodySweepCut.__new__(ViewProviderBodySweepCut)
            fixed.append(o)
    return fixed
