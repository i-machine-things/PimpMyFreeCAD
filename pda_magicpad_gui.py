# -*- coding: utf-8 -*-
# Pimp My FreeCAD - Magic Pad: ekranda duzenleme ve gorev paneli. Copyright (c) 2026 Sedat Kuran - MIT License
#
# Onizlemede boolean yok: yalniz eklenecek / kesilecek hacim yari saydam cizilir (ekleme yesil, kesme kirmizi; kesme
# katinin icinden de gorunur). Uzunluk, FreeCAD'in Pad gizmosuna benzeyen okla cekilir (surukleme Python'da): disari
# ekler, iceri keser. Govdeye kadar: surukerken duz prizma, birakinca ilk temasa kirpilmis hali. Paneldeki sayilar
# FreeCAD'in QuantitySpinBox'i: ifade (fx) baglanabilir. Kati yalniz OK'te hesaplanir; OK tek geri alma adimi.
import math

import FreeCAD as App
import FreeCADGui as Gui
import Part
from pivy import coin
from PySide import QtCore, QtWidgets

import pda_common as sm
import pda_magicpad as mp
from pda_i18n import T

V = App.Vector
_editor = None
fresh = None                                               # komutun yeni olusturdugu nesne: Cancel onu da geri alir
ADD_RGB, CUT_RGB = (0.15, 0.80, 0.45), (0.90, 0.20, 0.20)
GIZMO_PX = 80.0                                            # ok boyu (ekranda piksel, yaklasik)
MODE_TEXT = {"Length": "Length", "TwoLengths": "Two lengths", "Symmetric": "Symmetric", "ThroughAll": "Through all",
             "UpToBody": "Up to body", "UpToFirst": "Up to first face", "UpToLast": "Up to last face",
             "UpToFace": "Up to face", "UpToShape": "Up to shape"}


def start(obj):
    global _editor
    stop()
    _editor = Editor(obj)


def stop():
    global _editor
    if _editor is not None:
        e, _editor = _editor, None
        e.close()


def sm_graphics_view(view):
    """3B gorunumun QGraphicsView'i (SubD Modeler'daki gibi): silinmis eski sarmalayici onbellekten dusurulur."""
    gv = view.graphicsView()
    try:
        import shiboken6 as shiboken
    except ImportError:
        try:
            import shiboken2 as shiboken
        except ImportError:
            return gv
    if not shiboken.isValid(gv):
        shiboken.invalidate(gv)
        gv = view.graphicsView()
    return gv


class _ValueEdit(QtWidgets.QLineEdit):
    """Okun yanindaki yazinin yerinde acilan sayi alani: Enter / Return uygular, Esc vazgecer. QDoubleSpinBox degil:
    FreeCAD 27.1 (Qt 6.11) altinda dugmesiz + stil verilmis spin box'in ic yazi alani 1 piksele cokuyordu (olculdu),
    kutu bos ve siyah gorunuyordu. Birim de yazilabilir ("1 in", "2 cm"); birimsiz sayi mm."""

    def __init__(self, parent, done):
        super(_ValueEdit, self).__init__(parent)
        self.done = done
        self.closed = False
        self.setStyleSheet("QLineEdit { background: #18181c; color: #ff8c28; border: 1px solid #ff8c28; "
                           "border-radius: 4px; padding: 2px 6px; font-weight: bold; "
                           "selection-background-color: #ff8c28; selection-color: #18181c; }")

    def setValue(self, v):
        try:
            self.setText(App.Units.Quantity(float(v), App.Units.Length).UserString)
        except Exception:
            self.setText("%.2f mm" % float(v))

    def value(self):
        """Yazilan deger (mm) ya da okunamazsa None."""
        t = self.text().strip()
        if not t:
            return None
        try:
            q = App.Units.Quantity(t)
            if q.Unit == App.Units.Unit():                 # birimsiz: mm
                return float(q.Value)
            return float(q.getValueAs("mm"))
        except Exception:
            pass
        try:
            return float(t.lower().replace("mm", "").replace(",", ".").strip())
        except Exception:
            return None

    def _finish(self, value):
        if not self.closed:
            self.closed = True
            self.done(value)

    def keyPressEvent(self, ev):
        k = ev.key()
        if k in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            v = self.value()
            if v is None:                                  # okunamadi: alan acik kalir, cerceve kirmizi
                self.setStyleSheet(self.styleSheet().replace("1px solid #ff8c28", "1px solid #d9534f"))
                return
            self._finish(v)
            return
        if k == QtCore.Qt.Key_Escape:
            self._finish(None)
            return
        super(_ValueEdit, self).keyPressEvent(ev)

    # odak kaybinda kapanmaz: yaziya tiklayip tusu birakinca odak 3B gorunume donup alani hemen kapatiyordu.
    # Gorunumde baska yere tiklamak alani kapatir (Gizmo._btn).


def has_expression(obj, prop):
    try:
        return any(p == prop for p, _ in obj.ExpressionEngine)
    except Exception:
        return False


class Gizmo(object):
    """Uzunluk oku: FreeCAD'in Pad gizmosuna benzer (golgeli govde + koni, eksende ince taban cizgisi), surukleme
    tamamen Python'da. FreeCAD'in C++ SoLinearDraggerContainer'i Python'dan kullanilinca FreeCAD iki kez coktu
    (olculdu): kullanilmaz. Ok, eksen uzerinde 'deger' noktasindan (uzunlugun ucu) yon boyunca ekranda sabit boyda."""

    def __init__(self, view, parent, on_move, on_finish, on_value=None, on_flip=None):
        self.view, self.parent = view, parent
        self.on_move, self.on_finish, self.on_value = on_move, on_finish, on_value
        self.on_flip = on_flip                             # oka cift tiklama: yon degisir
        self.last_click = None                             # (zaman, konum): cift tiklamayi Coin vermiyor, burada bulunur
        self.text = ""
        self.lab = None                                    # (veri, w, h) yazi resmi
        self.lab_hot = False
        self.edit = None
        self.ok = True
        self.origin = self.dir = None
        self.value = 0.0
        self.size = 1.0
        self.hot = False
        self.value_for_edit = 0.0
        self.drag = None
        self.rgb = ADD_RGB
        self.root = coin.SoSwitch()
        self.root.ref()
        ann = coin.SoAnnotation()                          # derinlik testi yok: katinin icinde de gorunur
        self.pos = coin.SoTranslation()
        self.rot = coin.SoRotation()
        self.scl = coin.SoScale()
        self.mat = coin.SoMaterial()
        self.mat.specularColor = (0.4, 0.4, 0.4)
        self.mat.shininess = 0.6
        shaft_t = coin.SoTranslation()
        shaft_t.translation = (0, 0.5, 0)
        shaft = coin.SoCylinder()
        shaft.radius = 0.03
        shaft.height = 1.0
        cone_t = coin.SoTranslation()
        cone_t.translation = (0, 0.61, 0)
        cone = coin.SoCone()
        cone.bottomRadius = 0.1
        cone.height = 0.22
        knob = coin.SoSphere()
        knob.radius = 0.06
        for n in (self.pos, self.rot, self.scl, self.mat, knob, shaft_t, shaft, cone_t, cone):
            ann.addChild(n)
        # deger yazisi: okun yaninda turuncu kutu (SubD Modeler'in olcu yazisi gibi); tiklayinca deger yazilir
        lab = coin.SoAnnotation()
        lps = coin.SoPickStyle()
        lps.style = coin.SoPickStyle.UNPICKABLE
        self.lab_tr = coin.SoTranslation()
        self.lab_img = coin.SoImage()
        self.lab_img.horAlignment = coin.SoImage.CENTER
        self.lab_img.vertAlignment = coin.SoImage.HALF
        ldb = coin.SoDepthBuffer()                         # yazi her zaman okun onunde (aci ne olursa olsun)
        ldb.test = False
        ldb.write = False
        for n in (lps, ldb, self.lab_tr, self.lab_img):
            lab.addChild(n)
        grp = coin.SoGroup()
        grp.addChild(ann)
        grp.addChild(lab)
        self.root.addChild(grp)
        self.root.whichChild = -1
        parent.addChild(self.root)
        self.cb_btn = view.addEventCallbackPivy(coin.SoMouseButtonEvent.getClassTypeId(), self._btn)
        self.cb_mov = view.addEventCallbackPivy(coin.SoLocation2Event.getClassTypeId(), self._mov)

    def _color(self):
        c = (1.0, 0.85, 0.2) if (self.hot or self.drag is not None) else self.rgb
        self.mat.diffuseColor = c
        self.mat.emissiveColor = tuple(0.25 * x for x in c)

    def place(self, origin, direction, value, rgb, text=""):
        """origin: profil duzlemindeki eksen noktasi (dunya); direction: birim yon; value: okun eksendeki yeri;
        text: okun yanindaki deger yazisi."""
        self.origin, self.dir, self.value, self.rgb = origin, direction, value, rgb
        p = origin + direction * value
        self.pos.translation = (p.x, p.y, p.z)
        self.rot.rotation.setValue(coin.SbRotation(coin.SbVec3f(0, 1, 0),
                                                   coin.SbVec3f(direction.x, direction.y, direction.z)))
        self._color()
        if text != self.text:
            self.text = text
            self._label_image()
        self.update_scale(force=True)

    def _label_image(self):
        """Yazi kutusu resmi: koyu zemin, turuncu cerceve ve yazi (SubD'nin 'handle' olcusu); imlec ustundeyse acik."""
        from PySide import QtGui
        if not self.text:
            self.lab = None
            self.lab_img.image.setValue(coin.SbVec2s(0, 0), 4, b"")
            return
        try:
            dpr = sm_graphics_view(self.view).viewport().devicePixelRatioF()
        except Exception:
            dpr = 1.0
        f = QtGui.QFont()
        f.setPixelSize(max(int(round(15 * dpr)), 9))
        f.setBold(True)
        fm = QtGui.QFontMetrics(f)
        pad = int(round(5 * dpr))
        w, h = 2 * pad + fm.horizontalAdvance(self.text), fm.height() + pad
        qi = QtGui.QImage(w, h, QtGui.QImage.Format_ARGB32)
        qi.fill(0)
        col = QtGui.QColor(255, 200, 90) if self.lab_hot else QtGui.QColor(255, 140, 40)
        p = QtGui.QPainter(qi)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setFont(f)
        p.setBrush(QtGui.QColor(24, 24, 28, 225))
        p.setPen(QtGui.QPen(col, max(1.0, dpr)))
        p.drawRoundedRect(QtCore.QRectF(0.5, 0.5, w - 1, h - 1), 4 * dpr, 4 * dpr)
        p.setPen(col)
        p.drawText(QtCore.QRect(0, 0, w, h), QtCore.Qt.AlignCenter, self.text)
        p.end()
        rgba = qi.convertToFormat(QtGui.QImage.Format_RGBA8888).mirrored(False, True)   # SoImage: alt satir once
        data = bytes(rgba.constBits())[:w * h * 4]
        self.lab_img.image.setValue(coin.SbVec2s(w, h), 4, data)
        self.lab = (w, h)

    def _label_point(self):
        """Yazinin dunya konumu: okun ortasinin yaninda, ekranda okun dikinde. Ok ekranda yatay da dursa, kameraya
        dogru da baksa yazi okun ustune binmez: uzaklik yazinin o yondeki yari boyu kadar artirilir."""
        p = self.origin + self.dir * (self.value + self.size * 0.4)
        try:
            rot = self.view.getCameraOrientation()
            right = V(*rot.multVec(V(1, 0, 0)))
            up = V(*rot.multVec(V(0, 1, 0)))
        except Exception:
            right, up = V(1, 0, 0), V(0, 1, 0)
        ax, ay = self.dir.dot(right), self.dir.dot(up)     # okun ekrandaki yonu
        m = math.hypot(ax, ay)
        if m < 0.2:                                        # ok kameraya dogru: yazi saga
            px, py = 1.0, 0.0
        else:
            px, py = -ay / m, ax / m                       # dik yon, saga bakan
            if px < -1e-6 or (abs(px) <= 1e-6 and py < 0):
                px, py = -px, -py
        hw, hh = (self.lab[0] / 2.0, self.lab[1] / 2.0) if self.lab else (40.0, 10.0)
        k = self.size / float(GIZMO_PX)                    # piksel -> dunya
        dist = self.size * 0.18 + (hw * abs(px) + hh * abs(py)) * k
        return p + (right * px + up * py) * dist

    def update_scale(self, *args, **kw):
        if self.origin is None:
            return
        vv, W, H = sm._vv(self.view)
        p = self.origin + self.dir * self.value
        s = vv.getWorldToScreenScale(coin.SbVec3f(p.x, p.y, p.z), GIZMO_PX / float(H))
        if kw.get("force") or abs(s - self.size) > 1e-9 * max(s, 1.0):
            self.size = s
            self.scl.scaleFactor.setValue(s, s, s)
        q = self._label_point()                            # kamera donunce de yazi okun yaninda kalsin
        self.lab_tr.translation = (q.x, q.y, q.z)

    def _label_hit(self, pos):
        if self.lab is None or not self._visible() or self.origin is None:
            return False
        c = sm.screen_point(self.view, self._label_point())
        w, h = self.lab
        return abs(pos[0] - c[0]) <= w / 2.0 + 2 and abs(pos[1] - c[1]) <= h / 2.0 + 2

    def open_edit(self):
        """Yazinin yerinde sayi alani: Enter uygular, Esc vazgecer, odak kaybolunca kapanir."""
        if self.on_value is None or self.origin is None:
            return
        self.close_edit()
        try:
            gv = sm_graphics_view(self.view)
            vp = gv.viewport()
        except Exception:
            return
        dpr = vp.devicePixelRatioF()
        c = sm.screen_point(self.view, self._label_point())
        H = self.view.getSize()[1]
        e = _ValueEdit(vp, self._edit_done)
        e.setValue(self.value_for_edit)
        fm = e.fontMetrics()
        e.resize(max(fm.horizontalAdvance(e.text() + "000") + 24, 120), max(e.sizeHint().height(), fm.height() + 10))
        e.move(int(c[0] / dpr - e.width() / 2.0), int((H - c[1]) / dpr - e.height() / 2.0))
        e.show()
        e.setFocus()
        e.selectAll()
        self.edit = e

    def _edit_done(self, value):
        e, self.edit = self.edit, None
        if e is not None:
            e.hide()
            e.deleteLater()
        if value is not None and self.on_value is not None:
            self.on_value(value)

    def close_edit(self):
        if self.edit is not None:
            self._edit_done(None)

    def show(self, on):
        self.root.whichChild = 0 if on and self.origin is not None else -1

    def _visible(self):
        return self.root.whichChild.getValue() == 0        # alan nesnesi sayiyla karsilastirilmaz (hep 'farkli' cikiyordu)

    def _hit(self, pos):
        """Butun ok yakalanir: topuzun altindan koninin ucuna (0.72) kadar, 16 px genislikte. Once yalniz orta kisim
        tutuluyordu."""
        if not self._visible() or self.origin is None:
            return False
        p = self.origin + self.dir * self.value
        a = sm.screen_point(self.view, p - self.dir * (self.size * 0.12))
        b = sm.screen_point(self.view, p + self.dir * (self.size * 0.80))
        return sm._seg_dist(pos[0], pos[1], a, b) < 16.0

    def _param(self, pos):
        o, d = sm._ray(self.view, pos)
        return sm._line_param(o, d, self.origin, self.dir)

    def _btn(self, cb):
        ev = cb.getEvent()
        if ev.getButton() != coin.SoMouseButtonEvent.BUTTON1:
            return
        pos = ev.getPosition().getValue()
        if ev.getState() == coin.SoButtonEvent.DOWN:
            if self.edit is not None and not self._label_hit(pos):
                self.close_edit()                          # baska yere tiklandi: alan vazgecilerek kapanir
            if self._label_hit(pos):
                cb.setHandled()
                QtCore.QTimer.singleShot(0, self.open_edit)   # Coin olayindan cikinca: alan odagi alabilsin
                return
            if self._hit(pos):
                if self._double(pos):
                    cb.setHandled()
                    self.drag = None
                    QtCore.QTimer.singleShot(0, self.on_flip)
                    return
                t = self._param(pos)
                if t is not None:
                    cb.setHandled()
                    self.drag = (t, self.value)
                    self._color()
        elif self.drag is not None:
            cb.setHandled()
            self.drag = None
            self._color()
            self.on_finish()

    def _double(self, pos):
        """Okta ikinci tiklama, sistemin cift tiklama suresi icinde ve ayni yerde mi."""
        if self.on_flip is None:
            return False
        import time
        now = time.monotonic()
        last, self.last_click = self.last_click, (now, pos)
        if last is None:
            return False
        dt = (now - last[0]) * 1000.0
        if dt <= QtWidgets.QApplication.doubleClickInterval() and \
                abs(pos[0] - last[1][0]) <= 6 and abs(pos[1] - last[1][1]) <= 6:
            self.last_click = None
            return True
        return False

    def _mov(self, cb):
        pos = cb.getEvent().getPosition().getValue()
        if self.drag is None:
            h = self._hit(pos)
            if h != self.hot:
                self.hot = h
                self._color()
            lh = self._label_hit(pos)
            if lh != self.lab_hot:
                self.lab_hot = lh
                self._label_image()
            return
        if not sm._left_held():                            # tus pencere disinda birakildi
            self.drag = None
            self._color()
            self.on_finish()
            return
        cb.setHandled()
        t = self._param(pos)
        if t is None:
            return
        t0, v0 = self.drag
        self.on_move(v0 + (t - t0))

    def remove(self):
        if not self.ok:
            return
        self.ok = False
        self.close_edit()
        try:
            self.view.removeEventCallbackPivy(coin.SoMouseButtonEvent.getClassTypeId(), self.cb_btn)
            self.view.removeEventCallbackPivy(coin.SoLocation2Event.getClassTypeId(), self.cb_mov)
        except Exception:
            pass
        try:
            self.parent.removeChild(self.root)
        except Exception:
            pass
        self.root.unref()


class Editor(object):
    def __init__(self, obj):
        self.obj, self.doc = obj, obj.Document
        self.view = Gui.ActiveDocument.ActiveView
        self.gp = obj.getGlobalPlacement() if hasattr(obj, "getGlobalPlacement") else App.Placement()
        self.fresh = obj.Name if fresh is obj else None
        self.dragging = False
        self.axis = None                                   # (merkez, yon) yerel
        self.error = None
        self.prof = None
        self.doc.openTransaction(T("Edit Magic Pad"))
        mp.miss_prop(obj)                                  # eski surumle yapilmis nesnelere yeni ozellikler (Offset)
        self.parent = sm._scene_parent(self.view)
        self._nodes()
        self.gizmo = Gizmo(self.view, self.parent, self.on_gizmo, self.end_drag, self.set_length, self.flip)
        self._preview_on()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._scale_gizmo)
        self.timer.start(50)
        self.panel = Panel(self)
        self.refresh()
        Gui.Control.closeDialog()
        Gui.Control.showDialog(self.panel)

    # --- cizim dugumleri ---
    def _nodes(self):
        self.prev = coin.SoSeparator()
        self.prev.ref()
        for f in ("renderCaching", "boundingBoxCaching", "renderCulling", "pickCulling"):
            setattr(self.prev, f, coin.SoSeparator.OFF)
        hints = coin.SoShapeHints()
        hints.vertexOrdering = coin.SoShapeHints.UNKNOWN_ORDERING
        hints.creaseAngle = 0.6                            # egri yuzler yumusak golgelensin (fasetli seritler yok)
        self.mat = coin.SoMaterial()
        self.mat.transparency = 0.45
        self.f_coord, self.f_set = coin.SoCoordinate3(), coin.SoIndexedFaceSet()
        lm = coin.SoLightModel()
        lm.model = coin.SoLightModel.BASE_COLOR
        self.e_col = coin.SoBaseColor()
        ds = coin.SoDrawStyle()
        ds.lineWidth = 1.5
        self.e_coord, self.e_set = coin.SoCoordinate3(), coin.SoLineSet()
        pick = coin.SoPickStyle()                          # onizleme secilmez: gizmo ve govde tiklanabilsin
        pick.style = coin.SoPickStyle.UNPICKABLE
        nodes = [hints, self.mat]
        self.db = self.e_db = None
        if hasattr(coin, "SoDepthBuffer"):                 # kesme: hacim katinin icinde, derinlik testi kapali
            self.db = coin.SoDepthBuffer()
            self.db.write = False
            nodes.append(self.db)
            self.e_db = coin.SoDepthBuffer()
        faces = sm._sep(*(nodes + [self.f_coord, self.f_set]))
        e_nodes = [lm, self.e_col, ds] + ([self.e_db] if self.e_db is not None else []) + [self.e_coord, self.e_set]
        edges = sm._sep(*e_nodes)
        # kesisim cizgileri: aracin govdeyi kestigi yerler, beyaz kalin, ustte (birakinca hesaplanir)
        slm = coin.SoLightModel()
        slm.model = coin.SoLightModel.BASE_COLOR
        scol = coin.SoBaseColor()
        scol.rgb = (1.0, 1.0, 1.0)
        sds = coin.SoDrawStyle()
        sds.lineWidth = 3.0
        self.s_coord, self.s_set = coin.SoCoordinate3(), coin.SoLineSet()
        s_nodes = [slm, scol, sds]
        if hasattr(coin, "SoDepthBuffer"):
            sdb = coin.SoDepthBuffer()
            sdb.test = False
            s_nodes.append(sdb)
        sect = sm._sep(*(s_nodes + [self.s_coord, self.s_set]))
        self.prev.addChild(pick)
        self.prev.addChild(faces)
        self.prev.addChild(edges)
        self.prev.addChild(sect)
        self.parent.addChild(self.prev)

    def world(self, p):
        return self.gp.multVec(V(*tuple(p)))

    def world_dir(self, d):
        return self.gp.Rotation.multVec(V(*tuple(d)))

    # --- onizleme ---
    def refresh(self, trim=True):
        """Araci (boolean'siz) yeniden kurar ve cizer; gizmo yerine oturur."""
        obj = self.obj
        if self.prof is None:                              # profil duzenleme boyunca degismez: bir kez okunur
            self.prof = mp.profile(obj)
        prof = self.prof
        if isinstance(prof, str):
            self._show(None, T(prof))
            return
        face, c, n, origin, on_body = prof[:5]
        n = mp.direction(obj, prof, n)
        if obj.Reversed:
            n = -n
        self.axis = (c, n)
        res = mp.compute(obj, trim=trim, prof=prof)
        if isinstance(res, str):
            self._show(None, res)
            return
        self._show(res[0], None, res[2])
        self._section(res[0], res[1] if trim else None)    # surukerken yok: birakinca hesaplanir

    def _tip(self):
        """Okun eksendeki yeri (mm, profil duzleminden)."""
        o = self.obj
        L = float(o.Length)
        if o.Mode == "Symmetric":
            return abs(L) / 2.0
        if o.Mode == "UpToBody":
            return abs(L)
        if o.Mode == "ThroughAll":
            return L if abs(L) > 1e-9 else -1.0
        return L

    def _show(self, tool, err, op=None):
        self.error = err
        if op is None:
            op = mp.operation(self.obj, self.prof)
        rgb = CUT_RGB if op == "Subtraction" else ADD_RGB
        self.mat.diffuseColor = rgb
        self.e_col.rgb = tuple(0.6 * x for x in rgb)
        for db in (self.db, self.e_db):
            if db is not None:
                db.test = op != "Subtraction"
        if tool is None:
            self.f_set.coordIndex.setNum(0)
            self.e_set.numVertices.setNum(0)
            self.s_set.numVertices.setNum(0)
        else:
            self._mesh(tool)
        if self.axis is not None:
            c, n = self.axis
            L = float(self.obj.Length)
            try:
                text = App.Units.Quantity(L, App.Units.Length).UserString
            except Exception:
                text = "%.2f mm" % L
            self.gizmo.value_for_edit = L
            self.gizmo.place(self.world(c), self.world_dir(n), self._tip(), rgb, text)
            self.gizmo.show(not has_expression(self.obj, "Length") and self.obj.Mode not in ("ThroughAll",) + mp.AUTO)
        self.panel.status(err)

    def _mesh(self, tool):
        size = max(tool.BoundBox.DiagonalLength, 1.0)
        try:
            pts, tris = tool.tessellate(max(0.002 * size, 0.01))
        except Exception:
            pts, tris = [], []
        P = [tuple(self.world(p)) for p in pts]
        self.f_coord.point.setValues(0, len(P), P)
        self.f_coord.point.setNum(len(P))
        idx = [i for t in tris for i in (t[0], t[1], t[2], -1)]
        self.f_set.coordIndex.setValues(0, len(idx), idx)
        self.f_set.coordIndex.setNum(len(idx))
        self._lines(self.e_coord, self.e_set, [e for e in tool.Edges if not self._seam(tool, e)], size)

    @staticmethod
    def _seam(shape, e):
        """Dikis kenari (silindir gibi kapali yuzun iki ucu): tek bir yuze iki kez ait; cizilmez."""
        try:
            fs = shape.ancestorsOfType(e, Part.Face)
            return len(fs) >= 2 and all(f.isSame(fs[0]) for f in fs)
        except Exception:
            return False

    def _lines(self, coord, lset, edges, size):
        E, counts = [], []
        for e in edges:
            try:
                q = e.discretize(Deflection=max(0.002 * size, 0.01))
            except Exception:
                continue
            if len(q) < 2:
                continue
            E += [tuple(self.world(p)) for p in q]
            counts.append(len(q))
        coord.point.setValues(0, len(E), E)
        coord.point.setNum(len(E))
        lset.numVertices.setValues(0, len(counts), counts)
        lset.numVertices.setNum(len(counts))

    def _section(self, tool, base):
        """Aracin govdeyi kestigi egriler (tool.section(base)); govde yoksa ya da hesap tutmazsa bos."""
        edges = []
        if tool is not None and base is not None and not base.isNull():
            try:
                edges = tool.section(base).Edges
            except Exception:
                edges = []
        size = max(tool.BoundBox.DiagonalLength, 1.0) if tool is not None else 1.0
        self._lines(self.s_coord, self.s_set, edges, size)

    def picking(self, on):
        """Hedef / eksen secerken onizleme ve ok gizlenir: arkalarindaki yuz ve kenarlar gorunur ve rahat secilir."""
        try:
            idx = self.parent.findChild(self.prev)
            if on and idx >= 0:
                self.parent.removeChild(self.prev)         # prev ref'li: silinmez
                self.gizmo.show(False)
            elif not on and idx < 0:
                self.parent.addChild(self.prev)
                if not getattr(self, "closed", False):
                    self.refresh()
        except Exception:
            pass

    def _scale_gizmo(self):
        self.gizmo.update_scale()

    def _preview_on(self):
        vo = self.obj.ViewObject
        self.saved = dict(vis=vo.Visibility)
        base = getattr(self.obj, "BaseFeature", None)
        self.saved["base"] = base.ViewObject.Visibility if base is not None else None
        if base is not None:
            base.ViewObject.Visibility = True
        vo.Visibility = False

    def _preview_off(self):
        sv = getattr(self, "saved", None)
        if sv is None:
            return
        base = getattr(self.obj, "BaseFeature", None)
        if base is not None and sv["base"] is not None:
            base.ViewObject.Visibility = sv["base"]
        self.obj.ViewObject.Visibility = sv["vis"]

    # --- gizmo ---
    def on_gizmo(self, y):
        """Gizmo surukleniyor: y okun eksendeki yeni yeri."""
        o = self.obj
        if has_expression(o, "Length"):
            return
        self.dragging = True
        if o.Mode == "Symmetric":
            L = max(2.0 * abs(y), 0.01)
        elif o.Mode == "UpToBody":
            L = max(abs(y), 0.01)
        else:
            L = y
        o.Length = round(L, 2)
        self.panel.sync()
        self.refresh(trim=False)                           # surukerken govdeye kadar kirpilmaz: duz prizma

    def set_length(self, v):
        """Okun yanindaki alana yazilan uzunluk (ifade bagliysa dokunulmaz)."""
        o = self.obj
        if has_expression(o, "Length"):
            return
        if o.Mode in ("Symmetric", "UpToBody"):
            v = max(abs(v), 0.01)
        o.Length = float(v)
        self.panel.sync()
        self.refresh(trim=True)

    def flip(self):
        """Oka cift tiklama: yon tersine doner (Otomatik islemde ekle <-> kes de degisir)."""
        o = self.obj
        o.Reversed = not bool(o.Reversed)
        self.dragging = False
        self.panel.sync()
        self.refresh(trim=True)

    def end_drag(self):
        if not self.dragging:
            return
        self.dragging = False
        self.refresh(trim=True)
        self.panel.sync()

    # --- bitis ---
    def finish(self, ok):
        self.finished = True
        self.close()
        global _editor
        _editor = None
        if ok:
            self.doc.commitTransaction()
        else:
            self.doc.abortTransaction()
        Gui.ActiveDocument.resetEdit()
        while not ok and self.fresh and self.doc.getObject(self.fresh) is not None and self.doc.UndoCount:
            self.doc.undo()                                # yeni nesne: olusturma adimi da gider
        self.doc.recompute()

    def close(self):
        if getattr(self, "closed", False):
            return
        self.closed = True
        if not getattr(self, "finished", False):
            self.doc.commitTransaction()
            self.obj.touch()
            self.doc.recompute()
        self.timer.stop()
        self.panel.pick_off()
        self.gizmo.remove()
        try:
            self.parent.removeChild(self.prev)
        except Exception:
            pass
        self.prev.unref()
        self._preview_off()
        Gui.Control.closeDialog()


class _TargetPicker(object):
    """Hedef secerken bir sonraki 3B secimi panele iletir."""

    def __init__(self, panel):
        self.panel = panel

    def addSelection(self, doc, obj, sub, pnt):
        QtCore.QTimer.singleShot(0, self._take)

    def _take(self):
        try:
            sel = Gui.Selection.getSelectionEx("", 1)
        except Exception:
            sel = Gui.Selection.getSelectionEx()
        if not sel:
            return
        s = sel[-1]
        subs = [x for x in s.SubElementNames if x]
        if self.panel.observer is self:
            sub = subs[-1].split(".")[-1] if subs else ""
            if self.panel.pick_kind == "dir":
                self.panel.set_dir(s.Object, sub)
            elif self.panel.pick_kind == "base":
                self.panel.set_base(s.Object)
            else:
                self.panel.set_target(s.Object, sub)


class Panel(object):
    def __init__(self, ed):
        self.ed = ed
        obj = ed.obj
        self.syncing = False
        self.form = w = QtWidgets.QWidget()
        w.setWindowTitle(T("Magic Pad"))
        lay = QtWidgets.QVBoxLayout(w)
        self.err = QtWidgets.QLabel()
        self.err.setWordWrap(True)
        self.err.setStyleSheet("color: #d9534f;")
        lay.addWidget(self.err)
        self.op = QtWidgets.QLabel()
        lay.addWidget(self.op)
        hint = QtWidgets.QLabel(T("Drag the arrow: out of the solid adds, into it cuts."))
        hint.setWordWrap(True)
        lay.addWidget(hint)
        tip = QtWidgets.QLabel("\u21c5  " + T("Double-click the arrow to flip the direction."))
        tip.setWordWrap(True)
        tip.setStyleSheet("color: #e0a040;")               # okun yazisiyla ayni turuncu ton
        lay.addWidget(tip)
        g = QtWidgets.QGroupBox(T("Extent"))
        f = QtWidgets.QFormLayout(g)
        self.mode = QtWidgets.QComboBox()
        for m in mp.MODES:
            self.mode.addItem(T(MODE_TEXT[m]), m)
        f.addRow(T("Type"), self.mode)
        self.length = self._qspin("Length", "mm", -1e6, 1e6)
        f.addRow(T("Length"), self.length)
        self.length2 = self._qspin("Length2", "mm", 0.0, 1e6)
        self.l2_label = QtWidgets.QLabel(T("Second length"))
        f.addRow(self.l2_label, self.length2)
        self.miss = QtWidgets.QComboBox()
        self.miss.addItem(T("Cut off"), "Cut")
        self.miss.addItem(T("Go to Length"), "Length")
        self.miss_label = QtWidgets.QLabel(T("Missed part"))
        self.miss.setToolTip(T("Up to body: what happens to the part of the profile that does not meet the Body"))
        f.addRow(self.miss_label, self.miss)
        self.offset = self._qspin("Offset", "mm", -1e6, 1e6)
        self.offset.setToolTip(T("Shifts where the feature stops: positive goes past the target, negative stops "
                                 "before it"))
        self.off_label = QtWidgets.QLabel(T("Offset to the target"))
        f.addRow(self.off_label, self.offset)
        self.pick = QtWidgets.QPushButton(T("Select..."))
        self.pick.setCheckable(True)
        self.pick.setToolTip(T("Then click the face (Up to face) or the solid (Up to shape) in the 3D view"))
        self.target = QtWidgets.QLabel()
        self.target.setWordWrap(True)
        tw = QtWidgets.QWidget()
        tl = QtWidgets.QHBoxLayout(tw)
        tl.setContentsMargins(0, 0, 0, 0)
        tl.addWidget(self.pick)
        tl.addWidget(self.target, 1)
        self.t_label = QtWidgets.QLabel(T("Target"))
        f.addRow(self.t_label, tw)
        self.pick.toggled.connect(self._pick_toggled)
        self.observer = None
        self.pick_kind = None
        self.bpick = QtWidgets.QPushButton(T("Select..."))
        self.bpick.setCheckable(True)
        self.bpick.setToolTip(T("Solid the profile belongs to (not changed): gives the outward side and where the Up "
                                "to modes stop"))
        self.blabel = QtWidgets.QLabel()
        self.blabel.setWordWrap(True)
        bw = QtWidgets.QWidget()
        bl = QtWidgets.QHBoxLayout(bw)
        bl.setContentsMargins(0, 0, 0, 0)
        bl.addWidget(self.bpick)
        bl.addWidget(self.blabel, 1)
        self.b_label = QtWidgets.QLabel(T("Reference"))
        f.addRow(self.b_label, bw)
        self.bpick.toggled.connect(self._bpick_toggled)
        self.rev = QtWidgets.QCheckBox(T("Reversed"))
        f.addRow(self.rev)
        self.opmode = QtWidgets.QComboBox()
        self.opmode.addItem(T("Auto (by direction)"), "Auto")
        self.opmode.addItem(T("Add"), "Add")
        self.opmode.addItem(T("Cut"), "Cut")
        self.opmode.setToolTip(T("Auto: into the solid cuts, out of it adds. Choose Add or Cut when the direction "
                                 "does not tell it right."))
        self.opmode_label = QtWidgets.QLabel(T("Operation"))
        f.addRow(self.opmode_label, self.opmode)
        self.dirmode = QtWidgets.QComboBox()
        self.dirmode.addItem(T("Face normal"), "Normal")
        self.dirmode.addItem(T("Average of the faces"), "Average")
        self.dirmode.addItem(T("Custom (edge, face or axis)"), "Custom")
        f.addRow(T("Direction"), self.dirmode)
        self.dpick = QtWidgets.QPushButton(T("Select..."))
        self.dpick.setCheckable(True)
        self.dpick.setToolTip(T("Then click a straight edge, a flat face or a datum axis in the 3D view"))
        self.dref = QtWidgets.QLabel()
        self.dref.setWordWrap(True)
        dw = QtWidgets.QWidget()
        dl = QtWidgets.QHBoxLayout(dw)
        dl.setContentsMargins(0, 0, 0, 0)
        dl.addWidget(self.dpick)
        dl.addWidget(self.dref, 1)
        self.d_label = QtWidgets.QLabel(T("Axis"))
        f.addRow(self.d_label, dw)
        self.dpick.toggled.connect(self._dpick_toggled)
        self.sides = QtWidgets.QComboBox()
        self.sides.addItem(T("Follow the neighbouring faces"), "Neighbours")
        self.sides.addItem(T("Perpendicular (like Pad)"), "Perpendicular")
        self.sides_label = QtWidgets.QLabel(T("Side walls"))
        f.addRow(self.sides_label, self.sides)
        lay.addWidget(g)
        g = QtWidgets.QGroupBox(T("Shape"))
        f = QtWidgets.QFormLayout(g)
        self.taper = self._qspin("TaperAngle", "°", -89.0, 89.0)
        f.addRow(T("Taper angle"), self.taper)
        self.twist = self._qspin("Twist", "°", -3600.0, 3600.0)
        f.addRow(T("Twist"), self.twist)
        self.center = QtWidgets.QComboBox()
        self.center.addItem(T("Profile center"), "Profile center")
        self.center.addItem(T("Sketch origin"), "Sketch origin")
        f.addRow(T("Twist axis"), self.center)
        self.ease = QtWidgets.QCheckBox(T("Ease in / out"))
        f.addRow(self.ease)
        self.refine = QtWidgets.QCheckBox(T("Refine"))
        f.addRow(self.refine)
        lay.addWidget(g)
        note = QtWidgets.QLabel(T("Twist and taper rebuild the side faces: turning them on or off, or changing the "
                                  "profile of a twisted feature, can break fillets and sketches attached to it."))
        note.setWordWrap(True)
        note.setStyleSheet("color: gray;")
        lay.addWidget(note)
        lay.addStretch(1)
        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)
        self.timer.setInterval(120)
        self.timer.timeout.connect(self.apply)
        self.sync()
        for s in (self.length, self.length2, self.offset, self.taper, self.twist):
            self._connect_spin(s)
        for c in (self.mode, self.miss, self.center, self.opmode, self.dirmode, self.sides):
            c.currentIndexChanged.connect(lambda i: self._later())
        for c in (self.rev, self.ease, self.refine):
            c.toggled.connect(lambda on: self._later())

    def _qspin(self, prop, unit, lo, hi):
        """FreeCAD'in QuantitySpinBox'i, ozellige ifadeyle bagli (fx dugmesi); olmazsa duz QDoubleSpinBox."""
        try:
            w = Gui.UiLoader().createWidget("Gui::QuantitySpinBox")
            w.setProperty("unit", unit)
            w.setProperty("minimum", lo)
            w.setProperty("maximum", hi)
            w.setProperty("singleStep", 1.0)
            Gui.ExpressionBinding(w).bind(self.ed.obj, prop)
            w._qty = True
            return w
        except Exception:
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(2)
            s.setSuffix(" " + unit)
            s.setKeyboardTracking(False)
            s._qty = False
            return s

    def _connect_spin(self, w):
        try:
            w.valueChanged.connect(lambda *a: self._later())
        except Exception:
            try:
                w.editingFinished.connect(self._later)
            except Exception:
                pass

    @staticmethod
    def _get(w):
        if getattr(w, "_qty", False):
            v = w.property("rawValue")
            return float(v) if v is not None else 0.0
        return float(w.value())

    @staticmethod
    def _set(w, v):
        if getattr(w, "_qty", False):
            w.setProperty("rawValue", float(v))
        else:
            w.setValue(float(v))

    def _later(self):
        if not self.syncing:
            self.timer.start()

    def sync(self):
        """Nesneden panele."""
        o = self.ed.obj
        self.syncing = True
        try:
            self.mode.setCurrentIndex(max(self.mode.findData(o.Mode), 0))
            for w, prop in ((self.length, "Length"), (self.length2, "Length2"), (self.offset, "Offset"),
                            (self.taper, "TaperAngle"), (self.twist, "Twist")):
                if not w.hasFocus():
                    self._set(w, float(getattr(o, prop, 0.0)))
            self.miss.setCurrentIndex(max(self.miss.findData(getattr(o, "Miss", "Cut")), 0))
            self.opmode.setCurrentIndex(max(self.opmode.findData(getattr(o, "OpMode", "Auto")), 0))
            self.dirmode.setCurrentIndex(max(self.dirmode.findData(getattr(o, "ExtrudeDir", "Normal")), 0))
            self.sides.setCurrentIndex(max(self.sides.findData(getattr(o, "SideFaces", "Neighbours")), 0))
            self.rev.setChecked(bool(o.Reversed))
            self.center.setCurrentIndex(max(self.center.findData(o.TwistCenter), 0))
            self.ease.setChecked(bool(o.TwistEase))
            self.refine.setChecked(bool(getattr(o, "Refine", False)))
        finally:
            self.syncing = False
        mode = o.Mode
        self.length2.setVisible(mode == "TwoLengths")
        self.l2_label.setVisible(mode == "TwoLengths")
        self.miss.setVisible(mode in ("UpToBody", "UpToShape"))
        self.miss_label.setVisible(mode in ("UpToBody", "UpToShape"))
        self.length.setEnabled(mode not in ("UpToFirst", "UpToLast", "UpToFace"))
        upto = mode in ("UpToBody",) + mp.AUTO
        self.offset.setVisible(upto)
        self.off_label.setVisible(upto)
        tgt = mode in mp.TARGET
        self.pick.setVisible(tgt)
        self.target.setVisible(tgt)
        self.t_label.setVisible(tgt)
        if not tgt and self.pick_kind == "target":
            self.pick_off()
        self.target.setText(self._target_text())
        part = mp.is_part(o)
        for wdg in (self.bpick, self.blabel, self.b_label):
            wdg.setVisible(part)
        self.opmode.setVisible(not part)                   # Part: boolean yok, hep cekilen kati
        self.opmode_label.setVisible(not part)
        k = self.mode.findData("ThroughAll")
        if k >= 0:
            self.mode.model().item(k).setEnabled(not part)
        b = getattr(o, "Reference", None)
        if self.pick_kind != "base":
            self.blabel.setText(b.Label if b is not None else T("(none)"))
        faces = self._face_profile()
        custom = getattr(o, "ExtrudeDir", "Normal") == "Custom"
        self.dpick.setVisible(custom)
        self.dref.setVisible(custom)
        self.d_label.setVisible(custom)
        if not custom and self.pick_kind == "dir":
            self.pick_off()
        if custom and self.pick_kind is None and mp.custom_dir(o) is None and not getattr(self, "_auto_picked", False):
            self._auto_picked = True                       # Ozel secilince eksen secimi kendiliginden baslar
            QtCore.QTimer.singleShot(0, lambda: self.dpick.setChecked(True))
        self.dref.setText(self._dir_text())
        self.dirmode.model().item(1).setEnabled(faces)      # ortalama: yalniz secili yuzlerde anlamli
        self.sides.setVisible(faces)
        self.sides_label.setVisible(faces)
        self.status(self.ed.error)

    def apply(self):
        o = self.ed.obj
        vals = dict(Mode=self.mode.currentData(), Miss=self.miss.currentData(), Reversed=self.rev.isChecked(),
                    OpMode=self.opmode.currentData(), ExtrudeDir=self.dirmode.currentData(),
                    SideFaces=self.sides.currentData(),
                    TwistCenter=self.center.currentData(), TwistEase=self.ease.isChecked(),
                    Refine=self.refine.isChecked())
        for w, prop in ((self.length, "Length"), (self.length2, "Length2"), (self.offset, "Offset"),
                        (self.taper, "TaperAngle"), (self.twist, "Twist")):
            if not has_expression(o, prop):                # ifade bagliysa deger ifadeden gelir
                vals[prop] = self._get(w)
        for k, v in vals.items():
            if k not in o.PropertiesList:
                continue
            cur = getattr(o, k)
            cur = float(cur) if hasattr(cur, "Value") else cur
            if cur != v:
                setattr(o, k, v)
        self.sync()
        self.ed.refresh()

    def status(self, err):
        self.err.setText(err or "")
        self.err.setVisible(bool(err))
        self.op.setText("<b>%s</b>" % (T("Cut") if mp.operation(self.ed.obj, self.ed.prof) == "Subtraction"
                                       else T("Add")))

    # --- hedef secimi (Yuze / sekle kadar) ---
    def _target_text(self):
        link = getattr(self.ed.obj, "UpToTarget", None)
        if not link or link[0] is None:
            return T("(none)")
        subs = [x for x in (link[1] or []) if x]
        return link[0].Label + (" : " + subs[0] if subs else "")

    def _face_profile(self):
        link = getattr(self.ed.obj, "Profile", None)
        return bool(link and link[0] is not None and any(x.startswith("Face") for x in (link[1] or []) if x))

    def _dir_text(self):
        link = getattr(self.ed.obj, "DirRef", None)
        if not link or link[0] is None:
            return T("(none)")
        subs = [x for x in (link[1] or []) if x]
        return link[0].Label + (" : " + subs[0] if subs else "")

    def _start_pick(self, kind, label):
        self.pick_off()
        self.pick_kind = kind
        btn = {"target": self.pick, "dir": self.dpick, "base": self.bpick}[kind]
        btn.blockSignals(True)
        btn.setChecked(True)
        btn.blockSignals(False)
        self.observer = _TargetPicker(self)
        Gui.Selection.addObserver(self.observer)
        Gui.Selection.clearSelection()
        self.ed.picking(True)
        label.setText(T("Click the target in the 3D view") if kind == "target" else T("Click the axis in the 3D view"))

    def _pick_toggled(self, on):
        if on:
            self._start_pick("target", self.target)
        else:
            self.pick_off()
            self.target.setText(self._target_text())

    def _bpick_toggled(self, on):
        if on:
            self._start_pick("base", self.blabel)
            self.blabel.setText(T("Pick the solid in the 3D view"))
        else:
            self.pick_off()
            self.sync()

    def set_base(self, o):
        """Part surumu: referans kati (PartDesign ozelligi secildiyse butun Body); degistirilmez, gizlenmez."""
        g = o.getParentGeoFeatureGroup()
        if g is not None and g.isDerivedFrom("PartDesign::Body"):
            o = g
        obj = self.ed.obj
        if o == obj or not hasattr(o, "Shape") or not o.Shape.Solids:
            self.blabel.setText(T("Select a solid"))
            return
        try:
            obj.Reference = o
        except Exception as e:
            self.blabel.setText(str(e))
            return
        self.pick_off()
        Gui.Selection.clearSelection()
        self.sync()
        self.ed.refresh()

    def _dpick_toggled(self, on):
        if on:
            sel = Gui.Selection.getSelectionEx("", 1)       # once secilmis kenar / yuz / eksen varsa hemen alinir
            prof = getattr(self.ed.obj, "Profile", None)
            if sel and prof and sel[-1].Object == prof[0] and \
                    set(x.split(".")[-1] for x in sel[-1].SubElementNames) <= set(prof[1] or []):
                sel = []                                   # profilin kendi yuzleri yon sayilmaz
            if sel:
                s = sel[-1]
                subs = [x for x in s.SubElementNames if x]
                self.dpick.blockSignals(True)
                self.dpick.setChecked(False)
                self.dpick.blockSignals(False)
                self.set_dir(s.Object, subs[-1].split(".")[-1] if subs else "")
                if mp.custom_dir(self.ed.obj) is not None:
                    return
            self._start_pick("dir", self.dref)
        else:
            self.pick_off()
            self.dref.setText(self._dir_text())

    def pick_off(self):
        if self.observer is not None:
            try:
                Gui.Selection.removeObserver(self.observer)
            except Exception:
                pass
            self.observer = None
            self.ed.picking(False)
        self.pick_kind = None
        for b in (getattr(self, "pick", None), getattr(self, "dpick", None), getattr(self, "bpick", None)):
            if b is not None and b.isChecked():
                b.blockSignals(True)
                b.setChecked(False)
                b.blockSignals(False)

    def set_dir(self, o, sub):
        """Ozel yon: duz kenar, duz yuz ya da datum cizgi / duzlem."""
        obj = self.ed.obj
        link = (o, [sub] if sub else [])
        try:
            obj.DirRef = link
        except Exception as e:
            self.dref.setText(str(e))
            return
        if mp.custom_dir(obj) is None:
            obj.DirRef = None
            self.dref.setText(T("Select a straight edge, a flat face or a datum axis"))
            return
        self.pick_off()
        Gui.Selection.clearSelection()
        self.sync()
        self.ed.refresh()

    def set_target(self, o, sub):
        """Secilen hedefi baglar. Yuze kadar: yalniz yuz; sekle kadar: nesne (PartDesign ozelliginin yuzu secildiyse
        o ozelligin govdesi)."""
        obj = self.ed.obj
        mode = obj.Mode
        if mode == "UpToFace":
            if not sub or not sub.startswith("Face"):
                self.target.setText(T("Select a face"))
                return
            link = (o, [sub])
        else:
            g = o.getParentGeoFeatureGroup()
            if g is not None and g.isDerivedFrom("PartDesign::Body"):
                o = g
            if o == obj.getParentGeoFeatureGroup():
                self.target.setText(T("Select another solid (for this Body use Up to body)"))
                return
            link = (o, [])
        try:
            obj.UpToTarget = link
        except Exception as e:
            self.target.setText(str(e))
            return
        self.pick_off()
        Gui.Selection.clearSelection()
        self.sync()
        self.ed.refresh()

    def getStandardButtons(self):
        B = QtWidgets.QDialogButtonBox
        return sm.btn_flags(B.Ok, B.Cancel)

    def accept(self):
        self.timer.stop()
        self.apply()
        self.ed.obj.touch()
        self.ed.doc.recompute()
        if "Invalid" in self.ed.obj.State:                 # hatali sonucla cikilmaz: metin panelde
            self.status(str(self.ed.obj.getStatusString()))
            return False
        self.ed.finish(True)
        return True

    def reject(self):
        self.timer.stop()
        self.ed.finish(False)
        return True
