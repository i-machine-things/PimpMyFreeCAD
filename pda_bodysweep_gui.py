# -*- coding: utf-8 -*-
# Pimp My FreeCAD - Body Sweep: gorev paneli. Copyright (c) 2026 Sedat Kuran - MIT License
#
# Alet, yol ve kesilecek govdeler secimden alinir; yon (yolu takip et / dunya Z / destek yuzeyi), burulma ve tolerans
# ayarlanir. Zarf "Hesapla" ile arka plan is parcaciginda kurulur (FreeCAD donmaz, ilerleme panelde, Durdur ile
# kesilir); bitince hedefler kesilir. OK gerekiyorsa once hesaplar. OK degisiklikleri tek geri alma adimi olarak
# birakir, Cancel geri alir; yeni nesneyi de siler.
import threading
import time

import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtCore, QtWidgets

import pda_bodysweep_feature as bf
from pda_i18n import T

_panel = None
fresh = None                                               # komutun yeni olusturdugu nesne: Cancel onu da geri alir


def start(obj):
    global _panel
    stop()
    _panel = Panel(obj)
    Gui.Control.showDialog(_panel)


def stop():
    global _panel
    if _panel is not None:
        p, _panel = _panel, None
        p.close()


def _label(o, subs=None):
    if o is None:
        return "-"
    subs = [s for s in ([subs] if isinstance(subs, str) else list(subs or [])) if s]
    return o.Label + (" : " + ", ".join(subs) if subs else "")


class Panel(object):
    def __init__(self, obj):
        self.obj, self.doc = obj, obj.Document
        self.fresh = obj.Name if fresh is obj else None
        self.finished = False
        self.job = None                                    # arka plan hesabi: dict(thread, cancel, msg, result, t0)
        self.after_ok = False
        self.doc.openTransaction(T("Edit Body Sweep"))
        self.form = w = QtWidgets.QWidget()
        w.setWindowTitle(T("Body Sweep"))
        lay = QtWidgets.QVBoxLayout(w)
        # girdiler
        g = QtWidgets.QGroupBox(T("Tool and path"))
        grid = QtWidgets.QGridLayout(g)
        self.tool_lbl, self.path_lbl, self.sup_lbl = QtWidgets.QLabel(), QtWidgets.QLabel(), QtWidgets.QLabel()
        for r, (text, lbl, fn, tip) in enumerate((
                (T("Tool"), self.tool_lbl, self.take_tool, T("The solid that moves along the path (any Body / Part)")),
                (T("Path"), self.path_lbl, self.take_path, T("Edges, a wire or a sketch; one chain")))):
            grid.addWidget(QtWidgets.QLabel(text), r, 0)
            lbl.setWordWrap(True)
            grid.addWidget(lbl, r, 1)
            b = QtWidgets.QPushButton(T("From selection"))
            b.setToolTip(tip)
            b.clicked.connect(lambda checked=False, fn=fn: fn())
            grid.addWidget(b, r, 2)
        lay.addWidget(g)
        # kesilecekler
        g = QtWidgets.QGroupBox(T("Bodies to cut"))
        grid = QtWidgets.QGridLayout(g)
        self.targets = QtWidgets.QListWidget()
        self.targets.setMaximumHeight(90)
        grid.addWidget(self.targets, 0, 0, 1, 2)
        b = QtWidgets.QPushButton(T("Add selection"))
        b.setToolTip(T("PartDesign Bodies get a Body Sweep Cut step, other solids a Part Cut"))
        b.clicked.connect(lambda checked=False: self.add_targets())
        grid.addWidget(b, 1, 0)
        b = QtWidgets.QPushButton(T("Remove"))
        b.clicked.connect(lambda checked=False: self.remove_target())
        grid.addWidget(b, 1, 1)
        lay.addWidget(g)
        # hareket
        g = QtWidgets.QGroupBox(T("Motion"))
        grid = QtWidgets.QGridLayout(g)
        grid.addWidget(QtWidgets.QLabel(T("Orientation")), 0, 0)
        self.orient = QtWidgets.QComboBox()
        for v, text in zip(bf.ORIENT, (T("Follow path"), T("World Z up"))):
            self.orient.addItem(text, v)
        self.orient.setToolTip(T("Follow path: the tool turns with the path without twisting (rotation-minimizing "
                                 "frame), also on vertical and 3D paths. World Z up: the tool's top stays toward Z"))
        self.orient.setCurrentIndex(bf.ORIENT.index(obj.Orientation))
        self.orient.currentIndexChanged.connect(lambda i: self.set_prop("Orientation", bf.ORIENT[i]))
        grid.addWidget(self.orient, 0, 1, 1, 2)
        grid.addWidget(QtWidgets.QLabel(T("Support face")), 1, 0)
        self.sup_lbl.setWordWrap(True)
        self.sup_lbl.setToolTip(T("Optional: the tool stays normal to this face (overrides Orientation)"))
        grid.addWidget(self.sup_lbl, 1, 1)
        row = QtWidgets.QHBoxLayout()
        for text, fn in ((T("From selection"), self.take_support), (T("Clear"), self.clear_support)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda checked=False, fn=fn: fn())
            row.addWidget(b)
        grid.addLayout(row, 1, 2)
        grid.addWidget(QtWidgets.QLabel(T("Twist")), 2, 0)
        self.twist = QtWidgets.QDoubleSpinBox()
        self.twist.setRange(-bf.TWIST_MAX, bf.TWIST_MAX)
        self.twist.setDecimals(1)
        self.twist.setSuffix(" °")
        self.twist.setKeyboardTracking(False)
        self.twist.setToolTip(T("Total turn of the tool about the path along its length; the sign sets the direction"))
        self.twist.setValue(float(obj.Twist))
        self.twist.valueChanged.connect(lambda v: self.set_prop("Twist", float(v)))
        grid.addWidget(self.twist, 2, 1, 1, 2)
        grid.addWidget(QtWidgets.QLabel(T("Method")), 4, 0)
        bf.method_prop(obj)
        self.method = QtWidgets.QComboBox()
        for v, text in zip(bf.METHODS, (T("Fast"), T("Exact"))):
            self.method.addItem(text, v)
        self.method.setToolTip(T("Fast: copies of the tool, plane sections and their outer boundary (seconds; sharp edges "
                                 "of the tool are slightly rounded, ~0.5 mm). Exact: exact sections and corner tracks "
                                 "(convex tools, minutes)"))
        self.method.setCurrentIndex(bf.METHODS.index(obj.Method))
        self.method.currentIndexChanged.connect(lambda i: self.set_prop("Method", bf.METHODS[i]))
        grid.addWidget(self.method, 4, 1, 1, 2)
        grid.addWidget(QtWidgets.QLabel(T("Offset")), 5, 0)
        self.off = QtWidgets.QDoubleSpinBox()
        self.off.setRange(-50.0, 100.0)
        self.off.setDecimals(3)
        self.off.setSingleStep(0.1)
        self.off.setSuffix(" mm")
        self.off.setKeyboardTracking(False)
        self.off.setToolTip(T("Clearance: the swept volume grows by this much on every side (e.g. a channel wider than "
                              "the part that runs through it); negative shrinks it"))
        self.off.setValue(float(obj.Offset))
        self.off.valueChanged.connect(lambda v: self.set_prop("Offset", float(v)))
        grid.addWidget(self.off, 5, 1)
        self.join = QtWidgets.QComboBox()
        for v, text in (("Round", T("Round")), ("Sharp", T("Sharp"))):
            self.join.addItem(text, v)
        self.join.setToolTip(T("Round: edges and corners of the clearance are rounded (exact distance everywhere). "
                               "Sharp: the faces are moved and intersected (sharp corners)"))
        self.join.setCurrentIndex(0 if obj.OffsetJoin == "Round" else 1)
        self.join.currentIndexChanged.connect(lambda i: self.set_prop("OffsetJoin", ("Round", "Sharp")[i]))
        grid.addWidget(self.join, 5, 2)
        grid.addWidget(QtWidgets.QLabel(T("Tolerance")), 3, 0)
        self.tol = QtWidgets.QDoubleSpinBox()
        self.tol.setRange(0.001, 1.0)
        self.tol.setDecimals(3)
        self.tol.setSingleStep(0.01)
        self.tol.setSuffix(" mm")
        self.tol.setKeyboardTracking(False)
        self.tol.setToolTip(T("Largest deviation of the envelope: smaller is more exact and slower (0.2 mm for a quick "
                              "draft)"))
        self.tol.setValue(float(obj.Tolerance))
        self.tol.valueChanged.connect(lambda v: self.set_prop("Tolerance", float(v)))
        grid.addWidget(self.tol, 3, 1, 1, 2)
        lay.addWidget(g)
        # hesap
        row = QtWidgets.QHBoxLayout()
        self.b_run = QtWidgets.QPushButton(T("Compute"))
        self.b_run.clicked.connect(lambda checked=False: self.run())
        self.b_stop = QtWidgets.QPushButton(T("Stop"))
        self.b_stop.clicked.connect(lambda checked=False: self.halt())
        row.addWidget(self.b_run)
        row.addWidget(self.b_stop)
        lay.addLayout(row)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        lay.addWidget(self.status)
        lay.addStretch()
        self.timer = QtCore.QTimer()
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.poll)
        self.refresh()

    # --- durum ---
    def refresh(self):
        o = self.obj
        self.tool_lbl.setText(_label(o.Tool))
        self.path_lbl.setText("\n".join(_label(a, b) for a, b in o.Path) or "-")
        self.sup_lbl.setText(_label(*o.SupportFace) if o.SupportFace is not None else "-")
        self.targets.clear()
        for t in o.Targets:
            self.targets.addItem(_label(t))
        busy = self.job is not None
        self.b_run.setEnabled(not busy)
        self.b_stop.setEnabled(busy)
        if busy:
            return
        got = bf.inputs(o)
        if isinstance(got, str):
            self.status.setText(T(got))
        elif got[4] == o.InputKey and not o.SweptShape.isNull():
            self.status.setText(T("Up to date"))
        else:
            self.status.setText(T("Press Compute (or OK) to build the sweep"))

    def set_prop(self, name, val):
        if getattr(self.obj, name) != val:
            setattr(self.obj, name, val)
        self.refresh()

    # --- secim ---
    def take_tool(self):
        tool, _, _ = bf.classify(Gui.Selection.getSelectionEx())
        if tool is None:
            App.Console.PrintWarning(T("Body Sweep: select the tool solid\n"))
            return
        if tool in self.obj.Targets:
            self._set_targets([t for t in self.obj.Targets if t is not tool])
        self.obj.Tool = tool
        self.refresh()

    def take_path(self):
        _, path, _ = bf.classify(Gui.Selection.getSelectionEx())
        if not path:
            App.Console.PrintWarning(T("Body Sweep: select the path (edges or a sketch)\n"))
            return
        self.obj.Path = path
        self.refresh()

    def take_support(self):
        for so in Gui.Selection.getSelectionEx():
            faces = [s for s in so.SubElementNames if s.split(".")[-1].startswith("Face")]
            if faces:
                self.obj.SupportFace = (so.Object, [faces[0]])
                self.refresh()
                return
        App.Console.PrintWarning(T("Body Sweep: select a face\n"))

    def clear_support(self):
        self.obj.SupportFace = None
        self.refresh()

    def _set_targets(self, items):
        self.obj.setPropertyStatus("Targets", "-ReadOnly")
        self.obj.Targets = items
        self.obj.setPropertyStatus("Targets", "ReadOnly")

    def add_targets(self):
        new = list(self.obj.Targets)
        for so in Gui.Selection.getSelectionEx():
            o = bf.body_of(so.Object) or so.Object
            if hasattr(o, "Shape") and o.Shape.Solids and o is not self.obj.Tool and o not in new and o is not self.obj:
                new.append(o)
        self._set_targets(new)
        self.refresh()

    def remove_target(self):
        r = self.targets.currentRow()
        items = list(self.obj.Targets)
        if 0 <= r < len(items):
            del items[r]
            self._set_targets(items)
            self.refresh()

    # --- arka plan hesabi ---
    def run(self):
        if self.job is not None:
            return
        got = bf.inputs(self.obj)
        if isinstance(got, str):
            self.status.setText(T(got))
            return
        tool, edges, sup, p, key = got
        job = dict(cancel=threading.Event(), msg=T("Starting..."), result=None, key=key, t0=time.time())

        def work():
            try:
                job["result"] = bf.compute(tool, edges, sup, p, progress=lambda s: job.__setitem__("msg", s),
                                           cancel=job["cancel"])
            except Exception as e:
                job["result"] = str(e) or e.__class__.__name__
        job["thread"] = threading.Thread(target=work, daemon=True)
        self.job = job
        job["thread"].start()
        self.timer.start()
        self.refresh()
        self.status.setText(T("Computing..."))

    def halt(self):
        if self.job is not None:
            self.job["cancel"].set()
            self.after_ok = False
            self.status.setText(T("Stopping..."))

    def poll(self):
        job = self.job
        if job is None:
            self.timer.stop()
            return
        if job["thread"].is_alive():
            self.status.setText("%s (%d s)\n%s" % (T("Computing..."), time.time() - job["t0"], job["msg"]))
            return
        self.timer.stop()
        self.job = None
        res = job["result"]
        if self.finished:
            return
        if isinstance(res, str) or res is None:
            self.after_ok = False
            self.refresh()
            self.status.setText(T("Stopped") if res == "cancelled" else T("Body Sweep: the sweep failed (%s)") % res)
            return
        o = self.obj
        o.SweptShape = res
        o.InputKey = job["key"]
        o.Pending = False
        self.apply_targets()
        self.refresh()
        self.status.setText(T("Up to date") + " (%d s)" % (time.time() - job["t0"]))
        if self.after_ok:                                  # OK hesap bekliyordu: zarf artik guncel
            self.after_ok = False
            self.accept()

    def apply_targets(self):
        o = self.obj
        bf.sync_targets(o)
        o.touch()
        self.doc.recompute()
        if App.GuiUp:
            o.ViewObject.Visibility = not o.Targets        # kesilen govde varsa zarf gizli
            for c in bf.cuts_of(o):
                b = bf.body_of(c)
                if b is not None and b.Tip is c and "Invalid" not in c.State:   # PartDesign gibi: yalniz uc gorunur
                    if c.BaseFeature is not None:
                        c.BaseFeature.ViewObject.Visibility = False
                    c.ViewObject.Visibility = True

    # --- bitis ---
    def getStandardButtons(self):
        import pda_common
        B = QtWidgets.QDialogButtonBox
        return pda_common.btn_flags(B.Ok, B.Cancel)

    def accept(self):
        if self.job is not None:                           # hesap suruyor: bitince kapanir
            self.after_ok = True
            return False
        if bf.is_stale(self.obj):
            got = bf.inputs(self.obj)
            if isinstance(got, str):
                self.status.setText(T(got))
                return False
            self.after_ok = True
            self.run()
            return False
        self.obj.Pending = False
        self.apply_targets()
        self.finish(True)
        return True

    def reject(self):
        if self.job is not None:
            self.job["cancel"].set()
        self.finish(False)
        return True

    def finish(self, ok):
        global _panel
        self.finished = True
        self.timer.stop()
        _panel = None
        if ok:
            self.doc.commitTransaction()
        else:
            self.doc.abortTransaction()                    # resetEdit acik islemi kapatir: once geri alinir
        Gui.ActiveDocument.resetEdit()
        Gui.Control.closeDialog()
        while not ok and self.fresh and self.doc.getObject(self.fresh) is not None and self.doc.UndoCount:
            self.doc.undo()                                # yeni nesne: olusturma adimi da gider
        self.doc.recompute()

    def close(self):
        if self.finished:
            return
        self.finished = True
        self.timer.stop()
        if self.job is not None:
            self.job["cancel"].set()
        self.doc.commitTransaction()
        Gui.Control.closeDialog()
