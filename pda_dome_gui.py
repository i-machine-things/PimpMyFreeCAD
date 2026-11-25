# -*- coding: utf-8 -*-
# Pimp My FreeCAD - Dome: gorev paneli. Copyright (c) 2026 Sedat Kuran - MIT License
#
# Degisiklikler Apply (ya da OK) ile uygulanir: her sayi girisinde yeniden hesap buyuk govdelerde sistemi yoruyordu.
# Ayri onizleme yok, uygulanan kati gorunur. OK degisiklikleri tek geri alma adimi olarak birakir, Cancel geri alir;
# yeni nesneyi de siler.
import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtCore, QtWidgets

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


class Panel(object):
    def __init__(self, obj):
        self.obj, self.doc = obj, obj.Document
        self.fresh = obj.Name if fresh is obj else None
        self.finished = False
        self.doc.openTransaction(T("Edit Dome"))
        self.form = QtWidgets.QWidget()
        self.form.setWindowTitle(T("Dome"))
        lay = QtWidgets.QFormLayout(self.form)
        self.height = QtWidgets.QDoubleSpinBox()
        self.height.setRange(0.01, 100000.0)
        self.height.setDecimals(3)
        self.height.setSuffix(" mm")
        self.height.setValue(float(obj.Height))
        lay.addRow(T("Height"), self.height)
        self.rev = QtWidgets.QCheckBox(T("Reversed (cut into the solid)"))
        self.rev.setChecked(bool(obj.Reversed))
        lay.addRow(self.rev)
        self.ell = QtWidgets.QCheckBox(T("Tangent to steep walls (bullet shape)"))
        self.ell.setToolTip(T("Off: a smooth bump with a free slope at steep walls"))
        self.ell.setChecked(bool(obj.Elliptical))
        lay.addRow(self.ell)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        lay.addRow(self.status)
        self.dirty = False
        self.height.valueChanged.connect(lambda v: self.changed())
        self.rev.toggled.connect(lambda v: self.changed())
        self.ell.toggled.connect(lambda v: self.changed())
        self.show_status()

    def changed(self):
        """Degisiklik yalniz isaretlenir; hesap Apply / OK ile."""
        self.dirty = (abs(self.height.value() - float(self.obj.Height)) > 1e-9
                      or self.rev.isChecked() != bool(self.obj.Reversed)
                      or self.ell.isChecked() != bool(self.obj.Elliptical))
        self.show_status()

    def apply(self):
        if not self.dirty:
            return
        self.status.setText(T("Computing..."))
        self.status.setStyleSheet("")
        QtWidgets.QApplication.processEvents()
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            self.obj.Height = self.height.value()
            self.obj.Reversed = self.rev.isChecked()
            self.obj.Elliptical = self.ell.isChecked()
            self.doc.recompute()
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self.dirty = False
        self.show_status()

    def clicked(self, button):
        """Gorev panelinin Apply dugmesi (FreeCAD tamsayi dugme kodu verir)."""
        B = QtWidgets.QDialogButtonBox
        if int(getattr(button, "value", button)) == int(getattr(B.Apply, "value", B.Apply)):
            self.apply()

    def show_status(self):
        if self.dirty:
            self.status.setText(T("Not applied yet: press Apply (or OK) to rebuild the dome"))
            self.status.setStyleSheet("color: #e8a33d;")
            return
        self.status.setStyleSheet("")
        o = self.obj
        if "Invalid" in o.State:
            self.status.setText(str(o.getStatusString()))
            return
        lines = [T("Tangent error at the edges: %.3f degrees") % float(o.TangentError)]
        if o.SteepEdges:
            lines.append(T("%d edge(s) next to a steep wall are not tangent") % o.SteepEdges)
        self.status.setText("\n".join(lines))

    def getStandardButtons(self):
        import pda_common
        B = QtWidgets.QDialogButtonBox
        return pda_common.btn_flags(B.Ok, B.Apply, B.Cancel)

    def accept(self):
        self.apply()
        if "Invalid" in self.obj.State:                    # hatali yukseklikle cikilmaz: metin panelde
            return False
        self.finish(True)
        return True

    def reject(self):
        self.finish(False)
        return True

    def finish(self, ok):
        global _panel
        self.finished = True
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
        self.doc.commitTransaction()
        Gui.Control.closeDialog()
