"""Deteccion del hoyo en pantalla: vista superior de la placa (base x-y) con
todos los hoyos que encuentra el algoritmo y el ELEGIDO (el mas chico), que es
donde el controlador suelta el tornillo.

Mismo algoritmo y constantes que detectHoles() del screw_server (holes.py).
Lee la foto mas nueva hoyo_*.pcd del servidor de vision en uso; el calculo
corre en otro hilo para no trabar la interfaz. Solo muestra.
"""

import os
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from . import config, holes, kinematics as kin, parsers, theme, widgets as W

OTHER = "#7d838a"
_pool = ThreadPoolExecutor(max_workers=1)
_cache = {}   # nombre -> resultado (las fotos no cambian)


def newest_hole_capture(since=0.0):
    """Nombre de la foto de hoyo mas nueva del servidor en uso (o None)."""
    d = config.VISION_BUILD_DIR
    if not d.exists():
        return None
    files = [f for f in d.glob("hoyo_*.pcd") if os.path.getmtime(f) >= since]
    return max(files, key=os.path.getmtime).stem if files else None


def analyze(name):
    """Nube a la base + deteccion. Devuelve un dict para dibujar."""
    if name in _cache:
        return _cache[name]
    tool = parsers.read_tool_pose(config.capture_path(name, "_pose.txt"))
    pts = parsers.read_pcd(config.capture_path(name))
    if tool is None or not len(pts):
        return {"name": name, "ok": False, "why": "foto sin pose"}
    P = kin.cloud_to_base(pts, tool)
    P = P[np.isfinite(P).all(1)]
    res = holes.detect(P)
    out = {"name": name, "ok": res["ok"], "why": res["why"], "holes": res["holes"],
           "tilt": res.get("tilt_deg")}
    if "normal" in res:
        on_plate = np.abs((P - res["plane_point"]) @ res["normal"]) < holes.PLATE_BAND
        plate = P[on_plate]
        rng = np.random.default_rng(0)
        if len(plate) > 14000:
            plate = plate[rng.choice(len(plate), 14000, replace=False)]
        out["plate_xy"] = plate[:, :2]
    _cache[name] = out
    return out


class HoleView(QtWidgets.QWidget):
    """Vista superior de la placa con los hoyos. big=True para la presentacion."""

    changed = QtCore.pyqtSignal(str)   # nombre de la foto que se muestra

    def __init__(self, big=False, parent=None):
        super().__init__(parent)
        self.big = big
        self.name = None
        self.since = 0.0
        self.server_hole = None   # (centro, diametro mm) que acepto el controlador
        self._future = None
        self._last = None         # ultimo resultado dibujado
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.setMenuEnabled(False)
        self.plot.hideButtons()
        pi = self.plot.getPlotItem()
        pi.setLabel("bottom", "x base [m]")
        pi.setLabel("left", "y base [m]")
        pi.showGrid(x=True, y=True, alpha=0.15)
        for a in ("bottom", "left"):
            ax = pi.getAxis(a)
            ax.setPen(pg.mkPen(theme.BORDER))
            ax.setTextPen(pg.mkPen(theme.MUTED))
        self.plot.setMinimumHeight(220)
        lay.addWidget(self.plot, 1)
        self.summary = W.label("Esperando la foto de la placa (paso de visión del hoyo).",
                               color=theme.MUTED, wrap=True,
                               size=theme.PS_LABEL if big else None)
        lay.addWidget(self.summary)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._poll)

    # ------------------------------------------------------------ control
    def start(self, since=0.0):
        """Empieza a seguir las fotos de hoyo nuevas (desde 'since')."""
        self.since = since
        if not self._timer.isActive():
            self._timer.start(800)
        self._poll()

    def stop(self):
        self._timer.stop()

    def set_server_hole(self, center, diam_mm):
        new = (np.asarray(center, float), float(diam_mm)) if center is not None else None
        same = (new is None and self.server_hole is None) or (
            new is not None and self.server_hole is not None and np.allclose(new[0], self.server_hole[0]))
        self.server_hole = new
        if not same and self._last is not None:
            self._draw(self._last)

    def show_capture(self, name):
        if not name or name == self.name:
            return
        self.name = name
        self.summary.setText(f"Analizando {name}…")
        self._future = _pool.submit(analyze, name)

    def _poll(self):
        name = newest_hole_capture(self.since)
        if name and name != self.name:
            self.show_capture(name)
        if self._future is not None and self._future.done():
            f, self._future = self._future, None
            try:
                self._draw(f.result())
            except Exception as e:   # foto a medio escribir, etc.
                self.name = None
                self.summary.setText(f"No se pudo leer la foto: {e}")

    # ------------------------------------------------------------ dibujo
    def _draw(self, r):
        self._last = r
        pi = self.plot.getPlotItem()
        pi.clear()
        if "plate_xy" in r:
            xy = r["plate_xy"]
            pi.addItem(pg.ScatterPlotItem(xy[:, 0], xy[:, 1], size=2 if self.big else 1.5, pen=None,
                                          brush=pg.mkBrush(theme.CLOUD_LOW)))
        hs = r.get("holes", [])
        t = np.linspace(0, 2 * np.pi, 97)
        fs = 15 if self.big else 11
        for k, h in reversed(list(enumerate(hs))):
            c = h["center"]
            R = h["diameter_m"] / 2
            chosen = k == 0
            col = theme.ACCENT if chosen else OTHER
            pi.addItem(pg.PlotDataItem(c[0] + R * np.cos(t), c[1] + R * np.sin(t),
                                       pen=pg.mkPen(col, width=(4 if self.big else 3) if chosen else 1.6),
                                       fillLevel=None))
            txt = f"Ø {h['diameter_m'] * 1000:.1f} mm" + ("  ← ELEGIDO" if chosen else "")
            # el elegido arriba del circulo, los otros abajo: no se enciman
            lab = pg.TextItem(html=f"<span style='color:{col}; font-size:{fs}px; "
                                   f"font-weight:{700 if chosen else 400}'>{txt}</span>",
                              anchor=(0.5, 1) if chosen else (0.5, 0))
            lab.setPos(c[0], c[1] + R + 0.003 if chosen else c[1] - R - 0.004)
            pi.addItem(lab)
        if hs:
            c = hs[0]["center"]
            s = 0.008
            pen = pg.mkPen(theme.ACCENT, width=2)
            pi.addItem(pg.PlotDataItem([c[0] - s, c[0] + s], [c[1], c[1]], pen=pen))
            pi.addItem(pg.PlotDataItem([c[0], c[0]], [c[1] - s, c[1] + s], pen=pen))
        if self.server_hole is not None:
            c, _ = self.server_hole
            pi.addItem(pg.ScatterPlotItem([c[0]], [c[1]], size=14 if self.big else 11, symbol="o",
                                          pen=pg.mkPen(theme.CHART3, width=2), brush=None))
        pi.autoRange(padding=0.08)
        self.summary.setText(self._summary(r))
        self.summary.setStyleSheet(f"color: {theme.TEXT if r['ok'] else theme.WARNING};"
                                   + (f" font-size: {theme.PS_LABEL}px;" if self.big else ""))
        self.changed.emit(r["name"])

    def _summary(self, r):
        hh = r["name"].split("_")[-1]
        when = f"{hh[:2]}:{hh[2:4]}:{hh[4:6]}" if len(hh) == 6 else r["name"]
        if not r["ok"]:
            return f"Foto de las {when}: {r['why']}."
        hs = r["holes"]
        best = hs[0]
        c = best["center"]
        others = ", ".join(f"{h['diameter_m'] * 1000:.1f}" for h in hs[1:])
        txt = (f"<b>{len(hs)} hoyo{'s' if len(hs) != 1 else ''} detectado{'s' if len(hs) != 1 else ''}</b>"
               f" (foto de las {when}). <b style='color:{theme.ACCENT}'>Elegido: Ø "
               f"{best['diameter_m'] * 1000:.1f} mm</b>, el más chico, en x {c[0]:.3f}, y {c[1]:.3f} m")
        if others:
            txt += f". Los otros: {others} mm"
        if self.server_hole is not None:
            d = np.linalg.norm(self.server_hole[0][:2] - c[:2]) * 1000
            txt += (f". <span style='color:{theme.CHART3}'>○</span> el que aceptó el controlador"
                    + (f" (a {d:.1f} mm)" if d > 0.5 else ""))
        return txt + "."


def hole_phase(steps):
    """(paso de la foto del hoyo, ultimo paso que lo usa) de la secuencia, o
    None. Incluye abrir la pinza justo despues de bajar al hoyo."""
    photo = next((s["n"] for s in steps if s.get("type") == "vision" and s.get("vision_needs_hole")), None)
    if photo is None:
        return None
    uses = [s["n"] for s in steps if s.get("type") == "move" and s.get("vision_target") == "hole"]
    last = max(uses) if uses else photo
    by_n = {s["n"]: s for s in steps}
    if by_n.get(last + 1, {}).get("type") == "gripper":
        last += 1
    return photo, last


class HolePhase:
    """Decide cuando mostrar la vista del hoyo (de la foto hasta soltar el
    tornillo) y desde cuando buscar la foto nueva."""

    def __init__(self, view):
        self.view = view
        self.range = None
        self.active = False

    def set_steps(self, steps):
        self.range = hole_phase(steps)

    def update(self, n, status, st, live=True):
        """True si la vista debe estar visible."""
        on = bool(live and self.range and n and self.range[0] <= n <= self.range[1]
                  and status not in ("aborted",))
        if on and not self.active:
            # entrando en la foto: solo fotos nuevas; ya despues (la GUI se abrio
            # a media corrida): la mas nueva que haya
            self.view.start(since=time.time() - 3.0 if n == self.range[0] else 0.0)
        elif not on and self.active:
            self.view.stop()
        self.active = on
        if on:
            self.view.set_server_hole(st.get("vision_hole"), st.get("vision_hole_diam_mm", float("nan")))
        return on
