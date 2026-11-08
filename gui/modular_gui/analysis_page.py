"""Pagina 'Analisis': una corrida guardada (CSV) con graficas, metricas por
movimiento (lo mismo que tools/analizar_csv.py), trayectoria 3D y boton para
repetirla en 'En vivo'."""

import datetime
import os

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from . import kinematics as kin, runs, scene3d, theme, widgets as W

COLS = [("step", "Paso", "{:d}"), ("dist_cm", "Distancia\ncm", "{:.1f}"), ("dur_s", "Duración\ns", "{:.1f}"),
        ("lag_cm", "Atraso\ncm", "{:.2f}"), ("lateral_cm", "Lateral\ncm", "{:.2f}"),
        ("overshoot_mm", "Sobrepaso\nmm", "{:.1f}"), ("final_cm", "Error final\ncm", "{:.2f}"),
        ("vib_move", "Vib. moviendo\nNm", "{:.2f}"), ("vib_still", "Vib. quieto\nNm", "{:.2f}")]
# (llave, umbral amarillo, umbral rojo)
LIMITS = {"lateral_cm": (1.0, 2.0), "overshoot_mm": (3.0, 6.0), "final_cm": (1.0, 1.5),
          "vib_move": (1.5, 2.5), "vib_still": (1.0, 2.0), "lag_cm": (0.5, 1.0)}


class AnalysisPage(QtWidgets.QWidget):
    replay_requested = QtCore.pyqtSignal(object)   # Run

    def __init__(self, parent=None):
        super().__init__(parent)
        self.info = None
        self.run = None
        self.loader = None
        self.regions = []

        # ---------- encabezado: corrida + acciones ----------
        self.combo = QtWidgets.QComboBox()
        self.combo.setMinimumWidth(250)
        self.refresh_btn = QtWidgets.QPushButton("Actualizar")
        self.refresh_btn.setToolTip("Actualizar la lista de CSV")
        self.refresh_btn.clicked.connect(self.refresh_list)
        self.open_btn = QtWidgets.QPushButton("Abrir…")
        self.open_btn.clicked.connect(self._browse)
        # elegir una corrida la carga (sin boton "Cargar")
        self.combo.activated.connect(lambda _i: self.load(self.combo.currentData()))
        self.replay_btn = QtWidgets.QPushButton("Reproducir en En vivo")
        self.replay_btn.setObjectName("primary")
        self.replay_btn.setEnabled(False)
        self.replay_btn.clicked.connect(lambda: self.replay_requested.emit(self.run))
        head = QtWidgets.QHBoxLayout()
        head.setSpacing(10)
        tcol = QtWidgets.QVBoxLayout()
        tcol.setSpacing(2)
        tcol.addWidget(W.label("Análisis de corridas", obj="pageTitle"))
        self.page_sub = W.label("Un CSV de impedance_gen3: seguimiento, error y vibración por movimiento",
                                obj="pageSub")
        tcol.addWidget(self.page_sub)
        head.addLayout(tcol)
        head.addStretch(1)
        for w_ in (self.combo, self.refresh_btn, self.open_btn, self.replay_btn):
            head.addWidget(w_)

        self.c_dur = W.Kpi("Duración")
        self.c_steps = W.Kpi("Pasos")
        self.c_vibg = W.Kpi("Vibración global (J1–J4)")
        self.c_err = W.Kpi("Error final medio")
        self.c_status = W.Kpi("Estado")
        bar = W.Card(None, margins=16)
        bar.add(W.kpi_strip([self.c_dur, self.c_steps, self.c_vibg, self.c_err, self.c_status]))

        # ---------- graficas ----------
        X, Y, Z = theme.XYZ
        self.p_pos = W.plot(f"Posición de la muñeca [m] &nbsp; {W.swatch(X)} x &nbsp;"
                            f"{W.swatch(Y)} y &nbsp;{W.swatch(Z)} z &nbsp;&nbsp; "
                            f"<span style='color:{theme.MUTED};font-weight:400'>━ robot &nbsp; ╍ referencia"
                            " &nbsp; ┈ objetivo &nbsp; franjas: pasos alternos</span>")
        self.p_err = W.plot(f"Error [cm] &nbsp; {W.swatch(theme.ACCENT)} a la referencia (seguimiento) &nbsp; "
                            f"{W.swatch(theme.MUTED)} al objetivo")
        self.p_vib = W.plot("Vibración: torque rápido J1–J4 [Nm RMS, ventana 0.2 s]")
        self.p_aux = W.plot(f"{W.swatch(theme.ACCENT)} Pinza [%] &nbsp; {W.swatch(theme.CHART3)} Fixture [°]")
        plots = [self.p_pos, self.p_err, self.p_vib, self.p_aux]
        for p in plots[1:]:
            p.setXLink(self.p_pos)
        self.c_pos = [self.p_pos.plot(pen=W.pen(c, 2.0)) for c in theme.XYZ]
        self.c_ramp = [self.p_pos.plot(pen=W.pen(c, 1.2, "dash")) for c in theme.XYZ]
        self.c_tgt = [self.p_pos.plot(pen=W.pen(c, 1.0, "dot")) for c in theme.XYZ]
        self.c_err_t = self.p_err.plot(pen=W.pen(theme.MUTED, 1.2))
        self.c_err_r = self.p_err.plot(pen=W.pen(theme.ACCENT, 1.8))
        self.p_err.addLine(y=1.5, pen=W.pen(theme.TEXT, 1, "dash"), label="tolerancia",
                           labelOpts={"position": 0.02, "color": theme.MUTED, "anchors": [(0, 1), (0, 1)]})
        self.c_vib = self.p_vib.plot(pen=W.pen(theme.ACCENT, 1.6), fillLevel=0, brush=theme.ACCENT_FILL)
        self.p_vib.addLine(y=1.5, pen=W.pen(theme.TEXT, 1, "dash"), label="1.5 Nm",
                           labelOpts={"position": 0.02, "color": theme.MUTED, "anchors": [(0, 1), (0, 1)]})
        self.c_grip = self.p_aux.plot(pen=W.pen(theme.ACCENT, 1.8))
        self.c_fix = self.p_aux.plot(pen=W.pen(theme.CHART3, 1.8))
        self.p_aux.getPlotItem().setLabel("bottom", f"<span style='color:{theme.MUTED}'>tiempo [s]</span>")
        plot_w = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(plot_w)
        pl.setContentsMargins(0, 0, 0, 0)
        pl.setSpacing(10)
        for p, s in zip(plots, (3, 2, 2, 2)):
            pl.addWidget(p, s)

        # ---------- trayectoria 3D ----------
        self.scene = scene3d.SceneContent()
        self.path_tubes = []
        self.robot_ghost = scene3d.Robot(self.scene.renderer, opacity=0.35)
        self.robot_ghost.visible(False)
        self.view3d = scene3d.Scene3D(self.scene)

        tabs = QtWidgets.QTabWidget()
        tabs.addTab(plot_w, "Gráficas")
        tabs.addTab(self.view3d, "Trayectoria 3D")
        self.tabs = tabs

        # ---------- tabla ----------
        table_card = W.Card("Métricas por movimiento")
        self.table_meta = W.label("", obj="cardMeta")
        table_card.header_right(self.table_meta)
        self.table = QtWidgets.QTableWidget(0, len(COLS))
        self.table.setHorizontalHeaderLabels([c[1] for c in COLS])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.horizontalHeader().setMinimumSectionSize(44)
        self.table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.table.setMinimumWidth(520)
        self.table.setShowGrid(False)
        self.table.verticalHeader().setDefaultSectionSize(36)
        self.table.horizontalHeader().setDefaultAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self._num_font = theme.font(13, mono=True)   # cifras de la tabla en Plex Mono
        self.table.itemSelectionChanged.connect(self._zoom_to_row)
        table_card.add(self.table, 1)
        self.table_note = W.label(
            "Atraso: el robot detrás de la referencia (rampa). Lateral: fuera de la línea recta. "
            "Vib: torque rápido J1–J4. Clic en un renglón para acercar la gráfica. "
            "Se marcan en ámbar y rojo solo los valores sobre los umbrales.",
            obj="cardMeta", wrap=True)
        table_card.add(self.table_note)
        self.steps_label = W.label("", obj="cardMeta", wrap=True)

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.addWidget(tabs)
        split.addWidget(table_card)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        split.setSizes([920, 660])

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 24)
        lay.setSpacing(16)
        lay.addLayout(head)
        lay.addWidget(bar)
        lay.addWidget(split, 1)
        self.refresh_list()

    def set_task_info(self, info):
        self.info = info

    # ------------------------------------------------------------ lista
    def refresh_list(self):
        cur = self.combo.currentData()
        self.combo.clear()
        for f in runs.list_csvs():
            st = os.stat(f)
            when = datetime.datetime.fromtimestamp(st.st_mtime).strftime("%d/%m %H:%M:%S")
            self.combo.addItem(f"{when}   ·   {st.st_size / 1e6:.0f} MB", f)
            self.combo.setItemData(self.combo.count() - 1, os.path.basename(f), QtCore.Qt.ToolTipRole)
        if cur:
            i = self.combo.findData(cur)
            if i >= 0:
                self.combo.setCurrentIndex(i)

    def _browse(self):
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Abrir corrida", str(runs.config.BUILD_DIR), "CSV (*.csv)")
        if f:
            self.load(f)

    def load(self, path):
        if not path or (self.loader and self.loader.isRunning()):
            return
        self.c_status.set("Cargando…")
        self.combo.setEnabled(False)
        self.loader = runs.RunLoader(path, self)
        self.loader.loaded.connect(self._loaded)
        self.loader.failed.connect(self._failed)
        self.loader.start()

    def _failed(self, msg):
        self.combo.setEnabled(True)
        self.c_status.set("Error", theme.NEGATIVE)
        QtWidgets.QMessageBox.warning(self, "No se pudo cargar", msg)

    # ------------------------------------------------------------ mostrar
    def _loaded(self, run):
        self.run = run
        self.combo.setEnabled(True)
        self.replay_btn.setEnabled(run.joints_deg is not None)
        self.c_status.set("Cargada")
        name = os.path.basename(run.path)
        self.c_dur.set(f"{run.duration:.0f} s")
        steps = sorted(set(run.step.tolist()))
        self.c_steps.set(f"{steps[0]} a {steps[-1]}" if steps else "—")
        vib = run.vibration()
        vg = float(np.sqrt(np.mean(vib ** 2))) if vib is not None else float("nan")
        self.c_vibg.set(f"{vg:.2f} Nm RMS", None if vg < 1.0 else (theme.WARNING if vg < 1.5 else theme.NEGATIVE))
        finals = [m["final_cm"] for m in run.metrics]
        self.c_err.set(f"{np.mean(finals):.2f} cm" if finals else "—")

        # decimar para dibujar rapido
        k = max(1, run.n // 20000)
        t = run.t[::k]
        for j in range(3):
            self.c_pos[j].setData(t, run.pos[::k, j])
            self.c_ramp[j].setData(t, run.ramp[::k, j])
            self.c_tgt[j].setData(t, run.target[::k, j])
        self.c_err_t.setData(t, np.linalg.norm(run.pos - run.target, axis=1)[::k] * 100)
        self.c_err_r.setData(t, np.linalg.norm(run.err[:, :3], axis=1)[::k] * 100)
        vw = run.vibration_windowed()
        if vw is not None:
            self.c_vib.setData(t, vw[::k])
        if run.gripper is not None:
            self.c_grip.setData(t, run.gripper[::k])
        if run.fixture is not None:
            self.c_fix.setData(t, run.fixture[::k])
        self._shade_steps(run)
        self._fill_table(run)
        self._path3d(run)
        self.p_pos.getPlotItem().enableAutoRange()
        for p in (self.p_pos, self.p_err, self.p_vib, self.p_aux):
            p.getPlotItem().autoRange()
        # el error al objetivo salta con cada pose nueva: mostrar la zona util
        self.p_err.setYRange(0, 5)
        self.page_sub.setText(f"{name}, {run.n:,} muestras".replace(",", " "))
        self.table_meta.setText(f"{len(run.metrics)} movimientos")

    def _shade_steps(self, run):
        for p, item in self.regions:
            p.removeItem(item)
        self.regions = []
        band = QtGui.QColor(theme.RAISED)
        shade = False
        for k, (step, status, t0, t1) in enumerate(run.step_spans()):
            if status != 1 or t1 - t0 < 0.05:
                continue
            shade = not shade
            if shade:
                for p in (self.p_pos, self.p_err, self.p_vib, self.p_aux):
                    reg = pg.LinearRegionItem(values=(t0, t1), movable=False, brush=band,
                                              pen=pg.mkPen(None))
                    reg.setZValue(-10)
                    p.addItem(reg)
                    self.regions.append((p, reg))
            if t1 - t0 < 0.03 * max(run.duration, 1e-6):
                continue   # muy angosto: el numero se encima con el vecino
            txt = pg.TextItem(f"{step}", color=theme.MUTED, anchor=(0.5, 0))
            txt.setFont(theme.font(11))
            ymax = float(np.nanmax(run.pos)) if run.n else 0
            txt.setPos((t0 + t1) / 2, ymax + 0.02)
            self.p_pos.addItem(txt)
            self.regions.append((self.p_pos, txt))

    def _fill_table(self, run):
        ms = run.metrics
        self.table.setRowCount(len(ms))
        for i, m in enumerate(ms):
            for j, (key, _, fmt) in enumerate(COLS):
                v = m[key]
                txt = "—" if (isinstance(v, float) and not np.isfinite(v)) else fmt.format(v)
                it = QtWidgets.QTableWidgetItem(txt)
                it.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                it.setFont(self._num_font)
                if key in LIMITS and isinstance(v, float) and np.isfinite(v):
                    y, r = LIMITS[key]
                    if abs(v) >= y:
                        # color Y peso: no depende solo del color
                        bad = abs(v) >= r
                        it.setForeground(QtGui.QColor(theme.NEGATIVE if bad else theme.WARNING))
                        f = theme.font(13, 600 if bad else 500, mono=True)
                        it.setFont(f)
                        it.setToolTip(f"Sobre el umbral de {r if bad else y:g}")
                self.table.setItem(i, j, it)
        # columnas sin ningun dato en esta corrida (p. ej. Atraso sin rampa)
        for j, (key, _, _) in enumerate(COLS):
            empty = all(isinstance(m[key], float) and not np.isfinite(m[key]) for m in ms) if ms else False
            self.table.setColumnHidden(j, empty)

    def _zoom_to_row(self):
        rows = self.table.selectionModel().selectedRows()
        if not rows or self.run is None:
            return
        m = self.run.metrics[rows[0].row()]
        pad = max(1.0, 0.15 * (m["t1"] - m["t0"]))
        self.p_pos.setXRange(m["t0"] - pad, m["t1"] + pad + 2.0)

    def _path3d(self, run):
        r = self.scene.renderer
        for tb in self.path_tubes:
            r.RemoveActor(tb.actor)
        self.path_tubes = []
        R = np.array([kin.quat_to_matrix(*q) for q in run.quat[::50]])
        tcp = run.pos[::50] + R[:, :, 2] * kin.TCP_OFFSET_M
        steps = run.step[::50]
        cut = np.where(np.diff(steps) != 0)[0]
        starts = np.r_[0, cut + 1]
        ends = np.r_[cut + 1, len(steps) - 1]
        for a, b in zip(starts, ends):
            if b - a < 1:
                continue
            col = theme.ACCENT if len(self.path_tubes) % 2 == 0 else theme.ACCENT_SOFT
            tb = scene3d.Tube(r, col, 0.004, 0.95)
            tb.set(tcp[a:b + 1])
            self.path_tubes.append(tb)
        if run.joints_deg is not None:
            self.robot_ghost.set_joints(run.joints_deg[-1], run.gripper[-1] if run.gripper is not None else None)
            self.robot_ghost.visible(True)
        if len(tcp):
            self.scene.fit(np.vstack([tcp, [[0, 0, 0]]]))
        self.view3d.request_render()
