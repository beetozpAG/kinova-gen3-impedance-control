"""Pagina 'Vision': capturas del screw_server en 3D (nube en la base + agarre
+ pre-grasp), veredicto con las mismas reglas del controlador, histograma de
profundidad y foto de prueba (vision_request_test: NO mueve el robot)."""

import os
import re

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from . import config, holes, kinematics as kin, parsers, plate_edge, scene3d, theme, widgets as W


def read_max_depth():
    """Profundidad maxima de la nube (solo para dibujarla): MAX_DEPTH de
    kinova_grasp_V2/src/depth_camera.h. La version nueva no recorta (solo las
    fotos de hoyos, a 0.34 m en el servidor): None."""
    if config.VISION_VERSION != "v2":
        return None
    try:
        txt = (config.KG_V2_DIR / "src" / "depth_camera.h").read_text()
        m = re.search(r"const float MAX_DEPTH\s*=\s*([\d.]+)f?;", txt)
        return float(m.group(1)) if m else None
    except OSError:
        return None


def verdict(grasp, tool_pose, info):
    """Mismas revisiones que task::Sequencer::checkVision. -> (ok, [motivos], datos)."""
    p = info.get("params", {}) if info else {}
    lo = np.array(info.get("workspace_min_m", [-9, -9, -9])) if info else np.full(3, -9.0)
    hi = np.array(info.get("workspace_max_m", [9, 9, 9])) if info else np.full(3, 9.0)
    why, data = [], {}
    pos = grasp.get("position")
    if pos is None:
        return False, ["sin agarre"], data
    def inside(a, b):
        return bool(np.all(pos >= a) and np.all(pos <= b))
    low = info.get("low_zone_min_m") if info else None
    if not inside(lo, hi) and not (low is not None and inside(np.array(low), np.array(info["low_zone_max_m"]))):
        why.append("agarre fuera de la caja de trabajo")
    if tool_pose is not None:
        dxy = float(np.linalg.norm(pos[:2] - tool_pose[:2]))
        data["dxy"] = dxy
        if dxy > p.get("vision_max_xy_m", 0.2):
            why.append(f"a {dxy * 100:.0f} cm de la foto en horizontal (máx {p.get('vision_max_xy_m', 0.2) * 100:.0f})")
        if pos[2] > tool_pose[2] - p.get("vision_min_below_m", 0.03):
            why.append("no queda debajo de la foto")
    rpy = grasp.get("rpy")
    if rpy is not None:
        z = kin.rpy_to_matrix(rpy)[:, 2]
        tilt = float(np.degrees(np.arccos(np.clip(-z[2], -1, 1))))
        data["tilt"] = tilt
        if tilt > p.get("vision_max_tilt_deg", 20.0):
            why.append(f"pinza inclinada {tilt:.0f}° (máx {p.get('vision_max_tilt_deg', 20):.0f}°)")
    pre = grasp.get("pregrasp")
    if pre is not None:
        if np.any(pre < lo) or np.any(pre > hi):
            why.append("pre-grasp fuera de la caja")
        if np.linalg.norm(pre - pos) > p.get("vision_max_pregrasp_m", 0.15) or pre[2] <= pos[2]:
            why.append("pre-grasp no queda encima del agarre")
    return not why, why, data


class VisionPage(QtWidgets.QWidget):
    request_photo = QtCore.pyqtSignal(list)      # pose TCP [x y z rx ry rz]
    compute_grasp = QtCore.pyqtSignal(str)       # nombre de la captura
    manual_clicked = QtCore.pyqtSignal()         # procedimiento de Antony a mano

    def __init__(self, parent=None):
        super().__init__(parent)
        self.info = None
        self.results = {}        # nombre -> captura (dict de parsers)
        self.known = []
        self.current = None
        self.max_depth = read_max_depth()

        # ---------- 3D ----------
        self.scene = scene3d.SceneContent()
        r = self.scene.renderer
        self.cloud = scene3d.PointCloud(r, 3.0)
        self.cloud.visible(False)
        self.grasp_g = scene3d.Gripper(r, body=theme.hex_rgb(theme.ACCENT), finger=theme.hex_rgb(theme.ACCENT_SOFT), opacity=0.75)
        self.pre_g = scene3d.Gripper(r, body=theme.hex_rgb(theme.NEUTRAL), finger=theme.hex_rgb(theme.NEUTRAL), opacity=0.25)
        self.approach = scene3d.Tube(r, theme.CHART3, 0.0025, 0.9)
        self.cam_frame = scene3d.Frame(r, length=0.06, radius=0.003)
        self.cam_dot = scene3d.Sphere(r, theme.TEXT, 0.01)
        self.cam_label = scene3d.Label(r, "cámara", theme.TEXT, 12)
        # canto de la placa: la linea y la distancia a cada lado de la pinza
        self.edge_tube = scene3d.Tube(r, theme.CHART3, 0.003, 0.95)
        self.edge_ends = [scene3d.Sphere(r, theme.CHART3, 0.006) for _ in range(2)]
        self.edge_labels = [scene3d.Label(r, "", theme.TEXT, 14) for _ in range(2)]
        self.edge_items = [self.edge_tube] + self.edge_ends + self.edge_labels
        # hoyos de la placa (fotos hoyo_*): circulo de cada uno, el mas chico resaltado
        self.hole_rings = [scene3d.Tube(r, theme.ACCENT if k == 0 else theme.NEUTRAL, 0.0012 if k else 0.002, 0.95)
                           for k in range(6)]
        self.hole_dot = scene3d.Sphere(r, theme.ACCENT, 0.003)
        self.hole_labels = [scene3d.Label(r, "", theme.TEXT, 13) for _ in range(6)]
        self.hole_items = self.hole_rings + self.hole_labels + [self.hole_dot]
        for a in self.hole_items:
            a.visible(False)
        for a in self.edge_items:
            a.visible(False)
        for a in (self.grasp_g, self.pre_g, self.approach, self.cam_frame, self.cam_dot, self.cam_label):
            a.visible(False)
        self.view3d = scene3d.Scene3D(self.scene)

        # ---------- derecha ----------
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(16)
        right.setContentsMargins(0, 0, 0, 0)

        res = W.Card("Resultado")
        self.res_meta = W.label("", obj="cardMeta")
        res.header_right(self.res_meta)
        self.verdict_chip = W.Chip("Veredicto con las reglas del controlador")
        res.add(self.verdict_chip)
        self.why = W.label("", color=theme.WARNING, wrap=True)
        res.add(self.why)
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(14)
        self.vals = {}
        for k, (key, title) in enumerate([("score", "Puntaje de GPD"), ("z", "z del agarre [m]"), ("tilt", "Inclinación"),
                                          ("open", "Apertura"), ("pre", "z de la pre-grasp [m]"), ("yaw", "Giro Rz")]):
            box = QtWidgets.QVBoxLayout()
            box.setSpacing(2)
            t = W.label(title, obj="kpiLabel")
            t.setMinimumHeight(18)
            v = W.label("—", size=22)
            v.setMinimumHeight(30)
            v.setStyleSheet(f"font-size: 22px; font-family: \"{theme.UI_FONT}\";")
            box.addWidget(t)
            box.addWidget(v)
            grid.addLayout(box, k // 3, k % 3)
            self.vals[key] = v
        res.add(grid)
        self.edge_cb = QtWidgets.QCheckBox("Medir el canto de la placa")
        self.edge_cb.setToolTip("Para la placa parada: cuánta placa queda a cada lado de la pinza, "
                                "a lo largo del canto. Se activa solo en las fotos de la placa.")
        self.edge_cb.toggled.connect(lambda _on: self.current and self._show(self.current, auto_edge=False))
        res.add(self.edge_cb)
        self.edge_note = W.label("", wrap=True)
        self.edge_note.setVisible(False)
        res.add(self.edge_note)
        self.calc_btn = QtWidgets.QPushButton("Calcular agarre con GPD (sin robot)")
        self.calc_btn.clicked.connect(self._on_calc)
        self.calc_btn.setVisible(False)
        res.add(self.calc_btn)
        # la tarjeta conserva su alto (no se aplastan las cifras); cede el histograma
        res.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        right.addWidget(res)

        hist = W.Card("Distancia a la cámara")
        self.p_hist = W.plot()
        self.p_hist.setMinimumHeight(150)
        self.hist_curve = self.p_hist.plot(stepMode="center", fillLevel=0, brush=theme.ACCENT_FILL,
                                           pen=W.pen(theme.ACCENT, 1.5))
        self.p_hist.getPlotItem().setLabel("bottom", "<span style='color:%s'>distancia [m]</span>" % theme.MUTED)
        self.depth_line = None
        if self.max_depth:
            self.depth_line = self.p_hist.addLine(x=self.max_depth, pen=W.pen(theme.TEXT, 1, "dash"))
        hist.add(self.p_hist)
        self.hist_note = W.label("", obj="cardMeta", wrap=True)
        hist.add(self.hist_note)
        right.addWidget(hist)

        caps = W.Card("Capturas")
        self.follow = QtWidgets.QCheckBox("Mostrar la más nueva")
        self.follow.setChecked(True)
        caps.header_right(self.follow)
        self.cap_list = QtWidgets.QListWidget()
        self.cap_list.setMinimumHeight(120)
        self.cap_list.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.cap_list.setTextElideMode(QtCore.Qt.ElideRight)
        self.cap_list.currentRowChanged.connect(self._on_select)
        caps.add(self.cap_list, 1)
        right.addWidget(caps, 1)

        left = W.Card("Nube de puntos en la base")
        legend = W.label("", obj="cardMeta")
        legend.setText(f"{W.swatch(theme.ACCENT, 'box')} agarre &nbsp;&nbsp; "
                       f"{W.swatch(theme.NEUTRAL, 'box')} pre-grasp &nbsp;&nbsp; "
                       f"{W.swatch(theme.CHART3)} aproximación &nbsp;&nbsp; "
                       f"{W.swatch(theme.TEXT, 'dot1')} cámara &nbsp;&nbsp; nube: más oscura, más alta")
        left.header_right(legend)
        left.add(self.view3d, 1)

        # foto de prueba: arriba, junto al titulo (es la accion de esta pagina)
        self.pose_combo = QtWidgets.QComboBox()
        self.pose_combo.setMinimumWidth(240)
        self.photo_btn = QtWidgets.QPushButton("Tomar foto de prueba")
        self.photo_btn.setObjectName("primary")
        self.photo_btn.clicked.connect(self._on_photo)
        self.photo_hint = W.label("El robot debe estar en esa pose. No lo mueve.", obj="cardMeta")

        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(24, 20, 24, 24)
        outer.setSpacing(16)
        head = QtWidgets.QHBoxLayout()
        head.setSpacing(10)
        tcol = QtWidgets.QVBoxLayout()
        tcol.setSpacing(2)
        tcol.addWidget(W.label("Visión", obj="pageTitle"))
        tcol.addWidget(W.label("Cada foto del servidor de visión, con el agarre de GPD y las reglas del controlador",
                               obj="pageSub"))
        head.addLayout(tcol)
        head.addStretch(1)
        pc = QtWidgets.QVBoxLayout()
        pc.setSpacing(4)
        prow = QtWidgets.QHBoxLayout()
        prow.setSpacing(10)
        prow.addWidget(W.label("Pose", color=theme.MUTED))
        prow.addWidget(self.pose_combo)
        prow.addWidget(self.photo_btn)
        self.manual_btn = QtWidgets.QPushButton("Procedimiento a mano…")
        self.manual_btn.setToolTip("approach_object y grasps_from_camera de Antony, sin el controlador")
        self.manual_btn.clicked.connect(self.manual_clicked.emit)
        prow.addWidget(self.manual_btn)
        pc.addLayout(prow)
        self.photo_hint.setAlignment(QtCore.Qt.AlignRight)
        pc.addWidget(self.photo_hint)
        head.addLayout(pc)
        outer.addLayout(head)

        lay = QtWidgets.QHBoxLayout()
        lay.setSpacing(16)
        lay.addWidget(left, 5)
        rw = QtWidgets.QWidget()
        rw.setLayout(right)
        rw.setMinimumWidth(440)
        rw.setMaximumWidth(500)
        lay.addWidget(rw, 2)
        outer.addLayout(lay, 1)

        self._scan = QtCore.QTimer(self)
        self._scan.timeout.connect(self.scan)
        self._scan.start(1500)
        self.photo_allowed = True

    # ------------------------------------------------------------ datos
    def set_task_info(self, info):
        self.info = info
        self.pose_combo.clear()
        names = [p["name"] for p in info.get("poses", [])]
        for n in names:
            self.pose_combo.addItem(n)
        vision_poses = [s["pose"] for s in info.get("steps", []) if s["type"] == "vision"]
        if vision_poses and vision_poses[0] in names:
            self.pose_combo.setCurrentText(vision_poses[0])
        if self.current:
            self._show(self.current)   # p. ej. decidir si es foto de la placa

    def set_results(self, captures):
        """Capturas leidas de los logs del servidor (o de detect_ee_pose)."""
        changed = False
        for c in captures:
            old = self.results.get(c["name"])
            if old is None or (c.get("grasps") and not old.get("grasps")) or c.get("seq") != old.get("seq"):
                self.results[c["name"]] = c
                changed = True
        if changed and self.current:
            self._show(self.current)
            self._refresh_items()

    def attach_to_newest(self, grasp, since=0.0, source=""):
        """Pone un agarre (de la foto de prueba o del controlador) en la
        captura mas nueva tomada despues de 'since' (time.time())."""
        files = sorted(config.VISION_BUILD_DIR.glob("screw_*.pcd"), key=os.path.getmtime, reverse=True)
        if not files or os.path.getmtime(files[0]) < since - 1.0:
            return
        name = files[0].stem
        old = self.results.get(name)
        if old and old.get("grasps") and old.get("source") == "server":
            return   # el log del servidor manda (trae score y apertura)
        cap = {"name": name, "source": source}
        if grasp.get("fail"):
            cap.update(ok=False, answer=grasp["fail"], grasps=[])
        else:
            cap.update(ok=True, grasps=[dict(grasp, score=grasp.get("score", float("nan")))])
        self.results[name] = cap
        self.scan()
        self._refresh_items()
        if self.current == name or self.follow.isChecked():
            self._show(name)

    def set_photo_allowed(self, ok, why=""):
        self.photo_allowed = ok
        self.photo_btn.setEnabled(ok)
        self.photo_hint.setText(why or "El robot debe estar en esa pose. No lo mueve.")

    def scan(self):
        # screw_*: fotos con GPD; hoyo_*: solo nube, encima del hoyo (sin agarre)
        files = sorted([f for d in config.CAPTURE_DIRS if d.exists()
                        for f in (*d.glob("screw_*.pcd"), *d.glob("hoyo_*.pcd"), *d.glob("approach_*.pcd"))],
                       key=os.path.getmtime, reverse=True)
        names = [f.stem for f in files]
        if names == self.known:
            return
        new = [n for n in names if n not in self.known]
        self.known = names
        self._refresh_items()
        if self.follow.isChecked() and new:
            self.cap_list.setCurrentRow(0)
        elif self.current is None and names:
            self.cap_list.setCurrentRow(0)

    def _refresh_items(self):
        cur = self.current
        self.cap_list.blockSignals(True)
        self.cap_list.clear()
        for n in self.known:
            r = self.results.get(n)
            hh = n.split("_")[-1]
            stamp = f"{hh[:2]}:{hh[2:4]}:{hh[4:6]}" if len(hh) == 6 else hh
            if r is None or not r.get("grasps"):
                txt, col = f"{stamp}    sin resultado", theme.MUTED
                if r is not None and r.get("ok") is False:
                    txt, col = f"{stamp}    falla: {r.get('answer', '')}", theme.NEGATIVE
            else:
                g = r["grasps"][0]
                ok, _, _ = verdict(g, parsers.read_tool_pose(config.capture_path(n, "_pose.txt")), self.info)
                sc = g.get("score", float("nan"))
                txt = (f"{stamp}    " + ("aceptado" if ok else "rechazado")
                       + f"    z {g.get('position', [0, 0, 0])[2]:.3f} m"
                       + (f"    score {sc:.0f}" if np.isfinite(sc) else "")
                       + (f"    {r['source']}" if r.get("source") and r["source"] != "server" else ""))
                col = theme.TEXT if ok else theme.WARNING
            it = QtWidgets.QListWidgetItem(txt)
            it.setForeground(QtGui.QColor(col))
            it.setData(QtCore.Qt.UserRole, n)
            self.cap_list.addItem(it)
            if n == cur:
                self.cap_list.setCurrentItem(it)
        self.cap_list.blockSignals(False)

    def _on_select(self, row):
        it = self.cap_list.item(row)
        if it:
            self._show(it.data(QtCore.Qt.UserRole))

    def _plate_capture(self, tool):
        """True si la foto se tomo en la pose de un paso de vision de la placa."""
        if tool is None or not self.info:
            return False
        poses = {p["name"]: np.array(p["tcp_m"]) for p in self.info.get("poses", [])}
        for s in self.info.get("steps", []):
            if s["type"] == "vision" and "placa" in s["label"].lower() and s["pose"] in poses:
                if np.linalg.norm(poses[s["pose"]] - np.asarray(tool[:3])) < 0.05:
                    return True
        return False

    def _show_edge(self, pts, tool, g):
        """Canto de la placa: distancia de placa a cada lado de la pinza."""
        for a in self.edge_items:
            a.visible(False)
        self.edge_note.setVisible(False)
        self._edge_m = None
        if not self.edge_cb.isChecked() or g is None or tool is None:
            return
        m = plate_edge.measure(pts, tool, g)
        self._edge_m = m
        self.edge_note.setText(plate_edge.describe(m))
        self.edge_note.setStyleSheet(f"color: {theme.TEXT if m.get('ok') else theme.WARNING};")
        self.edge_note.setVisible(True)
        if not m.get("ok"):
            return
        self.edge_tube.set([m["end_neg"], m["end_pos"]])
        for k, (end, d, tr, side) in enumerate([(m["end_neg"], m["dist_neg_m"], m["trunc_neg"], "−y"),
                                                (m["end_pos"], m["dist_pos_m"], m["trunc_pos"], "+y")]):
            self.edge_ends[k].set(end)
            self.edge_labels[k].set(end + [0, 0, 0.02], f"{side}  {'≥ ' if tr else ''}{d * 100:.1f} cm")
        for a in self.edge_items:
            a.visible(True)

    def _show_holes(self, name, pts, tool):
        """Fotos del hoyo: los hoyos que encuentra screw_server (mismo algoritmo)."""
        for a in self.hole_items:
            a.visible(False)
        if not name.startswith("hoyo_") or tool is None or not len(pts):
            return
        res = holes.detect(kin.cloud_to_base(pts, tool))
        if not res["ok"]:
            self.edge_note.setText("Hoyos: " + res["why"] + ".")
            self.edge_note.setStyleSheet(f"color: {theme.WARNING};")
            self.edge_note.setVisible(True)
            return
        n = res["normal"]
        e1 = np.cross(n, [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        t = np.linspace(0, 2 * np.pi, 73)
        for k, h in enumerate(res["holes"][:len(self.hole_rings)]):
            R = h["diameter_m"] / 2
            ring = h["center"] + R * (np.outer(np.cos(t), e1) + np.outer(np.sin(t), e2)) + n * 0.001
            self.hole_rings[k].set(ring)
            self.hole_rings[k].visible(True)
            self.hole_labels[k].set(h["center"] + n * 0.015 + e1 * (R + 0.004),
                                    f"Ø {h['diameter_m'] * 1000:.1f} mm" + ("  ← tornillo" if k == 0 else ""))
            self.hole_labels[k].visible(True)
        best = res["holes"][0]
        self.hole_dot.set(best["center"])
        self.hole_dot.visible(True)
        c = best["center"]
        self.edge_note.setText(f"{len(res['holes'])} hoyos. El más chico (Ø {best['diameter_m'] * 1000:.1f} mm, "
                               f"borde ± {best['rms_m'] * 1000:.1f} mm) en x {c[0]:.4f}, y {c[1]:.4f}, "
                               f"z {c[2]:.4f} m: ahí va el tornillo. Placa inclinada {res['tilt_deg']:.1f}°.")
        self.edge_note.setStyleSheet(f"color: {theme.TEXT};")
        self.edge_note.setVisible(True)

    def _show(self, name, auto_edge=True):
        self.current = name
        hh = name.split("_")[-1]
        self.res_meta.setText(f"foto de las {hh[:2]}:{hh[2:4]}:{hh[4:6]}" if len(hh) == 6 else name)
        pcd = config.capture_path(name)
        tool = parsers.read_tool_pose(config.capture_path(name, "_pose.txt"))
        try:
            pts = parsers.read_pcd(pcd)
        except (OSError, ValueError, IndexError) as e:
            self.hist_note.setText(f"No se pudo leer {pcd.name}: {e}")
            return
        # histograma de distancia a la camara (eje z de la camara)
        if len(pts):
            d = pts[:, 2]
            lo = 0.15
            hi = max(0.40, (self.max_depth or 0.34) + 0.03)
            y, x = np.histogram(d, bins=np.arange(lo, hi, 0.0025))
            self.hist_curve.setData(x, y)
            near = int(np.sum(d < np.median(d) - 0.06))
            note = f"{len(pts):,} puntos".replace(",", " ")
            if self.max_depth:
                note += f", MAX_DEPTH {self.max_depth:.2f} m (línea punteada)"
            if near > 30:
                note += f". {near} puntos más de 6 cm arriba de la superficie: ¿ruido u objeto?"
            self.hist_note.setText(note)
        if tool is None or not len(pts):
            return
        P = kin.cloud_to_base(pts, tool)
        if len(P) > 60000:
            P = P[np.random.default_rng(0).choice(len(P), 60000, replace=False)]
        self.cloud.set(P)
        self.cloud.visible(True)
        T_tool = kin.tool_pose_matrix(tool)
        cam = T_tool[:3, 3] - T_tool[:3, 2] * config.TCP_OFFSET_M + T_tool[:3, :3] @ np.array(config.DEPTH_TO_INT_M)
        Tc = T_tool.copy()
        Tc[:3, 3] = cam
        self.cam_frame.set(Tc)
        self.cam_dot.set(cam)
        self.cam_label.set(cam + [0, 0, 0.025])
        for a in (self.cam_frame, self.cam_dot, self.cam_label):
            a.visible(True)

        cap = self.results.get(name)
        g = cap["grasps"][0] if cap and cap.get("grasps") else None
        if auto_edge:
            self.edge_cb.blockSignals(True)
            self.edge_cb.setChecked(self._plate_capture(tool))
            self.edge_cb.blockSignals(False)
        ok_grasp = g is not None and g.get("position") is not None and g.get("rpy") is not None
        self._show_edge(pts, tool, g if ok_grasp else None)
        self._show_holes(name, pts, tool)
        if g is not None and g.get("position") is not None and g.get("rpy") is not None:
            R = kin.rpy_to_matrix(g["rpy"])
            T_ee = kin.pose_matrix(g["position"], R)
            T_int = T_ee.copy()
            T_int[:3, 3] -= R[:, 2] * config.TCP_OFFSET_M
            self.grasp_g.set_pose(T_int)
            self.grasp_g.set_opening(g.get("opening_m", 0.04) + 0.006)
            self.grasp_g.visible(True)
            if g.get("pregrasp") is not None:
                T_pre = T_int.copy()
                T_pre[:3, 3] += g["pregrasp"] - g["position"]
                self.pre_g.set_pose(T_pre)
                self.pre_g.set_opening(config.GRIPPER_MAX_OPENING_M)
                self.pre_g.visible(True)
                self.approach.set([g["pregrasp"], g["position"]])
                self.approach.visible(True)
            ok, why, data = verdict(g, tool, self.info)
            # foto de la placa: mismas reglas extra que el controlador
            em = getattr(self, "_edge_m", None)
            if self.edge_cb.isChecked() and em is not None:
                max_skew = (self.info or {}).get("params", {}).get("plate_max_skew_deg", 20.0)
                if not em.get("ok"):
                    why.append("sin canto de la placa: " + em.get("why", ""))
                elif em["angle_deg"] > max_skew:
                    why.append(f"la pinza cierra a {em['angle_deg']:.0f}° de perpendicular a la placa "
                               f"(máx {max_skew:.0f}°)")
                ok = not why
            self.verdict_chip.set("Aceptado" if ok else "Rechazado", theme.POSITIVE if ok else theme.NEGATIVE)
            self.why.setText("  ·  ".join(why))
            self.why.setVisible(bool(why))
            sc = g.get("score", float("nan"))
            self.vals["score"].setText("—" if not np.isfinite(sc) else f"{sc:.0f}")
            # el score de GPD no es una regla del controlador: va sin color
            self.vals["z"].setText(f"{g['position'][2]:.3f}")
            self.vals["tilt"].setText(f"{data.get('tilt', float('nan')):.1f}°")
            self.vals["open"].setText(f"{g.get('opening_m', float('nan')) * 1000:.0f} mm")
            self.vals["pre"].setText(f"{g['pregrasp'][2]:.3f}" if g.get("pregrasp") is not None else "—")
            self.vals["yaw"].setText(f"{g['rpy'][2]:.0f}°")
            self.calc_btn.setVisible(False)
        else:
            for a in (self.grasp_g, self.pre_g, self.approach):
                a.visible(False)
            failed = cap is not None and cap.get("ok") is False
            hole_photo = name.startswith("hoyo_")
            self.verdict_chip.set("Falla de visión" if failed else "Foto del hoyo (sin GPD)" if hole_photo
                                  else "Sin agarre", theme.NEGATIVE if failed else theme.NEUTRAL)
            self.why.setText(cap.get("answer", "") if failed else "Esta captura no tiene agarre guardado.")
            self.why.setVisible(failed or not hole_photo)
            for v in self.vals.values():
                v.setText("—")
            self.calc_btn.setVisible(True)
        # camara hacia la nube
        # encuadre a la nube (y a la camara), no una posicion fija
        c = 0.5 * (P.min(axis=0) + P.max(axis=0))
        self.scene.fit(np.vstack([P[::20], [cam]]), direction=(1.0, -1.0, 1.1), margin=0.9, min_ext=0.08,
                       center=c + [0, 0, 0.04])
        self.view3d.request_render()

    def _on_photo(self):
        name = self.pose_combo.currentText()
        for p in (self.info or {}).get("poses", []):
            if p["name"] == name:
                self.request_photo.emit(list(p["tcp_m"]) + list(p["rpy_deg"]))
                self.photo_btn.setEnabled(False)
                QtCore.QTimer.singleShot(2000, lambda: self.photo_btn.setEnabled(self.photo_allowed))
                return

    def _on_calc(self):
        if self.current:
            self.calc_btn.setEnabled(False)
            self.calc_btn.setText("Calculando con GPD…")
            self.compute_grasp.emit(self.current)

    def calc_done(self):
        self.calc_btn.setEnabled(True)
        self.calc_btn.setText("Calcular agarre con GPD (sin robot)")
