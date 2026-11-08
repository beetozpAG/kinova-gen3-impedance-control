"""Vista de presentacion: para proyectar. El robot 3D grande, el paso actual
en palabras que entiende el publico y dos cifras que prueban que funciona.

Recibe los mismos estados que 'En vivo' (en vivo o repeticion). El boton
NO tiene logica propia: refleja el de 'En vivo' y al apretarlo llama al
mismo metodo (mismo NEXT, misma confirmacion). Esc o F5 para salir.
"""

import collections
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets

from . import hole_view, kinematics as kin, modules, runs, scene3d, theme, widgets as W


def plain_step(step):
    """Lo que hace el robot en el paso, dicho para el publico."""
    if not step:
        return ""
    t = step["type"]
    if t == "move" and step.get("vision_target") == "hole":
        dz = step.get("vision_dz_m") or 0.0
        if dz:
            return f"Se coloca {dz * 100:.0f} cm arriba del hoyo que eligió la cámara."
        if step["pose"].startswith("SOLTAR"):
            return "Baja en línea recta hasta el hoyo para soltar el tornillo."
        return "Se coloca justo encima del hoyo que eligió la cámara."
    if t == "move" and step.get("vision_target") == "plate_fixture":
        return (f"Lleva la placa a {step['pose']}, corregido por el punto donde la agarró: "
                "la placa queda en el mismo lugar del fixture.")
    if t == "move" and step.get("from_vision"):
        dz = step.get("vision_dz_m") or 0.0
        if dz:
            return f"Sube {abs(dz) * 100:.0f} cm en línea recta desde el agarre." if dz > 0 else \
                f"Baja {abs(dz) * 100:.0f} cm desde el agarre."
        if step.get("vision_target") == "pregrasp":
            return "Se acerca por encima del punto que encontró la cámara."
        return "Baja al agarre que encontró la cámara."
    if t == "move":
        return f"El brazo se mueve a la pose {step['pose']}."
    if t == "gripper":
        pct = step.get("gripper_pct", 0)
        return "Abre la pinza." if pct <= 10 else f"Cierra la pinza al {pct:.0f} %."
    if t == "fixture":
        return f"El fixture gira a {step.get('fixture_deg', 0):.0f}°."
    if t == "wait":
        return f"Espera {step.get('wait_s', 0):.1f} s a que el agarre se asiente."
    if t == "vision" and step.get("vision_approach"):
        obj = "la placa" if step.get("vision_object") == "PLATE" else "el perno"
        return (f"La cámara busca {obj} por su color y el brazo se coloca 30 cm arriba, "
                "hasta que se ve completo.")
    if t == "vision" and step.get("vision_object"):
        obj = "la placa" if step.get("vision_object") == "PLATE" else "el perno"
        return f"La cámara toma una foto y GPD busca dónde agarrar {obj}."
    if t == "vision" and step.get("vision_needs_hole"):
        return ("La cámara de profundidad mide la placa, encuentra sus hoyos y elige el más chico "
                "para meter el tornillo.")
    if t == "vision":
        return "La cámara toma una foto y GPD busca dónde agarrar."
    return ""


STATUS_PLAIN = {
    "waiting": "Listo. Esperando el botón del operador.",
    "running": "En movimiento.",
    "paused": "En pausa.",
    "done": "Secuencia terminada.",
    "aborted": "Secuencia abortada.",
}


class PresentPage(QtWidgets.QWidget):
    exit_requested = QtCore.pyqtSignal()
    next_clicked = QtCore.pyqtSignal()
    pause_clicked = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("present")
        self.info = None
        self.steps_by_n = {}
        self.mode = "live"

        # ---------- escena ----------
        self.scene = scene3d.SceneContent()
        r = self.scene.renderer
        self.robot = scene3d.Robot(r)
        self.robot.visible(False)
        self.target_dot = scene3d.Sphere(r, theme.CHART3, 0.014, 0.95)
        self.ramp_dot = scene3d.Cube(r, theme.CHART2, 0.018, 0.95)
        self.tcp_dot = scene3d.Sphere(r, theme.ACCENT, 0.011)
        self.trail = scene3d.Tube(r, theme.ACCENT, 0.005, 0.9)
        for a in (self.target_dot, self.ramp_dot, self.tcp_dot, self.trail):
            a.visible(False)
        self.trail_pts = collections.deque(maxlen=900)
        self.new_test = runs.NewTestWatch()   # borra la estela en cada prueba nueva
        self.pose_items = []
        self.box = None
        self.low_box = None
        self.view3d = scene3d.Scene3D(self.scene)
        self.q_from = self.q_to = None
        self.q_t0, self.q_dur, self.grip = 0.0, 0.2, None
        self._anim = QtCore.QTimer(self)
        self._anim.timeout.connect(self._animate)
        self._anim.start(33)

        # ---------- columna de texto ----------
        col = QtWidgets.QVBoxLayout()
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(0)
        self._styled = {}   # etiqueta -> [tamano base px, color, peso]: se escalan con la altura
        self.product = self._sl("MODULAR · Kinova Gen3", theme.PS_LABEL, theme.MUTED)
        col.addWidget(self.product)
        self.source = self._sl("En vivo", theme.PS_LABEL, theme.MUTED, wrap=True)
        col.addWidget(self.source)
        col.addSpacing(40)
        self.count = self._sl("", theme.PS_BODY, theme.MUTED)
        col.addWidget(self.count)
        col.addSpacing(6)
        self.title = self._sl("Esperando datos", theme.PS_STEP, weight=600, wrap=True)
        col.addWidget(self.title)
        col.addSpacing(10)
        # modulo(s) del modular que usa el paso: encabezado chico y nombres grandes
        self.modules_head = self._sl("", theme.PS_LABEL, theme.MUTED, weight=600)
        self.modules_head.setVisible(False)
        col.addWidget(self.modules_head)
        col.addSpacing(4)
        self.modules = self._sl("", theme.PS_MODULE, weight=700, wrap=True)
        self.modules.setVisible(False)
        col.addWidget(self.modules)
        col.addSpacing(12)
        self.plain = self._sl("", theme.PS_BODY, wrap=True)
        col.addWidget(self.plain)
        col.addSpacing(8)
        self.status = self._sl("", theme.PS_BODY, theme.MUTED, wrap=True)
        col.addWidget(self.status)
        col.addSpacing(20)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setTextVisible(False)
        self.progress.setStyleSheet("QProgressBar { max-height: 8px; min-height: 8px; border-radius: 4px; }"
                                    "QProgressBar::chunk { border-radius: 4px; }")
        col.addWidget(self.progress)
        col.addSpacing(28)

        figs = QtWidgets.QHBoxLayout()
        figs.setSpacing(28)
        self.f1 = self._figure("Error de posición")
        self.f2 = self._figure("Pinza")
        figs.addWidget(self.f1["w"], 1, QtCore.Qt.AlignTop)
        figs.addWidget(W.vline())
        figs.addWidget(self.f2["w"], 1, QtCore.Qt.AlignTop)
        col.addLayout(figs)
        col.addSpacing(20)
        self.vision = self._sl("", theme.PS_LABEL, wrap=True)
        self.vision.setVisible(False)
        col.addWidget(self.vision)
        col.addStretch(1)

        self.note = self._sl("", theme.PS_LABEL, theme.WARNING, weight=500, wrap=True)
        self.note.setVisible(False)
        col.addWidget(self.note)
        col.addSpacing(12)
        self.next_btn = QtWidgets.QPushButton("Siguiente paso")
        self.next_btn.setObjectName("next")
        self.next_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.next_btn.clicked.connect(self.next_clicked.emit)
        self.pause_btn = QtWidgets.QPushButton("Pausa")
        self.pause_btn.setObjectName("pause")
        self.pause_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.pause_btn.clicked.connect(self.pause_clicked.emit)
        brow = QtWidgets.QHBoxLayout()
        brow.setSpacing(12)
        brow.addWidget(self.next_btn, 1)
        brow.addWidget(self.pause_btn)
        col.addLayout(brow)
        col.addSpacing(8)
        col.addWidget(self._sl("Esc para volver a la consola", theme.FS_BODY, theme.MUTED))

        legend = self._sl("", theme.PS_LABEL, theme.MUTED, wrap=True)
        legend.setText(f"{W.swatch(theme.ACCENT, 'dot1')} punta de la pinza y su recorrido &nbsp;&nbsp; "
                       f"{W.swatch(theme.CHART2, 'box')} referencia &nbsp;&nbsp; "
                       f"{W.swatch(theme.CHART3, 'dot1')} a dónde va")
        # deteccion del hoyo junto al robot (de la foto hasta soltar el tornillo)
        self.hole_view = hole_view.HoleView(big=True)
        self.hole_view.setVisible(False)
        self.hole_phase = hole_view.HolePhase(self.hole_view)
        views = QtWidgets.QHBoxLayout()
        views.setSpacing(24)
        views.addWidget(self.view3d, 3)
        views.addWidget(self.hole_view, 2)
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(12)
        left.addLayout(views, 1)
        left.addWidget(legend)

        self.lay = QtWidgets.QHBoxLayout(self)
        self.lay.setContentsMargins(40, 40, 48, 40)
        self.lay.setSpacing(48)
        self.lay.addLayout(left, 60)
        cw = QtWidgets.QWidget()
        cw.setLayout(col)
        self.lay.addWidget(cw, 40)
        self._scale = None
        self._apply_scale(1.0)

    # ------------------------------------------------------------ escala
    def _sl(self, text, base, color=None, weight=400, wrap=False):
        lbl = QtWidgets.QLabel(text)
        lbl.setWordWrap(wrap)
        self._styled[lbl] = [base, color or theme.TEXT, weight]
        return lbl

    def _restyle(self, lbl, color=None):
        if color is not None:
            self._styled[lbl][1] = color
        base, col, weight = self._styled[lbl]
        lbl.setStyleSheet(f"font-size: {max(11, round(base * (self._scale or 1.0)))}px; "
                          f"color: {col}; font-weight: {weight};")

    def _apply_scale(self, k):
        """Todo el texto escala con la altura de la pantalla: 1080 px = 1.0
        (un proyector de 768 px queda en ~0.71, sin que nada se corte)."""
        if self._scale is not None and abs(k - self._scale) < 0.02:
            return
        self._scale = k
        for lbl in self._styled:
            self._restyle(lbl)
        for b in (self.next_btn, self.pause_btn):
            b.setStyleSheet(f"font-size: {round(theme.PS_BODY * k)}px;")
            b.setMinimumHeight(round(72 * k))
        m = round(40 * k)
        self.lay.setContentsMargins(m, m, round(48 * k), m)
        self.lay.setSpacing(round(48 * k))

    def resizeEvent(self, ev):
        self._apply_scale(max(0.55, min(1.3, self.height() / 1080.0)))
        super().resizeEvent(ev)

    def _figure(self, title):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(4)
        t = self._sl(title, theme.PS_LABEL, theme.MUTED)
        val = self._sl("—", theme.PS_FIGURE)
        sub = self._sl("", theme.PS_LABEL, theme.MUTED, wrap=True)
        for x in (t, val, sub):
            v.addWidget(x)
        return {"w": w, "title": t, "val": val, "sub": sub}

    # ------------------------------------------------------------ datos
    def set_task_info(self, info):
        self.info = info
        self.steps_by_n = {s["n"]: s for s in info.get("steps", [])}
        self.hole_phase.set_steps(info.get("steps", []))
        r = self.scene.renderer
        for a in self.pose_items:
            a.visible(False)
        self.pose_items = []
        used = {s["pose"] for s in info.get("steps", []) if s["type"] in ("move", "vision")}
        for p in info.get("poses", []):
            if p["name"] in used:
                m = scene3d.Sphere(r, theme.NEUTRAL, 0.008, 0.8)
                m.set(p["tcp_m"])
                lb = scene3d.Label(r, p["name"], theme.MUTED, 15)
                lb.set(np.array(p["tcp_m"]) + [0, 0, 0.035])
                self.pose_items += [m, lb]
        if self.box is None and "workspace_min_m" in info:
            self.box = self.scene.workspace_box(info["workspace_min_m"], info["workspace_max_m"])
        if self.low_box is None and "low_zone_min_m" in info:
            self.low_box = self.scene.workspace_box(info["low_zone_min_m"], info["low_zone_max_m"])
        self.scene.frame_robot(info.get("workspace_min_m"), info.get("workspace_max_m"))
        self.view3d.request_render()

    def set_mode(self, mode, name=""):
        self.mode = mode
        self.source.setText(f"Repetición de la corrida del {name}" if mode == "replay" else "En vivo")
        self.trail_pts.clear()

    def mirror_button(self, text, enabled, note="", pause_enabled=False):
        """Refleja los botones y el aviso de 'En vivo' (una sola logica)."""
        self.next_btn.setText(text)
        self.next_btn.setEnabled(enabled)
        self.pause_btn.setEnabled(pause_enabled)
        self.note.setText(note)
        self.note.setVisible(bool(note))

    def update_state(self, st):
        n = st.get("step_n")
        status = st.get("task_status")
        if self.new_test.update(n, status, (self.info or {}).get("first_step", 1)):
            self.trail_pts.clear()
            self.trail.visible(False)
        total = st.get("step_total") or (len(self.steps_by_n) or None)
        step = self.steps_by_n.get(n) if n else None
        replay = self.mode == "replay"
        if replay:
            step = None   # el CSV no guarda los nombres; no se inventan
        if n:
            shown = total if total and n <= total and not replay else None
            # en repeticion el titulo ya es "Paso n": no se repite arriba
            self.count.setText("" if replay else f"Paso {n}" + (f" de {shown}" if shown else ""))
            self.count.setVisible(not replay)
            first = (self.info or {}).get("first_step", 1)
            self.progress.setVisible(bool(shown))
            if shown:
                self.progress.setValue(100 if status == "done" else int(100 * max(0, n - first) / max(1, shown - first + 1)))
        self.title.setText(step["label"] if step else (f"Paso {n}" if n else "Esperando datos"))
        self.plain.setText(plain_step(step))
        self.plain.setVisible(bool(step))
        mods = modules.step_modules(step) if step else []
        self.modules_head.setText("MÓDULO" if len(mods) == 1 else "MÓDULOS")
        self.modules.setText("<br>".join(
            f"<span style='color:{modules.MODULES[k][1]}; font-size:130%'>●</span> {modules.MODULES[k][0]}"
            for k, _ in mods))
        self.modules_head.setVisible(bool(mods))
        self.modules.setVisible(bool(mods))
        self.hole_view.setVisible(self.hole_phase.update(n, status, st, live=not replay))
        self.status.setText(STATUS_PLAIN.get(status, ""))

        err = st.get("err_pos_cm")
        if err is None and "pose" in st and "target" in st:
            err = float(np.linalg.norm(st["pose"][:3] - st["target"][:3]) * 100)
        if err is not None:
            self.f1["val"].setText(f"{err:.2f} cm")
            self._restyle(self.f1["val"], theme.TEXT if err <= 1.5 else theme.WARNING)
            self.f1["sub"].setText("tolerancia 1.5 cm")
        if st.get("gripper_pct") is not None:
            self.f2["val"].setText(f"{st['gripper_pct']:.0f} %")
            self.f2["sub"].setText("100 % = cerrada")

        vt = st.get("vision_tcp")
        if not replay and step and step.get("from_vision") and vt is not None:
            self.vision.setText(f"La cámara encontró el agarre a {vt[2] * 100:.1f} cm de altura y el "
                                "controlador lo aceptó.")
            self.vision.setVisible(True)
        else:
            self.vision.setVisible(False)

        # 3D
        if st.get("joints_deg") is not None:
            q = np.asarray(st["joints_deg"], dtype=float)
            if self.q_to is None:
                self.q_from = self.q_to = q
            else:
                self.q_from = self._q_now()
                self.q_to = self.q_from + (q - self.q_from + 180.0) % 360.0 - 180.0
            self.q_t0 = time.monotonic()
            self.q_dur = 0.04 if replay else 0.2
            self.grip = st.get("gripper_pct", self.grip)
            self.robot.visible(True)
        if "pose" in st:
            R = kin.quat_to_matrix(*st["pose"][3:7])
            tcp = st["pose"][:3] + R[:, 2] * kin.TCP_OFFSET_M
            self.tcp_dot.set(tcp)
            self.tcp_dot.visible(True)
            self.trail_pts.append(tcp)
            if len(self.trail_pts) > 1:
                self.trail.set(np.array(self.trail_pts))
                self.trail.visible(True)
            if "ramp" in st:
                self.ramp_dot.set(np.asarray(st["ramp"]) + R[:, 2] * kin.TCP_OFFSET_M)
                self.ramp_dot.visible(True)
        if "target" in st:
            tg = st["target"]
            Rt = kin.quat_to_matrix(*tg[3:7])
            self.target_dot.set(tg[:3] + Rt[:, 2] * kin.TCP_OFFSET_M)
            self.target_dot.visible(True)
        self.view3d.request_render()

    def _q_now(self):
        if self.q_to is None:
            return None
        a = min(1.0, (time.monotonic() - self.q_t0) / max(self.q_dur, 1e-3))
        a = a * a * (3 - 2 * a)
        return self.q_from + (self.q_to - self.q_from) * a

    def _animate(self):
        q = self._q_now()
        if q is None or not self.isVisible():
            return
        self.robot.set_joints(q, self.grip)
        self.view3d.request_render()

    def keyPressEvent(self, ev):
        if ev.key() == QtCore.Qt.Key_Escape:
            self.exit_requested.emit()
            return
        super().keyPressEvent(ev)
