"""Pagina 'En vivo': robot 3D, paso actual, 'Siguiente paso', medidores,
secuencia y graficas. Recibe estados del tablero del controlador (en vivo) o
de una corrida guardada (repeticion)."""

import collections
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets

from . import hole_view, kinematics as kin, modules, runs, scene3d, theme, widgets as W

STATUS_TEXT = {
    "waiting": ("Esperando el botón", theme.WARNING),
    "running": ("Ejecutando", theme.ACCENT),
    "paused": ("En pausa", theme.WARNING),
    "done": ("Secuencia terminada", theme.POSITIVE),
    "aborted": ("Abortada", theme.NEGATIVE),
}
TYPE_TEXT = W.TYPE_TEXT


class LivePage(QtWidgets.QWidget):
    next_clicked = QtCore.pyqtSignal()
    pause_clicked = QtCore.pyqtSignal()
    mode_clicked = QtCore.pyqtSignal(bool)   # True = AUTOMATICO
    replay_play = QtCore.pyqtSignal()
    replay_pause = QtCore.pyqtSignal()
    replay_seek = QtCore.pyqtSignal(float)
    replay_speed = QtCore.pyqtSignal(float)
    replay_exit = QtCore.pyqtSignal()
    open_analysis = QtCore.pyqtSignal()
    present_clicked = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.info = None
        self.steps_by_n = {}
        self.live_available = False
        self.mode = "live"
        self.last_state = None
        self._next_block_until = 0.0
        self._sent = None            # (paso, estado) cuando se mando NEXT
        self.before_start = None     # funcion -> bool, antes del primer paso (app)
        self._run_t0 = None          # t del primer estado (para el resumen final)
        self._run_n0 = None

        # ---------- escena 3D ----------
        self.scene = scene3d.SceneContent()
        r = self.scene.renderer
        self.robot = scene3d.Robot(r)
        self.robot.visible(False)
        self.target_frame = scene3d.Frame(r, length=0.07, radius=0.005)
        self.target_frame.visible(False)
        self.target_dot = scene3d.Sphere(r, theme.CHART3, 0.012, 0.95)
        self.target_dot.visible(False)
        self.ramp_dot = scene3d.Cube(r, theme.CHART2, 0.016, 0.95)
        self.ramp_dot.visible(False)
        self.tcp_dot = scene3d.Sphere(r, theme.ACCENT, 0.009)
        self.tcp_dot.visible(False)
        self.trail = scene3d.Tube(r, theme.ACCENT, 0.0035, 0.85)
        self.trail_pts = collections.deque(maxlen=900)
        self.new_test = runs.NewTestWatch()   # borra la estela en cada prueba nueva
        self.pose_markers = []
        self.pose_labels = []
        self.box_actor = None
        self.low_box_actor = None   # zona baja (mesa de las placas), si hay
        self.view3d = scene3d.Scene3D(self.scene)

        # joints animados (el tablero llega a 5 Hz; se interpola a 30 Hz)
        self.q_from = None
        self.q_to = None
        self.q_t0 = 0.0
        self.q_dur = 0.2
        self.grip_pct = None
        self._anim = QtCore.QTimer(self)
        self._anim.timeout.connect(self._animate)
        self._anim.start(33)

        # ---------- barra de fuente (en vivo / repeticion) ----------
        self.source_bar = self._build_source_bar()

        # ---------- paso actual ----------
        step_card = W.Card("Paso actual")
        self.step_count = W.label("", obj="cardMeta")
        step_card.header_right(self.step_count)
        self.step_label = W.label("Esperando datos del controlador", size=20, weight=600, wrap=True)
        step_card.add(self.step_label)
        self.step_desc = W.label("", color=theme.MUTED, wrap=True)
        step_card.add(self.step_desc)
        # modulo(s) del modular que usa el paso (modules.py)
        self.module_lbl = W.label("", wrap=True)
        self.module_lbl.setVisible(False)
        step_card.add(self.module_lbl)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(24)
        self.status_chip = W.Chip("Estado")
        self.time_chip = W.Chip("Tiempo en el paso")
        row.addWidget(self.status_chip, 1)
        row.addWidget(self.time_chip, 1)
        step_card.add(row)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        step_card.add(self.progress)
        self.message = W.label("", color=theme.MUTED, wrap=True)
        step_card.add(self.message)
        # agarre de la vision junto al boton que lo va a usar
        self.vision_row = W.label("", wrap=True)
        self.vision_row.setVisible(False)
        step_card.add(self.vision_row)
        # aviso si el controlador no respondio al boton
        self.ack_note = W.label("", color=theme.WARNING, wrap=True, weight=500)
        self.ack_note.setVisible(False)
        step_card.add(self.ack_note)
        # resumen al terminar
        self.recap = QtWidgets.QWidget()
        rl = QtWidgets.QHBoxLayout(self.recap)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(12)
        self.recap_text = W.label("", wrap=True)
        rl.addWidget(self.recap_text, 1)
        self.recap_btn = QtWidgets.QPushButton("Ver en Análisis")
        self.recap_btn.clicked.connect(self.open_analysis.emit)
        rl.addWidget(self.recap_btn, 0, QtCore.Qt.AlignTop)
        self.recap.setVisible(False)
        step_card.add(self.recap)
        # modo de la secuencia: AUTOMATICO (por defecto en el controlador) o PASO A PASO
        mode_row = QtWidgets.QHBoxLayout()
        mode_row.setSpacing(0)
        mode_row.addWidget(W.label("Modo", obj="cardMeta"))
        mode_row.addSpacing(10)
        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_auto = QtWidgets.QPushButton("Automático")
        self.mode_step = QtWidgets.QPushButton("Paso a paso")
        for b, auto in ((self.mode_auto, True), (self.mode_step, False)):
            b.setObjectName("seg")
            b.setCheckable(True)
            b.setCursor(QtCore.Qt.PointingHandCursor)
            b.setAccessibleName(f"Modo {b.text()}")
            b.clicked.connect(lambda _c, a=auto: self._on_mode(a))
            self.mode_group.addButton(b)
            mode_row.addWidget(b)
        self.mode_auto.setChecked(True)
        mode_row.addSpacing(12)
        self.mode_note = W.label("", obj="cardMeta", wrap=True)
        mode_row.addWidget(self.mode_note, 1)
        step_card.add(mode_row)
        self._mode_sent = None       # modo pedido, hasta que el tablero lo confirme
        self.next_btn = QtWidgets.QPushButton("Siguiente paso")
        self.next_btn.setObjectName("next")
        self.next_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.next_btn.clicked.connect(self._on_next)
        # Pausa: el brazo se queda donde esta; 'Siguiente paso' reanuda el paso
        self.pause_btn = QtWidgets.QPushButton("Pausa")
        self.pause_btn.setObjectName("pause")
        self.pause_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.pause_btn.setAccessibleName("Pausar la secuencia")
        self.pause_btn.clicked.connect(self._on_pause)
        brow = QtWidgets.QHBoxLayout()
        brow.setSpacing(10)
        brow.addWidget(self.next_btn, 1)
        brow.addWidget(self.pause_btn)
        step_card.add(brow)
        self.next_hint = W.label("", obj="cardMeta", wrap=True)
        step_card.add(self.next_hint)

        # ---------- cifras ----------
        gauges = W.Card("Estado del robot")
        self.g_pos = W.Gauge("Error pos.", "cm", 5.0, decimals=2, tol=1.5)
        self.g_ori = W.Gauge("Error ori.", "°", 15.0, decimals=1, tol=5.0)
        self.g_grip = W.Gauge("Pinza", "%", 100.0, decimals=0)
        self.g_fix = W.Gauge("Fixture", "°", 120.0, decimals=1)
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(12)
        for k, g_ in enumerate((self.g_pos, self.g_ori, self.g_grip, self.g_fix)):
            if k:
                grid.addWidget(W.vline(), 0, 2 * k - 1)
            grid.addWidget(g_, 0, 2 * k)
            grid.setColumnStretch(2 * k, 1)
        gauges.add(grid)

        # ---------- secuencia ----------
        seq_card = W.Card("Secuencia")
        self.seq_meta = W.label("", obj="cardMeta")
        seq_card.header_right(self.seq_meta)
        self.step_list = W.StepList()
        self.seq_scroll = QtWidgets.QScrollArea()
        self.seq_scroll.setWidgetResizable(True)
        self.seq_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.seq_scroll.setWidget(self.step_list)
        seq_card.add(self.seq_scroll, 1)
        seq_card.add(W.label(" &nbsp;&nbsp; ".join(f"{W.swatch(c, 'dot1')} {n.replace('Sistemas ', '')}"
                                                   for n, c in (modules.MODULES[k] for k in modules.ORDER))
                             + " &nbsp;— módulo del modular", obj="cardMeta", wrap=True))

        # ---------- graficas ----------
        plots = W.Card("Señales", meta="últimos 60 s")
        self.joint_pick = QtWidgets.QComboBox()
        for k in range(7):
            self.joint_pick.addItem(f"Resaltar J{k + 1}", k)
        self.joint_pick.setCurrentIndex(1)
        self.joint_pick.currentIndexChanged.connect(self._style_tau)
        plots.header_right(self.joint_pick)
        pl = QtWidgets.QHBoxLayout()
        pl.setSpacing(24)
        self.p_err = W.plot("Error de posición [cm]")
        self.c_err = self.p_err.plot(pen=W.pen(theme.ACCENT, 2.0))
        self.p_err.addLine(y=1.5, pen=W.pen(theme.MUTED, 1, "dash"),
                           label="tolerancia", labelOpts={"position": 0.02, "color": theme.MUTED,
                                                           "anchors": [(0, 1), (0, 1)]})
        self.p_tau = W.plot("Torque enviado [Nm]")
        self.c_tau = [self.p_tau.plot(pen=W.pen(theme.CONTROL, 1.4)) for _ in range(7)]
        self._style_tau()
        pl.addWidget(self.p_err)
        pl.addWidget(self.p_tau)
        plots.add(pl)
        plots.setMinimumHeight(230)
        self.hist_t = collections.deque(maxlen=1500)
        self.hist_err = collections.deque(maxlen=1500)
        self.hist_tau = collections.deque(maxlen=1500)

        # ---------- vista 3D ----------
        view_card = W.Card("Robot")
        legend = W.label("", obj="cardMeta")
        legend.setText(f"{W.swatch(theme.ACCENT, 'dot1')} TCP y estela &nbsp;&nbsp; "
                       f"{W.swatch(theme.CHART2, 'box')} referencia (rampa) &nbsp;&nbsp; "
                       f"{W.swatch(theme.CHART3, 'dot1')} objetivo")
        view_card.header_right(legend)
        view_card.add(self.view3d, 1)
        opts = QtWidgets.QHBoxLayout()
        opts.setSpacing(18)
        self.cb_poses = QtWidgets.QCheckBox("Poses")
        self.cb_poses.setChecked(True)
        self.cb_labels = QtWidgets.QCheckBox("Nombres")
        self.cb_trail = QtWidgets.QCheckBox("Estela")
        self.cb_trail.setChecked(True)
        self.cb_box = QtWidgets.QCheckBox("Caja de trabajo")
        self.cb_box.setChecked(True)
        for cb in (self.cb_poses, self.cb_labels, self.cb_trail, self.cb_box):
            cb.toggled.connect(self._apply_view_options)
            opts.addWidget(cb)
        opts.addStretch(1)
        reset = QtWidgets.QPushButton("Centrar vista")
        reset.clicked.connect(self._reset_camera)
        opts.addWidget(reset)
        view_card.add(opts)

        # ---------- deteccion del hoyo (de la foto hasta soltar el tornillo) ----------
        self.hole_card = W.Card("Detección del hoyo")
        self.hole_card.header_right(W.label(f"{W.swatch(theme.ACCENT, 'dot1')} elegido &nbsp;&nbsp; "
                                            f"{W.swatch(hole_view.OTHER, 'dot1')} otros", obj="cardMeta"))
        self.hole_view = hole_view.HoleView()
        self.hole_card.add(self.hole_view, 1)
        self.hole_card.setVisible(False)
        self.hole_phase = hole_view.HolePhase(self.hole_view)

        # ---------- layout ----------
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(24, 20, 24, 24)
        outer.setSpacing(16)
        head = QtWidgets.QHBoxLayout()
        head.setSpacing(16)
        tcol = QtWidgets.QVBoxLayout()
        tcol.setSpacing(2)
        tcol.addWidget(W.label("En vivo", obj="pageTitle"))
        self.page_sub = W.label("Datos del tablero del controlador", obj="pageSub")
        tcol.addWidget(self.page_sub)
        head.addLayout(tcol)
        head.addStretch(1)
        head.addWidget(self.source_bar)
        self.present_btn = QtWidgets.QPushButton("Vista de presentación")
        self.present_btn.setToolTip("Pantalla para proyectar (F5). Esc para volver.")
        self.present_btn.clicked.connect(self.present_clicked.emit)
        head.addWidget(self.present_btn)
        outer.addLayout(head)

        left = QtWidgets.QVBoxLayout()
        left.setSpacing(16)
        view_row = QtWidgets.QHBoxLayout()
        view_row.setSpacing(16)
        view_row.addWidget(view_card, 3)
        view_row.addWidget(self.hole_card, 2)
        left.addLayout(view_row, 3)
        left.addWidget(plots, 1)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(16)
        right.setContentsMargins(0, 0, 0, 0)
        right.addWidget(step_card)
        right.addWidget(gauges)
        right.addWidget(seq_card, 1)
        lay = QtWidgets.QHBoxLayout()
        lay.setSpacing(16)
        lay.addLayout(left, 5)
        rw = QtWidgets.QWidget()
        rw.setLayout(right)
        rw.setMinimumWidth(440)
        rw.setMaximumWidth(500)
        lay.addWidget(rw, 2)
        outer.addLayout(lay, 1)
        self._update_next_button(None)

    def _style_tau(self):
        """Las 7 articulaciones en gris claro; la elegida en el acento."""
        k = self.joint_pick.currentData()
        for j, c in enumerate(self.c_tau):
            c.setPen(W.pen(theme.ACCENT if j == k else theme.CONTROL, 2.4 if j == k else 1.4))
            c.setZValue(10 if j == k else 0)

    # ------------------------------------------------------------ fuente
    def _build_source_bar(self):
        bar = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(10)
        self.rp_play = QtWidgets.QPushButton("Reproducir")
        self.rp_play.setMinimumWidth(124)
        self.rp_play.clicked.connect(self._toggle_play)
        self.rp_speed = QtWidgets.QComboBox()
        for s in ("x0.5", "x1", "x2", "x4", "x8"):
            self.rp_speed.addItem(s)
        self.rp_speed.setCurrentText("x1")
        self.rp_speed.currentTextChanged.connect(lambda s: self.replay_speed.emit(float(s[1:])))
        self.rp_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rp_slider.setRange(0, 1000)
        self.rp_slider.setMinimumWidth(320)
        self.rp_slider.sliderMoved.connect(self._seek)
        self.rp_time = W.label("0.0 de 0.0 s", color=theme.MUTED, mono=True)
        self.rp_exit = QtWidgets.QPushButton("Volver a en vivo")
        self.rp_exit.clicked.connect(self.replay_exit.emit)
        self._replay_widgets = (self.rp_play, self.rp_speed, self.rp_slider, self.rp_time, self.rp_exit)
        for w_ in self._replay_widgets:
            row.addWidget(w_, 1 if w_ is self.rp_slider else 0)
            w_.setVisible(False)
        self._playing = False
        return bar

    def set_mode(self, mode, name=""):
        self.mode = mode
        replay = mode == "replay"
        for w_ in self._replay_widgets:
            w_.setVisible(replay)
        if replay:
            self.page_sub.setText(f"Repetición de la corrida del {name}. El robot no se mueve.")
        else:
            self.page_sub.setText("Datos del tablero del controlador")
        if not replay:
            self._restore_steps()
        self.trail_pts.clear()
        self.hist_t.clear()
        self.hist_err.clear()
        self.hist_tau.clear()
        self._run_t0 = self._run_n0 = None
        self._sent = None
        self.ack_note.setVisible(False)
        self.recap.setVisible(False)
        self._update_next_button(self.last_state)

    def set_replay_steps(self, numbers):
        """En repeticion la lista muestra los pasos DE LA CORRIDA. El CSV no
        guarda los nombres, asi que no se inventan con la secuencia actual."""
        steps = [{"n": int(k), "type": "run", "label": f"Paso {int(k)}", "from_vision": False, "pose": ""}
                 for k in numbers]
        self.steps_by_n = {s["n"]: s for s in steps}
        self.step_list.set_steps(steps, steps[0]["n"] if steps else 1)
        if steps:
            self.seq_meta.setText(f"pasos {steps[0]['n']} a {steps[-1]['n']} de la corrida")
        self.step_label.setText("Repetición")
        self.step_desc.setText("El CSV no guarda los nombres de los pasos.")

    def _restore_steps(self):
        if not self.info:
            return
        steps = self.info.get("steps", [])
        self.steps_by_n = {s["n"]: s for s in steps}
        self.step_list.set_steps(steps, self.info.get("first_step", 1))
        self.seq_meta.setText(f"{len(steps)} pasos, arranca en el {self.info.get('first_step', 1)}" if steps else "")

    def set_playing(self, playing):
        self._playing = playing
        self.rp_play.setText("Pausa" if playing else "Reproducir")

    def _toggle_play(self):
        (self.replay_pause if self._playing else self.replay_play).emit()

    def _seek(self, v):
        self.replay_seek.emit(v / 1000.0)

    def set_replay_position(self, t, dur):
        self.rp_time.setText(f"{t:.1f} de {dur:.1f} s")
        if not self.rp_slider.isSliderDown() and dur > 0:
            self.rp_slider.setValue(int(1000 * t / dur))

    # ------------------------------------------------------------ datos
    def set_task_info(self, info):
        self.info = info
        steps = info.get("steps", [])
        self.steps_by_n = {s["n"]: s for s in steps}
        self.step_list.set_steps(steps, info.get("first_step", 1))
        self.hole_phase.set_steps(steps)
        self.seq_meta.setText(f"{len(steps)} pasos, arranca en el {info.get('first_step', 1)}" if steps else "")
        r = self.scene.renderer
        for a in self.pose_markers + self.pose_labels:
            a.visible(False)
        self.pose_markers, self.pose_labels = [], []
        used = {s["pose"] for s in steps if s["type"] in ("move", "vision")}
        for p in info.get("poses", []):
            if p["name"] not in used:
                continue
            m = scene3d.Sphere(r, theme.NEUTRAL, 0.007, 0.75)
            m.set(p["tcp_m"])
            lb = scene3d.Label(r, p["name"], theme.MUTED, 12)
            lb.set(np.array(p["tcp_m"]) + [0, 0, 0.03])
            self.pose_markers.append(m)
            self.pose_labels.append(lb)
        if self.box_actor is None and "workspace_min_m" in info:
            self.box_actor = self.scene.workspace_box(info["workspace_min_m"], info["workspace_max_m"])
        if self.low_box_actor is None and "low_zone_min_m" in info:
            self.low_box_actor = self.scene.workspace_box(info["low_zone_min_m"], info["low_zone_max_m"])
        self._reset_camera()
        self._apply_view_options()

    def set_live_available(self, ok):
        self.live_available = ok
        self._update_next_button(self.last_state)

    def update_state(self, st):
        self.last_state = st
        n = st.get("step_n")
        total = st.get("step_total") or (len(self.steps_by_n) or None)
        status = st.get("task_status")
        step = self.steps_by_n.get(n) if n else None
        if self.new_test.update(n, status, (self.info or {}).get("first_step", 1)):
            self.trail_pts.clear()
            self.trail.visible(False)
        if self._sent is not None and (n, status) != self._sent:
            self._sent = None
            self.ack_note.setVisible(False)
        t = st.get("t")
        if n and t is not None and (self._run_t0 is None or t < self._run_t0):
            self._run_t0, self._run_n0 = t, n

        # paso
        if n:
            shown_total = total if total and n <= total else None
            kind = TYPE_TEXT.get(step["type"], "") if step else ""
            self.step_count.setText(f"Paso {n}" + (f" de {shown_total}" if shown_total else "")
                                    + (f", {kind.lower()}" if kind else ""))
        if n and not step:
            # paso que no esta en la secuencia compilada (p. ej. una corrida vieja)
            self.step_label.setText(f"Paso {n}")
            self.step_desc.setText(st.get("step_desc", "") if self.mode == "live" else
                                   "El CSV no guarda los nombres de los pasos.")
        if step and step["type"] == "run":
            self.step_label.setText(step["label"])
        elif step:
            self.step_label.setText(step["label"])
            # describe = "[MOVER] etiqueta -> destino": la etiqueta ya es el titulo
            desc = step.get("describe", "")
            if " -> " in desc:
                desc = desc.split(" -> ", 1)[1]
                desc = desc[:1].upper() + desc[1:]
            self.step_desc.setText(desc)
        elif st.get("step_desc"):
            self.step_label.setText(st["step_desc"])
        if status in STATUS_TEXT:
            txt, col = STATUS_TEXT[status]
            if status == "paused" and step and step["type"] == "vision":
                txt = "En pausa, visión"
            self.status_chip.set(txt, col)
        elif status:
            self.status_chip.set(status, theme.NEUTRAL)
        if st.get("frozen"):
            self.status_chip.set("Congelada (control OFF)", theme.NEGATIVE)
        if st.get("step_time") is not None:
            self.time_chip.set(f"{st['step_time']:.1f} s", theme.NEUTRAL)
        if n and total:
            first = self.info.get("first_step", 1) if self.info else 1
            span = max(1, total - first + 1)
            self.progress.setValue(min(100, int(100 * max(0, n - first) / span)) if status != "done" else 100)
        if "message" in st:
            msg = st["message"]
            # "Paso 2: Ir a HOME_MOD" repite el titulo: no se muestra
            same = n and msg.strip().startswith(f"Paso {n}:")
            self.message.setText("" if same else msg)
            self.message.setVisible(bool(msg) and not same)
        mods = modules.names_html(step) if step else ""
        self.module_lbl.setText(f"<span style='color:{theme.MUTED}'>Módulo:</span> &nbsp;{mods}" if mods else "")
        self.module_lbl.setToolTip(modules.detail_html(step) if step else "")
        self.module_lbl.setVisible(bool(mods))
        self.hole_card.setVisible(self.hole_phase.update(n, status, st, live=self.mode == "live"))
        self._update_vision_row(st, step, status)
        self._update_recap(st, n, status, err_now=st.get("err_pos_cm"))
        self.step_list.set_current(n, status)
        if n:
            self.seq_scroll.ensureVisible(0, self.step_list.current_y() + 17, 0, 120)

        # medidores
        err = st.get("err_pos_cm")
        if err is None and "pose" in st and "target" in st:
            err = float(np.linalg.norm(st["pose"][:3] - st["target"][:3]) * 100)
        if err is not None:
            self.g_pos.set(err, color=None if err <= 1.5 else theme.WARNING)
        if st.get("err_ori_deg") is not None:
            self.g_ori.set(st["err_ori_deg"])
        if st.get("gripper_detected") is False:
            self.g_grip.set(None, "no detectada")
        elif st.get("gripper_pct") is not None:
            self.g_grip.set(st["gripper_pct"], f"comando {st['gripper_cmd']:.0f} %" if "gripper_cmd" in st else "")
        if st.get("fixture_q") is not None:
            sub = f"objetivo {st['fixture_target']:.0f}°" if st.get("fixture_target") is not None else ""
            self.g_fix.set(st["fixture_q"], sub, theme.NEGATIVE if st.get("fixture_fault") else None)

        # 3D
        if st.get("joints_deg") is not None:
            q = np.asarray(st["joints_deg"], dtype=float)
            if self.q_to is None:
                self.q_from = self.q_to = q
            else:
                self.q_from = self._q_now()
                # por el camino corto (J1, J3, J5, J7 giran sin fin)
                d = (q - self.q_from + 180.0) % 360.0 - 180.0
                self.q_to = self.q_from + d
            self.q_t0 = time.monotonic()
            self.q_dur = 0.04 if st.get("source") == "replay" else 0.2
            self.grip_pct = st.get("gripper_pct", self.grip_pct)
            self.robot.visible(True)
        if "pose" in st:
            pose = st["pose"]
            R = kin.quat_to_matrix(*pose[3:7])
            tcp = pose[:3] + R[:, 2] * kin.TCP_OFFSET_M
            self.tcp_dot.set(tcp)
            self.tcp_dot.visible(True)
            self.trail_pts.append(tcp)
            if "ramp" in st:
                self.ramp_dot.set(np.asarray(st["ramp"]) + R[:, 2] * kin.TCP_OFFSET_M)
                self.ramp_dot.visible(True)
        if "target" in st:
            tg = st["target"]
            Rt = kin.quat_to_matrix(*tg[3:7])
            Tt = kin.pose_matrix(tg[:3] + Rt[:, 2] * kin.TCP_OFFSET_M, Rt)
            self.target_frame.set(Tt)
            self.target_frame.visible(True)
            self.target_dot.set(Tt[:3, 3])
            self.target_dot.visible(True)
        self._apply_view_options(render=False)
        self.view3d.request_render()

        # graficas
        t = st.get("t")
        if t is not None:
            if self.hist_t and t < self.hist_t[-1]:
                self.hist_t.clear()
                self.hist_err.clear()
                self.hist_tau.clear()
            self.hist_t.append(t)
            self.hist_err.append(err if err is not None else np.nan)
            tau = st.get("tau_hw")
            self.hist_tau.append(np.asarray(tau) if tau is not None else np.full(7, np.nan))
            self._redraw_plots()
        self._update_next_button(st)

    def _redraw_plots(self):
        t = np.fromiter(self.hist_t, float)
        if len(t) < 2:
            return
        keep = t >= t[-1] - 60.0
        e = np.fromiter(self.hist_err, float)[keep]
        self.c_err.setData(t[keep], e)
        top = np.nanmax(e) if np.isfinite(e).any() else 0.0
        self.p_err.setYRange(0, max(2.0, 1.15 * top), padding=0)
        tau = np.array(self.hist_tau)[keep]
        for k, c in enumerate(self.c_tau):
            c.setData(t[keep], tau[:, k])

    def _q_now(self):
        if self.q_to is None:
            return None
        a = min(1.0, (time.monotonic() - self.q_t0) / max(self.q_dur, 1e-3))
        a = a * a * (3 - 2 * a)   # suave
        return self.q_from + (self.q_to - self.q_from) * a

    def _animate(self):
        q = self._q_now()
        if q is None or not self.isVisible():
            return
        self.robot.set_joints(q, self.grip_pct)
        self.view3d.request_render()

    def _apply_view_options(self, render=True):
        for m in self.pose_markers:
            m.visible(self.cb_poses.isChecked())
        for lb in self.pose_labels:
            lb.visible(self.cb_poses.isChecked() and self.cb_labels.isChecked())
        if self.cb_trail.isChecked() and len(self.trail_pts) > 1:
            self.trail.set(np.array(self.trail_pts))
            self.trail.visible(True)
        else:
            self.trail.visible(False)
        for a in (self.box_actor, self.low_box_actor):
            if a is not None:
                a.SetVisibility(self.cb_box.isChecked())
        if render:
            self.view3d.request_render()

    def _reset_camera(self):
        info = self.info or {}
        self.scene.frame_robot(info.get("workspace_min_m"), info.get("workspace_max_m"))
        self.view3d.request_render()

    # ------------------------------------------------------------ vision y final
    def _update_vision_row(self, st, step, status):
        """El agarre que va a usar el robot, junto al boton (no en otra pagina)."""
        msg = st.get("message", "") or ""
        uses = step is not None and (step["type"] == "vision" or step.get("from_vision"))
        if not uses or self.mode != "live":
            self.vision_row.setVisible(False)
            return
        if status == "paused" and "vision rechazada" in msg.lower():
            reason = msg.split(":", 1)[1].strip() if ":" in msg else msg
            self.vision_row.setText(f"<b>Visión rechazada por el controlador.</b> {reason}. "
                                    "El botón toma otra foto.")
            self.vision_row.setStyleSheet(f"color: {theme.WARNING};")
            self.vision_row.setVisible(True)
            return
        if "hoyo rechazado" in msg.lower():
            reason = msg.split("hoyo rechazado:", 1)[-1].strip()
            self.vision_row.setText(f"<b>Hoyo rechazado por el controlador.</b> {reason}. "
                                    "El botón toma otra foto.")
            self.vision_row.setStyleSheet(f"color: {theme.WARNING};")
            self.vision_row.setVisible(True)
            return
        if step.get("vision_approach"):
            state = st.get("approach_state")
            obj = "placa" if step.get("vision_object") == "PLATE" else "perno"
            if state and st.get("approach_ee") is not None:
                e, o = st["approach_ee"], st["approach_object"]
                txt = {"MOVE": f"<b>MOVE:</b> el {obj} no se ve completo; el brazo va a la pose del EE",
                       "INVIEW": f"<b>IN VIEW:</b> el {obj} se ve completo",
                       "ATTARGET": f"<b>En la pose</b>, pero el {obj} toca el borde de la imagen"}[state]
                self.vision_row.setText(f"{txt} (x {e[0]:.3f}, y {e[1]:.3f}, z {e[2]:.3f} m). "
                                        f"{obj.capitalize()} en x {o[0]:.3f}, y {o[1]:.3f} m.")
            else:
                self.vision_row.setText(f"Buscando el {obj} por su color…")
            self.vision_row.setStyleSheet(f"color: {theme.TEXT};")
            self.vision_row.setVisible(True)
            return
        if step.get("vision_target") == "hole":
            h = st.get("vision_hole")
            if h is not None:
                self.vision_row.setText(f"<b>Hoyo más chico:</b> Ø {st.get('vision_hole_diam_mm', float('nan')):.1f} mm "
                                        f"en x {h[0]:.4f}, y {h[1]:.4f} m. La pinza va encima de su centro.")
                self.vision_row.setStyleSheet(f"color: {theme.TEXT};")
                self.vision_row.setVisible(True)
            else:
                self.vision_row.setVisible(False)
            return
        if step.get("vision_target") == "plate_fixture" and st.get("plate_correction"):
            self.vision_row.setText("<b>Placa en la pinza:</b> " + st["plate_correction"] + ".")
            self.vision_row.setStyleSheet(f"color: {theme.TEXT};")
            self.vision_row.setVisible(True)
            return
        vt = st.get("vision_tcp")
        if vt is None or not step.get("from_vision"):
            # en el paso de la foto, los valores serian de la foto anterior
            self.vision_row.setVisible(False)
            return
        parts = [f"agarre z {vt[2]:.3f} m"]
        rpy = st.get("vision_rpy")
        if rpy is not None:
            zt = kin.rpy_to_matrix(rpy)[:, 2]
            parts.append(f"inclinación {np.degrees(np.arccos(np.clip(-zt[2], -1, 1))):.1f}°")
        pre = st.get("vision_pre")
        if pre is not None:
            parts.append(f"pre-grasp z {pre[2]:.3f} m")
        self.vision_row.setText("<b>Visión aceptada por el controlador:</b> " + ", ".join(parts) + ".")
        self.vision_row.setStyleSheet(f"color: {theme.TEXT};")
        self.vision_row.setVisible(True)

    def _update_recap(self, st, n, status, err_now=None):
        if status not in ("done", "aborted") or self.mode != "live":
            self.recap.setVisible(False)
            return
        t = st.get("t")
        dur = (t - self._run_t0) if (t is not None and self._run_t0 is not None) else None
        e = err_now
        if e is None and "pose" in st and "target" in st:
            e = float(np.linalg.norm(st["pose"][:3] - st["target"][:3]) * 100)
        if status == "done":
            txt = f"<b>Secuencia terminada.</b> Pasos {self._run_n0 or n} a {n}"
        else:
            txt = f"<b>Secuencia abortada en el paso {n}.</b> Pasos {self._run_n0 or n} a {n}"
        if dur is not None:
            txt += f" en {int(dur) // 60} min {int(dur) % 60:02d} s"
        txt += "."
        if e is not None:
            txt += f" Error de posición al terminar: {e:.2f} cm."
        self.recap_text.setText(txt)
        self.recap.setVisible(True)

    # ------------------------------------------------------------ boton
    def _update_pause_button(self, st):
        """Pausa solo con un paso en curso (en vivo y con el controlador)."""
        status = st.get("task_status") if st else None
        self.pause_btn.setEnabled(self.mode == "live" and self.live_available and status == "running"
                                  and self._sent is None)

    def _update_next_button(self, st):
        self._update_mode(st)
        self._update_pause_button(st)
        status = st.get("task_status") if st else None
        step = self.steps_by_n.get(st.get("step_n")) if st else None
        btn = self.next_btn
        if self.mode == "replay":
            btn.setText("Repetición, sin control")
            btn.setEnabled(False)
            self.next_hint.setText("En repetición el botón no hace nada.")
            return
        if not self.live_available:
            btn.setText("Controlador apagado")
            btn.setEnabled(False)
            self.next_hint.setText("Arráncalo en Sistema. ON/OFF y Quit están en la ventana del controlador.")
            return
        self.next_hint.setText("Hace lo mismo que el botón de la ventana del controlador.")
        blocked = time.monotonic() < self._next_block_until or self._sent is not None
        n = st.get("step_n") if st else None
        auto = bool(st and st.get("task_auto"))
        first = n is not None and self.info and n == self.info.get("first_step")
        if status == "waiting" and auto:
            btn.setText(("Iniciar en automático (paso {})" if first else "Seguir en automático (paso {})").format(n)
                        if n else "Iniciar en automático")
            self.next_hint.setText("Automático: hace todos los pasos solo y se detiene en pausas o si la "
                                  "visión rechaza algo. Para revisar cada paso, cambia a 'Paso a paso'.")
            btn.setEnabled(not blocked)
        elif status == "running" and auto:
            btn.setText(f"En automático: paso {n}…" if n else "En automático…")
            btn.setEnabled(False)
            self.next_hint.setText("Para que se detenga después de este paso, cambia a 'Paso a paso'.")
        elif status == "waiting":
            btn.setText(f"Ejecutar paso {n}" if n else "Siguiente paso")
            if step and step["type"] != "run":
                self.next_hint.setText(f"{step['label']}. Hace lo mismo que el botón de la ventana del controlador.")
            btn.setEnabled(not blocked)
        elif status == "paused" and "(operador)" in (st.get("message") or ""):
            btn.setText(f"Continuar paso {n}" if n else "Continuar")
            self.next_hint.setText("En pausa: el brazo se queda donde está. Continuar reanuda el paso desde "
                                  "el principio; la pinza y el fixture terminan lo que ya estaban haciendo.")
            btn.setEnabled(not blocked)
        elif status == "paused" and step and step["type"] == "vision":
            btn.setText("Tomar otra foto")
            btn.setEnabled(not blocked)
        elif status == "paused":
            btn.setText("Dar por terminado y seguir")
            btn.setEnabled(not blocked)
        elif status == "running":
            btn.setText("Ejecutando…")
            btn.setEnabled(False)
        elif status in ("done", "aborted"):
            btn.setText("Secuencia " + ("terminada" if status == "done" else "abortada"))
            btn.setEnabled(False)
        else:
            btn.setText("Siguiente paso")
            btn.setEnabled(not blocked)

    # ------------------------------------------------------------ modo
    def _on_mode(self, automatic):
        if self.mode != "live" or not self.live_available:
            return
        self._mode_sent = automatic
        self.mode_note.setText("Enviado, esperando al controlador…")
        self.mode_clicked.emit(automatic)
        QtCore.QTimer.singleShot(1500, self._check_mode_ack)

    def _check_mode_ack(self):
        if self._mode_sent is None:
            return
        self._mode_sent = None
        self.mode_note.setText("El controlador no confirmó el cambio de modo.")
        self._update_mode(self.last_state)

    def _update_mode(self, st):
        """Los botones muestran el modo que REPORTA el controlador."""
        usable = self.mode == "live" and self.live_available
        auto = st.get("task_auto") if st else None
        for b in (self.mode_auto, self.mode_step):
            b.setEnabled(usable and auto is not None)
        if auto is None:
            if usable:
                self.mode_note.setText("El controlador no reporta el modo (¿compilado antes del 2026-10-21?).")
            elif self._mode_sent is None:
                self.mode_note.setText("")
            return
        if self._mode_sent is not None and auto == self._mode_sent:
            self._mode_sent = None
        if self._mode_sent is None:
            self.mode_group.blockSignals(True)
            (self.mode_auto if auto else self.mode_step).setChecked(True)
            self.mode_group.blockSignals(False)
            self.mode_note.setText("Solo para en pausas y rechazos." if auto else "Un botón por paso.")

    def _on_next(self):
        st = self.last_state or {}
        first = self.info.get("first_step", 1) if self.info else 1
        # Al iniciar la secuencia (primer paso, esperando): antes los colores
        # de la placa y del perno (app._ask_colors). Cancelar no manda nada.
        if (st.get("task_status") == "waiting" and st.get("step_n") == first
                and self.before_start is not None and not self.before_start()):
            return
        self._next_block_until = time.monotonic() + 0.8   # sin doble clic
        self._sent = (st.get("step_n"), st.get("task_status"))
        self.next_btn.setEnabled(False)
        self.status_chip.set("Enviado, esperando al controlador…", theme.NEUTRAL)
        self.ack_note.setVisible(False)
        self.next_clicked.emit()
        QtCore.QTimer.singleShot(1500, self._check_ack)

    def _on_pause(self):
        st = self.last_state or {}
        self._sent = (st.get("step_n"), st.get("task_status"))
        self.pause_btn.setEnabled(False)
        self.next_btn.setEnabled(False)
        self.status_chip.set("Pausa enviada, esperando al controlador…", theme.NEUTRAL)
        self.ack_note.setVisible(False)
        self.pause_clicked.emit()
        QtCore.QTimer.singleShot(1500, self._check_ack)

    def _check_ack(self):
        """Si en 1.5 s el paso y el estado no cambiaron, el controlador no
        tomo el NEXT: se dice, en vez de reactivar el boton como si nada."""
        if self._sent is None:
            return
        self._sent = None
        self.ack_note.setText("El controlador no respondió al botón. Revisa su ventana: "
                              "puede estar con el control en OFF, ocupado o cerrado.")
        self.ack_note.setVisible(True)
        self._update_next_button(self.last_state)
