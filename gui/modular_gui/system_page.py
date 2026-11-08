"""Pagina 'Sistema': lanzar/parar el servidor de vision y el controlador,
compilar, parametros (de task_info) y logs."""

import datetime
import os

from PyQt5 import QtCore, QtWidgets

from . import color_dialog, config, theme, widgets as W


def _fmt_uptime(s):
    s = int(s)
    return f"{s // 3600:02d}:{s % 3600 // 60:02d}:{s % 60:02d}"


class ProcessCard(W.Card):
    start = QtCore.pyqtSignal()
    stop = QtCore.pyqtSignal()

    def __init__(self, title, subtitle, start_text, parent=None):
        super().__init__(title, parent)
        self.sub = W.label(subtitle, color=theme.MUTED, wrap=True)
        self.add(self.sub)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(24)
        self.chip = W.Chip("Estado")
        self.extra = W.Chip("Tiempo / respuesta")
        row.addWidget(self.chip)
        row.addWidget(self.extra)
        self.add(row)
        btns = QtWidgets.QHBoxLayout()
        btns.setSpacing(10)
        self.start_btn = QtWidgets.QPushButton(start_text)
        self.start_btn.clicked.connect(self.start.emit)
        self.stop_btn = QtWidgets.QPushButton("Detener (Ctrl+C)")
        self.stop_btn.setObjectName("danger")
        self.stop_btn.clicked.connect(self.stop.emit)
        btns.addWidget(self.start_btn, 1)
        btns.addWidget(self.stop_btn, 1)
        self.btn_row = btns
        self.add(btns)
        self.detail = W.label("", obj="cardMeta", wrap=True)
        self.add(self.detail)


class SystemPage(QtWidgets.QWidget):
    def show_colors(self, plate, bolt):
        """Colores con los que trabaja la vision (se eligen al iniciar la secuencia)."""
        names = [config.COLOR_NAMES.get(c, c) for c in (plate, bolt)]
        self.color_note.setText(f"Colores: placa {names[0]}, perno {names[1]}. Se eligen al iniciar "
                                "la secuencia (botón del primer paso), sin reiniciar la visión.")

    start_vision = QtCore.pyqtSignal()
    stop_vision = QtCore.pyqtSignal()
    start_controller = QtCore.pyqtSignal()
    stop_controller = QtCore.pyqtSignal()
    compile_requested = QtCore.pyqtSignal()
    reload_info = QtCore.pyqtSignal()
    follow_log = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.vision = ProcessCard(
            "Servidor de visión",
            f"screw_server de {config.VISION_DIR.name} (GPD). Centra la cámara sobre el objeto por su color "
            "y toma la foto cuando la secuencia o la foto de prueba la pide. "
            f"UDP {config.VISION_PORT}.", "Iniciar visión")
        self.vision.start.connect(self.start_vision.emit)
        # color de cada objeto: se elige al iniciar la secuencia (color_dialog);
        # la vision arranca con el ultimo elegido
        self.color_note = W.label("", obj="cardMeta", wrap=True)
        self.vision.add(self.color_note)
        self.show_colors(*color_dialog.saved_colors())
        if config.VISION_VERSION != "new":
            self.color_note.setText("Colores: solo con la versión nueva (MODULAR_VISION=v2 activo).")
        self.vision.stop.connect(self.stop_vision.emit)

        self.ctrl = ProcessCard(
            "Controlador de impedancia",
            "impedance_gen3: se conecta al robot y abre su ventana (slider ON/OFF, Siguiente paso, Quit). "
            "La interfaz lee su tablero; no cambia cómo se controla el robot.", "Iniciar controlador")
        self.ctrl.start.connect(self.start_controller.emit)
        self.ctrl.stop.connect(self.stop_controller.emit)
        self.compile_btn = QtWidgets.QPushButton("Compilar")
        self.compile_btn.setToolTip("make -j8 impedance_gen3 task_info")
        self.compile_btn.clicked.connect(self.compile_requested.emit)
        row2 = QtWidgets.QHBoxLayout()
        row2.setSpacing(10)
        row2.addWidget(self.compile_btn, 1)
        self.compile_btn.setVisible(config.CAN_COMPILE)
        self.ctrl.add(row2)
        self.follow_btn = QtWidgets.QPushButton("Seguir log…")
        self.follow_btn.setToolTip("Si arrancaste el controlador afuera con:  ./impedance_gen3 > archivo.log")
        self.follow_btn.clicked.connect(self._choose_log)
        row2.addWidget(self.follow_btn, 1)
        self.build_warn = W.label("", color=theme.WARNING, weight=500, wrap=True)
        self.ctrl.add(self.build_warn)

        safety = W.Card("Seguridad")
        for line in ("Esta interfaz no controla el robot: lanza programas, lee lo que imprimen y manda "
                     "Siguiente paso, igual que el botón del controlador.",
                     "ON/OFF del control y Quit están en la ventana GTK del controlador.",
                     "Si la interfaz se cierra o se traba, el controlador sigue igual.",
                     "Paro de emergencia siempre a la mano."):
            safety.add(W.label(line, wrap=True))

        # parametros
        self.params_card = W.Card("Parámetros compilados (task_info)")
        self.params_grid = QtWidgets.QGridLayout()
        self.params_grid.setHorizontalSpacing(24)
        self.params_grid.setVerticalSpacing(0)
        self.params_card.add(self.params_grid)
        rl = QtWidgets.QPushButton("Recargar")
        rl.clicked.connect(self.reload_info.emit)
        self.params_card.header_right(rl)

        # logs
        logs = W.Card("Logs")
        self.log_tabs = QtWidgets.QTabWidget()
        self.log_ctrl = W.LogView()
        self.log_vision = W.LogView()
        self.log_build = W.LogView()
        self.log_tabs.addTab(self.log_ctrl, "Controlador (eventos)")
        self.log_tabs.addTab(self.log_vision, "Visión")
        self.log_tabs.addTab(self.log_build, "Compilación / foto")
        logs.add(self.log_tabs, 1)

        left = QtWidgets.QVBoxLayout()
        left.setSpacing(16)
        left.setContentsMargins(0, 0, 0, 0)
        left.addWidget(self.vision)
        left.addWidget(self.ctrl)
        left.addWidget(safety)
        left.addStretch(1)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(16)
        right.addWidget(self.params_card)
        right.addWidget(logs, 1)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(24, 20, 24, 24)
        outer.setSpacing(16)
        tcol = QtWidgets.QVBoxLayout()
        tcol.setSpacing(2)
        tcol.addWidget(W.label("Sistema", obj="pageTitle"))
        tcol.addWidget(W.label("Programas, parámetros compilados y logs", obj="pageSub"))
        outer.addLayout(tcol)
        lay = QtWidgets.QHBoxLayout()
        lay.setSpacing(16)
        outer.addLayout(lay, 1)
        lw = QtWidgets.QWidget()
        lw.setLayout(left)
        lw.setMaximumWidth(560)
        lay.addWidget(lw, 2)
        lay.addLayout(right, 3)

    def _choose_log(self):
        f, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Log del controlador", str(config.BUILD_DIR),
                                                     "Logs (*.log *.txt);;Todos (*)")
        if f:
            self.follow_log.emit(f)

    # ------------------------------------------------------------ estado
    def set_process_state(self, card, running, external, pid, uptime, log_path, exit_code, ping=None):
        if running:
            card.chip.set("Corriendo", theme.POSITIVE)
            card.extra.set(_fmt_uptime(uptime), theme.NEUTRAL)
        elif external:
            card.chip.set("Corriendo afuera", theme.WARNING)
            card.extra.set("no se lanzó aquí", theme.NEUTRAL)
        else:
            card.chip.set("Detenido" if exit_code in (None, 0, -2) else f"Salió con código {exit_code}",
                          theme.NEUTRAL if exit_code in (None, 0, -2) else theme.NEGATIVE)
            card.extra.set("—", theme.NEUTRAL)
        if ping is not None and (running or external):
            card.extra.set(("contesta PING" if ping else "sin respuesta") + (f", {_fmt_uptime(uptime)}" if running else ""),
                           theme.POSITIVE if ping else theme.WARNING)
        card.start_btn.setEnabled(not running and not external)
        card.stop_btn.setEnabled(running)
        parts = []
        if external and not running:
            parts.append("Se lanzó fuera de la interfaz: detenlo en su terminal (Ctrl+C).")
        if pid:
            parts.append(f"PID {pid}")
        if log_path:
            parts.append(f"log: {os.path.relpath(log_path, config.PROJECT_DIR)}")
        card.detail.setText("   ".join(parts))
        card.detail.setVisible(bool(parts))

    def set_build_state(self):
        """Avisa si la configuracion es mas nueva que el binario."""
        try:
            b = os.path.getmtime(config.CONTROLLER_BIN)
        except OSError:
            self.build_warn.setText("No existe build/impedance_gen3: compílalo.")
            self.build_warn.setVisible(True)
            return
        newer = []
        for f in ("include/controller_config.hpp", "include/task_sequence.hpp", "src/main_impedance.cpp",
                  "include/cartesian_impedance_controller.hpp", "src/cartesian_impedance_controller.cpp"):
            p = config.PROJECT_DIR / f
            if p.exists() and os.path.getmtime(p) > b + 1:
                newer.append(os.path.basename(f))
        when = datetime.datetime.fromtimestamp(b).strftime("%d/%m %H:%M")
        self.build_warn.setVisible(bool(newer))
        if newer:
            self.build_warn.setText(f"{', '.join(newer)} cambió después de compilar ({when}). Compila antes de correr.")
        else:
            self.build_warn.setText("")
            self.build_warn.setVisible(False)
            self.build_warn.setToolTip(f"Compilado {when}")

    def set_task_info(self, info):
        while self.params_grid.count():
            it = self.params_grid.takeAt(0)
            if it.widget():
                it.widget().deleteLater()
        p = info.get("params", {})

        def vec(v, d=0):
            return "  ".join(f"{x:.{d}f}" for x in v)

        rows = [
            ("Rigidez traslación [N/m]", vec(p.get("stiffness", [])[:3])),
            ("Rigidez rotación [Nm/rad]", vec(p.get("stiffness", [])[3:])),
            ("Amortiguamiento (factor)", vec(p.get("damping_factors", []), 1)),
            ("Rampa", f"{p.get('reference_max_speed_m_s', 0) * 100:.0f} cm/s, {p.get('reference_max_angular_speed_deg_s', 0):.0f} °/s"
             if p.get("reference_max_speed_m_s") else "apagada"),
            ("pose_filter", f"{p.get('pose_filter', 1):g}"),
            ("Fricción", (f"ON, signo {p.get('friction_sign_factor', 0):.0e}, Kfric {vec(p.get('Kfric', []), 1)}"
                          if p.get("enable_friction_compensation") else "OFF")),
            ("Espacio nulo", (f"ON, K {p.get('nullspace_stiffness', 0):g}, pesos {vec(p.get('nullspace_weights', []))}"
                              if p.get("enable_nullspace") else "OFF")),
            ("Límite J4", f"ON, {p.get('j4_min_deg', 0):g}°" if p.get("enable_j4_limit") else "OFF"),
            ("Llegada", f"{p.get('position_tolerance_m', 0) * 100:.1f} cm, {p.get('orientation_tolerance_deg', 0):g}°"),
            ("Visión: inclinación máx", f"{p.get('vision_max_tilt_deg', 0):g}°"),
            ("Secuencia", f"{len(info.get('steps', []))} pasos, arranca en el {info.get('first_step', 1)}"
                          + (", confirma cada paso" if info.get("confirm_each_step") else "")),
        ]
        for i, (k, v) in enumerate(rows):
            kl = W.label(k, color=theme.MUTED)
            vl = W.label(v, mono=True)
            vl.setStyleSheet(f'font-family: "{theme.MONO_FONT}"; font-size: 13px;')
            for w_ in (kl, vl):
                w_.setMinimumHeight(32)
                w_.setStyleSheet(w_.styleSheet() + f" border-bottom: 1px solid {theme.RULE};")
            self.params_grid.addWidget(kl, i, 0)
            self.params_grid.addWidget(vl, i, 1)
        self.params_grid.setColumnStretch(1, 1)
        if info.get("error"):
            self.params_grid.addWidget(W.label(info["error"], color=theme.NEGATIVE, wrap=True), len(rows), 0, 1, 2)
