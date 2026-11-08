"""Ventana principal: encabezado, navegacion y la logica que une todo."""

import json
import os
import re
import socket
import subprocess
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from . import color_dialog, config, parsers, procs, runs, theme, widgets as W
from .analysis_page import AnalysisPage
from .live_page import LivePage
from .present_page import PresentPage
from .system_page import SystemPage
from .vision_page import VisionPage


class VisionPinger(QtCore.QObject):
    """PING al screw_server sin bloquear: manda uno cada tick y revisa si
    llego el PONG del anterior."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.alive = False
        self._pending = False

    def tick(self):
        got = False
        try:
            while True:
                data = self.sock.recv(64)
                if data.strip() == b"PONG":
                    got = True
        except (BlockingIOError, ConnectionRefusedError, OSError):
            pass
        if self._pending:
            self.alive = got
        try:
            self.sock.sendto(b"PING", ("127.0.0.1", config.VISION_PORT))
            self._pending = True
        except OSError:
            self._pending = False
            self.alive = False
        return self.alive


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MODULAR · Kinova Gen3 — Interfaz")
        self.resize(1680, 1000)
        config.LOG_DIR.mkdir(parents=True, exist_ok=True)
        procs.cleanup_logs()

        self.info = {}
        self.last_live = None
        self.last_live_at = 0.0
        self.vision_text = {}      # log -> texto

        # ---------- procesos ----------
        self.vision = procs.ManagedProcess(
            "vision", [config.SCREW_SERVER_BIN, *config.SCREW_SERVER_ARGS], config.VISION_BUILD_DIR, "vision",
            line_buffered=True, parent=self)
        ctrl_cmd = [config.CONTROLLER_BIN]
        if config.FAKE_CONTROLLER:
            ctrl_cmd = [sys.executable, str(config.GUI_DIR / "tests" / "fake_controller.py")]
            self.setWindowTitle(self.windowTitle() + "   [PRUEBA: controlador falso]")
        self.ctrl = procs.ManagedProcess("controlador", ctrl_cmd, config.BUILD_DIR, "controlador", parent=self)
        self.ctrl_tail = procs.LogTail(self)
        self.ctrl_tail.text.connect(self._on_ctrl_text)
        self.vision_tail = procs.LogTail(self)
        self.vision_tail.text.connect(self._on_vision_text)
        self.stream = parsers.DashboardStream()
        self.pinger = VisionPinger(self)
        self.replay = runs.Replay(self)
        self.ctrl_external = False
        self.vision_external = False

        # ---------- paginas ----------
        self.live = LivePage()
        self.visionp = VisionPage()
        self.analysis = AnalysisPage()
        self.system = SystemPage()
        self.present = PresentPage()

        self._build_ui()
        # el anillo de foco solo al navegar con Tab, no despues de cada clic
        for w_ in self.findChildren((QtWidgets.QPushButton, QtWidgets.QCheckBox)):
            w_.setFocusPolicy(QtCore.Qt.TabFocus)
        self._connect()
        self.load_task_info()
        self._load_old_vision_results()

        self._status_timer = QtCore.QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)
        self._status_timer.start(1000)
        self._clock = QtCore.QTimer(self)
        self._clock.timeout.connect(self._tick_clock)
        self._clock.start(500)
        self._refresh_status()
        QtWidgets.QShortcut(QtGui.QKeySequence("F11"), self, self._toggle_fullscreen)
        QtWidgets.QShortcut(QtGui.QKeySequence("F5"), self, self._toggle_present)
        self._esc = QtWidgets.QShortcut(QtGui.QKeySequence("Esc"), self, self._exit_present)
        self._esc.setEnabled(False)
        for i in range(4):
            QtWidgets.QShortcut(QtGui.QKeySequence(f"Ctrl+{i + 1}"), self,
                                lambda i=i: self._go_page(i))
        self._presenting = False
        self._was_fullscreen = False
        self._mirror_timer = QtCore.QTimer(self)
        self._mirror_timer.timeout.connect(self._mirror_present)
        self._mirror_timer.start(250)

    # ------------------------------------------------------------ UI
    def _build_ui(self):
        root = QtWidgets.QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        v = QtWidgets.QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        # Marco en L: barra superior + lateral sobre el mismo tono, sin linea
        # entre ellas; las paginas viven en una hoja blanca.
        header = QtWidgets.QWidget()
        header.setFixedHeight(56)
        h = QtWidgets.QHBoxLayout(header)
        h.setContentsMargins(24, 8, 24, 8)
        h.setSpacing(10)
        h.addWidget(LogoMark())
        tl = QtWidgets.QVBoxLayout()
        tl.setSpacing(0)
        tl.addStretch(1)
        tl.addWidget(W.label(config.TITLE, obj="productName"))
        tl.addWidget(W.label(config.SUBTITLE, obj="productScope"))
        tl.addStretch(1)
        h.addLayout(tl)
        h.addStretch(1)
        self.chip_ctrl = W.Chip("Controlador")
        self.chip_vis = W.Chip("Visión")
        self.chip_control = W.Chip("Control")
        self.chip_seq = W.Chip("Secuencia")
        for c in (self.chip_ctrl, self.chip_vis, self.chip_control, self.chip_seq):
            c.setMinimumWidth(150)
            h.addWidget(c)
            h.addSpacing(14)
        self.clock = W.label("", color=theme.MUTED, size=theme.FS_BODY, mono=True)
        self.clock.setMinimumWidth(70)
        self.clock.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        h.addWidget(self.clock)
        v.addWidget(header)
        self._header = header

        body = QtWidgets.QHBoxLayout()
        self._body = body
        body.setContentsMargins(0, 0, 8, 8)
        body.setSpacing(0)
        side = QtWidgets.QWidget()
        side.setFixedWidth(220)
        s = QtWidgets.QVBoxLayout(side)
        s.setContentsMargins(0, 12, 0, 16)
        s.setSpacing(2)
        self.stack = QtWidgets.QStackedWidget()
        self.nav = QtWidgets.QButtonGroup(self)
        for i, (name, page) in enumerate([
                ("En vivo", self.live), ("Visión", self.visionp),
                ("Análisis de corridas", self.analysis), ("Sistema", self.system)]):
            b = QtWidgets.QPushButton(name)
            b.setObjectName("navButton")
            b.setCheckable(True)
            b.setCursor(QtCore.Qt.PointingHandCursor)
            self.nav.addButton(b, i)
            s.addWidget(b)
            self.stack.addWidget(page)
        self.nav.button(0).setChecked(True)
        self.nav.idClicked.connect(self.stack.setCurrentIndex)
        s.addStretch(1)
        foot = W.label("Capa de interfaz: no controla el robot. ON/OFF y Quit están en la ventana "
                       "del controlador.\n\nF5  vista de presentación\nF11  pantalla completa\n"
                       "Ctrl+1 a 4  cambiar de página", obj="sideNote", wrap=True)
        foot.setContentsMargins(22, 0, 16, 0)
        s.addWidget(foot)
        body.addWidget(side)
        self._side = side
        sheet = QtWidgets.QFrame()
        sheet.setObjectName("sheet")
        sl = QtWidgets.QVBoxLayout(sheet)
        sl.setContentsMargins(1, 1, 1, 1)
        sl.addWidget(self.stack)
        self.stack.addWidget(self.present)   # indice 4: fuera del menu (F5)
        self._sheet = sheet
        body.addWidget(sheet, 1)
        v.addLayout(body, 1)

    def _connect(self):
        sp = self.system
        sp.start_vision.connect(self._start_vision)
        sp.stop_vision.connect(lambda: self._stop(self.vision, "el servidor de visión"))
        sp.start_controller.connect(self._start_controller)
        sp.stop_controller.connect(lambda: self._stop(self.ctrl, "el controlador"))
        sp.compile_requested.connect(self._compile)
        sp.reload_info.connect(self.load_task_info)
        sp.follow_log.connect(self._follow_external_log)
        self.vision.changed.connect(self._refresh_status)
        self.ctrl.changed.connect(self._refresh_status)

        self.live.next_clicked.connect(self._send_next)
        self.live.before_start = self._ask_colors
        self.live.pause_clicked.connect(self._send_pause)
        self.live.mode_clicked.connect(self._send_mode)
        self.live.replay_play.connect(self._replay_play)
        self.live.replay_pause.connect(self._replay_pause)
        self.live.replay_seek.connect(lambda f: self.replay.seek(f * (self.replay.run.duration if self.replay.run else 0)))
        self.live.replay_speed.connect(lambda s: setattr(self.replay, "speed", s))
        self.live.replay_exit.connect(self._replay_exit)
        self.replay.state.connect(self._on_replay_state)
        self.replay.position.connect(self.live.set_replay_position)
        self.replay.finished.connect(lambda: self.live.set_playing(False))
        self.analysis.replay_requested.connect(self._start_replay)
        self.live.present_clicked.connect(self._enter_present)
        self.live.open_analysis.connect(self._open_latest_run)
        self.present.exit_requested.connect(self._exit_present)
        self.present.next_clicked.connect(self.live._on_next)
        self.present.pause_clicked.connect(self.live._on_pause)

        self.visionp.request_photo.connect(self._take_photo)
        self.visionp.compute_grasp.connect(self._compute_grasp)
        self.visionp.manual_clicked.connect(self._open_manual)

    # ------------------------------------------------------------ task_info
    def load_task_info(self):
        try:
            out = subprocess.run([str(config.TASK_INFO_BIN)], capture_output=True, text=True, timeout=10)
            self.info = json.loads(out.stdout)
        except Exception as e:   # noqa: BLE001
            self.info = {"steps": [], "poses": [], "params": {}, "error": f"task_info: {e} (compila task_info)"}
        for page in (self.live, self.visionp, self.analysis, self.system, self.present):
            page.set_task_info(self.info)
        self.system.set_build_state()

    # ------------------------------------------------------------ procesos
    def _open_manual(self):
        """Procedimiento de Antony a mano (approach_object / grasps_from_camera)."""
        if getattr(self, "_manual", None) is None:
            from . import manual_vision
            self._manual = manual_vision.ManualVisionDialog(lambda: self.ctrl.running(), self)
        self._manual.show()
        self._manual.raise_()

    def _start_vision(self):
        if not config.SCREW_SERVER_BIN.exists():
            QtWidgets.QMessageBox.warning(self, "Falta el servidor", f"No existe {config.SCREW_SERVER_BIN}")
            return
        # color de la placa y del perno elegidos en Sistema
        self.vision.cmd = [config.SCREW_SERVER_BIN, *config.screw_server_args(*color_dialog.saved_colors())]
        self.vision.start()
        self.vision_tail.follow(self.vision.log_path)
        self.system.log_vision.clear()

    def _start_controller(self):
        if not config.FAKE_CONTROLLER and not config.CONTROLLER_BIN.exists():
            QtWidgets.QMessageBox.warning(self, "Falta el controlador", "Compila build/impedance_gen3 primero.")
            return
        warn = []
        if self.system.build_warn.text():
            warn.append(self.system.build_warn.text())
        uses_vision = any(s["type"] == "vision" and s["n"] >= self.info.get("first_step", 1)
                          for s in self.info.get("steps", []))
        if uses_vision and not self.pinger.alive:
            warn.append("⚠ La secuencia usa visión y el servidor no contesta: el controlador no va a arrancar.")
        txt = ("Se va a conectar al robot y abrir su ventana (slider ON/OFF, Siguiente paso, Quit).\n\n"
               f"Secuencia: {len(self.info.get('steps', []))} pasos, arranca en el {self.info.get('first_step', 1)}.\n\n"
               "¿Paro de emergencia a la mano?")
        if warn:
            txt += "\n\n" + "\n".join(warn)
        r = QtWidgets.QMessageBox.question(self, "Iniciar controlador", txt,
                                           QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
                                           QtWidgets.QMessageBox.Cancel)
        if r != QtWidgets.QMessageBox.Yes:
            return
        self.stream = parsers.DashboardStream()
        self.ctrl.start()
        self.ctrl_tail.follow(self.ctrl.log_path)
        self.system.log_ctrl.clear()
        self.system.log_ctrl.append_lines([f"[interfaz] controlador lanzado, log {self.ctrl.log_path}"])

    def _follow_external_log(self, path):
        self.stream = parsers.DashboardStream()
        self.ctrl_tail.follow(path, from_start=False)
        self.system.log_ctrl.append_lines([f"[interfaz] siguiendo {path}"])

    def _stop(self, proc, what):
        r = QtWidgets.QMessageBox.question(
            self, "Detener", f"¿Detener {what}? (igual que Ctrl+C)",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel, QtWidgets.QMessageBox.Cancel)
        if r == QtWidgets.QMessageBox.Yes:
            proc.interrupt()

    def _compile(self):
        if self.ctrl.running():
            QtWidgets.QMessageBox.information(self, "Compilar", "Detén el controlador antes de compilar.")
            return
        self.system.compile_btn.setEnabled(False)
        self.system.log_tabs.setCurrentIndex(2)
        self.system.log_build.append_lines(["$ make -j8 impedance_gen3 task_info"])
        p = QtCore.QProcess(self)
        p.setWorkingDirectory(str(config.BUILD_DIR))
        p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        p.readyReadStandardOutput.connect(
            lambda: self.system.log_build.append_lines(
                bytes(p.readAllStandardOutput()).decode(errors="replace").rstrip().splitlines()))

        def done(code, _status):
            self.system.log_build.append_lines([f"[compilación terminada: {'OK' if code == 0 else 'ERROR ' + str(code)}]"])
            self.system.compile_btn.setEnabled(True)
            self.load_task_info()

        p.finished.connect(done)
        p.start("make", ["-j8", "impedance_gen3", "task_info"])

    # ------------------------------------------------------------ controlador
    def _on_ctrl_text(self, text):
        st = self.stream.feed(text)
        ev = self.stream.take_events()
        if ev:
            self.system.log_ctrl.append_lines(ev)
        if st is not None:
            vt = st.get("vision_tcp")
            prev = self.last_live.get("vision_tcp") if self.last_live else None
            if vt is not None and (prev is None or not (vt == prev).all()):
                g = {"position": vt, "rpy": st.get("vision_rpy"), "pregrasp": st.get("vision_pre")}
                self.visionp.attach_to_newest({k: v for k, v in g.items() if v is not None},
                                              time.time() - 600, "controlador")
            self.last_live = st
            self.last_live_at = time.monotonic()
            if self.live.mode == "live":
                self.live.update_state(st)
                self.present.update_state(st)
            self._update_header_from_state(st)

    def _ask_colors(self):
        """Antes del primer paso: color de la placa y del perno, y se los
        manda al servidor de vision ("COLORS"). False = no iniciar."""
        if config.VISION_VERSION != "new" or config.FAKE_CONTROLLER:
            return True     # V2 no entiende COLORS; el controlador falso no tiene vision
        dlg = color_dialog.ColorDialog(self)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return False
        plate, bolt = dlg.colors()
        color_dialog.save_colors(plate, bolt)
        ok, why = procs.send_colors(plate, bolt)
        if not ok:
            QtWidgets.QMessageBox.warning(
                self, "Colores", f"No se pudieron cambiar los colores: {why}.\n"
                "La secuencia no se inició. Revisa que el servidor de visión esté corriendo.")
            return False
        names = [config.COLOR_NAMES.get(c, c) for c in (plate, bolt)]
        self.system.log_vision.append_lines([f"[interfaz] colores: placa {names[0]}, perno {names[1]}"])
        self.system.show_colors(plate, bolt)
        return True

    def _send_next(self):
        try:
            procs.send_next_step()
            self.system.log_ctrl.append_lines(["[interfaz] 'Siguiente paso' enviado"])
        except OSError as e:
            QtWidgets.QMessageBox.warning(self, "Siguiente paso", f"No se pudo enviar: {e}")

    def _send_pause(self):
        try:
            procs.send_pause()
            self.system.log_ctrl.append_lines(["[interfaz] 'Pausa' enviada"])
        except OSError as e:
            QtWidgets.QMessageBox.warning(self, "Pausa", f"No se pudo enviar: {e}")

    def _send_mode(self, automatic):
        try:
            procs.send_mode(automatic)
            self.system.log_ctrl.append_lines(
                [f"[interfaz] modo {'AUTOMATICO' if automatic else 'PASO A PASO'} enviado"])
        except OSError as e:
            QtWidgets.QMessageBox.warning(self, "Modo", f"No se pudo enviar: {e}")

    # ------------------------------------------------------------ vision
    def _on_vision_text(self, text):
        self.system.log_vision.append_lines(parsers.strip_ansi(text).rstrip("\n").splitlines())
        key = str(self.vision_tail.path)
        self.vision_text[key] = self.vision_text.get(key, "") + text
        if "Answer sent" in text or "Saved screw_" in text:
            caps = parsers.parse_vision_log(self.vision_text[key])
            for c in caps:
                c["source"] = "server"
            self.visionp.set_results(caps)

    def _load_old_vision_results(self):
        caps = []
        for f in sorted(config.LOG_DIR.glob("vision_*.log"), key=os.path.getmtime):
            try:
                caps += [dict(c, source="server") for c in parsers.parse_vision_log(f.read_text(errors="replace"))]
            except OSError:
                pass
        for f in sorted(config.LOG_DIR.glob("detect_screw_*.txt"), key=os.path.getmtime):
            name = f.stem[len("detect_"):]
            try:
                caps += [dict(c, source="GPD recalculado")
                         for c in parsers.parse_vision_log(f"Saved {name}.pcd\n" + f.read_text(errors="replace"))]
            except OSError:
                pass
        if caps:
            self.visionp.set_results(caps)

    def _noplot_cfg(self):
        dst = config.LOG_DIR / "gpd_noplot.cfg"
        txt = config.GPD_CONFIG.read_text()
        txt = re.sub(r"^(plot_\w+)\s*=\s*1", r"\1 = 0", txt, flags=re.M)
        dst.write_text(txt)
        return dst

    def _compute_grasp(self, name):
        cfg = self._noplot_cfg()
        out_file = config.LOG_DIR / f"detect_{name}.txt"
        p = QtCore.QProcess(self)
        p.setWorkingDirectory(str(config.VISION_BUILD_DIR))
        p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        buf = []
        p.readyReadStandardOutput.connect(lambda: buf.append(bytes(p.readAllStandardOutput()).decode(errors="replace")))

        def done(code, _s):
            text = "".join(buf)
            out_file.write_text(text)
            caps = parsers.parse_vision_log(f"Saved {name}.pcd\n" + text)
            for c in caps:
                c["source"] = "GPD recalculado"
            self.visionp.results.pop(name, None)
            self.visionp.set_results(caps)
            self.visionp.calc_done()
            self.system.log_build.append_lines([f"[{config.GRASPS_FROM_FILE_BIN.name} {name}: código {code}]"])

        p.finished.connect(done)
        p.start(str(config.GRASPS_FROM_FILE_BIN),
                [str(cfg), str(config.capture_path(name)), str(config.capture_path(name, "_pose.txt"))])

    def _take_photo(self, pose):
        args = [f"{x:.5f}" for x in pose]
        self.system.log_build.append_lines(["$ vision_request_test " + " ".join(args)])
        p = QtCore.QProcess(self)
        p.setWorkingDirectory(str(config.BUILD_DIR))
        p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        buf = []
        since = time.time()

        def read():
            txt = bytes(p.readAllStandardOutput()).decode(errors="replace")
            buf.append(txt)
            self.system.log_build.append_lines(txt.rstrip().splitlines())

        def done(_code, _s):
            g = parsers.parse_vision_test("".join(buf))
            if g:
                self.visionp.attach_to_newest(g, since, "foto de prueba")

        p.readyReadStandardOutput.connect(read)
        p.finished.connect(done)
        p.start(str(config.VISION_TEST_BIN), args)

    # ------------------------------------------------------------ repeticion
    def _start_replay(self, run):
        if run is None:
            return
        self.replay.pause()
        self.replay.load(run)
        self.live.set_mode("replay", run_label(run.path))
        self.live.set_replay_steps(sorted(set(int(k) for k in run.step.tolist())))
        self.present.set_mode("replay", run_label(run.path))
        if not self._presenting:
            self.nav.button(0).setChecked(True)
            self.stack.setCurrentIndex(0)
        self.replay.seek(0.0)
        self._replay_play()

    def _replay_play(self):
        self.replay.play()
        self.live.set_playing(True)

    def _replay_pause(self):
        self.replay.pause()
        self.live.set_playing(False)

    def _replay_exit(self):
        self.replay.pause()
        self.live.set_playing(False)
        self.live.set_mode("live")
        self.present.set_mode("live")
        if self.last_live:
            self.live.update_state(self.last_live)
            self.present.update_state(self.last_live)

    def _on_replay_state(self, st):
        if self.live.mode == "replay":
            self.live.update_state(st)
            self.present.update_state(st)
            self._update_header_from_state(st)

    # ------------------------------------------------------------ estado
    def _refresh_status(self):
        ctrl_run = self.ctrl.running()
        self.ctrl_external = not ctrl_run and procs.process_running("impedance_gen3")
        vis_run = self.vision.running()
        self.vision_external = not vis_run and procs.process_running("screw_server")
        ping = self.pinger.tick() if (vis_run or self.vision_external) else False
        if not (vis_run or self.vision_external):
            self.pinger.alive = False

        self.system.set_process_state(self.system.vision, vis_run, self.vision_external, self.vision.pid() if vis_run else None,
                                      self.vision.uptime(), self.vision.log_path, self.vision.exit_code, ping)
        self.system.set_process_state(self.system.ctrl, ctrl_run, self.ctrl_external, self.ctrl.pid() if ctrl_run else None,
                                      self.ctrl.uptime(), self.ctrl.log_path, self.ctrl.exit_code)
        self.system.set_build_state()

        live_ok = ctrl_run or self.ctrl_external
        self.live.set_live_available(live_ok)
        if ctrl_run:
            fresh = time.monotonic() - self.last_live_at < 2.0
            self.chip_ctrl.set("Corriendo" if fresh else "Arrancando…", theme.POSITIVE if fresh else theme.WARNING)
        elif self.ctrl_external:
            self.chip_ctrl.set("Corriendo afuera", theme.WARNING)
        else:
            self.chip_ctrl.set("Apagado", theme.NEUTRAL)
        if vis_run or self.vision_external:
            self.chip_vis.set("Lista" if ping else "Ocupada, sin PONG", theme.POSITIVE if ping else theme.WARNING)
        else:
            self.chip_vis.set("Apagada", theme.NEUTRAL)
        photo_ok = (vis_run or self.vision_external) and not live_ok
        why = "" if photo_ok else ("Detén el controlador para tomar fotos de prueba." if live_ok
                                   else "Inicia el servidor de visión en 'Sistema'.")
        self.visionp.set_photo_allowed(photo_ok, why)
        if not live_ok and self.live.mode == "live":
            self.chip_control.set("—", theme.NEUTRAL)
            self.chip_seq.set("—", theme.NEUTRAL)

    def _update_header_from_state(self, st):
        c = st.get("control")
        if st.get("source") == "replay":
            self.chip_control.set("Repetición", theme.NEUTRAL)
        elif c == "ON":
            self.chip_control.set("ON, impedancia", theme.POSITIVE)
        elif c == "RAMPA":
            self.chip_control.set("Entrando…", theme.WARNING)
        elif c == "OFF":
            self.chip_control.set("OFF, gravedad", theme.NEUTRAL)
        n, tot = st.get("step_n"), st.get("step_total") or len(self.info.get("steps", []))
        status = st.get("task_status")
        names = {"waiting": "espera", "running": "ejecuta", "paused": "pausa", "done": "terminada", "aborted": "abortada"}
        cols = {"waiting": theme.WARNING, "running": theme.ACCENT, "paused": theme.WARNING, "done": theme.POSITIVE,
                "aborted": theme.NEGATIVE}
        if n:
            tot_txt = f" de {tot}" if tot and n <= tot else ""
            self.chip_seq.set(f"Paso {n}{tot_txt}, {names.get(status, status or '')}", cols.get(status, theme.NEUTRAL))

    def _tick_clock(self):
        self.clock.setText(time.strftime("%H:%M:%S"))

    # ------------------------------------------------------------ presentacion
    def _go_page(self, i):
        if self._presenting:
            self._exit_present()
        self.nav.button(i).setChecked(True)
        self.stack.setCurrentIndex(i)

    def _toggle_present(self):
        (self._exit_present if self._presenting else self._enter_present)()

    def _enter_present(self):
        if self._presenting:
            return
        self._presenting = True
        self._was_fullscreen = self.isFullScreen()
        self._header.setVisible(False)
        self._side.setVisible(False)
        self._body.setContentsMargins(0, 0, 0, 0)
        self._sheet.setStyleSheet("#sheet { border: none; border-radius: 0; }")
        self.stack.setCurrentIndex(4)
        self._esc.setEnabled(True)
        self._mirror_present()
        if self.last_live and self.live.mode == "live":
            self.present.update_state(self.last_live)
        self.showFullScreen()
        self.present.setFocus()

    def _exit_present(self):
        if not self._presenting:
            return
        self._presenting = False
        self._esc.setEnabled(False)
        self._header.setVisible(True)
        self._side.setVisible(True)
        self._body.setContentsMargins(0, 0, 8, 8)
        self._sheet.setStyleSheet("")
        self.stack.setCurrentIndex(max(0, self.nav.checkedId()))
        if not self._was_fullscreen:
            self.showNormal()

    def _mirror_present(self):
        """La vista de presentacion refleja el boton de 'En vivo' (una sola logica)."""
        if not self._presenting:
            return
        lv = self.live
        self.present.mirror_button(lv.next_btn.text(), lv.next_btn.isEnabled(),
                                   lv.ack_note.text() if lv.ack_note.isVisibleTo(lv) else "",
                                   lv.pause_btn.isEnabled())

    def _open_latest_run(self):
        """'Ver en Analisis' del resumen final: la corrida mas nueva."""
        self.analysis.refresh_list()
        csvs = runs.list_csvs()
        if csvs:
            self.analysis.load(csvs[0])
        self._go_page(2)

    def _toggle_fullscreen(self):
        self.showNormal() if self.isFullScreen() else self.showFullScreen()

    def closeEvent(self, ev):
        running = [n for n, p in (("controlador", self.ctrl), ("visión", self.vision)) if p.running()]
        if running:
            r = QtWidgets.QMessageBox.question(
                self, "Cerrar interfaz",
                f"Siguen corriendo: {', '.join(running)}.\n\nAl cerrar la interfaz NO se detienen "
                "(el controlador conserva su ventana con ON/OFF y Quit).\n\n¿Cerrar la interfaz?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel, QtWidgets.QMessageBox.Cancel)
            if r != QtWidgets.QMessageBox.Yes:
                ev.ignore()
                return
        for v in (self.live.view3d, self.visionp.view3d, self.analysis.view3d, self.present.view3d):
            if v.vtk_widget is not None:
                v.vtk_widget.Finalize()
        ev.accept()


def run_label(path):
    """impedance_shadow_data_2026-10-16_16-10-39.csv -> '16/10 16:10:39'."""
    m = re.search(r"(\d{4})-(\d{2})-(\d{2})_(\d{2})-(\d{2})-(\d{2})", os.path.basename(path))
    return f"{m.group(3)}/{m.group(2)} {m.group(4)}:{m.group(5)}:{m.group(6)}" if m else os.path.basename(path)


class LogoMark(QtWidgets.QWidget):
    """Marca: un brazo de tres eslabones en blanco sobre el acento."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(32, 32)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(theme.ACCENT))
        p.drawRoundedRect(QtCore.QRectF(0, 0, 32, 32), 8, 8)
        p.setPen(QtGui.QPen(QtGui.QColor(theme.ON_PRIMARY), 2.6, cap=QtCore.Qt.RoundCap, join=QtCore.Qt.RoundJoin))
        path = QtGui.QPainterPath(QtCore.QPointF(9, 25))
        path.lineTo(11, 14)
        path.lineTo(21, 9)
        path.lineTo(24, 17)
        p.drawPath(path)
        p.setBrush(QtGui.QColor(theme.ON_PRIMARY))
        p.setPen(QtCore.Qt.NoPen)
        for pt in ((11, 14), (21, 9)):
            p.drawEllipse(QtCore.QPointF(*pt), 2.4, 2.4)


def main():
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("MODULAR Gen3")
    theme.apply(app)
    w = MainWindow()
    w.show()
    # sin foco inicial: el primer control no debe aparecer resaltado al abrir
    QtCore.QTimer.singleShot(0, lambda: app.focusWidget() and app.focusWidget().clearFocus())
    return app.exec_()
