"""Lanzar/parar programas y leer sus logs, sin acoplarlos a la interfaz.

Reglas de seguridad:
  - Cada programa corre en SU PROPIA sesion (start_new_session) y escribe a
    un ARCHIVO (no a una tuberia): si la interfaz se cierra o se traba, el
    programa no se bloquea ni se entera.
  - Parar = la misma senal que Ctrl+C (SIGINT): el controlador hace su
    transicion a gravedad y guarda el CSV como siempre.
  - Al cerrar la interfaz NO se mata nada.
"""

import os
import signal
import socket
import subprocess
import time
from pathlib import Path

from PyQt5 import QtCore

from . import config


class ManagedProcess(QtCore.QObject):
    """Un programa lanzado por la interfaz (o detectado corriendo afuera)."""

    changed = QtCore.pyqtSignal()

    def __init__(self, name, cmd, cwd, log_prefix, line_buffered=False, parent=None):
        super().__init__(parent)
        self.name = name
        self.cmd = [str(c) for c in cmd]
        self.cwd = str(cwd)
        self.log_prefix = log_prefix
        self.line_buffered = line_buffered
        self.proc = None
        self.log_path = None
        self.started_at = None
        self.exit_code = None
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(300)

    # ---- estado ----
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def pid(self):
        return self.proc.pid if self.proc else None

    def uptime(self):
        return time.time() - self.started_at if self.running() and self.started_at else 0.0

    def binary_exists(self):
        return Path(self.cmd[0]).exists()

    # ---- acciones ----
    def start(self):
        if self.running():
            return
        config.LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = config.LOG_DIR / f"{self.log_prefix}_{stamp}.log"
        cmd = self.cmd
        if self.line_buffered:
            # printf a un archivo va por bloques: stdbuf lo hace por lineas.
            cmd = ["stdbuf", "-oL", "-eL"] + cmd
        log = open(self.log_path, "wb")
        try:
            self.proc = subprocess.Popen(
                cmd, cwd=self.cwd, stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True)
        finally:
            log.close()   # el hijo tiene su propia copia del descriptor
        self.started_at = time.time()
        self.exit_code = None
        self.changed.emit()

    def interrupt(self):
        """Como Ctrl+C."""
        if self.running():
            self.proc.send_signal(signal.SIGINT)

    def kill(self):
        if self.running():
            self.proc.kill()

    def _poll(self):
        if self.proc is not None and self.exit_code is None:
            code = self.proc.poll()
            if code is not None:
                self.exit_code = code
                self.changed.emit()


class LogTail(QtCore.QObject):
    """Lee lo nuevo de un archivo de log cada 100 ms."""

    text = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.path = None
        self.pos = 0
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._read)
        self._timer.start(100)

    def follow(self, path, from_start=True):
        self.path = Path(path) if path else None
        self.pos = 0
        if self.path and not from_start and self.path.exists():
            self.pos = self.path.stat().st_size

    def _read(self):
        if not self.path or not self.path.exists():
            return
        size = self.path.stat().st_size
        if size < self.pos:
            self.pos = 0   # el archivo se trunco
        if size == self.pos:
            return
        with open(self.path, "rb") as f:
            f.seek(self.pos)
            data = f.read(min(size - self.pos, 4 * 1024 * 1024))
        self.pos += len(data)
        self.text.emit(data.decode("utf-8", errors="replace"))


def process_running(name):
    """True si hay un proceso con ese nombre (lanzado por quien sea)."""
    try:
        out = subprocess.run(["pgrep", "-x", name], capture_output=True, text=True, timeout=2)
        return out.returncode == 0
    except Exception:
        return False


def ping_vision(timeout_s=0.3):
    """PING al screw_server (no hace nada mas: no toma foto)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(timeout_s)
            s.sendto(b"PING", ("127.0.0.1", config.VISION_PORT))
            data = s.recv(64)
            return data.strip() == b"PONG"
    except Exception:
        return False


def send_colors(plate, bolt, timeout_s=2.0):
    """Colores de la placa y del perno al screw_server ("COLORS placa perno"),
    sin reiniciarlo. Devuelve (True, "") o (False, motivo)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(timeout_s)
            s.sendto(f"COLORS {plate} {bolt}".encode(), ("127.0.0.1", config.VISION_PORT))
            answer = s.recv(256).decode(errors="replace").strip()
    except OSError as e:
        return False, f"el servidor de visión no respondió ({e})"
    if answer == f"COLORS OK {plate} {bolt}":
        return True, ""
    return False, answer or "respuesta vacía"


def send_next_step():
    """'Siguiente paso' al controlador (igual que su boton GTK)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(b"NEXT", ("127.0.0.1", config.GUI_COMMAND_PORT))


def send_pause():
    """'Pausa' al controlador (igual que su boton GTK): el brazo se queda
    donde esta y 'Siguiente paso' reanuda el paso."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(b"PAUSE", ("127.0.0.1", config.GUI_COMMAND_PORT))


def send_mode(automatic):
    """Modo de la secuencia: AUTOMATICO ("AUTO") o PASO A PASO ("STEP").
    El controlador lo aplica desde el paso siguiente; no arranca nada."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(b"AUTO" if automatic else b"STEP", ("127.0.0.1", config.GUI_COMMAND_PORT))


def latest_log(prefix):
    files = sorted(config.LOG_DIR.glob(f"{prefix}_*.log"), key=os.path.getmtime)
    return files[-1] if files else None


def cleanup_logs(keep=15):
    for prefix in ("controlador", "vision", "compilar", "foto"):
        files = sorted(config.LOG_DIR.glob(f"{prefix}_*.log"), key=os.path.getmtime)
        for f in files[:-keep]:
            try:
                f.unlink()
            except OSError:
                pass
