"""Corridas guardadas (CSV de impedance_gen3): carga, metricas y repeticion.

El CSV trae 1 renglon por ciclo (1 kHz). La referencia de la rampa no esta
guardada: se calcula como pose - pose_err (pose_err = p - p_d).
"""

import glob
import os

import numpy as np
from PyQt5 import QtCore

from . import config
from .parsers import TASK_STATUS_CODE

class NewTestWatch:
    """Dice cuando empieza una prueba nueva, para borrar la estela del TCP:
    el primer paso pasa de esperando a ejecutando (se inicio la secuencia) o
    el numero de paso regresa (otra corrida: el controlador se reinicio)."""

    def __init__(self):
        self.n = self.status = None

    def update(self, n, status, first_step):
        new = (n is not None and self.n is not None and n < self.n) or \
              (n == first_step and status == "running" and self.status == "waiting" and self.n == n)
        if n is not None:
            self.n, self.status = n, status
        return new


_VECS = {"q": 7, "dq": 7, "pose": 7, "pose_err": 6, "ref_target": 7, "tau_hw": 7,
         "tau_task": 7, "tau_friction": 7, "tau_null": 7, "cart_vel": 6}
_SCALARS = ["t", "task_step", "task_status", "gripper_pos", "fixture_q"]


def list_csvs():
    files = glob.glob(str(config.BUILD_DIR / "impedance_shadow_data_*.csv"))
    files = [f for f in files if os.path.getsize(f) > 10000]
    return sorted(files, key=os.path.getmtime, reverse=True)


class Run:
    """Una corrida cargada en memoria (numpy)."""

    def __init__(self, path):
        self.path = path
        with open(path) as f:
            header = f.readline().strip().split(",")
        ix = {n: i for i, n in enumerate(header)}
        cols, names = [], []
        for name in _SCALARS:
            if name in ix:
                cols.append(ix[name])
                names.append((name, None))
        for name, n in _VECS.items():
            if f"{name}[0][0]" in ix:
                for j in range(n):
                    cols.append(ix[f"{name}[{j}][0]"])
                    names.append((name, j))
        data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=cols, ndmin=2)
        data = data[np.isfinite(data).all(1)]
        self.n = len(data)
        self.cols = {}
        for k, (name, j) in enumerate(names):
            if j is None:
                self.cols[name] = data[:, k]
            else:
                self.cols.setdefault(name, np.zeros((self.n, _VECS[name])))[:, j] = data[:, k]
        c = self.cols
        self.t = c["t"] - c["t"][0]
        self.pos = c["pose"][:, :3]
        self.quat = c["pose"][:, 3:7]
        self.err = c["pose_err"]
        self.ramp = self.pos - self.err[:, :3]
        self.target = c["ref_target"][:, :3]
        self.target_quat = c["ref_target"][:, 3:7]
        self.step = c.get("task_step", np.zeros(self.n)).astype(int)
        self.status = c.get("task_status", np.zeros(self.n)).astype(int)
        self.joints_deg = np.degrees(c["q"]) if "q" in c else None
        self.tau_hw = c.get("tau_hw")
        self.gripper = c.get("gripper_pos")
        self.fixture = c.get("fixture_q")
        self._vib = None
        # Indice donde empezo el tramo (paso, estado) de cada renglon
        key = self.step * 10 + self.status
        change = np.r_[True, key[1:] != key[:-1]] if self.n else np.array([], bool)
        self.span_start = np.maximum.accumulate(np.where(change, np.arange(self.n), 0)) if self.n else change

    @property
    def duration(self):
        return float(self.t[-1]) if self.n else 0.0

    def vibration(self):
        """Parte rapida (~>20 Hz) del torque mandado J1-J4, RMS por muestra [Nm]."""
        if self._vib is None and self.tau_hw is not None:
            k = np.ones(50) / 50.0
            hp = [self.tau_hw[:, j] - np.convolve(self.tau_hw[:, j], k, "same") for j in range(4)]
            self._vib = np.sqrt(sum(h ** 2 for h in hp))
        return self._vib

    def vibration_windowed(self, window_s=0.2):
        v = self.vibration()
        if v is None:
            return None
        n = max(1, int(window_s * 1000))
        return np.sqrt(np.convolve(v ** 2, np.ones(n) / n, "same"))

    def step_metrics(self):
        """Por cada paso MOVER: lo mismo que tools/analizar_csv.py."""
        out = []
        vib = self.vibration()
        dt = np.diff(self.t, prepend=self.t[0] - 0.001)
        dt[dt <= 0] = 0.001
        ref_speed = np.r_[0.0, np.linalg.norm(np.diff(self.ramp, axis=0), axis=1)] / dt
        idx_all = np.arange(self.n)
        for k in sorted(set(self.step.tolist())):
            idx = np.where((self.step == k) & (self.status == 1))[0]
            if len(idx) < 100:
                continue
            i0, i1 = idx[0], idx[-1]
            start, goal = self.pos[i0], self.target[i1]
            length = float(np.linalg.norm(goal - start))
            if length < 0.01:
                continue
            u = (goal - start) / length
            moving = idx[ref_speed[idx] > 0.005]
            still = idx[ref_speed[idx] <= 0.001]
            lag = float(np.mean((self.ramp[moving] - self.pos[moving]) @ u)) if len(moving) > 100 else np.nan
            if len(moving) <= 100:
                moving = np.array([], dtype=int)
            nxt = np.where((idx_all > i1) & (self.status == 1) & (self.step != k))[0]
            end = min(self.n - 1, i1 + 3000, nxt[0] if len(nxt) else self.n - 1)
            seg = self.pos[i0:end] - start
            lateral = float(np.max(np.linalg.norm(seg - np.outer(seg @ u, u), axis=1)))
            overshoot = max(0.0, float(np.max((self.pos[i0:end] - goal) @ u)))
            rms = lambda ii: float(np.sqrt(np.mean(vib[ii] ** 2))) if vib is not None and len(ii) > 100 else np.nan
            out.append({
                "step": int(k), "dist_cm": length * 100, "dur_s": float(self.t[i1] - self.t[i0]),
                "lag_cm": lag * 100, "lateral_cm": lateral * 100, "overshoot_mm": overshoot * 1000,
                "final_cm": float(np.linalg.norm(self.pos[i1] - goal)) * 100,
                "vib_move": rms(moving), "vib_still": rms(still),
                "t0": float(self.t[i0]), "t1": float(self.t[i1]),
            })
        return out

    def step_spans(self):
        """[(paso, estado, t0, t1)] tramos consecutivos."""
        spans = []
        if not self.n:
            return spans
        key = self.step * 10 + self.status
        cut = np.where(np.diff(key) != 0)[0]
        starts = np.r_[0, cut + 1]
        ends = np.r_[cut, self.n - 1]
        for a, b in zip(starts, ends):
            spans.append((int(self.step[a]), int(self.status[a]), float(self.t[a]), float(self.t[b])))
        return spans

    def state_at(self, i):
        """Estado tipo tablero en el renglon i (para repetir en 'En vivo')."""
        i = int(np.clip(i, 0, self.n - 1))
        st = {
            "source": "replay", "t": float(self.t[i]), "control": "ON",
            "pose": np.r_[self.pos[i], self.quat[i]], "target": np.r_[self.target[i], self.target_quat[i]],
            "ramp": self.ramp[i], "pose_error": self.err[i],
            "step_n": int(self.step[i]), "task_status": TASK_STATUS_CODE.get(int(self.status[i]), "?"),
            "err_pos_cm": float(np.linalg.norm(self.pos[i] - self.target[i]) * 100),
        }
        q1, q2 = self.quat[i], self.target_quat[i]
        d = abs(float(np.dot(q1, q2)) / max(np.linalg.norm(q1) * np.linalg.norm(q2), 1e-9))
        st["err_ori_deg"] = float(np.degrees(2 * np.arccos(min(1.0, d))))
        if self.joints_deg is not None:
            st["joints_deg"] = self.joints_deg[i]
        if self.tau_hw is not None:
            st["tau_hw"] = self.tau_hw[i]
        if self.gripper is not None:
            st["gripper_pct"] = float(self.gripper[i])
            st["gripper_detected"] = True
        if self.fixture is not None and np.isfinite(self.fixture[i]):
            st["fixture_q"] = float(self.fixture[i])
        st["step_time"] = float(self.t[i] - self.t[self.span_start[i]])
        return st


class RunLoader(QtCore.QThread):
    """Carga un CSV sin congelar la interfaz."""

    loaded = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, path, parent=None):
        super().__init__(parent)
        self.path = path

    def run(self):
        try:
            run = Run(self.path)
            run.vibration()
            run.metrics = run.step_metrics()
            self.loaded.emit(run)
        except Exception as e:   # noqa: BLE001 (mostrar cualquier error al usuario)
            self.failed.emit(f"{os.path.basename(self.path)}: {e}")


class Replay(QtCore.QObject):
    """Repite una corrida a velocidad real (o x2, x5...) emitiendo estados."""

    state = QtCore.pyqtSignal(dict)
    position = QtCore.pyqtSignal(float, float)   # t, duracion
    finished = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.run = None
        self.speed = 1.0
        self.t = 0.0
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._last = None

    def load(self, run):
        self.run = run
        self.t = 0.0

    def playing(self):
        return self._timer.isActive()

    def play(self):
        if self.run is None:
            return
        if self.t >= self.run.duration:
            self.t = 0.0
        self._last = QtCore.QElapsedTimer()
        self._last.start()
        self._timer.start(40)

    def pause(self):
        self._timer.stop()

    def seek(self, t):
        if self.run is None:
            return
        self.t = float(np.clip(t, 0, self.run.duration))
        self._emit()

    def _tick(self):
        dt = self._last.restart() / 1000.0
        self.t += dt * self.speed
        if self.t >= self.run.duration:
            self.t = self.run.duration
            self._emit()
            self._timer.stop()
            self.finished.emit()
            return
        self._emit()

    def _emit(self):
        i = int(np.searchsorted(self.run.t, self.t))
        self.state.emit(self.run.state_at(i))
        self.position.emit(self.t, self.run.duration)
