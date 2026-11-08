"""Lectura de lo que imprimen el controlador y el servidor de vision.

Controlador (src/main_impedance.cpp, printDashboard): cada dashboard_period_s
borra la pantalla (ESC[2J ESC[H) e imprime el tablero completo, que termina
con la linea "JOINTS [deg]: ...". Lo que se imprime entre tableros (avisos,
'Siguiente paso', errores) son EVENTOS.

Vision (kinova_grasp_V2/src/screw_server.cpp + grasp_pipeline): por cada foto
"Saved screw_HHMMSS.pcd ...", los agarres ("Grasp 0  (score ...)", pre-grasp,
EE position, EE orientation, opening) y "Answer sent: SCREW n OK|FAIL ...".
"""

import re

import numpy as np

CLEAR = "\x1b[2J\x1b[H"
_ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
_KV = re.compile(r"^\s*([^:]+?)\s*:\s*(.*)$")
_FLOAT = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|nan|inf", re.IGNORECASE)

# Estado de la secuencia como lo imprime task::statusName
TASK_STATUS = {
    "ESPERANDO 'SIGUIENTE PASO'": "waiting",
    "EJECUTANDO": "running",
    "PAUSADA": "paused",
    "TERMINADA": "done",
    "ABORTADA": "aborted",
}
# task_status en el CSV (enum task::Status)
TASK_STATUS_CODE = {0: "waiting", 1: "running", 2: "paused", 3: "done", 4: "aborted"}


def strip_ansi(s):
    return _ANSI.sub("", s)


def floats(s):
    out = []
    for tok in _FLOAT.findall(s):
        try:
            out.append(float(tok))
        except ValueError:
            pass
    return out


def _vector_after(lines, header, n):
    """Primer renglon con n numeros despues del renglon que empieza con header."""
    for i, line in enumerate(lines):
        if line.strip().startswith(header):
            for nxt in lines[i + 1:i + 4]:
                v = floats(nxt)
                if len(v) == n and "," in nxt:
                    return np.array(v)
            return None
    return None


def parse_dashboard(block):
    """Un tablero completo -> dict de estado (solo las llaves que encuentre)."""
    lines = strip_ansi(block).splitlines()
    st = {"source": "live"}
    kv = {}
    for line in lines:
        m = _KV.match(line)
        if m:
            kv.setdefault(m.group(1).strip(), m.group(2).strip())

    if "Time" in kv:
        v = floats(kv["Time"])
        st["t"] = v[0] if v else None
    if "Hardware blend" in kv:
        v = floats(kv["Hardware blend"])
        if v:
            st["blend"] = v[0] / 100.0

    for key, header, n in [
        ("target", "REFERENCE TARGET", 7), ("pose", "POSE [x y z", 7),
        ("cart_vel", "CARTESIAN VELOCITY", 6), ("tau_task", "TAU TASK", 7),
        ("tau_null", "TAU NULLSPACE", 7), ("tau_imp", "TAU IMPEDANCE TOTAL", 7),
        ("tau_friction", "FRICTION COMPENSATION", 7), ("tau_gravity", "GRAVITY TORQUE", 7),
        ("tau_virtual", "TOTAL TORQUE - VIRTUAL", 7), ("tau_hw", "TORQUE ACTUALLY USED", 7),
        ("current_cmd", "CURRENT ACTUALLY SENT", 7), ("current_meas", "MEASURED CURRENT", 7),
    ]:
        v = _vector_after(lines, header, n)
        if v is not None:
            st[key] = v

    for line in lines:
        if line.startswith("DELTA XI"):
            v = floats(line.split(":", 1)[1])
            if len(v) == 6:
                st["pose_error"] = np.array(v)
        elif line.startswith("JOINTS [deg]:"):
            v = floats(line.split(":", 1)[1])
            if len(v) == 7:
                st["joints_deg"] = np.array(v)

    health = {}
    for k in ("Finite values", "Pose error", "Cartesian velocity", "Impedance torque",
              "Virtual torque limit", "Virtual current limit"):
        if k in kv:
            health[k] = kv[k]
    if health:
        st["health"] = health

    # ---- secuencia (task::Sequencer::statusText + taskStatusText) ----
    if "Estado" in kv:
        st["task_status"] = TASK_STATUS.get(kv["Estado"], kv["Estado"].lower())
    if "Paso" in kv:
        m = re.match(r"(\d+)/(\d+)\s*(.*)", kv["Paso"])
        if m:
            st["step_n"], st["step_total"] = int(m.group(1)), int(m.group(2))
            st["step_desc"] = m.group(3).strip()
    if "Tiempo en el paso" in kv:
        v = floats(kv["Tiempo en el paso"])
        st["step_time"] = v[0] if v else None
    if "Error de posicion" in kv:
        v = floats(kv["Error de posicion"])
        if v:
            st["err_pos_cm"] = v[0]
            if len(v) > 1:
                st["tol_pos_cm"] = v[1]
    if "Error de orientacion" in kv:
        v = floats(kv["Error de orientacion"])
        if v:
            st["err_ori_deg"] = v[0]
    if "Mensaje" in kv:
        st["message"] = kv["Mensaje"]
    st["frozen"] = any("congelada" in line for line in lines)

    if "Gripper" in kv:
        g = kv["Gripper"]
        if "NOT DETECTED" in g:
            st["gripper_detected"] = False
        else:
            v = floats(g)
            st["gripper_detected"] = True
            if v:
                st["gripper_pct"] = v[0]
            if len(v) > 1:
                st["gripper_cmd"] = v[1]
    if "Fixture" in kv:
        f = kv["Fixture"]
        st["fixture_text"] = f
        mq = re.search(r"q\s*=\s*([-+\d.eE]+)", f)
        mt = re.search(r"target\s+([-+\d.eE]+)", f)
        if mq:
            st["fixture_q"] = float(mq.group(1))
        if mt:
            st["fixture_target"] = float(mt.group(1))
        st["fixture_arrived"] = "(arrived)" in f
    if "Fixture fault" in kv:
        st["fixture_fault"] = kv["Fixture fault"]

    for key, label in [("vision_tcp", "Vision (screw TCP) [m]"),
                       ("vision_rpy", "Vision Rx Ry Rz [deg]"),
                       ("vision_pre", "Vision pre-grasp [m]")]:
        if label in kv:
            v = floats(kv[label])
            if len(v) == 3:
                st[key] = np.array(v)
    if "Vision last reply" in kv:
        st["vision_last"] = kv["Vision last reply"]
    if "Modo" in kv:
        # AUTOMATICO o PASO A PASO (controlador del 2026-10-21 en adelante)
        st["task_auto"] = kv["Modo"].strip().upper().startswith("AUTOMATICO")
    if "Placa en la pinza" in kv:
        st["plate_correction"] = kv["Placa en la pinza"]
    if "Vision approach" in kv:
        # "#N MOVE  EE x y z  objeto cx cy cz (n puntos)"
        txt = kv["Vision approach"]
        m = re.match(r"#(\d+)\s+(MOVE|INVIEW|ATTARGET)\s+EE\s+(.*?)\s+objeto\s+(.*?)\s*\((\d+) puntos\)", txt)
        if m:
            st["approach_state"] = m.group(2)
            ee, obj = floats(m.group(3)), floats(m.group(4))
            if len(ee) == 3 and len(obj) == 3:
                st["approach_ee"] = np.array(ee)
                st["approach_object"] = np.array(obj)
    if "Hoyo de la vision [m]" in kv:
        # "x y z  (diametro D mm)"
        v = floats(kv["Hoyo de la vision [m]"].replace("(", " ").replace(")", " "))
        if len(v) >= 4:
            st["vision_hole"] = np.array(v[:3])
            st["vision_hole_diam_mm"] = v[3]

    # Control: la mezcla llega a 100 % con el slider ON
    b = st.get("blend")
    if b is not None:
        st["control"] = "ON" if b >= 0.999 else ("OFF" if b <= 0.001 else "RAMPA")

    # Referencia de la rampa = pose - pose_error (pose_error = p - p_d)
    if "pose" in st and "pose_error" in st:
        st["ramp"] = st["pose"][:3] - st["pose_error"][:3]
    return st


class DashboardStream:
    """Separa el texto del controlador en tableros (estado) y eventos."""

    JOINTS = "JOINTS [deg]:"

    def __init__(self):
        self.buf = ""
        self.last = None      # ultimo estado completo
        self.events = []      # renglones nuevos fuera del tablero

    def feed(self, text):
        """Agrega texto; regresa el estado mas nuevo (o None)."""
        self.buf += text
        newest = None
        while True:
            if not self.buf.startswith(CLEAR):
                i = self.buf.find(CLEAR)
                if i < 0:
                    # Solo eventos: los renglones completos
                    done, sep, rest = self.buf.rpartition("\n")
                    if sep:
                        self._add_events(done)
                        self.buf = rest
                    break
                self._add_events(self.buf[:i])
                self.buf = self.buf[i:]
                continue
            j = self.buf.find(self.JOINTS)
            nxt = self.buf.find(CLEAR, len(CLEAR))
            if j >= 0 and (nxt < 0 or j < nxt):
                end = self.buf.find("\n", j)
                if end < 0:
                    break   # falta el final del renglon JOINTS
                st = parse_dashboard(self.buf[len(CLEAR):end + 1])
                self.buf = self.buf[end + 1:]
            elif nxt >= 0:
                # Tablero sin JOINTS (controlador sin ese renglon)
                st = parse_dashboard(self.buf[len(CLEAR):nxt])
                self.buf = self.buf[nxt:]
            else:
                break   # tablero incompleto: esperar
            if "pose" in st:
                self.last = newest = st
        if len(self.buf) > 2_000_000:
            self.buf = self.buf[-200_000:]
        return newest

    def _add_events(self, text):
        for line in strip_ansi(text).splitlines():
            line = line.rstrip()
            if line.strip():
                self.events.append(line)

    def take_events(self):
        ev, self.events = self.events, []
        return ev


# ---------------------------------------------------------------- vision

def parse_vision_log(text):
    """Log de screw_server -> lista de capturas (dict) en orden."""
    caps = []
    cur = None
    grasp_idx = None
    for raw in strip_ansi(text).splitlines():
        line = raw.strip()
        m = re.match(r"Saved (screw_\d+)\.pcd", line)
        if m:
            cur = {"name": m.group(1), "grasps": []}
            caps.append(cur)
            grasp_idx = None
            continue
        if cur is None:
            continue
        m = re.match(r"Point cloud:\s*(\d+) points", line)
        if m:
            cur["points"] = int(m.group(1))
        m = re.match(r"Grasp (\d+)\s+\(score ([-+\d.eE]+)\)", line)
        if m:
            grasp_idx = int(m.group(1))
            cur["grasps"].append({"score": float(m.group(2))})
            continue
        if grasp_idx is not None and cur["grasps"]:
            g = cur["grasps"][-1]
            if line.startswith("gripper opening:"):
                v = floats(line)
                if v:
                    g["opening_m"] = v[0]
            elif line.startswith("1. pre-grasp"):
                v = floats(line.split(":", 1)[1])
                if len(v) >= 3:
                    g["pregrasp"] = np.array(v[:3])
            elif line.startswith("2. EE position"):
                v = floats(line.split(":", 1)[1])
                if len(v) >= 3:
                    g["position"] = np.array(v[:3])
            elif line.startswith("EE orientation"):
                v = floats(line.split(":", 1)[1])
                if len(v) >= 3:
                    g["rpy"] = np.array(v[:3])
        m = re.match(r"Answer sent: SCREW (\d+) (OK|FAIL)\s*(.*)", line)
        if m:
            cur["seq"] = int(m.group(1))
            cur["ok"] = m.group(2) == "OK"
            cur["answer"] = m.group(3)
            cur = None
    return caps


def parse_vision_test(text):
    """Salida de build/vision_request_test -> agarre (dict) o {'fail': motivo}."""
    g = {}
    for raw in strip_ansi(text).splitlines():
        line = raw.strip()
        if line.startswith("VISION FALLO:"):
            return {"fail": line.split(":", 1)[1].strip()}
        if line.startswith("Tornillo (TCP del mejor agarre"):
            v = floats(line.split(":", 1)[1])
            if len(v) >= 5:
                g["position"] = np.array(v[:3])
                g["score"] = v[3]
        elif line.startswith("Pre-grasp:"):
            v = floats(line.split(":", 1)[1])
            if len(v) >= 3:
                g["pregrasp"] = np.array(v[:3])
        elif line.startswith("Orientacion"):
            v = floats(line.split(":", 1)[1])
            if len(v) >= 3:
                g["rpy"] = np.array(v[:3])
    return g if "position" in g else None


def read_tool_pose(pose_file):
    """screw_HHMMSS_pose.txt -> [x y z tx ty tz] (TCP en la base, m y grados)."""
    try:
        for line in open(pose_file):
            if line.startswith("tool_pose"):
                v = floats(line.split("=", 1)[1])
                if len(v) == 6:
                    return np.array(v)
    except OSError:
        pass
    return None


def read_pcd(path):
    """PCD binario 'FIELDS x y z' (lo que guarda kinova_grasp) -> Nx3 float32."""
    with open(path, "rb") as f:
        data = f.read()
    key = b"DATA binary\n"
    i = data.find(key)
    if i < 0:
        key = b"DATA ascii\n"
        i = data.find(key)
        if i < 0:
            raise ValueError("PCD no soportado")
        hdr = data[:i].decode()
        pts = np.loadtxt(data[i + len(key):].decode().splitlines())
        return pts.reshape(-1, 3).astype(np.float32)
    hdr = data[:i].decode()
    n = int([ln for ln in hdr.splitlines() if ln.startswith("POINTS")][0].split()[1])
    if "FIELDS x y z\n" not in hdr:
        raise ValueError("PCD sin 'FIELDS x y z'")
    a = np.frombuffer(data[i + len(key):i + len(key) + n * 12], dtype=np.float32)
    return a.reshape(-1, 3)
