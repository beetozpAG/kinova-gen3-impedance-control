"""Procedimiento de Antony a mano (kinova_grasp_NEW_VER), SIN el controlador:

  1. Pose inicial: el robot movido con la app web, la camara ve el objeto.
  2. ./approach_object COLOR -> EE target. MOVE: mandar esa pose al robot
     (app web), esperar, FOCUS NOW y otra vez. IN VIEW: paso 3.
  3. ./grasps_from_camera CFG --color COLOR --filter
       placa: ../cfg/gpd_plate_params.cfg   perno: ../cfg/gpd_params.cfg  (color: azul o rojo)

Corre esos mismos programas (compilados en kinova_grasp_NEW_VER/build_modular)
y muestra MOVE / IN VIEW y las poses. Con el controlador corriendo NO: el robot
deja de actualizar su pose de herramienta en servocontrol de bajo nivel y estos
programas la leen del robot; ahi lo hace la secuencia (pasos 'centrar la
camara' + foto con color), con la pose que manda el controlador.
"""

import re

from PyQt5 import QtCore, QtWidgets

from . import config, theme, widgets as W

# (texto, configuracion de GPD, clave del ultimo color elegido al iniciar la secuencia)
OBJECTS = [
    ("Placa", "../cfg/gpd_plate_params.cfg", "plate_color"),
    ("Perno", "../cfg/gpd_params.cfg", "bolt_color"),
]
_NUM = r"(-?\d+(?:\.\d+)?)"
RE_TARGET = re.compile(rf"EE target:\s*x = {_NUM}\s*y = {_NUM}\s*z = {_NUM}.*?tx = {_NUM}\s*ty = {_NUM}\s*tz = {_NUM}")
RE_POS = re.compile(rf"2\. EE position \(m\): x = {_NUM}\s+y = {_NUM}\s+z = {_NUM}")
RE_PRE = re.compile(rf"1\. pre-grasp \(m\):\s+x = {_NUM}\s+y = {_NUM}\s+z = {_NUM}")
RE_ORI = re.compile(rf"EE orientation \(deg\): tx = {_NUM}\s+ty = {_NUM}\s+tz = {_NUM}")


class ManualVisionDialog(QtWidgets.QDialog):
    def __init__(self, controller_running, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Procedimiento de Antony a mano")
        self.resize(760, 640)
        self.controller_running = controller_running
        self.proc = None
        self.buf = []
        self.kind = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(24, 20, 24, 20)
        lay.setSpacing(12)
        lay.addWidget(W.label("Procedimiento de Antony a mano", obj="pageTitle"))
        lay.addWidget(W.label(
            "Sin el controlador: mueve el robot con la app web a una pose donde la cámara vea el objeto. "
            "<b>1.</b> Acercarse: si dice <b>MOVE</b>, manda la pose <i>EE target</i> al robot (app web), "
            "espera a que se detenga, presiona <b>FOCUS NOW</b> y repite. Si dice <b>IN VIEW</b>, pasa al 2. "
            "<b>2.</b> Calcular el agarre (con color y filtro). Con el controlador corriendo esto lo hace la "
            "secuencia sola (pasos <i>centrar la cámara</i>).", color=theme.MUTED, wrap=True))

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(W.label("Objeto", color=theme.MUTED))
        self.obj = QtWidgets.QComboBox()
        for text, _cfg, _key in OBJECTS:
            self.obj.addItem(text)
        row.addWidget(self.obj)
        row.addWidget(W.label("Color", color=theme.MUTED))
        self.color = QtWidgets.QComboBox()
        for c in config.OBJECT_COLORS:
            if c in config.ENABLED_COLORS:
                self.color.addItem(config.COLOR_NAMES.get(c, c), c)
        row.addWidget(self.color)
        self.obj.currentIndexChanged.connect(self._object_changed)
        self._object_changed(0)
        self.b_approach = QtWidgets.QPushButton("1. Acercarse  (approach_object)")
        self.b_approach.setObjectName("primary")
        self.b_approach.clicked.connect(self._approach)
        self.b_grasp = QtWidgets.QPushButton("2. Calcular agarre  (grasps_from_camera)")
        self.b_grasp.clicked.connect(self._grasp)
        row.addWidget(self.b_approach)
        row.addWidget(self.b_grasp)
        row.addStretch(1)
        lay.addLayout(row)
        self.cmd_lbl = W.label("", obj="cardMeta", mono=True, wrap=True)
        lay.addWidget(self.cmd_lbl)
        self.state = W.label("", size=22, weight=600, wrap=True)
        lay.addWidget(self.state)
        self.pose = W.label("", mono=True, wrap=True)
        self.pose.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        lay.addWidget(self.pose)
        self.log = W.LogView(3000)
        lay.addWidget(self.log, 1)
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._update_enabled)
        self._timer.start(500)
        self._update_enabled()

    # ------------------------------------------------------------ estado
    def _blocked(self):
        if config.VISION_VERSION != "new":
            return "Solo con la versión nueva (MODULAR_VISION=v2 está activo)."
        if self.controller_running():
            return ("El controlador está corriendo: el robot no actualiza su pose de herramienta y estos "
                    "programas la leen de ahí. Usa la secuencia (pasos 'centrar la cámara').")
        if self.proc is not None:
            return "Corriendo…"
        return ""

    def _update_enabled(self):
        why = self._blocked()
        for b in (self.b_approach, self.b_grasp, self.obj, self.color):
            b.setEnabled(not why)
        if why and self.proc is None:
            self.cmd_lbl.setText(why)

    # ------------------------------------------------------------ comandos
    def _run(self, kind, program, args):
        self.kind = kind
        self.buf = []
        self.state.setText("Corriendo…")
        self.state.setStyleSheet(f"color: {theme.MUTED};")
        self.pose.setText("")
        self.log.clear()
        self.cmd_lbl.setText(f"$ cd {config.VISION_BUILD_DIR}\n$ ./{program} " + " ".join(args))
        self.proc = QtCore.QProcess(self)
        self.proc.setWorkingDirectory(str(config.VISION_BUILD_DIR))
        self.proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._read)
        self.proc.finished.connect(self._done)
        self.proc.start(str(config.VISION_BUILD_DIR / program), args)
        self._update_enabled()

    def _object_changed(self, i):
        """Al cambiar de objeto pone el ultimo color elegido para el."""
        _t, _cfg, key = OBJECTS[i]
        default = config.PLATE_COLOR if key == "plate_color" else config.BOLT_COLOR
        want = str(QtCore.QSettings("MODULAR", "gui").value(key, default))
        j = self.color.findData(want)
        self.color.setCurrentIndex(j if j >= 0 else max(0, self.color.findData(default)))

    def _approach(self):
        self._run("approach", "approach_object", [self.color.currentData()])

    def _grasp(self):
        _t, cfg, _key = OBJECTS[self.obj.currentIndex()]
        self._run("grasp", "grasps_from_camera", [cfg, "--color", self.color.currentData(), "--filter"])

    def _read(self):
        txt = bytes(self.proc.readAllStandardOutput()).decode(errors="replace")
        self.buf.append(txt)
        self.log.append_lines(txt.rstrip("\n").splitlines())

    def _done(self, code, _status):
        text = "".join(self.buf)
        self.proc = None
        if self.kind == "approach":
            self._show_approach(text, code)
        else:
            self._show_grasp(text, code)
        self._update_enabled()

    def _show_approach(self, text, code):
        m = RE_TARGET.search(text)
        if "IN VIEW" in text:
            st, col = "IN VIEW: el objeto se ve completo. Pasa al 2 (calcular el agarre).", theme.POSITIVE
        elif "AT TARGET" in text:
            st, col = "En la pose, pero el objeto toca el borde de la imagen (más grande que la imagen).", theme.WARNING
        elif "MOVE" in text:
            st, col = ("MOVE: manda esta pose al robot (app web), espera a que se detenga, FOCUS NOW y "
                       "vuelve a presionar 1."), theme.ACCENT
        else:
            st, col = f"No terminó (código {code}). Revisa el log.", theme.NEGATIVE
        self.state.setText(st)
        self.state.setStyleSheet(f"color: {col};")
        if m:
            x, y, z, tx, ty, tz = (float(v) for v in m.groups())
            self.pose.setText(f"EE target:  x {x:.4f}  y {y:.4f}  z {z:.4f} m    "
                              f"tx {tx:.1f}  ty {ty:.1f}  tz {tz:.1f} °")

    def _show_grasp(self, text, code):
        pos, pre, ori = RE_POS.search(text), RE_PRE.search(text), RE_ORI.search(text)
        if not pos:
            self.state.setText(f"Sin agarre (código {code}). Revisa el log.")
            self.state.setStyleSheet(f"color: {theme.NEGATIVE};")
            return
        self.state.setText("Agarre 0 (el mejor):")
        self.state.setStyleSheet(f"color: {theme.POSITIVE};")
        lines = []
        if pre:
            lines.append("1. pre-grasp:  x {:.4f}  y {:.4f}  z {:.4f} m".format(*(float(v) for v in pre.groups())))
        lines.append("2. agarre:     x {:.4f}  y {:.4f}  z {:.4f} m".format(*(float(v) for v in pos.groups())))
        if ori:
            lines.append("   orientación: tx {:.1f}  ty {:.1f}  tz {:.1f} °".format(*(float(v) for v in ori.groups())))
        self.pose.setText("\n".join(lines))
