"""Capturas de las paginas fuera de pantalla (sin abrir ventanas).
   QT_QPA_PLATFORM=offscreen MODULAR_GUI_NO3D=1 .venv/bin/python tests/screenshots.py OUT_DIR
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from PyQt5 import QtCore, QtWidgets
from modular_gui import app as appmod, theme, runs, parsers

out = sys.argv[1]
qapp = QtWidgets.QApplication(sys.argv)
theme.apply(qapp)
w = appmod.MainWindow()
w.resize(1680, 1000)
w.show()

def spin(ms):
    loop = QtCore.QEventLoop(); QtCore.QTimer.singleShot(ms, loop.quit); loop.exec_()

csv = runs.list_csvs()[0]
run = runs.Run(csv); run.vibration(); run.metrics = run.step_metrics()
w.analysis._loaded(run)
spin(300)
# En vivo con una repeticion detenida a la mitad
w._start_replay(run); spin(1500); w._replay_pause(); w.replay.seek(run.duration * 0.30); spin(600)
for i, name in enumerate(["vivo", "vision", "analisis", "sistema"]):
    w.nav.button(i).click(); spin(700)
    w.grab().save(os.path.join(out, f"gui_{name}.png"))
# vista de presentacion (F5) con la repeticion
w._enter_present(); w.showNormal(); w.resize(1680, 1000); spin(800)
w.grab().save(os.path.join(out, "gui_presentacion.png"))
w._exit_present(); spin(300)
print("ok")
