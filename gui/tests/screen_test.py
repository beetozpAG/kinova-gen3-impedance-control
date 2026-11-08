"""Prueba EN PANTALLA (abre la ventana ~10 s): captura con el 3D real.
   .venv/bin/python tests/screen_test.py OUT_DIR
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from PyQt5 import QtCore, QtWidgets
from modular_gui import app as appmod, theme, runs

out = sys.argv[1]
qapp = QtWidgets.QApplication(sys.argv)
theme.apply(qapp)
w = appmod.MainWindow()
w.resize(1680, 1000)
w.show()

def spin(ms):
    loop = QtCore.QEventLoop(); QtCore.QTimer.singleShot(ms, loop.quit); loop.exec_()

def shot(name):
    qapp.primaryScreen().grabWindow(w.winId()).save(os.path.join(out, f"screen_{name}.png"))

run = runs.Run(runs.list_csvs()[0]); run.vibration(); run.metrics = run.step_metrics()
w.analysis._loaded(run)
w._start_replay(run); w.replay.speed = 4.0; spin(3500)
w.live.cb_labels.setChecked(False); spin(400)
shot("vivo")
w.nav.button(1).click(); spin(300)
w.visionp.cap_list.setCurrentRow(0); spin(1200); shot("vision")
w.nav.button(2).click(); w.analysis.tabs.setCurrentIndex(1); spin(1200); shot("tray3d")
w._replay_pause()
w.close()
print("ok")
