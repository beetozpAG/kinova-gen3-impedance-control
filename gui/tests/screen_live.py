"""EN PANTALLA (~15 s): controlador falso moviendo el robot 3D.
   MODULAR_GUI_FAKE_CONTROLLER=1 .venv/bin/python tests/screen_live.py OUT_DIR"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from PyQt5 import QtCore, QtWidgets
from modular_gui import app as appmod, theme, runs
out = sys.argv[1]
qapp = QtWidgets.QApplication(sys.argv); theme.apply(qapp)
QtWidgets.QMessageBox.question = staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Yes)
w = appmod.MainWindow(); w.resize(1680, 1000); w.show()
def spin(ms):
    loop = QtCore.QEventLoop(); QtCore.QTimer.singleShot(ms, loop.quit); loop.exec_()
w._start_controller(); spin(1500)
for _ in range(4):           # fixture, HOME, pinza, NEW_OFFSET_GENERAL
    w.live.next_btn.click(); spin(2600)
qapp.primaryScreen().grabWindow(w.winId()).save(os.path.join(out, "screen_live.png"))
run = runs.Run(runs.list_csvs()[0]); run.vibration(); run.metrics = run.step_metrics()
w.analysis._loaded(run); w.nav.button(2).click(); w.analysis.tabs.setCurrentIndex(1); spin(1200)
qapp.primaryScreen().grabWindow(w.winId()).save(os.path.join(out, "screen_tray3d.png"))
w.ctrl.interrupt(); spin(800); w.close(); print("ok")
