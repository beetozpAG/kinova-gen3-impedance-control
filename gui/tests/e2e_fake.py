"""Prueba de punta a punta SIN robot: controlador falso + boton + GPD.
   MODULAR_GUI_FAKE_CONTROLLER=1 QT_QPA_PLATFORM=offscreen MODULAR_GUI_NO3D=1 \
       .venv/bin/python tests/e2e_fake.py OUT_DIR
"""
import os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from PyQt5 import QtCore, QtWidgets
from modular_gui import app as appmod, theme, config

assert config.FAKE_CONTROLLER, "correr con MODULAR_GUI_FAKE_CONTROLLER=1"
out = sys.argv[1]
qapp = QtWidgets.QApplication(sys.argv)
theme.apply(qapp)
QtWidgets.QMessageBox.question = staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Yes)
w = appmod.MainWindow(); w.resize(1680, 1000); w.show()
fails = 0
def spin(ms):
    loop = QtCore.QEventLoop(); QtCore.QTimer.singleShot(ms, loop.quit); loop.exec_()
def check(ok, what):
    global fails
    print(("  OK   " if ok else "  FALLA ") + what); fails += 0 if ok else 1

w._start_controller(); spin(2500)
st = w.last_live
check(st is not None and st.get("step_n") == w.info["first_step"], f"tablero leido (paso {st and st.get('step_n')})")
check(st is not None and st.get("joints_deg") is not None and st.get("task_status") == "waiting", "articulaciones y estado 'waiting'")
# AUTOMATICO por defecto: el boton inicia y la GUI lo muestra
check(st.get("task_auto") is True and w.live.mode_auto.isChecked() and w.live.mode_auto.isEnabled(),
      "modo AUTOMATICO por defecto (leido del controlador)")
check(w.live.next_btn.isEnabled() and "iniciar en automático" in w.live.next_btn.text().lower(),
      "boton: " + w.live.next_btn.text())
# cambiar a PASO A PASO desde la GUI
w.live.mode_step.click(); spin(900)
check(w.last_live.get("task_auto") is False and w.live.mode_step.isChecked() and w.live._mode_sent is None,
      "PASO A PASO: el controlador lo confirma")
check("GUI: modo PASO A PASO" in w.system.log_ctrl.toPlainText(), "cambio de modo en el log del controlador")
st = w.last_live
check(w.live.next_btn.isEnabled() and f"ejecutar paso {st['step_n']}" in w.live.next_btn.text().lower(),
      "boton habilitado y dice que paso ejecuta: " + w.live.next_btn.text())
n0 = st["step_n"]
w.live.next_btn.click(); spin(600)
check(w.last_live.get("task_status") == "running", "tras el boton: EJECUTANDO")
check(not w.live.next_btn.isEnabled(), "boton deshabilitado mientras ejecuta")
for _ in range(60):
    spin(250)
    if w.last_live.get("step_n", 0) > n0: break
check(w.last_live.get("step_n", 0) == n0 + 1, f"avanzo al paso {w.last_live.get('step_n')}")
check(not w.live.ack_note.isVisibleTo(w.live), "sin aviso de 'no respondio' (el controlador tomo el NEXT)")
# vista de presentacion: refleja el mismo boton y sale con Esc
w._enter_present(); spin(600)
check(w.stack.currentIndex() == 4 and w.present.next_btn.text() == w.live.next_btn.text(),
      "presentacion refleja el boton: " + w.present.next_btn.text())
w.grab().save(os.path.join(out, "e2e_presentacion.png"))
w._exit_present(); spin(300)
check(w.stack.currentIndex() != 4 and w._header.isVisible(), "salio de la presentacion")
# de vuelta a AUTOMATICO: el siguiente boton sigue solo varios pasos
w.live.mode_auto.click(); spin(900)
check(w.last_live.get("task_auto") is True and w.live.mode_auto.isChecked(), "AUTOMATICO otra vez")
n1 = w.last_live.get("step_n", 0)
w.live.next_btn.click()
for _ in range(80):
    spin(250)
    if w.last_live.get("step_n", 0) >= n1 + 2: break
check(w.last_live.get("step_n", 0) >= n1 + 2 and "en automático" in w.live.next_btn.text().lower(),
      f"en AUTOMATICO avanza solo del paso {n1} al {w.last_live.get('step_n')} con un boton")
# Pausa en automatico: se queda en el paso; 'Continuar' reanuda ese paso
for _ in range(20):
    if w.last_live.get("task_status") == "running": break
    spin(100)
check(w.live.pause_btn.isEnabled(), "boton Pausa habilitado mientras ejecuta")
n2 = w.last_live.get("step_n")
w.live.pause_btn.click(); spin(900)
check(w.last_live.get("task_status") == "paused" and w.last_live.get("step_n") == n2
      and not w.live.pause_btn.isEnabled(), f"Pausa: PAUSADA en el paso {n2}")
check(w.live.next_btn.isEnabled() and f"continuar paso {n2}" in w.live.next_btn.text().lower(),
      f"el boton dice '{w.live.next_btn.text()}'")
spin(1200)
check(w.last_live.get("task_status") == "paused" and w.last_live.get("step_n") == n2, "sigue en pausa sin boton")
w.live.next_btn.click(); spin(900)
check(w.last_live.get("task_status") == "running" and w.last_live.get("step_n", 0) >= n2,
      "Continuar: reanuda la secuencia")
ev = w.system.log_ctrl.toPlainText()
check("GUI: 'Pausa'" in ev and "[interfaz] 'Pausa' enviada" in ev, "evento de la pausa en el log")
check("GUI: 'Siguiente paso'" in ev and "[interfaz] 'Siguiente paso' enviado" in ev, "evento del boton en el log")
w.nav.button(0).click(); spin(500); w.grab().save(os.path.join(out, "e2e_vivo.png"))

# GPD sin robot sobre una foto guardada
w.nav.button(1).click(); spin(200)
name = "screw_120518"
if (config.capture_path(name)).exists():
    w.visionp.follow.setChecked(False)
    for i in range(w.visionp.cap_list.count()):
        if w.visionp.cap_list.item(i).data(QtCore.Qt.UserRole) == name:
            w.visionp.cap_list.setCurrentRow(i)
    w.visionp.calc_btn.click()
    for _ in range(120):
        spin(500)
        if w.visionp.results.get(name, {}).get("grasps"): break
    g = w.visionp.results.get(name, {}).get("grasps")
    check(bool(g), f"agarre calculado con GPD ({len(g) if g else 0} agarres)")
    spin(500); w.grab().save(os.path.join(out, "e2e_vision.png"))

w.ctrl.interrupt(); spin(1500)
check(not w.ctrl.running(), "controlador detenido con Ctrl+C")
check("Stopping experiment" in w.system.log_ctrl.toPlainText(), "salida limpia en el log")
# NEXT sin nadie que lo reciba: debe avisar a los 1.5 s
w.live._on_next(); spin(1900)
check(w.live.ack_note.isVisibleTo(w.live) and "no respondió" in w.live.ack_note.text(),
      "aviso cuando el controlador no responde al NEXT")
w.nav.button(3).click(); spin(400); w.grab().save(os.path.join(out, "e2e_sistema.png"))
print("FALLAS:", fails)
w.close()
sys.exit(1 if fails else 0)
