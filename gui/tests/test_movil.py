"""Prueba del control desde el celular, SIN robot: controlador falso + vision falsa.

    .venv/bin/python tests/test_movil.py

Usa sus propios colores guardados (XDG_CONFIG_HOME temporal), no los de la GUI.
Con --dejar-abierto PUERTO deja todo corriendo con PIN 123456, para ver la pantalla.
"""
import http.client
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

GUI = Path(__file__).resolve().parents[1]
PIN = "123456"
abierto = "--dejar-abierto" in sys.argv
PUERTO = int(sys.argv[sys.argv.index("--dejar-abierto") + 1]) if abierto else 18420
tmp = tempfile.TemporaryDirectory()
LOG = Path(tmp.name) / "controlador_prueba.log"
fallas = 0


def check(ok, que):
    global fallas
    print(("  OK    " if ok else "  FALLA ") + que, flush=True)
    fallas += 0 if ok else 1


class VisionFalsa(threading.Thread):
    """Responde como screw_server: PING y COLORS, y anota lo que recibe."""

    def __init__(self):
        super().__init__(daemon=True)
        self.encendida = True
        self.colores = []
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("127.0.0.1", 5006))

    def run(self):
        while True:
            datos, de = self.s.recvfrom(256)
            if not self.encendida:
                continue
            texto = datos.decode()
            if texto == "PING":
                self.s.sendto(b"PONG", de)
            elif texto.startswith("COLORS "):
                _, placa, perno = texto.split()
                self.colores.append((placa, perno))
                self.s.sendto(f"COLORS OK {placa} {perno}".encode(), de)


class Cliente:
    def __init__(self, galleta=None):
        self.galleta = galleta

    def pedir(self, metodo, ruta, datos=None, origen=None):
        c = http.client.HTTPConnection("127.0.0.1", PUERTO, timeout=8)
        cabeceras = {"Content-Type": "application/json"}
        if self.galleta:
            cabeceras["Cookie"] = self.galleta
        if origen:
            cabeceras["Origin"] = origen
        c.request(metodo, ruta, json.dumps(datos) if datos is not None else None, cabeceras)
        r = c.getresponse()
        cuerpo = r.read()
        galleta = r.getheader("Set-Cookie")
        c.close()
        if galleta:
            self.galleta = galleta.split(";")[0]
            self.cookie_completa = galleta
        try:
            return r.status, json.loads(cuerpo)
        except ValueError:
            return r.status, {}

    def estado(self):
        return self.pedir("GET", "/api/estado")[1]

    def esperar(self, condicion, s=8.0):
        fin = time.time() + s
        while time.time() < fin:
            e = self.estado()
            if condicion(e):
                return e
            time.sleep(0.1)
        return self.estado()


vision = VisionFalsa()
vision.start()
log = open(LOG, "wb")
falso = subprocess.Popen([sys.executable, "-u", str(GUI / "tests" / "fake_controller.py")], stdout=log,
                         stderr=subprocess.STDOUT, cwd=GUI)
entorno = dict(os.environ, MODULAR_MOVIL_PIN=PIN, MODULAR_MOVIL_BLOQUEO_S="1", XDG_CONFIG_HOME=tmp.name)
entorno.pop("MODULAR_GUI_FAKE_CONTROLLER", None)
servidor = subprocess.Popen([sys.executable, str(GUI / "movil" / "servidor.py"), "--puerto", str(PUERTO),
                             "--log", str(LOG)], env=entorno, cwd=GUI, stdout=subprocess.DEVNULL)
try:
    for _ in range(50):
        try:
            socket.create_connection(("127.0.0.1", PUERTO), timeout=0.2).close()
            break
        except OSError:
            time.sleep(0.1)

    if abierto:
        print("listo", flush=True)
        servidor.wait()
        sys.exit(0)

    a = Cliente()
    check(a.pedir("GET", "/")[0] == 200, "la pagina abre sin PIN (ahi se pide)")
    check(a.pedir("GET", "/api/estado")[0] == 401, "sin PIN no hay estado")
    check(a.pedir("POST", "/api/seguir", {"paso": 1, "estado": "waiting"})[0] == 401, "sin PIN no se puede seguir")
    check(a.pedir("POST", "/api/pausa", {})[0] == 401, "sin PIN no se puede pausar")
    check(a.pedir("GET", "/fonts/../servidor.py")[0] == 404, "no sirve archivos fuera de la lista")

    for _ in range(4):
        check(a.pedir("POST", "/api/entrar", {"pin": "000000"})[0] == 401, "PIN incorrecto rechazado")
    check(a.pedir("POST", "/api/entrar", {"pin": "000000"})[0] == 429, "al quinto fallo se bloquea")
    check(a.pedir("POST", "/api/entrar", {"pin": PIN})[0] == 429, "bloqueado: ni el PIN correcto entra")
    time.sleep(1.1)
    check(a.pedir("POST", "/api/entrar", {"pin": PIN})[0] == 200, "tras el bloqueo entra con el PIN")
    check("HttpOnly" in a.cookie_completa and "SameSite=Strict" in a.cookie_completa, "cookie HttpOnly y SameSite")
    check(a.pedir("POST", "/api/pausa", {}, origen="http://otro.example")[0] == 403, "otro origen rechazado")

    e = a.esperar(lambda e: e["vivo"] and e["vision"])
    check(e["vivo"] and e["estado"] == "waiting" and e["paso"] == 1, f"tablero leido: {e['estado']}, paso {e['paso']}")
    check(e["seguir"]["activo"] and e["seguir"]["texto"] == "Iniciar secuencia", "boton: " + e["seguir"]["texto"])
    check(not e["pausar"], "esperando no se puede pausar")
    check(a.pedir("POST", "/api/pausa", {})[0] == 409, "pausa rechazada sin paso en curso")

    check(a.pedir("POST", "/api/colores", {"placa": "pink", "perno": "red"})[0] == 400, "rosa (pendiente) rechazado")
    check(a.pedir("POST", "/api/colores", {"placa": "red", "perno": "blue"})[0] == 200, "colores aceptados")
    check(vision.colores[-1] == ("red", "blue"), "la vision recibio COLORS red blue")
    check(a.estado()["colores"] == {"placa": "red", "perno": "blue"}, "se guardan y se reportan")

    check(a.pedir("POST", "/api/seguir", {"paso": 9, "estado": "waiting"})[0] == 409, "orden de un paso viejo rechazada")
    check(a.estado()["estado"] == "waiting", "...y el controlador no se movio")

    vision.encendida = False
    a.esperar(lambda e: not e["vision"])
    c = a.pedir("POST", "/api/seguir", {"paso": 1, "estado": "waiting"})
    check(c[0] == 409 and "visión" in c[1]["error"], "sin vision no inicia: " + c[1].get("error", ""))
    check(a.estado()["estado"] == "waiting", "...y el controlador sigue esperando")
    vision.encendida = True
    a.esperar(lambda e: e["vision"])

    n = len(vision.colores)
    check(a.pedir("POST", "/api/seguir", {"paso": 1, "estado": "waiting"})[0] == 200, "iniciar aceptado")
    check(len(vision.colores) == n + 1, "al iniciar vuelve a mandar los colores, como la GUI")
    e = a.esperar(lambda e: e["estado"] == "running")
    check(e["estado"] == "running", "el controlador ejecuta")
    check(e["pausar"] and not e["seguir"]["activo"], "ejecutando: Pausar activo, Seguir no")
    check(a.pedir("POST", "/api/colores", {"placa": "blue", "perno": "red"})[0] == 409, "colores bloqueados ejecutando")

    check(a.pedir("POST", "/api/pausa", {})[0] == 200, "pausa aceptada")
    e = a.esperar(lambda e: e["estado"] == "paused")
    paso = e["paso"]
    check(e["estado"] == "paused", f"controlador en pausa en el paso {paso}")
    check(e["seguir"]["texto"] == f"Continuar paso {paso}", "boton: " + e["seguir"]["texto"])
    check(not e["pausar"] and e["colores_editables"], "en pausa: sin Pausar, y colores editables")
    check(a.pedir("POST", "/api/colores", {"placa": "blue", "perno": "red"})[0] == 200, "colores en pausa aceptados")

    time.sleep(0.9)
    check(a.pedir("POST", "/api/seguir", {"paso": paso, "estado": "paused"})[0] == 200, "continuar aceptado")
    e = a.esperar(lambda e: e["estado"] == "running")
    check(e["estado"] == "running" and e["paso"] == paso, "reanuda el mismo paso")
    check(a.pedir("POST", "/api/seguir", {"paso": paso, "estado": "running"})[0] == 409, "ejecutando no acepta Seguir")

    falso.terminate()
    falso.wait(timeout=5)
    e = a.esperar(lambda e: not e["vivo"], 5)
    check(not e["vivo"] and not e["pausar"] and not e["seguir"]["activo"], "controlador caido: se reporta y sin acciones")
    check(e["colores_editables"], "con el controlador apagado los colores se pueden elegir")
finally:
    for p in (servidor, falso):
        if p.poll() is None:
            p.terminate()
    log.close()

# QR de la terminal (movil/qr.py): se lee con OpenCV si esta instalado.
sys.path.insert(0, str(GUI / "movil"))
from qr import qr_matriz  # noqa: E402
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
if cv2 is None:
    print("  --    QR: sin OpenCV, no se revisa")
else:
    url = "http://10.214.112.188:8420/#pin=123456"
    m = np.array(qr_matriz(url), dtype=bool)
    img = cv2.resize(np.pad(np.where(m, 0, 255).astype(np.uint8), 4, constant_values=255), None,
                     fx=10, fy=10, interpolation=cv2.INTER_NEAREST)
    check(cv2.QRCodeDetector().detectAndDecode(img)[0] == url, "el QR se lee y trae la URL con el PIN")
pagina = (GUI / "movil" / "index.html").read_text(encoding="utf-8")
check("#pin=" in pagina or "pin=(\\d{6})" in pagina, "la pagina entra con el PIN del QR (#pin=...)")

print(f"FALLAS: {fallas}")
sys.exit(1 if fallas else 0)
