#!/usr/bin/env python3
"""Control desde el celular: color de la placa y del perno, pausar y seguir.

Es una ampliacion de la GUI, no la sustituye, y como ella NO controla el robot:
lee el tablero que el controlador ya escribe en su log y manda los mismos
comandos por UDP (NEXT, PAUSE, COLORS) con las funciones de modular_gui.procs.
Si el celular o este programa fallan, el controlador sigue igual: el paro de
emergencia sigue siendo el fisico.

    ./run_movil.sh                 URL, PIN y QR en la terminal; Ctrl+C para cerrar
    ./run_movil.sh --log ARCHIVO   sigue ese log (controlador lanzado a mano)
"""

import argparse
import hmac
import json
import os
import re
import secrets
import subprocess
import sys
import threading
import time
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI.parent))

from modular_gui import color_dialog, config, parsers, procs  # noqa: E402
from qr import qr_terminal  # noqa: E402

PUERTO = 8420
TABLERO_VIGENTE_S = 2.5     # el controlador escribe su tablero cada 0.2 s
ANTIRREBOTE_S = 0.8         # igual que el boton de la GUI
INTENTOS_PIN = 5
BLOQUEO_PIN_S = float(os.environ.get("MODULAR_MOVIL_BLOQUEO_S", "30"))
SESION_S = 12 * 3600
COOKIE = "modular_movil"
FUENTES = {f"/fonts/{n}.ttf": config.GUI_DIR / "fonts" / f"{n}.ttf"
           for n in ("IBMPlexSans-Regular", "IBMPlexSans-Medium", "IBMPlexSans-SemiBold", "IBMPlexMono-Medium")}


def registro(texto):
    print(f"[{time.strftime('%H:%M:%S')}] {texto}", flush=True)


class Tablero(threading.Thread):
    """Ultimo estado del tablero del controlador, leido de su log."""

    def __init__(self, log_fijo=None):
        super().__init__(daemon=True)
        self.log_fijo = Path(log_fijo) if log_fijo else None
        self.lock = threading.Lock()
        self.estado = None
        self.vivo = False
        self.ruta = None
        self.pos = 0
        self.flujo = parsers.DashboardStream()

    def run(self):
        while True:
            try:
                self._leer()
            except OSError:
                with self.lock:
                    self.vivo = False
            time.sleep(0.1)

    def _leer(self):
        ruta = self.log_fijo or procs.latest_log("controlador")
        if ruta is None or not ruta.exists():
            with self.lock:
                self.vivo, self.estado = False, None
            return
        info = ruta.stat()
        if ruta != self.ruta or info.st_size < self.pos:
            self.ruta = ruta
            self.pos = max(0, info.st_size - 256 * 1024)     # no releer logs enormes
            self.flujo = parsers.DashboardStream()
            with self.lock:
                self.estado = None
        if info.st_size > self.pos:
            with open(ruta, "rb") as f:
                f.seek(self.pos)
                datos = f.read(info.st_size - self.pos)
            self.pos += len(datos)
            nuevo = self.flujo.feed(datos.decode("utf-8", errors="replace"))
            self.flujo.take_events()
            if nuevo:
                with self.lock:
                    self.estado = nuevo
        with self.lock:
            self.vivo = self.estado is not None and time.time() - info.st_mtime < TABLERO_VIGENTE_S

    def leer(self):
        with self.lock:
            return (dict(self.estado) if self.estado else None), self.vivo


class Vision(threading.Thread):
    """¿Responde el servidor de vision? Un PING por segundo, sin tomar fotos."""

    def __init__(self):
        super().__init__(daemon=True)
        self.lista = False

    def run(self):
        while True:
            self.lista = procs.ping_vision()
            time.sleep(1.0)


class Secuencia:
    """Los pasos de task_info: primer paso, y tipo y nombre de cada uno."""

    def __init__(self):
        self.primero = None
        self.tipos = {}
        self.nombres = {}

    def cargar(self):
        if self.primero is not None:
            return
        try:
            salida = subprocess.run([str(config.TASK_INFO_BIN)], capture_output=True, text=True, timeout=10)
            info = json.loads(salida.stdout)
            self.tipos = {s["n"]: s["type"] for s in info.get("steps", [])}
            self.nombres = {s["n"]: s.get("label") for s in info.get("steps", [])}
            self.primero = info.get("first_step", 1)
        except (OSError, ValueError, subprocess.SubprocessError):
            pass


class Control:
    """Que se puede hacer ahora (las mismas reglas que los botones de live_page)."""

    def __init__(self, log_fijo=None):
        self.tablero = Tablero(log_fijo)
        self.vision = Vision()
        self.secuencia = Secuencia()
        self.lock = threading.Lock()
        self.ultimo_seguir = 0.0
        self.tablero.start()
        self.vision.start()

    def _es_primero(self, st):
        return st.get("task_status") == "waiting" and st.get("step_n") == (self.secuencia.primero or 1)

    def _seguir(self, st, vivo):
        if not vivo or not st:
            return False, "Controlador apagado"
        estado, n = st.get("task_status"), st.get("step_n")
        automatico = bool(st.get("task_auto"))
        if estado == "waiting":
            if self._es_primero(st):
                return True, "Iniciar secuencia"
            return True, "Seguir en automático" if automatico else f"Ejecutar paso {n}"
        if estado == "paused":
            if "(operador)" in (st.get("message") or ""):
                return True, f"Continuar paso {n}"
            if self.secuencia.tipos.get(n) == "vision":
                return True, "Tomar otra foto"
            return True, "Dar por terminado y seguir"
        if estado == "running":
            return False, "En automático…" if automatico else "Ejecutando…"
        if estado == "done":
            return False, "Secuencia terminada"
        if estado == "aborted":
            return False, "Secuencia abortada"
        return False, "Sin estado"

    def _colores_editables(self, st, vivo):
        return not (vivo and st and st.get("task_status") == "running")

    def resumen(self):
        self.secuencia.cargar()
        st, vivo = self.tablero.leer()
        placa, perno = color_dialog.saved_colors()
        activo, texto = self._seguir(st, vivo)
        st = st or {}
        return {
            "vivo": vivo,
            "estado": st.get("task_status") if vivo else None,
            "paso": st.get("step_n") if vivo else None,
            "total": st.get("step_total") if vivo else None,
            "descripcion": (self.secuencia.nombres.get(st.get("step_n")) or st.get("step_desc")) if vivo else None,
            "mensaje": st.get("message") if vivo else None,
            "automatico": st.get("task_auto") if vivo else None,
            "vision": self.vision.lista,
            "colores": {"placa": placa, "perno": perno},
            "opciones": [{"id": c, "nombre": config.COLOR_NAMES.get(c, c), "activo": c in config.ENABLED_COLORS}
                         for c in config.OBJECT_COLORS],
            "colores_editables": self._colores_editables(st, vivo),
            "seguir": {"activo": activo, "texto": texto},
            "pausar": vivo and st.get("task_status") == "running",
        }

    def cambiar_colores(self, placa, perno):
        for c in (placa, perno):
            if c not in config.ENABLED_COLORS:
                return 400, "Ese color no está disponible."
        st, vivo = self.tablero.leer()
        if not self._colores_editables(st, vivo):
            return 409, "Los colores se cambian con la secuencia detenida."
        with self.lock:
            if self.vision.lista and config.VISION_VERSION == "new":
                ok, motivo = procs.send_colors(placa, perno)
                if not ok:
                    return 409, f"No se pudieron cambiar los colores: {motivo}."
            color_dialog.save_colors(placa, perno)
        registro(f"colores: placa {placa}, perno {perno}")
        return 200, ""

    def pausar(self):
        # Sin self.lock: nada debe poder retrasar una pausa.
        st, vivo = self.tablero.leer()
        if not (vivo and st and st.get("task_status") == "running"):
            return 409, "No hay un paso en curso que pausar."
        procs.send_pause()
        registro(f"PAUSA (paso {st.get('step_n')})")
        return 200, ""

    def seguir(self, paso, estado):
        with self.lock:
            st, vivo = self.tablero.leer()
            activo, _texto = self._seguir(st, vivo)
            if not activo:
                return 409, "Ahora no se puede seguir."
            # Lo que el operador vio en su pantalla tiene que ser lo que hay:
            # con mala senal la orden podria referirse a un paso que ya paso.
            if paso != st.get("step_n") or estado != st.get("task_status"):
                return 409, "La secuencia cambió mientras presionabas. Revisa y vuelve a intentar."
            if time.monotonic() - self.ultimo_seguir < ANTIRREBOTE_S:
                return 409, "Espera un momento."
            self.secuencia.cargar()
            if self._es_primero(st) and config.VISION_VERSION == "new" and not config.FAKE_CONTROLLER:
                placa, perno = color_dialog.saved_colors()
                ok, motivo = procs.send_colors(placa, perno)
                if not ok:
                    return 409, f"No se inició: {motivo}. Revisa que el servidor de visión esté corriendo."
            procs.send_next_step()
            self.ultimo_seguir = time.monotonic()
        registro(f"SEGUIR (paso {paso}, {estado})")
        return 200, ""


class Acceso:
    """PIN y sesiones. Solo es para la red local: HTTP sin cifrar."""

    def __init__(self, pin):
        self.pin = pin
        self.lock = threading.Lock()
        self.sesiones = {}
        self.fallos = 0
        self.bloqueado_hasta = 0.0

    def entrar(self, pin):
        """-> (estado HTTP, token o mensaje)"""
        with self.lock:
            ahora = time.monotonic()
            if ahora < self.bloqueado_hasta:
                return 429, f"Demasiados intentos. Espera {int(self.bloqueado_hasta - ahora) + 1} s."
            if not hmac.compare_digest(str(pin).encode(), self.pin.encode()):
                self.fallos += 1
                if self.fallos >= INTENTOS_PIN:
                    self.fallos = 0
                    self.bloqueado_hasta = ahora + BLOQUEO_PIN_S
                    registro("PIN: demasiados intentos, bloqueado")
                    return 429, f"Demasiados intentos. Espera {int(BLOQUEO_PIN_S)} s."
                return 401, "PIN incorrecto."
            self.fallos = 0
            token = secrets.token_urlsafe(32)
            self.sesiones[token] = time.time() + SESION_S
            return 200, token

    def valida(self, token):
        with self.lock:
            return bool(token) and self.sesiones.get(token, 0) > time.time()


def crear_manejador(control, acceso):
    class Manejador(BaseHTTPRequestHandler):
        server_version = "MODULAR-movil"
        protocol_version = "HTTP/1.1"

        def log_message(self, *_args):
            pass

        def _responder(self, codigo, cuerpo, tipo="application/json; charset=utf-8", extra=()):
            self.send_response(codigo)
            self.send_header("Content-Type", tipo)
            self.send_header("Content-Length", str(len(cuerpo)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            for k, v in extra:
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(cuerpo)

        def _json(self, codigo, datos, extra=()):
            self._responder(codigo, json.dumps(datos).encode(), extra=extra)

        def _resultado(self, codigo, error):
            self._json(codigo, {"ok": True} if codigo == 200 else {"error": error})

        def _sesion(self):
            galleta = cookies.SimpleCookie(self.headers.get("Cookie", ""))
            return acceso.valida(galleta[COOKIE].value) if COOKIE in galleta else False

        def _mismo_origen(self):
            origen = self.headers.get("Origin")
            return origen is None or urlsplit(origen).netloc == self.headers.get("Host")

        def _cuerpo(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n > 1024:
                return None
            try:
                datos = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return None
            return datos if isinstance(datos, dict) else None

        def do_GET(self):
            ruta = urlsplit(self.path).path
            if ruta == "/":
                self._responder(200, (AQUI / "index.html").read_bytes(), "text/html; charset=utf-8")
            elif ruta in FUENTES:
                self._responder(200, FUENTES[ruta].read_bytes(), "font/ttf")
            elif ruta == "/api/estado":
                if self._sesion():
                    self._json(200, control.resumen())
                else:
                    self._json(401, {"error": "Falta el PIN."})
            else:
                self._json(404, {"error": "No existe."})

        def do_POST(self):
            ruta = urlsplit(self.path).path
            datos = self._cuerpo()
            if datos is None:
                self._json(400, {"error": "Petición inválida."})
            elif not self._mismo_origen():
                self._json(403, {"error": "Origen no permitido."})
            elif ruta == "/api/entrar":
                codigo, valor = acceso.entrar(datos.get("pin", ""))
                if codigo == 200:
                    galleta = f"{COOKIE}={valor}; Path=/; Max-Age={SESION_S}; HttpOnly; SameSite=Strict"
                    self._json(200, {"ok": True}, extra=[("Set-Cookie", galleta)])
                else:
                    self._json(codigo, {"error": valor})
            elif not self._sesion():
                self._json(401, {"error": "Falta el PIN."})
            elif ruta == "/api/pausa":
                self._resultado(*control.pausar())
            elif ruta == "/api/seguir":
                self._resultado(*control.seguir(datos.get("paso"), datos.get("estado")))
            elif ruta == "/api/colores":
                self._resultado(*control.cambiar_colores(datos.get("placa"), datos.get("perno")))
            else:
                self._json(404, {"error": "No existe."})

    return Manejador


def direcciones_locales():
    """IPv4 de esta PC, sin la local. La del robot se marca: el celular no va ahi."""
    try:
        salida = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    ips = re.findall(r"inet (\d+\.\d+\.\d+\.\d+)/", salida)
    return [ip for ip in ips if not ip.startswith("127.")]


def main():
    ap = argparse.ArgumentParser(description="Control desde el celular (colores, pausar, seguir).")
    ap.add_argument("--puerto", type=int, default=PUERTO)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--log", help="log del controlador a seguir (por defecto, el ultimo de la GUI)")
    ap.add_argument("--sin-qr", action="store_true", help="no mostrar el codigo QR")
    args = ap.parse_args()

    pin = os.environ.get("MODULAR_MOVIL_PIN") or f"{secrets.randbelow(10 ** 6):06d}"
    control = Control(args.log)
    servidor = ThreadingHTTPServer((args.host, args.puerto), crear_manejador(control, Acceso(pin)))

    print("\nControl desde el celular (misma red WiFi que esta PC)")
    ips = direcciones_locales()
    for ip in ips:
        nota = "   <- red del robot, no es para el celular" if ip.startswith("192.168.1.") else ""
        print(f"  http://{ip}:{args.puerto}{nota}")
    print(f"  PIN: {pin}")
    # QR con la primera IP que no es la del robot; lleva el PIN despues de '#'
    # (el navegador no lo manda en la peticion; la pagina entra sola con el).
    wifi = [ip for ip in ips if not ip.startswith("192.168.1.")]
    if wifi and not args.sin_qr:
        url = f"http://{wifi[0]}:{args.puerto}/#pin={pin}"
        print(f"\nEscanea con la camara del celular ({wifi[0]}; entra sin escribir el PIN):")
        print(qr_terminal(url))
    elif not wifi:
        print("\nSin QR: esta PC no tiene otra red ademas de la del robot (conectala al WiFi).")
    print("Desde el celular se puede mover el robot real: paro de emergencia a la mano.")
    print("Ctrl+C para cerrar; el controlador sigue igual.\n", flush=True)
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        servidor.server_close()


if __name__ == "__main__":
    main()
