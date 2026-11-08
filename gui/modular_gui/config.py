"""Rutas, puertos y textos de la interfaz grafica.

La interfaz es SOLO una capa encima de los programas que ya existen:
  - impedance_gen3 (controlador): se lanza con su salida a un archivo de log
    y la interfaz lee ese archivo (el tablero que ya imprime cada 0.2 s).
  - screw_server (vision: kinova_grasp_NEW_VER + MODULAR/servidor_vision, o
    kinova_grasp_V2 con MODULAR_VISION=v2): igual, salida a un log.
  - task_info: secuencia, poses y parametros en JSON (sin robot).
  - "Siguiente paso" / "Pausa": texto "NEXT" / "PAUSE" por UDP a
    127.0.0.1:GUI_COMMAND_PORT; el controlador lo trata exactamente como los
    botones de su ventana GTK.
Nada de esto cambia como se controla el robot.
"""

import os
from pathlib import Path

# Controlador_Impedancia/
PROJECT_DIR = Path(__file__).resolve().parents[2]
BUILD_DIR = PROJECT_DIR / "build"
GUI_DIR = PROJECT_DIR / "gui"
LOG_DIR = GUI_DIR / "logs"

CONTROLLER_BIN = BUILD_DIR / "impedance_gen3"
# Solo para probar la interfaz sin robot (tests/fake_controller.py):
#   MODULAR_GUI_FAKE_CONTROLLER=1 ./run_gui.sh
FAKE_CONTROLLER = bool(os.environ.get("MODULAR_GUI_FAKE_CONTROLLER"))
TASK_INFO_BIN = BUILD_DIR / "task_info"
# La carpeta exportada trae los programas ya compilados: no hay nada que compilar.
CAN_COMPILE = (BUILD_DIR / "Makefile").exists()
VISION_TEST_BIN = BUILD_DIR / "vision_request_test"

# Vision (2026-10-26): kinova_grasp_NEW_VER de Antony con nuestro screw_server
# (MODULAR/servidor_vision, compilado en kinova_grasp_NEW_VER/build_modular).
#   MODULAR_VISION=v2 ./run_gui.sh   -> la anterior (kinova_grasp_V2/build_local)
#   MODULAR_PLATE_COLOR=blue ...     -> GPD solo en la placa de ese color (cfg/plate_colors.cfg)
KG_V2_DIR = Path.home() / "Escritorio" / "kinova_grasp_V2"
# MODULAR_VISION_DIR: la carpeta exportada (exportar/exportar.sh) trae su propia copia.
KG_NEW_DIR = Path(os.environ.get("MODULAR_VISION_DIR") or Path.home() / "Escritorio" / "kinova_grasp_NEW_VER")
VISION_VERSION = "v2" if os.environ.get("MODULAR_VISION", "").lower() == "v2" else "new"
if VISION_VERSION == "v2":
    VISION_DIR = KG_V2_DIR
    VISION_BUILD_DIR = KG_V2_DIR / "build_local"
    GRASPS_FROM_FILE_BIN = VISION_BUILD_DIR / "detect_ee_pose"
else:
    VISION_DIR = KG_NEW_DIR
    VISION_BUILD_DIR = KG_NEW_DIR / "build_modular"
    GRASPS_FROM_FILE_BIN = VISION_BUILD_DIR / "grasps_from_file"
SCREW_SERVER_BIN = VISION_BUILD_DIR / "screw_server"

# Color de cada objeto (cfg/plate_colors.cfg de Antony). Placa y perno pueden
# ser azul o rojo (por defecto placa azul, perno rojo); rosa despues. La GUI
# guarda la eleccion (Sistema); el screw_server acepta cualquier color de
# plate_colors.cfg en --plate-color y --bolt-color.
COLOR_NAMES = {"blue": "Azul", "red": "Rojo", "pink": "Rosa"}
OBJECT_COLORS = ["blue", "red", "pink"]  # pink: pendiente (deshabilitado)
PLATE_COLORS = BOLT_COLORS = OBJECT_COLORS
ENABLED_COLORS = {"blue", "red"}
PLATE_COLOR = os.environ.get("MODULAR_PLATE_COLOR", "blue")
BOLT_COLOR = os.environ.get("MODULAR_BOLT_COLOR", "red")


# Altura de la mesa en la BASE [m] (screw_server --table-z): el servidor tira
# lo que este debajo (mas un margen de 2 cm). -0.015 = TABLE_Z_BASE de Antony
# (robot en su lugar). Con el robot movido el 2026-10-27 fue -0.20. Si se
# mueve el robot o la mesa, medirla otra vez (el servidor avisa).
TABLE_Z_BASE = float(os.environ.get("MODULAR_TABLE_Z", "-0.015"))


def screw_server_args(plate_color=None, bolt_color=None):
    """Argumentos del screw_server: placa -> gpd_plate_params.cfg + su color,
    perno -> gpd_params.cfg + su color (como grasps_from_camera --color)."""
    if VISION_VERSION != "new":
        return ["../cfg/gpd_params.cfg"]
    return ["../cfg/gpd_params.cfg", "--plate-color", plate_color or PLATE_COLOR,
            "--bolt-color", bolt_color or BOLT_COLOR, "--table-z", f"{TABLE_Z_BASE:.3f}"]


SCREW_SERVER_ARGS = screw_server_args()
GPD_CONFIG = VISION_DIR / "cfg" / "gpd_params.cfg"
# Fotos guardadas (screw_*, hoyo_*): las del servidor en uso primero, luego las
# de la otra version (para ver las anteriores).
CAPTURE_DIRS = [VISION_BUILD_DIR] + [d for d in (KG_NEW_DIR / "build_modular", KG_V2_DIR / "build_local")
                                     if d != VISION_BUILD_DIR]


def capture_path(name, suffix=".pcd"):
    """Archivo de una foto guardada (name + suffix) en la carpeta donde este."""
    for d in CAPTURE_DIRS:
        if (d / f"{name}.pcd").exists():
            return d / f"{name}{suffix}"
    return VISION_BUILD_DIR / f"{name}{suffix}"

# Puertos UDP en localhost
VISION_PORT = 5006          # screw_server
GUI_COMMAND_PORT = 5011     # "Siguiente paso" (NEXT), "Pausa" (PAUSE) y modo (AUTO / STEP) hacia el controlador

# Geometria (igual que el controlador y kinova_grasp_V2)
TCP_OFFSET_M = 0.12                         # interfaz -> TCP (Tool configuration)
DEPTH_TO_INT_M = (0.0275, 0.0660, -0.00305)  # camara de profundidad -> interfaz (guia Gen3, tabla 92)
# Camara de profundidad (lo imprime screw_server al arrancar): ancho, alto, fx, fy, cx, cy
DEPTH_INTRINSICS = (480, 270, 333.981, 333.981, 239.205, 141.683)
TABLE_Z_M = -0.02                            # la base esta sobre una plataforma de 2 cm
FINGERTIP_FROM_INT_M = 0.1493                # 2F-85 abierta (grasp_to_ee.h)
GRIPPER_MAX_OPENING_M = 0.085

# Textos de la presentacion
TITLE = "MODULAR · Kinova Gen3"
SUBTITLE = "Control de impedancia cartesiana  ·  Vision GPD  ·  Fixture  ·  Robotiq 2F-85"
