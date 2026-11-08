"""Tema claro de la interfaz: marco gris azulado (barra superior + lateral) y
las paginas sobre una hoja blanca. Colores, fuentes y hoja de estilo Qt.

Reglas (estilo "framed sheet"): el color va en los datos, no en el marco;
superficies planas (sin sombras ni degradados); un solo color de acento
(ACCENT) en el dato principal y en la seleccion.
"""

from pathlib import Path

from PyQt5 import QtGui

# ---------- superficies ----------
FRAME = "#eef2f5"       # barra superior y lateral (gris azulado, acero)
SHEET = "#ffffff"       # hoja donde viven las paginas
RAISED = "#f4f6f8"      # pozo / fondo de la vista 3D
ACTIVE = "#e3e8ec"      # renglon activo (nav, paso actual)
BORDER = "#e3e7ea"      # bordes de tarjetas y de la hoja
RULE = "#eef0f2"        # renglones de tablas
CONTROL = "#878e95"     # bordes de controles (3.3:1 sobre blanco)
TRACK = "#c9d0d6"       # riel de barras y sliders

# ---------- texto ----------
TEXT = "#131416"
MUTED = "#5d6166"       # secundario (4.5:1 sobre blanco)
FAINT = "#6b6f74"       # notas
LOG_TEXT = "#2b2e31"
SELECTION = "#cfe3e8"   # texto seleccionado

# ---------- datos ----------
ACCENT = "#1b6c80"      # el dato principal: TCP, error, paso actual
ACCENT_SOFT = "#6fa7b5" # el mismo tono aclarado (tramos alternos, dedos del agarre)
ACCENT_FILL = (27, 108, 128, 36)   # relleno de 14 % bajo una curva
CHART2 = "#2f70bb"      # referencia (rampa)
CHART3 = "#a86f0c"      # objetivo
PRIMARY = "#1d1f20"     # boton principal (casi negro)
PRIMARY_HOVER = "#33373a"
ON_PRIMARY = "#ffffff"

# ---------- estado ----------
POSITIVE = "#1e7b4f"
NEGATIVE = "#c4242a"
NEGATIVE_SOFT = "#fff5f4"
WARNING = "#92400e"
NEUTRAL = "#7d838a"     # punto de "apagado" (3.4:1 sobre el marco)

# ---------- escena 3D ----------
FLOOR = "#e9edf0"
FLOOR_GRID = "#d3d9de"
ROBOT_LINK = "#5a6168"
ROBOT_JOINT = "#2e3338"
GRIPPER_FINGER = "#8a9199"
CLOUD_LOW = "#b9d3da"   # nube: abajo claro -> ACCENT -> arriba oscuro
CLOUD_HIGH = "#0b2f38"

# Ejes x, y, z (convencion de robotica, apagados)
XYZ = (NEGATIVE, POSITIVE, CHART2)

ASSET_DIR = Path(__file__).resolve().parent.parent / "assets"
FONT_DIR = Path(__file__).resolve().parent.parent / "fonts"
CHECK_SVG = (ASSET_DIR / "check.svg").as_posix()
CHEVRON_SVG = (ASSET_DIR / "chevron.svg").as_posix()
UI_FONT = "IBM Plex Sans"
MONO_FONT = "IBM Plex Mono"
UI_FONT_FILE = (FONT_DIR / "IBMPlexSans-Regular.ttf").as_posix()

# Tamanos [px]
FS_TITLE = 22
FS_SECTION = 16
FS_BODY = 14
FS_SMALL = 13

# Vista de presentacion (proyector) [px]
PS_LABEL = 18
PS_BODY = 22
PS_STEP = 44
PS_MODULE = 32   # nombre del modulo del paso (grande, se lee desde lejos)
PS_FIGURE = 40


def hex_rgb(color):
    """'#rrggbb' -> (r, g, b) en 0..1 (para VTK)."""
    c = color.lstrip("#")
    return tuple(int(c[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def font(px=FS_BODY, weight=400, mono=False):
    f = QtGui.QFont(MONO_FONT if mono else UI_FONT)
    f.setPixelSize(int(px))
    f.setWeight({400: QtGui.QFont.Normal, 500: QtGui.QFont.Medium,
                 600: QtGui.QFont.DemiBold}.get(weight, QtGui.QFont.Normal))
    return f


def _load_fonts():
    global UI_FONT, MONO_FONT
    loaded = set()
    for f in sorted(FONT_DIR.glob("*.ttf")):
        i = QtGui.QFontDatabase.addApplicationFont(str(f))
        if i >= 0:
            loaded.update(QtGui.QFontDatabase.applicationFontFamilies(i))
    if "IBM Plex Sans" not in loaded:
        UI_FONT = "Inter"
    if "IBM Plex Mono" not in loaded:
        MONO_FONT = "DejaVu Sans Mono"


def apply(app):
    _load_fonts()
    app.setFont(font())
    pal = QtGui.QPalette()
    for role, color in [
        (QtGui.QPalette.Window, FRAME), (QtGui.QPalette.Base, SHEET),
        (QtGui.QPalette.AlternateBase, RAISED), (QtGui.QPalette.Text, TEXT),
        (QtGui.QPalette.WindowText, TEXT), (QtGui.QPalette.Button, SHEET),
        (QtGui.QPalette.ButtonText, TEXT), (QtGui.QPalette.Highlight, ACTIVE),
        (QtGui.QPalette.HighlightedText, TEXT), (QtGui.QPalette.ToolTipBase, SHEET),
        (QtGui.QPalette.ToolTipText, TEXT), (QtGui.QPalette.PlaceholderText, FAINT),
    ]:
        pal.setColor(role, QtGui.QColor(color))
    app.setPalette(pal)
    app.setStyleSheet(style())


def style():
    return f"""
QWidget {{ color: {TEXT}; background: transparent; font-family: "{UI_FONT}"; font-size: {FS_BODY}px; }}
QMainWindow, #root {{ background: {FRAME}; }}
QToolTip {{ background: {SHEET}; color: {TEXT}; border: 1px solid {CONTROL}; padding: 6px; }}
QMessageBox, QDialog, QFileDialog {{ background: {SHEET}; }}

#sheet {{ background: {SHEET}; border: 1px solid {BORDER}; border-radius: 16px; }}
#productName {{ font-size: 15px; font-weight: 600; }}
#productScope {{ font-size: {FS_SMALL}px; color: {MUTED}; }}
#navButton {{
    text-align: left; padding: 0 12px; border: none; border-radius: 8px; background: transparent;
    color: {MUTED}; font-size: {FS_BODY}px; min-height: 34px; margin: 0 10px;
}}
#navButton:hover {{ color: {TEXT}; background: transparent; }}
#navButton:checked {{ background: {ACTIVE}; color: {TEXT}; font-weight: 500; }}
#navButton:focus {{ border: 2px solid {ACCENT}; }}
#sideNote {{ font-size: {FS_SMALL}px; color: {MUTED}; }}

#pageTitle {{ font-size: {FS_TITLE}px; font-weight: 600; }}
#pageSub {{ font-size: {FS_BODY}px; color: {MUTED}; }}

#card {{ background: {SHEET}; border: 1px solid {BORDER}; border-radius: 8px; }}
#cardTitle {{ font-size: {FS_SECTION}px; font-weight: 500; }}
#cardMeta {{ font-size: {FS_SMALL}px; color: {MUTED}; }}
#kpiLabel {{ font-size: {FS_SMALL}px; color: {MUTED}; }}
#divider {{ background: {BORDER}; }}

QPushButton {{
    background: {SHEET}; border: 1px solid {CONTROL}; border-radius: 6px;
    padding: 0 14px; min-height: 32px; font-weight: 500;
}}
QPushButton:hover {{ background: {RAISED}; }}
QPushButton:pressed {{ background: {ACTIVE}; }}
QPushButton:focus {{ border: 2px solid {ACCENT}; }}
QPushButton:disabled {{ color: {FAINT}; background: {RAISED}; border-color: {BORDER}; }}
QPushButton#primary {{ background: {PRIMARY}; color: {ON_PRIMARY}; border: none; }}
QPushButton#primary:hover {{ background: {PRIMARY_HOVER}; }}
QPushButton#primary:focus {{ border: 2px solid {ACCENT}; }}
QPushButton#primary:disabled {{ background: {ACTIVE}; color: {MUTED}; }}
QPushButton#danger {{ color: {NEGATIVE}; border: 1px solid {NEGATIVE}; }}
QPushButton#danger:hover {{ background: {NEGATIVE_SOFT}; }}
QPushButton#danger:focus {{ border: 2px solid {ACCENT}; }}
QPushButton#danger:disabled {{ color: {FAINT}; border-color: {BORDER}; background: {RAISED}; }}
QPushButton#seg {{ min-height: 30px; padding: 0 12px; color: {MUTED}; }}
QPushButton#seg:checked {{ background: {ACTIVE}; color: {TEXT}; border: 1px solid {ACCENT}; font-weight: 600; }}
QPushButton#seg:checked:disabled {{ color: {MUTED}; border-color: {CONTROL}; }}
QPushButton#next {{
    background: {PRIMARY}; color: {ON_PRIMARY}; border: 2px solid {PRIMARY}; border-radius: 6px;
    font-size: 17px; font-weight: 500; min-height: 56px;
}}
QPushButton#next:hover {{ background: {PRIMARY_HOVER}; border-color: {PRIMARY_HOVER}; }}
QPushButton#pause {{
    color: {NEGATIVE}; background: {RAISED}; border: 2px solid {NEGATIVE}; border-radius: 6px;
    font-size: 17px; font-weight: 500; min-height: 56px; padding: 0 20px;
}}
QPushButton#pause:hover {{ background: {NEGATIVE_SOFT}; }}
QPushButton#pause:focus {{ border: 2px solid {ACCENT}; }}
QPushButton#pause:disabled {{ color: {FAINT}; border-color: {BORDER}; background: {RAISED}; }}
QPushButton#next:pressed {{ background: {TEXT}; }}
QPushButton#next:focus {{ border: 2px solid {ACCENT}; }}
QPushButton#next:disabled {{ background: {ACTIVE}; color: {MUTED}; border-color: {ACTIVE}; }}

QPlainTextEdit, QTextEdit {{
    background: {RAISED}; border: none; border-radius: 8px;
    font-family: "{MONO_FONT}"; font-size: {FS_SMALL}px; color: {LOG_TEXT}; padding: 8px;
    selection-background-color: {SELECTION}; selection-color: {TEXT};
}}
QListWidget {{ background: transparent; border: 1px solid transparent; border-radius: 6px; outline: none; }}
QListWidget:focus {{ border: 1px solid {ACCENT}; }}
QListWidget::item {{ border-radius: 6px; padding: 6px 8px; margin: 0; border-bottom: 1px solid {RULE}; }}
QListWidget::item:selected {{ background: {ACTIVE}; color: {TEXT}; }}
QListWidget::item:hover:!selected {{ background: {RAISED}; }}
QComboBox {{
    background: {SHEET}; border: 1px solid {CONTROL}; border-radius: 6px; padding: 0 10px; min-height: 32px;
}}
QComboBox:focus {{ border: 2px solid {ACCENT}; }}
QComboBox::drop-down {{ border: none; width: 26px; }}
QComboBox::down-arrow {{ image: url("{CHEVRON_SVG}"); width: 12px; height: 12px; }}
QComboBox QAbstractItemView {{
    background: {SHEET}; border: 1px solid {CONTROL}; selection-background-color: {ACTIVE};
    selection-color: {TEXT}; outline: none;
}}
QTableWidget {{
    background: {SHEET}; border: 1px solid transparent; gridline-color: transparent; outline: none;
    selection-background-color: {ACTIVE}; selection-color: {TEXT};
}}
QTableWidget:focus {{ border: 1px solid {ACCENT}; }}
QHeaderView {{ background: {SHEET}; }}
QHeaderView::section {{
    background: {SHEET}; color: {MUTED}; border: none; border-bottom: 1px solid {BORDER};
    padding: 6px 6px; font-size: {FS_SMALL}px; font-weight: 400;
}}
QTableWidget::item {{ padding: 0 6px; border-bottom: 1px solid {RULE}; }}
QScrollArea {{ background: transparent; border: none; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {CONTROL}; border-radius: 4px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: {CONTROL}; border-radius: 4px; min-width: 30px; }}
QProgressBar {{ background: {TRACK}; border: none; border-radius: 3px; max-height: 6px; min-height: 6px; }}
QProgressBar::chunk {{ border-radius: 3px; background: {ACCENT}; }}
QSlider::groove:horizontal {{ height: 4px; background: {TRACK}; border-radius: 2px; }}
QSlider::handle:horizontal:focus {{ border: 2px solid {ACCENT}; }}
QSlider::handle:horizontal {{
    background: {SHEET}; border: 1px solid {CONTROL}; width: 16px; margin: -7px 0; border-radius: 8px;
}}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 2px; }}
QCheckBox {{ spacing: 8px; color: {TEXT}; }}
QCheckBox::indicator {{
    width: 16px; height: 16px; border-radius: 4px; border: 1px solid {CONTROL}; background: {SHEET};
}}
QCheckBox::indicator:checked {{ background: {TEXT}; border-color: {TEXT}; image: url("{CHECK_SVG}"); }}
QCheckBox:focus {{ color: {ACCENT}; }}
QCheckBox::indicator:focus {{ border: 2px solid {ACCENT}; }}
QSplitter::handle {{ background: {SHEET}; }}
QTabWidget::pane {{ border: none; border-top: 1px solid {BORDER}; top: -1px; }}
QTabBar {{ background: transparent; }}
QTabBar::tab {{
    background: transparent; color: {MUTED}; padding: 8px 2px; margin-right: 22px; border: none;
    border-bottom: 2px solid transparent;
}}
QTabBar::tab:selected {{ color: {TEXT}; border-bottom: 2px solid {TEXT}; }}
QTabBar::tab:focus {{ color: {ACCENT}; border-bottom: 2px solid {ACCENT}; }}
QTabBar::tab:hover:!selected {{ color: {TEXT}; }}
"""
