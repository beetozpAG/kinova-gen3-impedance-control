"""Colores de la placa y del perno, elegidos al iniciar la secuencia.

El servidor de vision arranca con los ultimos colores elegidos (QSettings) y
los cambia en marcha con "COLORS placa perno" (procs.send_colors); asi no hay
que reiniciarlo para cambiar de color.

Cada objeto se dibuja en el color elegido: lo que se ve es lo que la vision va
a buscar. Tambien se proyecta, por eso todo es grande.
"""

import math

from PyQt5 import QtCore, QtGui, QtWidgets

from . import config, theme, widgets as W

KEYS = (("plate_color", config.PLATE_COLORS, config.PLATE_COLOR, "Placa"),
        ("bolt_color", config.BOLT_COLORS, config.BOLT_COLOR, "Perno"))

# Color de las piezas reales (dato, no tema): solo se usa en este dialogo.
OBJECT_RGB = {"blue": "#1f5fbf", "red": "#d92d2d", "pink": "#e0669a"}
HOLDER = "#26292c"      # soporte negro del perno


def saved_colors():
    """(color de la placa, color del perno) elegidos la ultima vez."""
    settings = QtCore.QSettings("MODULAR", "gui")
    out = []
    for key, _colors, default, _label in KEYS:
        want = str(settings.value(key, default))
        out.append(want if want in config.ENABLED_COLORS else default)
    return tuple(out)


def save_colors(plate, bolt):
    settings = QtCore.QSettings("MODULAR", "gui")
    settings.setValue("plate_color", plate)
    settings.setValue("bolt_color", bolt)


def _object_color(name):
    return QtGui.QColor(OBJECT_RGB.get(name, theme.NEUTRAL))


class ObjectView(QtWidgets.QWidget):
    """La placa o el perno dibujados en el color elegido."""

    def __init__(self, key, color, parent=None):
        super().__init__(parent)
        self.key = key
        self._color = _object_color(color)
        self.setFixedHeight(220)
        self._anim = QtCore.QVariantAnimation(self)
        self._anim.setDuration(220)
        self._anim.setEasingCurve(QtCore.QEasingCurve.OutExpo)
        self._anim.valueChanged.connect(self._on_color)

    def set_color(self, name):
        self._anim.stop()
        self._anim.setStartValue(self._color)
        self._anim.setEndValue(_object_color(name))
        self._anim.start()

    def _on_color(self, color):
        self._color = color
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(theme.RAISED))
        p.drawRoundedRect(QtCore.QRectF(self.rect()), 8, 8)
        p.translate(self.width() / 2, self.height() / 2)
        if self.key == "plate_color":
            self._draw_plate(p)
        else:
            self._draw_bolt(p)

    def _draw_plate(self, p):
        # De frente y con el canto abajo, como en el soporte: dos hoyos arriba.
        w, h, edge = 210, 136, 14
        x, y = -w / 2, -(h + edge) / 2
        p.setBrush(self._color.darker(135))
        p.drawRoundedRect(QtCore.QRectF(x, y + edge, w, h), 5, 5)
        p.setBrush(self._color)
        p.drawRoundedRect(QtCore.QRectF(x, y, w, h), 5, 5)
        p.setBrush(QtGui.QColor(theme.RAISED))
        for cx in (-w * 0.29, w * 0.29):
            p.drawEllipse(QtCore.QPointF(cx, y + h * 0.36), 21, 21)

    def _draw_bolt(self, p):
        # Visto desde la camara de la muneca: cabeza hexagonal sobre su soporte.
        p.setBrush(QtGui.QColor(HOLDER))
        p.drawEllipse(QtCore.QPointF(0, 0), 92, 92)
        hexagon = QtGui.QPolygonF()
        for i in range(6):
            a = math.radians(60.0 * i)
            hexagon.append(QtCore.QPointF(70 * math.cos(a), 70 * math.sin(a)))
        p.setBrush(self._color.darker(135))
        p.drawPolygon(hexagon.translated(0, 9))
        p.setBrush(self._color)
        p.drawPolygon(hexagon)


class SwatchButton(QtWidgets.QAbstractButton):
    """Una opcion de color: muestra de color y nombre."""

    def __init__(self, name, enabled, parent=None):
        super().__init__(parent)
        self.name = name
        self.title = config.COLOR_NAMES.get(name, name)
        self.setCheckable(True)
        self.setEnabled(enabled)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMinimumHeight(64)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.setAccessibleName(self.title + ("" if enabled else " (pendiente)"))
        if enabled:
            self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setAttribute(QtCore.Qt.WA_Hover)

    def sizeHint(self):
        return QtCore.QSize(112, 64)

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        box = QtCore.QRectF(self.rect()).adjusted(1.5, 1.5, -1.5, -1.5)
        on = self.isChecked()
        hover = self.underMouse() and self.isEnabled() and not on
        fill = theme.ACTIVE if on else theme.RAISED if hover or not self.isEnabled() else theme.SHEET
        border, width = (theme.ACCENT, 2.5) if self.hasFocus() else \
            (theme.TEXT, 2.5) if on else \
            (theme.BORDER if not self.isEnabled() else theme.CONTROL, 1.0)
        p.setPen(QtGui.QPen(QtGui.QColor(border), width))
        p.setBrush(QtGui.QColor(fill))
        p.drawRoundedRect(box, 6, 6)

        center = QtCore.QPointF(box.left() + 25, box.center().y())
        if self.isEnabled():
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(_object_color(self.name))
        else:   # pendiente: solo el contorno
            p.setPen(QtGui.QPen(_object_color(self.name), 2.0, QtCore.Qt.DashLine))
            p.setBrush(QtCore.Qt.NoBrush)
        p.drawEllipse(center, 14, 14)
        if on:  # la seleccion no depende solo del color
            p.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2.6, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap,
                                QtCore.Qt.RoundJoin))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawPolyline(QtGui.QPolygonF([center + QtCore.QPointF(-6.5, 0.5),
                                            center + QtCore.QPointF(-2, 5),
                                            center + QtCore.QPointF(6.5, -5)]))

        text = QtCore.QRectF(box.left() + 47, box.top(), box.width() - 49, box.height())
        p.setPen(QtGui.QColor(theme.TEXT if self.isEnabled() else theme.MUTED))
        if self.isEnabled():
            p.setFont(theme.font(17, 600 if on else 500))
            p.drawText(text, QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, self.title)
        else:
            p.setFont(theme.font(15, 500))
            p.drawText(text.adjusted(0, -9, 0, -9), QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, self.title)
            p.setFont(theme.font(theme.FS_SMALL))
            p.drawText(text.adjusted(0, 10, 0, 10), QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, "pendiente")


class ColorPicker(QtWidgets.QWidget):
    """Un objeto: su nombre, su dibujo y las opciones de color."""

    changed = QtCore.pyqtSignal()

    def __init__(self, key, colors, label, current, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)
        lay.addWidget(W.label(label, size=theme.PS_BODY, weight=600))
        self.view = ObjectView(key, current)
        lay.addWidget(self.view)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        self.group = QtWidgets.QButtonGroup(self)
        self.group.setExclusive(True)
        for c in colors:
            btn = SwatchButton(c, c in config.ENABLED_COLORS)
            btn.setChecked(c == current)
            self.group.addButton(btn)
            row.addWidget(btn)
        lay.addLayout(row)
        self.group.buttonToggled.connect(self._on_toggled)

    def _on_toggled(self, btn, checked):
        if checked:
            self.view.set_color(btn.name)
            self.changed.emit()

    def color(self):
        return self.group.checkedButton().name


class ColorDialog(QtWidgets.QDialog):
    """Antes del primer paso: color de la placa y del perno."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Colores de la secuencia")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(32, 28, 32, 28)
        lay.setSpacing(8)
        lay.addWidget(W.label("¿De qué color son la placa y el perno?", size=theme.PS_FIGURE - 8, weight=600,
                              wrap=True))
        lay.addWidget(W.label("La visión busca cada objeto por su color (cfg/plate_colors.cfg). "
                              "Se aplica a esta secuencia sin reiniciar el servidor de visión.",
                              color=theme.MUTED, size=theme.FS_SECTION, wrap=True))
        lay.addSpacing(20)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(28)
        self.pickers = {}
        for (key, colors, _default, label), current in zip(KEYS, saved_colors()):
            if current not in config.ENABLED_COLORS:
                current = next(c for c in colors if c in config.ENABLED_COLORS)
            picker = ColorPicker(key, colors, label, current)
            picker.changed.connect(self._update_summary)
            row.addWidget(picker, 1)
            self.pickers[key] = picker
        lay.addLayout(row)
        lay.addSpacing(20)
        self.summary = W.label(size=theme.FS_SECTION)
        lay.addWidget(self.summary)
        lay.addSpacing(8)
        buttons = QtWidgets.QHBoxLayout()
        buttons.setSpacing(12)
        cancel = QtWidgets.QPushButton("Cancelar")
        cancel.setStyleSheet("min-height: 58px; min-width: 110px;")
        cancel.setAutoDefault(False)
        cancel.clicked.connect(self.reject)
        ok = QtWidgets.QPushButton("Iniciar secuencia")
        ok.setObjectName("next")
        ok.setDefault(True)
        ok.clicked.connect(self.accept)
        buttons.addWidget(cancel)
        buttons.addWidget(ok, 1)
        lay.addLayout(buttons)
        ok.setFocus()
        self.setMinimumWidth(820)
        self._update_summary()

    def _update_summary(self):
        plate, bolt = self.colors()
        name = lambda c: config.COLOR_NAMES.get(c, c).lower()
        plate_name = "roja" if plate == "red" else name(plate)
        self.summary.setText(f"La visión va a buscar la <b>placa {plate_name}</b> "
                             f"y el <b>perno {name(bolt)}</b>.")

    def colors(self):
        return self.pickers["plate_color"].color(), self.pickers["bolt_color"].color()
