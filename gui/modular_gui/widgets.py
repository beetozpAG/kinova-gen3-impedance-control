"""Componentes visuales: tarjetas, estados, cifras (KPI), lista de pasos,
visor de log y graficas (pyqtgraph)."""

import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from . import modules, theme

pg.setConfigOptions(antialias=True, background=theme.SHEET, foreground=theme.MUTED)


class Card(QtWidgets.QFrame):
    """Tarjeta blanca con borde de 1 px. Titulo a la izquierda y, si se
    pasa, algo a la derecha del titulo (header_right)."""

    def __init__(self, title=None, parent=None, margins=20, meta=None):
        super().__init__(parent)
        self.setObjectName("card")
        self.lay = QtWidgets.QVBoxLayout(self)
        self.lay.setContentsMargins(margins, margins - 4 if title else margins, margins, margins)
        self.lay.setSpacing(12)
        self.title_label = None
        self.header = None
        if title:
            self.header = QtWidgets.QHBoxLayout()
            self.header.setSpacing(12)
            t = QtWidgets.QLabel(title)
            t.setObjectName("cardTitle")
            self.header.addWidget(t)
            self.title_label = t
            self.meta = QtWidgets.QLabel(meta or "")
            self.meta.setObjectName("cardMeta")
            self.header.addWidget(self.meta)
            self.header.addStretch(1)
            self.lay.addLayout(self.header)

    def add(self, w, stretch=0):
        if isinstance(w, QtWidgets.QLayout):
            self.lay.addLayout(w, stretch)
        else:
            self.lay.addWidget(w, stretch)
        return w

    def header_right(self, w):
        if self.header is not None:
            self.header.addWidget(w)
        return w


def label(text="", obj=None, color=None, size=None, bold=False, mono=False, wrap=False, weight=None):
    """size en px."""
    lbl = QtWidgets.QLabel(text)
    if obj:
        lbl.setObjectName(obj)
    css = []
    if color:
        css.append(f"color: {color};")
    if size:
        css.append(f"font-size: {size}px;")
    if bold or weight:
        css.append(f"font-weight: {weight or 600};")
    if mono:
        css.append(f'font-family: "{theme.MONO_FONT}";')
    if css:
        lbl.setStyleSheet(" ".join(css))
    lbl.setWordWrap(wrap)
    return lbl


def vline():
    f = QtWidgets.QFrame()
    f.setObjectName("divider")
    f.setFixedWidth(1)
    return f


def hline():
    f = QtWidgets.QFrame()
    f.setObjectName("divider")
    f.setFixedHeight(1)
    return f


class Chip(QtWidgets.QWidget):
    """Estado: etiqueta gris arriba y '● valor' abajo. El punto lleva el color
    del estado; el texto queda en tinta."""

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.title = title
        self.text = "—"
        self.color = theme.NEUTRAL
        self.setMinimumHeight(40)
        self.setMinimumWidth(120)
        self.setAccessibleName(title)

    def set(self, text, color, pulse=False):   # noqa: ARG002 (pulse: ya no parpadea)
        if (text, color) != (self.text, self.color):
            self.text, self.color = text, color
            self.setAccessibleDescription(text)
            self.updateGeometry()
            self.update()

    def sizeHint(self):
        fm = QtGui.QFontMetrics(theme.font(theme.FS_BODY, 500))
        return QtCore.QSize(max(120, fm.horizontalAdvance(self.text) + 30), 40)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = QtCore.QRectF(self.rect())
        p.setFont(theme.font(theme.FS_SMALL))
        p.setPen(QtGui.QColor(theme.MUTED))
        p.drawText(QtCore.QRectF(0, 0, r.width(), 17), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, self.title)
        p.setBrush(QtGui.QColor(self.color))
        p.setPen(QtCore.Qt.NoPen)
        p.drawEllipse(QtCore.QPointF(4, 29), 4, 4)
        p.setFont(theme.font(theme.FS_BODY, 500))
        p.setPen(QtGui.QColor(theme.TEXT))
        fm = QtGui.QFontMetrics(p.font())
        p.drawText(QtCore.QRectF(14, 19, r.width() - 14, 21), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                   fm.elidedText(self.text, QtCore.Qt.ElideRight, int(r.width() - 14)))


class Gauge(QtWidgets.QWidget):
    """Cifra: etiqueta, valor grande con unidad gris, una barra delgada con la
    proporcion real (y una marca en la tolerancia) y una nota abajo."""

    def __init__(self, title, unit, vmax, color=None, decimals=1, tol=None, parent=None):
        super().__init__(parent)
        self.title, self.unit, self.vmax = title, unit, vmax
        self.decimals, self.tol = decimals, tol
        self.value = None
        self.sub = ""
        self.value_color = theme.TEXT
        self.setMinimumSize(86, 100)
        self.setAccessibleName(title)

    def set(self, value, sub=None, color=None):
        self.value = value
        if sub is not None:
            self.sub = sub
        # solo se colorea la cifra cuando es una mala noticia
        self.value_color = color if color in (theme.NEGATIVE, theme.WARNING) else theme.TEXT
        self.setAccessibleDescription("—" if value is None else
                                      f"{value:.{self.decimals}f} {self.unit} {self.sub}".strip())
        self.update()

    def sizeHint(self):
        return QtCore.QSize(150, 104)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w = self.width()
        p.setFont(theme.font(theme.FS_SMALL))
        p.setPen(QtGui.QColor(theme.MUTED))
        p.drawText(QtCore.QRectF(0, 0, w, 18), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, self.title)
        # valor + unidad
        fv = theme.font(24)
        fv.setStyleStrategy(QtGui.QFont.PreferAntialias)
        p.setFont(fv)
        txt = "—" if self.value is None else f"{self.value:.{self.decimals}f}"
        p.setPen(QtGui.QColor(self.value_color if self.value is not None else theme.FAINT))
        fm = QtGui.QFontMetrics(fv)
        p.drawText(QtCore.QRectF(0, 20, w, 38), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, txt)
        if self.value is not None:
            p.setFont(theme.font(theme.FS_BODY))
            p.setPen(QtGui.QColor(theme.MUTED))
            x = fm.horizontalAdvance(txt) + 5
            p.drawText(QtCore.QRectF(x, 28, w - x, 26), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, self.unit)
        # barra
        y = 64
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(theme.ACTIVE))
        p.drawRoundedRect(QtCore.QRectF(0, y, w - 8, 4), 2, 2)
        if self.value is not None and self.vmax:
            frac = max(0.0, min(1.0, abs(self.value) / self.vmax))
            p.setBrush(QtGui.QColor(self.value_color if self.value_color != theme.TEXT else theme.ACCENT))
            if frac > 0:
                p.drawRoundedRect(QtCore.QRectF(0, y, max(4.0, (w - 8) * frac), 4), 2, 2)
        if self.tol is not None and self.vmax:
            x = (w - 8) * min(1.0, self.tol / self.vmax)
            p.setPen(QtGui.QPen(QtGui.QColor(theme.TEXT), 1))
            p.drawLine(QtCore.QPointF(x, y - 3), QtCore.QPointF(x, y + 7))
        # nota
        p.setFont(theme.font(theme.FS_SMALL))
        p.setPen(QtGui.QColor(theme.MUTED))
        sub = self.sub or (f"tol. {self.tol:g} {self.unit}" if self.tol is not None else "")
        p.drawText(QtCore.QRectF(0, 76, w, 18), QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                   QtGui.QFontMetrics(p.font()).elidedText(sub, QtCore.Qt.ElideRight, w))


def kpi_strip(cells):
    """Un contenedor partido por lineas de 1 px en celdas iguales."""
    w = QtWidgets.QWidget()
    h = QtWidgets.QHBoxLayout(w)
    h.setContentsMargins(0, 0, 0, 0)
    h.setSpacing(16)
    for i, c in enumerate(cells):
        if i:
            h.addWidget(vline())
        h.addWidget(c, 1)
    return w


TYPE_TEXT = {"move": "Mover", "gripper": "Pinza", "fixture": "Fixture", "wait": "Esperar", "vision": "Visión"}


class StepList(QtWidgets.QWidget):
    """Lista de la secuencia con el paso actual resaltado."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.steps = []
        self.current = None      # numero (1..N)
        self.status = None
        self.first = 1
        self.row_h = 34
        self.setMinimumWidth(320)
        self.setAccessibleName("Secuencia")

    def set_steps(self, steps, first_step=1):
        self.steps = steps
        self.first = first_step
        self.setMinimumHeight(len(steps) * self.row_h + 4)
        self.update()

    def set_current(self, n, status):
        if (n, status) != (self.current, self.status):
            self.current, self.status = n, status
            cur = next((s for s in self.steps if s["n"] == n), None)
            self.setAccessibleDescription(f"Paso {n}: {cur['label']}" if cur else "")
            self.update()

    def event(self, ev):
        if ev.type() == QtCore.QEvent.ToolTip:
            i = int(ev.pos().y() // self.row_h)
            if 0 <= i < len(self.steps):
                s = self.steps[i]
                mods = ", ".join(modules.MODULES[k][0] for k, _ in modules.step_modules(s))
                QtWidgets.QToolTip.showText(ev.globalPos(), f"{s['n']}. {s['label']}"
                                            + (f"  ({s['pose']})" if s.get("pose") else "")
                                            + (f"\nMódulo: {mods}" if mods else ""), self)
            return True
        return super().event(ev)

    def current_y(self):
        return ((self.current or 1) - 1) * self.row_h

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w = self.width()
        f_num = theme.font(theme.FS_SMALL)
        f_num.setStyleHint(QtGui.QFont.Monospace)
        for i, s in enumerate(self.steps):
            n = s["n"]
            y = i * self.row_h
            r = QtCore.QRectF(0, y + 1, w - 2, self.row_h - 2)
            skipped = n < self.first
            is_cur = n == self.current
            done = self.current is not None and n < self.current and not skipped
            if self.status == "done" and self.current is not None and n <= self.current:
                done, is_cur = True, False
            if is_cur:
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor(theme.ACTIVE))
                p.drawRoundedRect(r, 6, 6)
                p.setBrush(QtGui.QColor(theme.ACCENT))
                p.drawRoundedRect(QtCore.QRectF(r.left(), r.top() + 7, 3, r.height() - 14), 1.5, 1.5)
            else:
                p.setPen(QtGui.QPen(QtGui.QColor(theme.RULE), 1))
                p.drawLine(QtCore.QPointF(8, y + self.row_h - 0.5), QtCore.QPointF(w - 8, y + self.row_h - 0.5))
            # numero (dato: el numero de paso del controlador)
            p.setFont(f_num)
            p.setPen(QtGui.QColor(theme.MUTED if not is_cur else theme.TEXT))
            p.drawText(QtCore.QRectF(r.left() + 10, y, 24, self.row_h),
                       QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, str(n))
            # hecho
            if done:
                p.setPen(QtGui.QPen(QtGui.QColor(theme.POSITIVE), 1.6, cap=QtCore.Qt.RoundCap,
                                    join=QtCore.Qt.RoundJoin))
                cx, cy = r.left() + 47, y + self.row_h / 2
                path = QtGui.QPainterPath(QtCore.QPointF(cx - 4, cy))
                path.lineTo(cx - 1, cy + 3)
                path.lineTo(cx + 5, cy - 4)
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawPath(path)
            # detalle a la derecha (gris)
            extra = ""
            if s["type"] == "gripper":
                extra = f"{s['gripper_pct']:.0f} %"
            elif s["type"] == "fixture":
                extra = f"{s['fixture_deg']:.0f}°"
            elif s["type"] == "wait":
                extra = f"{s['wait_s']:.1f} s"
            elif s["type"] == "move" and s.get("vision_target") == "plate_fixture":
                extra = s["pose"] + " corregida"
            elif s["type"] == "move" and not s["from_vision"]:
                extra = s["pose"]
            elif s["type"] == "move":
                extra = "visión"
            elif s["type"] == "vision":
                extra = "foto"
            if is_cur and self.status in ("paused", "aborted"):
                extra = "Pausa" if self.status == "paused" else "Abortada"
            fe = theme.font(theme.FS_SMALL, 500 if is_cur and self.status in ("paused", "aborted") else 400)
            p.setFont(fe)
            fme = QtGui.QFontMetrics(fe)
            ew = min(fme.horizontalAdvance(extra) + 4, int(w * 0.42))
            ecol = theme.MUTED
            if is_cur and self.status == "paused":
                ecol = theme.WARNING
            elif is_cur and self.status == "aborted":
                ecol = theme.NEGATIVE
            p.setPen(QtGui.QColor(ecol))
            p.drawText(QtCore.QRectF(w - 12 - ew, y, ew, self.row_h), QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                       fme.elidedText(extra, QtCore.Qt.ElideMiddle, ew))
            # modulos del modular que usa el paso: un punto de color por modulo
            mods = [k for k, _ in modules.step_modules(s)]
            xd = w - 12 - ew - 10
            p.setPen(QtCore.Qt.NoPen)
            for k in reversed(mods):
                col = QtGui.QColor(modules.MODULES[k][1])
                if skipped or done:
                    col.setAlpha(110)
                p.setBrush(col)
                p.drawEllipse(QtCore.QPointF(xd, y + self.row_h / 2), 3.6, 3.6)
                xd -= 11
            # texto
            tf = theme.font(theme.FS_BODY, 500 if is_cur else 400)
            p.setFont(tf)
            tc = theme.TEXT if (is_cur or not (skipped or done)) else theme.MUTED
            p.setPen(QtGui.QColor(tc))
            fm = QtGui.QFontMetrics(tf)
            x0 = r.left() + 62
            avail = int(xd + 4 - 8 - x0)
            p.drawText(QtCore.QRectF(x0, y, avail, self.row_h), QtCore.Qt.AlignVCenter,
                       fm.elidedText(s["label"], QtCore.Qt.ElideRight, avail))


class LogView(QtWidgets.QPlainTextEdit):
    def __init__(self, max_lines=4000, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(max_lines)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)

    def append_lines(self, lines):
        if not lines:
            return
        bar = self.verticalScrollBar()
        at_end = bar.value() >= bar.maximum() - 4
        self.appendPlainText("\n".join(lines))
        if at_end:
            bar.setValue(bar.maximum())


def plot(title=None, ylabel=None, height=None):
    w = pg.PlotWidget()
    w.setBackground(theme.SHEET)
    pi = w.getPlotItem()
    pi.showGrid(x=False, y=True, alpha=0.35)
    for ax in ("left", "bottom"):
        a = pi.getAxis(ax)
        a.setPen(pg.mkPen(theme.BORDER))
        a.setTextPen(pg.mkPen(theme.MUTED))
        a.setStyle(tickFont=theme.font(theme.FS_SMALL), tickLength=0)
    pi.getAxis("left").setWidth(38)
    if title:
        pi.setTitle(f"<span style='color:{theme.TEXT};font-size:{theme.FS_BODY}px;"
                    f"font-family:\"{theme.UI_FONT}\";font-weight:500'>{title}</span>", justify="left")
    if ylabel:
        pi.setLabel("left", f"<span style='color:{theme.MUTED}'>{ylabel}</span>")
    if height:
        w.setMinimumHeight(height)
    w.setMenuEnabled(False)
    return w


def pen(color, width=2.0, style=None):
    kw = {"color": color, "width": width}
    if style == "dash":
        kw["style"] = QtCore.Qt.DashLine
    elif style == "dot":
        kw["style"] = QtCore.Qt.DotLine
    return pg.mkPen(**kw)


def swatch(color, kind="line"):
    """Muestra de leyenda en HTML: la misma marca que en la grafica."""
    glyph = {"line": "━", "dash": "╍", "dot": "┈", "dot1": "●", "box": "■"}[kind]
    return f"<span style='color:{color}'>{glyph}</span>"


class Kpi(QtWidgets.QWidget):
    """Cifra sin barra: etiqueta gris y valor. Una mala noticia (WARNING/
    NEGATIVE) lleva color Y peso, para no depender solo del color."""

    def __init__(self, title, parent=None):
        super().__init__(parent)
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(2)
        self.t = label(title, obj="kpiLabel")
        self.v = label("—")
        self._style(theme.FAINT)
        v.addWidget(self.t)
        v.addWidget(self.v)

    def _style(self, color, weight=400):
        self.v.setStyleSheet(f"font-size: 22px; color: {color}; font-weight: {weight};")

    def set(self, text, color=None, pulse=False):   # noqa: ARG002 (misma firma que Chip)
        self.v.setText(text)
        bad = color in (theme.NEGATIVE, theme.WARNING)
        self._style(color if bad else theme.TEXT, 600 if bad else 400)
        self.setAccessibleDescription(text)
