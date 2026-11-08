"""Escena 3D con VTK: robot Gen3 + pinza, poses, estela, nube de puntos y
agarre de la vision. Solo dibuja.

SceneContent tiene el renderer y los actores (se puede renderizar fuera de
pantalla para pruebas); Scene3D lo mete en un widget de Qt.
"""

import os

import numpy as np
import vtk
from vtkmodules.util import numpy_support
from PyQt5 import QtCore, QtWidgets

from . import config, kinematics as kin, theme

C = theme.hex_rgb


def _matrix(T):
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, float(T[i, j]))
    return m


def _actor(source_or_output, color, opacity=1.0, specular=0.3):
    mapper = vtk.vtkPolyDataMapper()
    if hasattr(source_or_output, "GetOutputPort"):
        mapper.SetInputConnection(source_or_output.GetOutputPort())
    else:
        mapper.SetInputData(source_or_output)
    a = vtk.vtkActor()
    a.SetMapper(mapper)
    p = a.GetProperty()
    p.SetColor(*C(color) if isinstance(color, str) else color)
    p.SetOpacity(opacity)
    p.SetSpecular(specular)
    p.SetSpecularPower(30)
    return a


class Tube:
    """Polilinea con grosor (estela, eslabones, flechas)."""

    def __init__(self, renderer, color, radius=0.004, opacity=1.0, sides=16):
        self.points = vtk.vtkPoints()
        self.lines = vtk.vtkCellArray()
        self.poly = vtk.vtkPolyData()
        self.poly.SetPoints(self.points)
        self.poly.SetLines(self.lines)
        self.tube = vtk.vtkTubeFilter()
        self.tube.SetInputData(self.poly)
        self.tube.SetRadius(radius)
        self.tube.SetNumberOfSides(sides)
        self.tube.CappingOn()
        self.actor = _actor(self.tube, color, opacity)
        renderer.AddActor(self.actor)

    def set(self, pts):
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        self.points.Reset()
        self.lines.Reset()
        if len(pts) >= 2:
            self.points.SetData(numpy_support.numpy_to_vtk(pts.copy(), deep=True))
            self.lines.InsertNextCell(len(pts))
            for i in range(len(pts)):
                self.lines.InsertCellPoint(i)
        self.points.Modified()
        self.lines.Modified()
        self.poly.Modified()

    def visible(self, on):
        self.actor.SetVisibility(bool(on))


class Frame:
    """Ejes x y z (rojo, verde, azul) de una pose."""

    def __init__(self, renderer, length=0.08, radius=0.004, opacity=1.0):
        self.length = length
        self.tubes = [Tube(renderer, c, radius, opacity, sides=10) for c in theme.XYZ]

    def set(self, T):
        o = T[:3, 3]
        for k, tube in enumerate(self.tubes):
            tube.set([o, o + T[:3, k] * self.length])

    def visible(self, on):
        for t in self.tubes:
            t.visible(on)


class Sphere:
    def __init__(self, renderer, color, radius=0.01, opacity=1.0):
        self.src = vtk.vtkSphereSource()
        self.src.SetRadius(radius)
        self.src.SetThetaResolution(24)
        self.src.SetPhiResolution(24)
        self.actor = _actor(self.src, color, opacity)
        renderer.AddActor(self.actor)

    def set(self, p):
        self.actor.SetPosition(*[float(x) for x in p])

    def visible(self, on):
        self.actor.SetVisibility(bool(on))


class Cube:
    """Marcador cubico: se distingue de las esferas por la forma, no solo
    por el color (la referencia de la rampa)."""

    def __init__(self, renderer, color, side=0.018, opacity=1.0):
        self.src = vtk.vtkCubeSource()
        self.src.SetXLength(side)
        self.src.SetYLength(side)
        self.src.SetZLength(side)
        self.actor = _actor(self.src, color, opacity)
        renderer.AddActor(self.actor)

    def set(self, p):
        self.actor.SetPosition(*[float(x) for x in p])

    def visible(self, on):
        self.actor.SetVisibility(bool(on))


class Gripper:
    """Robotiq 2F-85 simplificada en el frame de la INTERFAZ (z hacia afuera,
    dedos que cierran sobre x)."""

    def __init__(self, renderer, body=theme.ROBOT_JOINT, finger=theme.GRIPPER_FINGER, opacity=1.0):
        self.assembly = vtk.vtkAssembly()
        self.parts = []

        def box(xl, yl, zl, color):
            src = vtk.vtkCubeSource()
            src.SetXLength(xl)
            src.SetYLength(yl)
            src.SetZLength(zl)
            a = _actor(src, color, opacity)
            self.assembly.AddPart(a)
            return a

        # Acople + cuerpo (de la interfaz a la base de los dedos, ~0.088 m)
        coupling = vtk.vtkCylinderSource()
        coupling.SetRadius(0.038)
        coupling.SetHeight(0.02)
        coupling.SetResolution(40)
        c_actor = _actor(coupling, body, opacity)
        c_actor.SetOrientation(90, 0, 0)       # eje del cilindro (y) -> z
        c_actor.SetPosition(0, 0, 0.01)
        self.assembly.AddPart(c_actor)
        self.body = box(0.088, 0.045, 0.068, body)
        self.body.SetPosition(0, 0, 0.02 + 0.034)
        # Dedos: eslabon + almohadilla
        self.fingers = []
        for _ in range(2):
            link = box(0.012, 0.022, 0.045, body)
            pad = box(0.008, 0.022, 0.0375, finger)
            self.fingers.append((link, pad))
        self.set_opening(config.GRIPPER_MAX_OPENING_M)
        renderer.AddActor(self.assembly)

    def set_opening(self, opening_m):
        w = float(np.clip(opening_m, 0.0, config.GRIPPER_MAX_OPENING_M))
        tip = config.FINGERTIP_FROM_INT_M
        for side, (link, pad) in zip((-1, 1), self.fingers):
            x = side * (w / 2 + 0.004)
            pad.SetPosition(x, 0, tip - 0.0375 / 2)
            link.SetPosition(side * (w / 2 + 0.014), 0, 0.088 + 0.0225)

    def set_pose(self, T_int):
        self.assembly.SetUserMatrix(_matrix(T_int))

    def visible(self, on):
        self.assembly.SetVisibility(bool(on))


class Robot:
    """Gen3 7 GDL como tubos entre los frames DH + esferas en las
    articulaciones + pinza."""

    def __init__(self, renderer, opacity=1.0, ghost=False):
        link_c = theme.ROBOT_LINK if not ghost else theme.NEUTRAL
        joint_c = theme.ROBOT_JOINT if not ghost else theme.NEUTRAL
        self.links = Tube(renderer, link_c, radius=0.036, opacity=opacity, sides=32)
        self.joints = [Sphere(renderer, joint_c, radius=0.041, opacity=opacity) for _ in range(7)]
        base = vtk.vtkCylinderSource()
        base.SetRadius(0.06)
        base.SetHeight(0.06)
        base.SetResolution(48)
        self.base = _actor(base, link_c, opacity)
        self.base.SetOrientation(90, 0, 0)
        self.base.SetPosition(0, 0, 0.03)
        renderer.AddActor(self.base)
        self.gripper = Gripper(renderer, opacity=opacity)
        self.frames = None

    def set_joints(self, q_deg, gripper_pct=None):
        self.frames = kin.frames(q_deg)
        pts = [T[:3, 3] for T in self.frames[:8]]
        # sin puntos repetidos (el tubo no los dibuja bien)
        clean = [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - clean[-1]) > 1e-4:
                clean.append(p)
        self.links.set(clean)
        for k in range(7):
            self.joints[k].set(self.frames[k + 1][:3, 3])
        self.gripper.set_pose(self.frames[7])
        if gripper_pct is not None:
            self.gripper.set_opening(config.GRIPPER_MAX_OPENING_M * (1.0 - gripper_pct / 100.0))

    def visible(self, on):
        self.links.visible(on)
        self.base.SetVisibility(bool(on))
        for j in self.joints:
            j.visible(on)
        self.gripper.visible(on)


class PointCloud:
    def __init__(self, renderer, point_size=3.0):
        self.poly = vtk.vtkPolyData()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.poly)
        mapper.SetScalarModeToUsePointData()
        mapper.SetColorModeToDirectScalars()
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(mapper)
        self.actor.GetProperty().SetPointSize(point_size)
        self.actor.GetProperty().SetRenderPointsAsSpheres(True)
        renderer.AddActor(self.actor)

    def set(self, pts, colors=None):
        pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        vp = vtk.vtkPoints()
        vp.SetData(numpy_support.numpy_to_vtk(pts.copy(), deep=True))
        verts = vtk.vtkCellArray()
        n = len(pts)
        cells = np.empty((n, 2), dtype=np.int64)
        cells[:, 0] = 1
        cells[:, 1] = np.arange(n)
        verts.SetCells(n, numpy_support.numpy_to_vtkIdTypeArray(cells.ravel(), deep=True))
        self.poly.SetPoints(vp)
        self.poly.SetVerts(verts)
        if colors is None:
            colors = height_colors(pts[:, 2])
        arr = numpy_support.numpy_to_vtk(np.ascontiguousarray(colors, dtype=np.uint8), deep=True)
        arr.SetName("rgb")
        self.poly.GetPointData().SetScalars(arr)
        self.poly.Modified()

    def visible(self, on):
        self.actor.SetVisibility(bool(on))


def height_colors(z):
    """Color por altura en un solo tono: claro (abajo) -> acento -> oscuro (arriba)."""
    z = np.asarray(z, dtype=float)
    lo, hi = np.percentile(z, 2), np.percentile(z, 98)
    s = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)
    stops = np.array([C(theme.CLOUD_LOW), C(theme.ACCENT), C(theme.CLOUD_HIGH)])
    x = s * (len(stops) - 1)
    i = np.minimum(x.astype(int), len(stops) - 2)
    f = (x - i)[:, None]
    rgb = stops[i] * (1 - f) + stops[i + 1] * f
    return (rgb * 255).astype(np.uint8)


class Label:
    def __init__(self, renderer, text, color=theme.MUTED, size=13):
        self.actor = vtk.vtkBillboardTextActor3D()
        self.actor.SetInput(text)
        tp = self.actor.GetTextProperty()
        tp.SetColor(*C(color))
        tp.SetFontSize(size)
        tp.SetBold(False)
        if os.path.exists(theme.UI_FONT_FILE):
            tp.SetFontFamily(vtk.VTK_FONT_FILE)
            tp.SetFontFile(theme.UI_FONT_FILE)
        else:
            tp.SetFontFamilyToArial()
        tp.SetJustificationToCentered()
        renderer.AddActor(self.actor)

    def set(self, p, text=None):
        self.actor.SetPosition(*[float(x) for x in p])
        if text is not None:
            self.actor.SetInput(text)

    def visible(self, on):
        self.actor.SetVisibility(bool(on))


class SceneContent:
    """Renderer con piso, ejes de la base y utilidades."""

    def __init__(self):
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(*C(theme.RAISED))
        self.renderer.UseFXAAOn()
        self._floor()
        Frame(self.renderer, length=0.15, radius=0.005).set(np.eye(4))
        self.default_camera()

    def _floor(self, half=1.0, step=0.1):
        z = config.TABLE_Z_M
        plane = vtk.vtkPlaneSource()
        plane.SetOrigin(-0.4, -half, z - 0.001)
        plane.SetPoint1(1.2, -half, z - 0.001)
        plane.SetPoint2(-0.4, half, z - 0.001)
        floor = _actor(plane, theme.FLOOR, opacity=1.0, specular=0.0)
        floor.GetProperty().SetLighting(False)   # tono plano, sin sombra por el angulo
        self.renderer.AddActor(floor)
        grid = vtk.vtkAppendPolyData()
        for x in np.arange(-0.4, 1.2001, step):
            ln = vtk.vtkLineSource()
            ln.SetPoint1(x, -half, z)
            ln.SetPoint2(x, half, z)
            grid.AddInputConnection(ln.GetOutputPort())
        for y in np.arange(-half, half + 0.0001, step):
            ln = vtk.vtkLineSource()
            ln.SetPoint1(-0.4, y, z)
            ln.SetPoint2(1.2, y, z)
            grid.AddInputConnection(ln.GetOutputPort())
        a = _actor(grid, theme.FLOOR_GRID, opacity=1.0, specular=0.0)
        a.GetProperty().SetLighting(False)
        a.GetProperty().SetLineWidth(1.0)
        self.renderer.AddActor(a)

    def workspace_box(self, lo, hi, color=theme.NEUTRAL):
        src = vtk.vtkOutlineSource()
        src.SetBounds(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
        a = _actor(src, color, opacity=0.7, specular=0.0)
        a.GetProperty().SetLineWidth(1.5)
        self.renderer.AddActor(a)
        return a

    def default_camera(self, focal=(0.33, 0.05, 0.22), position=(1.3, -1.05, 0.88)):
        cam = self.renderer.GetActiveCamera()
        cam.SetFocalPoint(*focal)
        cam.SetPosition(*position)
        cam.SetViewUp(0, 0, 1)
        cam.SetViewAngle(32)
        self.renderer.ResetCameraClippingRange()

    def fit(self, pts, direction=(1.0, -1.0, 0.8), margin=1.6, min_ext=0.25, center=None):
        """Encuadra unos puntos desde una direccion (centro: el de la caja de
        los puntos, o el que se pase)."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        c = 0.5 * (pts.min(axis=0) + pts.max(axis=0)) if center is None else np.asarray(center, dtype=float)
        ext = max(min_ext, float(np.max(np.linalg.norm(pts - c, axis=1))))
        d = np.asarray(direction, dtype=float)
        d /= np.linalg.norm(d)
        self.default_camera(focal=tuple(c), position=tuple(c + d * ext * margin * 1.5))

    def frame_robot(self, lo=None, hi=None):
        """Encuadra el robot completo (base hasta ~0.9 m) y, si se pasa, la
        caja de trabajo."""
        pts = [[0, 0, config.TABLE_Z_M], [0, 0, 0.9], [0.45, 0, 0.3]]
        if lo is not None and hi is not None:
            pts += [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        self.fit(np.array(pts, dtype=float), direction=(1.0, -0.8, 0.55), margin=1.05)

    def snapshot(self, path, size=(1280, 800)):
        """Render fuera de pantalla a PNG (para pruebas)."""
        win = vtk.vtkRenderWindow()
        win.SetOffScreenRendering(1)
        win.SetSize(*size)
        win.AddRenderer(self.renderer)
        win.Render()
        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(win)
        w2i.Update()
        png = vtk.vtkPNGWriter()
        png.SetFileName(str(path))
        png.SetInputConnection(w2i.GetOutputPort())
        png.Write()
        win.RemoveRenderer(self.renderer)
        win.Finalize()


class Scene3D(QtWidgets.QWidget):
    """Widget de Qt con una SceneContent adentro."""

    def __init__(self, content, parent=None):
        super().__init__(parent)
        self.content = content
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.vtk_widget = None
        if os.environ.get("MODULAR_GUI_NO3D"):
            lbl = QtWidgets.QLabel("Vista 3D")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setStyleSheet(f"background: {theme.RAISED}; color: {theme.MUTED}; border-radius: 6px;")
            lay.addWidget(lbl)
            return
        from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        lay.addWidget(self.vtk_widget)
        win = self.vtk_widget.GetRenderWindow()
        win.AddRenderer(content.renderer)
        win.SetMultiSamples(4)
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.vtk_widget.SetInteractorStyle(style)
        self.vtk_widget.Initialize()
        self._dirty = False
        self._gl_checked = False
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._render_if_dirty)
        self._timer.start(33)

    def request_render(self):
        self._dirty = True

    def _check_software_gl(self):
        """Sin tarjeta de video (llvmpipe) el antialiasing deja la vista en negro."""
        self._gl_checked = True
        win = self.vtk_widget.GetRenderWindow()
        win.Render()
        if any(name in win.ReportCapabilities() for name in ("llvmpipe", "softpipe")):
            win.SetMultiSamples(0)
            self._dirty = True

    def _render_if_dirty(self):
        if self.vtk_widget is not None and self.isVisible() and not self._gl_checked:
            self._check_software_gl()
        if self._dirty and self.vtk_widget is not None and self.isVisible():
            self._dirty = False
            self.vtk_widget.GetRenderWindow().Render()

    def closeEvent(self, ev):
        if self.vtk_widget is not None:
            self.vtk_widget.Finalize()
        super().closeEvent(ev)
