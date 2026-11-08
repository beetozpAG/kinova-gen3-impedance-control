"""Modulos del proyecto modular que intervienen en cada paso de la secuencia.

Los tres modulos registrados (Formato de Registro de Proyectos Modulares):
  - Sistemas de Control: control de impedancia del manipulador y control en
    cascada posicion-corriente del motor del fixture.
  - Sistemas Electronicos: electronica del fixture (ESP32, driver del motor,
    adquisicion y acondicionamiento de corriente y posicion).
  - Sistemas Inteligentes: percepcion RGB-D (GPD para agarres, canto de la
    placa, deteccion de hoyos) y referencias de movimiento desde la vision.

La asignacion se hace por tipo de paso (task_info): editar step_modules si se
quiere repartir distinto. Solo se muestra; no cambia nada del control.
"""

from . import theme

# clave -> (nombre, color)
MODULES = {
    "control": ("Sistemas de Control", theme.ACCENT),
    "electronica": ("Sistemas Electrónicos", "#b45309"),
    "inteligentes": ("Sistemas Inteligentes", "#7c3aed"),
}
ORDER = ("control", "electronica", "inteligentes")


def step_modules(step):
    """[(clave, que hace ese modulo en el paso), ...] del mas al menos directo."""
    if not step:
        return []
    t = step.get("type")
    target = step.get("vision_target")
    if t == "move" and not step.get("from_vision"):
        return [("control", "control de impedancia cartesiana: lleva la pinza a la pose")]
    if t == "move":
        why = {
            "pregrasp": "la pre-grasp sale del agarre que encontró GPD",
            "grasp": "la pose sale del agarre que encontró GPD",
            "plate_fixture": "pose del fixture corregida con el canto de la placa medido por visión",
            "hole": "x, y del hoyo más chico detectado en la nube de la placa",
        }.get(target, "referencia calculada a partir de la visión")
        return [("control", "control de impedancia cartesiana: lleva la pinza a la pose"),
                ("inteligentes", why)]
    if t == "vision" and step.get("vision_approach"):
        obj = "la placa" if step.get("vision_object") == "PLATE" else "el perno"
        return [("inteligentes", f"color + nube RGB-D: dónde está {obj} y si se ve completo (approach_object)"),
                ("control", "lleva la cámara 30 cm arriba del objeto, centrada")]
    if t == "vision":
        if step.get("vision_needs_hole"):
            what = "nube RGB-D de la placa: plano, huecos y círculo de cada hoyo; elige el más chico"
        elif step.get("vision_cloud_only"):
            what = "nube RGB-D de la escena (sin GPD)"
        elif step.get("vision_object") == "PLATE" or "placa" in step.get("label", "").lower():
            what = "nube RGB-D + GPD solo en la placa (por su color): agarre y medida de su canto"
        else:
            what = "nube RGB-D + GPD solo en el perno (por su color): agarre"
        return [("inteligentes", what)]
    if t == "fixture":
        return [("electronica", "ESP32, driver y sensores de corriente y posición del motor del fixture"),
                ("control", "control en cascada posición–corriente del fixture")]
    if t == "gripper":
        return [("control", "pinza Robotiq 2F-85: posición, velocidad y fuerza")]
    if t == "wait":
        return [("control", "sostiene la pose mientras se asienta el agarre")]
    return []


def names_html(step, sep=" &nbsp;·&nbsp; "):
    """'● Sistemas de Control · ● Sistemas Inteligentes' en HTML."""
    return sep.join(f"<span style='color:{MODULES[k][1]}'>●</span> {MODULES[k][0]}"
                    for k, _ in step_modules(step))


def detail_html(step):
    """Un renglon por modulo: '● Nombre — que hace'."""
    return "<br>".join(f"<span style='color:{MODULES[k][1]}'>●</span> <b>{MODULES[k][0]}</b>"
                       f" <span style='color:{theme.MUTED}'>— {what}</span>"
                       for k, what in step_modules(step))
