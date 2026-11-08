"""Modulo del modular de cada paso y fase de la vista del hoyo (sin robot).
   .venv/bin/python tests/test_modules.py
"""
import json, os, subprocess, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from modular_gui import config, hole_view, modules

info = json.loads(subprocess.check_output([str(config.TASK_INFO_BIN)]))
steps = info["steps"]
fails = 0


def check(ok, what):
    global fails
    print(("  OK   " if ok else "  FAIL ") + what)
    fails += not ok


keys = {s["n"]: [k for k, _ in modules.step_modules(s)] for s in steps}
check(all(keys[n] for n in keys), f"los {len(steps)} pasos tienen al menos un modulo")
check(all(keys[s["n"]][0] == "electronica" and "control" in keys[s["n"]] for s in steps if s["type"] == "fixture"),
      "fixture: Electronicos (y Control en cascada)")
check(all(keys[s["n"]] == (["inteligentes", "control"] if s.get("vision_approach") else ["inteligentes"]) for s in steps if s["type"] == "vision"), "vision: Inteligentes (centrar: + Control)")
check(all(keys[s["n"]] == ["control", "inteligentes"] for s in steps if s["type"] == "move" and s["from_vision"]),
      "movimiento con referencia de la vision: Control + Inteligentes")
check(all(keys[s["n"]] == ["control"] for s in steps if s["type"] == "move" and not s["from_vision"]),
      "movimiento a pose fija: Control")
hole = next(s for s in steps if s.get("vision_needs_hole"))
check("hoyo" in modules.step_modules(hole)[0][1], "la foto del hoyo dice que elige el hoyo")
rng = hole_view.hole_phase(steps)
uses = [s["n"] for s in steps if s.get("vision_target") == "hole"]
check(rng is not None and rng[0] == hole["n"] and rng[1] >= max(uses), f"vista del hoyo de la foto ({rng[0]}) a soltar ({rng[1]})")
print("FALLAS:", fails)
sys.exit(1 if fails else 0)
