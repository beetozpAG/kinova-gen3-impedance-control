"""Prueba de plate_edge con una placa sintetica (respuesta conocida) y con
las fotos reales de la placa del 2026-10-19 si existen.
   .venv/bin/python tests/test_plate_edge.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np

from modular_gui import config, kinematics as kin, parsers, plate_edge as pe

fails = 0


def check(ok, what):
    global fails
    print(("  OK   " if ok else "  FALLA ") + what)
    fails += 0 if ok else 1


# Camara mirando hacia abajo en ARRIBA_PLACA1_NEW (pose del TOOL)
tool = np.array([0.338, -0.444, 0.291, 180.0, 0.0, 90.0])
rng = np.random.default_rng(1)


def plate_cloud(y0, y1, x=0.46, z=0.17, thick=0.003):
    """Canto de una placa parada a lo largo de y base, mas el soporte abajo."""
    n = 4000
    pts = np.c_[x + rng.uniform(-thick / 2, thick / 2, n), rng.uniform(y0, y1, n),
                z + rng.normal(0, 0.0008, n)]
    floor = np.c_[rng.uniform(0.40, 0.52, 1500), rng.uniform(-0.55, -0.33, 1500), np.full(1500, 0.08)]
    P = np.vstack([pts, floor])
    return kin.base_to_cloud(P, tool)


print("1) Placa sintetica de 20 cm, agarre a 6 cm de un extremo")
cam = plate_cloud(-0.52, -0.32)
for tilt in (0.0, 35.0):
    # pinza cerrando en x base (perpendicular al canto), y de la pinza = +y base;
    # inclinada 'tilt' grados alrededor de su eje y
    rpy = np.array([180.0, tilt, 180.0])       # inclina la aproximacion en x (sobre el canto)
    R = kin.rpy_to_matrix(rpy)
    contact = np.array([0.46, -0.46, 0.17])
    tcp = contact - R[:, 2] * 0.03            # la TCP queda 3 cm antes del canto sobre la aproximacion
    m = pe.measure(cam, tool, {"position": tcp, "rpy": rpy})
    ok = m.get("ok")
    check(ok, f"inclinacion {tilt:.0f}°: se midio ({m.get('why', '')})")
    if ok:
        neg, pos = sorted([m["dist_neg_m"], m["dist_pos_m"]])
        check(abs(neg - 0.06) < 0.004 and abs(pos - 0.14) < 0.004,
              f"  distancias {m['dist_neg_m'] * 100:.1f} / {m['dist_pos_m'] * 100:.1f} cm (esperado 6 / 14)")
        check(abs(m["length_m"] - 0.20) < 0.006, f"  canto {m['length_m'] * 100:.1f} cm (esperado 20)")
        check(not m["trunc_neg"] and not m["trunc_pos"], "  extremos dentro de la imagen")

print("2) Agarre lejos del canto -> no mide")
m = pe.measure(cam, tool, {"position": np.array([0.50, -0.46, 0.20]), "rpy": [180.0, 0.0, 180.0]})
check(not m.get("ok"), f"rechazado: {m.get('why')}")

print("3) Placa mas larga que la foto -> 'al menos' en ese lado")
long_cam = plate_cloud(-0.75, -0.32)
long_cam = long_cam[long_cam[:, 2] > 0]
W_px, H_px, fx, fy, cx, cy = config.DEPTH_INTRINSICS
col = cx - fx * long_cam[:, 0] / long_cam[:, 2]
row = cy - fy * long_cam[:, 1] / long_cam[:, 2]
long_cam = long_cam[(col >= 0) & (col < W_px) & (row >= 0) & (row < H_px)]   # lo que ve la camara
m = pe.measure(long_cam, tool, {"position": np.array([0.46, -0.46, 0.20]), "rpy": [180.0, 0.0, 180.0]})
check(m.get("ok") and (m["trunc_neg"] or m["trunc_pos"]) and not (m["trunc_neg"] and m["trunc_pos"]),
      f"un extremo marcado como corte de la foto (neg {m.get('trunc_neg')}, pos {m.get('trunc_pos')})")

print("4) Fotos reales (si existen)")
caps = {}
for f in sorted((config.GUI_DIR / "logs").glob("vision_*.log")):
    for c in parsers.parse_vision_log(f.read_text(errors="replace")):
        caps[c["name"]] = c
name = "screw_174350"
pcd = config.capture_path(name)
if pcd.exists() and caps.get(name, {}).get("grasps"):
    pts = parsers.read_pcd(pcd)
    tp = parsers.read_tool_pose(config.capture_path(name, "_pose.txt"))
    lengths = [pe.measure(pts, tp, g).get("length_m", np.nan) for g in caps[name]["grasps"][:3]]
    check(np.nanmax(lengths) - np.nanmin(lengths) < 0.01,
          f"{name}: mismo canto con 3 agarres distintos ({', '.join(f'{x * 100:.1f}' for x in lengths)} cm)")
else:
    print("     (no estan las fotos de la placa; se omite)")

print("FALLAS:", fails)
sys.exit(1 if fails else 0)
