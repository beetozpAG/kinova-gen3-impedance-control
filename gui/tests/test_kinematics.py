"""FK del Gen3 contra las poses del laboratorio (app web). Correr:
   .venv/bin/python tests/test_kinematics.py
"""
import json, os, subprocess, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from modular_gui import kinematics as kin

here = os.path.dirname(os.path.abspath(__file__))
info = json.loads(subprocess.check_output([os.path.join(here, '..', '..', 'build', 'task_info')]))
worst_p = worst_r = 0.0
for p in info['poses']:
    if p['joints_deg'] is None:
        continue
    T = kin.frames(p['joints_deg'])[-1]
    dp = np.linalg.norm(T[:3, 3] - np.array(p['tcp_m']))
    Rw = kin.rpy_to_matrix(p['rpy_deg'])
    dr = np.degrees(np.arccos(np.clip((np.trace(Rw.T @ T[:3, :3]) - 1) / 2, -1, 1)))
    worst_p, worst_r = max(worst_p, dp), max(worst_r, dr)
    print(f"  {p['name']:26s} {dp*100:5.2f} cm  {dr:5.2f} grados")
ok = worst_p < 0.07 and worst_r < 7.0
print(("OK" if ok else "FALLA"), f"peor: {worst_p*100:.2f} cm, {worst_r:.2f} grados")
sys.exit(0 if ok else 1)
