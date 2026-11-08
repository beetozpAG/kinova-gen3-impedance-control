"""Controlador FALSO para probar la interfaz sin robot (solo pruebas).

Imprime el tablero con el mismo formato que impedance_gen3 cada 0.2 s y
escucha 'NEXT', 'PAUSE', 'AUTO' y 'STEP' en UDP 127.0.0.1:5011 (como el controlador
real; AUTOMATICO por defecto). Mueve unas
articulaciones de mentira entre las poses de la secuencia. No toca el robot.
"""
import json, math, os, signal, socket, subprocess, sys, time
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..'))
from modular_gui import kinematics as kin

info = json.loads(subprocess.check_output([os.path.join(here, '..', '..', 'build', 'task_info')]))
steps = info['steps']; poses = {p['name']: p for p in info['poses']}
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 5011)); sock.setblocking(False)
stop = False
def on_int(*_):
    global stop; stop = True
signal.signal(signal.SIGINT, on_int)

print("Conectando al robot FALSO...\nModo: SIMULACION (tests/fake_controller.py)", flush=True)
q = np.array(poses['HOME_MOD']['joints_deg'], float); q_goal = q.copy()
idx = info['first_step'] - 1; status = 'waiting'; t0 = time.time(); step_t0 = t0; grip = 0.0
auto = True
operator_paused = False
def start_step():
    global status, step_t0, q_goal
    status = 'running'; step_t0 = time.time()
    s = steps[idx]
    if s['type'] == 'move' and s['pose'] in poses and poses[s['pose']]['joints_deg']:
        q_goal = np.array(poses[s['pose']]['joints_deg'], float)
while not stop:
    try:
        while True:
            data = sock.recv(64)
            if data == b'NEXT':
                print("\nGUI: 'Siguiente paso'", flush=True)
                if status in ('waiting', 'paused'):
                    operator_paused = False
                    start_step()
            elif data == b'PAUSE':
                print("\nGUI: 'Pausa'", flush=True)
                if status in ('running', 'waiting'):
                    status = 'paused'; operator_paused = True; q_goal = q.copy()
            elif data in (b'AUTO', b'STEP') and auto != (data == b'AUTO'):
                auto = data == b'AUTO'
                print("\nGUI: modo " + ("AUTOMATICO" if auto else "PASO A PASO"), flush=True)
    except BlockingIOError:
        pass
    s = steps[idx]
    d = (q_goal - q + 180) % 360 - 180
    q = q + np.clip(d, -2.0, 2.0)
    if status == 'running':
        if s['type'] == 'gripper':
            grip += np.clip(s['gripper_pct'] - grip, -8, 8)
        if (s['type'] != 'move' or np.abs(d).max() < 0.5) and time.time() - step_t0 > 1.0:
            idx += 1
            if idx >= len(steps):
                status = 'done'; idx = len(steps) - 1
            elif auto:
                start_step()
            else:
                status = 'waiting'
    T = kin.frames(q)[7]; R = T[:3, :3]
    w = math.sqrt(max(1e-9, 1 + R[0,0] + R[1,1] + R[2,2])) / 2
    quat = [w, (R[2,1]-R[1,2])/(4*w), (R[0,2]-R[2,0])/(4*w), (R[1,0]-R[0,1])/(4*w)]
    p = T[:3, 3]; tgt = kin.frames(q_goal)[7][:3, 3]
    v = lambda a: ", ".join(f"{x:.4f}" for x in a)
    names = {'waiting': "ESPERANDO 'SIGUIENTE PASO'", 'running': 'EJECUTANDO', 'paused': 'PAUSADA',
             'done': 'TERMINADA'}
    msg = (f"PAUSA (operador) en el paso {idx+1} {s['describe']}: el brazo se queda donde esta. "
           "'Siguiente paso' reanuda este paso." if operator_paused else f"Paso {idx+1}: {s['label']}")
    print("\x1b[2J\x1b[H" + f"""============================================================
 Time                    : {time.time()-t0:.4f} s
 Hardware blend          : 100.0000 %
------------------------------------------------------------

REFERENCE TARGET [x y z qw qx qy qz]
{v(list(tgt) + quat)}

POSE [x y z qw qx qy qz]
{v(list(p) + quat)}

TORQUE ACTUALLY USED FOR HARDWARE [Nm]
{v(np.sin(np.arange(7) + time.time()) * 3 - [0, 15, 3, 8, 1, 1, 1])}
DELTA XI [dx dy dz dRx dRy dRz]:  {v([0.002, -0.001, 0.001, 0, 0, 0])}
 Estado                  : {names[status]}
 Modo                    : {'AUTOMATICO' if auto else 'PASO A PASO'}
 Paso                    : {idx+1}/{len(steps)}  {s['describe']}
 Tiempo en el paso       : {time.time()-step_t0:.3f} s
 Error de posicion       : {np.linalg.norm(p-tgt)*100:.3f} cm (tol 1.500)
 Error de orientacion    : 0.800 grados (tol 5.000)
 Mensaje                 : {msg}
 Gripper                 : {grip:.2f} % (command {grip:.2f} %)
 Fixture                 : POSICION, q = -90.00 deg, target -90.00 deg (arrived)
============================================================

JOINTS [deg]: {' '.join(f'{x:.3f}' for x in q)}
""", end="", flush=True)
    time.sleep(0.2)
print("\nStopping experiment...\nCSV guardado (falso)", flush=True)
