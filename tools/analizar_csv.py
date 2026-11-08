#!/usr/bin/env python3
# Analiza un CSV de impedance_gen3 para tunear rigidez / rampa / friccion.
#
#   python3 tools/analizar_csv.py                 el CSV mas nuevo de build/
#   python3 tools/analizar_csv.py ARCHIVO.csv     uno en particular
#
# Por cada paso MOVER de la secuencia (task_status = 1, ejecutando):
#   dist      distancia del inicio al objetivo [cm]
#   dur       duracion del paso [s]
#   atraso    distancia media del robot DETRAS de la referencia de la rampa,
#             sobre la direccion del movimiento [cm] (< 0: va adelante)
#   lateral   desvio maximo de la linea recta inicio -> objetivo [cm]
#   oversh    cuanto se pasa del objetivo en la direccion del movimiento,
#             hasta 3 s despues del paso [mm]
#   final     error al terminar el paso [cm]
#   vib mov / vib quieto
#             parte rapida (> ~20 Hz) del torque mandado, J1-J4, RMS [Nm],
#             mientras la referencia se mueve / mientras esta quieta.
#             Sin rampa ni filtro, quieto: ~0.6 Nm (2026-10-16).
#
# La referencia de la rampa no esta en el CSV: se calcula como
# pose - pose_err (pose_err = p - p_d del controlador).
# Solo usa numpy.

import glob
import os
import sys

import numpy as np


def load(path):
    with open(path) as f:
        header = f.readline().strip().split(',')
    data = np.genfromtxt(path, delimiter=',', skip_header=1, invalid_raise=False)
    if data.ndim != 2 or len(data) < 100:
        sys.exit(f"{path}: sin datos")
    data = data[np.isfinite(data).all(1)]
    return {n: i for i, n in enumerate(header)}, data


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        files = sorted(glob.glob(os.path.join(here, '..', 'build', 'impedance_shadow_data_*.csv')),
                       key=os.path.getmtime)
        files = [f for f in files if os.path.getsize(f) > 10000]
        if not files:
            sys.exit("no hay CSV en build/")
        path = files[-1]

    ix, d = load(path)

    def col(name, j):
        return d[:, ix[f'{name}[{j}][0]']]

    def highpass(x, n=50):   # quita lo lento: x - media movil de 50 ms
        return x - np.convolve(x, np.ones(n) / n, 'same')

    t = d[:, ix['t']]
    step = d[:, ix['task_step']].astype(int)
    status = d[:, ix['task_status']].astype(int)
    pos = np.c_[col('pose', 0), col('pose', 1), col('pose', 2)]
    ref = pos - np.c_[col('pose_err', 0), col('pose_err', 1), col('pose_err', 2)]
    goal_all = np.c_[col('ref_target', 0), col('ref_target', 1), col('ref_target', 2)]
    vib = np.sqrt(sum(highpass(col('tau_hw', j)) ** 2 for j in range(4)))
    dt = np.diff(t, prepend=t[0] - 0.001)
    dt[dt <= 0] = 0.001
    ref_speed = np.r_[0.0, np.linalg.norm(np.diff(ref, axis=0), axis=1)] / dt

    def rms(x):
        return np.sqrt(np.mean(x ** 2)) if len(x) > 100 else float('nan')

    print(f"{os.path.basename(path)}: {t[-1] - t[0]:.0f} s, pasos {step.min()}-{step.max()}")
    print(f"vibracion global J1-J4: {rms(vib):.2f} Nm\n")
    print(" paso | dist cm |  dur s | atraso cm | lateral cm | oversh mm | final cm | vib mov / quieto Nm")
    for k in sorted(set(step)):
        idx = np.where((step == k) & (status == 1))[0]
        if len(idx) < 100:
            continue
        i0, i1 = idx[0], idx[-1]
        start, goal = pos[i0], goal_all[i1]
        length = np.linalg.norm(goal - start)
        if length < 0.01:
            continue   # no es un movimiento (pinza, espera, fixture...)
        u = (goal - start) / length
        moving = idx[ref_speed[idx] > 0.005]
        still = idx[ref_speed[idx] <= 0.001]
        # Sin rampa la referencia salta (1 muestra): el atraso no aplica.
        lag = (ref[moving] - pos[moving]) @ u if len(moving) > 100 else np.array([np.nan])
        if len(moving) <= 100:
            moving = np.array([], dtype=int)
        # Hasta 3 s despues, sin meterse en el siguiente paso que se ejecute.
        nxt = np.where((np.arange(len(t)) > i1) & (status == 1) & (step != k))[0]
        end = min(len(t) - 1, i1 + 3000, nxt[0] if len(nxt) else len(t) - 1)
        seg = pos[i0:end] - start
        lateral = np.max(np.linalg.norm(seg - np.outer(seg @ u, u), axis=1))
        overshoot = max(0.0, np.max((pos[i0:end] - goal) @ u))
        final = np.linalg.norm(pos[i1] - goal)
        print(f" {k:4} | {length * 100:7.1f} | {t[i1] - t[i0]:6.1f} | {np.mean(lag) * 100:9.2f} |"
              f" {lateral * 100:10.2f} | {overshoot * 1000:9.1f} | {final * 100:8.2f} |"
              f" {rms(vib[moving]):5.2f} / {rms(vib[still]):5.2f}")


if __name__ == '__main__':
    main()
