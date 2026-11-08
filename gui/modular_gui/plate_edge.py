"""Donde agarra la pinza a lo largo del canto de una placa parada.

La camara ve la placa desde arriba: su canto superior es una franja recta
y delgada. Con el agarre de GPD (la pinza cierra a lo ancho del grosor)
se mide cuanta placa queda a cada lado de la pinza, a lo largo del canto.

Pasos (todo en la base del robot):
  1. Franja del canto: puntos a menos de RADIUS del agarre, cuya altura
     esta a menos de BAND del punto mas alto (el canto es lo mas alto).
  2. Linea del canto: RANSAC en horizontal, con pares de puntos a menos de
     SEED_R del agarre; inliers a menos de SLAB de la linea. No se supone
     que la pinza cierra perpendicular al canto (GPD a veces la inclina).
  3. Direccion del canto u: componente principal (PCA) de la linea, con el
     signo de +y de la pinza. Se reporta el angulo entre u y el eje y de
     la pinza (0 = cierre perpendicular al canto) y a cuanto queda el
     agarre de la linea.
  4. s = (p - agarre) . u. Se busca el tramo continuo que contiene s = 0
     (huecos de hasta GAP). Sus extremos son el fin de la placa a cada
     lado: dist_neg (lado -y de la pinza) y dist_pos (lado +y).
  5. Si un extremo cae en el borde de la foto, la placa sigue fuera de la
     imagen: la distancia es un minimo ("al menos").
  6. Margen libre = distancia - medio ancho de los dedos (21.6 mm / 2).

Solo mide; no cambia el agarre ni mueve el robot.
"""

import numpy as np

from . import config, kinematics as kin

RADIUS = 0.30            # [m] puntos considerados alrededor del agarre
BAND = 0.015             # [m] altura de la franja del canto bajo el punto mas alto
SLAB = 0.006             # [m] distancia maxima a la linea del canto (grosor + ruido)
SEED_R = 0.03            # [m] la linea se busca con puntos a menos de esto del agarre
RANSAC_ITERS = 300
MAX_OFFSET = 0.015       # [m] distancia maxima del agarre a la linea del canto
GAP = 0.008              # [m] hueco maximo dentro del canto
FINGER_HALF = 0.0216 / 2  # [m] medio ancho de los dedos (hand_height de GPD)
EDGE_PX = 6               # [px] cercania al borde de la imagen = truncado
MIN_POINTS = 40


def measure(points_cam, tool_pose, grasp):
    """points_cam: nube en el frame de la camara (N x 3), como la guarda el
    screw_server. tool_pose: pose del TOOL al tomar la foto. grasp: dict
    con 'position' y 'rpy' (de la vision). Regresa un dict; 'ok' = False
    con 'why' si no se pudo medir."""
    out = {"ok": False}
    if grasp.get("position") is None or grasp.get("rpy") is None or len(points_cam) < MIN_POINTS:
        out["why"] = "sin agarre o nube vacía"
        return out
    pts_cam = np.asarray(points_cam, dtype=float)
    P = kin.cloud_to_base(pts_cam, tool_pose)
    tcp = np.asarray(grasp["position"], dtype=float)
    R = kin.rpy_to_matrix(grasp["rpy"])
    y_side, z_app = R[:, 1], R[:, 2]

    # 1. franja del canto (lo mas alto cerca del agarre)
    near = np.linalg.norm(P[:, :2] - tcp[:2], axis=1) < RADIUS
    if near.sum() < MIN_POINTS:
        out["why"] = "pocos puntos cerca del agarre"
        return out
    z_top = np.percentile(P[near, 2], 99.5)
    band = near & (P[:, 2] > z_top - BAND)
    # Punto de agarre sobre el canto: donde el eje de la pinza (aproximacion)
    # cruza la altura del canto. Con la pinza vertical es la TCP en planta;
    # inclinada, la TCP queda corrida en horizontal respecto a los dedos.
    edge_z = float(np.median(P[band, 2]))
    g = tcp.copy()
    if abs(z_app[2]) > 0.3:
        g = tcp + z_app * (edge_z - tcp[2]) / z_app[2]
    # 2. linea del canto: RANSAC en horizontal con pares de puntos cerca del
    #    agarre (no se supone que la pinza cierra perpendicular al canto)
    B = P[band]
    d_g = np.linalg.norm(B[:, :2] - g[:2], axis=1)
    seeds = np.flatnonzero(d_g < SEED_R)
    if len(seeds) < 2:
        out["why"] = "no hay canto de placa junto al agarre"
        return out
    rng = np.random.default_rng(0)
    best, best_n = None, 0
    for _ in range(RANSAC_ITERS):
        i, j = rng.choice(seeds, 2, replace=False)
        d = B[j, :2] - B[i, :2]
        L = np.linalg.norm(d)
        if L < 0.01:
            continue
        nrm = np.array([-d[1], d[0]]) / L
        inl = np.abs((B[:, :2] - B[i, :2]) @ nrm) < SLAB
        n = int(inl.sum())
        if n > best_n:
            best, best_n = inl, n
    if best is None or best_n < MIN_POINTS:
        out["why"] = "no se encontró una línea recta (canto) junto al agarre"
        return out
    E = B[best]

    # 3. direccion del canto (PCA de los puntos de la linea), con el signo de
    #    +y de la pinza
    xy = E[:, :2] - E[:, :2].mean(axis=0)
    w, V = np.linalg.eigh(xy.T @ xy)
    u = np.r_[V[:, -1], 0.0]
    if u @ y_side < 0:
        u = -u
    nrm = np.r_[-u[1], u[0], 0.0]
    off = float((g - E.mean(axis=0)) @ nrm)
    if abs(off) > MAX_OFFSET:
        out["why"] = f"el agarre queda a {abs(off) * 100:.1f} cm del canto"
        return out
    y_h = np.r_[y_side[:2], 0.0]
    y_h /= max(np.linalg.norm(y_h), 1e-9)
    angle = float(np.degrees(np.arccos(np.clip(abs(u @ y_h), -1, 1))))
    straight = float(np.sqrt(w[0] / max(len(E), 1)))   # ancho de la franja (desv. estandar)

    # 4. tramo continuo que contiene al agarre
    s = np.sort((E - g) @ u)
    if s[0] > GAP or s[-1] < -GAP:
        out["why"] = "el agarre queda fuera del canto de la placa"
        return out
    k0 = int(np.searchsorted(s, 0.0))
    lo = max(k0 - 1, 0)
    while lo > 0 and s[lo] - s[lo - 1] <= GAP:
        lo -= 1
    hi = min(k0, len(s) - 1)
    while hi < len(s) - 1 and s[hi + 1] - s[hi] <= GAP:
        hi += 1
    if abs(min(abs(s[max(k0 - 1, 0)]), abs(s[min(k0, len(s) - 1)]))) > GAP:
        out["why"] = "hay un hueco en el canto justo donde agarra"
        return out
    d_neg, d_pos = float(-s[lo]), float(s[hi])

    # 5. extremo en el borde de la IMAGEN (no de la nube: la nube casi solo
    #    tiene la placa) -> la placa sigue fuera de la foto
    W_px, H_px, fx, fy, cx, cy = config.DEPTH_INTRINSICS

    def at_border(p_base):
        x, y, z = kin.base_to_cloud(p_base[None, :], tool_pose)[0]
        if z <= 0:
            return True
        col = cx - fx * x / z      # depth_camera.h: x = -(z/fx)(col - cx)
        row = cy - fy * y / z      #                 y = -(z/fy)(row - cy)
        return bool(col < EDGE_PX or col > W_px - 1 - EDGE_PX or row < EDGE_PX or row > H_px - 1 - EDGE_PX)

    end_neg = g + u * s[lo]
    end_pos = g + u * s[hi]
    end_neg[2] = end_pos[2] = float(np.median(E[:, 2]))
    out.update(
        ok=True,
        dist_neg_m=d_neg, dist_pos_m=d_pos,
        free_neg_m=d_neg - FINGER_HALF, free_pos_m=d_pos - FINGER_HALF,
        trunc_neg=at_border(end_neg), trunc_pos=at_border(end_pos),
        length_m=d_neg + d_pos,
        angle_deg=angle, width_m=straight, n_points=int(len(E)), offset_m=off,
        end_neg=end_neg, end_pos=end_pos, u=u, edge_z=float(end_neg[2]),
        y_side=y_side,
    )
    return out


def describe(m):
    """Texto corto en espanol para la interfaz."""
    if not m.get("ok"):
        return "Canto de la placa: " + m.get("why", "no se pudo medir")

    def side(d, free, trunc):
        txt = f"{'al menos ' if trunc else ''}{d * 100:.1f} cm"
        return txt + (f" (libre {free * 100:.1f} cm)" if free > 0 else " (el dedo queda fuera)")

    return (f"Placa a cada lado de la pinza: lado −y {side(m['dist_neg_m'], m['free_neg_m'], m['trunc_neg'])}, "
            f"lado +y {side(m['dist_pos_m'], m['free_pos_m'], m['trunc_pos'])}. "
            f"Canto visible {m['length_m'] * 100:.1f} cm; cierre a {m['angle_deg']:.0f}° de perpendicular.")
