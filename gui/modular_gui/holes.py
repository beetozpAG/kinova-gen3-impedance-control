"""Hoyos de la placa en una nube (foto encima del hoyo, 2026-10-21).

Mismo algoritmo y constantes que detectHoles() en
kinova_grasp_V2/src/screw_server.cpp (mantener los dos iguales):
  1. Plano de la placa: RANSAC sobre la nube (normal a menos de
     PLANE_MAX_TILT de la vertical, inliers a PLANE_TOL), afinado con PCA.
  2. Puntos de la placa (a PLATE_BAND del plano) en una rejilla de RES sobre
     el plano (ejes: x de la base proyectado y n x e1). Cierre de 1 celda
     (dilatar y erosionar, vecinos en cruz) para tapar huecos sueltos.
  3. Hoyos: regiones VACIAS (vecinos en cruz) que no tocan el borde de la
     rejilla y con area de un circulo entre MIN_DIAM y MAX_DIAM.
  4. Borde: anillo de 3 celdas alrededor de la region; en 72 sectores desde
     su centro, el punto de la placa mas cercano. Circulo por minimos
     cuadrados (Kasa). Se rechaza si se ve menos del 90 % del borde, si el
     residuo es grande o si el area no se parece a la del circulo.
Hoyo elegido: el MAS CHICO.
"""
import numpy as np

PLANE_TOL = 0.0015
PLANE_REFINE = 0.002
PLANE_MAX_TILT_DEG = 15.0
PLANE_ITERS = 300
PLATE_BAND = 0.003
RES = 0.001
# El hoyo real mas chico es de 30 mm y aqui se lee de 27 a 33 mm; los hoyos falsos
# (huecos de la nube) se leen de 13 a 24 mm y el controlador va al mas chico.
MIN_DIAM = 0.025
# Region vacia antes de ajustar el circulo: sale menor que el circulo si el borde es ruidoso.
MIN_REGION_DIAM = 0.010
MAX_DIAM = 0.050
RING_CELLS = 3
SECTORS = 72
MIN_SECTORS = 65
MAX_RMS_FRAC = 0.15
AREA_RATIO = (0.6, 1.3)


def _cross_dilate(m):
    out = m.copy()
    out[1:, :] |= m[:-1, :]
    out[:-1, :] |= m[1:, :]
    out[:, 1:] |= m[:, :-1]
    out[:, :-1] |= m[:, 1:]
    return out


def _cross_erode(m):
    # fuera de la rejilla cuenta como vacio
    out = m.copy()
    out[1:, :] &= m[:-1, :]
    out[:-1, :] &= m[1:, :]
    out[:, 1:] &= m[:, :-1]
    out[:, :-1] &= m[:, 1:]
    out[0, :] = out[-1, :] = False
    out[:, 0] = out[:, -1] = False
    return out


def _label(empty):
    """Regiones conectadas en cruz: lista de (filas, columnas)."""
    seen = np.zeros_like(empty)
    H, W = empty.shape
    regions = []
    for i0, j0 in zip(*np.nonzero(empty)):
        if seen[i0, j0]:
            continue
        stack = [(i0, j0)]
        seen[i0, j0] = True
        cells = []
        while stack:
            i, j = stack.pop()
            cells.append((i, j))
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < H and 0 <= b < W and empty[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    stack.append((a, b))
        regions.append(np.array(cells))
    return regions


def plate_plane(P, seed=0):
    """(punto, normal hacia arriba) del plano dominante casi horizontal, o None."""
    rng = np.random.default_rng(seed)
    if len(P) < 100:
        return None
    best_n, best = 0, None
    max_tilt = np.cos(np.radians(PLANE_MAX_TILT_DEG))
    for _ in range(PLANE_ITERS):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        L = np.linalg.norm(n)
        if L < 1e-9:
            continue
        n /= L
        if abs(n[2]) < max_tilt:
            continue
        k = int(np.count_nonzero(np.abs((P - a) @ n) < PLANE_TOL))
        if k > best_n:
            best_n, best = k, (a, n)
    if best is None:
        return None
    a, n = best
    inl = P[np.abs((P - a) @ n) < PLANE_REFINE]
    c = inl.mean(0)
    w, v = np.linalg.eigh((inl - c).T @ (inl - c))
    n = v[:, 0]
    if n[2] < 0:
        n = -n
    return c, n


def detect(P):
    """P: nube en la BASE (N x 3). Devuelve dict con 'ok', 'holes' (lista,
    del mas chico al mas grande: center, diameter_m, rms_m), 'why', 'tilt_deg'."""
    P = np.asarray(P, float)
    P = P[np.isfinite(P).all(1)]
    pl = plate_plane(P)
    if pl is None:
        return {"ok": False, "why": "no se encontro la cara de la placa", "holes": []}
    c, n = pl
    tilt = float(np.degrees(np.arccos(min(1.0, abs(n[2])))))
    e1 = np.array([1.0, 0.0, 0.0]) - n * n[0]
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    plate = P[np.abs((P - c) @ n) < PLATE_BAND]
    uv = np.c_[(plate - c) @ e1, (plate - c) @ e2]
    mn = uv.min(0)
    ij = np.floor((uv - mn) / RES).astype(int)
    shape = tuple(ij.max(0) + 1)
    occ = np.zeros(shape, bool)
    occ[ij[:, 0], ij[:, 1]] = True
    occ = _cross_erode(_cross_dilate(occ))
    holes = []
    min_cells = np.pi / 4 * (MIN_REGION_DIAM / RES) ** 2
    max_cells = np.pi / 4 * (MAX_DIAM / RES) ** 2
    for cells in _label(~occ):
        if len(cells) < min_cells or len(cells) > max_cells:
            continue
        if cells[:, 0].min() == 0 or cells[:, 1].min() == 0 or \
                cells[:, 0].max() == shape[0] - 1 or cells[:, 1].max() == shape[1] - 1:
            continue
        mask = np.zeros(shape, bool)
        mask[cells[:, 0], cells[:, 1]] = True
        ring = mask
        for _ in range(RING_CELLS):
            ring = _cross_dilate(ring)
        ring &= ~mask
        q = uv[ring[ij[:, 0], ij[:, 1]]]
        if len(q) < MIN_SECTORS:
            continue
        c0 = (cells.mean(0) + 0.5) * RES + mn
        ang = np.arctan2(q[:, 1] - c0[1], q[:, 0] - c0[0])
        r = np.linalg.norm(q - c0, axis=1)
        sector = np.clip(((ang + np.pi) / (2 * np.pi) * SECTORS).astype(int), 0, SECTORS - 1)
        edge = []
        for s in range(SECTORS):
            w = sector == s
            if w.any():
                edge.append(q[w][np.argmin(r[w])])
        if len(edge) < MIN_SECTORS:
            continue
        edge = np.array(edge)
        A = np.c_[2 * edge, np.ones(len(edge))]
        sol = np.linalg.lstsq(A, (edge ** 2).sum(1), rcond=None)[0]
        cc = sol[:2]
        R2 = sol[2] + cc @ cc
        if R2 <= 0:
            continue
        R = float(np.sqrt(R2))
        rms = float(np.sqrt(np.mean((np.linalg.norm(edge - cc, axis=1) - R) ** 2)))
        ratio = len(cells) * RES * RES / (np.pi * R2)
        if not (MIN_DIAM <= 2 * R <= MAX_DIAM) or rms > MAX_RMS_FRAC * R or \
                not (AREA_RATIO[0] <= ratio <= AREA_RATIO[1]):
            continue
        holes.append({"center": c + cc[0] * e1 + cc[1] * e2, "diameter_m": 2 * R, "rms_m": rms})
    holes.sort(key=lambda h: h["diameter_m"])
    return {"ok": bool(holes), "why": "" if holes else "no se encontro ningun hoyo redondo",
            "holes": holes, "tilt_deg": tilt, "plane_point": c, "normal": n}
