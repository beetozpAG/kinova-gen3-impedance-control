"""Cinematica directa del Kinova Gen3 7 GDL (solo para dibujar).

Parametros DH clasicos de la guia de usuario del Gen3 (r12.0, tabla de DH
clasicos), hasta la INTERFAZ (brida). La TCP de la app web esta 0.12 m mas
adelante sobre el eje z de la herramienta (Tool configuration).
Validado contra las poses del laboratorio (articulaciones de la app web ->
TCP de la app web): ver gui/tests/test_kinematics.py.
"""

import numpy as np

# (alpha, a, d, theta_offset) por articulacion; antes, la base: Rx(pi).
_DH = [
    (np.pi / 2, 0.0, -(0.1564 + 0.1284), 0.0),
    (np.pi / 2, 0.0, -(0.0054 + 0.0064), np.pi),
    (np.pi / 2, 0.0, -(0.2104 + 0.2104), np.pi),
    (np.pi / 2, 0.0, -(0.0064 + 0.0064), np.pi),
    (np.pi / 2, 0.0, -(0.2084 + 0.1059), np.pi),
    (np.pi / 2, 0.0, 0.0, np.pi),
    (np.pi, 0.0, -(0.1059 + 0.0615), np.pi),
]

TCP_OFFSET_M = 0.12


def _dh(alpha, a, d, theta):
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def frames(q_deg):
    """Lista de 9 transformaciones 4x4 en la base: base, J1..J7 (la ultima es
    la interfaz) y la TCP."""
    q = np.radians(np.asarray(q_deg, dtype=float))
    T = _dh(np.pi, 0.0, 0.0, 0.0)
    out = [np.eye(4)]
    for i, (alpha, a, d, off) in enumerate(_DH):
        T = T @ _dh(alpha, a, d, q[i] + off)
        out.append(T.copy())
    tcp = T.copy()
    tcp[:3, 3] = T[:3, 3] + T[:3, 2] * TCP_OFFSET_M
    out.append(tcp)
    return out


def rpy_to_matrix(rpy_deg):
    """Rx Ry Rz de la app web (R = Rz * Ry * Rx, grados)."""
    rx, ry, rz = np.radians(rpy_deg)
    cx, sx, cy, sy, cz, sz = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry), np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_to_matrix(qw, qx, qy, qz):
    n = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    w, x, y, z = qw / n, qx / n, qy / n, qz / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_matrix(position, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position
    return T


def tool_pose_matrix(tool_pose):
    """[x y z Rx Ry Rz] (TCP de la app web, m y grados) -> 4x4 en la base."""
    tool_pose = np.asarray(tool_pose, dtype=float)
    return pose_matrix(tool_pose[:3], rpy_to_matrix(tool_pose[3:6]))


def cloud_to_base(points_depth, tool_pose, tcp_offset_m=TCP_OFFSET_M,
                  depth_to_int=(0.0275, 0.0660, -0.00305)):
    """Nube de la camara de profundidad (como la guarda screw_server) -> base.

    Igual que kinova_grasp_V2: DEPTH -> INT es solo una traslacion (guia del
    Gen3, tabla 92) y la INTERFAZ esta tcp_offset_m detras de la TCP.
    """
    T_tool = tool_pose_matrix(tool_pose)
    R = T_tool[:3, :3]
    p_int0 = T_tool[:3, 3] - R[:, 2] * tcp_offset_m
    pts = np.asarray(points_depth, dtype=float) + np.asarray(depth_to_int)
    return pts @ R.T + p_int0


def base_to_cloud(points_base, tool_pose, tcp_offset_m=TCP_OFFSET_M,
                  depth_to_int=(0.0275, 0.0660, -0.00305)):
    """Inversa de cloud_to_base: puntos en la base -> frame de la camara."""
    T_tool = tool_pose_matrix(tool_pose)
    R = T_tool[:3, :3]
    p_int0 = T_tool[:3, 3] - R[:, 2] * tcp_offset_m
    return (np.asarray(points_base, dtype=float) - p_int0) @ R - np.asarray(depth_to_int)
