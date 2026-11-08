#pragma once

// Procesamiento de un agarre de GPD: validacion y transformaciones
//   camara -> interfaz -> base -> pose objetivo de la INTERFAZ (brida).
// Sin red ni robot: se puede probar por separado (tests/grasp_pipeline_test.cpp).

#include "grasp_message.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <sstream>
#include <string>

namespace grasp
{

struct VisionParams
{
    // MODO SOLO IMPRIMIR: se reciben, validan, transforman y registran los
    // agarres. NUNCA se llama a setReferencePose desde la vision.
    bool enabled = true;

    // Puerto UDP donde el control escucha a la vision (1313 ya lo usa udp_server).
    int udp_port = 5005;

    // Agarres con score menor se rechazan (ajustar a la escala de GPD).
    double min_score = 0.0;

    // El agarre debe estar a lo mas a esta distancia de la camara [m].
    double max_distance_m = 1.2;

    // El robot debe estar quieto al recibir el agarre (la nube se tomo con
    // la q actual). Velocidad articular maxima permitida [rad/s].
    double max_joint_speed_rad_s = 0.05;

    // Camaras del modulo de vision Kinova respecto a la INTERFAZ
    // (guia Gen3 r12.0, "Vision module sensors reference frames"):
    // misma orientacion que la interfaz, solo desplazadas [m].
    Eigen::Vector3d color_offset_m = Eigen::Vector3d(0.0, 0.05639, -0.00305);
    Eigen::Vector3d depth_offset_m = Eigen::Vector3d(0.02750, 0.06600, -0.00305);

    // Ejes de la herramienta expresados en el frame de la mano de GPD
    // (columnas = tool x, tool y, tool z). Por defecto:
    //   tool z = approach, tool x = binormal (cierre), tool y = axis.
    // VERIFICAR con el robot hacia donde cierran los dedos de la Robotiq.
    Eigen::Matrix3d gpd_to_tool =
        (Eigen::Matrix3d() << 0, 0, 1,
                              1, 0, 0,
                              0, 1, 0).finished();

    // Distancia sobre el approach entre la INTERFAZ y el punto que GPD reporta
    // como posicion del agarre [m]. 0.12 = TCP de la Tool configuration web.
    // VERIFICAR que punto es la "posicion" en el codigo de GPD.
    double grasp_point_to_interface_m = 0.12;

    // Pre-agarre: retroceso adicional sobre el approach [m].
    double pregrasp_distance_m = 0.10;
};


struct Pose
{
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
};

inline Pose compose(const Pose& a, const Pose& b)
{
    return {a.p + a.R * b.p, a.R * b.R};
}


// "" si el mensaje es aceptable; si no, el motivo del rechazo.
inline std::string validateGrasp(const GraspMessage& m, const VisionParams& vp)
{
    if (m.frame != static_cast<uint16_t>(CameraFrame::Color) &&
        m.frame != static_cast<uint16_t>(CameraFrame::Depth))
        return "frame desconocido";

    auto finite3 = [](const float* v) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };
    if (!finite3(m.pos_m) || !finite3(m.approach) || !finite3(m.binormal) ||
        !finite3(m.axis) || !std::isfinite(m.score) || !std::isfinite(m.width_m))
        return "valores NaN/Inf";

    const Eigen::Vector3d a(m.approach[0], m.approach[1], m.approach[2]);
    const Eigen::Vector3d b(m.binormal[0], m.binormal[1], m.binormal[2]);
    const Eigen::Vector3d c(m.axis[0], m.axis[1], m.axis[2]);
    constexpr double tol = 0.05;
    if (std::abs(a.norm() - 1) > tol || std::abs(b.norm() - 1) > tol || std::abs(c.norm() - 1) > tol)
        return "ejes no unitarios";
    if (std::abs(a.dot(b)) > tol || std::abs(a.dot(c)) > tol || std::abs(b.dot(c)) > tol)
        return "ejes no ortogonales";
    if (a.cross(b).dot(c) < 1 - tol)
        return "ejes no forman un frame derecho (approach x binormal != axis)";

    if (m.score < vp.min_score)
        return "score bajo";

    const Eigen::Vector3d p_m(m.pos_m[0], m.pos_m[1], m.pos_m[2]);
    if (p_m.z() <= 0.0)
        return "agarre detras de la camara (z <= 0)";
    if (p_m.norm() > vp.max_distance_m)
        return "agarre demasiado lejos de la camara";

    return "";
}


// Agarre en el frame de la camara (metros), R = [approach | binormal | axis]
// re-ortonormalizada (los float del mensaje traen error pequeno).
inline Pose graspInCamera(const GraspMessage& m)
{
    Eigen::Matrix3d R;
    R.col(0) = Eigen::Vector3d(m.approach[0], m.approach[1], m.approach[2]);
    R.col(1) = Eigen::Vector3d(m.binormal[0], m.binormal[1], m.binormal[2]);
    R.col(2) = Eigen::Vector3d(m.axis[0], m.axis[1], m.axis[2]);
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Pose g;
    g.R = svd.matrixU() * svd.matrixV().transpose();
    g.p = Eigen::Vector3d(m.pos_m[0], m.pos_m[1], m.pos_m[2]);
    return g;
}

inline Pose cameraInInterface(uint16_t frame, const VisionParams& vp)
{
    Pose c;   // misma orientacion que la interfaz (guia Kinova)
    c.p = frame == static_cast<uint16_t>(CameraFrame::Depth) ? vp.depth_offset_m : vp.color_offset_m;
    return c;
}


struct GraspTargets
{
    Pose grasp_base;           // agarre de GPD en la base (ejes de GPD)
    Pose interface_grasp;      // donde debe quedar la INTERFAZ para agarrar
    Pose interface_pregrasp;   // la misma, retrocedida pregrasp_distance_m
};

// interface_in_base = GetPose(q) (el modelo termina en la interfaz).
inline GraspTargets computeTargets(const GraspMessage& m, const Pose& interface_in_base,
                                   const VisionParams& vp)
{
    GraspTargets t;
    t.grasp_base = compose(compose(interface_in_base, cameraInInterface(m.frame, vp)), graspInCamera(m));

    // Orientacion de la herramienta/interfaz: ejes de GPD -> ejes de la tool.
    const Eigen::Matrix3d R_tool = t.grasp_base.R * vp.gpd_to_tool;
    const Eigen::Vector3d approach = R_tool.col(2);

    t.interface_grasp.R = R_tool;
    t.interface_grasp.p = t.grasp_base.p - vp.grasp_point_to_interface_m * approach;

    t.interface_pregrasp.R = R_tool;
    t.interface_pregrasp.p = t.interface_grasp.p - vp.pregrasp_distance_m * approach;
    return t;
}

}  // namespace grasp
