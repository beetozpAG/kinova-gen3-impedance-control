#pragma once

// SECUENCIA DE TAREA (Fase 2: placa -> fixture).
//
// Maquina de estados que recorre una lista de pasos:
//   Move     -> nueva referencia cartesiana (setReferencePose); termina
//               cuando la pose MEDIDA llega a la pose objetivo y el robot
//               esta quieto durante settle_s.
//   Gripper  -> comando a la pinza; termina cuando la posicion de la pinza
//               deja de cambiar (si cerro sobre la placa no llega a 100 %).
//   Fixture  -> POS <grados> al ESP32; termina con FixtureClient::settled().
//   Wait     -> espera fija.
//
// Las poses se escriben COMO LAS MUESTRA LA APP WEB DE KINOVA: posicion de la
// TCP (Tool configuration, 0.12 m sobre el eje z de la herramienta) en metros
// y Rx Ry Rz en grados con R = Rz * Ry * Rx. Aqui se pasan a la INTERFAZ
// (donde termina el modelo): p_interfaz = p_tcp - R * [0 0 tcp_offset_m].
//
// Reglas:
//   - La secuencia solo avanza con el control ACTIVO (slider ON y, con el
//     hardware habilitado, la rampa de entrada completa). Con OFF se congela.
//   - El primer paso siempre espera el boton "Siguiente paso". Con
//     confirm_each_step = true (PASO A PASO), TODOS los pasos lo esperan;
//     con false (AUTOMATICO) solo el primero y las pausas. Se cambia con la
//     secuencia corriendo (setAutomatic, desde la GUI): cuenta a partir del
//     paso siguiente; si ya esta esperando el boton, sigue esperandolo.
//   - Si un paso no termina a tiempo, la secuencia se PAUSA (el robot sostiene
//     la referencia). "Siguiente paso" da el paso por terminado y sigue.
//   - Pausa del operador (boton "Pausa", 2026-10-27): el brazo se queda donde
//     esta (referencia = pose medida) y "Siguiente paso" REANUDA el mismo
//     paso desde el principio (no lo da por terminado). La pinza y el fixture
//     terminan el comando que ya tenian.
//   - Una falla del fixture o de la pinza ABORTA la secuencia (el robot
//     sostiene la ultima referencia; no se manda nada mas).
//   - VISION: un paso Vision pide una foto al servidor de vision (screw_server)
//     y recibe el agarre del tornillo (TCP en la base):
//       * servidor de kinova_grasp_V2: pose de agarre (posicion + Rx Ry Rz) y
//         pre-grasp (10 cm atras sobre la aproximacion, misma orientacion).
//         Pasos moveToVisionPregrasp / moveToVisionGrasp.
//       * servidor viejo (kinova_grasp): solo la posicion. Pasos moveToVision
//         (posicion + dz con la orientacion de una pose fija).
//     Si la vision falla o da un agarre fuera de los limites (caja, distancia
//     a la foto, inclinacion de la pinza), la secuencia se PAUSA y
//     "Siguiente paso" toma otra foto.
//     visionCaptureCloud (2026-10-20): solo guarda la nube de puntos, sin
//     GPD (p. ej. encima del hoyo antes de soltar el tornillo). No da agarre
//     ni cambia el de la ultima foto.
//     visionCaptureHole (2026-10-21): igual, y el servidor da el centro del
//     hoyo MAS CHICO de la placa; moveToHole va a una pose fija (altura y
//     orientacion) con x y de la TCP sobre ese centro.
//     visionApproach (2026-10-27, approach_object de Antony): pide APPROACH;
//     va a la pose del EE que deja la camara de color 30 cm arriba del
//     objeto (por su color), toma otra foto y pregunta otra vez; termina
//     cuando el objeto esta completo en la imagen (IN VIEW) Y el robot ya esta
//     a menos de approach_center_tol_m de esa pose (centrado, verificado con
//     una foto). La foto siguiente (CAPTURE ... PLATE|BOLT) es
//     grasps_from_camera --color --filter.
//
// No usa red, puertos ni robot: se prueba sola en tests/task_sequence_test.cpp.

#include "fixture_client.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace task
{

// Pose tal como la muestra la app web de Kinova (TCP en la base).
struct TaskPose
{
    std::string name;
    Eigen::Vector3d tcp_position_m;   // X Y Z [m]
    Eigen::Vector3d rpy_deg;          // Rx Ry Rz [grados]

    // Articulaciones en esa pose, como las muestra la app web [grados, 0-360
    // o +-180]. Opcional (NaN = no hay): si estan, al moverse a esta pose la
    // postura del espacio nulo (codo) pasa a ser esta.
    Eigen::Matrix<double, 7, 1> joints_deg =
        Eigen::Matrix<double, 7, 1>::Constant(std::numeric_limits<double>::quiet_NaN());

    bool hasJoints() const { return joints_deg.allFinite(); }
};

// Articulaciones de la app web [grados] -> [rad] en [-pi, pi].
inline Eigen::Matrix<double, 7, 1> jointsToRad(const Eigen::Matrix<double, 7, 1>& deg)
{
    Eigen::Matrix<double, 7, 1> r;
    for (int i = 0; i < 7; ++i)
        r(i) = std::remainder(deg(i), 360.0) * (M_PI / 180.0);
    return r;
}

// Pose sin definir: la validacion la rechaza con un mensaje claro.
inline TaskPose undefinedPose(const std::string& name)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {name, Eigen::Vector3d::Constant(nan), Eigen::Vector3d::Constant(nan)};
}


enum class StepType
{
    Move,
    Gripper,
    Fixture,
    Wait,
    Vision
};

// A que punto de la vision va un Move con vision.
enum class VisionTarget
{
    PositionDz,   // posicion de agarre + dz, orientacion de una pose fija
    Pregrasp,     // pre-grasp de la vision, con su orientacion
    Grasp,        // pose de agarre de la vision, con su orientacion
    PlateFixture, // pose fija (fixture) corregida por donde se agarro la placa
    Hole          // pose fija (altura, orientacion) con x y sobre el hoyo de la vision
};

struct Step
{
    StepType type = StepType::Wait;
    std::string label;

    std::string pose;                 // Move: nombre de la pose
                                      // Move con vision: pose de la postura (y de la
                                      //   orientacion con PositionDz)
                                      // Vision: pose donde se toma la foto (limites del resultado)
    bool from_vision = false;         // Move: ir a un punto de la vision
    VisionTarget vision_target = VisionTarget::PositionDz;
    double vision_dz_m = 0.0;         //       PositionDz y Grasp: desplazamiento en z de la base [m]
    double vision_z_m = std::numeric_limits<double>::quiet_NaN();
                                      //       Grasp: z FIJA de la base [m] (x y y orientacion
                                      //       del agarre); NaN = usar vision_dz_m
    bool vision_needs_edge = false;   // Vision: la respuesta debe traer el canto de la placa
    bool vision_cloud_only = false;   // Vision: solo guardar la nube (sin GPD)
    bool vision_needs_hole = false;   // Vision (nube): la respuesta debe traer el hoyo
    bool vision_approach = false;     // Vision: centrar la camara sobre el objeto (APPROACH)
    std::string vision_object;        // Vision: "PLATE" o "BOLT" (configuracion y color en el servidor)

    double gripper_pct = 0.0;         // Gripper: 0 = abierta, 100 = cerrada
    double gripper_speed_pct = 0.0;   //          velocidad [%]
    double gripper_force_pct = 0.0;   //          fuerza [%]

    double fixture_deg = 0.0;         // Fixture: angulo objetivo [grados]

    double wait_s = 0.0;              // Wait: tiempo [s]
};

inline Step moveTo(const std::string& label, const std::string& pose)
{
    Step s;
    s.type = StepType::Move;
    s.label = label;
    s.pose = pose;
    return s;
}

inline Step gripper(const std::string& label, double pct, double speed_pct, double force_pct)
{
    Step s;
    s.type = StepType::Gripper;
    s.label = label;
    s.gripper_pct = pct;
    s.gripper_speed_pct = speed_pct;
    s.gripper_force_pct = force_pct;
    return s;
}

inline Step fixtureTo(const std::string& label, double deg)
{
    Step s;
    s.type = StepType::Fixture;
    s.label = label;
    s.fixture_deg = deg;
    return s;
}

inline Step waitFor(const std::string& label, double seconds)
{
    Step s;
    s.type = StepType::Wait;
    s.label = label;
    s.wait_s = seconds;
    return s;
}

// Pide a la vision la posicion del tornillo. near_pose: pose donde esta el
// robot al tomar la foto; el resultado debe quedar cerca y debajo de ella.
inline Step visionCapture(const std::string& label, const std::string& near_pose)
{
    Step s;
    s.type = StepType::Vision;
    s.label = label;
    s.pose = near_pose;
    s.vision_object = "BOLT";
    return s;
}

// Centra la camara de color sobre el objeto (object "PLATE" o "BOLT"; el
// servidor sabe su color): APPROACH -> va a la pose del EE y pregunta otra
// vez ... -> IN VIEW con el robot ya en esa pose (centrado). posture_pose:
// postura del codo y limites (approach_max_xy_m alrededor de ella). Si en
// approach_max_moves movimientos no queda en vista, PAUSA; si queda en vista
// pero no centrado, sigue (avisa).
inline Step visionApproach(const std::string& label, const std::string& posture_pose,
                           const std::string& object)
{
    Step s;
    s.type = StepType::Vision;
    s.label = label;
    s.pose = posture_pose;
    s.vision_approach = true;
    s.vision_object = object;
    return s;
}

// Solo toma la nube de puntos y la guarda (el servidor no corre GPD). No da
// agarre: los pasos moveToVision siguen usando la ultima foto con GPD.
inline Step visionCaptureCloud(const std::string& label, const std::string& pose)
{
    Step s = visionCapture(label, pose);
    s.vision_object.clear();
    s.vision_cloud_only = true;
    return s;
}

// Foto solo de nube (sin GPD) y el centro del hoyo mas chico de la placa.
// Sin hoyo, o con un hoyo lejos de 'pose', PAUSA (otra foto).
inline Step visionCaptureHole(const std::string& label, const std::string& pose)
{
    Step s = visionCaptureCloud(label, pose);
    s.vision_needs_hole = true;
    return s;
}

// Va a (posicion de la vision + dz en z de la base) con la orientacion de
// orientation_pose (sin orientacion de la vision).
inline Step moveToVision(const std::string& label, const std::string& orientation_pose, double dz_m)
{
    Step s;
    s.type = StepType::Move;
    s.label = label;
    s.pose = orientation_pose;
    s.from_vision = true;
    s.vision_dz_m = dz_m;
    return s;
}

// Va a la pre-grasp que da la vision (servidor V2), con la orientacion del
// agarre. posture_pose: pose cuyas articulaciones son la postura del codo.
inline Step moveToVisionPregrasp(const std::string& label, const std::string& posture_pose)
{
    Step s = moveToVision(label, posture_pose, 0.0);
    s.vision_target = VisionTarget::Pregrasp;
    return s;
}

// Va a la pose de agarre que da la vision (servidor V2) + dz en z de la base,
// con la orientacion del agarre (dz > 0: subir recto con el tornillo).
inline Step moveToVisionGrasp(const std::string& label, const std::string& posture_pose,
                              double dz_m = 0.0)
{
    Step s = moveToVision(label, posture_pose, dz_m);
    s.vision_target = VisionTarget::Grasp;
    return s;
}

// Sube (o va) a la z FIJA z_m de la base [m] sobre el agarre de la vision:
// x y y orientacion del agarre (2026-11-03: subir la placa a 0.364). Si z_m
// queda abajo del agarre, PAUSA.
inline Step moveToVisionGraspAtZ(const std::string& label, const std::string& posture_pose, double z_m)
{
    Step s = moveToVisionGrasp(label, posture_pose);
    s.vision_z_m = z_m;
    return s;
}

// Foto de la placa: ademas del agarre, el servidor debe medir el canto de la
// placa en el agarre (sus dos extremos). Sin canto, PAUSA (otra foto).
inline Step visionCapturePlate(const std::string& label, const std::string& near_pose)
{
    Step s = visionCapture(label, near_pose);
    s.vision_needs_edge = true;
    s.vision_object = "PLATE";
    return s;
}

// Va a la pose fija 'pose' (p. ej. COLOCAR_PLACA, grabada con la placa
// agarrada al centro del canto) CORREGIDA por como se agarro de verdad: la
// placa queda en el fixture donde quedaria con el agarre al centro, sin
// importar en que punto del canto la tomo la vision, a que altura ni con que
// giro. Usa el canto de la ultima foto (visionCapturePlate) y la pose MEDIDA
// al terminar el paso de agarre. Postura del codo: la de 'pose'.
inline Step moveToPlateCorrected(const std::string& label, const std::string& pose)
{
    Step s = moveToVision(label, pose, 0.0);
    s.vision_target = VisionTarget::PlateFixture;
    return s;
}

// Va a la pose fija 'pose' (su altura + dz_m y su orientacion) con x y de la
// TCP sobre el centro del hoyo de la ultima visionCaptureHole (+
// hole_offset_xy_m). Postura del codo: la de 'pose'.
inline Step moveToHole(const std::string& label, const std::string& pose, double dz_m = 0.0)
{
    Step s = moveToVision(label, pose, dz_m);
    s.vision_target = VisionTarget::Hole;
    return s;
}

// true si el paso necesita la orientacion y la pre-grasp de la vision.
inline bool needsVisionPose(const Step& s)
{
    return s.type == StepType::Move && s.from_vision && s.vision_target != VisionTarget::PositionDz &&
           s.vision_target != VisionTarget::Hole;
}


struct TaskParams
{
    // false = el programa se comporta exactamente como antes (sin secuencia).
    bool enabled = false;

    // true = cada paso espera el boton "Siguiente paso" (PASO A PASO).
    // false = AUTOMATICO: solo el primer paso (y las pausas) lo esperan.
    bool confirm_each_step = true;

    // Duracion maxima del experimento con la secuencia activa [s]
    // (reemplaza experiment_duration_s, que es corto para una secuencia).
    double duration_s = 900.0;

    // TCP de la app web (Tool configuration): distancia interfaz -> TCP
    // sobre el eje z de la herramienta [m].
    double tcp_offset_m = 0.12;

    // ---- Llegada de los movimientos ----
    double position_tolerance_m = 0.015;
    double orientation_tolerance_deg = 5.0;
    double still_linear_speed_m_s = 0.01;      // "quieto"
    double still_angular_speed_rad_s = 0.05;
    double settle_s = 0.5;                     // tiempo seguido dentro de tolerancia
    double move_timeout_s = 90.0;              // despues: PAUSA

    // ---- Pinza ----
    double gripper_min_time_s = 0.5;           // antes de esto no se evalua
    double gripper_still_pct = 0.5;            // cambio de posicion "quieta" [%]
    double gripper_settle_s = 0.3;             // ventana para decidir que se detuvo
    double gripper_timeout_s = 6.0;            // despues: PAUSA

    // ---- Fixture ----
    bool use_fixture = true;
    fixture::FixtureParams fixture{};
    double fixture_timeout_s = 10.0;           // despues: PAUSA

    // ---- Vision (screw_server de kinova_grasp_V2, UDP en localhost) ----
    int vision_server_port = 5006;
    // Incluye GPD y el tiempo de cerrar sus ventanas de graficas [s].
    double vision_timeout_s = 180.0;           // despues: PAUSA
    // El resultado (TCP) debe quedar a lo mas a esta distancia horizontal de
    // la pose donde se tomo la foto, y al menos vision_min_below_m debajo.
    double vision_max_xy_m = 0.20;
    double vision_min_below_m = 0.03;
    // Con orientacion (servidor V2): inclinacion maxima del eje z de la pinza
    // respecto a la vertical hacia abajo [grados], y distancia maxima
    // pre-grasp -> agarre [m] (el servidor usa 0.10).
    double vision_max_tilt_deg = 20.0;
    double vision_max_pregrasp_m = 0.15;

    // ---- Placa al fixture con el agarre de la vision (moveToPlateCorrected) ----
    // Agarre "de referencia" con el que se grabaron ARRIBA_FIXTURE_NEW y
    // COLOCAR_PLACA: al centro del canto, pinza perpendicular, y la TCP esta
    // distancia ARRIBA del canto, sobre el eje de la pinza [m].
    double plate_nominal_tcp_above_edge_m = 0.012;
    // Limites de la correccion respecto a la pose fija (si se pasa: aborta).
    double plate_max_shift_m = 0.12;
    double plate_max_angle_deg = 30.0;
    // Foto de la placa: angulo maximo entre el cierre de la pinza y la
    // normal del canto [grados]. Mas que esto, la placa gira o se resbala al
    // cerrar: la foto se rechaza (el boton toma otra).
    double plate_max_skew_deg = 20.0;

    // ---- Tornillo al hoyo de la vision (visionCaptureHole / moveToHole) ----
    // Se suma al centro del hoyo (x y de la base) [m]: calibracion si el
    // tornillo cae siempre corrido hacia el mismo lado. 0 = TCP sobre el centro.
    Eigen::Vector2d hole_offset_xy_m = Eigen::Vector2d::Zero();
    // Distancia horizontal maxima del hoyo a la pose fija de cada moveToHole
    // [m]; mas que esto ABORTA.
    double hole_max_shift_m = 0.20;
    // Distancia horizontal maxima del hoyo a la pose de la foto [m]; mas que
    // esto la foto se rechaza (PAUSA, el boton toma otra).
    double hole_max_from_photo_m = 0.35;

    // ---- Centrar la camara sobre el objeto (visionApproach) ----
    int approach_max_moves = 5;            // movimientos antes de rendirse
    double approach_max_move_m = 0.30;     // de la posicion medida al objetivo [m]
    double approach_max_xy_m = 0.30;       // objetivo a lo mas esto de la pose del paso (horizontal) [m]
    double approach_max_tilt_deg = 10.0;   // la camara mira hacia abajo: eje z de la pinza vs vertical
    // Centrado: IN VIEW y el robot a menos de esto de la pose que da el
    // servidor [m] (POSITION_TOLERANCE de approach_pose.h de Antony). Si no,
    // va a esa pose y pregunta otra vez.
    double approach_center_tol_m = 0.01;

    // Paso con el que arranca la secuencia (1 = el primero). Sirve para
    // probar solo una parte (p. ej. los tornillos).
    size_t first_step = 1;

    // ---- Seguridad ----
    // Toda pose (TCP) debe estar dentro de esta caja de la base [m].
    // AJUSTAR a la mesa del laboratorio.
    Eigen::Vector3d workspace_min_m = Eigen::Vector3d(0.20, -0.50, 0.02);
    Eigen::Vector3d workspace_max_m = Eigen::Vector3d(0.85, 0.50, 0.70);
    // Zona baja (2026-10-27): otra caja, p. ej. sobre una mesa mas baja que
    // la del robot. Una pose vale si esta en la caja de trabajo O en esta.
    // Apagada con low_zone_enabled = false.
    bool low_zone_enabled = false;
    Eigen::Vector3d low_zone_min_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d low_zone_max_m = Eigen::Vector3d::Zero();
    // Distancia maxima de un movimiento (entre poses consecutivas de la
    // lista, de la pose inicial del robot al primer movimiento y, al iniciar
    // cada movimiento, de la posicion MEDIDA al objetivo) [m].
    double max_move_distance_m = 0.60;

    std::vector<TaskPose> poses;
    std::vector<Step> steps;
};


// Pose de la INTERFAZ para una pose de la app web.
struct InterfaceTarget
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

inline Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d& rpy_deg)
{
    constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(rpy_deg(2) * DEG2RAD, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy_deg(1) * DEG2RAD, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy_deg(0) * DEG2RAD, Eigen::Vector3d::UnitX());
    q.normalize();
    return q;
}

inline InterfaceTarget toInterface(const TaskPose& pose, double tcp_offset_m)
{
    InterfaceTarget t;
    t.orientation = rpyToQuaternion(pose.rpy_deg);
    t.position = pose.tcp_position_m - t.orientation * Eigen::Vector3d(0.0, 0.0, tcp_offset_m);
    return t;
}

// Inverso de toInterface: pose de la interfaz (modelo) -> TCP como la app web
// [x y z m, Rx Ry Rz grados, R = Rz*Ry*Rx, Ry en [-90, 90]].
inline void interfaceToToolPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                double tcp_offset_m, double out[6])
{
    constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;
    const Eigen::Matrix3d R = orientation.normalized().toRotationMatrix();
    const Eigen::Vector3d tcp = position + R * Eigen::Vector3d(0.0, 0.0, tcp_offset_m);
    out[0] = tcp.x();
    out[1] = tcp.y();
    out[2] = tcp.z();
    out[3] = std::atan2(R(2, 1), R(2, 2)) * RAD2DEG;
    out[4] = std::asin(std::clamp(-R(2, 0), -1.0, 1.0)) * RAD2DEG;
    out[5] = std::atan2(R(1, 0), R(0, 0)) * RAD2DEG;
}

// Angulo entre dos orientaciones [rad], en [0, pi].
inline double angleBetween(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
    const double d = std::abs(a.normalized().dot(b.normalized()));
    return 2.0 * std::acos(std::min(1.0, d));
}

inline const TaskPose* findPose(const TaskParams& p, const std::string& name)
{
    for (const auto& pose : p.poses)
        if (pose.name == name)
            return &pose;
    return nullptr;
}

inline bool usesFixture(const TaskParams& p)
{
    for (const auto& s : p.steps)
        if (s.type == StepType::Fixture)
            return true;
    return false;
}

inline bool usesGripper(const TaskParams& p)
{
    for (const auto& s : p.steps)
        if (s.type == StepType::Gripper)
            return true;
    return false;
}

inline bool usesVision(const TaskParams& p)
{
    for (size_t i = (p.first_step >= 1 ? p.first_step - 1 : 0); i < p.steps.size(); ++i)
        if (p.steps[i].type == StepType::Vision)
            return true;
    return false;
}

// Primer paso Move a una pose fija desde first_step (nullptr si no hay o si
// el primer movimiento depende de la vision).
inline const TaskPose* firstMovePose(const TaskParams& p)
{
    for (size_t i = (p.first_step >= 1 ? p.first_step - 1 : 0); i < p.steps.size(); ++i)
    {
        const Step& s = p.steps[i];
        if (s.type == StepType::Move)
            return s.from_vision ? nullptr : findPose(p, s.pose);
    }
    return nullptr;
}


// "" si la configuracion es valida; si no, la lista de problemas.

// Dentro de la caja de trabajo o de la zona baja (si esta encendida).
inline bool inWorkspace(const TaskParams& p, const Eigen::Vector3d& x)
{
    const auto inside = [&](const Eigen::Vector3d& lo, const Eigen::Vector3d& hi) {
        return x.allFinite() && (x.array() >= lo.array()).all() && (x.array() <= hi.array()).all();
    };
    return inside(p.workspace_min_m, p.workspace_max_m) ||
           (p.low_zone_enabled && inside(p.low_zone_min_m, p.low_zone_max_m));
}
inline std::string validate(const TaskParams& p)
{
    std::ostringstream err;
    auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };

    if (!positive(p.duration_s) || !std::isfinite(p.tcp_offset_m) || p.tcp_offset_m < 0.0 ||
        !positive(p.position_tolerance_m) || !positive(p.orientation_tolerance_deg) ||
        !positive(p.still_linear_speed_m_s) || !positive(p.still_angular_speed_rad_s) ||
        !positive(p.settle_s) || !positive(p.move_timeout_s) || !positive(p.gripper_min_time_s) ||
        !positive(p.gripper_still_pct) || !positive(p.gripper_settle_s) ||
        !positive(p.gripper_timeout_s) || !positive(p.fixture_timeout_s) ||
        !positive(p.max_move_distance_m) || !positive(p.vision_timeout_s) ||
        !positive(p.vision_max_xy_m) || !std::isfinite(p.vision_min_below_m) ||
        p.vision_min_below_m < 0.0 || p.vision_server_port <= 0 || p.vision_server_port > 65535 ||
        !positive(p.vision_max_tilt_deg) || p.vision_max_tilt_deg > 90.0 ||
        !positive(p.vision_max_pregrasp_m) || !positive(p.hole_max_shift_m) ||
        !positive(p.hole_max_from_photo_m) ||
        !p.hole_offset_xy_m.allFinite() || p.hole_offset_xy_m.norm() > 0.05 ||
        p.approach_max_moves < 1 || p.approach_max_moves > 20 || !positive(p.approach_max_move_m) ||
        !positive(p.approach_max_xy_m) || !positive(p.approach_max_tilt_deg) ||
        !positive(p.approach_center_tol_m))
        err << "  - tolerancias, tiempos y distancias deben ser finitos y > 0\n";

    if (!p.workspace_min_m.allFinite() || !p.workspace_max_m.allFinite() ||
        (p.workspace_min_m.array() >= p.workspace_max_m.array()).any())
        err << "  - workspace_min_m debe ser menor que workspace_max_m en x, y, z\n";
    if (p.low_zone_enabled &&
        (!p.low_zone_min_m.allFinite() || !p.low_zone_max_m.allFinite() ||
         (p.low_zone_min_m.array() >= p.low_zone_max_m.array()).any()))
        err << "  - low_zone_min_m debe ser menor que low_zone_max_m en x, y, z\n";

    std::set<std::string> names;
    for (const auto& pose : p.poses)
    {
        if (pose.name.empty())
        {
            err << "  - hay una pose sin nombre\n";
            continue;
        }
        if (!names.insert(pose.name).second)
            err << "  - la pose '" << pose.name << "' esta repetida\n";

        if (!pose.tcp_position_m.allFinite() || !pose.rpy_deg.allFinite())
        {
            err << "  - la pose '" << pose.name
                << "' no esta definida (copia X Y Z [m] y Rx Ry Rz [grados] de la app web)\n";
            continue;
        }
        if (!pose.hasJoints() && !pose.joints_deg.array().isNaN().all())
            err << "  - la pose '" << pose.name
                << "': las 7 articulaciones deben estar todas o ninguna\n";
        if (!inWorkspace(p, pose.tcp_position_m))
            err << "  - la pose '" << pose.name << "' (" << pose.tcp_position_m.transpose()
                << " m) esta fuera de la caja de trabajo\n";
    }

    if (p.steps.empty())
        err << "  - la lista de pasos esta vacia\n";
    if (p.first_step < 1 || p.first_step > p.steps.size())
        err << "  - first_step debe estar entre 1 y el numero de pasos\n";

    const TaskPose* previous_move = nullptr;
    bool vision_before = false;   // hay un paso Vision antes (desde first_step)
    bool hole_before = false;     // hay un paso visionCaptureHole antes
    for (size_t i = 0; i < p.steps.size(); ++i)
    {
        const Step& s = p.steps[i];
        const std::string where = "  - paso " + std::to_string(i + 1) + " ('" + s.label + "'): ";

        switch (s.type)
        {
            case StepType::Move:
            {
                const TaskPose* pose = findPose(p, s.pose);
                if (!pose)
                {
                    err << where << "la pose '" << s.pose << "' no existe\n";
                    break;
                }
                if (s.from_vision)
                {
                    if (!std::isfinite(s.vision_dz_m) || s.vision_dz_m < -0.05 || s.vision_dz_m > 0.30 ||
                        (s.vision_target == VisionTarget::Pregrasp && s.vision_dz_m != 0.0))
                        err << where << "dz de vision fuera de [-0.05, 0.30] m\n";
                    if (!std::isnan(s.vision_z_m) &&
                        (s.vision_target != VisionTarget::Grasp || s.vision_dz_m != 0.0 ||
                         !std::isfinite(s.vision_z_m) || s.vision_z_m < p.workspace_min_m.z() ||
                         s.vision_z_m > p.workspace_max_m.z()))
                        err << where << "z fija: solo con el agarre (sin dz) y dentro de la caja de trabajo\n";
                    if (s.vision_target == VisionTarget::Hole)
                    {
                        if (!hole_before && i + 1 >= p.first_step)
                            err << where << "va al hoyo de la vision pero no hay una foto del hoyo antes\n";
                    }
                    else if (!vision_before && i + 1 >= p.first_step)
                        err << where << "usa la posicion de la vision pero no hay un paso de vision antes\n";
                    previous_move = nullptr;   // distancia: se revisa al ejecutar
                    break;
                }
                if (previous_move && pose->tcp_position_m.allFinite() &&
                    previous_move->tcp_position_m.allFinite())
                {
                    const double d = (pose->tcp_position_m - previous_move->tcp_position_m).norm();
                    if (d > p.max_move_distance_m)
                        err << where << "movimiento de " << d << " m desde '" << previous_move->name
                            << "' (maximo " << p.max_move_distance_m << " m)\n";
                }
                previous_move = pose;
                break;
            }
            case StepType::Gripper:
                if (!std::isfinite(s.gripper_pct) || s.gripper_pct < 0.0 || s.gripper_pct > 100.0)
                    err << where << "posicion de pinza fuera de 0-100 %\n";
                if (!std::isfinite(s.gripper_speed_pct) || s.gripper_speed_pct < 1.0 ||
                    s.gripper_speed_pct > 100.0)
                    err << where << "velocidad de pinza fuera de 1-100 %\n";
                if (!std::isfinite(s.gripper_force_pct) || s.gripper_force_pct < 0.0 ||
                    s.gripper_force_pct > 100.0)
                    err << where << "fuerza de pinza fuera de 0-100 %\n";
                break;
            case StepType::Fixture:
                if (!p.use_fixture)
                    err << where << "hay un paso de fixture pero use_fixture = false\n";
                if (!std::isfinite(s.fixture_deg) || s.fixture_deg < p.fixture.min_deg ||
                    s.fixture_deg > p.fixture.max_deg)
                    err << where << "angulo del fixture fuera de [" << p.fixture.min_deg << ", "
                        << p.fixture.max_deg << "] grados\n";
                break;
            case StepType::Wait:
                if (!std::isfinite(s.wait_s) || s.wait_s < 0.0)
                    err << where << "tiempo de espera invalido\n";
                break;
            case StepType::Vision:
                if (!findPose(p, s.pose))
                    err << where << "la pose '" << s.pose << "' no existe\n";
                if (!s.vision_object.empty() && s.vision_object != "PLATE" && s.vision_object != "BOLT")
                    err << where << "objeto de vision '" << s.vision_object << "' (PLATE o BOLT)\n";
                if (s.vision_approach && (s.vision_object.empty() || s.vision_cloud_only))
                    err << where << "centrar la camara necesita el objeto (PLATE o BOLT)\n";
                if (i + 1 >= p.first_step && !s.vision_cloud_only && !s.vision_approach)
                    vision_before = true;
                if (i + 1 >= p.first_step && s.vision_needs_hole)
                    hole_before = true;
                break;
        }
    }

    return err.str();
}


enum class Status
{
    WaitingConfirm,   // esperando "Siguiente paso" para iniciar el paso actual
    Running,
    Paused,           // el paso no termino a tiempo; "Siguiente paso" lo da por terminado
    Done,
    Aborted
};

inline const char* statusName(Status s)
{
    switch (s)
    {
        case Status::WaitingConfirm: return "ESPERANDO 'SIGUIENTE PASO'";
        case Status::Running:        return "EJECUTANDO";
        case Status::Paused:         return "PAUSADA";
        case Status::Done:           return "TERMINADA";
        case Status::Aborted:        return "ABORTADA";
    }
    return "?";
}

inline const char* stepTypeName(StepType t)
{
    switch (t)
    {
        case StepType::Move:    return "MOVER";
        case StepType::Gripper: return "PINZA";
        case StepType::Fixture: return "FIXTURE";
        case StepType::Wait:    return "ESPERAR";
        case StepType::Vision:  return "VISION";
    }
    return "?";
}


// Lo que el lazo de control mide y le pasa a la secuencia en cada ciclo.
struct Inputs
{
    double dt = 0.0;
    bool active = false;          // slider ON (y rampa completa con hardware)
    bool confirm = false;         // se presiono "Siguiente paso" (flanco)
    bool pause = false;           // se presiono "Pausa" (flanco)

    Eigen::Vector3d position = Eigen::Vector3d::Zero();               // interfaz medida
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    double linear_speed_m_s = 0.0;                                    // |J dq| lineal
    double angular_speed_rad_s = 0.0;                                 // |J dq| angular

    bool gripper_detected = false;
    double gripper_pos_pct = 0.0;

    bool fixture_settled = false;
    bool fixture_fault = false;
    std::string fixture_fault_text;

    // Ultima respuesta del servidor de vision (vision_seq = 0: ninguna).
    uint32_t vision_seq = 0;
    bool vision_ok = false;
    Eigen::Vector3d vision_tcp_m = Eigen::Vector3d::Zero();   // TCP en la base [m]
    // Solo el servidor V2: orientacion del agarre y pre-grasp (TCP).
    bool vision_has_pose = false;
    Eigen::Vector3d vision_rpy_deg = Eigen::Vector3d::Zero();       // Rx Ry Rz [grados]
    Eigen::Vector3d vision_pregrasp_m = Eigen::Vector3d::Zero();    // TCP en la base [m]
    // Canto de la placa en el agarre: sus dos extremos en la base [m].
    bool vision_has_edge = false;
    Eigen::Vector3d vision_edge_a_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d vision_edge_b_m = Eigen::Vector3d::Zero();
    // La respuesta es de una foto solo de nube (sin GPD).
    bool vision_is_cloud = false;
    // APPROACH: vision_tcp_m / vision_rpy_deg = pose objetivo del EE (TCP),
    // estado 1 MOVE, 2 INVIEW, 3 ATTARGET y centroide del objeto.
    bool vision_is_approach = false;
    int vision_approach_state = 0;
    Eigen::Vector3d vision_centroid_m = Eigen::Vector3d::Zero();
    // Nube: centro del hoyo mas chico de la placa (base [m]) y su diametro.
    bool vision_has_hole = false;
    Eigen::Vector3d vision_hole_m = Eigen::Vector3d::Zero();
    double vision_hole_diam_m = 0.0;
    std::string vision_text;                                  // motivo de falla / info
};

// Lo que el lazo de control debe ejecutar en este ciclo.
struct Commands
{
    bool set_reference = false;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    bool set_gripper = false;
    double gripper_pct = 0.0;
    double gripper_speed_pct = 0.0;
    double gripper_force_pct = 0.0;

    bool move_fixture = false;
    double fixture_deg = 0.0;

    bool request_vision = false;   // pedir una foto con este numero
    uint32_t vision_seq = 0;
    bool vision_cloud_only = false; // solo la nube, sin GPD
    std::string vision_object;      // "PLATE", "BOLT" o ""
    bool vision_approach = false;   // APPROACH (centrar la camara), no foto de agarre

    // Postura del espacio nulo de la pose a la que se va (TaskPose::joints_deg).
    bool set_posture = false;
    Eigen::Matrix<double, 7, 1> posture_rad = Eigen::Matrix<double, 7, 1>::Zero();
};


class Sequencer
{
public:
    // Lanza std::runtime_error si la configuracion no es valida.
    explicit Sequencer(TaskParams params)
        : p_(std::move(params))
    {
        const std::string errors = validate(p_);
        if (!errors.empty())
            throw std::runtime_error("Secuencia de tarea invalida:\n" + errors);

        for (const auto& s : p_.steps)
        {
            InterfaceTarget t{};
            if (s.type == StepType::Move && !s.from_vision)
                t = toInterface(*findPose(p_, s.pose), p_.tcp_offset_m);
            targets_.push_back(t);
            needs_pose_ = needs_pose_ || needsVisionPose(s);
        }
        index_ = p_.first_step - 1;
        message_ = "Presiona 'Siguiente paso' para iniciar: " + describe(index_);
    }

    Commands update(const Inputs& in)
    {
        Commands cmd;
        last_in_ = in;

        if (status_ == Status::Done || status_ == Status::Aborted)
            return cmd;
        if (in.pause && (status_ == Status::Running || status_ == Status::WaitingConfirm))
        {
            operatorPause(in, cmd);
            return cmd;
        }
        if (!in.active)
            return cmd;

        switch (status_)
        {
            case Status::WaitingConfirm:
                if (in.confirm)
                    activate(cmd);
                break;

            case Status::Paused:
                // Pausa del operador: reanuda el mismo paso. Vision sin
                // resultado: el boton toma otra foto (no hay posicion con que
                // seguir).
                if (in.confirm && operator_paused_)
                {
                    operator_paused_ = false;
                    activate(cmd);
                }
                else if (in.confirm && p_.steps[index_].type == StepType::Vision)
                    activate(cmd);
                else if (in.confirm)
                    finishStep(cmd, "terminado por el operador");
                break;

            case Status::Running:
                step_time_ += in.dt;
                runStep(in, cmd);
                break;

            default:
                break;
        }
        return cmd;
    }

    // Aborta desde fuera (p. ej. el comando al fixture fallo).
    void abort(const std::string& reason)
    {
        if (status_ == Status::Done || status_ == Status::Aborted)
            return;
        status_ = Status::Aborted;
        message_ = "ABORTADA en el paso " + std::to_string(index_ + 1) + ": " + reason;
    }

    // AUTOMATICO (true) o PASO A PASO (false). Cuenta desde el paso
    // siguiente: no arranca un paso que este esperando el boton.
    void setAutomatic(bool automatic) { p_.confirm_each_step = !automatic; }
    bool automatic() const { return !p_.confirm_each_step; }

    Status status() const { return status_; }
    size_t stepIndex() const { return index_; }       // 0-based; == stepCount() al terminar
    size_t stepCount() const { return p_.steps.size(); }
    double stepTime() const { return step_time_; }
    const std::string& message() const { return message_; }
    const TaskParams& params() const { return p_; }
    bool finished() const { return status_ == Status::Done || status_ == Status::Aborted; }

    // Objetivo de la interfaz del paso Move actual (si lo es y ya se conoce).
    bool currentMoveTarget(InterfaceTarget& out) const
    {
        if (index_ < p_.steps.size() && p_.steps[index_].vision_approach && approach_moving_)
        {
            out = targets_[index_];
            return true;
        }
        if (index_ >= p_.steps.size() || p_.steps[index_].type != StepType::Move)
            return false;
        if (p_.steps[index_].from_vision && status_ == Status::WaitingConfirm)
            return false;   // se calcula al iniciar el paso
        out = targets_[index_];
        return true;
    }

    // Ultima posicion valida de la vision (TCP en la base).
    bool visionPosition(Eigen::Vector3d& out) const
    {
        if (!have_vision_)
            return false;
        out = vision_tcp_;
        return true;
    }

    // Orientacion y pre-grasp de la ultima vision valida (solo servidor V2).
    bool visionPose(Eigen::Vector3d& rpy_deg, Eigen::Vector3d& pregrasp) const
    {
        if (!have_vision_ || !vision_has_pose_)
            return false;
        rpy_deg = vision_rpy_;
        pregrasp = vision_pregrasp_;
        return true;
    }

    // Texto para el tablero.
    std::string statusText() const
    {
        std::ostringstream o;
        o.setf(std::ios::fixed);
        o.precision(3);
        o << " Estado                  : " << statusName(status_) << "\n";
        o << " Modo                    : " << (automatic() ? "AUTOMATICO" : "PASO A PASO") << "\n";
        if (index_ < p_.steps.size())
            o << " Paso                    : " << index_ + 1 << "/" << p_.steps.size() << "  "
              << describe(index_) << "\n";
        else
            o << " Paso                    : " << p_.steps.size() << "/" << p_.steps.size() << "\n";
        if (status_ == Status::Running)
            o << " Tiempo en el paso       : " << step_time_ << " s\n";

        InterfaceTarget t;
        if (currentMoveTarget(t) && status_ != Status::WaitingConfirm)
        {
            o << " Error de posicion       : " << (last_in_.position - t.position).norm() * 100.0
              << " cm (tol " << p_.position_tolerance_m * 100.0 << ")\n"
              << " Error de orientacion    : "
              << angleBetween(last_in_.orientation, t.orientation) * 180.0 / 3.14159265358979323846
              << " grados (tol " << p_.orientation_tolerance_deg << ")\n";
        }
        o << " Mensaje                 : " << message_ << "\n";
        if (!last_in_.active && !finished())
            o << " (congelada: el control no esta activo; enciende el slider)\n";
        return o.str();
    }

    std::string describe(size_t i) const
    {
        if (i >= p_.steps.size())
            return "";
        const Step& s = p_.steps[i];
        std::ostringstream o;
        o << "[" << stepTypeName(s.type) << "] " << s.label;
        switch (s.type)
        {
            case StepType::Move:
                if (s.from_vision && s.vision_target == VisionTarget::PlateFixture)
                    o << " -> pose '" << s.pose << "' corregida por donde se agarro la placa";
                else if (s.from_vision && s.vision_target == VisionTarget::Hole && s.vision_dz_m == 0.0)
                    o << " -> pose '" << s.pose << "' con x y sobre el hoyo de la vision";
                else if (s.from_vision && s.vision_target == VisionTarget::Hole)
                    o << " -> pose '" << s.pose << "' " << (s.vision_dz_m >= 0.0 ? "+ " : "- ")
                      << std::abs(s.vision_dz_m) * 100.0 << " cm en z, con x y sobre el hoyo de la vision";
                else if (s.from_vision && s.vision_target == VisionTarget::Pregrasp)
                    o << " -> pre-grasp de la vision";
                else if (s.from_vision && s.vision_target == VisionTarget::Grasp && std::isfinite(s.vision_z_m))
                    o << " -> agarre de la vision hasta z = " << s.vision_z_m * 100.0
                      << " cm (orientacion del agarre)";
                else if (s.from_vision && s.vision_target == VisionTarget::Grasp && s.vision_dz_m == 0.0)
                    o << " -> agarre de la vision";
                else if (s.from_vision && s.vision_target == VisionTarget::Grasp)
                    o << " -> agarre de la vision " << (s.vision_dz_m >= 0.0 ? "+ " : "- ")
                      << std::abs(s.vision_dz_m) * 100.0 << " cm en z (orientacion del agarre)";
                else if (s.from_vision)
                    o << " -> vision " << (s.vision_dz_m >= 0.0 ? "+ " : "- ")
                      << std::abs(s.vision_dz_m) * 100.0 << " cm en z (orientacion de '" << s.pose
                      << "')";
                else
                    o << " -> pose '" << s.pose << "'";
                break;
            case StepType::Gripper: o << " -> " << s.gripper_pct << " %"; break;
            case StepType::Fixture: o << " -> " << s.fixture_deg << " grados"; break;
            case StepType::Wait:    o << " -> " << s.wait_s << " s"; break;
            case StepType::Vision:
                if (s.vision_approach)
                {
                    o << " -> centrar la camara 30 cm arriba del "
                      << (s.vision_object == "PLATE" ? "la placa" : "perno") << " (por su color)";
                    break;
                }
                o << " -> foto en '" << s.pose << "'"
                  << (s.vision_object == "PLATE"  ? " (placa por su color"
                      : s.vision_object == "BOLT" ? " (perno por su color" : "")
                  << (s.vision_object.empty() ? "" : ")")
                  << (s.vision_needs_hole   ? " (solo nube, sin GPD; hoyo mas chico)"
                      : s.vision_cloud_only ? " (solo nube, sin GPD)"
                      : s.vision_needs_edge ? " (y canto de la placa)" : "");
                break;
        }
        return o.str();
    }

private:
    void activate(Commands& cmd)
    {
        const Step& s = p_.steps[index_];
        status_ = Status::Running;
        step_time_ = 0.0;
        settle_time_ = 0.0;
        gripper_window_time_ = 0.0;
        gripper_window_start_ = last_in_.gripper_pos_pct;
        message_ = "Ejecutando " + describe(index_);

        switch (s.type)
        {
            case StepType::Move:
            {
                if (s.from_vision && s.vision_target == VisionTarget::Hole)
                {
                    if (!have_hole_)
                    {
                        abort("no hay hoyo de la vision");
                        return;
                    }
                    const TaskPose* fixed = findPose(p_, s.pose);
                    const Eigen::Vector2d xy = hole_.head<2>() + p_.hole_offset_xy_m;
                    const double shift = (xy - fixed->tcp_position_m.head<2>()).norm();
                    if (!(shift <= p_.hole_max_shift_m))
                    {
                        std::ostringstream why;
                        why.setf(std::ios::fixed);
                        why.precision(3);
                        why << "el hoyo queda a " << shift << " m en horizontal de '" << s.pose
                            << "' (maximo " << p_.hole_max_shift_m << " m)";
                        abort(why.str());
                        return;
                    }
                    const TaskPose goal{"hoyo",
                                        Eigen::Vector3d(xy.x(), xy.y(), fixed->tcp_position_m.z() + s.vision_dz_m),
                                        fixed->rpy_deg};
                    if (!inWorkspace(p_, goal.tcp_position_m))
                    {
                        abort("el objetivo sobre el hoyo queda fuera de la caja de trabajo");
                        return;
                    }
                    targets_[index_] = toInterface(goal, p_.tcp_offset_m);
                    std::ostringstream how;
                    how.setf(std::ios::fixed);
                    how.precision(1);
                    how << " (hoyo de " << hole_diam_ * 1000.0 << " mm, " << shift * 100.0 << " cm de '"
                        << s.pose << "')";
                    message_ = "Ejecutando " + describe(index_) + how.str();
                }
                else if (s.from_vision)
                {
                    if (!have_vision_)
                    {
                        abort("no hay posicion de la vision");
                        return;
                    }
                    if (s.vision_target == VisionTarget::PlateFixture)
                    {
                        InterfaceTarget t;
                        std::string why;
                        if (!plateFixtureTarget(*findPose(p_, s.pose), t, why))
                        {
                            abort(why);
                            return;
                        }
                        targets_[index_] = t;
                        message_ = "Ejecutando " + describe(index_) + " (" + plate_correction_text_ + ")";
                    }
                    TaskPose goal;
                    if (s.vision_target == VisionTarget::PlateFixture)
                    {
                        // ya calculado arriba
                    }
                    else if (s.vision_target == VisionTarget::PositionDz)
                    {
                        const TaskPose* o = findPose(p_, s.pose);
                        goal = {"vision", vision_tcp_ + Eigen::Vector3d(0.0, 0.0, s.vision_dz_m),
                                o->rpy_deg};
                    }
                    else
                    {
                        if (!vision_has_pose_)
                        {
                            abort("la vision no dio orientacion ni pre-grasp");
                            return;
                        }
                        goal = {"vision",
                                s.vision_target == VisionTarget::Pregrasp
                                    ? vision_pregrasp_
                                    : Eigen::Vector3d(vision_tcp_ + Eigen::Vector3d(0.0, 0.0, s.vision_dz_m)),
                                vision_rpy_};
                        if (s.vision_target == VisionTarget::Grasp && std::isfinite(s.vision_z_m))
                        {
                            if (s.vision_z_m < vision_tcp_.z())
                            {
                                std::ostringstream why;
                                why.setf(std::ios::fixed);
                                why.precision(3);
                                why << "la z fija " << s.vision_z_m << " m queda abajo del agarre ("
                                    << vision_tcp_.z() << " m)";
                                abort(why.str());
                                return;
                            }
                            goal.tcp_position_m.z() = s.vision_z_m;
                        }
                    }
                    if (s.vision_target != VisionTarget::PlateFixture)
                    {
                        if (!inWorkspace(p_, goal.tcp_position_m))
                        {
                            abort("el objetivo de vision queda fuera de la caja de trabajo");
                            return;
                        }
                        targets_[index_] = toInterface(goal, p_.tcp_offset_m);
                    }
                }
                // Revision al ejecutar: del robot (medido) al objetivo.
                const double d = (targets_[index_].position - last_in_.position).norm();
                if (!std::isfinite(d) || d > p_.max_move_distance_m)
                {
                    std::ostringstream why;
                    why.setf(std::ios::fixed);
                    why.precision(3);
                    why << "movimiento de " << d << " m desde la posicion actual (maximo "
                        << p_.max_move_distance_m << " m)";
                    abort(why.str());
                    return;
                }
                cmd.set_reference = true;
                cmd.position = targets_[index_].position;
                cmd.orientation = targets_[index_].orientation;

                // Con vision, s.pose da la postura (y la orientacion con
                // PositionDz).
                if (const TaskPose* pose = findPose(p_, s.pose); pose && pose->hasJoints())
                {
                    cmd.set_posture = true;
                    cmd.posture_rad = jointsToRad(pose->joints_deg);
                }
                break;
            }
            case StepType::Gripper:
                cmd.set_gripper = true;
                cmd.gripper_pct = s.gripper_pct;
                cmd.gripper_speed_pct = s.gripper_speed_pct;
                cmd.gripper_force_pct = s.gripper_force_pct;
                break;
            case StepType::Fixture:
                cmd.move_fixture = true;
                cmd.fixture_deg = s.fixture_deg;
                break;
            case StepType::Wait:
                break;
            case StepType::Vision:
                approach_moving_ = false;
                approach_moves_ = 0;
                requestVision(cmd);
                break;
        }
    }

    void requestVision(Commands& cmd)
    {
        const Step& s = p_.steps[index_];
        ++vision_seq_;
        cmd.request_vision = true;
        cmd.vision_seq = vision_seq_;
        cmd.vision_cloud_only = s.vision_cloud_only;
        cmd.vision_object = s.vision_object;
        cmd.vision_approach = s.vision_approach;
        message_ = "Ejecutando " + describe(index_) + " (esperando a la vision, foto #" +
                   std::to_string(vision_seq_) +
                   (s.vision_approach ? ", movimiento " + std::to_string(approach_moves_) : "") + ")";
    }

    // Respuesta APPROACH: en vista (INVIEW / ATTARGET) y ya en la pose ->
    // termina; si no, va a la pose y luego pregunta otra vez (foto nueva).
    void approachReply(const Inputs& in, Commands& cmd)
    {
        const Step& s = p_.steps[index_];
        if (!in.vision_is_approach)
        {
            pause("el servidor no contesto al APPROACH (screw_server de kinova_grasp_NEW_VER?)");
            return;
        }
        const Eigen::Vector3d& tcp = in.vision_tcp_m;
        const TaskPose* near = findPose(p_, s.pose);
        std::ostringstream why;
        why.setf(std::ios::fixed);
        why.precision(3);
        const bool in_view = in.vision_approach_state == 2 || in.vision_approach_state == 3;
        const double far = (toInterface({"approach", tcp, in.vision_rpy_deg}, p_.tcp_offset_m).position -
                            in.position).norm();
        const bool centered = far <= p_.approach_center_tol_m;
        if (in_view && (centered || approach_moves_ >= p_.approach_max_moves))
        {
            std::ostringstream how;
            how.setf(std::ios::fixed);
            how.precision(3);
            how << "terminado: " << (in.vision_approach_state == 2 ? "IN VIEW" : "AT TARGET")
                << (centered ? " y centrado" : "") << ", objeto en x " << in.vision_centroid_m.x()
                << "  y " << in.vision_centroid_m.y() << "  z " << in.vision_centroid_m.z() << " m ("
                << approach_moves_ << " movimientos";
            if (!centered)
                how << "; quedo a " << far * 100.0 << " cm de la pose centrada";
            how << ")";
            finishStep(cmd, how.str());
            return;
        }
        if (approach_moves_ >= p_.approach_max_moves)
        {
            why << "el objeto no quedo completo en la imagen en " << p_.approach_max_moves << " movimientos";
            pause(why.str());
            return;
        }
        // revision del objetivo
        const Eigen::Vector3d z_tool = rpyToQuaternion(in.vision_rpy_deg) * Eigen::Vector3d::UnitZ();
        const double tilt = std::acos(std::clamp(-z_tool.z(), -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
        const double dxy = (tcp.head<2>() - near->tcp_position_m.head<2>()).norm();
        if (!tcp.allFinite() || !in.vision_rpy_deg.allFinite())
            why << "pose no finita";
        else if (!inWorkspace(p_, tcp))
            why << "pose " << tcp.transpose() << " m fuera de la caja de trabajo";
        else if (dxy > p_.approach_max_xy_m)
            why << "pose a " << dxy << " m en horizontal de '" << near->name << "' (maximo "
                << p_.approach_max_xy_m << " m)";
        else if (tilt > p_.approach_max_tilt_deg)
            why << "la camara no mira hacia abajo (" << tilt << " grados)";
        else if (!(far <= p_.approach_max_move_m))
            why << "movimiento de " << far << " m (maximo " << p_.approach_max_move_m << " m)";
        if (!why.str().empty())
        {
            pause("centrado rechazado: " + why.str());
            return;
        }
        targets_[index_] = toInterface({"approach", tcp, in.vision_rpy_deg}, p_.tcp_offset_m);
        cmd.set_reference = true;
        cmd.position = targets_[index_].position;
        cmd.orientation = targets_[index_].orientation;
        if (const TaskPose* pose = findPose(p_, s.pose); pose && pose->hasJoints())
        {
            cmd.set_posture = true;
            cmd.posture_rad = jointsToRad(pose->joints_deg);
        }
        approach_moving_ = true;
        ++approach_moves_;
        settle_time_ = 0.0;
        move_time_ = 0.0;
        std::ostringstream how;
        how.setf(std::ios::fixed);
        how.precision(1);
        how << "Ejecutando " << describe(index_) << " (" << (in_view ? "IN VIEW, centrando" : "MOVE")
            << ": movimiento " << approach_moves_ << " de " << far * 100.0 << " cm)";
        message_ = how.str();
    }

    void finishStep(Commands& cmd, const std::string& how)
    {
        const std::string done = "Paso " + std::to_string(index_ + 1) + " " + how;
        ++index_;
        step_time_ = 0.0;

        if (index_ >= p_.steps.size())
        {
            status_ = Status::Done;
            message_ = done + ". Secuencia terminada.";
            return;
        }
        if (p_.confirm_each_step)
        {
            status_ = Status::WaitingConfirm;
            message_ = done + ". Presiona 'Siguiente paso': " + describe(index_);
            return;
        }
        activate(cmd);
    }

    // Boton "Pausa": con un paso en curso el brazo se queda donde esta; la
    // respuesta de vision que llegue mientras tanto se ignora (al reanudar se
    // pide otra).
    void operatorPause(const Inputs& in, Commands& cmd)
    {
        if (status_ == Status::Running)
        {
            cmd.set_reference = true;
            cmd.position = in.position;
            cmd.orientation = in.orientation;
        }
        approach_moving_ = false;
        operator_paused_ = true;
        status_ = Status::Paused;
        message_ = "PAUSA (operador) en el paso " + std::to_string(index_ + 1) + " " + describe(index_) +
                   ": el brazo se queda donde esta. 'Siguiente paso' reanuda este paso.";
    }

    void pause(const std::string& why)
    {
        status_ = Status::Paused;
        message_ = "PAUSA en el paso " + std::to_string(index_ + 1) + ": " + why +
                   (p_.steps[index_].type == StepType::Vision
                        ? ". 'Siguiente paso' toma otra foto."
                        : ". 'Siguiente paso' lo da por terminado y continua.");
    }

    // "" si el hoyo de la vision es aceptable para el paso actual: debajo de
    // la pose de la foto y cerca de ella.
    std::string checkHole(const Inputs& in) const
    {
        if (!in.vision_has_hole)
            return "no se encontro ningun hoyo redondo en la placa (se ve toda la placa? screw_server "
                   "actualizado?)";
        const Eigen::Vector3d& h = in.vision_hole_m;
        const TaskPose* near = findPose(p_, p_.steps[index_].pose);
        std::ostringstream why;
        why.setf(std::ios::fixed);
        why.precision(3);
        if (!h.allFinite() || !std::isfinite(in.vision_hole_diam_m) || in.vision_hole_diam_m <= 0.0)
            return "hoyo no finito";
        if (!inWorkspace(p_, h))
        {
            why << "hoyo en " << h.transpose() << " m, fuera de la caja de trabajo";
            return why.str();
        }
        const double dxy = (h.head<2>() - near->tcp_position_m.head<2>()).norm();
        if (dxy > p_.hole_max_from_photo_m)
        {
            why << "hoyo a " << dxy << " m en horizontal de '" << near->name << "' (maximo "
                << p_.hole_max_from_photo_m << " m)";
            return why.str();
        }
        if (h.z() > near->tcp_position_m.z() - p_.vision_min_below_m)
        {
            why << "hoyo (z = " << h.z() << " m) no esta debajo de '" << near->name << "'";
            return why.str();
        }
        return "";
    }

    // "" si el agarre de la vision es aceptable para el paso actual.
    std::string checkVision(const Inputs& in) const
    {
        const Eigen::Vector3d& tcp = in.vision_tcp_m;
        const TaskPose* near = findPose(p_, p_.steps[index_].pose);
        std::ostringstream why;
        why.setf(std::ios::fixed);
        why.precision(3);
        if (!tcp.allFinite())
            return "posicion no finita";
        if (!inWorkspace(p_, tcp))
        {
            why << "posicion " << tcp.transpose() << " m fuera de la caja de trabajo";
            return why.str();
        }
        const double dxy = (tcp.head<2>() - near->tcp_position_m.head<2>()).norm();
        if (dxy > p_.vision_max_xy_m)
        {
            why << "posicion a " << dxy << " m en horizontal de '" << near->name << "' (maximo "
                << p_.vision_max_xy_m << " m)";
            return why.str();
        }
        if (tcp.z() > near->tcp_position_m.z() - p_.vision_min_below_m)
        {
            why << "posicion (z = " << tcp.z() << " m) no esta debajo de '" << near->name << "'";
            return why.str();
        }

        if (p_.steps[index_].vision_needs_edge && !in.vision_has_edge)
            return "no se midio el canto de la placa junto al agarre (se ven los dos extremos? "
                   "screw_server actualizado?)";
        if (p_.steps[index_].vision_needs_edge && in.vision_has_pose && in.vision_rpy_deg.allFinite())
        {
            // componente del canto sobre el eje de cierre (x de la pinza)
            const Eigen::Vector3d x_close = rpyToQuaternion(in.vision_rpy_deg) * Eigen::Vector3d::UnitX();
            const Eigen::Vector3d e = (in.vision_edge_b_m - in.vision_edge_a_m).normalized();
            const double skew_deg =
                std::asin(std::clamp(std::abs(e.dot(x_close)), 0.0, 1.0)) * 180.0 / 3.14159265358979323846;
            if (!e.allFinite() || skew_deg > p_.plate_max_skew_deg)
            {
                why.precision(0);
                why << "la pinza cierra a " << skew_deg << " grados de perpendicular a la placa (maximo "
                    << p_.plate_max_skew_deg << ")";
                return why.str();
            }
        }
        if (!in.vision_has_pose)
            return needs_pose_ ? "el servidor no manda orientacion ni pre-grasp (usa el "
                                 "screw_server de kinova_grasp_V2)"
                               : "";

        const Eigen::Vector3d& pre = in.vision_pregrasp_m;
        if (!in.vision_rpy_deg.allFinite() || !pre.allFinite())
            return "orientacion o pre-grasp no finita";
        // Inclinacion: angulo entre el eje z de la pinza y la vertical hacia abajo.
        const Eigen::Vector3d z_tool = rpyToQuaternion(in.vision_rpy_deg) * Eigen::Vector3d::UnitZ();
        const double tilt_deg =
            std::acos(std::clamp(-z_tool.z(), -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
        if (tilt_deg > p_.vision_max_tilt_deg)
        {
            why.precision(1);
            why << "pinza inclinada " << tilt_deg << " grados de la vertical (maximo "
                << p_.vision_max_tilt_deg << ")";
            return why.str();
        }
        if (!inWorkspace(p_, pre))
        {
            why << "pre-grasp " << pre.transpose() << " m fuera de la caja de trabajo";
            return why.str();
        }
        if ((pre - tcp).norm() > p_.vision_max_pregrasp_m || pre.z() <= tcp.z())
        {
            why << "pre-grasp " << pre.transpose() << " m no esta encima del agarre (maximo "
                << p_.vision_max_pregrasp_m << " m)";
            return why.str();
        }
        return "";
    }

    void runStep(const Inputs& in, Commands& cmd)
    {
        const Step& s = p_.steps[index_];

        switch (s.type)
        {
            case StepType::Move:
            {
                const InterfaceTarget& t = targets_[index_];
                const double e_p = (in.position - t.position).norm();
                const double e_r = angleBetween(in.orientation, t.orientation);
                const bool ok =
                    e_p <= p_.position_tolerance_m &&
                    e_r <= p_.orientation_tolerance_deg * 3.14159265358979323846 / 180.0 &&
                    in.linear_speed_m_s <= p_.still_linear_speed_m_s &&
                    in.angular_speed_rad_s <= p_.still_angular_speed_rad_s;

                settle_time_ = ok ? settle_time_ + in.dt : 0.0;
                if (settle_time_ >= p_.settle_s)
                {
                    // Agarre de la vision (sin dz): se guarda donde quedo la TCP
                    // MEDIDA, que es donde la pinza va a cerrar sobre la placa.
                    if (s.from_vision && s.vision_target == VisionTarget::Grasp && s.vision_dz_m == 0.0 &&
                        !std::isfinite(s.vision_z_m))
                    {
                        grasp_R_ = in.orientation.normalized().toRotationMatrix();
                        grasp_tcp_ = in.position + grasp_R_ * Eigen::Vector3d(0.0, 0.0, p_.tcp_offset_m);
                        have_grasp_measured_ = true;
                    }
                    finishStep(cmd, "terminado");
                }
                else if (step_time_ > p_.move_timeout_s)
                {
                    std::ostringstream why;
                    why.setf(std::ios::fixed);
                    why.precision(1);
                    why << "no llego en " << p_.move_timeout_s << " s (error " << e_p * 100.0
                        << " cm, " << e_r * 180.0 / 3.14159265358979323846 << " grados)";
                    pause(why.str());
                }
                break;
            }

            case StepType::Gripper:
            {
                if (!in.gripper_detected)
                {
                    abort("no hay pinza");
                    break;
                }
                // Termina cuando la posicion deja de cambiar durante una ventana.
                gripper_window_time_ += in.dt;
                if (gripper_window_time_ >= p_.gripper_settle_s)
                {
                    const bool still =
                        std::abs(in.gripper_pos_pct - gripper_window_start_) <= p_.gripper_still_pct;
                    gripper_window_time_ = 0.0;
                    gripper_window_start_ = in.gripper_pos_pct;

                    if (still && step_time_ >= p_.gripper_min_time_s)
                    {
                        std::ostringstream how;
                        how.setf(std::ios::fixed);
                        how.precision(1);
                        how << "terminado (pinza en " << in.gripper_pos_pct << " %)";
                        finishStep(cmd, how.str());
                        break;
                    }
                }
                if (step_time_ > p_.gripper_timeout_s)
                    pause("la pinza no se detuvo en " + std::to_string(p_.gripper_timeout_s) + " s");
                break;
            }

            case StepType::Fixture:
                if (in.fixture_fault)
                    abort("falla del fixture: " + in.fixture_fault_text);
                else if (in.fixture_settled)
                    finishStep(cmd, "terminado");
                else if (step_time_ > p_.fixture_timeout_s)
                    pause("el fixture no llego en " + std::to_string(p_.fixture_timeout_s) + " s");
                break;

            case StepType::Wait:
                if (step_time_ >= s.wait_s)
                    finishStep(cmd, "terminado");
                break;

            case StepType::Vision:
                if (s.vision_approach && approach_moving_)
                {
                    // llegada como un Move; luego otra foto para ver si ya quedo centrado
                    move_time_ += in.dt;
                    const InterfaceTarget& t = targets_[index_];
                    const double e_p = (in.position - t.position).norm();
                    const double e_r = angleBetween(in.orientation, t.orientation);
                    const bool ok =
                        e_p <= p_.position_tolerance_m &&
                        e_r <= p_.orientation_tolerance_deg * 3.14159265358979323846 / 180.0 &&
                        in.linear_speed_m_s <= p_.still_linear_speed_m_s &&
                        in.angular_speed_rad_s <= p_.still_angular_speed_rad_s;
                    settle_time_ = ok ? settle_time_ + in.dt : 0.0;
                    if (settle_time_ >= p_.settle_s)
                    {
                        approach_moving_ = false;
                        step_time_ = 0.0;   // tiempo de la vision desde la nueva foto
                        requestVision(cmd);
                    }
                    else if (move_time_ > p_.move_timeout_s)
                    {
                        std::ostringstream why;
                        why.setf(std::ios::fixed);
                        why.precision(1);
                        why << "no llego a la pose de centrado en " << p_.move_timeout_s << " s (error "
                            << e_p * 100.0 << " cm)";
                        pause(why.str());
                    }
                    break;
                }
                if (in.vision_seq == vision_seq_ && s.vision_approach)
                {
                    if (!in.vision_ok)
                        pause("la vision fallo: " + in.vision_text);
                    else
                        approachReply(in, cmd);
                    break;
                }
                if (in.vision_seq == vision_seq_)
                {
                    if (!in.vision_ok)
                    {
                        pause("la vision fallo: " + in.vision_text);
                        break;
                    }
                    if (s.vision_cloud_only != in.vision_is_cloud)
                    {
                        pause(s.vision_cloud_only ? "el servidor no contesto con una nube (screw_server actualizado?)"
                                                  : "el servidor contesto con una nube, no con un agarre");
                        break;
                    }
                    if (s.vision_cloud_only && s.vision_needs_hole)
                    {
                        const std::string bad = checkHole(in);
                        if (!bad.empty())
                        {
                            pause("hoyo rechazado: " + bad);
                            break;
                        }
                        hole_ = in.vision_hole_m;
                        hole_diam_ = in.vision_hole_diam_m;
                        have_hole_ = true;
                        std::ostringstream how;
                        how.setf(std::ios::fixed);
                        how.precision(4);
                        how << "terminado: hoyo mas chico en x " << hole_.x() << "  y " << hole_.y() << "  z "
                            << hole_.z() << " m";
                        how.precision(1);
                        how << ", diametro " << hole_diam_ * 1000.0 << " mm (" << in.vision_text << ")";
                        finishStep(cmd, how.str());
                        break;
                    }
                    if (s.vision_cloud_only)
                    {
                        finishStep(cmd, "terminado: nube guardada (" + in.vision_text + ")");
                        break;
                    }
                    const std::string bad = checkVision(in);
                    if (!bad.empty())
                    {
                        pause("vision rechazada: " + bad);
                        break;
                    }
                    vision_tcp_ = in.vision_tcp_m;
                    vision_has_pose_ = in.vision_has_pose;
                    vision_rpy_ = in.vision_rpy_deg;
                    vision_pregrasp_ = in.vision_pregrasp_m;
                    have_vision_ = true;
                    have_edge_ = in.vision_has_edge;
                    edge_a_ = in.vision_edge_a_m;
                    edge_b_ = in.vision_edge_b_m;
                    have_grasp_measured_ = false;   // foto nueva: el agarre medido es del siguiente
                    std::ostringstream how;
                    how.setf(std::ios::fixed);
                    how.precision(3);
                    how << "terminado: tornillo (TCP) en x " << vision_tcp_.x() << "  y "
                        << vision_tcp_.y() << "  z " << vision_tcp_.z() << " m";
                    if (vision_has_pose_)
                    {
                        how.precision(1);
                        how << ", Rx Ry Rz " << vision_rpy_.x() << " " << vision_rpy_.y() << " "
                            << vision_rpy_.z() << " grados";
                        how.precision(3);
                        how << ", pre-grasp z " << vision_pregrasp_.z() << " m";
                    }
                    if (have_edge_)
                    {
                        how.precision(1);
                        how << ", canto de la placa " << (edge_b_ - edge_a_).norm() * 100.0 << " cm";
                    }
                    if (!in.vision_text.empty())
                        how << " (" << in.vision_text << ")";
                    finishStep(cmd, how.str());
                }
                else if (step_time_ > p_.vision_timeout_s)
                {
                    pause("la vision no contesto en " + std::to_string(p_.vision_timeout_s) +
                          " s (corre screw_server?)");
                }
                break;
        }
    }

    TaskParams p_;
    std::vector<InterfaceTarget> targets_;

    Status status_ = Status::WaitingConfirm;
    size_t index_ = 0;
    double step_time_ = 0.0;
    double settle_time_ = 0.0;
    double gripper_window_time_ = 0.0;
    double gripper_window_start_ = 0.0;
    std::string message_;
    Inputs last_in_{};

    uint32_t vision_seq_ = 0;     // numero de la ultima foto pedida
    // visionApproach: moviendose a la pose de centrado, movimientos hechos y
    // si este movimiento es el centrado final (objeto ya en vista).
    bool approach_moving_ = false;
    bool operator_paused_ = false;   // la pausa la pidio el operador (boton Pausa)
    int approach_moves_ = 0;
    double move_time_ = 0.0;
    bool have_vision_ = false;
    Eigen::Vector3d vision_tcp_ = Eigen::Vector3d::Zero();
    bool vision_has_pose_ = false;
    Eigen::Vector3d vision_rpy_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vision_pregrasp_ = Eigen::Vector3d::Zero();
    bool needs_pose_ = false;     // algun paso usa la orientacion / pre-grasp
    // Placa: canto de la ultima foto y agarre medido.
    bool have_edge_ = false;
    // Hoyo de la ultima visionCaptureHole aceptada.
    bool have_hole_ = false;
    Eigen::Vector3d hole_ = Eigen::Vector3d::Zero();
    double hole_diam_ = 0.0;
    Eigen::Vector3d edge_a_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d edge_b_ = Eigen::Vector3d::Zero();
    bool have_grasp_measured_ = false;
    Eigen::Matrix3d grasp_R_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d grasp_tcp_ = Eigen::Vector3d::Zero();
    std::string plate_correction_text_;

public:
    // Pose de la interfaz para 'fixed' (grabada con la placa agarrada al
    // centro) corregida por el agarre real. Ver moveToPlateCorrected.
    //
    // En el frame de la TCP al agarrar (x = cierre, y = a lo largo de los
    // dedos, z = aproximacion), la placa queda con:
    //   o   = centro del canto, sin la componente x (los dedos la centran),
    //   phi = giro del canto alrededor de x (respecto a y).
    // Referencia: o_ref = (0, 0, plate_nominal_tcp_above_edge_m), phi = 0.
    // Para que la placa quede igual que con la referencia en la pose fija F:
    //   T = T_F * P_ref * inv(P)   ->   R = R_F * Rx(-phi),
    //                                   p = p_F + R_F * (o_ref - Rx(-phi) * o)
    bool plateFixtureTarget(const TaskPose& fixed, InterfaceTarget& out, std::string& why)
    {
        constexpr double PI = 3.14159265358979323846;
        if (!have_edge_)
        {
            why = "no hay canto de la placa (falta visionCapturePlate antes)";
            return false;
        }
        Eigen::Matrix3d Rg;
        Eigen::Vector3d tg;
        if (have_grasp_measured_)
        {
            Rg = grasp_R_;
            tg = grasp_tcp_;
        }
        else
        {
            Rg = rpyToQuaternion(vision_rpy_).toRotationMatrix();
            tg = vision_tcp_;
        }
        const Eigen::Vector3d c = 0.5 * (edge_a_ + edge_b_);
        Eigen::Vector3d e_t = Rg.transpose() * (edge_b_ - edge_a_).normalized();
        e_t.x() = 0.0;   // los dedos alinean la placa con su plano
        if (e_t.norm() < 0.5)
        {
            why = "el canto de la placa no queda entre los dedos (casi paralelo al cierre)";
            return false;
        }
        e_t.normalize();
        double phi = std::atan2(e_t.z(), e_t.y());
        if (phi > PI / 2) phi -= PI;     // la placa da igual girada 180 grados
        if (phi < -PI / 2) phi += PI;
        Eigen::Vector3d o = Rg.transpose() * (c - tg);
        o.x() = 0.0;
        const Eigen::Vector3d o_ref(0.0, 0.0, p_.plate_nominal_tcp_above_edge_m);

        const Eigen::Matrix3d RF = rpyToQuaternion(fixed.rpy_deg).toRotationMatrix();
        const Eigen::Matrix3d Rc = Eigen::AngleAxisd(-phi, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const Eigen::Matrix3d R = RF * Rc;
        const Eigen::Vector3d tcp = fixed.tcp_position_m + RF * (o_ref - Rc * o);

        const double shift = (tcp - fixed.tcp_position_m).norm();
        const double ang = std::abs(phi) * 180.0 / PI;
        std::ostringstream info;
        info.setf(std::ios::fixed);
        info.precision(1);
        info << "placa agarrada a " << o.y() * 100.0 << " cm del centro del canto, TCP "
             << o.z() * 100.0 << " cm arriba del canto (ref " << o_ref.z() * 100.0 << "), giro " << ang
             << " grados; correccion " << shift * 100.0 << " cm"
             << (have_grasp_measured_ ? "" : " (agarre de la vision, no medido)");
        plate_correction_text_ = info.str();
        if (!tcp.allFinite() || shift > p_.plate_max_shift_m || ang > p_.plate_max_angle_deg)
        {
            why = "correccion de la placa demasiado grande: " + plate_correction_text_;
            return false;
        }
        if (!inWorkspace(p_, tcp))
        {
            why = "la pose corregida del fixture queda fuera de la caja de trabajo";
            return false;
        }
        out.orientation = Eigen::Quaterniond(R).normalized();
        out.position = tcp - out.orientation * Eigen::Vector3d(0.0, 0.0, p_.tcp_offset_m);
        return true;
    }

    const std::string& plateCorrectionText() const { return plate_correction_text_; }

    // Hoyo de la vision aceptado (base [m]) y su diametro [m].
    bool visionHole(Eigen::Vector3d& center, double& diameter_m) const
    {
        if (!have_hole_)
            return false;
        center = hole_;
        diameter_m = hole_diam_;
        return true;
    }
};

}  // namespace task
