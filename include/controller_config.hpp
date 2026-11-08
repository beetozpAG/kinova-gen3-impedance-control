#pragma once

#include "cartesian_impedance_controller.hpp"
#include "grasp_pipeline.hpp"
#include "task_sequence.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace project_config
{

// ============================================================
// SECUENCIA DE TAREA - FASE 2: PLACA -> FIXTURE
// ============================================================
//
// Se activa con  task.enabled = true  (abajo, en ControllerConfig).
// Todo lo demas de la secuencia se edita AQUI.
//
// POSES: copiadas TAL CUAL de la app web de Kinova (Tool configuration de
// 0.12 m): X Y Z en METROS, Rx Ry Rz en GRADOS (R = Rz * Ry * Rx).
// El programa las pasa solo a la interfaz (ya no hay que sumar los 12 cm).
// Una pose sin definir (undefinedPose) hace que el programa NO arranque.
//
// Poses grabadas en el lab el 2026-10-13.

// QUE SECUENCIA CORRE (cambiar aqui y recompilar):
//   Completa        -> TODO (2026-10-20): placa por vision (foto en
//                      ARRIBA_PLACA1_NEW, agarre + canto) -> fixture con
//                      ARRIBA_FIXTURE_NEW / COLOCAR_PLACA corregidas por donde
//                      se agarro -> tornillo por vision -> encima del hoyo,
//                      nube del hoyo (sin GPD) -> soltar el tornillo.
//                      Arranca en el paso 1.
//   SoloPlacaVision -> solo la placa por vision hasta el fixture (2026-10-19).
//   Normal          -> placa con poses fijas + tornillo; arranca en el
//                      paso 20 (el tornillo).
enum class Secuencia
{
    Normal,
    SoloPlacaVision,
    Completa
};
inline constexpr Secuencia SECUENCIA = Secuencia::Completa;

inline task::TaskParams phase2Task(Secuencia mode = SECUENCIA)
{
    task::TaskParams t;

    using V = Eigen::Vector3d;
    // Articulaciones de la app web [grados] en cada pose (2026-10-14): postura
    // del codo para el espacio nulo (enable_nullspace). Revisadas con el modelo
    // (Gen3_model + 12.4 cm de pinza): coinciden con la pose cartesiana a
    // < 1 cm salvo donde dice otra cosa.
    auto Q = [](double j1, double j2, double j3, double j4, double j5, double j6, double j7)
    {
        Eigen::Matrix<double, 7, 1> q;
        q << j1, j2, j3, j4, j5, j6, j7;
        return q;
    };
    t.poses = {
        {"HOME_MOD",           V(0.536, 0.012, 0.395), V(0.0, 180.0, 0.0),
                               Q(356.3, 31.7, 0.12, 50.4, 0.6, 97.3, 356.7)},
        // Intermedios (2026-10-14): HOME_MOD -> placa y HOME_MOD -> tornillos.
        // Articulaciones: 5.7 cm / 5 grados de la pose (z 0.338 en el modelo);
        // la pose es la buena (revisada en el lab), solo afecta la postura del codo.
        {"INTERMEDIO_TOMARPLACA",    V(0.447, 0.297, 0.394), V(-0.1, -179.9, 32.4),
                                     Q(328.5, 37.9, 350.2, 50.2, 6.0, 97.7, 353.0)},
        // Articulaciones: 3 cm de la pose.
        {"INTERMEDIO_TOMARTORNILLO", V(0.143, 0.536, 0.477), V(-0.8, 173.6, 74.6),
                                     Q(292.2, 28.3, 338.7, 44.3, 12.0, 102.0, 351.5)},
        // Placa del lado y negativo (2026-10-14). Articulaciones revisadas con
        // el modelo: < 1 cm y < 1 grado de cada pose.
        {"NEW_OFFSET_GENERAL",     V(0.333, -0.385, 0.316), V(179.9, 0.0, 90.0),
                                   Q(40.7, 28.1, 10.4, 71.6, 355.6, 80.1, 320.3)},
        {"ARRIBA_PLACA1_NEW",      V(0.340, -0.447, 0.293), V(180.0, 0.0, 90.0),
                                   Q(45.4, 36.5, 10.5, 61.7, 354.3, 81.8, 324.5)},
        // Foto inicial con mas mesa a la vista (lab, 2026-10-30): la camara va
        // inclinada ~36 grados. Con el modelo: 5 mm y 0.8 grados de sus
        // articulaciones. El agarre sigue con ARRIBA_PLACA1_NEW de referencia.
        {"VISTA_TOTAL",            V(0.071, -0.487, 0.480), V(143.8, 3.9, 86.3),
                                   Q(93.172, 22.138, 10.275, 61.517, 320.182, 97.022, 5.176)},
        {"TOMAR_PLACA1_NEW",       V(0.340, -0.447, 0.182), V(180.0, 0.0, 90.0),
                                   Q(45.4, 41.1, 9.5, 74.1, 353.7, 64.8, 325.2)},
        // Inclinada ~34 grados.
        {"NEW_INTERMEDIO_FIXTURE", V(0.567, -0.254, 0.544), V(176.7, 33.8, 170.2),
                                   Q(17.9, 26.11, 23.51, 43.06, 332.68, 82.29, 23.84)},

        // Placa del lado y positivo (anteriores, ya NO se usan en los pasos).
        {"OFFSET_GENERAL",     V(0.407, 0.465, 0.362), V(180.0, 0.0, 180.0),
                               Q(309.7, 47.4, 356.6, 27.0, 3.55, 105.0, 308.0)},
        {"ARRIBA_PLACA1",      V(0.478, 0.486, 0.236), V(180.0, 0.0, 180.0),
                               Q(313.0, 60.0, 357.0, 23.7, 3.5, 95.5, 311.7)},
        // J3 llego como 377.3: con 357.3 cuadra (1.2 cm, 1.7 grados).
        {"TOMAR_PLACA1",       V(0.478, 0.486, 0.190), V(-180.0, 0.0, 180.0),
                               Q(313.3, 60.0, 357.3, 34.0, 3.8, 85.3, 311.3)},
        // Unica pose inclinada (~51 grados respecto a vertical). Convencion
        // Rz*Ry*Rx verificada con estas articulaciones (0.9 grados).
        {"INTERMEDIO_FIXTURE", V(0.656, 0.283, 0.534), V(-0.4, 133.7, 22.2),
                               Q(336.6, 28.1, 354.2, 47.0, 7.7, 58.2, 353.5)},
        {"ARRIBA_FIXTURE_NEW", V(0.554, 0.001, 0.478), V(179.4, 6.5, 179.5),
                               Q(357.7, 33.0, 359.7, 32.6, 0.6, 106.9, 357.0)},
        // Bajar la placa al fixture antes de soltarla. Posicion y orientacion
        // sacadas del modelo con sus articulaciones (14 cm abajo de
        // ARRIBA_FIXTURE_NEW); confirmar con la app web.
        {"COLOCAR_PLACA",      V(0.555, -0.005, 0.339), V(180.0, 7.2, 179.1),
                               Q(358.5, 29.1, 358.8, 65.3, 1.0, 78.4, 356.5)},
        // Bajar un poco mas la placa antes de soltarla (lab, 2026-10-21).
        // Agarre al centro, como COLOCAR_PLACA. x: el usuario la dio 0.517;
        // sus articulaciones dan 0.558 en el modelo y se confirmo 0.557.
        {"COLOCAR_PLACA_ABAJO", V(0.557, 0.001, 0.322), V(179.4, 6.4, 179.4),
                                Q(358.0, 30.0, 359.6, 67.0, 0.54, 76.0, 357.0)},

        // Tornillos. En ARRIBA_TOMARTORNILLOS se toma la foto (la misma pose
        // que la captura live_124852 de kinova_grasp). Sus articulaciones
        // tambien son la postura de los pasos con vision.
        {"ARRIBA_TOMARTORNILLOS", V(0.063, 0.552, 0.291), V(0.0, -180.0, 0.0),
                                  Q(277.5, 35.0, 353.1, 63.5, 4.9, 81.0, 271.0)},
        {"ARRIBA_TORNILLO1",      V(0.519, 0.046, 0.310), V(0.0, 180.0, 0.0),
                                  Q(352.3, 29.5, 0.55, 69.6, 0.3, 80.0, 352.4)},
        {"SOLTAR_TORNILLO1",      V(0.519, 0.046, 0.256), V(180.0, 0.2, 180.0),
                                  Q(352.4, 31.4, 0.3, 76.5, 0.4, 71.3, 352.3)},
        // Foto de la placa en el fixture para buscar los hoyos (lab,
        // 2026-10-21). OJO: con el modelo de la camara, desde aqui la camara
        // ve ~1/3 de la placa (la foto del 2026-10-20 desde ARRIBA_TORNILLO1
        // la veia toda); el usuario decidio usarla asi.
        {"TOMAR_FOTO_PLACA",      V(0.524, -0.096, 0.395), V(0.0, -180.0, 0.0),
                                  Q(7.6, 31.0, 1.0, 51.0, 0.0, 97.0, 8.1)},
    };

    // Caja de trabajo (TCP, base) con margen alrededor de las poses de arriba.
    // z minimo 0.06: los tornillos (vision) pueden estar mas abajo que la placa.
    // y minimo -0.55: la placa ahora esta en y = -0.45 (2026-10-14).
    // y minimo -0.65 (2026-11-03): el centrado desde VISTA_TOTAL pidio
    // y = -0.585 y se rechazaba por la caja.
    t.workspace_min_m = V(0.00, -0.65, 0.06);
    t.workspace_max_m = V(0.75, 0.65, 0.65);

    // Zona baja (task::TaskParams low_zone_*): apagada. Se uso el 2026-10-27
    // con el robot movido (mesa de las placas 18.5 cm mas baja, z -0.20):
    // t.low_zone_enabled = true; min (0, -0.70, -0.18), max (0.75, -0.30,
    // 0.65), vision_max_xy_m 0.30 y approach_max_move_m 0.40. Ese mismo dia
    // se regreso el robot a su lugar.

    // Centrado desde VISTA_TOTAL (2026-10-30): placa en su soporte 25-27 cm,
    // plana sobre la mesa unos 40 cm. Sigue el limite de approach_max_xy_m
    // alrededor de ARRIBA_PLACA1_NEW y la caja de trabajo.
    // 0.61 (2026-11-03): rango de trabajo calculado a mano por el equipo.
    t.approach_max_move_m = 0.61;
    // Agarre (y tornillo) a lo mas esto en horizontal de la pose de la foto
    // (ARRIBA_PLACA1_NEW / ARRIBA_TOMARTORNILLOS) [m]. 0.35 (2026-11-03;
    // antes 0.25 y 0.20, el valor por defecto de TaskParams).
    t.vision_max_xy_m = 0.35;

    // HOME_MOD -> ARRIBA_TOMARTORNILLOS mide 0.725 m (el camino recto pasa a
    // 0.41 m del eje de la base). 0.80 deja margen para el error de llegada
    // (la distancia se revisa tambien desde la posicion MEDIDA).
    t.max_move_distance_m = 0.80;

    // Pinza (0 = abierta, 100 = cerrada). Velocidad y fuerza probadas con
    // gripper_test en el lab (2026-10-13): agarra la placa firmemente.
    constexpr double GRIPPER_SLOW  = 30.0;   // [%] "lentamente"
    constexpr double GRIPPER_FORCE = 20.0;   // [%]
    constexpr double GRIP_PLATE    = 94.5;   // [%] cerrar sobre la placa
    constexpr double RELEASE_PLATE = 45.0;   // [%] abrir para soltarla
    constexpr double RELEASE_COMPLETE = 10.0;
    constexpr double GRIP_SCREW    = 64.0;   // [%] cerrar sobre el tornillo
    constexpr double RELEASE_SCREW = 30.0;   // [%] abrir para soltarlo

    // Tornillo: el screw_server de kinova_grasp_V2 da la pose de agarre
    // (ya compensa el avance de las puntas de la pinza al cerrar) y la
    // pre-grasp 10 cm atras sobre la aproximacion. Ya no hay offset en z
    // (antes SCREW_ABOVE = 0.06, SCREW_GRASP = 0.02 con el servidor viejo).
    // Despues de cerrar sube solo en z de la base [m].
    constexpr double SCREW_LIFT = 0.07;
    // Agarrar el tornillo esta altura ARRIBA del agarre de GPD [m]: bajaba
    // de mas y no lo tomaba bien (2026-11-03).
    constexpr double SCREW_GRASP_UP = 0.003;   // 2026-11-03: 5 mm -> 3 mm
    // Antes de bajar al hoyo: esta altura encima de ARRIBA_TORNILLO1 [m].
    constexpr double HOLE_ABOVE = 0.05;
    // Soltar el tornillo esta altura arriba de SOLTAR_TORNILLO1 [m], para que
    // el robot no pegue con la placa (2026-11-03).
    constexpr double SCREW_RELEASE_UP = 0.01;

    // Fixture [grados]: 0 = colgado (donde se hace ZERO al arrancar).
    constexpr double FIXTURE_WAIT    = -90.0;   // posicion de espera
    constexpr double FIXTURE_RECEIVE = -23.0;   // recibe la placa

    // ---- Placa con poses fijas (TOMAR_PLACA1_NEW) ----
    const std::vector<task::Step> plate_fixed = {
        task::fixtureTo("Fixture a posicion de espera",  FIXTURE_WAIT),
        task::moveTo   ("Ir a HOME_MOD",                 "HOME_MOD"),
        task::gripper  ("Abrir pinza",                   0.0,  GRIPPER_SLOW, GRIPPER_FORCE),
        task::moveTo   ("Ir a NEW_OFFSET_GENERAL",       "NEW_OFFSET_GENERAL"),
        task::moveTo   ("Encima de la placa",            "ARRIBA_PLACA1_NEW"),
        task::moveTo   ("Bajar a la placa",              "TOMAR_PLACA1_NEW"),
        task::gripper  ("Cerrar pinza (agarrar placa)",  GRIP_PLATE, GRIPPER_SLOW, GRIPPER_FORCE),
        task::waitFor  ("Asentar el agarre",             0.5),
        task::moveTo   ("Levantar la placa",             "ARRIBA_PLACA1_NEW"),
        task::moveTo   ("Regresar a NEW_OFFSET_GENERAL", "NEW_OFFSET_GENERAL"),
        task::moveTo   ("Ir a NEW_INTERMEDIO_FIXTURE",   "NEW_INTERMEDIO_FIXTURE"),
        task::fixtureTo("Girar el fixture para recibir", FIXTURE_RECEIVE),
        task::moveTo   ("Encima del fixture",            "ARRIBA_FIXTURE_NEW"),
        task::moveTo   ("Bajar a colocar la placa",      "COLOCAR_PLACA"),
        task::gripper  ("Abrir pinza (soltar placa)",    RELEASE_PLATE, GRIPPER_SLOW, GRIPPER_FORCE),
        // Subir recto antes de ir a HOME_MOD, para no arrastrar la placa.
        task::moveTo   ("Subir del fixture",             "ARRIBA_FIXTURE_NEW"),
        task::moveTo   ("Regresar a HOME_MOD",           "HOME_MOD"),
        task::fixtureTo("Fixture a posicion de espera",  FIXTURE_WAIT),

    };

    // ---- Placa por vision, al fixture corregida ----
    // La foto se toma en ARRIBA_PLACA1_NEW (sus articulaciones son la
    // postura de los pasos con vision); el servidor mide tambien el canto
    // de la placa. Subir/bajar: z de la base desde el agarre, sin girar la
    // muneca. En el fixture, ARRIBA_FIXTURE_NEW y COLOCAR_PLACA se
    // CORRIGEN por donde se agarro la placa (moveToPlateCorrected): la
    // placa queda donde quedaria agarrada al centro del canto.
    // Con la placa agarrada sube esto en z de la base desde el agarre [m],
    // sin moverse en x y ni girar; de ahi va directo a NEW_OFFSET_GENERAL.
    // 2026-11-03: se probo subir a z FIJA 0.364 (task::moveToVisionGraspAtZ)
    // y el robot se detuvo con falla: lejos de la base el brazo quedo casi
    // estirado y al ir a NEW_OFFSET_GENERAL J4 cruzo 0 grados (singularidad
    // del codo, J7 a 49 grados/s).
    constexpr double PLATE_LIFT = 0.08;
    // Agarre de referencia (con el que se grabaron ARRIBA_FIXTURE_NEW y
    // COLOCAR_PLACA): al centro del canto y la TCP esta altura ARRIBA del
    // canto [m]. 0.012 = TOMAR_PLACA1_NEW (z 0.182) - canto medido el
    // 2026-10-19 (z ~0.170). VERIFICAR en la primera prueba: si la placa
    // entra muy abajo/arriba en el fixture, ajustar este valor.
    t.plate_nominal_tcp_above_edge_m = 0.012;
    // Tornillo al hoyo: se suma al centro del hoyo de la vision (x, y de la
    // base) [m]. 0 = TCP sobre el centro. Si el tornillo cae siempre corrido,
    // poner aqui al reves lo que se corre (maximo 5 cm).
    t.hole_offset_xy_m = Eigen::Vector2d(0.0, 0.0);
    const std::vector<task::Step> plate_vision = {
        task::fixtureTo("Fixture a posicion de espera",  FIXTURE_WAIT),
        task::moveTo   ("Ir a HOME_MOD",                 "HOME_MOD"),
        task::gripper  ("Abrir pinza",                   0.0,  GRIPPER_SLOW, GRIPPER_FORCE),
        task::moveTo   ("Ir a NEW_OFFSET_GENERAL",       "NEW_OFFSET_GENERAL"),
        task::moveTo   ("Encima de la placa (foto)",     "ARRIBA_PLACA1_NEW"),
        task::moveTo   ("Ir a VISTA_TOTAL",              "VISTA_TOTAL"),
        // approach_object de Antony (2026-10-27): la camara de color 30 cm
        // arriba de la placa (por su color) hasta IN VIEW; luego la foto es
        // grasps_from_camera gpd_plate_params.cfg --color --filter.
        task::visionApproach("Vision: centrar la camara sobre la placa", "ARRIBA_PLACA1_NEW", "PLATE"),
        task::visionCapturePlate("Vision: buscar placa", "ARRIBA_PLACA1_NEW"),
        task::moveToVisionPregrasp("Ir a la pre-grasp",  "ARRIBA_PLACA1_NEW"),
        task::moveToVisionGrasp   ("Bajar a la placa",   "ARRIBA_PLACA1_NEW"),
        task::gripper  ("Cerrar pinza (agarrar placa)",  GRIP_PLATE, GRIPPER_SLOW, GRIPPER_FORCE),
        task::waitFor  ("Asentar el agarre",             0.5),
        task::moveToVisionGrasp   ("Subir 8 cm con la placa", "ARRIBA_PLACA1_NEW", PLATE_LIFT),
        // Ya no pasa por ARRIBA_PLACA1_NEW (2026-11-03):
        //        task::moveTo("Regresar encima de la placa", "ARRIBA_PLACA1_NEW"),
        task::moveTo   ("Regresar a NEW_OFFSET_GENERAL", "NEW_OFFSET_GENERAL"),
        task::moveTo   ("Ir a NEW_INTERMEDIO_FIXTURE",   "NEW_INTERMEDIO_FIXTURE"),
        task::fixtureTo("Girar el fixture para recibir", FIXTURE_RECEIVE),
        task::moveToPlateCorrected("Encima del fixture (corregida)",       "ARRIBA_FIXTURE_NEW"),
        task::moveToPlateCorrected("Bajar a colocar la placa (corregida)", "COLOCAR_PLACA"),
        task::moveToPlateCorrected("Bajar mas la placa (corregida)",       "COLOCAR_PLACA_ABAJO"),
        task::gripper  ("Abrir pinza (soltar placa)",    RELEASE_PLATE, GRIPPER_SLOW, GRIPPER_FORCE),
        // Sube recto desde donde la solto, para no arrastrar la placa.
        //task::moveToPlateCorrected("Subir del fixture (corregida)",        "ARRIBA_FIXTURE_NEW"),
        task::moveTo   ("Regresar a HOME_MOD",           "HOME_MOD"),
        task::fixtureTo("Fixture a posicion de espera",  FIXTURE_WAIT),
    };

    // ---- Tornillo por vision y nube del hoyo ----
    const std::vector<task::Step> screw = {
        task::moveTo       ("Ir a INTERMEDIO_TOMARPLACA",    "INTERMEDIO_TOMARPLACA"),
        //task::moveTo       ("Ir a INTERMEDIO_TOMARTORNILLO", "INTERMEDIO_TOMARTORNILLO"),
        task::moveTo       ("Encima de los tornillos",       "ARRIBA_TOMARTORNILLOS"),
        // Ya esta en 45 % si viene de soltar la placa; necesario si se
        // arranca aqui.
        task::gripper      ("Abrir pinza (45 %)",            RELEASE_PLATE, GRIPPER_SLOW, GRIPPER_FORCE),
        // approach_object del perno (rojo) y luego grasps_from_camera
        // gpd_params.cfg --color --filter (2026-10-27).
        task::visionApproach("Vision: centrar la camara sobre el perno", "ARRIBA_TOMARTORNILLOS", "BOLT"),
        task::visionCapture("Vision: buscar tornillo",       "ARRIBA_TOMARTORNILLOS"),
        task::gripper      ("Abrir pinza (90 %)",            RELEASE_COMPLETE, GRIPPER_SLOW, GRIPPER_FORCE),
        task::moveToVisionPregrasp("Ir a la pre-grasp",      "ARRIBA_TOMARTORNILLOS"),
        task::moveToVisionGrasp   ("Bajar al tornillo",      "ARRIBA_TOMARTORNILLOS", SCREW_GRASP_UP),
        task::gripper      ("Cerrar pinza (tornillo)",       GRIP_SCREW, GRIPPER_SLOW, GRIPPER_FORCE),
        // Sube recto en z con el tornillo, sin girar la muneca.
        // + SCREW_GRASP_UP: 7 cm desde donde cerro (no desde el agarre de GPD).
        task::moveToVisionGrasp   ("Subir 7 cm",             "ARRIBA_TOMARTORNILLOS", SCREW_LIFT + SCREW_GRASP_UP),
        task::moveTo       ("Regresar encima de tornillos",  "ARRIBA_TOMARTORNILLOS"),
        task::moveTo       ("Ir a posicion intermedia",      "INTERMEDIO_TOMARPLACA"),
        task::moveTo       ("Ir a HOME_MOD",                 "HOME_MOD"),
        task::moveTo       ("Ir a tomar foto de la placa",   "TOMAR_FOTO_PLACA"),
        // Foto de la placa en TOMAR_FOTO_PLACA (2026-10-21): solo nube
        // (hoyo_HHMMSS.pcd), sin GPD; el servidor da el centro del hoyo MAS
        // CHICO de la placa. Regresa a HOME_MOD y luego la TCP
        // va sobre ese centro con la altura y orientacion de ARRIBA_TORNILLO1
        // y baja a la altura de SOLTAR_TORNILLO1. Calibracion si cae siempre
        // corrido: t.hole_offset_xy_m (abajo).
        // Sin vision (antes): task::moveTo("Bajar a soltar el tornillo", "SOLTAR_TORNILLO1")
        task::visionCaptureHole("Vision: buscar el hoyo mas chico (sin GPD)", "TOMAR_FOTO_PLACA"),
        task::moveTo       ("Regresar a HOME_MOD",           "HOME_MOD"),
        // Primero 10 cm arriba de ARRIBA_TORNILLO1 sobre el hoyo (de HOME_MOD
        // a la placa en diagonal chocaba), luego baja solo en z.
        task::moveToHole   ("10 cm encima del hoyo (vision)", "ARRIBA_TORNILLO1", HOLE_ABOVE),
        task::moveToHole   ("Encima del hoyo (vision)",      "ARRIBA_TORNILLO1"),
        task::moveToHole   ("Bajar a soltar el tornillo (vision)", "SOLTAR_TORNILLO1", SCREW_RELEASE_UP),
        task::gripper      ("Abrir pinza (soltar tornillo)", RELEASE_SCREW, GRIPPER_SLOW, GRIPPER_FORCE),
        task::moveTo       ("Regresar a HOME_MOD",           "HOME_MOD"),
    };

    auto join = [](std::vector<task::Step> a, const std::vector<task::Step>& b)
    {
        a.insert(a.end(), b.begin(), b.end());
        return a;
    };
    switch (mode)
    {
        case Secuencia::Normal:
            // Paso con el que arranca (1 = todo; 19 = solo el tornillo).
            t.steps = join(plate_fixed, screw);
            t.first_step = 20;
            break;
        case Secuencia::SoloPlacaVision:
            t.steps = plate_vision;
            t.first_step = 1;
            break;
        case Secuencia::Completa:
            t.steps = join(plate_vision, screw);
            t.first_step = 1;
            break;
    }

    return t;
}

enum class CartesianReferenceMode
{
    InitialPose,
    RelativeOffset,
    AbsolutePose
};


// Fraccion de la compensacion de friccion segun la distancia al objetivo:
// 0 con dist <= within_m, 1 con dist >= within_m + fade_m, lineal entre los
// dos (fade_m = 0: escalon en within_m). dist no finita = 1 (como antes).
inline double frictionScaleNearTarget(double dist_m, double within_m, double fade_m)
{
    if (!std::isfinite(dist_m))
        return 1.0;
    if (fade_m <= 0.0)
        return dist_m <= within_m ? 0.0 : 1.0;
    return std::clamp((dist_m - within_m) / fade_m, 0.0, 1.0);
}

// Fraccion de la accion de control (impedancia + espacio nulo + friccion)
// en el objetivo: 0 solo si la posicion esta a <= within_m Y la orientacion a
// <= within_deg; sube en rampa con la que este mas lejos (1 con dist >=
// within_m + fade_m o angulo >= within_deg + fade_deg). No finito = 1.
inline double controlScaleNearTarget(double dist_m, double ang_deg, double within_m, double fade_m,
                                     double within_deg, double fade_deg)
{
    return std::max(frictionScaleNearTarget(dist_m, within_m, fade_m),
                    frictionScaleNearTarget(ang_deg, within_deg, fade_deg));
}

struct ControllerConfig
{
    using Controller = impedance::CartesianImpedanceController;
    using Vec3 = Eigen::Vector3d;
    using Vec6 = Controller::Vec6;
    using Vec7 = Controller::Vec7;


    double update_hz = 1000.0;

    // Terminal dashboard refresh period.
    double dashboard_period_s = 0.20;

    double experiment_duration_s = 120.0;



    // CARTESIAN REFERENCE
    //
    // InitialPose holds the startup pose, RelativeOffset applies the
    // offsets below, and AbsolutePose uses a pose in the base frame.
    //
    // Position:
    //   [dx, dy, dz]
    //   expressed in ROBOT BASE frame.
    //   Units: meters.
    //
    // Orientation:
    //   [roll, pitch, yaw]
    //   relative to the INITIAL end-effector orientation.
    //   Units: degrees.
    //

    CartesianReferenceMode cartesian_reference_mode =
        CartesianReferenceMode::AbsolutePose;


    // Position offset [m]
    Eigen::Vector3d position_offset =
        (Eigen::Vector3d()
            << 0.05,   // +5 cm in base X
            0.1,   // Y
            0.00)   // Z
        .finished();


    // Orientation offset [deg]
    Eigen::Vector3d orientation_offset_rpy_deg =
        (Eigen::Vector3d()
            << 0.0,    // Roll
            0.0,    // Pitch
            0.0)    // Yaw
        .finished();


    // Absolute position in robot base frame [m]
    Eigen::Vector3d absolute_position =
        (Eigen::Vector3d()
            << 0.0603, 0.552, 0.291)
        .finished();


    // Absolute orientation in robot base frame [deg].
    // Convention: Rz(yaw) * Ry(pitch) * Rx(roll).
    Eigen::Vector3d absolute_orientation_rpy_deg =
        (Eigen::Vector3d()
            << 0, -180, 0)
        .finished();


    // CARTESIAN STIFFNESS
    Vec7 Kfric = {1.5, 3.5,1,2,1,1,1};



    Vec6 stiffness =
        (Vec6()
            << 20, 18, 20,
               10,  10, 10)
        .finished();


    // RIGIDEZ CERCA DEL OBJETIVO (2026-10-21)
    //
    // Con 40 N/m el robot se quedaba atorado a 4-6.6 cm del objetivo y habia
    // que empujarlo (CSV 2026-10-21 17-42-15, pasos 12, 13, 17 y 31, cargando
    // la placa o el tornillo): 5 cm x 40 N/m = 2 N, menos que la friccion
    // estatica. Cerca del objetivo la rigidez de TRASLACION sube a
    // near_target_stiffness; lejos se queda la de arriba (movimientos largos
    // suaves). Distancia = |posicion - objetivo de la referencia|.
    // Sube lineal entre near_target_start_m y near_target_full_m (sin
    // escalon), por eso cambia sin stiffness_filter (ese tarda ~10 s).
    // La rotacion no cambia. false = siempre la rigidez de arriba.
    bool enable_near_target_stiffness = true;
    double near_target_stiffness = 142.0;   // N/m (x, y, z)
    double near_target_start_m = 0.085;     // a esta distancia empieza a subir
    double near_target_full_m = 0.065;      // de aqui para adentro: 80


    // CARTESIAN DAMPING FACTORS
    
    //

    Vec6 damping_factors =
        Vec6::Ones();


    // NULLSPACE (postura del brazo)
    //
    // El Gen3 tiene 7 articulaciones para 6 GDL de la tarea: le sobra uno, el
    // giro del codo alrededor de la linea hombro-muneca. Sin espacio nulo
    // nada sostiene ese giro y el codo se va de lado (en el modelo ese
    // movimiento es casi solo J3 y J5 en sentidos opuestos).
    //
    // tau_null = N * (K * w .* (q_ref - q) - D * dq), N = proyector al espacio
    // nulo: estatico, no empuja la herramienta.
    //
    // q_ref arranca en la q inicial y se filtra hacia la postura
    // (nullspace_filter), asi que no hay salto al prender.
    bool enable_nullspace = true;

    double nullspace_stiffness = 5.0;          // Nm/rad
    double nullspace_damping_factor = 1.0;     // D = factor * 2 sqrt(K)

    // true: postura = q al arrancar el programa (comportamiento anterior).
    // false: postura = nullspace_posture_deg (no depende de donde arranque).
    bool nullspace_use_initial_q = false;

    // Postura deseada [deg]. q medida en HOME_MOD (CSV 2026-10-14 12-54-33).
    Vec7 nullspace_posture_deg =
        (Vec7() << -5.0, 31.0, 2.0, 51.0, -1.0, 97.0, -3.0).finished();

    // Peso por articulacion del resorte de postura. Solo J3 y J5: son las del
    // giro del codo; J1, J2, J4, J6, J7 las fija la tarea y jalarlas hacia
    // HOME torceria el codo en poses lejos de HOME.
    Vec7 nullspace_weights =
        (Vec7() << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0).finished();

    // Tope de |tau_null| por articulacion [Nm]; escala todo el vector.
    double nullspace_max_torque = 8.0;

    // Se suma a la postura [rad].
    Vec7 nullspace_offset =
        Vec7::Zero();


    // LIMITE SUAVE DE J4 (codo doblado)
    //
    // Con J4 cerca de 0 el brazo queda estirado (singularidad) y la tarea ya
    // no lo saca (paso 11 del CSV 2026-10-14 15-32-11: J4 = -2, J6 = 122).
    // El espacio nulo no puede evitarlo: con la herramienta quieta J4 no se
    // mueve. Este resorte actua DIRECTO en J4 y solo si J4 < j4_min_deg;
    // solo empuja hacia doblar el codo. Todas las poses del lab tienen J4
    // entre 24 y 77 grados: en la secuencia normal no se activa.
    // El programa no arranca si J4 ya esta debajo del limite.
    // Cerca del limite (j4_min_deg + j4_friction_fade_deg) la compensacion de
    // friccion deja de empujar a estirar J4 (rampa por posicion), asi la
    // friccion real lo detiene. NO meter tau_limit en la direccion de la
    // friccion: vibra (CSV 2026-10-14 15-56-07).
    bool enable_j4_limit = false;

    double j4_min_deg = 15.0;
    double j4_limit_stiffness = 20.0;     // Nm/rad (5 grados -> 1.7 Nm)
    double j4_limit_damping = 9.0;        // Nms/rad (~2 sqrt(K))
    double j4_limit_max_torque = 8.0;     // Nm
    double j4_friction_fade_deg = 5.0;    // 20 -> 15 grados: friccion a estirar 100% -> 0%


    // DESIRED CARTESIAN WRENCH
    //
    // [Fx, Fy, Fz, Mx, My, Mz]
    //
    // Force:  N
    // Moment: Nm
    //
    // Example:
    //
    // wrench =
    //     (Vec6() << 0, 0, -5, 0, 0, 0).finished();
    //
    // ============================================================

    bool enable_wrench = false;

    Vec6 wrench =
        Vec6::Zero();




    // FILTERS
    

    double pose_filter = 1; //0.10;

    // RAMPA DE LA REFERENCIA (antes de pose_filter)
    //
    // La referencia va en linea recta hacia cada pose nueva a velocidad
    // constante y se detiene justo en ella (no se arrastra al final como
    // pose_filter). Tiempo de un movimiento = max(distancia / lineal,
    // giro / angular); p. ej. 72 cm a 0.05 m/s = 14.4 s.
    // 0 en cualquiera = sin rampa (la referencia salta, como antes).
    // Con pose_filter < 1 los dos se suman: usar pose_filter = 1.
    // OJO: igual que pose_filter, mantiene el error chico todo el movimiento
    // y la compensacion de friccion puede temblar (friction_sign_factor).
    double reference_max_speed_m_s = 0; // 0.05; // con cero se desactiva
    double reference_max_angular_speed_deg_s =0 ;// 15.0;
    double stiffness_filter = 0.10;
    double wrench_filter = 0.10;
    double nullspace_filter = 0.10;


    // IMPEDANCE TORQUE SLEW-RATE LIMIT

    double delta_tau_max = 1.0;


    // GRAVITY TRANSITIONS

    int gravity_warmup_cycles = 200;
    int gravity_shutdown_cycles = 200;


    //

    double impedance_blend_time_s = 1.0;


    // FRICTION COMPENSATION (LuGre observer from main_op_space.cpp)
    //
    // Only used by main_impedance.cpp. Acts only with the slider ON and
    // enters with the same blend as the impedance torque.
    // Direction is taken from the Cartesian spring (tau_spring), not from
    // the damping term.

    bool enable_friction_compensation = true;

    // Smooth sign: tanh(factor * tau_spring^3). 1e5 = original value
    // (almost a switch). Lower (100-1000) smooths it if it chatters.
    double friction_sign_factor = 10000.0;

    // Fixed RK4 step [s], as in the original.
    double friction_dt = 0.001;

    // SIN COMPENSACION EN EL OBJETIVO (2026-10-22)
    //
    // La compensacion sirve para MOVERSE; ya en el objetivo la friccion real
    // es la que lo deja quieto. En el paso 28 (bajar al tornillo) del CSV
    // 2026-10-22 09-39-00 volteaba +-6.3 Nm en J1 y J3 42 veces por segundo
    // cada que el robot cruzaba el objetivo (el resorte pedia 0.05 Nm) y
    // vibraba +-1 mm a ~22 Hz; el paso nunca terminaba (no se quedaba
    // quieto) y la pinza no cerraba.
    // Con el error de POSICION al objetivo <= friction_off_within_m la
    // compensacion es 0 (todas las articulaciones). Arriba regresa en rampa
    // y es 100 % desde friction_off_within_m + friction_off_fade_m. El
    // observador sigue corriendo; solo se escala su salida.
    // OJO: solo mira la posicion; con la posicion bien y la orientacion
    // chueca tampoco compensa.
    bool enable_friction_off_near_target = true;
    double friction_off_within_m = 0.003;   // <= 3 mm: 0
    double friction_off_fade_m = 0.002;     // 100 % desde 5 mm

    // SOLO GRAVEDAD EN EL OBJETIVO (2026-10-27, pedido del usuario)
    //
    // Ya en el objetivo el robot no hace nada: el torque es solo la gravedad
    // y lo deja quieto la friccion real de las articulaciones. Se escala
    // TODA la accion de control (resorte, amortiguamiento, espacio nulo y
    // compensacion de friccion) por controlScaleNearTarget: 0 con la
    // posicion a <= 5 mm Y la orientacion a <= 2 grados del objetivo; 100 %
    // desde 7 mm o 3 grados (rampa, sin escalon: un corte en seco haria
    // relevo on/off como la friccion del 2026-10-22). Si la carga lo baja,
    // el resorte regresa poco a poco y puede quedarse quieto entre 5 y 7 mm.
    // El objetivo es el del paso actual (tambien en pinza, espera y vision).
    bool enable_gravity_only_near_target = true;
    double gravity_only_within_m = 0.005;      // <= 5 mm (y orientacion): solo gravedad
    double gravity_only_fade_m = 0.002;        // control completo desde 7 mm
    double gravity_only_within_deg = 2.0;      // y <= 2 grados
    double gravity_only_fade_deg = 1.0;        // control completo desde 3 grados


    // VISION (GPD) - SOLO IMPRIMIR
    //
    // Recibe agarres por UDP, los valida, los pasa a la base y los muestra
    // en el dashboard y el CSV. NUNCA mueve el robot (no hay
    // setReferencePose desde la vision). Todos los parametros y sus
    // supuestos por verificar estan en include/grasp_pipeline.hpp.
    // Ejemplos: vision.enabled = false; vision.udp_port = 5005;
    //
    // OJO: esto es independiente de la vision de TORNILLOS de la secuencia
    // (pasos visionCapture / moveToVisionPregrasp / moveToVisionGrasp en
    // phase2Task, screw_server de kinova_grasp_V2 en el puerto
    // task.vision_server_port). Esa SI mueve el robot (posicion y orientacion
    // del agarre), solo dentro de los limites de TaskParams (vision_max_xy_m,
    // vision_min_below_m, vision_max_tilt_deg, caja de trabajo).

    grasp::VisionParams vision{};


    // SECUENCIA DE TAREA (Fase 2)
    //
    // enabled = false -> el programa hace exactamente lo de antes.
    // enabled = true  -> la referencia inicial es la pose ACTUAL del robot
    //                    (no se mueve al arrancar) y la duracion es
    //                    task.duration_s.
    //                    Los pasos avanzan con el boton "Siguiente paso".
    // Poses y pasos: phase2Task() arriba. Tolerancias, tiempos y caja de
    // trabajo: include/task_sequence.hpp (TaskParams).
    // Puerto del fixture: task.fixture.port (por defecto /dev/ttyUSB0).

    task::TaskParams task = [] {
        task::TaskParams t = phase2Task();
        t.enabled = true;  // false = solo impedancia (demo). true = secuencia con poses del lab (2026-10-13)
        // AUTOMATICO por defecto (2026-10-21): solo el primer paso y las
        // pausas esperan el boton. Se cambia a PASO A PASO desde la GUI.
        t.confirm_each_step = false;
        return t;
    }();


    // SHADOW DIAGNOSTIC THRESHOLDS
    

    double max_shadow_position_error_m = 0.25;
    double max_shadow_rotation_error_rad = 1.0;

    double max_shadow_linear_speed_m_s = 1.0;
    double max_shadow_angular_speed_rad_s = 3.0;

    double max_shadow_impedance_torque_nm = 30.0;
};

} // namespace project_config
