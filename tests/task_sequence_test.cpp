// Prueba de la secuencia de tarea (include/task_sequence.hpp) sin robot.
// Simula un robot que sigue la referencia con retraso, una pinza y un fixture.

#include "controller_config.hpp"
#include "screw_vision_client.hpp"
#include "task_sequence.hpp"

#include <cmath>
#include <map>
#include <iostream>
#include <string>
#include <vector>

namespace
{

int failures = 0;

void check(bool ok, const std::string& what)
{
    std::cout << (ok ? "  OK   " : "  FAIL ") << what << "\n";
    if (!ok)
        ++failures;
}

bool contains(const std::string& s, const std::string& part)
{
    return s.find(part) != std::string::npos;
}

// Tarea de prueba GENERICA (independiente de las poses del lab): 14 pasos,
// herramienta hacia abajo, fixture a -90 y de regreso a 0.
task::TaskParams definedTask()
{
    task::TaskParams t;
    const Eigen::Vector3d down(-179.8, -1.0, 179.9);
    t.poses = {
        {"inicio",         Eigen::Vector3d(0.50, 0.00, 0.40), down},
        {"placa_arriba",   Eigen::Vector3d(0.60, -0.20, 0.30), down},
        {"placa",          Eigen::Vector3d(0.60, -0.20, 0.20), down},
        {"fixture_arriba", Eigen::Vector3d(0.60, 0.20, 0.30), down},
        {"fixture",        Eigen::Vector3d(0.60, 0.20, 0.22), down},
    };
    t.steps = {
        task::moveTo   ("Ir a inicio",                  "inicio"),
        task::gripper  ("Abrir pinza",                  0.0,  30.0, 20.0),
        task::moveTo   ("Encima de la placa",           "placa_arriba"),
        task::moveTo   ("Bajar a la placa",             "placa"),
        task::gripper  ("Cerrar pinza (agarrar placa)", 95.0, 30.0, 20.0),
        task::waitFor  ("Asentar el agarre",            0.5),
        task::moveTo   ("Levantar la placa",            "placa_arriba"),
        task::moveTo   ("Acercarse al fixture",         "fixture_arriba"),
        task::fixtureTo("Girar el fixture",             -90.0),
        task::moveTo   ("Bajar al fixture",             "fixture"),
        task::gripper  ("Abrir pinza (soltar placa)",   0.0,  30.0, 20.0),
        task::moveTo   ("Retirarse del fixture",        "fixture_arriba"),
        task::fixtureTo("Regresar el fixture",          0.0),
        task::moveTo   ("Regresar a inicio",            "inicio"),
    };
    t.enabled = true;
    return t;
}

// Simulacion del mundo.
struct World
{
    Eigen::Vector3d p_ref = Eigen::Vector3d(0.50, 0.00, 0.52);
    Eigen::Quaterniond q_ref = task::rpyToQuaternion(Eigen::Vector3d(-179.8, -1.0, 179.9));
    Eigen::Vector3d p = p_ref;
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = q_ref;

    double gripper = 50.0, gripper_cmd = 50.0, gripper_speed = 0.0;
    double plate_block_pct = -1.0;   // >= 0: la placa detiene el cierre ahi

    double fixture_target = 0.0, fixture_since = 1e9;
    bool fixture_fault = false;

    // Servidor de vision simulado: contesta 3 s despues de cada peticion.
    // Las primeras vision_fails respuestas son FAIL.
    // Como el servidor V2: agarre casi vertical girado en yaw, pre-grasp
    // 10 cm atras sobre la aproximacion (z de la pinza).
    Eigen::Vector3d screw_tcp = Eigen::Vector3d(0.08, 0.57, 0.17);
    Eigen::Vector3d screw_rpy = Eigen::Vector3d(-1.0, -179.3, 34.0);
    bool screw_has_pose = true;
    // Canto de la placa (servidor con EDGE). Por defecto no hay.
    bool has_edge = false;
    Eigen::Vector3d edge_a = Eigen::Vector3d::Zero(), edge_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d pregrasp() const
    {
        return screw_tcp - 0.10 * (task::rpyToQuaternion(screw_rpy) * Eigen::Vector3d::UnitZ());
    }
    int vision_fails = 0;
    uint32_t vision_pending = 0;
    double vision_reply_at = 1e9;
    uint32_t vision_seq = 0;
    bool vision_ok = false;
    std::string vision_text;
    // Peticion CLOUD (solo nube): contesta CLOUD. reply_cloud_as_screw: un
    // servidor que contesta SCREW a todo.
    bool vision_pending_cloud = false;
    bool vision_is_cloud = false;
    bool reply_cloud_as_screw = false;
    // Hoyo mas chico que manda el servidor con la nube (por defecto justo
    // debajo de SOLTAR_TORNILLO1, en la cara de la placa).
    bool has_hole = true;
    Eigen::Vector3d hole = Eigen::Vector3d(0.519, 0.046, 0.208);
    double hole_diam = 0.026;
    // APPROACH (centrar la camara): las primeras approach_moves respuestas de
    // cada objeto son MOVE a approach_tcp, luego INVIEW en el mismo lugar.
    // approach_drift: la pose se corre esto en cada respuesta (nunca centra).
    bool vision_pending_approach = false, vision_is_approach = false;
    std::string vision_pending_object;
    int approach_moves = 1, approach_state = 0;
    std::map<std::string, int> approach_count;
    Eigen::Vector3d plate_approach_tcp = Eigen::Vector3d(0.36, -0.43, 0.32);
    Eigen::Vector3d plate_approach_rpy = Eigen::Vector3d(180.0, 0.0, 90.0);
    Eigen::Vector3d bolt_approach_tcp = Eigen::Vector3d(0.07, 0.56, 0.28);
    Eigen::Vector3d bolt_approach_rpy = Eigen::Vector3d(0.0, -180.0, 0.0);
    Eigen::Vector3d approach_tcp = Eigen::Vector3d::Zero(), approach_rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d approach_drift = Eigen::Vector3d::Zero();

    std::vector<std::string> log;   // comandos en orden

    void apply(const task::Commands& c, double t)
    {
        if (c.set_reference)
        {
            p_ref = c.position;
            q_ref = c.orientation;
            log.push_back("ref");
        }
        if (c.set_gripper)
        {
            gripper_cmd = c.gripper_pct;
            gripper_speed = c.gripper_speed_pct;
            log.push_back("gripper " + std::to_string(int(c.gripper_pct)));
        }
        if (c.move_fixture)
        {
            fixture_target = c.fixture_deg;
            fixture_since = t;
            log.push_back("fixture " + std::to_string(int(c.fixture_deg)));
        }
        if (c.request_vision)
        {
            vision_pending = c.vision_seq;
            vision_pending_cloud = c.vision_cloud_only;
            vision_pending_approach = c.vision_approach;
            vision_pending_object = c.vision_object;
            vision_reply_at = t + 3.0;
            log.push_back(c.vision_approach ? "centrar" : c.vision_cloud_only ? "nube" : "vision");
        }
    }

    void visionStep(double t)
    {
        if (vision_pending != 0 && t >= vision_reply_at)
        {
            vision_seq = vision_pending;
            vision_pending = 0;
            vision_ok = vision_fails <= 0;
            vision_is_cloud = vision_pending_cloud && !reply_cloud_as_screw;
            vision_is_approach = vision_pending_approach;
            vision_text = !vision_ok ? "GPD no encontro agarres" : vision_is_cloud ? "hoyo_120000.pcd, 9000 points" : "";
            if (vision_is_approach && vision_ok)
            {
                const bool plate = vision_pending_object == "PLATE";
                approach_tcp = plate ? plate_approach_tcp : bolt_approach_tcp;
                approach_rpy = plate ? plate_approach_rpy : bolt_approach_rpy;
                const int k = ++approach_count[vision_pending_object];
                approach_tcp += approach_drift * k;
                approach_state = k <= approach_moves ? 1 : 2;
                vision_text = approach_state == 1 ? "MOVE" : "INVIEW";
            }
            --vision_fails;
        }
    }

    void step(double dt)
    {
        // Robot: primer orden (tau 1.5 s) con 5 mm de error en estado estable en z.
        const Eigen::Vector3d goal = p_ref + Eigen::Vector3d(0, 0, -0.005);
        v = (goal - p) / 1.5;
        p += v * dt;
        q = q.slerp(std::min(1.0, dt / 1.5), q_ref);

        // Pinza: 1 % de recorrido por cada (velocidad/10) % por segundo.
        const double rate = gripper_speed * 2.0 * dt;
        double target = gripper_cmd;
        if (plate_block_pct >= 0.0 && target > plate_block_pct)
            target = plate_block_pct;
        if (gripper < target)
            gripper = std::min(target, gripper + rate);
        else
            gripper = std::max(target, gripper - rate);
    }

    task::Inputs inputs(double dt, double t, bool active, bool confirm) const
    {
        task::Inputs in;
        in.dt = dt;
        in.active = active;
        in.confirm = confirm;
        in.position = p;
        in.orientation = q;
        in.linear_speed_m_s = v.norm();
        in.angular_speed_rad_s = 0.0;
        in.gripper_detected = true;
        in.gripper_pos_pct = gripper;
        in.fixture_settled = (t - fixture_since) > 1.5;   // ~1 s de giro + asentarse
        in.fixture_fault = fixture_fault;
        in.fixture_fault_text = fixture_fault ? "*** FALLA: sobrecorriente ***" : "";
        in.vision_seq = vision_seq;
        in.vision_ok = vision_ok;
        in.vision_tcp_m = screw_tcp;
        in.vision_has_pose = screw_has_pose;
        in.vision_rpy_deg = screw_rpy;
        in.vision_pregrasp_m = pregrasp();
        in.vision_has_edge = has_edge;
        in.vision_edge_a_m = edge_a;
        in.vision_edge_b_m = edge_b;
        in.vision_is_cloud = vision_is_cloud;
        in.vision_is_approach = vision_is_approach;
        if (vision_is_approach)
        {
            in.vision_tcp_m = approach_tcp;
            in.vision_rpy_deg = approach_rpy;
            in.vision_approach_state = approach_state;
            in.vision_centroid_m = approach_tcp - Eigen::Vector3d(0.0, 0.0, 0.18);
        }
        in.vision_has_hole = vision_is_cloud && has_hole;
        in.vision_hole_m = hole;
        in.vision_hole_diam_m = hole_diam;
        in.vision_text = vision_text;
        return in;
    }
};

// Indice (0-based) del primer paso que cumple pred.
template <typename Pred>
size_t findStep(const task::TaskParams& t, Pred pred)
{
    for (size_t i = 0; i < t.steps.size(); ++i)
        if (pred(t.steps[i]))
            return i;
    return t.steps.size();
}

size_t visionIndex(const task::TaskParams& t)
{
    // primera foto de AGARRE (no el paso de centrar la camara)
    return findStep(t, [](const task::Step& s) { return s.type == task::StepType::Vision && !s.vision_approach; });
}

}  // namespace


int main()
{
    constexpr double dt = 0.001;

    std::cout << "1) Configuracion del lab (poses del 2026-10-13)\n";
    {
        const project_config::ControllerConfig cfg{};
        std::cout << "     task.enabled = " << (cfg.task.enabled ? "true" : "false")
                  << " (lo cambia el usuario en el lab; no se revisa)\n";
        const std::string err = task::validate(cfg.task);
        if (!err.empty())
            std::cout << err;
        check(err.empty(), "las poses y pasos del lab pasan la validacion");
        using project_config::Secuencia;
        if (project_config::SECUENCIA == Secuencia::SoloPlacaVision)
            check(cfg.task.steps.size() == 23, "23 pasos (placa por vision al fixture)");
        else if (project_config::SECUENCIA == Secuencia::Completa)
            check(cfg.task.steps.size() == 43, "43 pasos (22 placa por vision + 21 tornillo)");
        else
            check(cfg.task.steps.size() == 39, "39 pasos (18 placa + 21 tornillo)");
        for (Secuencia m : {Secuencia::Normal, Secuencia::SoloPlacaVision, Secuencia::Completa})
        {
            task::TaskParams t = cfg.task;
            const task::TaskParams mt = project_config::phase2Task(m);
            t.steps = mt.steps;
            t.first_step = mt.first_step;
            check(task::validate(t).empty(), "las 3 secuencias pasan la validacion");
        }
        check(task::usesVision(cfg.task), "usa vision");
        bool threw = false;
        try { task::Sequencer s(cfg.task); } catch (const std::exception&) { threw = true; }
        check(!threw, "el Sequencer se construye");

        // Una pose sin definir sigue bloqueando el arranque.
        task::TaskParams bad = cfg.task;
        bad.poses[0] = task::undefinedPose(bad.poses[0].name);
        check(task::validate(bad).find("no esta definida") != std::string::npos,
              "una pose sin definir se detecta");
    }

    std::cout << "\n2) Conversion app web (TCP) -> interfaz\n";
    {
        const task::TaskPose web{"x", Eigen::Vector3d(0.656, -0.071, 0.247),
                                 Eigen::Vector3d(-179.8, -1.0, 179.9)};
        const task::InterfaceTarget t = task::toInterface(web, 0.12);
        std::cout << "     interfaz = " << t.position.transpose() << "\n";
        check(std::abs(t.position.z() - 0.367) < 0.001, "z de la interfaz = 0.367 m (24.7 + 12 cm)");
        check((t.position.head<2>() - Eigen::Vector2d(0.656, -0.071)).norm() < 0.003,
              "x, y casi iguales (herramienta casi vertical)");
        const Eigen::Vector3d tool_z = t.orientation * Eigen::Vector3d::UnitZ();
        check(tool_z.z() < -0.99, "el eje z de la herramienta apunta hacia abajo");
    }

    std::cout << "\n3) Validacion de errores\n";
    {
        task::TaskParams t = definedTask();
        check(task::validate(t).empty(), "poses de prueba validas");

        t.steps.push_back(task::moveTo("x", "no_existe"));
        t.steps.push_back(task::fixtureTo("x", 120.0));
        t.steps.push_back(task::gripper("x", 150.0, 50.0, 1.0));
        t.poses.push_back({"lejos", Eigen::Vector3d(1.2, 0.0, 0.3), Eigen::Vector3d(180, 0, 180)});
        t.poses.push_back({"inicio", Eigen::Vector3d(0.5, 0.0, 0.4), Eigen::Vector3d(180, 0, 180)});
        const std::string err = task::validate(t);
        check(contains(err, "'no_existe' no existe"), "pose inexistente");
        check(contains(err, "angulo del fixture fuera"), "angulo del fixture fuera de -90 a 0");
        check(contains(err, "posicion de pinza fuera"), "pinza fuera de 0-100 %");
        check(contains(err, "'lejos'") && contains(err, "fuera de la caja"), "pose fuera de la caja de trabajo");
        check(contains(err, "'inicio' esta repetida"), "nombre repetido");

        task::TaskParams far = definedTask();
        far.poses[2].tcp_position_m = Eigen::Vector3d(0.25, 0.45, 0.10);
        far.max_move_distance_m = 0.40;
        check(contains(task::validate(far), "movimiento de"), "movimiento demasiado largo entre poses");
    }

    std::cout << "\n4) Secuencia completa sin confirmar cada paso\n";
    {
        task::TaskParams t = definedTask();
        t.confirm_each_step = false;
        task::Sequencer seq(t);
        World w;
        w.plate_block_pct = 93.0;   // la placa detiene el cierre en 93 %

        double time = 0.0;
        bool confirm_sent = false;
        bool froze_ok = true;
        size_t frozen_index = 0;
        double frozen_step_time = 0.0;
        bool frozen_set = false;

        while (!seq.finished() && time < 600.0)
        {
            // 10-15 s: slider OFF (la secuencia se congela a media tarea)
            const bool active = !(time > 10.0 && time < 15.0);
            if (!active && !frozen_set)
            {
                frozen_index = seq.stepIndex();
                frozen_step_time = seq.stepTime();
                frozen_set = true;
            }
            if (!active && (seq.stepIndex() != frozen_index || seq.stepTime() != frozen_step_time))
                froze_ok = false;

            const bool confirm = (time > 1.0 && !confirm_sent);
            confirm_sent = confirm_sent || confirm;

            w.apply(seq.update(w.inputs(dt, time, active, confirm)), time);
            w.step(dt);
            time += dt;
        }

        std::cout << "     terminada en " << time << " s simulados: " << seq.message() << "\n";
        check(seq.status() == task::Status::Done, "la secuencia TERMINA");
        check(frozen_set && frozen_index > 0 && frozen_index < t.steps.size(),
              "el slider OFF cae a media secuencia (paso " + std::to_string(frozen_index + 1) + ")");
        check(froze_ok, "con el slider OFF no avanza (ni el paso ni su reloj)");

        const std::vector<std::string> expected = {
            "ref", "gripper 0", "ref", "ref", "gripper 95", "ref", "ref",
            "fixture -90", "ref", "gripper 0", "ref", "fixture 0", "ref"};
        check(w.log == expected, "comandos en el orden de la Fase 2");
        if (w.log != expected)
        {
            std::cout << "     obtenido:";
            for (const auto& s : w.log) std::cout << " [" << s << "]";
            std::cout << "\n";
        }
        check(std::abs(w.gripper - 0.0) < 1e-9, "la pinza termina abierta");
    }

    std::cout << "\n5) Confirmar cada paso: sin boton no avanza\n";
    {
        task::TaskParams t = definedTask();
        t.confirm_each_step = true;
        task::Sequencer seq(t);
        World w;
        w.plate_block_pct = 93.0;

        double time = 0.0;
        for (; time < 20.0; time += dt)
        {
            w.apply(seq.update(w.inputs(dt, time, true, false)), time);
            w.step(dt);
        }
        check(seq.status() == task::Status::WaitingConfirm && seq.stepIndex() == 0,
              "20 s sin boton: sigue esperando el paso 1");
        check(w.log.empty(), "no se mando ningun comando");

        int presses = 0;
        while (!seq.finished() && time < 900.0)
        {
            const bool confirm = seq.status() == task::Status::WaitingConfirm;
            presses += confirm ? 1 : 0;
            w.apply(seq.update(w.inputs(dt, time, true, confirm)), time);
            w.step(dt);
            time += dt;
        }
        check(seq.status() == task::Status::Done, "termina presionando el boton en cada paso");
        check(presses == static_cast<int>(t.steps.size()), "un boton por paso (14)");
    }

    std::cout << "\n6) Timeout de movimiento -> pausa -> el operador continua\n";
    {
        task::TaskParams t = definedTask();
        t.confirm_each_step = false;
        t.position_tolerance_m = 0.002;   // el robot simulado tiene 5 mm de error
        t.move_timeout_s = 10.0;
        task::Sequencer seq(t);
        World w;

        double time = 0.0;
        bool paused = false;
        for (; time < 30.0 && !paused; time += dt)
        {
            w.apply(seq.update(w.inputs(dt, time, true, time < 2 * dt)), time);
            w.step(dt);
            paused = seq.status() == task::Status::Paused;
        }
        check(paused && seq.stepIndex() == 0, "se pausa en el paso 1 al no llegar");
        check(contains(seq.message(), "no llego"), "el mensaje explica el error");

        w.apply(seq.update(w.inputs(dt, time, true, true)), time);
        check(seq.status() == task::Status::Running && seq.stepIndex() == 1,
              "'Siguiente paso' continua con el paso 2");
    }

    std::cout << "\n7) Falla del fixture -> abortada\n";
    {
        task::TaskParams t = definedTask();
        t.confirm_each_step = false;
        task::Sequencer seq(t);
        World w;
        w.plate_block_pct = 93.0;

        double time = 0.0;
        while (!seq.finished() && time < 600.0)
        {
            if (!w.log.empty() && w.log.back() == "fixture -90")
            {
                w.fixture_fault = true;
                w.fixture_since = 1e9;
            }
            w.apply(seq.update(w.inputs(dt, time, true, time < 2 * dt)), time);
            w.step(dt);
            time += dt;
        }
        check(seq.status() == task::Status::Aborted, "la secuencia se aborta");
        check(seq.stepIndex() == 8 && contains(seq.message(), "sobrecorriente"),
              "en el paso 9 (girar el fixture) con el texto de la falla");
        check(w.log.back() == "fixture -90", "despues de la falla no se manda nada mas");
    }

    std::cout << "\n8) Sin pinza -> abortada en el primer paso de pinza\n";
    {
        task::TaskParams t = definedTask();
        t.confirm_each_step = false;
        task::Sequencer seq(t);
        World w;
        double time = 0.0;
        while (!seq.finished() && time < 120.0)
        {
            task::Inputs in = w.inputs(dt, time, true, time < 2 * dt);
            in.gripper_detected = false;
            w.apply(seq.update(in), time);
            w.step(dt);
            time += dt;
        }
        check(seq.status() == task::Status::Aborted && seq.stepIndex() == 1,
              "abortada en el paso 2 (abrir pinza)");
    }

    std::cout << "\n9) Secuencia del lab completa (boton en cada paso)\n";
    {
        project_config::ControllerConfig cfg{};
        cfg.task.steps = project_config::phase2Task(project_config::Secuencia::Normal).steps;   // secuencia normal
        cfg.task.first_step = 1;   // la secuencia completa (en el lab puede arrancar en otro paso)
        cfg.task.confirm_each_step = true;   // PASO A PASO (el lab arranca en AUTOMATICO)
        task::Sequencer seq(cfg.task);
        World w;
        w.plate_block_pct = 85.0;   // la placa detiene el cierre antes del 92 %

        double time = 0.0;
        int presses = 0;
        const size_t pre_idx = findStep(cfg.task, [](const task::Step& s) {
            return s.from_vision && s.vision_target == task::VisionTarget::Pregrasp;
        });
        const size_t grasp_idx = findStep(cfg.task, [](const task::Step& s) {
            return s.from_vision && s.vision_target == task::VisionTarget::Grasp;
        });
        check(pre_idx == visionIndex(cfg.task) + 2 && grasp_idx == pre_idx + 1,
              "vision -> abrir pinza -> pre-grasp -> agarre");
        Eigen::Vector3d grasp_ref = Eigen::Vector3d::Constant(std::nan(""));
        Eigen::Vector3d pre_ref = grasp_ref;
        Eigen::Vector3d lift_ref = grasp_ref;
        Eigen::Quaterniond lift_q = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond grasp_q = Eigen::Quaterniond::Identity();
        while (!seq.finished() && time < 1200.0)
        {
            const bool confirm = seq.status() == task::Status::WaitingConfirm;
            presses += confirm ? 1 : 0;
            const size_t before = seq.stepIndex();
            w.apply(seq.update(w.inputs(dt, time, true, confirm)), time);
            if (seq.stepIndex() == pre_idx && before == pre_idx && seq.status() == task::Status::Running)
                pre_ref = w.p_ref;     // pre-grasp
            if (seq.stepIndex() == grasp_idx && before == grasp_idx && seq.status() == task::Status::Running)
            {
                grasp_ref = w.p_ref;   // bajar al tornillo
                grasp_q = w.q_ref;
            }
            if (seq.stepIndex() == grasp_idx + 2 && before == grasp_idx + 2 &&
                seq.status() == task::Status::Running)
            {
                lift_ref = w.p_ref;    // subir con el tornillo
                lift_q = w.q_ref;
            }
            w.visionStep(time);
            w.step(dt);
            time += dt;
        }
        std::cout << "     terminada en " << time << " s simulados: " << seq.message() << "\n";
        check(seq.status() == task::Status::Done, "la secuencia del lab TERMINA sin pausas");
        check(presses == int(cfg.task.steps.size()), "un boton por paso");

        const std::vector<std::string> expected = {
            "fixture -90", "ref", "gripper 0", "ref", "ref", "ref", "gripper 94",
            "ref", "ref", "ref", "fixture -23", "ref", "ref", "gripper 45", "ref", "ref", "fixture -90",
            "ref", "ref", "gripper 45", "centrar", "ref", "centrar", "vision", "gripper 10", "ref", "ref", "gripper 64", "ref", "ref", "ref",
            "ref", "ref", "nube", "ref", "ref", "ref", "ref", "gripper 30", "ref"};
        check(w.log == expected, "comandos en el orden pedido");
        if (w.log != expected)
        {
            std::cout << "     obtenido:";
            for (const auto& s : w.log) std::cout << " [" << s << "]";
            std::cout << "\n";
        }
        const auto home = task::toInterface(*task::findPose(cfg.task, "HOME_MOD"), cfg.task.tcp_offset_m);
        check((w.p_ref - home.position).norm() < 1e-9, "la ultima referencia es HOME_MOD");
        check(std::abs(w.gripper - 30.0) < 1e-9, "la pinza queda en 30 % (tornillo soltado)");

        // Pasos 24-25: interfaz = TCP de la vision - 12 cm sobre el z de la
        // pinza, con la orientacion de la vision (sin offset en z).
        const Eigen::Quaterniond q_screw = task::rpyToQuaternion(w.screw_rpy);
        const Eigen::Vector3d tool_z = q_screw * Eigen::Vector3d::UnitZ();
        // SCREW_GRASP_UP: el paso "Bajar al tornillo" se queda esto arriba (z de la base)
        const double screw_up = cfg.task.steps[findStep(cfg.task, [](const task::Step& s) {
                                    return s.label == "Bajar al tornillo";
                                })].vision_dz_m;
        const Eigen::Vector3d want_gpd = w.screw_tcp - 0.12 * tool_z;
        const Eigen::Vector3d want = want_gpd + Eigen::Vector3d(0.0, 0.0, screw_up);
        const Eigen::Vector3d want_pre = w.pregrasp() - 0.12 * tool_z;
        std::cout << "     referencia al bajar al tornillo: " << grasp_ref.transpose() << " (+"
                  << screw_up * 1000.0 << " mm en z)\n";
        check(screw_up >= 0.0 && screw_up <= 0.02 && (grasp_ref - want).norm() < 1e-9 &&
                  task::angleBetween(grasp_q, q_screw) < 1e-9,
              "bajar al tornillo = agarre de la vision + SCREW_GRASP_UP en z (orientacion de la vision)");
        check((pre_ref - want_pre).norm() < 1e-9 &&
                  std::abs((pre_ref - want_gpd).norm() - 0.10) < 1e-9,
              "pre-grasp = 10 cm atras sobre la aproximacion");
        check(lift_ref.allFinite() && (lift_ref - (want + Eigen::Vector3d(0.0, 0.0, 0.07))).norm() < 1e-9 &&
                  task::angleBetween(lift_q, q_screw) < 1e-9,
              "despues de cerrar sube solo 7 cm en z, sin girar la muneca");
    }

    std::cout << "\n10) Vision: falla -> pausa -> el boton toma otra foto\n";
    {
        project_config::ControllerConfig cfg{};
        cfg.task.steps = project_config::phase2Task(project_config::Secuencia::Normal).steps;   // secuencia normal
        cfg.task.first_step = visionIndex(cfg.task) + 1;   // la foto del tornillo (sin centrar antes)
        check(task::validate(cfg.task).empty(), "arrancar en la foto del tornillo es valido");
        task::Sequencer seq(cfg.task);
        World w;
        w.p_ref = task::toInterface(*task::findPose(cfg.task, "HOME_MOD"), 0.12).position;
        w.p = w.p_ref;
        w.vision_fails = 1;

        double time = 0.0;
        int requests = 0;
        bool paused_on_vision = false;
        const size_t vi = visionIndex(cfg.task);
        while (!seq.finished() && time < 600.0 && seq.stepIndex() < vi + 2)
        {
            const bool confirm = seq.status() == task::Status::WaitingConfirm ||
                                 seq.status() == task::Status::Paused;
            if (seq.status() == task::Status::Paused && seq.stepIndex() == vi)
                paused_on_vision = true;
            const task::Commands c = seq.update(w.inputs(dt, time, true, confirm));
            requests += c.request_vision ? 1 : 0;
            w.apply(c, time);
            w.visionStep(time);
            w.step(dt);
            time += dt;
        }
        check(paused_on_vision, "la primera foto falla -> PAUSA en el paso de vision");
        check(requests == 2, "el boton pide otra foto (2 peticiones)");
        Eigen::Vector3d v;
        check(seq.visionPosition(v) && (v - w.screw_tcp).norm() < 1e-9,
              "con la segunda foto guarda la posicion del tornillo");
    }

    std::cout << "\n11) Vision fuera de limites -> pausa, no se mueve\n";
    {
        struct Bad
        {
            std::string name;
            Eigen::Vector3d tcp;
            Eigen::Vector3d rpy = Eigen::Vector3d(-1.0, -179.3, 34.0);
            bool has_pose = true;
        };
        for (const Bad& b : std::vector<Bad>{
                 {"lejos en horizontal", Eigen::Vector3d(0.45, 0.55, 0.17)},   // 0.39 m de la foto (max 0.35)
                 {"arriba de la camara", Eigen::Vector3d(0.07, 0.56, 0.30)},
                 {"fuera de la caja (z)", Eigen::Vector3d(0.07, 0.56, 0.02)},
                 {"pinza inclinada 30 grados", Eigen::Vector3d(0.08, 0.57, 0.17),
                  Eigen::Vector3d(0.0, 150.0, 0.0)},
                 {"pinza hacia arriba", Eigen::Vector3d(0.08, 0.57, 0.17), Eigen::Vector3d(0.0, 0.0, 0.0)},
                 {"servidor viejo (sin orientacion)", Eigen::Vector3d(0.08, 0.57, 0.17),
                  Eigen::Vector3d::Zero(), false}})
        {
            const std::string& name = b.name;
            project_config::ControllerConfig cfg{};
            cfg.task.steps = project_config::phase2Task(project_config::Secuencia::Normal).steps;   // secuencia normal
            cfg.task.first_step = 19;
            task::Sequencer seq(cfg.task);
            World w;
            w.p_ref = task::toInterface(*task::findPose(cfg.task, "HOME_MOD"), 0.12).position;
            w.p = w.p_ref;
            w.screw_tcp = b.tcp;
            w.screw_rpy = b.rpy;
            w.screw_has_pose = b.has_pose;

            double time = 0.0;
            while (time < 300.0 && seq.status() != task::Status::Paused && !seq.finished())
            {
                const bool confirm = seq.status() == task::Status::WaitingConfirm;
                w.apply(seq.update(w.inputs(dt, time, true, confirm)), time);
                w.visionStep(time);
                w.step(dt);
                time += dt;
            }
            Eigen::Vector3d v;
            check(seq.status() == task::Status::Paused && seq.stepIndex() == visionIndex(cfg.task) &&
                      contains(seq.message(), "vision rechazada") && !seq.visionPosition(v),
                  "rechaza: " + name);
        }

        task::TaskParams t = project_config::ControllerConfig{}.task;
        t.steps.insert(t.steps.begin(), task::moveToVision("x", "HOME_MOD", 0.05));
        t.first_step = 1;
        check(contains(task::validate(t), "no hay un paso de vision antes"),
              "moverse con la vision sin paso de vision antes es invalido");
    }

    std::cout << "\n12. Postura del codo (articulaciones de cada pose)\n";
    {
        const task::TaskParams t = project_config::ControllerConfig{}.task;
        bool all = true;
        for (const auto& pose : t.poses)
            all = all && pose.hasJoints();
        check(all, "todas las poses del lab traen articulaciones");

        // Cada Move manda la postura de su pose (con vision: la de la
        // orientacion), en [-pi, pi].
        task::TaskParams u = t;
        u.confirm_each_step = false;
        u.steps = {task::moveTo("a", "HOME_MOD"), task::moveTo("b", "ARRIBA_TOMARTORNILLOS")};
        u.first_step = 1;
        task::Sequencer seq(u);
        task::Inputs in{};
        in.dt = 0.001;
        in.active = true;
        in.confirm = true;
        in.position = task::toInterface(*task::findPose(u, "HOME_MOD"), u.tcp_offset_m).position;
        in.orientation = task::toInterface(*task::findPose(u, "HOME_MOD"), u.tcp_offset_m).orientation;
        task::Commands c{};
        for (int k = 0; k < 10 && !c.set_reference; ++k)
            c = seq.update(in);
        Eigen::Matrix<double, 7, 1> home;
        home << -3.7, 31.7, 0.12, 50.4, 0.6, 97.3, -3.3;
        check(c.set_reference && c.set_posture &&
                  (c.posture_rad - home * (M_PI / 180.0)).norm() < 1e-9,
              "HOME_MOD manda su postura (356.3 -> -3.7 grados)");

        task::TaskParams partial = t;
        partial.poses[0].joints_deg(3) = std::numeric_limits<double>::quiet_NaN();
        check(contains(task::validate(partial), "todas o ninguna"),
              "articulaciones incompletas son invalidas");

        task::TaskParams none = t;
        for (auto& pose : none.poses)
            pose.joints_deg.setConstant(std::numeric_limits<double>::quiet_NaN());
        check(task::validate(none).empty(), "sin articulaciones tambien es valido (opcional)");
    }

    std::cout << "\n13. Pose del TCP para la vision (interfaz -> app web)\n";
    {
        const task::TaskParams t = project_config::ControllerConfig{}.task;
        bool ok = true;
        for (const auto& pose : t.poses)
        {
            const task::InterfaceTarget it = task::toInterface(pose, t.tcp_offset_m);
            double tool[6];
            task::interfaceToToolPose(it.position, it.orientation, t.tcp_offset_m, tool);
            const Eigen::Vector3d p(tool[0], tool[1], tool[2]);
            const double ang = task::angleBetween(task::rpyToQuaternion(Eigen::Vector3d(tool[3], tool[4], tool[5])),
                                                  task::rpyToQuaternion(pose.rpy_deg));
            ok = ok && (p - pose.tcp_position_m).norm() < 1e-9 && ang < 1e-6 &&
                 std::abs(tool[4]) <= 90.0 + 1e-9;
        }
        check(ok, "ida y vuelta exacta en las " + std::to_string(t.poses.size()) + " poses del lab");

        double tool[6];
        const task::TaskPose* a = task::findPose(t, "ARRIBA_TOMARTORNILLOS");
        const task::InterfaceTarget it = task::toInterface(*a, t.tcp_offset_m);
        task::interfaceToToolPose(it.position, it.orientation, t.tcp_offset_m, tool);
        check(std::abs(tool[0] - 0.063) < 1e-9 && std::abs(tool[1] - 0.552) < 1e-9 &&
                  std::abs(tool[2] - 0.291) < 1e-9,
              "ARRIBA_TOMARTORNILLOS -> 0.063 0.552 0.291 (la pose de la foto)");
    }

    std::cout << "\n14. Respuesta del screw_server (V2 con pose, viejo solo posicion)\n";
    {
        screw_vision::Reply r;
        check(screw_vision::parseReply(
                  "SCREW 3 OK 0.04122 0.56544 0.12950 -0.98280 -179.30500 33.99130 "
                  "0.04319 0.56469 0.22948 772.683 5",
                  r) &&
                  r.ok && r.has_pose && r.seq == 3 && r.count == 5 &&
                  std::abs(r.tcp_m.z() - 0.1295) < 1e-12 && std::abs(r.rpy_deg.z() - 33.9913) < 1e-12 &&
                  std::abs(r.pregrasp_m.z() - 0.22948) < 1e-12 && std::abs(r.score - 772.683) < 1e-9,
              "V2: agarre + orientacion + pre-grasp");
        check(screw_vision::parseReply("SCREW 4 OK 0.1 0.5 0.11 1164 12", r) && r.ok && !r.has_pose &&
                  std::abs(r.tcp_m.z() - 0.11) < 1e-12 && r.count == 12,
              "viejo: solo posicion");
        check(!screw_vision::parseReply("SCREW 5 OK 0.1 0.5 0.11 1 2 3 1164 12", r),
              "numero de campos incorrecto -> se ignora");
        check(!screw_vision::parseReply("SCREW 6 OK 0.1 0.5 0.11 nan 2 3 0.1 0.5 0.21 1 2", r),
              "NaN -> se ignora");
        check(screw_vision::parseReply("SCREW 7 FAIL GPD found no grasps", r) && !r.ok &&
                  r.text == "GPD found no grasps",
              "FAIL con motivo");
    }

    std::cout << "\n15. Canto de la placa en la respuesta (EDGE / NOEDGE)\n";
    {
        screw_vision::Reply r;
        const std::string base = "SCREW 8 OK 0.46 -0.46 0.21 -179.3 2.5 174.1 0.457 -0.466 0.31 389.8 5";
        check(screw_vision::parseReply(base + " EDGE 0.464 -0.521 0.177 0.464 -0.335 0.177", r) && r.ok &&
                  r.has_pose && r.has_edge && std::abs(r.edge_a_m.y() + 0.521) < 1e-12 &&
                  std::abs(r.edge_b_m.z() - 0.177) < 1e-12,
              "con EDGE: los dos extremos del canto");
        check(screw_vision::parseReply(base + " NOEDGE", r) && r.ok && r.has_pose && !r.has_edge,
              "con NOEDGE: agarre sin canto");
        check(screw_vision::parseReply(base, r) && r.ok && !r.has_edge, "servidor sin canto (14 campos)");
        check(!screw_vision::parseReply(base + " EDGE 1 2 3 4 5", r), "EDGE incompleto -> se ignora");
        check(!screw_vision::parseReply(base + " XEDGE", r), "palabra desconocida -> se ignora");
    }

    std::cout << "\n16. Placa al fixture: queda en el mismo lugar sin importar el agarre\n";
    {
        // Placa parada: canto a lo largo de y de la base, x = 0.46, z = 0.17,
        // de y = -0.53 a -0.35 (centro en y = -0.44).
        const Eigen::Vector3d ea(0.46, -0.53, 0.17), eb(0.46, -0.35, 0.17);
        const Eigen::Vector3d center = 0.5 * (ea + eb);
        const Eigen::Vector3d grasp_rpy(180.0, 0.0, 180.0);   // cierra en x de la base
        const task::TaskPose fix{"fix", Eigen::Vector3d(0.60, 0.20, 0.25), Eigen::Vector3d(180.0, 0.0, 180.0)};

        // Corre foto -> pre-grasp -> agarre -> cerrar -> fixture corregido y
        // regresa (pose TCP del agarre medida, pose TCP objetivo del fixture).
        auto run = [&](const Eigen::Vector3d& grasp_tcp, const Eigen::Vector3d& rpy, double h_ref,
                       Eigen::Matrix3d& Rg, Eigen::Vector3d& tg, Eigen::Matrix3d& Rf, Eigen::Vector3d& tf,
                       task::Status& end, std::string& msg) {
            task::TaskParams t;
            const Eigen::Vector3d down(180.0, 0.0, 180.0);
            t.poses = {{"foto", Eigen::Vector3d(0.46, -0.44, 0.33), down}, fix};
            t.steps = {
                task::visionCapturePlate  ("foto",      "foto"),
                task::moveToVisionPregrasp("pre-grasp", "foto"),
                task::moveToVisionGrasp   ("agarre",    "foto"),
                task::gripper             ("cerrar",    93.0, 30.0, 20.0),
                // sube a z fija (como PLATE_LIFT_Z): la correccion debe seguir
                // usando donde AGARRO, no donde quedo arriba
                task::moveToVisionGraspAtZ("subir",     "foto", 0.364),
                task::moveToPlateCorrected("fixture",   "fix"),
            };
            t.enabled = true;
            t.confirm_each_step = false;
            t.workspace_min_m = Eigen::Vector3d(0.0, -0.8, 0.0);
            t.workspace_max_m = Eigen::Vector3d(0.9, 0.8, 0.7);
            t.max_move_distance_m = 1.0;
            t.plate_nominal_tcp_above_edge_m = h_ref;
            task::Sequencer seq(t);
            World w;
            w.p_ref = w.p = Eigen::Vector3d(0.46, -0.44, 0.45);
            w.q_ref = w.q = task::rpyToQuaternion(down);
            w.screw_tcp = grasp_tcp;
            w.screw_rpy = rpy;
            w.has_edge = true;
            w.edge_a = ea;
            w.edge_b = eb;
            w.plate_block_pct = 93.0;
            const double dt = 0.01;
            bool pressed = false, got_grasp = false;
            for (double time = 0.0; time < 120.0 && !seq.finished(); time += dt)
            {
                w.visionStep(time);
                const size_t before = seq.stepIndex();
                const task::Commands c = seq.update(w.inputs(dt, time, true, !pressed));
                pressed = true;
                if (!got_grasp && before == 2 && seq.stepIndex() == 3)
                {
                    // donde quedo la TCP al terminar el agarre (lo que guarda la secuencia)
                    Rg = w.q.normalized().toRotationMatrix();
                    tg = w.p + Rg * Eigen::Vector3d(0, 0, 0.12);
                    got_grasp = true;
                }
                w.apply(c, time);
                w.step(dt);
            }
            Rf = w.q_ref.normalized().toRotationMatrix();
            tf = w.p_ref + Rf * Eigen::Vector3d(0, 0, 0.12);
            end = seq.status();
            msg = seq.plateCorrectionText();
        };
        // Centro del canto en el frame de la TCP, como lo sujetan los dedos (x = 0).
        auto inHand = [&](const Eigen::Matrix3d& Rg, const Eigen::Vector3d& tg) {
            Eigen::Vector3d o = Rg.transpose() * (center - tg);
            o.x() = 0.0;
            return o;
        };
        const double h_ref = 0.012;
        const Eigen::Vector3d ref_center =
            fix.tcp_position_m + task::rpyToQuaternion(fix.rpy_deg) * Eigen::Vector3d(0, 0, h_ref);

        struct Case { const char* what; Eigen::Vector3d tcp; Eigen::Vector3d rpy; };
        const Case cases[] = {
            {"agarre al centro y a la altura de referencia", Eigen::Vector3d(0.46, -0.44, 0.182), grasp_rpy},
            {"agarre 5 cm hacia +y y 2.8 cm mas arriba",      Eigen::Vector3d(0.46, -0.39, 0.210), grasp_rpy},
            {"agarre 7 cm hacia -y, pinza girada 180 grados", Eigen::Vector3d(0.46, -0.51, 0.195),
             Eigen::Vector3d(180.0, 0.0, 0.0)},
            {"agarre 3 cm hacia +y con la pinza inclinada 12 grados sobre el canto",
             Eigen::Vector3d(0.46, -0.41, 0.20), Eigen::Vector3d(180.0, 0.0, 180.0) + Eigen::Vector3d(12.0, 0.0, 0.0)},
        };
        for (const Case& c : cases)
        {
            Eigen::Matrix3d Rg, Rf;
            Eigen::Vector3d tg, tf;
            task::Status end;
            std::string msg;
            run(c.tcp, c.rpy, h_ref, Rg, tg, Rf, tf, end, msg);
            const Eigen::Vector3d placed = tf + Rf * inHand(Rg, tg);
            const double err = (placed - ref_center).norm();
            // el canto queda paralelo al de referencia (y de la pinza en 'fix')
            const Eigen::Vector3d e_in_hand = Rg.transpose() * (eb - ea).normalized();
            const Eigen::Vector3d e_placed = Rf * Eigen::Vector3d(0.0, e_in_hand.y(), e_in_hand.z()).normalized();
            const Eigen::Vector3d e_ref = task::rpyToQuaternion(fix.rpy_deg) * Eigen::Vector3d::UnitY();
            const double par = std::abs(e_placed.dot(e_ref));
            check(end == task::Status::Done && err < 1e-6 && par > 1.0 - 1e-9,
                  std::string(c.what) + ": centro de la placa a " + std::to_string(err * 1000.0) +
                      " mm del de referencia");
            if (c.tcp.y() == -0.39)
                std::cout << "       (" << msg << ")\n";
        }

        // Correccion mas grande que el limite -> aborta, no se mueve al fixture.
        {
            Eigen::Matrix3d Rg, Rf;
            Eigen::Vector3d tg, tf;
            task::Status end;
            std::string msg;
            run(Eigen::Vector3d(0.46, -0.44, 0.182), grasp_rpy, 0.20, Rg, tg, Rf, tf, end, msg);
            check(end == task::Status::Aborted, "correccion de 19 cm (mas que plate_max_shift_m) -> abortada");
        }
        // Pinza cerrando a 45 grados del canto -> la foto se rechaza.
        {
            task::TaskParams t;
            const Eigen::Vector3d down(180.0, 0.0, 180.0);
            t.poses = {{"foto", Eigen::Vector3d(0.46, -0.44, 0.33), down}, fix};
            t.steps = {task::visionCapturePlate("foto", "foto"), task::moveToPlateCorrected("fixture", "fix")};
            t.enabled = true;
            t.confirm_each_step = false;
            t.workspace_min_m = Eigen::Vector3d(0.0, -0.8, 0.0);
            t.workspace_max_m = Eigen::Vector3d(0.9, 0.8, 0.7);
            task::Sequencer seq(t);
            World w;
            w.p_ref = w.p = Eigen::Vector3d(0.46, -0.44, 0.45);
            w.q_ref = w.q = task::rpyToQuaternion(down);
            w.screw_tcp = Eigen::Vector3d(0.46, -0.44, 0.19);
            w.screw_rpy = Eigen::Vector3d(180.0, 0.0, 135.0);
            w.has_edge = true;
            w.edge_a = ea;
            w.edge_b = eb;
            for (double time = 0.0; time < 10.0; time += 0.01)
            {
                w.visionStep(time);
                const task::Commands c = seq.update(w.inputs(0.01, time, true, time < 0.005));
                w.apply(c, time);
                w.step(0.01);
            }
            check(seq.status() == task::Status::Paused && contains(seq.message(), "perpendicular"),
                  "pinza a 45 grados del canto -> PAUSA: " + seq.message().substr(0, 90));
        }
        // Foto sin canto -> pausa en la vision.
        {
            task::TaskParams t;
            const Eigen::Vector3d down(180.0, 0.0, 180.0);
            t.poses = {{"foto", Eigen::Vector3d(0.46, -0.44, 0.33), down}, fix};
            t.steps = {task::visionCapturePlate("foto", "foto"), task::moveToPlateCorrected("fixture", "fix")};
            t.enabled = true;
            t.confirm_each_step = false;
            t.workspace_min_m = Eigen::Vector3d(0.0, -0.8, 0.0);
            t.workspace_max_m = Eigen::Vector3d(0.9, 0.8, 0.7);
            task::Sequencer seq(t);
            World w;
            w.p_ref = w.p = Eigen::Vector3d(0.46, -0.44, 0.45);
            w.q_ref = w.q = task::rpyToQuaternion(down);
            w.screw_tcp = Eigen::Vector3d(0.46, -0.44, 0.19);
            w.screw_rpy = down;
            for (double time = 0.0; time < 10.0; time += 0.01)
            {
                w.visionStep(time);
                const task::Commands c = seq.update(w.inputs(0.01, time, true, time < 0.005));
                w.apply(c, time);
                w.step(0.01);
            }
            check(seq.status() == task::Status::Paused && contains(seq.message(), "canto de la placa"),
                  "foto de la placa sin canto -> PAUSA (otra foto)");
        }
    }

    std::cout << "\n17. Foto solo de nube (CLOUD, sin GPD) encima del hoyo\n";
    {
        screw_vision::Reply r;
        const double tool[6] = {0.1, 0.5, 0.3, 180.0, 0.0, 90.0};
        check(screw_vision::formatRequest(9, tool, true).rfind("CLOUD 9 ", 0) == 0 &&
                  screw_vision::formatRequest(9, tool).rfind("CAPTURE 9 ", 0) == 0,
              "peticion CLOUD / CAPTURE");
        const std::string plate_req = screw_vision::formatRequest(9, tool, false, "PLATE");
        check(plate_req.rfind("CAPTURE 9 ", 0) == 0 && plate_req.size() > 6 &&
                  plate_req.compare(plate_req.size() - 6, 6, " PLATE") == 0 &&
                  screw_vision::formatRequest(9, tool, true, "PLATE").find("PLATE") == std::string::npos &&
                  screw_vision::formatRequest(9, tool).find("PLATE") == std::string::npos,
              "foto de la placa: CAPTURE ... PLATE (solo ahi)");
        const std::string ap = screw_vision::formatRequest(4, tool, false, "BOLT", true);
        check(ap.rfind("APPROACH 4 ", 0) == 0 && ap.compare(ap.size() - 5, 5, " BOLT") == 0,
              "centrar: APPROACH ... BOLT");
        check(screw_vision::parseReply("APPROACH 4 OK MOVE 0.47473 0.09985 0.30628 0.0 -180.0 0.002 "
                                       "0.47914 0.12812 0.12933 305", r) &&
                  r.ok && r.is_approach && r.approach_state == 1 && r.count == 305 &&
                  std::abs(r.tcp_m.z() - 0.30628) < 1e-9 && std::abs(r.centroid_m.y() - 0.12812) < 1e-9,
              "APPROACH MOVE: pose del EE y centroide");
        check(screw_vision::parseReply("APPROACH 5 OK INVIEW 0.5 0 0.3 180 0 180 0.5 0 0.1 9", r) &&
                  r.approach_state == 2 &&
                  screw_vision::parseReply("APPROACH 6 FAIL no blue object in the image", r) && !r.ok &&
                  r.is_approach && r.text == "no blue object in the image" &&
                  !screw_vision::parseReply("APPROACH 7 OK WAIT 0.5 0 0.3 180 0 180 0.5 0 0.1 9", r),
              "APPROACH INVIEW / FAIL / estado invalido");
        check(screw_vision::parseReply("CLOUD 9 OK hoyo_174350 51234", r) && r.ok && r.is_cloud &&
                  r.seq == 9 && r.text == "hoyo_174350" && r.count == 51234,
              "CLOUD OK: nombre y puntos");
        check(screw_vision::parseReply("CLOUD 10 FAIL could not read the depth camera", r) && !r.ok &&
                  r.is_cloud && r.text == "could not read the depth camera",
              "CLOUD FAIL con motivo");
        check(!screw_vision::parseReply("CLOUD 11 OK hoyo_1", r), "CLOUD incompleto -> se ignora");
        check(screw_vision::parseReply("SCREW 3 OK 0.1 0.5 0.11 1164 12", r) && !r.is_cloud,
              "SCREW no es nube");

        // En la secuencia: la nube no cambia el agarre del tornillo.
        for (bool as_screw : {false, true})
        {
            project_config::ControllerConfig cfg{};
            cfg.task.steps = project_config::phase2Task(project_config::Secuencia::Normal).steps;
            cfg.task.confirm_each_step = true;
            const size_t ci = findStep(cfg.task, [](const task::Step& s) { return s.vision_cloud_only; });
            check(ci < cfg.task.steps.size() && cfg.task.steps[ci].pose == "TOMAR_FOTO_PLACA" &&
                      cfg.task.steps[ci].vision_needs_hole && cfg.task.steps[ci + 1].pose == "HOME_MOD" &&
                      cfg.task.steps[ci + 2].vision_target == task::VisionTarget::Hole &&
                      cfg.task.steps[ci + 2].vision_dz_m > 0.0 &&
                      cfg.task.steps[ci + 3].vision_target == task::VisionTarget::Hole &&
                      cfg.task.steps[ci + 4].vision_target == task::VisionTarget::Hole &&
                      cfg.task.steps[ci + 4].pose == "SOLTAR_TORNILLO1",
                  "foto en TOMAR_FOTO_PLACA -> HOME_MOD -> encima del hoyo + HOLE_ABOVE -> encima -> bajar");
            cfg.task.first_step = ci + 1;
            check(task::validate(cfg.task).empty(), "arrancar en la nube es valido (no da posicion)");
            task::Sequencer seq(cfg.task);
            World w;
            w.reply_cloud_as_screw = as_screw;
            int clouds = 0, gpd = 0;
            for (double time = 0.0; time < 10.0 && seq.stepIndex() == ci && seq.status() != task::Status::Paused;
                 time += 0.01)
            {
                w.visionStep(time);
                const task::Commands c = seq.update(w.inputs(0.01, time, true, time < 0.005));
                clouds += (c.request_vision && c.vision_cloud_only) ? 1 : 0;
                gpd += (c.request_vision && !c.vision_cloud_only) ? 1 : 0;
                w.apply(c, time);
                w.step(0.01);
            }
            Eigen::Vector3d v;
            if (!as_screw)
                check(clouds == 1 && gpd == 0 && seq.stepIndex() == ci + 1 &&
                          contains(seq.message(), "hoyo mas chico") && !seq.visionPosition(v),
                      "pide solo la nube, termina con el hoyo y no guarda agarre");
            else
                check(seq.status() == task::Status::Paused && seq.stepIndex() == ci &&
                          contains(seq.message(), "no contesto con una nube"),
                      "servidor viejo (contesta SCREW) -> PAUSA");
        }
    }

    std::cout << "\n17b. Hoyo de la vision: respuesta, objetivo y rechazos\n";
    {
        screw_vision::Reply r;
        check(screw_vision::parseReply("CLOUD 3 OK hoyo_163544 77400 HOLE 0.49644 0.18033 0.20605 0.02600 3", r) &&
                  r.ok && r.is_cloud && r.has_hole && r.hole_count == 3 &&
                  std::abs(r.hole_m.y() - 0.18033) < 1e-12 && std::abs(r.hole_diam_m - 0.026) < 1e-12,
              "CLOUD con HOLE: centro, diametro y numero de hoyos");
        check(screw_vision::parseReply("CLOUD 3 OK hoyo_1 77400 NOHOLE", r) && r.ok && !r.has_hole,
              "CLOUD con NOHOLE");
        check(!screw_vision::parseReply("CLOUD 3 OK hoyo_1 77400 HOLE 0.5 0.1 0.2", r),
              "HOLE incompleto -> se ignora");
        check(!screw_vision::parseReply("CLOUD 3 OK hoyo_1 77400 HOLE 0.5 nan 0.2 0.02 1", r),
              "HOLE con NaN -> se ignora");

        // Corre desde la foto del hoyo hasta bajar; devuelve el estado y las
        // referencias (TCP) de los dos movimientos al hoyo.
        Eigen::Vector3d high_ref;
        auto run = [&](World w, Eigen::Vector2d offset, Eigen::Vector3d& above, Eigen::Vector3d& down,
                       std::string& msg) {
            project_config::ControllerConfig cfg{};
            cfg.task.steps = project_config::phase2Task(project_config::Secuencia::Normal).steps;
            cfg.task.hole_offset_xy_m = offset;
            cfg.task.confirm_each_step = true;
            const size_t ci = findStep(cfg.task, [](const task::Step& s) { return s.vision_needs_hole; });
            cfg.task.first_step = ci + 1;
            task::Sequencer seq(cfg.task);
            const auto start = task::toInterface(*task::findPose(cfg.task, "TOMAR_FOTO_PLACA"), 0.12);
            w.p_ref = w.p = start.position;
            w.q_ref = w.q = start.orientation;
            above = down = high_ref = Eigen::Vector3d::Constant(std::nan(""));
            for (double time = 0.0; time < 80.0 && seq.stepIndex() < ci + 5 && seq.status() != task::Status::Paused &&
                                    !seq.finished();
                 time += 0.01)
            {
                const bool confirm = seq.status() == task::Status::WaitingConfirm;
                const size_t before = seq.stepIndex();
                const task::Commands c = seq.update(w.inputs(0.01, time, true, confirm));
                w.apply(c, time);
                if (c.set_reference && before == seq.stepIndex())
                {
                    const Eigen::Vector3d tcp = w.p_ref + w.q_ref.normalized().toRotationMatrix().col(2) * 0.12;
                    if (seq.stepIndex() == ci + 2) high_ref = tcp;
                    if (seq.stepIndex() == ci + 3) above = tcp;
                    if (seq.stepIndex() == ci + 4) down = tcp;
                }
                w.visionStep(time);
                w.step(0.01);
            }
            msg = seq.message();
            return seq.status();
        };
        Eigen::Vector3d above, down;
        std::string msg;
        World w;
        w.hole = Eigen::Vector3d(0.4964, 0.1803, 0.206);   // el hoyo de 26 mm de la foto del 2026-10-20
        run(w, Eigen::Vector2d::Zero(), above, down, msg);
        // HOLE_ABOVE de phase2Task (el usuario lo cambia: 10 cm, 5 cm, ...)
        const task::TaskParams lab = project_config::phase2Task(project_config::Secuencia::Normal);
        const double hole_above =
            lab.steps[findStep(lab, [](const task::Step& s) { return s.vision_target == task::VisionTarget::Hole; })]
                .vision_dz_m;
        check(hole_above > 0.0 && std::abs(high_ref.x() - 0.4964) < 1e-6 && std::abs(high_ref.y() - 0.1803) < 1e-6 &&
                  std::abs(high_ref.z() - (0.310 + hole_above)) < 1e-6,
              "primero encima + HOLE_ABOVE: x y del hoyo, z de ARRIBA_TORNILLO1 + HOLE_ABOVE");
        check(std::abs(above.x() - 0.4964) < 1e-6 && std::abs(above.y() - 0.1803) < 1e-6 &&
                  std::abs(above.z() - 0.310) < 1e-6,
              "encima del hoyo: x y del hoyo, z de ARRIBA_TORNILLO1");
        const double release_up =
            lab.steps[findStep(lab, [](const task::Step& s) {
                return s.vision_target == task::VisionTarget::Hole && s.pose == "SOLTAR_TORNILLO1";
            })].vision_dz_m;
        check(release_up >= 0.0 && std::abs(down.x() - 0.4964) < 1e-6 && std::abs(down.y() - 0.1803) < 1e-6 &&
                  std::abs(down.z() - (0.256 + release_up)) < 1e-6,
              "bajar: x y del hoyo, z de SOLTAR_TORNILLO1 + SCREW_RELEASE_UP");
        run(w, Eigen::Vector2d(0.003, -0.002), above, down, msg);
        check(std::abs(down.x() - 0.4994) < 1e-6 && std::abs(down.y() - 0.1783) < 1e-6,
              "hole_offset_xy_m se suma al centro del hoyo");

        World none;
        none.has_hole = false;
        check(run(none, Eigen::Vector2d::Zero(), above, down, msg) == task::Status::Paused &&
                  contains(msg, "no se encontro ningun hoyo"),
              "sin hoyo -> PAUSA (otra foto)");
        World far;
        far.hole = Eigen::Vector3d(0.524, 0.30, 0.208);
        check(run(far, Eigen::Vector2d::Zero(), above, down, msg) == task::Status::Paused &&
                  contains(msg, "hoyo rechazado"),
              "hoyo a 40 cm de la foto -> PAUSA");
        World side;
        side.hole = Eigen::Vector3d(0.75, 0.0, 0.208);   // 25 cm de la foto (aceptado), 24 de ARRIBA_TORNILLO1
        check(run(side, Eigen::Vector2d::Zero(), above, down, msg) == task::Status::Aborted &&
                  contains(msg, "en horizontal de 'ARRIBA_TORNILLO1'") && !high_ref.allFinite(),
              "hoyo a mas de 20 cm de ARRIBA_TORNILLO1 -> ABORTA sin moverse");
        World high;
        high.hole = Eigen::Vector3d(0.519, 0.046, 0.38);   // TOMAR_FOTO_PLACA z 0.395 - 3 cm = 0.365
        check(run(high, Eigen::Vector2d::Zero(), above, down, msg) == task::Status::Paused &&
                  contains(msg, "no esta debajo"),
              "hoyo arriba de la camara -> PAUSA");

        task::TaskParams t = project_config::ControllerConfig{}.task;
        t.steps.insert(t.steps.begin(), task::moveToHole("x", "HOME_MOD"));
        t.first_step = 1;
        check(contains(task::validate(t), "no hay una foto del hoyo antes"),
              "ir al hoyo sin foto del hoyo antes es invalido");
    }

    std::cout << "\n18. Secuencia COMPLETA: placa por vision -> fixture -> tornillo -> nube del hoyo\n";
    {
        project_config::ControllerConfig cfg{};
        const task::TaskParams full = project_config::phase2Task(project_config::Secuencia::Completa);
        cfg.task.steps = full.steps;
        cfg.task.first_step = full.first_step;
        cfg.task.plate_nominal_tcp_above_edge_m = full.plate_nominal_tcp_above_edge_m;
        task::Sequencer seq(cfg.task);

        // Foto 1 (placa): agarre en TOMAR_PLACA1_NEW, canto 18 cm centrado
        // 1.2 cm debajo de la TCP y perpendicular al cierre -> correccion ~0.
        const task::TaskPose* take = task::findPose(cfg.task, "TOMAR_PLACA1_NEW");
        const Eigen::Matrix3d R = task::rpyToQuaternion(take->rpy_deg).toRotationMatrix();
        const Eigen::Vector3d c = take->tcp_position_m + R.col(2) * full.plate_nominal_tcp_above_edge_m;
        World w;
        w.plate_block_pct = 85.0;
        const Eigen::Vector3d screw_tcp = w.screw_tcp, screw_rpy = w.screw_rpy;
        w.screw_tcp = take->tcp_position_m;
        w.screw_rpy = take->rpy_deg;
        w.has_edge = true;
        w.edge_a = c - 0.09 * R.col(1);
        w.edge_b = c + 0.09 * R.col(1);

        int photos = 0, clouds = 0;
        double time = 0.0;
        Eigen::Vector3d fix_ref = Eigen::Vector3d::Constant(std::nan(""));
        const auto colocar = task::toInterface(*task::findPose(cfg.task, "COLOCAR_PLACA"), cfg.task.tcp_offset_m);
        while (!seq.finished() && seq.status() != task::Status::Paused && time < 2000.0)
        {
            const bool confirm = seq.status() == task::Status::WaitingConfirm;
            const task::Commands cmd = seq.update(w.inputs(dt, time, true, confirm));
            if (cmd.request_vision && !cmd.vision_cloud_only && !cmd.vision_approach && ++photos == 2)
            {
                // Foto 2 (tornillo): como el servidor con un tornillo, sin canto.
                w.screw_tcp = screw_tcp;
                w.screw_rpy = screw_rpy;
                w.has_edge = false;
            }
            clouds += (cmd.request_vision && cmd.vision_cloud_only) ? 1 : 0;
            w.apply(cmd, time);
            if (cmd.set_reference && contains(seq.message(), "'COLOCAR_PLACA'"))
                fix_ref = w.p_ref;
            w.visionStep(time);
            w.step(dt);
            time += dt;
        }
        std::cout << "     " << seq.message() << "\n     " << seq.plateCorrectionText()
                  << "\n     COLOCAR_PLACA - referencia [mm]: " << (fix_ref - colocar.position).transpose() * 1000.0
                  << "\n";
        check(seq.status() == task::Status::Done, "la secuencia completa TERMINA sin pausas");
        check(photos == 2 && clouds == 1, "2 fotos con GPD (placa, tornillo) + 1 nube del hoyo");
        // Solo queda la profundidad: el robot simulado aun se arrastra unos mm
        // al darse por llegado, y la correccion usa la TCP MEDIDA (bien).
        check(contains(seq.plateCorrectionText(), "0.0 cm del centro") &&   // 0.0 o -0.0
                  contains(seq.plateCorrectionText(), "giro 0.0 grados") &&
                  (fix_ref - colocar.position).norm() < 0.01,
              "agarre al centro -> COLOCAR_PLACA casi sin correccion (centro 0, giro 0, < 1 cm)");
    }

    std::cout << "\n19. AUTOMATICO / PASO A PASO\n";
    {
        project_config::ControllerConfig lab{};
        check(!lab.task.confirm_each_step, "el lab arranca en AUTOMATICO");

        // Corre la secuencia del lab (completa) y cuenta los botones.
        auto run = [&](bool automatic, int switch_at_step, int& presses, size_t& waits_after_switch) {
            project_config::ControllerConfig cfg{};
            const task::TaskParams full = project_config::phase2Task(project_config::Secuencia::Completa);
            cfg.task.steps = full.steps;
            cfg.task.first_step = 1;
            cfg.task.plate_nominal_tcp_above_edge_m = full.plate_nominal_tcp_above_edge_m;
            task::Sequencer seq(cfg.task);
            seq.setAutomatic(automatic);
            const task::TaskPose* take = task::findPose(cfg.task, "TOMAR_PLACA1_NEW");
            const Eigen::Matrix3d R = task::rpyToQuaternion(take->rpy_deg).toRotationMatrix();
            const Eigen::Vector3d c = take->tcp_position_m + R.col(2) * full.plate_nominal_tcp_above_edge_m;
            World w;
            w.plate_block_pct = 85.0;
            const Eigen::Vector3d screw_tcp = w.screw_tcp, screw_rpy = w.screw_rpy;
            w.screw_tcp = take->tcp_position_m;
            w.screw_rpy = take->rpy_deg;
            w.has_edge = true;
            w.edge_a = c - 0.09 * R.col(1);
            w.edge_b = c + 0.09 * R.col(1);
            presses = 0;
            waits_after_switch = 0;
            int photos = 0;
            for (double time = 0.0; !seq.finished() && seq.status() != task::Status::Paused && time < 3000.0;
                 time += dt)
            {
                if (switch_at_step >= 0 && seq.stepIndex() + 1 == size_t(switch_at_step))
                {
                    seq.setAutomatic(!automatic);
                    switch_at_step = -1;
                }
                const bool confirm = seq.status() == task::Status::WaitingConfirm;
                presses += confirm ? 1 : 0;
                if (confirm && switch_at_step < 0 && seq.stepIndex() > 0)
                    ++waits_after_switch;
                const task::Commands cmd = seq.update(w.inputs(dt, time, true, confirm));
                if (cmd.request_vision && !cmd.vision_cloud_only && !cmd.vision_approach && ++photos == 2)
                {
                    w.screw_tcp = screw_tcp;
                    w.screw_rpy = screw_rpy;
                    w.has_edge = false;
                }
                w.apply(cmd, time);
                w.visionStep(time);
                w.step(dt);
            }
            return seq.status();
        };
        int presses = 0;
        size_t waits = 0;
        check(run(true, -1, presses, waits) == task::Status::Done && presses == 1,
              "AUTOMATICO: la secuencia completa con UN solo boton (el de inicio)");
        check(run(false, -1, presses, waits) == task::Status::Done && presses == 43,
              "PASO A PASO: un boton por paso (43)");
        const task::Status st30 = run(true, 30, presses, waits);
        std::cout << "     botones: " << presses << "\n";
        check(st30 == task::Status::Done && presses == 1 + 13,
              "AUTOMATICO -> PASO A PASO durante el paso 30: botones para los pasos 31 a 43");

        // Cambiar a AUTOMATICO con un paso esperando el boton no lo arranca.
        project_config::ControllerConfig cfg{};
        cfg.task.confirm_each_step = true;
        task::Sequencer seq(cfg.task);
        World w;
        seq.setAutomatic(true);
        bool moved = false;
        for (double time = 0.0; time < 2.0; time += dt)
        {
            const task::Commands cmd = seq.update(w.inputs(dt, time, true, false));
            moved = moved || cmd.set_reference || cmd.set_gripper || cmd.move_fixture || cmd.request_vision;
        }
        check(seq.status() == task::Status::WaitingConfirm && !moved && seq.automatic(),
              "cambiar el modo no arranca un paso que espera el boton");
        check(contains(seq.statusText(), "Modo                    : AUTOMATICO"), "el tablero dice el modo");
    }

    std::cout << "\n20. Centrar la camara sobre el objeto (APPROACH, approach_object de Antony)\n";
    {
        // Corre solo el paso de centrar la placa; devuelve el estado y cuenta
        // peticiones y referencias.
        auto run = [&](World w, std::string& msg, int& asks, int& refs, Eigen::Vector3d& last_ref) {
            project_config::ControllerConfig cfg{};
            const task::TaskParams full = project_config::phase2Task(project_config::Secuencia::Completa);
            cfg.task.steps = full.steps;
            const size_t ai = findStep(cfg.task, [](const task::Step& s) { return s.vision_approach; });
            cfg.task.first_step = ai + 1;
            cfg.task.confirm_each_step = true;
            task::Sequencer seq(cfg.task);
            w.p_ref = w.p = task::toInterface(*task::findPose(cfg.task, "ARRIBA_PLACA1_NEW"), 0.12).position;
            asks = refs = 0;
            for (double time = 0.0; time < 300.0 && seq.stepIndex() == ai && seq.status() != task::Status::Paused;
                 time += dt)
            {
                const bool confirm = seq.status() == task::Status::WaitingConfirm;
                const task::Commands c = seq.update(w.inputs(dt, time, true, confirm));
                asks += c.request_vision ? 1 : 0;
                if (c.set_reference)
                {
                    ++refs;
                    last_ref = c.position + c.orientation.toRotationMatrix().col(2) * 0.12;
                }
                if (c.request_vision)
                    check(c.vision_approach && c.vision_object == "PLATE", "pide APPROACH ... PLATE");
                w.apply(c, time);
                w.visionStep(time);
                w.step(dt);
            }
            msg = seq.message();
            return std::make_pair(seq.status(), seq.stepIndex() == ai ? 0 : 1);
        };
        std::string msg;
        int asks = 0, refs = 0;
        Eigen::Vector3d ref;
        World w;
        auto r = run(w, msg, asks, refs, ref);
        check(r.second == 1 && asks == 2 && refs == 1 && (ref - w.plate_approach_tcp).norm() < 1e-9,
              "MOVE -> va a la pose del EE -> otra foto -> IN VIEW: termina (2 fotos, 1 movimiento)");
        World far = w;
        far.approach_moves = 99;
        r = run(far, msg, asks, refs, ref);
        check(r.first == task::Status::Paused && refs == 5 && contains(msg, "no quedo completo"),
              "siempre MOVE: PAUSA despues de 5 movimientos");
        World lejos = w;
        lejos.plate_approach_tcp = Eigen::Vector3d(0.34, -0.10, 0.32);   // 35 cm de ARRIBA_PLACA1_NEW
        r = run(lejos, msg, asks, refs, ref);
        check(r.first == task::Status::Paused && refs == 0 && contains(msg, "centrado rechazado"),
              "pose a 35 cm de la pose del paso: PAUSA sin moverse");
        World tilted = w;
        tilted.plate_approach_rpy = Eigen::Vector3d(150.0, 0.0, 90.0);
        r = run(tilted, msg, asks, refs, ref);
        check(r.first == task::Status::Paused && refs == 0 && contains(msg, "no mira hacia abajo"),
              "camara inclinada 30 grados: PAUSA sin moverse");
        World inview = w;
        inview.approach_moves = 0;   // IN VIEW desde la primera foto, pero lejos del centrado
        r = run(inview, msg, asks, refs, ref);
        check(r.second == 1 && asks == 2 && refs == 1 && contains(msg, "centrado"),
              "IN VIEW desde lejos: se centra (1 movimiento), otra foto lo verifica y termina");
        World drift = inview;
        drift.approach_drift = Eigen::Vector3d(-0.03, 0.0, 0.0);   // cada foto 3 cm mas alla (hacia la pose del paso)
        r = run(drift, msg, asks, refs, ref);
        check(r.second == 1 && asks == 6 && refs == 5 && contains(msg, "quedo a"),
              "IN VIEW pero nunca centrado: 5 movimientos y sigue avisando la distancia");
    }

    std::cout << "\n21. Pausa del operador (boton Pausa)\n";
    {
        task::TaskParams t = definedTask();
        t.confirm_each_step = false;
        task::Sequencer seq(t);
        World w;
        double time = 0.0;
        auto run = [&](double until, bool first_confirm) {
            for (; time < until && !seq.finished(); time += dt)
            {
                w.apply(seq.update(w.inputs(dt, time, true, first_confirm && time < dt)), time);
                w.step(dt);
            }
        };
        run(0.0 + dt, true);
        while (!(seq.stepIndex() == 2 && seq.stepTime() > 1.0) && time < 120.0)
            run(time + dt, false);
        const Eigen::Vector3d at = w.p;
        task::Inputs in = w.inputs(dt, time, true, false);
        in.pause = true;
        task::Commands c = seq.update(in);
        w.apply(c, time);
        check(seq.status() == task::Status::Paused && seq.stepIndex() == 2 && contains(seq.message(), "operador"),
              "a media marcha: PAUSA en el mismo paso");
        check(c.set_reference && (c.position - at).norm() < 1e-12,
              "la referencia queda en la pose medida (el brazo se queda donde esta)");
        run(time + 10.0, false);
        check(seq.status() == task::Status::Paused && seq.stepIndex() == 2 && (w.p - at).norm() < 0.015,
              "sin boton sigue en pausa y el robot no se va (" + std::to_string((w.p - at).norm() * 100.0) + " cm)");
        c = seq.update(w.inputs(dt, time, true, true));
        w.apply(c, time);
        const task::InterfaceTarget goal = task::toInterface(*task::findPose(t, "placa_arriba"), t.tcp_offset_m);
        check(seq.status() == task::Status::Running && seq.stepIndex() == 2 && c.set_reference &&
                  (c.position - goal.position).norm() < 1e-9,
              "'Siguiente paso' REANUDA el paso (no lo da por terminado): vuelve a su objetivo");
        run(time + 600.0, false);
        check(seq.status() == task::Status::Done, "despues la secuencia termina");

        task::TaskParams t2 = definedTask();
        t2.confirm_each_step = true;
        task::Sequencer seq2(t2);
        World w2;
        time = 0.0;
        for (; time < 60.0 && !(seq2.stepIndex() == 1 && seq2.status() == task::Status::WaitingConfirm); time += dt)
        {
            w2.apply(seq2.update(w2.inputs(dt, time, true, seq2.stepIndex() == 0 &&
                                                               seq2.status() == task::Status::WaitingConfirm)), time);
            w2.step(dt);
        }
        in = w2.inputs(dt, time, true, false);
        in.pause = true;
        c = seq2.update(in);
        check(seq2.status() == task::Status::Paused && !c.set_reference && seq2.stepIndex() == 1,
              "esperando el boton: PAUSA sin tocar la referencia");
        c = seq2.update(w2.inputs(dt, time, true, true));
        check(seq2.status() == task::Status::Running && seq2.stepIndex() == 1 && c.set_gripper,
              "'Siguiente paso' arranca ese mismo paso");
        in = w2.inputs(dt, time, false, false);
        in.pause = true;
        seq2.update(in);
        check(seq2.status() == task::Status::Paused, "tambien pausa con el control apagado (slider OFF)");
    }

    std::cout << "\n22. Zona baja (mesa de las placas mas baja que la del robot)\n";
    {
        const task::TaskParams lab = project_config::phase2Task(project_config::Secuencia::Completa);
        check(!lab.low_zone_enabled && !task::inWorkspace(lab, Eigen::Vector3d(0.24, -0.60, -0.15)),
              "configuracion del lab (robot en su lugar): zona baja apagada");
        // la del 2026-10-27 con el robot movido
        task::TaskParams t = lab;
        t.low_zone_enabled = true;
        t.low_zone_min_m = Eigen::Vector3d(0.00, -0.70, -0.18);
        t.low_zone_max_m = Eigen::Vector3d(0.75, -0.30, 0.65);
        check(task::validate(t).empty(), "con la zona baja encendida la configuracion es valida");
        check(task::inWorkspace(t, Eigen::Vector3d(0.24, -0.60, -0.15)),
              "placa en la mesa baja (0.24, -0.60, -0.15): dentro");
        check(task::inWorkspace(t, Eigen::Vector3d(0.21, -0.60, 0.03)), "centrado sobre la placa (z 0.03): dentro");
        check(!task::inWorkspace(t, Eigen::Vector3d(0.40, 0.00, 0.00)),
              "z 0.00 sobre la mesa del robot (y 0.00): FUERA (piso 0.06)");
        check(!task::inWorkspace(t, Eigen::Vector3d(0.30, -0.25, -0.10)),
              "z -0.10 en el borde de la mesa del robot (y -0.25): FUERA");
        check(!task::inWorkspace(t, Eigen::Vector3d(0.24, -0.60, -0.19)), "1 cm sobre la mesa baja: FUERA");
        check(!task::inWorkspace(t, Eigen::Vector3d(0.24, std::nan(""), 0.2)), "NaN: FUERA");
        task::TaskParams off = t;
        off.low_zone_enabled = false;
        check(!task::inWorkspace(off, Eigen::Vector3d(0.24, -0.60, -0.15)), "zona baja apagada: la placa queda FUERA");
        task::TaskParams bad = t;
        bad.low_zone_min_m.z() = 0.9;
        check(contains(task::validate(bad), "low_zone_min_m"), "zona baja mal definida: error de validacion");
    }

    std::cout << "\n" << (failures == 0 ? "TODAS LAS PRUEBAS PASARON" : "HAY PRUEBAS FALLIDAS")
              << " (" << failures << " fallas)\n";
    return failures == 0 ? 0 : 1;
}
