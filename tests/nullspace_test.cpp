// Prueba del espacio nulo (postura) del controlador, SIN robot.
//
// Usa el Jacobiano del modelo (Gen3_model.h) en posturas reales del CSV
// 2026-10-14 12-54-33 y revisa:
//   1. tau_null no empuja la herramienta: J * tau_null ~ 0.
//   2. Con peso 0 una articulacion no jala hacia su referencia.
//   3. El resorte de J3/J5 regresa el codo (signo correcto).
//   4. El tope escala todo el vector (misma direccion).
//   5. Apagado (K = 0, D = 0) -> tau_null = 0.
//   6. J3 es de giro continuo: 359 deg vs 1 deg es un error de 2 deg.
//   7. Las articulaciones de cada pose del lab (phase2Task) dan, con el
//      modelo + pinza, su pose cartesiana (atrapa dedazos como J3 = 377.3).
//   8. Limite suave de J4: solo debajo del limite, solo empuja hacia doblar,
//      tope, sin escalon al entrar, y se suma a tau_impedance.
//   9. Simulacion del paso 11 (CSV 15-32-11): desde el brazo estirado
//      (J4 = -2) hacia OFFSET_GENERAL, con y sin el limite.

#include "cartesian_impedance_controller.hpp"
#include "controller_config.hpp"
#include "gen3_adapter.hpp"
#include "task_sequence.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using impedance::CartesianImpedanceController;
using Vec7 = CartesianImpedanceController::Vec7;
using Mat67 = CartesianImpedanceController::Mat67;

static int failures = 0;

static void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? " OK " : "FALLA", what);
    if (!ok)
        ++failures;
}

static Vec7 deg(double a, double b, double c, double d, double e, double f, double g)
{
    Vec7 q;
    q << a, b, c, d, e, f, g;
    return q * (M_PI / 180.0);
}

static Mat67 jac(const Vec7& q)
{
    return gen3_adapter::jacobian(gen3_adapter::toMini(q));
}

// Controlador con solo espacio nulo (rigidez cartesiana 0), sin filtros.
static CartesianImpedanceController make(double k_null, double d_factor, const Vec7& w, double max_tau)
{
    CartesianImpedanceController c;
    c.setCartesianStiffness(CartesianImpedanceController::Vec6::Zero());
    c.setNullspaceStiffness(k_null);
    c.setDampingFactors(CartesianImpedanceController::Vec6::Ones(), d_factor);
    c.setNullspaceWeights(w);
    c.setNullspaceMaxTorque(max_tau);
    c.setFiltering(1000.0, 1.0, 1.0, 1.0, 1.0);
    return c;
}

static Vec7 tauNull(CartesianImpedanceController& c, const Vec7& q, const Vec7& q_ref, const Vec7& dq)
{
    c.initialize(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), q);
    c.setNullspaceReference(q_ref);
    return c.compute(q, dq, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), jac(q)).tau_null;
}

int main()
{
    Vec7 w_elbow;
    w_elbow << 0, 0, 1, 0, 1, 0, 0;

    const Vec7 home = deg(-5, 31, 2, 51, -1, 97, -3);
    const Vec7 postures[] = {
        home,
        deg(-62, 47, 38, 36, -27, 102, -40),   // paso 5, codo ya girado
        deg(-59, 57, 65, 19, -56, 113, -40),   // paso 11
        deg(18, 55, 61, -39, -90, 128, -30),   // paso 14, codo volteado
    };

    std::printf("1. tau_null no empuja la herramienta (J * tau_null ~ 0)\n");
    for (const Vec7& q : postures)
    {
        auto c = make(20.0, 1.0, w_elbow, 0.0);
        Vec7 dq = deg(3, -2, 10, 1, -8, 2, 4);
        Vec7 tn = tauNull(c, q, home, dq);
        double push = (jac(q) * tn).norm();
        char msg[160];
        std::snprintf(msg, sizeof msg, "|tau_null| %.2f Nm, |J*tau_null| %.1e", tn.norm(), push);
        check(tn.norm() > 0.1 && push < 1e-6 * (1 + tn.norm()), msg);
    }

    std::printf("2. Peso 0 en J1: error solo en J1 no genera torque\n");
    {
        auto c = make(20.0, 1.0, w_elbow, 0.0);
        Vec7 q_ref = home;
        q_ref(0) += 0.8;
        Vec7 tn = tauNull(c, home, q_ref, Vec7::Zero());
        check(tn.norm() < 1e-9, "tau_null = 0");
    }

    std::printf("3. Codo girado (J3 +30 deg) -> el resorte lo regresa\n");
    {
        auto c = make(20.0, 1.0, w_elbow, 0.0);
        Vec7 q = home;
        q(2) += 30 * M_PI / 180;
        Vec7 tn = tauNull(c, q, home, Vec7::Zero());
        // Potencia del resorte sobre el error: tau . (q_ref - q) > 0
        check(tn(2) < 0 && tn.dot(home - q) > 0, "empuja J3 hacia la postura");
    }

    std::printf("4. Tope de torque escala todo el vector\n");
    {
        Vec7 q = home;
        q(2) += 60 * M_PI / 180;
        auto c_free = make(200.0, 0.0, w_elbow, 0.0);
        auto c_cap = make(200.0, 0.0, w_elbow, 5.0);
        Vec7 a = tauNull(c_free, q, home, Vec7::Zero());
        Vec7 b = tauNull(c_cap, q, home, Vec7::Zero());
        double cosang = a.dot(b) / (a.norm() * b.norm());
        char msg[160];
        std::snprintf(msg, sizeof msg, "sin tope max %.1f Nm, con tope max %.2f Nm, cos %.6f",
                      a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff(), cosang);
        check(a.cwiseAbs().maxCoeff() > 5.0 && std::abs(b.cwiseAbs().maxCoeff() - 5.0) < 1e-9 &&
                  cosang > 1 - 1e-12,
              msg);
    }

    std::printf("5. Apagado (K = 0, D = 0) -> tau_null = 0\n");
    {
        auto c = make(0.0, 0.0, w_elbow, 8.0);
        Vec7 q = home;
        q(2) += 0.5;
        Vec7 tn = tauNull(c, q, home, deg(5, 5, 5, 5, 5, 5, 5));
        check(tn.norm() == 0.0, "tau_null = 0");
    }

    std::printf("6. J3 continua: 359 deg vs 1 deg = 2 deg de error\n");
    {
        Vec7 q_ref = home;
        q_ref(2) = 1 * M_PI / 180;
        Vec7 q = home;
        q(2) = 359 * M_PI / 180;
        Vec7 q_near = home;
        q_near(2) = -1 * M_PI / 180;
        auto c1 = make(20.0, 0.0, w_elbow, 0.0);
        auto c2 = make(20.0, 0.0, w_elbow, 0.0);
        Vec7 a = tauNull(c1, q, q_ref, Vec7::Zero());
        Vec7 b = tauNull(c2, q_near, q_ref, Vec7::Zero());
        check((a - b).norm() < 1e-6, "mismo torque que con -1 deg");
    }

    std::printf("7. Articulaciones de cada pose vs su pose cartesiana (modelo)\n");
    {
        // El modelo da la brida; la app web, el TCP 12.4 cm mas abajo sobre
        // el eje z de la herramienta.
        constexpr double GRIPPER_LENGTH_M = 0.124;
        const task::TaskParams t = project_config::ControllerConfig{}.task;
        for (const auto& pose : t.poses)
        {
            if (!pose.hasJoints())
                continue;
            const auto fk = gen3_adapter::pose(gen3_adapter::toMini(task::jointsToRad(pose.joints_deg)));
            const Eigen::Vector3d tcp = fk.position + fk.orientation * Eigen::Vector3d(0, 0, GRIPPER_LENGTH_M);
            const double dpos = (tcp - pose.tcp_position_m).norm();
            const double dang = fk.orientation.angularDistance(task::rpyToQuaternion(pose.rpy_deg)) * 180 / M_PI;
            char msg[200];
            std::snprintf(msg, sizeof msg, "%-26s %4.1f cm  %4.1f grados", pose.name.c_str(), dpos * 100, dang);
            check(dpos < 0.07 && dang < 7.0, msg);
        }
    }

    std::printf("8. Limite suave de J4\n");
    {
        CartesianImpedanceController::JointSoftLimit lim;
        lim.enabled = true;
        lim.joint = 3;
        lim.min_rad = 15 * M_PI / 180;
        lim.stiffness = 20.0;
        lim.damping = 9.0;
        lim.max_torque = 8.0;

        auto limitAt = [&](double j4_deg, double dq4, bool enabled = true)
        {
            auto c = make(0.0, 0.0, w_elbow, 0.0);
            CartesianImpedanceController::JointSoftLimit l = lim;
            l.enabled = enabled;
            c.setJointSoftLimit(l);
            c.setMaxTorqueDelta(100.0);
            Vec7 q = home;
            q(3) = j4_deg * M_PI / 180;
            Vec7 dq = Vec7::Zero();
            dq(3) = dq4;
            c.initialize(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), q);
            return c.compute(q, dq, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), jac(q));
        };

        check(limitAt(20, 0).tau_limit.norm() == 0.0, "J4 = 20 deg (arriba del limite): 0");
        auto o10 = limitAt(10, 0);
        check(std::abs(o10.tau_limit(3) - 20.0 * 5 * M_PI / 180) < 1e-9 &&
                  o10.tau_limit.norm() == std::abs(o10.tau_limit(3)),
              "J4 = 10 deg: 20 Nm/rad * 5 deg = 1.75 Nm, solo en J4");
        check(std::abs(limitAt(-30, 0).tau_limit(3) - 8.0) < 1e-12, "J4 = -30 deg: tope 8 Nm");
        check(limitAt(14.99, -0.5).tau_limit(3) < 0.05,
              "entrando rapido justo en el limite: casi 0 (sin escalon)");
        check(limitAt(10, -0.5).tau_limit(3) > o10.tau_limit(3),
              "entrando (dq < 0): el amortiguamiento empuja mas");
        check(limitAt(14, 3.0).tau_limit(3) == 0.0, "saliendo rapido: nunca jala hacia el limite");
        check(limitAt(-30, 0, false).tau_limit.norm() == 0.0, "apagado: 0");
        auto o = limitAt(10, 0);
        check(std::abs(o.tau_impedance(3) - o.tau_limit(3)) < 1e-12, "se suma a tau_impedance");
    }

    std::printf("9. Simulacion paso 11: brazo estirado -> OFFSET_GENERAL\n");
    {
        // Modelo simple (el modelo no trae matriz de masas): gravedad
        // compensada, sin friccion seca, inercia diagonal aproximada por
        // articulacion y friccion viscosa b. Ganancias actuales de la config.
        // Solo para ver hacia donde jala el controlador.
        const project_config::ControllerConfig cfg{};
        const task::TaskPose* og = task::findPose(cfg.task, "OFFSET_GENERAL");
        const task::InterfaceTarget target = task::toInterface(*og, cfg.task.tcp_offset_m);
        const Vec7 stuck = deg(-51, 61, -5, -2, 5, 122, -52);
        Vec7 inertia;
        inertia << 1.0, 1.5, 0.5, 0.5, 0.1, 0.1, 0.05;   // kg m^2
        constexpr double b = 1.0, dt = 0.001, T = 60.0;

        for (bool with_limit : {false, true})
        {
            CartesianImpedanceController c;
            c.setCartesianStiffness(cfg.stiffness);
            c.setNullspaceStiffness(cfg.nullspace_stiffness);
            c.setDampingFactors(cfg.damping_factors, cfg.nullspace_damping_factor);
            c.setNullspaceWeights(cfg.nullspace_weights);
            c.setNullspaceMaxTorque(cfg.nullspace_max_torque);
            c.setFiltering(1000.0, 1.0, 1.0, 1.0, 1.0);
            c.setMaxTorqueDelta(cfg.delta_tau_max);
            if (with_limit)
            {
                CartesianImpedanceController::JointSoftLimit l;
                l.enabled = true;
                l.joint = 3;
                l.min_rad = cfg.j4_min_deg * M_PI / 180;
                l.stiffness = cfg.j4_limit_stiffness;
                l.damping = cfg.j4_limit_damping;
                l.max_torque = cfg.j4_limit_max_torque;
                c.setJointSoftLimit(l);
            }

            Vec7 q = stuck;
            c.initialize(target.position, target.orientation, q);
            c.setNullspaceReference(task::jointsToRad(og->joints_deg));
            Vec7 dq = Vec7::Zero();
            for (double t = 0; t < T; t += dt)
            {
                const auto f = gen3_adapter::pose(gen3_adapter::toMini(q));
                const auto out = c.compute(q, dq, f.position, f.orientation, jac(q));
                dq += dt * (out.tau_impedance - b * dq).cwiseQuotient(inertia);
                q += dq * dt;
            }
            const auto f = gen3_adapter::pose(gen3_adapter::toMini(q));
            const double err = (f.position - target.position).norm();
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac(q));
            char msg[200];
            std::snprintf(msg, sizeof msg,
                          "%s: tras %.0f s J4 = %5.1f deg, error %4.1f cm, sing. %.3f",
                          with_limit ? "CON limite" : "SIN limite", T, q(3) * 180 / M_PI, err * 100,
                          svd.singularValues()(5));
            if (with_limit)
                check(q(3) * 180 / M_PI > cfg.j4_min_deg - 1.0 && err < 0.01, msg);
            else
                std::printf("  [info] %s\n", msg);
        }
    }

    std::printf("\n10. Rampa de la referencia (linea recta, velocidad constante)\n");
    {
        // Robot quieto en p0: la referencia interna es p - pose_error.
        CartesianImpedanceController c;
        c.setFiltering(1000.0, 1.0, 0.1, 0.1, 0.1);
        c.setReferenceSpeedLimit(0.05, 15.0 * M_PI / 180.0);
        const Vec7 q = deg(0, 30, 0, 60, 0, 90, 0);
        const Mat67 J = jac(q);
        const Eigen::Vector3d p0(0.5, 0.0, 0.4);
        const Eigen::Quaterniond R0 = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d p1 = p0 + Eigen::Vector3d(0.24, -0.18, 0.0);   // 30 cm
        const Eigen::Quaterniond R1(Eigen::AngleAxisd(30.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
        c.initialize(p0, R0, q);
        c.setReferencePose(p1, R1);

        const Vec7 dq = Vec7::Zero();
        double max_off_line = 0.0, d3 = 0.0, a3 = 0.0, d59 = 0.0, d61 = 0.0, a61 = 0.0;
        for (int k = 1; k <= 7000; ++k)
        {
            const auto o = c.compute(q, dq, p0, R0, J);
            const Eigen::Vector3d pr = p0 - o.pose_error.head<3>();
            const Eigen::Vector3d u = (p1 - p0).normalized();
            max_off_line = std::max(max_off_line, ((pr - p0) - u * u.dot(pr - p0)).norm());
            const double ang = o.pose_error.tail<3>().norm() * 180.0 / M_PI;
            if (k == 3000) { d3 = (pr - p0).norm(); a3 = ang; }
            if (k == 5900) d59 = (p1 - pr).norm();
            if (k == 6100) { d61 = (p1 - pr).norm(); a61 = std::abs(ang - 30.0); }
        }
        char msg[160];
        std::snprintf(msg, sizeof msg, "a los 3 s va en %.1f cm y %.1f grados (mitad: 15 cm, 15 grados)",
                      d3 * 100, a3);
        check(std::abs(d3 - 0.15) < 1e-3 && std::abs(a3 - 15.0) < 0.1, msg);
        check(d59 > 1e-3 && d61 < 1e-9 && a61 < 1e-6, "llega justo a los 6 s (30 cm / 5 cm/s) y se detiene");
        check(max_off_line < 1e-9, "va en linea recta");

        CartesianImpedanceController off;
        off.setFiltering(1000.0, 1.0, 0.1, 0.1, 0.1);
        off.initialize(p0, R0, q);
        off.setReferencePose(p1, R1);
        const auto o = off.compute(q, dq, p0, R0, J);
        check((p0 - o.pose_error.head<3>() - p1).norm() < 1e-12, "sin rampa (0) la referencia salta como antes");
    }

    std::printf("\n11. Rigidez cerca del objetivo (2026-10-21)\n");
    {
        // La funcion con valores fijos de prueba (base 40, cerca 60, de 8 a
        // 5 cm); el config solo se revisa que tenga sentido (el usuario lo
        // tunea).
        using Vec6 = CartesianImpedanceController::Vec6;
        const project_config::ControllerConfig cfg{};
        check(cfg.near_target_stiffness >= 0.0 && cfg.near_target_full_m >= 0.0 &&
                  cfg.near_target_start_m > cfg.near_target_full_m,
              "config: cerca >= 0 y 0 <= full_m < start_m");
        Vec6 base;
        base << 40, 40, 42, 10, 10, 10;
        auto k = [&](double d) {
            return CartesianImpedanceController::nearTargetStiffness(base, 60.0, 0.08, 0.05, d);
        };
        check(k(0.30) == base && k(0.08) == base, "lejos (30 cm, 8 cm): la de siempre");
        const Vec6 mid = k(0.065);
        check(std::abs(mid(0) - 50.0) < 1e-9 && std::abs(mid(2) - 51.0) < 1e-9, "6.5 cm: a la mitad");
        check(k(0.05).head<3>() == Eigen::Vector3d::Constant(60.0) &&
              k(0.0).head<3>() == Eigen::Vector3d::Constant(60.0), "5 cm y 0 cm: 60 en x, y, z");
        check(k(0.0).tail<3>() == base.tail<3>(), "la rotacion no cambia");
        check(k(std::nan("")) == base, "distancia NaN: la de siempre");
        check(CartesianImpedanceController::nearTargetStiffness(base, 20.0, 0.08, 0.05, 0.0) == base,
              "nunca baja de la de siempre");
        double prev = base(0), jump = 0.0;
        for (double d = 0.10; d >= 0.0; d -= 0.0001)
        {
            jump = std::max(jump, std::abs(k(d)(0) - prev));
            prev = k(d)(0);
        }
        check(jump < 0.1, "sin escalon: cada 0.1 mm cambia menos de 0.1 N/m");

        // setCartesianStiffnessNow: entra en este ciclo aunque stiffness_filter
        // sea lento (0.10 = ~10 s); setCartesianStiffness se queda filtrado.
        const Vec7 q = home;
        const Mat67 J = jac(q);
        const Eigen::Vector3d p0(0.5, 0.0, 0.3);
        const Eigen::Vector3d e(0.03, 0.0, 0.0);
        auto spring = [&](bool now) {
            CartesianImpedanceController c;
            c.setCartesianStiffness(base);
            c.setFiltering(1000.0, 1.0, 0.10, 1.0, 1.0);
            c.initialize(p0, Eigen::Quaterniond::Identity(), q);
            if (now)
                c.setCartesianStiffnessNow(k(0.03));
            else
                c.setCartesianStiffness(k(0.03));
            return c.compute(q, Vec7::Zero(), p0 + e, Eigen::Quaterniond::Identity(), J).tau_spring;
        };
        Vec6 f60 = Vec6::Zero(), f40 = Vec6::Zero();
        f60(0) = -60.0 * e(0);
        f40(0) = -base(0) * e(0);
        check((spring(true) - J.transpose() * f60).norm() < 1e-9, "Now: con 3 cm empuja con 60 N/m de inmediato");
        check((spring(false) - J.transpose() * f40).norm() < 1e-3, "normal: sigue en la de antes (filtrada)");
    }

    std::printf("\n12. Sin compensacion de friccion en el objetivo (2026-10-22)\n");
    {
        const project_config::ControllerConfig cfg{};
        check(cfg.enable_friction_off_near_target && cfg.friction_off_within_m >= 0.0 &&
                  cfg.friction_off_fade_m >= 0.0,
              "config: encendido, distancias >= 0");
        auto f = [](double d) { return project_config::frictionScaleNearTarget(d, 0.005, 0.002); };
        check(f(0.0) == 0.0 && f(0.003) == 0.0 && f(0.005) == 0.0, "0, 3 y 5 mm: compensacion 0");
        check(std::abs(f(0.006) - 0.5) < 1e-9, "6 mm: 50 %");
        check(f(0.007) == 1.0 && f(0.05) == 1.0 && f(0.30) == 1.0, "7 mm, 5 cm, 30 cm: 100 % (igual que antes)");
        check(f(std::nan("")) == 1.0, "distancia NaN: 100 % (igual que antes)");
        check(project_config::frictionScaleNearTarget(0.005, 0.005, 0.0) == 0.0 &&
                  project_config::frictionScaleNearTarget(0.0051, 0.005, 0.0) == 1.0,
              "rampa 0: escalon justo en 5 mm");
    }

    std::printf("\n13. Solo gravedad en el objetivo (2026-10-27)\n");
    {
        const project_config::ControllerConfig cfg{};
        check(cfg.enable_gravity_only_near_target && cfg.gravity_only_within_m == 0.005 &&
                  cfg.gravity_only_fade_m == 0.002 && cfg.gravity_only_within_deg == 2.0 &&
                  cfg.gravity_only_fade_deg == 1.0,
              "config: encendido, 5 mm + 2 mm de rampa, 2 grados + 1 de rampa");
        auto c = [&](double d, double a) {
            return project_config::controlScaleNearTarget(d, a, cfg.gravity_only_within_m, cfg.gravity_only_fade_m,
                                                          cfg.gravity_only_within_deg, cfg.gravity_only_fade_deg);
        };
        check(c(0.0, 0.0) == 0.0 && c(0.005, 2.0) == 0.0, "en el objetivo (<= 5 mm y <= 2 grados): 0 = solo gravedad");
        check(std::abs(c(0.006, 0.0) - 0.5) < 1e-9, "6 mm bien orientado: 50 %");
        check(c(0.007, 0.0) == 1.0 && c(0.05, 0.0) == 1.0, "7 mm y 5 cm: control completo");
        check(c(0.002, 3.0) == 1.0 && std::abs(c(0.002, 2.5) - 0.5) < 1e-9,
              "a 2 mm pero girado 3 grados: control completo (2.5 grados: 50 %)");
        check(c(std::nan(""), 0.0) == 1.0 && c(0.0, std::nan("")) == 1.0, "NaN: control completo");
        double jump = 0.0;
        for (double d = 0.0; d < 0.02; d += 1e-5)
            jump = std::max(jump, std::abs(c(d + 1e-5, 0.0) - c(d, 0.0)));
        check(jump <= 0.0051, "sin escalon: cada 0.01 mm cambia a lo mas 0.5 %");
    }

    std::printf("\n%s (%d fallas)\n", failures ? "HAY FALLAS" : "TODO OK", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
