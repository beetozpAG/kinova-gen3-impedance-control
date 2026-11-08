#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtk/gtk.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <boost/asio/io_context.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>

#include <data_logger.h>

#include "Matrix.h"
#include "Gen3_model.h"
#include "kinova_robot.h"
#include "udp_server.h"

#include "cartesian_impedance_controller.hpp"
#include "controller_config.hpp"
#include "grasp_receiver.hpp"
#include "fixture_client.hpp"
#include "screw_vision_client.hpp"
#include "task_sequence.hpp"


// ============================================================
// NAMESPACES / TYPES
// ============================================================

using namespace matrix;
using namespace KinovaGen3;

using kinova_robot::KinovaRobotLowLevel;
using impedance::CartesianImpedanceController;
using project_config::CartesianReferenceMode;

using Vec7E = CartesianImpedanceController::Vec7;
using Vec6E = CartesianImpedanceController::Vec6;
using Mat67E = CartesianImpedanceController::Mat67;

using data_logger::DataLogger;
using data_logger::field;
using named_tuple::operator""_nm;




constexpr bool ENABLE_IMPEDANCE_HARDWARE = true;

constexpr const char* ROBOT_IP = "192.168.1.10";
constexpr const char* USERNAME = "admin";
constexpr const char* PASSWORD = "admin";

constexpr float PI = 3.14159265358979323846f;



const project_config::ControllerConfig cfg{};


const char* cartesianReferenceModeName(CartesianReferenceMode mode)
{
    switch (mode)
    {
        case CartesianReferenceMode::InitialPose:
            return "INITIAL POSE";
        case CartesianReferenceMode::RelativeOffset:
            return "RELATIVE OFFSET";
        case CartesianReferenceMode::AbsolutePose:
            return "ABSOLUTE POSE";
    }

    return "UNKNOWN";
}


template <typename T, size_t N, size_t M>
struct data_logger::csv::csv_traits<matrix::Matrix<T, N, M>>
    : csv_traits<std::array<std::array<T, M>, N>>
{
    template <size_t I>
        requires(I < N)
    static constexpr decltype(auto)
    get(const matrix::Matrix<T, N, M>& mat)
    {
        return std::get<I>(mat.data);
    }
};


// LOGGER


using ExperimentLogger =
    DataLogger<
        field<float,  "t">,

        field<vec<7>, "q">,
        field<vec<7>, "dq">,

        field<vec<7>, "pose">,
        field<vec<6>, "pose_err">,
        field<vec<6>, "cart_vel">,

        field<vec<7>, "tau_task">,
        field<vec<7>, "tau_null">,
        field<vec<7>, "tau_ext">,
        field<vec<7>, "tau_imp">,

        field<vec<7>, "tau_g">,
        field<vec<7>, "tau_virtual">,
        field<vec<7>, "tau_hw">,

        field<vec<7>, "current_virtual">,
        field<vec<7>, "current_cmd">,
        field<vec<7>, "current_meas">,
        field<vec<7>, "tau_meas">,

        field<vec<7>, "tau_spring">,
        field<vec<7>, "friction_FF">,
        field<vec<7>, "tau_friction">,

        field<vec<7>, "ref_target">,

        // VISION (GPD), solo imprimir:
        // grasp_status: 0 = sin agarres, 1 = ultimo aceptado, -1 = ultimo rechazado
        field<float,  "grasp_status">,
        field<float,  "grasp_score">,
        field<vec<7>, "grasp_base">,        // ultimo agarre valido, en la base
        field<vec<7>, "grasp_pregrasp">,    // pose de pre-agarre de la INTERFAZ

        // SECUENCIA DE TAREA (0 si esta desactivada):
        // task_step: paso actual 1..N (N+1 = terminada)
        // task_status: 0 esperando boton, 1 ejecutando, 2 pausada,
        //              3 terminada, 4 abortada
        field<float,  "task_step">,
        field<float,  "task_status">,
        field<float,  "gripper_pos">,       // posicion medida de la pinza [%]
        field<float,  "fixture_q">,         // angulo medido del fixture [grados]

        // Limite suave de J4: torque que empuja a doblar el codo [Nm]
        field<float,  "tau_j4_limit">,

        // Rigidez de traslacion en x usada este ciclo [N/m] (sube cerca del
        // objetivo, enable_near_target_stiffness)
        field<float,  "k_trans">,

        // Fraccion de la compensacion de friccion usada (0 en el objetivo,
        // enable_friction_off_near_target; 1 = completa)
        field<float,  "friction_scale">,

        // Fraccion de TODA la accion de control (0 = solo gravedad en el
        // objetivo, enable_gravity_only_near_target; 1 = completa)
        field<float,  "control_scale">
    >;

ExperimentLogger logger;



volatile std::sig_atomic_t stop_requested = 0;
std::atomic_bool impedance_enabled{false};
// Boton "Siguiente paso" de la GUI (secuencia de tarea).
std::atomic_bool task_confirm_requested{false};
std::atomic_bool task_pause_requested{false};   // boton "Pausa" (GTK o GUI)
// Modo de la secuencia pedido por la GUI: -1 nada, 1 AUTOMATICO, 0 PASO A PASO.
std::atomic_int task_mode_requested{-1};
std::mutex gui_dashboard_mutex;
std::string gui_dashboard_text = "Control OFF - gravity compensation active";

void signalHandler(int)
{
    stop_requested = 1;
}


Vec7E toEigen7(const vec<7>& v)
{
    Vec7E result;

    for (size_t i = 0; i < 7; ++i)
    {
        result(static_cast<Eigen::Index>(i)) =
            static_cast<double>(v[i][0]);
    }

    return result;
}


Mat67E toEigenJacobian(const mat<6, 7>& J)
{
    Mat67E result;

    for (size_t r = 0; r < 6; ++r)
    {
        for (size_t c = 0; c < 7; ++c)
        {
            result(
                static_cast<Eigen::Index>(r),
                static_cast<Eigen::Index>(c)
            ) =
                static_cast<double>(J[r][c]);
        }
    }

    return result;
}


vec<7> eigenToVec7(const Vec7E& v)
{
    vec<7> result{};

    for (size_t i = 0; i < 7; ++i)
    {
        result[i][0] =
            static_cast<float>(
                v(static_cast<Eigen::Index>(i))
            );
    }

    return result;
}


vec<6> eigenToVec6(const Vec6E& v)
{
    vec<6> result{};

    for (size_t i = 0; i < 6; ++i)
    {
        result[i][0] =
            static_cast<float>(
                v(static_cast<Eigen::Index>(i))
            );
    }

    return result;
}


Eigen::Vector3d getPosition(const vec<7>& pose)
{
    return Eigen::Vector3d(
        pose[0][0],
        pose[1][0],
        pose[2][0]
    );
}


Eigen::Quaterniond getOrientation(const vec<7>& pose)
{
    // GetPose format:
    // [x y z qw qx qy qz]

    Eigen::Quaterniond q(
        pose[3][0],
        pose[4][0],
        pose[5][0],
        pose[6][0]
    );

    if (!q.coeffs().allFinite())
    {
        throw std::runtime_error(
            "GetPose returned non-finite quaternion"
        );
    }

    if (q.norm() < 1e-10)
    {
        throw std::runtime_error(
            "GetPose returned zero quaternion"
        );
    }

    q.normalize();

    return q;
}



void wrapJoints(vec<7>& q)
{
    for (size_t i = 0; i < 7; ++i)
    {
        while (q[i][0] > PI)
        {
            q[i][0] -= 2.0f * PI;
        }

        while (q[i][0] < -PI)
        {
            q[i][0] += 2.0f * PI;
        }
    }
}




bool isFinite(const vec<7>& v)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (!std::isfinite(v[i][0]))
        {
            return false;
        }
    }

    return true;
}


bool isFinite(const vec<6>& v)
{
    for (size_t i = 0; i < 6; ++i)
    {
        if (!std::isfinite(v[i][0]))
        {
            return false;
        }
    }

    return true;
}



float maxAbs(const vec<7>& v)
{
    float result = 0.0f;

    for (size_t i = 0; i < 7; ++i)
    {
        result =
            std::max(
                result,
                static_cast<float>(
                    std::abs(v[i][0])
                )
            );
    }

    return result;
}


float maxAbs(const vec<6>& v)
{
    float result = 0.0f;

    for (size_t i = 0; i < 6; ++i)
    {
        result =
            std::max(
                result,
                static_cast<float>(
                    std::abs(v[i][0])
                )
            );
    }

    return result;
}



bool poseErrorWarning(const vec<6>& e)
{
    for (size_t i = 0; i < 3; ++i)
    {
        if (std::abs(e[i][0]) >
            cfg.max_shadow_position_error_m)
        {
            return true;
        }
    }

    for (size_t i = 3; i < 6; ++i)
    {
        if (std::abs(e[i][0]) >
            cfg.max_shadow_rotation_error_rad)
        {
            return true;
        }
    }

    return false;
}


bool cartVelocityWarning(const vec<6>& v)
{
    for (size_t i = 0; i < 3; ++i)
    {
        if (std::abs(v[i][0]) >
            cfg.max_shadow_linear_speed_m_s)
        {
            return true;
        }
    }

    for (size_t i = 3; i < 6; ++i)
    {
        if (std::abs(v[i][0]) >
            cfg.max_shadow_angular_speed_rad_s)
        {
            return true;
        }
    }

    return false;
}


bool impedanceTorqueWarning(const vec<7>& tau)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(tau[i][0]) >
            cfg.max_shadow_impedance_torque_nm)
        {
            return true;
        }
    }

    return false;
}


// PHYSICAL LIMITS

bool torqueLimitExceeded(const vec<7>& tau)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(tau[i][0]) >
            maxTau[i][0])
        {
            return true;
        }
    }

    return false;
}


bool currentLimitExceeded(const vec<7>& current)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(current[i][0]) >
            maxCurrent[i][0])
        {
            return true;
        }
    }

    return false;
}


void saturateTorque(vec<7>& tau)
{
    for (size_t i = 0; i < 7; ++i)
    {
        if (std::abs(tau[i][0]) >
            maxTau[i][0])
        {
            tau[i][0] =
                std::copysign(
                    maxTau[i][0],
                    tau[i][0]
                );
        }
    }
}


// ================= OBSERVADOR DE FRICCION =================
// Mismo codigo que FrictionObserver de main_op_space_local.cpp, que a su vez
// es el Friction_Observer de main_op_space.cpp. Por articulacion:
//   u     = | dq + (vs/sqrt2) * tanh(100 * tau^3) |
//   g(u)  = FC + (FS - FC) * exp(-(u/vs)^2)
//   dz    = u - sigma0 * |u| * z / g(u)          (RK4, u y g(u) fijos en el paso)
//   FF    = sigma0 * z_nuevo + sigma1 * dz_anterior
//   tau_f = smoothSign(tau, factor) .* |FF|      (smoothSign = tanh(factor * tau^3))
// tau es el resorte de la impedancia (tau_spring), sin amortiguamiento.
struct FrictionOutput {
    vec<7> FF{};     // Friccion estimada por el observador [Nm]
    vec<7> torque{}; // Compensacion aplicada (con signo del resorte) [Nm]
};

struct FrictionObserver {
    vec<7> z{};
    FrictionObserver() { reset(); }
    // Estado inicial del original: z = FC / sigma0 (friccion de Coulomb).
    void reset() {
        for (size_t i=0; i<7; ++i) z[i][0]=KinovaGen3::FC[i][0]/KinovaGen3::sigma0[i][0];
    }
    FrictionOutput step(const vec<7>& dq, const vec<7>& tau_ctrl, float h) {
        using namespace KinovaGen3;
        using namespace boost::numeric::odeint;
        if (!isFinite(dq)) throw std::runtime_error("dq no finito en friccion");
        if (!isFinite(tau_ctrl)) throw std::runtime_error("Torque de control no finito en friccion");
        FrictionOutput o;

        // Velocidad auxiliar u y curva de Stribeck g(u), fijas durante el paso.
        vec<7> vl{}, gv{}, z_dot{};
        for (size_t i=0; i<7; ++i) {
            const float tau=tau_ctrl[i][0];
            vl[i][0]=std::abs(dq[i][0]+(v0[i][0]/std::sqrt(2.0f))*std::tanh(100.0f*tau*tau*tau));
            gv[i][0]=FC[i][0]+(FS[i][0]-FC[i][0])*std::exp(-std::pow(vl[i][0]/v0[i][0],2.0f));
            // dz/dt = u - sigma0*|u|*z/g(u)   (antes de integrar, para el termino sigma1)
            z_dot[i][0]=vl[i][0]-sigma0[i][0]*std::abs(vl[i][0])*z[i][0]*(1.0f/gv[i][0]);
        }

        // Integracion RK4 de z con Boost.Odeint, igual que Friction_Observer original.
        runge_kutta4<std::array<vec<7>,1>, float> stepper;
        auto int_z_dot=[&](const std::array<vec<7>,1>& zs, std::array<vec<7>,1>& dzdt, const float) {
            for (size_t i=0; i<7; ++i)
                dzdt[0][i][0]=vl[i][0]-sigma0[i][0]*std::abs(vl[i][0])*zs[0][i][0]*(1.0f/gv[i][0]);
        };
        std::array<vec<7>,1> z_temp={z};
        stepper.do_step(int_z_dot, z_temp, 0.0f, h);
        z=z_temp[0];

        for (size_t i=0; i<7; ++i)
            o.FF[i][0]=sigma0[i][0]*z[i][0]+sigma1[i][0]*z_dot[i][0];
        o.torque=smoothSign(tau_ctrl,static_cast<float>(cfg.friction_sign_factor)).hadamard(abs(o.FF));
        if (!isFinite(z)) throw std::runtime_error("Estado de friccion no finito");
        if (!isFinite(o.FF)) throw std::runtime_error("Friccion estimada no finita");
        if (!isFinite(o.torque)) throw std::runtime_error("Compensacion de friccion no finita");
        return o;
    }
};


void saveLogger()
{
    logger.save_csv(
        "impedance_shadow_data.csv"
    );

    const auto t =
        std::time(nullptr);

    const auto tm =
        *std::localtime(&t);

    std::ostringstream ss;

    ss << std::put_time(
        &tm,
        "%Y-%m-%d_%H-%M-%S"
    );

    logger.save_csv(
        "impedance_shadow_data_" +
        ss.str() +
        ".csv"
    );

    std::cout
        << "\nData saved:\n"
        << "  impedance_shadow_data.csv\n"
        << "  impedance_shadow_data_"
        << ss.str()
        << ".csv\n";
}


// GRAVITY-ONLY TRANSITION

void sendGravity(
    KinovaRobotLowLevel& robot,
    int cycles)
{
    for (int cycle = 0;
         cycle < cycles &&
         !robot.timeout;
         ++cycle)
    {
        vec<7> q =
            robot.getPos();

        wrapJoints(q);

        if (!isFinite(q))
        {
            throw std::runtime_error(
                "Non-finite q during gravity transition"
            );
        }

        vec<7> gravity =
            GravityVector(q);

        if (!isFinite(gravity))
        {
            throw std::runtime_error(
                "Non-finite gravity torque"
            );
        }

        // Absolute torque barrier.
        saturateTorque(gravity);

        const vec<7> current =
            KTGR.hadamard(gravity);

        if (!isFinite(current))
        {
            throw std::runtime_error(
                "Non-finite gravity current"
            );
        }

        if (currentLimitExceeded(current))
        {
            throw std::runtime_error(
                "Gravity current exceeds maxCurrent"
            );
        }

        robot.sendCur(current);
    }
}



const char* healthString(bool warning)
{
    return warning
        ? "WARNING"
        : "OK";
}

// DASHBOARD

void printDashboard(
    double time,
    double impedance_blend,

    const vec<7>& ref_target,

    const vec<7>& pose,
    const vec<6>& pose_error,
    const vec<6>& cart_velocity,

    const vec<7>& tau_task,
    const vec<7>& tau_null,
    const vec<7>& tau_ext,
    const vec<7>& tau_imp,
    const vec<7>& tau_friction,

    const vec<7>& gravity,
    const vec<7>& tau_virtual,
    const vec<7>& tau_hw,

    const vec<7>& current_virtual,
    const vec<7>& current_cmd,
    const vec<7>& current_meas,

    bool finite_ok,
    bool pose_warning,
    bool velocity_warning,
    bool impedance_warning,
    bool virtual_tau_warning,
    bool virtual_current_warning,

    const std::string& vision_status,
    const std::string& task_status,
    const Vec6E& stiffness_now,
    bool near_target,
    double friction_scale,
    double control_scale)
{
    std::cout
        << "\033[2J\033[H";

    std::cout
        << "============================================================\n"
        << "       KINOVA GEN3 CARTESIAN IMPEDANCE SHADOW TEST\n"
        << "============================================================\n";

    if constexpr (ENABLE_IMPEDANCE_HARDWARE)
    {
        std::cout
            << " MODE : IMPEDANCE + GRAVITY HARDWARE\n";
    }
    else
    {
        std::cout
            << " MODE : SHADOW / VIRTUAL IMPEDANCE\n"
            << " HW   : GRAVITY COMPENSATION ONLY\n";
    }

    std::cout
        << "------------------------------------------------------------\n"
        << " Cartesian task          : ENABLED\n"
        << " Nullspace               : "
        << (cfg.enable_nullspace ? "ENABLED" : "DISABLED")
        << "\n"
        << " Desired wrench          : "
        << (cfg.enable_wrench ? "ENABLED" : "DISABLED")
        << "\n"
        << " Impedance -> Hardware   : "
        << (ENABLE_IMPEDANCE_HARDWARE ? "YES" : "NO")
        << "\n"
        << "------------------------------------------------------------\n"
        << std::fixed
        << std::setprecision(4)
        << " Time                    : "
        << time
        << " s\n"
        << " Hardware blend          : "
        << impedance_blend * 100.0
        << " %\n"
        << "------------------------------------------------------------\n";


    std::cout
        << "\nREFERENCE TARGET [x y z qw qx qy qz]\n"
        << ref_target
        << "\n";


    std::cout
        << "\nPOSE [x y z qw qx qy qz]\n"
        << pose
        << "\n";





    std::cout
        << "\nCARTESIAN VELOCITY J*dq\n"
        << cart_velocity
        << "\n";


    std::cout
        << "\nTAU TASK - VIRTUAL [Nm]\n"
        << tau_task
        << "\n";


    std::cout
        << "\nTAU NULLSPACE - VIRTUAL [Nm]\n"
        << tau_null
        << "\n";


    std::cout
        << "\nTAU EXTERNAL WRENCH - VIRTUAL [Nm]\n"
        << tau_ext
        << "\n";


    std::cout
        << "\nTAU IMPEDANCE TOTAL - VIRTUAL [Nm]\n"
        << "(rate-limited total controller output)\n"
        << tau_imp
        << "\n";


    std::cout
        << "\nFRICTION COMPENSATION [Nm]\n"
        << (cfg.enable_friction_compensation
                ? "(slider ON only, direction from spring torque)\n"
                : "(DISABLED)\n")
        << tau_friction
        << "\n";


    std::cout
        << "\nGRAVITY TORQUE [Nm]\n"
        << gravity
        << "\n";


    std::cout
        << "\nTOTAL TORQUE - VIRTUAL [Nm]\n"
        << "gravity + sat(tau_imp + tau_friction)\n"
        << tau_virtual
        << "\n";


    std::cout
        << "\nTORQUE ACTUALLY USED FOR HARDWARE [Nm]\n"
        << tau_hw
        << "\n";


    std::cout
        << "\nVIRTUAL CURRENT [A]\n"
        << current_virtual
        << "\n";


    std::cout
        << "\nCURRENT ACTUALLY SENT [A]\n"
        << current_cmd
        << "\n";


    std::cout
        << "\nMEASURED CURRENT [A]\n"
        << current_meas
        << "\n";


    std::cout
        << "\n------------------------------------------------------------\n"
        << " SHADOW HEALTH\n"
        << "------------------------------------------------------------\n"
        << " Finite values           : "
        << (finite_ok ? "OK" : "FAIL")
        << "\n"
        << " Pose error              : "
        << healthString(pose_warning)
        << "\n"
        << " Cartesian velocity      : "
        << healthString(velocity_warning)
        << "\n"
        << " Impedance torque        : "
        << healthString(impedance_warning)
        << "\n"
        << " Virtual torque limit    : "
        << healthString(virtual_tau_warning)
        << "\n"
        << " Virtual current limit   : "
        << healthString(virtual_current_warning)
        << "\n";


    std::cout
        << "\n------------------------------------------------------------\n"
        << " MAX VALUES\n"
        << "------------------------------------------------------------\n"
        << " Max |DeltaXi|           : "
        << maxAbs(pose_error)
        << "\n"
        << "DELTA XI [dx dy dz dRx dRy dRz]:  "
        << pose_error
        << "\n"
        << " Rigidez traslacion      : "
        << stiffness_now.head<3>().transpose()
        << " N/m"
        << (near_target ? "  (cerca del objetivo)" : "")
        << "\n"
        << " Compensacion friccion   : "
        << friction_scale * 100.0
        << " %"
        << (friction_scale < 1.0 ? "  (en el objetivo)" : "")
        << "\n"
        << " Accion de control       : "
        << control_scale * 100.0
        << " %"
        << (control_scale <= 0.0 ? "  (en el objetivo: solo gravedad)"
            : control_scale < 1.0 ? "  (llegando: rampa)" : "")
        << "\n"
        << " Max |J*dq|              : "
        << maxAbs(cart_velocity)
        << "\n"
        << " Max |tau_task|          : "
        << maxAbs(tau_task)
        << " Nm\n"
        << " Max |tau_null|          : "
        << maxAbs(tau_null)
        << " Nm\n"
        << " Max |tau_ext|           : "
        << maxAbs(tau_ext)
        << " Nm\n"
        << " Max |tau_imp|           : "
        << maxAbs(tau_imp)
        << " Nm\n"
        << " Max |tau_virtual|       : "
        << maxAbs(tau_virtual)
        << " Nm\n"
        << " Max |I_virtual|         : "
        << maxAbs(current_virtual)
        << " A\n"
        << " Max |I_HW|              : "
        << maxAbs(current_cmd)
        << " A\n";


    std::cout
        << "\n------------------------------------------------------------\n"
        << " CONTROL PATH\n"
        << "------------------------------------------------------------\n"
        << " Gravity -> Hardware     : YES\n"
        << " Task calculation        : YES\n"
        << " Nullspace calculation   : "
        << (cfg.enable_nullspace ? "YES" : "NO")
        << "\n"
        << " Wrench calculation      : "
        << (cfg.enable_wrench ? "YES" : "NO")
        << "\n"
        << " Impedance -> Hardware   : "
        << (ENABLE_IMPEDANCE_HARDWARE ? "YES" : "NO")
        << "\n"
        << " Data Logger             : YES\n"
        << "------------------------------------------------------------\n"
        << " Ctrl+C -> gravity transition -> restore modes -> save CSV\n"
        << "============================================================\n";


    std::cout
        << "\n------------------------------------------------------------\n"
        << " VISION (GPD) - PRINT ONLY\n"
        << "------------------------------------------------------------\n"
        << vision_status
        << "============================================================\n";

    if (cfg.task.enabled)
    {
        std::cout
            << "\n------------------------------------------------------------\n"
            << " TASK SEQUENCE (button 'Siguiente paso' in the GUI)\n"
            << "------------------------------------------------------------\n"
            << task_status
            << "============================================================\n";
    }

    std::cout.flush();
}



static gboolean control_set_state(GtkSwitch*, gboolean state, gpointer)
{
    impedance_enabled.store(state, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(gui_dashboard_mutex);
        gui_dashboard_text = state
            ? "Control ON - gravity + Cartesian impedance"
            : "Control OFF - gravity compensation active";
    }
    std::cout << (state ? "\nGUI: impedance control ENABLED\n" : "\nGUI: impedance control DISABLED; gravity only\n");
    return FALSE;
}


static gboolean update_label(GtkWidget* info_label)
{
    if (info_label)
    {
        std::string text;
        {
            std::lock_guard<std::mutex> lock(gui_dashboard_mutex);
            text = gui_dashboard_text;
        }
        gtk_label_set_text(GTK_LABEL(info_label), text.c_str());
    }
    return TRUE;
}


static void task_next_step(GtkWidget*, gpointer)
{
    task_confirm_requested.store(true, std::memory_order_relaxed);
    std::cout << "\nGUI: 'Siguiente paso'\n";
}


// Pausa de la secuencia (2026-10-27): el brazo se queda donde esta y
// "Siguiente paso" reanuda el paso (task::Sequencer::operatorPause).
static void task_pause(GtkWidget*, gpointer)
{
    task_pause_requested.store(true, std::memory_order_relaxed);
    std::cout << "\nGUI: 'Pausa'\n";
}


// Boton "Siguiente paso" de la interfaz grafica (gui/), por UDP en
// 127.0.0.1:GUI_COMMAND_PORT, texto "NEXT". Corre en su propio hilo, fuera
// del lazo de control, y hace EXACTAMENTE lo mismo que el boton de la
// ventana GTK (task_next_step). Ademas "AUTO" / "STEP" (2026-10-21): modo
// AUTOMATICO o PASO A PASO de la secuencia (Sequencer::setAutomatic), y
// "PAUSE" (2026-10-27): igual que el boton GTK "Pausa". Nada mas: no
// prende/apaga el control ni lo detiene.
constexpr int GUI_COMMAND_PORT = 5011;

static void guiCommandListener()
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::cout << "\nGUI: no se pudo abrir el socket de la interfaz grafica\n";
        return;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(GUI_COMMAND_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
        std::cout << "\nGUI: puerto " << GUI_COMMAND_PORT
                  << " ocupado; el boton de la interfaz grafica no funcionara\n";
        ::close(fd);
        return;
    }
    timeval tv{0, 200000};   // revisar stop_requested cada 0.2 s
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (!stop_requested)
    {
        char buf[64];
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n == 4 && std::string(buf, 4) == "NEXT")
        {
            task_next_step(nullptr, nullptr);
        }
        else if (n == 4 && std::string(buf, 4) == "AUTO")
        {
            task_mode_requested.store(1, std::memory_order_relaxed);
        }
        else if (n == 4 && std::string(buf, 4) == "STEP")
        {
            task_mode_requested.store(0, std::memory_order_relaxed);
        }
        else if (n == 5 && std::string(buf, 5) == "PAUSE")
        {
            task_pause(nullptr, nullptr);
        }
    }
    ::close(fd);
}


static void quit_application(GtkWidget*, gpointer user_data)
{
    stop_requested = 1;
    if (user_data)
        g_application_quit(G_APPLICATION(user_data));
}


static void activate(GtkApplication *app, gpointer /*user_data*/)
{
  GtkWidget *window;
  GtkWidget *grid;
  GtkWidget *controls_grid;
  GtkWidget *button;
  GtkWidget *control_switch;
  GtkWidget *label;
  GtkWidget *label_slider;

  window = gtk_application_window_new(app);
  gtk_window_set_title(GTK_WINDOW(window), "Kinova Gen3 Cartesian Impedance");
  gtk_container_set_border_width(GTK_CONTAINER(window), 10);

  grid = gtk_grid_new();
  gtk_grid_set_column_spacing(GTK_GRID(grid), 20);
  controls_grid = gtk_grid_new();
  gtk_grid_set_column_spacing(GTK_GRID(controls_grid), 20);

  gtk_container_add(GTK_CONTAINER(window), grid);

  control_switch = gtk_switch_new();
  gtk_widget_set_halign(control_switch, GTK_ALIGN_START);
  gtk_widget_set_valign(control_switch, GTK_ALIGN_START);
  gtk_widget_set_hexpand(control_switch, false);
  gtk_widget_set_vexpand(control_switch, false);
  g_signal_connect(control_switch, "state-set", G_CALLBACK(control_set_state),
                   NULL);

  label_slider = gtk_label_new(" Control Action ");
  gtk_grid_attach(GTK_GRID(controls_grid), label_slider, 1, 0, 2, 1);
  gtk_grid_attach(GTK_GRID(controls_grid), control_switch, 3, 0, 1, 1);

  label = gtk_label_new("info");
  gtk_grid_attach(GTK_GRID(grid), label, 0, 1, 3, 1);
  g_timeout_add(16, G_SOURCE_FUNC(update_label), label);

  button = gtk_button_new_with_label("Quit");
  gtk_widget_set_halign(button, GTK_ALIGN_START);
  gtk_widget_set_valign(button, GTK_ALIGN_START);
  gtk_widget_set_hexpand(button, false);
  gtk_widget_set_vexpand(button, false);
  g_signal_connect(button, "clicked", G_CALLBACK(quit_application), app);
  gtk_grid_attach(GTK_GRID(controls_grid), button, 4, 0, 1, 1);

  if (cfg.task.enabled)
  {
    GtkWidget* next_button = gtk_button_new_with_label("Siguiente paso");
    gtk_widget_set_halign(next_button, GTK_ALIGN_START);
    gtk_widget_set_valign(next_button, GTK_ALIGN_START);
    g_signal_connect(next_button, "clicked", G_CALLBACK(task_next_step), nullptr);
    gtk_grid_attach(GTK_GRID(controls_grid), next_button, 5, 0, 1, 1);

    GtkWidget* pause_button = gtk_button_new_with_label("Pausa");
    gtk_widget_set_halign(pause_button, GTK_ALIGN_START);
    gtk_widget_set_valign(pause_button, GTK_ALIGN_START);
    g_signal_connect(pause_button, "clicked", G_CALLBACK(task_pause), nullptr);
    gtk_grid_attach(GTK_GRID(controls_grid), pause_button, 6, 0, 1, 1);
  }

  gtk_grid_attach(GTK_GRID(grid), controls_grid, 0, 2, 4, 1);


  gtk_widget_show_all(window);
}







//main

// CARTESIAN REFERENCE FROM CONFIG VALUES
// Convention for the first reference (demo mode, task.enabled = false):
// Same convention for the first reference and for every extra pose:
//   InitialPose    -> initial pose (values ignored)
//   RelativeOffset -> p_d = p_0 + position (base frame),
//                     R_d = R_0 * R(rpy)   (relative to the INITIAL EE)
//   AbsolutePose   -> p_d = position, R_d = Rz(yaw)*Ry(pitch)*Rx(roll)
// R(rpy) = Rz(yaw) * Ry(pitch) * Rx(roll), rpy in degrees.

struct CartesianTarget
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

CartesianTarget computeReference(
    CartesianReferenceMode mode,
    const Eigen::Vector3d& initial_position,
    const Eigen::Quaterniond& initial_orientation,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& rpy_deg)
{
    constexpr double DEG2RAD =
        3.14159265358979323846 / 180.0;

    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(rpy_deg(2) * DEG2RAD, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(rpy_deg(1) * DEG2RAD, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rpy_deg(0) * DEG2RAD, Eigen::Vector3d::UnitX());

    CartesianTarget target{
        initial_position,
        initial_orientation
    };

    if (mode == CartesianReferenceMode::RelativeOffset)
    {
        // position is expressed in the ROBOT BASE frame.
        target.position = initial_position + position;

        // Rotation relative to the INITIAL EE orientation: R_d = R_0 * R_offset
        target.orientation = initial_orientation * rotation;
    }
    else if (mode == CartesianReferenceMode::AbsolutePose)
    {
        target.position = position;
        target.orientation = rotation;
    }

    target.orientation.normalize();

    if (!target.position.allFinite() ||
        !target.orientation.coeffs().allFinite())
    {
        throw std::runtime_error(
            "Desired Cartesian pose contains NaN/Inf"
        );
    }

    return target;
}


vec<7> targetToVec7(const CartesianTarget& target)
{
    vec<7> result{};

    result[0][0] = static_cast<float>(target.position.x());
    result[1][0] = static_cast<float>(target.position.y());
    result[2][0] = static_cast<float>(target.position.z());
    result[3][0] = static_cast<float>(target.orientation.w());
    result[4][0] = static_cast<float>(target.orientation.x());
    result[5][0] = static_cast<float>(target.orientation.y());
    result[6][0] = static_cast<float>(target.orientation.z());

    return result;
}


// VISION (GPD) - SOLO IMPRIMIR
//
// Convierte la pose del modelo [x y z qw qx qy qz] (termina en la INTERFAZ)
// a grasp::Pose y de regreso, y procesa cada agarre recibido.

grasp::Pose vec7ToPose(const vec<7>& v)
{
    grasp::Pose pose;
    pose.p = Eigen::Vector3d(v[0][0], v[1][0], v[2][0]);
    pose.R = Eigen::Quaterniond(v[3][0], v[4][0], v[5][0], v[6][0])
                 .normalized()
                 .toRotationMatrix();
    return pose;
}

vec<7> poseToVec7(const grasp::Pose& pose)
{
    const Eigen::Quaterniond q(pose.R);
    vec<7> v{};
    v[0][0] = static_cast<float>(pose.p.x());
    v[1][0] = static_cast<float>(pose.p.y());
    v[2][0] = static_cast<float>(pose.p.z());
    v[3][0] = static_cast<float>(q.w());
    v[4][0] = static_cast<float>(q.x());
    v[5][0] = static_cast<float>(q.y());
    v[6][0] = static_cast<float>(q.z());
    return v;
}

struct VisionState
{
    unsigned long received = 0;
    unsigned long accepted = 0;
    unsigned long rejected = 0;
    int status = 0;                       // 0 sin agarres, 1 aceptado, -1 rechazado
    bool has_valid = false;
    double last_time = -1.0;              // tiempo del experimento al recibir
    std::string last_result = "sin agarres todavia";
    grasp::GraspMessage valid_msg{};      // ultimo agarre VALIDO
    grasp::GraspTargets targets{};        // y sus poses en la base
    double valid_time = -1.0;
};

// Valida un agarre y, si es aceptable, calcula sus poses en la base.
// NO mueve el robot.
void processGrasp(
    VisionState& vs,
    const grasp::GraspMessage& msg,
    const vec<7>& pose,
    const vec<7>& dq,
    double time)
{
    ++vs.received;
    vs.last_time = time;

    std::string reason =
        grasp::validateGrasp(msg, cfg.vision);

    if (reason.empty() &&
        maxAbs(dq) > cfg.vision.max_joint_speed_rad_s)
    {
        reason = "robot en movimiento (la nube no corresponde a la q actual)";
    }

    grasp::GraspTargets targets;

    if (reason.empty())
    {
        targets = grasp::computeTargets(msg, vec7ToPose(pose), cfg.vision);

        if (!targets.grasp_base.p.allFinite() ||
            !targets.interface_pregrasp.p.allFinite() ||
            !targets.interface_grasp.R.allFinite())
        {
            reason = "transformacion no finita";
        }
    }

    std::ostringstream text;

    if (!reason.empty())
    {
        ++vs.rejected;
        vs.status = -1;
        text << "RECHAZADO (seq " << msg.seq << "): " << reason;
        vs.last_result = text.str();
        return;
    }

    ++vs.accepted;
    vs.status = 1;
    vs.has_valid = true;
    vs.valid_msg = msg;
    vs.targets = targets;
    vs.valid_time = time;
    text << "ACEPTADO (seq " << msg.seq << ")";
    vs.last_result = text.str();
}

std::string visionStatusText(
    const VisionState& vs,
    unsigned long bad_packets,
    double time)
{
    std::ostringstream o;
    o << std::fixed << std::setprecision(4);

    if (!cfg.vision.enabled)
    {
        o << " Vision                  : DISABLED\n";
        return o.str();
    }

    o << " UDP port                : " << cfg.vision.udp_port << "\n"
      << " Last message            : " << vs.last_result << "\n"
      << " Received / OK / rejected: " << vs.received << " / " << vs.accepted
      << " / " << vs.rejected << "   (bad packets: " << bad_packets << ")\n";

    if (vs.has_valid)
    {
        const auto& t = vs.targets;
        o << " Last valid grasp        : seq " << vs.valid_msg.seq
          << ", score " << vs.valid_msg.score
          << ", camera " << (vs.valid_msg.frame == 1 ? "DEPTH" : "COLOR")
          << ", " << std::setprecision(1) << (time - vs.valid_time) << " s ago\n"
          << std::setprecision(4)
          << " Grasp point (base) [m]  : " << t.grasp_base.p.transpose() << "\n"
          << " Interface @grasp [m]    : " << t.interface_grasp.p.transpose() << "\n"
          << " Interface @pregrasp [m] : " << t.interface_pregrasp.p.transpose() << "\n"
          << " Approach (tool z, base) : " << t.interface_grasp.R.col(2).transpose() << "\n";
    }

    o << " Robot motion from vision: NONE (print-only mode)\n";
    return o.str();
}


// TASK SEQUENCE - text for the dashboard

std::string taskStatusText(
    const task::Sequencer* sequencer,
    const fixture::FixtureClient* fixture_client,
    const screw_vision::Client* screw_client,
    const KinovaRobotLowLevel& robot)
{
    if (!sequencer)
    {
        return " Task sequence           : DISABLED\n";
    }

    std::ostringstream o;
    o << std::fixed << std::setprecision(2)
      << sequencer->statusText();

    if (robot.gripperDetected)
    {
        o << " Gripper                 : " << robot.gripperPos[0][0]
          << " % (command " << robot.gripperPosCommand[0][0] << " %)\n";
    }
    else
    {
        o << " Gripper                 : NOT DETECTED\n";
    }

    if (fixture_client)
    {
        o << " Fixture                 : " << fixture::modeName(fixture_client->mode());
        if (fixture_client->sampleFresh())
        {
            o << ", q = " << fixture_client->sample().q_deg << " deg";
        }
        if (std::isfinite(fixture_client->target()))
        {
            o << ", target " << fixture_client->target() << " deg"
              << (fixture_client->settled() ? " (arrived)" : "");
        }
        o << "\n";
        if (fixture_client->hasFault())
        {
            o << " Fixture fault           : " << fixture_client->faultText() << "\n";
        }
    }
    else
    {
        o << " Fixture                 : not used\n";
    }

    if (screw_client)
    {
        Eigen::Vector3d screw;
        if (sequencer->visionPosition(screw))
        {
            o << std::setprecision(3)
              << " Vision (screw TCP) [m]  : " << screw.transpose() << "\n";
            Eigen::Vector3d rpy, pre;
            if (sequencer->visionPose(rpy, pre))
            {
                o << std::setprecision(1)
                  << " Vision Rx Ry Rz [deg]   : " << rpy.transpose() << "\n"
                  << std::setprecision(3)
                  << " Vision pre-grasp [m]    : " << pre.transpose() << "\n";
            }
            if (!sequencer->plateCorrectionText().empty())
                o << " Placa en la pinza       : " << sequencer->plateCorrectionText() << "\n";
        }
        else
        {
            o << " Vision (screw TCP)      : no result yet\n";
        }
        const screw_vision::Reply& r = screw_client->last();
        if (r.seq != 0 && !r.ok)
        {
            o << " Vision last reply       : #" << r.seq << " FAIL " << r.text << "\n";
        }
        else if (r.seq != 0 && r.is_approach)
        {
            o << std::setprecision(3)
              << " Vision approach         : #" << r.seq << " " << r.text << "  EE "
              << r.tcp_m.transpose() << "  objeto " << r.centroid_m.transpose() << " (" << r.count
              << " puntos)\n";
        }
        else if (r.seq != 0 && r.is_cloud)
        {
            o << " Vision cloud (no GPD)   : #" << r.seq << " " << r.text << ".pcd, "
              << r.count << " points\n";
        }
        Eigen::Vector3d hole;
        double hole_diam = 0.0;
        if (sequencer->visionHole(hole, hole_diam))
        {
            o << std::setprecision(4)
              << " Hoyo de la vision [m]   : " << hole.transpose() << "  (diametro "
              << std::setprecision(1) << hole_diam * 1000.0 << " mm)\n";
        }
    }

    return o.str();
}


int controlThreadMain()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::unique_ptr<KinovaRobotLowLevel> robot;

    try
    {
        // IO / UDP
        
        boost::asio::io_context io_context;

        udp_server server{
            io_context,
            "KG3"
        };


        // VISION (GPD): receptor UDP no bloqueante, SOLO IMPRIMIR.
        // Se crea antes de conectar el robot: si el puerto falla, no arranca.

        std::unique_ptr<grasp::GraspReceiver> grasp_receiver;

        if (cfg.vision.enabled)
        {
            grasp_receiver =
                std::make_unique<grasp::GraspReceiver>(
                    cfg.vision.udp_port
                );

            std::cout
                << "\nVision: listening for GPD grasps on UDP port "
                << cfg.vision.udp_port
                << " (print-only, robot is never moved by vision)\n";
        }

        VisionState vision;


        // TASK SEQUENCE (Fase 2). Se valida y se conecta el fixture ANTES
        // de conectar el robot: un error de configuracion no toca el robot.

        std::unique_ptr<task::Sequencer> sequencer;
        std::unique_ptr<fixture::FixtureClient> fixture_client;
        std::unique_ptr<screw_vision::Client> screw_client;

        if (cfg.task.enabled)
        {
            sequencer =
                std::make_unique<task::Sequencer>(cfg.task);

            std::cout
                << "\nTask sequence: " << sequencer->stepCount()
                << " steps, validated.\n";

            if (cfg.task.use_fixture && task::usesFixture(cfg.task))
            {
                std::cout
                    << "Fixture: opening " << cfg.task.fixture.port
                    << " (the ESP32 may reboot; the fixture must be at its"
                    << " ORIGINAL position)...\n";

                fixture_client =
                    std::make_unique<fixture::FixtureClient>(cfg.task.fixture);

                fixture_client->connect();
                fixture_client->stop();
                fixture_client->pollFor(0.3);
                fixture_client->zero();
                fixture_client->start();

                std::cout
                    << "Fixture: ZERO + START done, holding 0 degrees.\n";
            }

            if (task::usesVision(cfg.task))
            {
                std::cout
                    << "Vision: checking screw_server on UDP port "
                    << cfg.task.vision_server_port << "...\n";

                screw_client =
                    std::make_unique<screw_vision::Client>(
                        cfg.task.vision_server_port);

                screw_client->ping(3.0);

                std::cout << "Vision: screw_server answered.\n";
            }
        }


        // CONNECT
        
        std::cout
            << "\nConnecting to Kinova Gen3 at "
            << ROBOT_IP
            << "...\n";


        robot =
            std::make_unique<KinovaRobotLowLevel>(
                ROBOT_IP,
                USERNAME,
                PASSWORD
            );


        robot->wait_callback_user_data =
            &io_context;


        robot->wait_callback =
            [](void* context)
            {
                static_cast<
                    boost::asio::io_context*
                >(context)->poll_one();
            };


        // INITIAL FEEDBACK
        
        vec<7> q =
            robot->getPos();

        vec<7> dq =
            robot->getVel();

        wrapJoints(q);


        if (!isFinite(q) ||
            !isFinite(dq))
        {
            throw std::runtime_error(
                "Initial q/dq contains NaN/Inf"
            );
        }


        vec<7> last_pose =
            GetPose(q);


        if (!isFinite(last_pose))
        {
            throw std::runtime_error(
                "Initial GetPose contains NaN/Inf"
            );
        }


        const Eigen::Vector3d
            initial_position =
                getPosition(last_pose);


        const Eigen::Quaterniond
            initial_orientation =
                getOrientation(last_pose);


        const Vec7E
            initial_q =
                toEigen7(q);


        // CONTROLLER CONFIGURATION
        
        CartesianImpedanceController controller;


        // Cartesian stiffness
        controller.setCartesianStiffness(
            cfg.stiffness
        );


        // Nullspace stiffness
        controller.setNullspaceStiffness(
            cfg.enable_nullspace
                ? cfg.nullspace_stiffness
                : 0.0
        );


        controller.setNullspaceWeights(
            cfg.nullspace_weights
        );

        controller.setNullspaceMaxTorque(
            cfg.nullspace_max_torque
        );


        // J4 soft limit (elbow kept bent)
        if (cfg.enable_j4_limit)
        {
            const double j4_min_rad =
                cfg.j4_min_deg * (M_PI / 180.0);

            if (initial_q(3) < j4_min_rad)
            {
                throw std::runtime_error(
                    "J4 = " +
                    std::to_string(initial_q(3) * 180.0 / M_PI) +
                    " deg is already below j4_min_deg = " +
                    std::to_string(cfg.j4_min_deg) +
                    " deg: bend the elbow (move to HOME_MOD) before starting"
                );
            }

            CartesianImpedanceController::JointSoftLimit limit;
            limit.enabled = true;
            limit.joint = 3;
            limit.min_rad = j4_min_rad;
            limit.stiffness = cfg.j4_limit_stiffness;
            limit.damping = cfg.j4_limit_damping;
            limit.max_torque = cfg.j4_limit_max_torque;

            controller.setJointSoftLimit(
                limit
            );
        }


        // Cartesian + nullspace damping
        controller.setDampingFactors(
            cfg.damping_factors,

            cfg.enable_nullspace
                ? cfg.nullspace_damping_factor
                : 0.0
        );


        controller.setFiltering(
            cfg.update_hz,
            cfg.pose_filter,
            cfg.stiffness_filter,
            cfg.wrench_filter,
            cfg.nullspace_filter
        );


        controller.setMaxTorqueDelta(
            cfg.delta_tau_max
        );


        controller.setReferenceSpeedLimit(
            cfg.reference_max_speed_m_s,
            cfg.reference_max_angular_speed_deg_s * (M_PI / 180.0)
        );


        controller.setWrench(
            cfg.enable_wrench
                ? cfg.wrench
                : Vec6E::Zero()
        );


        // DESIRED CARTESIAN REFERENCE (first target)

        const bool reference_is_absolute =
            cfg.cartesian_reference_mode ==
            CartesianReferenceMode::AbsolutePose;

        // With the task sequence the first reference is the CURRENT pose:
        // the robot does not move until the first step is confirmed.
        const CartesianTarget first_reference =
            cfg.task.enabled
                ? CartesianTarget{initial_position, initial_orientation}
                : computeReference(
                cfg.cartesian_reference_mode,
                initial_position,
                initial_orientation,
                reference_is_absolute
                    ? cfg.absolute_position
                    : cfg.position_offset,
                reference_is_absolute
                    ? cfg.absolute_orientation_rpy_deg
                    : cfg.orientation_offset_rpy_deg
            );

        const Eigen::Vector3d desired_position =
            first_reference.position;

        const Eigen::Quaterniond desired_orientation =
            first_reference.orientation;


        // FRICTION GAIN CHECK: Kfric must be finite and >= 0
        // (a negative gain would push against the motion).

        if (!cfg.Kfric.allFinite() ||
            (cfg.Kfric.array() < 0.0).any())
        {
            throw std::runtime_error(
                "Kfric must be finite and >= 0 for every joint"
            );
        }


        // TASK SEQUENCE CHECKS that need the robot: gripper present and
        // first move not too far from the initial pose.

        if (sequencer)
        {
            if (task::usesGripper(cfg.task) &&
                !robot->gripperDetected)
            {
                throw std::runtime_error(
                    "Task sequence uses the gripper but the robot reports no gripper"
                );
            }

            if (const task::TaskPose* first_move =
                    task::firstMovePose(cfg.task))
            {
                const task::InterfaceTarget first_target =
                    task::toInterface(*first_move, cfg.task.tcp_offset_m);

                const double distance =
                    (first_target.position - initial_position).norm();

                if (distance > cfg.task.max_move_distance_m)
                {
                    throw std::runtime_error(
                        "Task sequence: first pose '" + first_move->name +
                        "' is " + std::to_string(distance) +
                        " m from the current pose (max " +
                        std::to_string(cfg.task.max_move_distance_m) +
                        " m). Move the robot closer first."
                    );
                }
            }
        }


        // INITIALIZE CONTROLLER WITH DESIRED REFERENCE
        
        controller.initialize(
            desired_position,
            desired_orientation,
            initial_q
        );
        

        Vec7E nullspace_reference =
            initial_q;


        if (cfg.enable_nullspace)
        {
            if (!cfg.nullspace_use_initial_q)
            {
                nullspace_reference =
                    cfg.nullspace_posture_deg * (M_PI / 180.0);
            }

            nullspace_reference +=
                cfg.nullspace_offset;

            if (!nullspace_reference.allFinite())
            {
                throw std::runtime_error(
                    "nullspace reference contains NaN/Inf"
                );
            }
        }


        controller.setNullspaceReference(
            nullspace_reference
        );


        
        std::cout
            << "\n============================================================\n"
            << " CONTROLLER CONFIGURATION\n"
            << "============================================================\n"
            << " Cartesian task    : ENABLED\n"
            << " Reference mode   : "
            << cartesianReferenceModeName(
                    cfg.cartesian_reference_mode)
            << "\n"
            << " Nullspace         : "
            << (cfg.enable_nullspace
                    ? "ENABLED"
                    : "DISABLED")
            << "\n"
            << " J4 soft limit     : "
            << (cfg.enable_j4_limit
                    ? "ENABLED (J4 >= " + std::to_string(cfg.j4_min_deg) + " deg)"
                    : std::string("DISABLED"))
            << "\n"
            << " Desired wrench    : "
            << (cfg.enable_wrench
                    ? "ENABLED"
                    : "DISABLED")
            << "\n"
            << " Impedance -> HW   : "
            << (ENABLE_IMPEDANCE_HARDWARE
                    ? "YES"
                    : "NO")
            << "\n"
            << " Gravity -> HW     : YES\n"
            << "============================================================\n";


        std::cout
            << "\nDesired position [m]:\n"
            << desired_position.transpose()
            << "\n"
            << "\nDesired quaternion [w x y z]:\n"
            << desired_orientation.w() << " "
            << desired_orientation.x() << " "
            << desired_orientation.y() << " "
            << desired_orientation.z() << "\n";


        std::cout
            << "\nStiffness:\n"
            << cfg.stiffness.transpose()
            << "\n";

        if (cfg.enable_near_target_stiffness)
        {
            if (!std::isfinite(cfg.near_target_stiffness) || cfg.near_target_stiffness < 0 ||
                !std::isfinite(cfg.near_target_full_m) || cfg.near_target_full_m < 0 ||
                !std::isfinite(cfg.near_target_start_m) ||
                cfg.near_target_start_m <= cfg.near_target_full_m)
                throw std::runtime_error(
                    "CONFIG: near_target_stiffness >= 0 y "
                    "0 <= near_target_full_m < near_target_start_m");

            std::cout
                << "Cerca del objetivo: traslacion "
                << cfg.near_target_stiffness
                << " N/m a menos de "
                << cfg.near_target_full_m * 100.0
                << " cm (sube desde "
                << cfg.near_target_start_m * 100.0
                << " cm)\n";
        }

        if (cfg.enable_friction_off_near_target)
        {
            if (!std::isfinite(cfg.friction_off_within_m) || cfg.friction_off_within_m < 0 ||
                !std::isfinite(cfg.friction_off_fade_m) || cfg.friction_off_fade_m < 0)
                throw std::runtime_error(
                    "CONFIG: friction_off_within_m y friction_off_fade_m >= 0");

            std::cout
                << "Friccion: sin compensacion a <= "
                << cfg.friction_off_within_m * 1000.0
                << " mm del objetivo (100 % desde "
                << (cfg.friction_off_within_m + cfg.friction_off_fade_m) * 1000.0
                << " mm)\n";
        }


        std::cout
            << "\nDamping factors:\n"
            << cfg.damping_factors.transpose()
            << "\n";


        if (cfg.enable_nullspace)
        {
            std::cout
                << "\nNullspace stiffness: "
                << cfg.nullspace_stiffness
                << " Nm/rad, damping factor "
                << cfg.nullspace_damping_factor
                << ", max |tau_null| "
                << cfg.nullspace_max_torque
                << " Nm\n"
                << "Nullspace weights:\n"
                << cfg.nullspace_weights.transpose()
                << "\n"
                << "Nullspace posture [deg] ("
                << (cfg.nullspace_use_initial_q
                        ? "initial q + offset"
                        : "nullspace_posture_deg + offset")
                << "):\n"
                << (nullspace_reference * (180.0 / M_PI)).transpose()
                << "\n"
                << "Initial q [deg]:\n"
                << (initial_q * (180.0 / M_PI)).transpose()
                << "\n";
        }


        if (cfg.enable_wrench)
        {
            std::cout
                << "\nDesired wrench:\n"
                << cfg.wrench.transpose()
                << "\n";
        }


        // CURRENT MODE
        
        using enum kinova_robot::ControlMode;


        robot->setControlMode(
            {
                CUR,
                CUR,
                CUR,
                CUR,
                CUR,
                CUR,
                CUR
            }
        );


        // GRAVITY STARTUP

        std::cout
            << "\nStarting gravity compensation...\n";


        sendGravity(
            *robot,
            cfg.gravity_warmup_cycles
        );


        std::cout
            << "Gravity compensation active.\n"
            << "Starting shadow experiment.\n";


        // CLOCK

        using Clock =
            std::chrono::steady_clock;


        const auto experiment_start =
            Clock::now();


        auto previous_loop_time =
            experiment_start;


        double last_display_time =
            -cfg.dashboard_period_s;


        double impedance_blend =
            0.0;


        // Friction observer state (reset on every OFF -> ON transition).
        FrictionObserver friction;

        bool was_enabled =
            false;


        // Current target pose [x y z qw qx qy qz], for logging/dashboard.
        CartesianTarget current_target =
            first_reference;


        // CONTROL LOOP

        while (
            !stop_requested &&
            !robot->timeout
        )
        {
            const auto now =
                Clock::now();


            const double time =
                std::chrono::duration<double>(
                    now -
                    experiment_start
                ).count();


            const double dt =
                std::chrono::duration<double>(
                    now -
                    previous_loop_time
                ).count();


            previous_loop_time =
                now;


            const double experiment_duration_s =
                cfg.task.enabled
                    ? cfg.task.duration_s
                    : cfg.experiment_duration_s;

            if (
                experiment_duration_s > 0.0
                &&
                time >=
                    experiment_duration_s
            )
            {
                break;
            }


            // 1. ROBOT FEEDBACK

            q =
                robot->getPos();

            dq =
                robot->getVel();

            wrapJoints(q);


            const vec<7>
                current_meas =
                    robot->getCur();


            const vec<7>
                tau_meas =
                    robot->getTau();


            if (!isFinite(q) ||
                !isFinite(dq) ||
                !isFinite(current_meas) ||
                !isFinite(tau_meas))
            {
                throw std::runtime_error(
                    "Non-finite robot feedback"
                );
            }


            // 2. ANALYTIC MODEL

            const vec<7> pose =
                GetPose(
                    q,
                    last_pose
                );


            last_pose =
                pose;


            const mat<6, 7> J_lab =
                GetJacobian(q);


            const vec<7> gravity =
                GravityVector(q);


            if (!isFinite(pose) ||
                !isFinite(gravity))
            {
                throw std::runtime_error(
                    "Non-finite analytic model output"
                );
            }


            // 2b. VISION (GPD) - SOLO IMPRIMIR
            //
            // poll() no bloquea. Si llego un agarre se valida y se pasa a la
            // base con la pose actual de la interfaz. No se mueve el robot.

            if (grasp_receiver)
            {
                grasp::GraspMessage grasp_msg;

                if (grasp_receiver->poll(grasp_msg))
                {
                    processGrasp(
                        vision,
                        grasp_msg,
                        pose,
                        dq,
                        time
                    );
                }
            }


            // 3. CONVERT TO EIGEN FIXED-SIZE TYPES

            const Vec7E q_eigen =
                toEigen7(q);


            const Vec7E dq_eigen =
                toEigen7(dq);


            const Mat67E J_eigen =
                toEigenJacobian(J_lab);


            const Eigen::Vector3d position =
                getPosition(pose);


            const Eigen::Quaterniond orientation =
                getOrientation(pose);


            if (!q_eigen.allFinite() ||
                !dq_eigen.allFinite() ||
                !J_eigen.allFinite() ||
                !position.allFinite() ||
                !orientation.coeffs().allFinite())
            {
                throw std::runtime_error(
                    "Non-finite Eigen state"
                );
            }


            // 3b. SLIDER STATE

            const bool enabled =
                impedance_enabled.load(std::memory_order_relaxed);


            // 3c. TASK SEQUENCE (Fase 2)
            //
            // Only advances with the control ACTIVE: slider ON and, with the
            // hardware enabled, the impedance ramp complete (blend of the
            // previous cycle). Its commands are executed here: new reference,
            // gripper command, fixture angle.

            if (sequencer)
            {
                const int mode_request =
                    task_mode_requested.exchange(-1, std::memory_order_relaxed);
                if (mode_request >= 0 &&
                    sequencer->automatic() != (mode_request == 1))
                {
                    sequencer->setAutomatic(mode_request == 1);
                    std::cout << "\nGUI: modo "
                              << (mode_request == 1 ? "AUTOMATICO" : "PASO A PASO")
                              << "\n";
                }

                if (fixture_client)
                {
                    fixture_client->poll();
                }

                if (screw_client)
                {
                    screw_client->poll();
                }

                const Vec6E task_velocity =
                    J_eigen * dq_eigen;

                task::Inputs task_in;
                task_in.dt = dt;
                task_in.active =
                    enabled &&
                    (!ENABLE_IMPEDANCE_HARDWARE ||
                     impedance_blend >= 1.0);
                task_in.confirm =
                    task_confirm_requested.exchange(
                        false, std::memory_order_relaxed);
                task_in.pause =
                    task_pause_requested.exchange(
                        false, std::memory_order_relaxed);
                task_in.position = position;
                task_in.orientation = orientation;
                task_in.linear_speed_m_s =
                    task_velocity.head<3>().norm();
                task_in.angular_speed_rad_s =
                    task_velocity.tail<3>().norm();
                task_in.gripper_detected =
                    robot->gripperDetected;
                task_in.gripper_pos_pct =
                    robot->gripperPos[0][0];

                if (fixture_client)
                {
                    task_in.fixture_settled =
                        fixture_client->settled();
                    task_in.fixture_fault =
                        fixture_client->hasFault();
                    task_in.fixture_fault_text =
                        fixture_client->faultText();
                }

                if (screw_client)
                {
                    const screw_vision::Reply& r =
                        screw_client->last();
                    task_in.vision_seq = r.seq;
                    task_in.vision_ok = r.ok;
                    task_in.vision_tcp_m = r.tcp_m;
                    task_in.vision_has_pose = r.has_pose;
                    task_in.vision_rpy_deg = r.rpy_deg;
                    task_in.vision_pregrasp_m = r.pregrasp_m;
                    task_in.vision_has_edge = r.has_edge;
                    task_in.vision_edge_a_m = r.edge_a_m;
                    task_in.vision_edge_b_m = r.edge_b_m;
                    task_in.vision_is_cloud = r.is_cloud;
                    task_in.vision_is_approach = r.is_approach;
                    task_in.vision_approach_state = r.approach_state;
                    task_in.vision_centroid_m = r.centroid_m;
                    task_in.vision_has_hole = r.has_hole;
                    task_in.vision_hole_m = r.hole_m;
                    task_in.vision_hole_diam_m = r.hole_diam_m;
                    task_in.vision_text =
                        !r.ok          ? r.text
                        : r.is_approach ? r.text + ", " + std::to_string(r.count) + " points of the object"
                        : r.is_cloud    ? r.text + ".pcd, " + std::to_string(r.count) + " points" +
                                           (r.has_hole ? ", " + std::to_string(r.hole_count) + " holes" : "")
                                     : "GPD: " + std::to_string(r.count) + " grasps";
                }

                const size_t step_before =
                    sequencer->stepIndex();
                const task::Status status_before =
                    sequencer->status();

                const task::Commands task_cmd =
                    sequencer->update(task_in);

                if (task_cmd.set_reference)
                {
                    current_target =
                        CartesianTarget{
                            task_cmd.position,
                            task_cmd.orientation
                        };

                    controller.setReferencePose(
                        current_target.position,
                        current_target.orientation
                    );
                }

                // Postura del codo de la pose (articulaciones de la app web).
                // Se filtra con nullspace_filter, sin salto.
                if (task_cmd.set_posture &&
                    cfg.enable_nullspace)
                {
                    const Vec7E posture =
                        task_cmd.posture_rad +
                        cfg.nullspace_offset;

                    if (!posture.allFinite())
                    {
                        throw std::runtime_error(
                            "task posture contains NaN/Inf"
                        );
                    }

                    controller.setNullspaceReference(
                        posture
                    );
                }

                if (task_cmd.set_gripper)
                {
                    robot->gripperPosCommand[0][0] =
                        static_cast<float>(task_cmd.gripper_pct);
                    robot->gripperVelCommand[0][0] =
                        static_cast<float>(task_cmd.gripper_speed_pct);
                    robot->gripperForceCommand =
                        static_cast<float>(task_cmd.gripper_force_pct);
                }

                if (task_cmd.move_fixture)
                {
                    if (!fixture_client)
                    {
                        sequencer->abort("fixture step but no fixture connected");
                    }
                    else
                    {
                        try
                        {
                            fixture_client->moveTo(task_cmd.fixture_deg);
                        }
                        catch (const std::exception& fixture_error)
                        {
                            sequencer->abort(fixture_error.what());
                        }
                    }
                }

                if (task_cmd.request_vision)
                {
                    // TOOL pose (web app convention): the server needs it to
                    // go camera -> base. From the model with the measured
                    // joints, NOT robot->toolPose: in low-level servoing the
                    // robot stops updating it and it stays at the startup
                    // pose (2026-10-14: photos at ARRIBA_TOMARTORNILLOS sent
                    // as HOME_MOD -> "vision rechazada"). Same TCP offset as
                    // the task poses (~7 mm rms vs the web app).
                    double tool_pose[6];
                    task::interfaceToToolPose(
                        position,
                        orientation,
                        cfg.task.tcp_offset_m,
                        tool_pose
                    );
                    bool pose_ok = true;
                    for (int i = 0; i < 6; ++i)
                    {
                        pose_ok = pose_ok && std::isfinite(tool_pose[i]);
                    }

                    if (!screw_client)
                    {
                        sequencer->abort("vision step but no vision client");
                    }
                    else if (!pose_ok)
                    {
                        sequencer->abort("TOOL pose from the model is not finite");
                    }
                    else if (!screw_client->request(task_cmd.vision_seq, tool_pose,
                                                    task_cmd.vision_cloud_only,
                                                    task_cmd.vision_object,
                                                    task_cmd.vision_approach))
                    {
                        sequencer->abort("could not send the request to screw_server");
                    }
                }

                if (sequencer->stepIndex() != step_before ||
                    sequencer->status() != status_before)
                {
                    std::cout
                        << "\nTask [t = " << time << " s]: "
                        << task::statusName(sequencer->status())
                        << " - " << sequencer->message() << "\n";
                }
            }


            // 4. FULL SHADOW IMPEDANCE CONTROLLER
            //
            // compute() returns:
            //
            // tau_task
            // tau_null
            // tau_ext
            // tau_impedance
            // pose_error
            
            // 3b. RIGIDEZ CERCA DEL OBJETIVO
            //
            // La de traslacion sube cuando la herramienta esta cerca del
            // objetivo de la referencia (ver controller_config.hpp).
            const double target_dist_m =
                (position - current_target.position).norm();

            Vec6E stiffness_now =
                cfg.stiffness;

            if (cfg.enable_near_target_stiffness)
            {
                stiffness_now =
                    CartesianImpedanceController::nearTargetStiffness(
                        cfg.stiffness,
                        cfg.near_target_stiffness,
                        cfg.near_target_start_m,
                        cfg.near_target_full_m,
                        target_dist_m
                    );

                controller.setCartesianStiffnessNow(
                    stiffness_now
                );
            }


            const auto ctrl =
                controller.compute(
                    q_eigen,
                    dq_eigen,
                    position,
                    orientation,
                    J_eigen
                );


            if (!ctrl.tau_task.allFinite() ||
                !ctrl.tau_spring.allFinite() ||
                !ctrl.tau_null.allFinite() ||
                !ctrl.tau_limit.allFinite() ||
                !ctrl.tau_ext.allFinite() ||
                !ctrl.tau_impedance.allFinite() ||
                !ctrl.pose_error.allFinite())
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: controller returned NaN/Inf"
                );
            }


            const vec<7> tau_task =
                eigenToVec7(
                    ctrl.tau_task
                );


            const vec<7> tau_null =
                eigenToVec7(
                    ctrl.tau_null
                );


            const vec<7> tau_ext =
                eigenToVec7(
                    ctrl.tau_ext
                );


            const vec<7> tau_imp =
                eigenToVec7(
                    ctrl.tau_impedance
                );


            const vec<6> pose_error =
                eigenToVec6(
                    ctrl.pose_error
                );


            const vec<7> tau_spring =
                eigenToVec7(
                    ctrl.tau_spring
                );

            
            const vec<7> Kfric =
                        eigenToVec7(
                            cfg.Kfric
                        );

            // 4b. FRICTION COMPENSATION
            //
            // Only with the slider ON. Direction from the spring torque
            // (no damping), as in main_op_space.cpp. z is reset OFF -> ON.
            
            FrictionOutput friction_out{};

            // Fraccion de la compensacion usada (1 = completa).
            double friction_scale =
                1.0;

            if (enabled &&
                cfg.enable_friction_compensation)
            {
                if (!was_enabled)
                {
                    friction.reset();
                }

                friction_out =
                    friction.step(
                        dq,
                        tau_spring.hadamard(Kfric),
                        static_cast<float>(cfg.friction_dt)
                    );

                // En el objetivo no compensa (controller_config.hpp,
                // enable_friction_off_near_target): ahi solo hacia vibrar.
                if (cfg.enable_friction_off_near_target)
                {
                    friction_scale =
                        project_config::frictionScaleNearTarget(
                            target_dist_m,
                            cfg.friction_off_within_m,
                            cfg.friction_off_fade_m
                        );

                    for (int j = 0; j < 7; ++j)
                        friction_out.torque[j][0] *=
                            static_cast<float>(friction_scale);
                }

                // Limite de J4: cerca del limite la compensacion de friccion
                // NO empuja a estirar el codo. Se apaga en rampa (por
                // posicion, sin escalon) desde j4_min + fade hasta j4_min;
                // la friccion real lo detiene ahi. Hacia doblar no se toca.
                // (2026-10-14 15-56-07: meter tau_limit en la direccion de la
                // friccion la hacia saltar +-5.5 Nm a ~60 Hz = vibracion.)
                if (cfg.enable_j4_limit &&
                    friction_out.torque[3][0] < 0.0f)
                {
                    const double fade_rad =
                        cfg.j4_friction_fade_deg * (M_PI / 180.0);
                    const double above =
                        q_eigen(3) - cfg.j4_min_deg * (M_PI / 180.0);
                    const double scale =
                        std::clamp(above / fade_rad, 0.0, 1.0);

                    friction_out.torque[3][0] *=
                        static_cast<float>(scale);
                }
            }

            was_enabled =
                enabled;


            // Impedance + friction, limited to maxTau before gravity.
            vec<7> tau_control =
                tau_imp +
                friction_out.torque;

            // 4c. SOLO GRAVEDAD EN EL OBJETIVO (controller_config.hpp,
            // enable_gravity_only_near_target): a <= 5 mm y <= 2 grados del
            // objetivo no hay accion de control; rampa hasta 7 mm / 3 grados.
            double control_scale =
                1.0;

            if (cfg.enable_gravity_only_near_target)
            {
                const double target_ang_deg =
                    orientation.angularDistance(current_target.orientation) *
                    (180.0 / M_PI);

                control_scale =
                    project_config::controlScaleNearTarget(
                        target_dist_m,
                        target_ang_deg,
                        cfg.gravity_only_within_m,
                        cfg.gravity_only_fade_m,
                        cfg.gravity_only_within_deg,
                        cfg.gravity_only_fade_deg
                    );

                for (int j = 0; j < 7; ++j)
                    tau_control[j][0] *=
                        static_cast<float>(control_scale);
            }

            saturateTorque(
                tau_control
            );


            // 5. CARTESIAN VELOCITY
            
            const Vec6E
                cart_velocity_eigen =
                    J_eigen *
                    dq_eigen;


            if (!cart_velocity_eigen.allFinite())
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: J*dq contains NaN/Inf"
                );
            }


            const vec<6>
                cart_velocity =
                    eigenToVec6(
                        cart_velocity_eigen
                    );


            // 6. VIRTUAL TOTAL TORQUE
            
            const vec<7>
                tau_virtual =
                    gravity +
                    tau_control;


            if (!isFinite(tau_virtual))
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: tau_virtual contains NaN/Inf"
                );
            }


            // 7. VIRTUAL CURRENT

            const vec<7>
                current_virtual =
                    KTGR.hadamard(
                        tau_virtual
                    );


            if (!isFinite(current_virtual))
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: current_virtual contains NaN/Inf"
                );
            }


            // 8. SHADOW HEALTH

            const bool finite_ok =
                isFinite(pose_error)
                &&
                isFinite(cart_velocity)
                &&
                isFinite(tau_task)
                &&
                isFinite(tau_null)
                &&
                isFinite(tau_ext)
                &&
                isFinite(tau_imp)
                &&
                isFinite(tau_virtual);


            if (!finite_ok)
            {
                throw std::runtime_error(
                    "SHADOW SAFETY: non-finite controller signal"
                );
            }


            const bool pose_warning =
                poseErrorWarning(
                    pose_error
                );


            const bool velocity_warning =
                cartVelocityWarning(
                    cart_velocity
                );


            const bool impedance_warning =
                impedanceTorqueWarning(
                    tau_imp
                );


            const bool virtual_tau_warning =
                torqueLimitExceeded(
                    tau_virtual
                );


            const bool virtual_current_warning =
                currentLimitExceeded(
                    current_virtual
                );


            // 9. HARDWARE TORQUE

            vec<7> tau_hw{};


            if constexpr (
                ENABLE_IMPEDANCE_HARDWARE)
            {

                if (!enabled)
                {
                    impedance_blend = 0.0;
                }
                else if (cfg.impedance_blend_time_s > 0.0)
                {
                    impedance_blend +=
                        dt / cfg.impedance_blend_time_s;
                }
                else
                {
                    impedance_blend = 1.0;
                }

                impedance_blend =
                    std::clamp(impedance_blend, 0.0, 1.0);


                tau_hw =
                    gravity;


                for (size_t joint = 0;
                     joint < 7;
                     ++joint)
                {
                    // blend * (impedance + friction)
                    tau_hw[joint][0] +=
                        static_cast<float>(
                            impedance_blend
                        )
                        *
                        tau_control[joint][0];
                }


                saturateTorque(
                    tau_hw
                );
            }
            else
            {
                // SHADOW MODE
                //
                // ONLY GRAVITY GOES TO HARDWARE.

                impedance_blend =
                    0.0;


                tau_hw =
                    gravity;
            }


            if (!isFinite(tau_hw))
            {
                throw std::runtime_error(
                    "HARDWARE SAFETY: tau_hw contains NaN/Inf"
                );
            }


            // 10. TORQUE -> CURRENT

            const vec<7>
                current_cmd =
                    KTGR.hadamard(
                        tau_hw
                    );


            if (!isFinite(current_cmd))
            {
                throw std::runtime_error(
                    "HARDWARE SAFETY: current_cmd contains NaN/Inf"
                );
            }


            // 11. REAL HARDWARE CURRENT BARRIER

            if (
                currentLimitExceeded(
                    current_cmd
                )
            )
            {
                throw std::runtime_error(
                    "HARDWARE SAFETY: current command exceeds maxCurrent"
                );
            }


            // 12. SEND TO ROBOT

            robot->sendCur(
                current_cmd
            );


            // 13. LOGGER

            logger.log(

                "t"_nm =
                    static_cast<float>(
                        time
                    ),

                "q"_nm =
                    q,

                "dq"_nm =
                    dq,

                "pose"_nm =
                    pose,

                "pose_err"_nm =
                    pose_error,

                "cart_vel"_nm =
                    cart_velocity,

                "tau_task"_nm =
                    tau_task,

                "tau_null"_nm =
                    tau_null,

                "tau_ext"_nm =
                    tau_ext,

                "tau_imp"_nm =
                    tau_imp,

                "tau_g"_nm =
                    gravity,

                "tau_virtual"_nm =
                    tau_virtual,

                "tau_hw"_nm =
                    tau_hw,

                "current_virtual"_nm =
                    current_virtual,

                "current_cmd"_nm =
                    current_cmd,

                "current_meas"_nm =
                    current_meas,

                "tau_meas"_nm =
                    tau_meas,

                "tau_spring"_nm =
                    tau_spring,

                "friction_FF"_nm =
                    friction_out.FF,

                "tau_friction"_nm =
                    friction_out.torque,

                "ref_target"_nm =
                    targetToVec7(current_target),

                "grasp_status"_nm =
                    static_cast<float>(vision.status),

                "grasp_score"_nm =
                    vision.has_valid ? vision.valid_msg.score : 0.0f,

                "grasp_base"_nm =
                    vision.has_valid
                        ? poseToVec7(vision.targets.grasp_base)
                        : vec<7>{},

                "grasp_pregrasp"_nm =
                    vision.has_valid
                        ? poseToVec7(vision.targets.interface_pregrasp)
                        : vec<7>{},

                "task_step"_nm =
                    sequencer
                        ? static_cast<float>(sequencer->stepIndex() + 1)
                        : 0.0f,

                "task_status"_nm =
                    sequencer
                        ? static_cast<float>(sequencer->status())
                        : 0.0f,

                "gripper_pos"_nm =
                    robot->gripperDetected
                        ? robot->gripperPos[0][0]
                        : 0.0f,

                "fixture_q"_nm =
                    (fixture_client && fixture_client->sample().valid)
                        ? static_cast<float>(fixture_client->sample().q_deg)
                        : 0.0f,

                "tau_j4_limit"_nm =
                    static_cast<float>(ctrl.tau_limit(3)),

                "k_trans"_nm =
                    static_cast<float>(stiffness_now(0)),

                "friction_scale"_nm =
                    static_cast<float>(friction_scale),
                "control_scale"_nm =
                    static_cast<float>(control_scale)
            );



            if (
                time -
                last_display_time
                >=
                cfg.dashboard_period_s
            )
            {
                last_display_time =
                    time;


                printDashboard(

                    time,

                    impedance_blend,

                    targetToVec7(current_target),

                    pose,

                    pose_error,

                    cart_velocity,

                    tau_task,

                    tau_null,

                    tau_ext,

                    tau_imp,

                    friction_out.torque,

                    gravity,

                    tau_virtual,

                    tau_hw,

                    current_virtual,

                    current_cmd,

                    current_meas,

                    finite_ok,

                    pose_warning,

                    velocity_warning,

                    impedance_warning,

                    virtual_tau_warning,

                    virtual_current_warning,

                    visionStatusText(
                        vision,
                        grasp_receiver ? grasp_receiver->badPackets() : 0,
                        time
                    ),

                    taskStatusText(
                        sequencer.get(),
                        fixture_client.get(),
                        screw_client.get(),
                        *robot
                    ),

                    stiffness_now,

                    (stiffness_now.head<3>() - cfg.stiffness.head<3>()).norm() > 1e-9,

                    friction_scale,

                    control_scale
                );

                // Para la interfaz grafica (gui/): articulaciones medidas en
                // grados. Solo se imprime; no cambia nada del control.
                std::cout << "\nJOINTS [deg]:";
                for (int j = 0; j < 7; ++j)
                {
                    std::cout << " " << std::setprecision(3) << q_eigen(j) * (180.0 / M_PI);
                }
                std::cout << "\n" << std::flush;

                if (sequencer)
                {
                    std::lock_guard<std::mutex> lock(gui_dashboard_mutex);
                    gui_dashboard_text =
                        std::string(enabled
                            ? "Control ON - gravity + Cartesian impedance\n"
                            : "Control OFF - gravity compensation active\n")
                        + "Tarea: " + task::statusName(sequencer->status())
                        + "\n" + sequencer->message();
                }
            }
        }



        std::cout
            << "\n\nStopping experiment...\n"
            << "Returning to gravity-only transition...\n";


        sendGravity(
            *robot,
            cfg.gravity_shutdown_cycles
        );


        robot.reset();


        saveLogger();


        std::cout
            << "\nExperiment finished cleanly.\n";


        return 0;
    }


    // ========================================================
    // STD EXCEPTION
    // ========================================================

    catch (const std::exception& error)
    {
        std::cerr
            << "\n\n"
            << "============================================================\n"
            << " SAFETY / EXCEPTION EXIT\n"
            << "============================================================\n"
            << error.what()
            << "\n";


        if (robot)
        {
            try
            {
                std::cerr
                    << "Attempting gravity-only transition...\n";


                sendGravity(
                    *robot,
                    cfg.gravity_shutdown_cycles
                );
            }
            catch (
                const std::exception&
                gravity_error
            )
            {
                std::cerr
                    << "Gravity transition failed: "
                    << gravity_error.what()
                    << "\n";
            }
            catch (...)
            {
                std::cerr
                    << "Gravity transition failed "
                    << "with unknown exception.\n";
            }


            robot.reset();
        }


        try
        {
            saveLogger();
        }
        catch (...)
        {
            std::cerr
                << "Could not save CSV after exception.\n";
        }


        return 1;
    }


    // ========================================================
    // UNKNOWN EXCEPTION
    // ========================================================

    catch (...)
    {
        std::cerr
            << "\nUnknown fatal exception.\n";


        if (robot)
        {
            try
            {
                sendGravity(
                    *robot,
                    cfg.gravity_shutdown_cycles
                );
            }
            catch (...)
            {
            }


            robot.reset();
        }


        try
        {
            saveLogger();
        }
        catch (...)
        {
        }


        return 1;
    }
}


int main(int argc, char** argv)
{
    GtkApplication* app = gtk_application_new(
        "org.kinova.impedance", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app, "activate", G_CALLBACK(activate), nullptr);

    std::thread control_thread(controlThreadMain);
    std::thread gui_command_thread(guiCommandListener);
    const int status = g_application_run(G_APPLICATION(app), argc, argv);

    stop_requested = 1;
    control_thread.join();
    gui_command_thread.join();
    g_object_unref(app);
    return status;
}
