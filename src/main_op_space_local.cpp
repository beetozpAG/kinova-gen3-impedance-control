// Control operacional local para Kinova Gen3 (7 DoF).
// Derivado de main_op_space.cpp: sin vecino UDP, tanh, estados virtuales
// ni control articular. Conserva el observador de friccion (LuGre) del original.
// Usa las dependencias del proyecto MODULAR/Controlador_Impedancia:
// Matrix.h, Gen3_model.h, KinovaRobot/kinova_robot.h, GTK3, Kortex y la
// clase CartesianImpedanceController (Eigen).
// --self-test comprueba las leyes sin abrir GTK ni conectarse al robot.
// HARDWARE: slider ON  -> gravedad + blend * (PD cartesiano + friccion)
//                         (SE APLICA AL ROBOT).
//           slider OFF -> solo gravedad.
// SHADOW:   la ley de impedancia cartesiana se calcula y registra SIEMPRE
//           (ON y OFF), pero NUNCA se envia al robot.
// El punto controlado es el de GetPose, NO el TCP Robotiq de MATLAB.
/* Compilacion desde una terminal (requiere las librerias ya construidas):
proyecto='/home/beetozp/Escritorio/MODULAR/Controlador_Impedancia'
sdk="$proyecto/build/_deps/kortex_api_rel-src"
c++ -std=c++20 -O2 -Wall -Wextra -pthread -D_OS_UNIX \
  -I"$proyecto" -I"$proyecto/Matrix" -I"$proyecto/KinovaRobot" -I"$proyecto/include" \
  -I"$sdk/include" -I"$sdk/include/client" -I"$sdk/include/common" \
  -I"$sdk/include/messages" -I"$sdk/include/client_stubs" \
  $(pkg-config --cflags eigen3) \
  "$proyecto/src/main_op_space_local.cpp" \
  "$proyecto/src/cartesian_impedance_controller.cpp" \
  "$proyecto/build/KinovaRobot/libkinova_robot.a" \
  "$sdk/lib/release/libKortexApiCpp.a" \
  $(pkg-config --cflags --libs gtk+-3.0) -latomic -o ./main_op_space_local
./main_op_space_local --self-test
La ejecucion sin --self-test SI se conecta al robot, envia gravedad y,
con el slider ON, envia tambien el PD cartesiano.
No se necesita udp_server.h ni SaveData.h.
*/

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <gtk/gtk.h>
#include <Eigen/Dense>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include "Matrix.h"
#include "Gen3_model.h"
#include "kinova_robot.h"
#include "cartesian_impedance_controller.hpp"
#include "controller_config.hpp"

using namespace matrix;
using kinova_robot::KinovaRobotLowLevel;
using impedance::CartesianImpedanceController;
using Clock = std::chrono::steady_clock;
constexpr float PI_LOCAL = 3.14159265358979323846f;

// ================= CONFIGURACION LOCAL =================
// true: con el slider ON el PD cartesiano SE APLICA al robot (con blend).
// false: el PD tambien queda solo calculado; el hardware recibe gravedad.
constexpr bool ENABLE_PD_HARDWARE = true;
// Ganancias, filtros, nullspace y wrench de la impedancia SHADOW:
// se toman de controller_config.hpp del proyecto de impedancia.
// La referencia de pose es la MISMA que la del PD (local_reference).
const project_config::ControllerConfig IMPEDANCE_CFG{};
constexpr const char* ROBOT_IP = "192.168.1.10";
enum class ReferenceMode { RelativeOffset, AbsolutePose };
constexpr ReferenceMode REFERENCE_MODE = ReferenceMode::RelativeOffset;
const vec<3> POSITION_OFFSET = {0.00f, 0.00f, 0.00f}; // m, en la base

const vec<3> ORIENTATION_OFFSET_RPY = {0, 0, 0}; // grados, respecto al EE inicial

const vec<3> ABSOLUTE_POSITION = {0.5f, 0.0f, 0.6f}; // m, ejemplo sin validar
const vec<3> ABSOLUTE_RPY = {0, 0, 0}; // roll, pitch, yaw en grados, base
// Se conserva k y th de calcTau original; se elimina la no linealidad tanh.

constexpr float K_LOCAL = 7.0f;
constexpr float TH_LOCAL = 1;
// Amortiguamiento NUEVO sobre velocidad medida [vx vy vz wx wy wz].
// Valores iniciales de trabajo, no ajuste validado en hardware.
const vec<6> D_LOCAL = {6, 6, 6, 2.5f, 2.5f, 2.5f}; // N s/m y Nm s/rad
constexpr double DURATION_S = 60.0;
constexpr double BLEND_TIME_S = 1.0; // Solo incorporacion de torque al hardware
constexpr int LOG_EVERY = 10;

// ================= COMPENSACION DE FRICCION =================
// Observador tipo LuGre de main_op_space.cpp (Friction_Observer), con los
// parametros sigma0, sigma1, FC, FS y v0 de Gen3_model.h.
// Solo actua con el slider ON y entra con el mismo blend que el PD.
constexpr bool ENABLE_FRICTION_COMPENSATION = true;
// Signo suave: tanh(FACTOR * tau_PD^3). 100000 es el valor del original
// (casi un interruptor). Bajarlo (100-1000) suaviza si hay vibracion.
constexpr float FRICTION_SIGN_FACTOR = 1000.0f;
// Escala de la compensacion: 1 = igual que el original. Menor = mas conservador.
constexpr float FRICTION_GAIN = 1.0f;
constexpr float FRICTION_DT = 0.001f; // Paso fijo del RK4, como el original

std::atomic_bool stop_requested{false}, control_on{false}, worker_done{false};
static_assert(std::atomic_bool::is_always_lock_free);
std::mutex status_mutex;
std::string status_text = "Conectando; control OFF";
int worker_result = 0; 

void signal_handler(int) { stop_requested.store(true, std::memory_order_relaxed); }
void set_status(const std::string& text) {
    std::lock_guard<std::mutex> lock(status_mutex);
    status_text = text;
}

template <size_t N, size_t M>
void require_finite(const mat<N, M>& x, const char* name) {
    for (size_t r=0; r<N; ++r)
        for (size_t c=0; c<M; ++c)
            if (!std::isfinite(x[r][c])) throw std::runtime_error(name);
}

vec<7> normalize_pose(vec<7> pose) {
    require_finite(pose, "Pose no finita");
    double norm2=0;
    for (size_t i=3; i<7; ++i) norm2+=double(pose[i][0])*pose[i][0];
    if (norm2<1e-12) throw std::runtime_error("Cuaternion nulo");
    const float norm=static_cast<float>(std::sqrt(norm2));
    for (size_t i=3; i<7; ++i) pose[i][0]/=norm;
    return pose;
}

// [w x y z], R = Rz(yaw) Ry(pitch) Rx(roll).
std::array<float,4> quaternion_rpy(const vec<3>& rpy) {
    require_finite(rpy, "RPY no finito");
    const float a=rpy[0][0]*PI_LOCAL/360, b=rpy[1][0]*PI_LOCAL/360,
                c=rpy[2][0]*PI_LOCAL/360;
    const float cr=std::cos(a), sr=std::sin(a), cp=std::cos(b),
                sp=std::sin(b), cy=std::cos(c), sy=std::sin(c);
    return {cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy};
}

vec<7> local_reference(const vec<7>& initial) {
    vec<7> desired=normalize_pose(initial);
    const auto delta=quaternion_rpy(REFERENCE_MODE==ReferenceMode::RelativeOffset
                                  ? ORIENTATION_OFFSET_RPY : ABSOLUTE_RPY);
    if (REFERENCE_MODE==ReferenceMode::RelativeOffset) {
        for (size_t i=0; i<3; ++i) desired[i][0]+=POSITION_OFFSET[i][0];
        const float w=desired[3][0], x=desired[4][0], y=desired[5][0], z=desired[6][0];
        const auto [a,b,c,d]=delta;
        desired[3][0]=w*a-x*b-y*c-z*d;
        desired[4][0]=w*b+x*a+y*d-z*c;
        desired[5][0]=w*c-x*d+y*a+z*b;
        desired[6][0]=w*d+x*c-y*b+z*a;
    } else {
        for (size_t i=0; i<3; ++i) desired[i][0]=ABSOLUTE_POSITION[i][0];
        for (size_t i=0; i<4; ++i) desired[i+3][0]=delta[i];
    }
    return normalize_pose(desired);
}

struct TaskOutput {
    vec<7> error{}; // [p-pd; Q-Qd], NO vector eje-angulo
    vec<6> velocity{}, wrench{};
    vec<7> proportional{}, damping{}, torque{};
};

TaskOutput operational_pd(vec<7> actual, vec<7> desired,
                          const mat<6,7>& J, const vec<7>& dq) {
    actual=normalize_pose(actual); desired=normalize_pose(desired);
    require_finite(J, "Jacobiano no finito");
    require_finite(dq, "Velocidad no finita");
    // Q y -Q representan la misma orientacion: seleccionar hemisferio comun.
    float dot=0;
    for (size_t i=3; i<7; ++i) dot+=actual[i][0]*desired[i][0];
    if (dot<0) for (size_t i=3; i<7; ++i) desired[i][0]*=-1;
    const float eta=actual[3][0];
    const vec<3> beta={actual[4][0],actual[5][0],actual[6][0]};
    const auto U=cV(trans(-beta),eta*mat<3,3>::identity()-skew(beta));
    const mat<6,7> A=cV(cH(mat<3,3>::identity(),mat<3,4>::zeros()),
                       cH(mat<3,3>::zeros(),(TH_LOCAL*0.5f)*trans(U)));
    TaskOutput o;
    o.error=actual-desired;
    o.velocity=J*dq; // Jacobiano del modelo: primero lineal, despues angular
    // Misma estructura de calcTau, sustituyendo tanh(e/th) por e/th.

    const vec<6> force_p=-(K_LOCAL/TH_LOCAL)*(A*o.error);
    const vec<6> force_d=-D_LOCAL.hadamard(o.velocity);
    o.proportional=trans(J)*force_p;
    o.damping=trans(J)*force_d;
    o.wrench=force_p+force_d;
    o.torque=o.proportional+o.damping;

    require_finite(o.error, "Error no finito");
    require_finite(o.velocity, "Velocidad cartesiana no finita");
    require_finite(o.wrench, "Wrench no finito");
    require_finite(o.torque, "Torque operacional no finito");
    return o;
}

vec<7> limited_torque(vec<7> tau) {
    require_finite(tau, "Torque solicitado no finito");
    for (size_t i=0; i<7; ++i)
        tau[i][0]=std::clamp(tau[i][0],-KinovaGen3::maxTau[i][0],KinovaGen3::maxTau[i][0]);
    return tau;
}
vec<7> torque_to_current(const vec<7>& tau) {
    require_finite(tau, "Torque no finito");
    const auto current=KinovaGen3::KTGR.hadamard(tau);
    require_finite(current, "Corriente no finita");
    for (size_t i=0; i<7; ++i)
        if (std::abs(current[i][0])>KinovaGen3::maxCurrent[i][0])
            throw std::runtime_error("Corriente excede maxCurrent");
    return current;
}

// ================= OBSERVADOR DE FRICCION =================
// Misma matematica que Friction_Observer de main_op_space.cpp, por articulacion:
//   u    = | dq + (vs/sqrt2) * tanh(100 * tau^3) |
//   g(u) = FC + (FS - FC) * exp(-(u/vs)^2)
//   dz   = u - sigma0 * |u| * z / g(u)          (RK4, u y g(u) fijos en el paso)
//   FF   = sigma0 * z_nuevo + sigma1 * dz_anterior
//   tau_f = FRICTION_GAIN * tanh(FACTOR * tau^3) * |FF|
struct FrictionOutput {
    vec<7> FF{};     // Friccion estimada por el observador [Nm]
    vec<7> torque{}; // Compensacion aplicada (con signo del PD) [Nm]
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
        require_finite(dq, "dq no finito en friccion");
        require_finite(tau_ctrl, "Torque de control no finito en friccion");
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
        o.torque=smoothSign(tau_ctrl,FRICTION_SIGN_FACTOR).hadamard(abs(o.FF));
        require_finite(z, "Estado de friccion no finito");
        require_finite(o.FF, "Friccion estimada no finita");
        require_finite(o.torque, "Compensacion de friccion no finita");
        return o;
    }
};

// ================= IMPEDANCIA SHADOW =================
using Vec7E = CartesianImpedanceController::Vec7;
using Vec6E = CartesianImpedanceController::Vec6;
using Mat67E = CartesianImpedanceController::Mat67;

Vec7E to_eigen(const vec<7>& v) {
    Vec7E r; for (size_t i=0; i<7; ++i) r(Eigen::Index(i))=v[i][0]; return r;
}
Mat67E to_eigen(const mat<6,7>& J) {
    Mat67E r;
    for (size_t i=0; i<6; ++i)
        for (size_t j=0; j<7; ++j) r(Eigen::Index(i),Eigen::Index(j))=J[i][j];
    return r;
}
template <int N>
vec<N> to_mini(const Eigen::Matrix<double,N,1>& v) {
    vec<N> r{}; for (int i=0; i<N; ++i) r[i][0]=static_cast<float>(v(i)); return r;
}
Eigen::Vector3d position_of(const vec<7>& pose) {
    return {pose[0][0], pose[1][0], pose[2][0]};
}
Eigen::Quaterniond orientation_of(const vec<7>& pose) { // [w x y z]
    return Eigen::Quaterniond(pose[3][0], pose[4][0], pose[5][0], pose[6][0]).normalized();
}

// Configura la clase del proyecto con controller_config.hpp y la referencia local.
void setup_impedance(CartesianImpedanceController& c, const vec<7>& desired,
                     const vec<7>& q0) {
    const auto& cfg=IMPEDANCE_CFG;
    c.setCartesianStiffness(cfg.stiffness);
    c.setNullspaceStiffness(cfg.enable_nullspace ? cfg.nullspace_stiffness : 0.0);
    c.setDampingFactors(cfg.damping_factors,
                        cfg.enable_nullspace ? cfg.nullspace_damping_factor : 0.0);
    c.setFiltering(cfg.update_hz, cfg.pose_filter, cfg.stiffness_filter,
                   cfg.wrench_filter, cfg.nullspace_filter);
    c.setMaxTorqueDelta(cfg.delta_tau_max);
    c.setWrench(cfg.enable_wrench ? cfg.wrench : Vec6E::Zero());
    const Vec7E q0e=to_eigen(q0);
    c.initialize(position_of(desired), orientation_of(desired), q0e);
    c.setNullspaceReference(cfg.enable_nullspace ? Vec7E(q0e+cfg.nullspace_offset) : q0e);
}

struct ShadowOutput {
    vec<6> pose_error{};
    vec<7> task{}, null{}, ext{}, impedance{};
};

ShadowOutput shadow_impedance(CartesianImpedanceController& c, const vec<7>& q,
                              const vec<7>& dq, const vec<7>& pose, const mat<6,7>& J) {
    const auto out=c.compute(to_eigen(q), to_eigen(dq), position_of(pose),
                             orientation_of(pose), to_eigen(J));
    if (!out.tau_task.allFinite() || !out.tau_null.allFinite() ||
        !out.tau_ext.allFinite() || !out.tau_impedance.allFinite() ||
        !out.pose_error.allFinite())
        throw std::runtime_error("Impedancia shadow no finita");
    return {to_mini<6>(out.pose_error), to_mini<7>(out.tau_task), to_mini<7>(out.tau_null),
            to_mini<7>(out.tau_ext), to_mini<7>(out.tau_impedance)};
}

void gravity_transition(KinovaRobotLowLevel& robot) {
    for (int i=0; i<200 && !robot.timeout; ++i) {
        const auto q=robot.getPos();
        require_finite(q, "q no finito durante transicion");
        const auto tau=limited_torque(KinovaGen3::GravityVector(q));
        robot.sendCur(torque_to_current(tau));
    }
}

template <size_t N>
void csv_vector(std::ostream& file, const vec<N>& v) {
    for (size_t i=0; i<N; ++i) file << ',' << v[i][0];
}
void csv_names(std::ostream& file, const char* name, size_t n) {
    for (size_t i=0; i<n; ++i) file << ',' << name << '_' << i+1;
}

void run_control() {
    std::unique_ptr<KinovaRobotLowLevel> robot;
    try {
        // Abrir registro antes de conectar. Errores de disco se propagan.
        const auto stamp=std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::ofstream log("op_space_local_"+std::to_string(stamp)+".csv");
        log.exceptions(std::ios::failbit|std::ios::badbit);
        log << "t_s,on,hardware,blend";
        csv_names(log,"q_rad",7); csv_names(log,"dq_rad_s",7);
        csv_names(log,"pose_xyz_wxyz",7); csv_names(log,"desired_xyz_wxyz",7);
        csv_names(log,"error_xyz_quat",7); csv_names(log,"velocity_linear_angular",6);
        csv_names(log,"tau_task_raw_Nm",7); csv_names(log,"tau_task_limited_Nm",7);
        csv_names(log,"tau_gravity_Nm",7); csv_names(log,"tau_requested_Nm",7);
        csv_names(log,"tau_hw_Nm",7); csv_names(log,"current_cmd_A",7);
        csv_names(log,"shadow_pose_err_xyz_rot",6);
        csv_names(log,"shadow_tau_task_Nm",7); csv_names(log,"shadow_tau_null_Nm",7);
        csv_names(log,"shadow_tau_ext_Nm",7); csv_names(log,"shadow_tau_imp_Nm",7);
        csv_names(log,"shadow_tau_total_Nm",7); csv_names(log,"shadow_current_A",7);
        log << ",shadow_over_limits,friction_enabled";
        csv_names(log,"friction_FF_Nm",7); csv_names(log,"tau_friction_Nm",7);
        log << '\n' << std::setprecision(9);
        robot=std::make_unique<KinovaRobotLowLevel>(ROBOT_IP,"admin","admin");
        auto q=robot->getPos(); require_finite(q,"Estado inicial no finito");
        const auto desired=local_reference(KinovaGen3::GetPose(q));
        std::cout << "Referencia local [x y z qw qx qy qz]:\n" << desired << '\n';
        CartesianImpedanceController impedance;
        setup_impedance(impedance,desired,q);
        using enum kinova_robot::ControlMode;
        robot->setControlMode({CUR,CUR,CUR,CUR,CUR,CUR,CUR});
        gravity_transition(*robot);
        const auto start=Clock::now(); auto previous=start;
        double last_display=-1, blend=0; size_t cycle=0;
        FrictionObserver friction; bool was_on=false;
        
        while (!stop_requested.load(std::memory_order_relaxed) && !robot->timeout) {
            const auto now=Clock::now();
            const double t=std::chrono::duration<double>(now-start).count();
            const double dt=std::chrono::duration<double>(now-previous).count();
            previous=now;
            if (DURATION_S>0 && t>=DURATION_S) break;
            q=robot->getPos(); const auto dq=robot->getVel();
            require_finite(q,"q no finito"); require_finite(dq,"dq no finito");
            require_finite(robot->getCur(),"Corriente medida no finita");
            require_finite(robot->getTau(),"Torque medido no finito");

            // Sin envolver q: la cinematica trigonometrica admite vueltas completas.
            const auto pose=normalize_pose(KinovaGen3::GetPose(q));
            const auto J=KinovaGen3::GetJacobian(q);
            require_finite(J,"Jacobiano no finito");
            const auto g=KinovaGen3::GravityVector(q); require_finite(g,"Gravedad no finita");
            const bool on=control_on.load(std::memory_order_relaxed);
            TaskOutput task{};
            if (on) task=operational_pd(pose,desired,J,dq);
            // OFF no evalua el PD; sus campos de diagnostico se registran en cero.
            const auto tau_task=limited_torque(task.torque);

            // FRICCION: solo con ON. Al pasar OFF -> ON se reinicia el estado z.
            // La direccion se toma SOLO del resorte (J^T F_p), sin el amortiguamiento
            // -D J dq, igual que el original, cuyo calcTau no tenia amortiguamiento.
            FrictionOutput fric{};
            if (on && ENABLE_FRICTION_COMPENSATION) {
                if (!was_on) friction.reset();
                fric=friction.step(dq,task.proportional,FRICTION_DT);
            }
            was_on=on;
            const auto tau_ctrl=limited_torque(tau_task+fric.torque); // PD + friccion
            const auto requested=g+tau_ctrl;
            require_finite(requested,"Suma de torques no finita");

            // SHADOW: impedancia cartesiana calculada SIEMPRE (ON y OFF), nunca enviada.
            const auto shadow=shadow_impedance(impedance,q,dq,pose,J);
            const vec<7> shadow_total=g+shadow.impedance;
            const vec<7> shadow_current=KinovaGen3::KTGR.hadamard(shadow_total);
            require_finite(shadow_current,"Corriente shadow no finita");
            bool shadow_over=false; // Diagnostico: la corriente shadow excederia maxCurrent
            for (size_t i=0; i<7; ++i)
                shadow_over|=std::abs(shadow_current[i][0])>KinovaGen3::maxCurrent[i][0];

            if constexpr (ENABLE_PD_HARDWARE) {
                blend=on ? (BLEND_TIME_S>0 ? std::min(1.0,blend+dt/BLEND_TIME_S):1.0):0.0;
            } else { blend=0; }

            // Al robot va gravedad + blend * (PD + friccion). La impedancia shadow NO interviene.
            const auto tau_hw=limited_torque(g+static_cast<float>(blend)*tau_ctrl);
            const auto current=torque_to_current(tau_hw);
            robot->sendCur(current);
            if (cycle++%LOG_EVERY==0) {
                log << t << ',' << on << ',' << ENABLE_PD_HARDWARE << ',' << blend;
                csv_vector(log,q); csv_vector(log,dq); csv_vector(log,pose); csv_vector(log,desired);
                csv_vector(log,task.error); csv_vector(log,task.velocity);
                csv_vector(log,task.torque); csv_vector(log,tau_task); csv_vector(log,g);
                csv_vector(log,requested); csv_vector(log,tau_hw); csv_vector(log,current);
                csv_vector(log,shadow.pose_error); csv_vector(log,shadow.task);
                csv_vector(log,shadow.null); csv_vector(log,shadow.ext);
                csv_vector(log,shadow.impedance); csv_vector(log,shadow_total);
                csv_vector(log,shadow_current); log << ',' << shadow_over
                    << ',' << ENABLE_FRICTION_COMPENSATION;
                csv_vector(log,fric.FF); csv_vector(log,fric.torque); log << '\n';
            }
            if (t-last_display>=0.2) {
                last_display=t;
                std::ostringstream info;
                info << (ENABLE_PD_HARDWARE ? "HARDWARE: gravedad + PD cartesiano (ON)"
                                            : "PD solo calculado: hardware solo gravedad")
                     << (ENABLE_FRICTION_COMPENSATION ? " + friccion" : " (sin friccion)")
                     << "\nControl " << (on?"ON":"OFF") << " | blend=" << std::fixed
                     << std::setprecision(2) << blend << " | t=" << std::setprecision(3) << t
                     << " s\nPose actual [xyz wxyz]:\n" << pose
                     << "\nReferencia local [xyz wxyz]:\n" << desired
                     << "\nPD: torque sin saturar [Nm]:\n" << task.torque
                     << "\nFriccion compensada [Nm]:\n" << fric.torque
                     //<< "\n ERROR DE POSICION:  \n" << task.error 
                     << "\nSHADOW impedancia (no enviada)"
                     << (shadow_over ? " - EXCEDERIA maxCurrent" : "")
                     << "\nError [dx dy dz dRx dRy dRz]:\n" << shadow.pose_error*100
                     << "\ntau_imp [Nm]:\n" << shadow.impedance;
                set_status(info.str());
            }
            
        }
        if (robot->timeout) throw std::runtime_error("Timeout del robot");
        gravity_transition(*robot);
        robot.reset(); // Wrapper local restaura POS y single-level.
        log.flush();
    } catch (const std::exception& e) {
        worker_result=1; std::cerr << "Control detenido: " << e.what() << '\n';
    } catch (...) {
        worker_result=1; std::cerr << "Control detenido: excepcion desconocida\n";
    }
    if (robot) {
        try { gravity_transition(*robot); }
        catch (...) { std::cerr << "No se pudo completar la transicion de gravedad\n"; }
        robot.reset();
    }
    worker_done.store(true,std::memory_order_release);
}

gboolean switch_changed(GtkSwitch*, gboolean state, gpointer) {
    control_on.store(state,std::memory_order_relaxed); return FALSE;
}
void close_window(GtkButton*, gpointer window) { gtk_widget_destroy(GTK_WIDGET(window)); }
struct Display { GtkWidget* label=nullptr; GtkWidget* window=nullptr; guint timer=0; } display;
void window_destroyed(GtkWidget*, gpointer) {
    stop_requested.store(true);
    if (display.timer) { g_source_remove(display.timer); display.timer=0; }
}
gboolean refresh_display(gpointer) {
    if (worker_done.load(std::memory_order_acquire)) {
        display.timer=0; gtk_widget_destroy(display.window); return G_SOURCE_REMOVE;
    }
    std::lock_guard<std::mutex> lock(status_mutex);
    gtk_label_set_text(GTK_LABEL(display.label),status_text.c_str());
    return G_SOURCE_CONTINUE;
}
void activate(GtkApplication* app, gpointer) {
    display.window=gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(display.window),"Control operacional local - Gen3");
    gtk_container_set_border_width(GTK_CONTAINER(display.window),12);
    auto* box=gtk_box_new(GTK_ORIENTATION_VERTICAL,10);
    gtk_container_add(GTK_CONTAINER(display.window),box);
    auto* sw=gtk_switch_new(); gtk_switch_set_active(GTK_SWITCH(sw),FALSE);
    g_signal_connect(sw,"state-set",G_CALLBACK(switch_changed),nullptr);
    gtk_box_pack_start(GTK_BOX(box),gtk_label_new("Control Action (OFF = gravedad)"),FALSE,FALSE,0);
    gtk_box_pack_start(GTK_BOX(box),sw,FALSE,FALSE,0);
    display.label=gtk_label_new("Conectando; control OFF");
    gtk_box_pack_start(GTK_BOX(box),display.label,TRUE,TRUE,0);
    auto* quit=gtk_button_new_with_label("Quit");
    g_signal_connect(quit,"clicked",G_CALLBACK(close_window),display.window);
    gtk_box_pack_start(GTK_BOX(box),quit,FALSE,FALSE,0);
    g_signal_connect(display.window,"destroy",G_CALLBACK(window_destroyed),nullptr);
    display.timer=g_timeout_add(100,refresh_display,nullptr);
    gtk_widget_show_all(display.window);
}

int self_test();

int main(int argc,char** argv) {
    if (argc==2 && std::string(argv[1])=="--self-test") return self_test();
    std::signal(SIGINT,signal_handler); std::signal(SIGTERM,signal_handler);
    auto* app=gtk_application_new("org.kinova.opspacelocal",G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app,"activate",G_CALLBACK(activate),nullptr);
    std::thread worker(run_control);
    const int gui_result=g_application_run(G_APPLICATION(app),argc,argv);
    stop_requested.store(true);
    if (display.timer) g_source_remove(display.timer);
    worker.join(); g_object_unref(app);
    return worker_result ? worker_result : gui_result;
}

// Pruebas matematicas sin hardware; nunca construyen KinovaRobotLowLevel.
int self_test() {
    try {
        auto check=[](bool ok,const char* msg) { if(!ok) throw std::runtime_error(msg); };
        vec<7> pose={0,0,0,1,0,0,0}, desired=pose, dq{};
        mat<6,7> J{}; for(size_t i=0;i<6;++i) J[i][i]=1;
        auto out=operational_pd(pose,desired,J,dq);
        for(size_t i=0;i<7;++i) check(std::abs(out.torque[i][0])<1e-6,"Equilibrio");
        desired[0][0]=0.05f; out=operational_pd(pose,desired,J,dq);
        check(std::abs(out.torque[0][0]-K_LOCAL/TH_LOCAL*0.05f)<1e-5,"Signo traslacion");
        desired=pose; desired[3][0]=-1; out=operational_pd(pose,desired,J,dq);
        for(size_t i=0;i<7;++i) check(std::abs(out.torque[i][0])<1e-6,"Q y -Q");
        desired=pose; desired[3][0]=std::cos(0.1f); desired[6][0]=std::sin(0.1f);
        out=operational_pd(pose,desired,J,dq);
        check(out.torque[5][0]>0,"Signo orientacion deseada Z");
        desired=pose; for(size_t i=0;i<7;++i) dq[i][0]=0.1f*float(i+1);
        out=operational_pd(pose,desired,J,dq);

        float power=0; for(size_t i=0;i<7;++i) power+=dq[i][0]*out.damping[i][0];
        check(power<0,"Amortiguamiento disipativo");
        bool rejected=false;
        try { operational_pd(vec<7>{},pose,J,dq); } catch (...) { rejected=true; }
        check(rejected,"Rechazo cuaternion cero");
        auto invalid=pose; invalid[0][0]=std::nanf(""); rejected=false;
        try { operational_pd(invalid,pose,J,dq); } catch (...) { rejected=true; }
        check(rejected,"Rechazo NaN");

        const auto q_model=vec<7>{10,30,-20,45,15,-30,20}*(PI_LOCAL/180);
        const auto pose_model=normalize_pose(KinovaGen3::GetPose(q_model));
        out=operational_pd(pose_model,pose_model,KinovaGen3::GetJacobian(q_model),vec<7>{});
        for(size_t i=0;i<7;++i) check(std::abs(out.torque[i][0])<1e-5,"Equilibrio modelo Gen3");
        vec<7> large{}; for(size_t i=0;i<7;++i) large[i][0]=10000;
        const auto limited=limited_torque(large); const auto current=torque_to_current(limited);
        
        for(size_t i=0;i<7;++i) {
            check(limited[i][0]<=KinovaGen3::maxTau[i][0],"Saturacion torque");
            check(current[i][0]<=KinovaGen3::maxCurrent[i][0],"Limite corriente");
        }

        // Impedancia shadow: equilibrio y signo con el modelo Gen3.
        const auto J_model=KinovaGen3::GetJacobian(q_model);
        CartesianImpedanceController imp;
        setup_impedance(imp,pose_model,q_model);
        auto sh=shadow_impedance(imp,q_model,vec<7>{},pose_model,J_model);
        for(size_t i=0;i<7;++i) check(std::abs(sh.impedance[i][0])<1e-5,"Equilibrio impedancia");
        auto ref=pose_model; ref[0][0]+=0.05f; // referencia 5 cm adelante en X
        CartesianImpedanceController imp2;
        setup_impedance(imp2,ref,q_model);
        for(int k=0;k<200;++k) sh=shadow_impedance(imp2,q_model,vec<7>{},pose_model,J_model);
        check(std::abs(sh.pose_error[0][0]+0.05f)<1e-4,"Error impedancia X");
        // tau_task = J^T F  ->  F = (J J^T)^-1 J tau_task; debe empujar hacia +X.
        const Mat67E Je=to_eigen(J_model);
        Vec7E te; for(int i=0;i<7;++i) te(i)=sh.task[i][0];
        const Vec6E wrench=(Je*Je.transpose()).ldlt().solve(Je*te);
        check(wrench(0)>0,"Signo impedancia traslacion");

        // Friccion: signo del PD, magnitud acotada, cero sin accion y estado finito.
        vec<7> tau_pos{}, tau_neg{};
        for(size_t i=0;i<7;++i) { tau_pos[i][0]=0.3f; tau_neg[i][0]=-0.3f; }
        FrictionObserver fo;
        auto fr=fo.step(vec<7>{},tau_pos,FRICTION_DT);
        for(size_t i=0;i<7;++i) {
            check(fr.torque[i][0]>0,"Friccion signo +");
            check(std::abs(fr.torque[i][0]-FRICTION_GAIN*KinovaGen3::FC[i][0])<0.5f,
                  "Friccion inicial ~FC");
        }
        fo.reset(); fr=fo.step(vec<7>{},tau_neg,FRICTION_DT);
        for(size_t i=0;i<7;++i) check(fr.torque[i][0]<0,"Friccion signo -");
        fo.reset(); fr=fo.step(vec<7>{},vec<7>{},FRICTION_DT);
        for(size_t i=0;i<7;++i) check(fr.torque[i][0]==0,"Friccion sin accion");
        fo.reset(); vec<7> dq_move{};
        for(size_t i=0;i<7;++i) dq_move[i][0]=0.5f;
        for(int k=0;k<5000;++k) fr=fo.step(dq_move,tau_pos,FRICTION_DT);
        for(size_t i=0;i<7;++i)
            check(fr.torque[i][0]>0 && fr.torque[i][0]<=FRICTION_GAIN*KinovaGen3::FS[i][0]+0.5f,
                  "Friccion acotada en movimiento");
        rejected=false; auto dq_nan=dq_move; dq_nan[0][0]=std::nanf("");
        try { fo.step(dq_nan,tau_pos,FRICTION_DT); } catch (...) { rejected=true; }
        check(rejected,"Friccion rechaza NaN");
        const auto big=limited_torque(tau_pos*100.0f+fr.torque);
        for(size_t i=0;i<7;++i) check(std::abs(big[i][0])<=KinovaGen3::maxTau[i][0],"Saturacion PD+friccion");
        std::cout << "Pruebas de control operacional local: OK (sin hardware)\n";
        return 0;
    } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
