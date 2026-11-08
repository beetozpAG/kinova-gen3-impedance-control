#pragma once

#include <Eigen/Dense>

namespace impedance
{

class CartesianImpedanceController
{
public:
    using Vec7 = Eigen::Matrix<double, 7, 1>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat67 = Eigen::Matrix<double, 6, 7>;
    using Mat66 = Eigen::Matrix<double, 6, 6>;

    struct Output
    {
        Vec7 tau_task = Vec7::Zero();
        // Solo el resorte de la tarea: J^T (-K * pose_error), sin -D J dq.
        // Informativo (ya incluido en tau_task); da la direccion a la friccion.
        Vec7 tau_spring = Vec7::Zero();
        Vec7 tau_null = Vec7::Zero();
        // Limite suave de articulacion (setJointSoftLimit); 0 fuera del limite.
        Vec7 tau_limit = Vec7::Zero();
        Vec7 tau_ext = Vec7::Zero();
        Vec7 tau_impedance = Vec7::Zero();

        Vec6 pose_error = Vec6::Zero();
    };

    // Limite suave inferior de UNA articulacion, actua directo en ella (no
    // pasa por el espacio nulo). Si q(joint) < min_rad:
    //   tau = clamp(K * (min - q) - r * D * dq, 0, max_torque)
    // r sube de 0 a 1 en los primeros damping_ramp_rad (sin escalon al
    // entrar). Solo empuja hacia afuera del limite, nunca hacia adentro.
    struct JointSoftLimit
    {
        bool enabled = false;
        int joint = 3;                 // 0..6 (3 = J4)
        double min_rad = 0.0;
        double stiffness = 0.0;        // Nm/rad
        double damping = 0.0;          // Nms/rad
        double damping_ramp_rad = 0.035;
        double max_torque = 0.0;       // Nm
    };

    CartesianImpedanceController();

    void initialize(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Vec7& q);

    void setReferencePose(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation);

    void setNullspaceReference(
        const Vec7& q_null);

    void setCartesianStiffness(
        const Vec6& stiffness);

    // Como setCartesianStiffness pero SIN stiffness_filter: K y D cambian en
    // este ciclo. Solo para cambios que ya son suaves por si solos (rigidez
    // cerca del objetivo, que depende de la distancia). El torque sigue
    // limitado por setMaxTorqueDelta.
    void setCartesianStiffnessNow(
        const Vec6& stiffness);

    // Rigidez cerca del objetivo: la de TRASLACION (x, y, z) vale base lejos
    // (dist >= start_m), near cerca (dist <= full_m) y sube lineal entre los
    // dos. Nunca baja de base; la rotacion queda igual. dist no finita = base.
    static Vec6 nearTargetStiffness(
        const Vec6& base,
        double near,
        double start_m,
        double full_m,
        double dist_m);

    void setNullspaceStiffness(
        double stiffness);

    // Peso de cada articulacion en el resorte de postura (>= 0). 0 = esa
    // articulacion no jala hacia su referencia. El amortiguamiento del
    // espacio nulo se aplica a todas.
    void setNullspaceWeights(
        const Vec7& weights);

    // Tope de |tau_null| por articulacion [Nm] (0 = sin tope). Si se pasa,
    // se escala TODO el vector (misma direccion, sigue en el espacio nulo).
    void setNullspaceMaxTorque(
        double max_torque);

    void setJointSoftLimit(
        const JointSoftLimit& limit);

    void setDampingFactors(
        const Vec6& cartesian_factors,
        double nullspace_factor);

    void setWrench(
        const Vec6& wrench);

    void setFiltering(
        double update_hz,
        double pose_filter,
        double stiffness_filter,
        double wrench_filter,
        double nullspace_filter);

    void setMaxTorqueDelta(
        double delta_tau_max);

    // Rampa de la referencia: la referencia va en LINEA RECTA hacia el
    // objetivo de setReferencePose a velocidad constante (lineal [m/s] y
    // angular [rad/s], sincronizadas: posicion y orientacion llegan juntas)
    // y se detiene justo en el. Despues pasa por pose_filter (1 = sin filtro).
    // 0 en cualquiera de las dos = sin rampa (salto al objetivo).
    void setReferenceSpeedLimit(
        double max_linear_m_s,
        double max_angular_rad_s);

    Output compute(
        const Vec7& q,
        const Vec7& dq,
        const Eigen::Vector3d& position,
        Eigen::Quaterniond orientation,
        const Mat67& J);

private:
    static Eigen::Vector3d orientationError(
        const Eigen::Quaterniond& desired,
        Eigen::Quaterniond current);

    static double dampingRule(
        double stiffness);

    static double filterStep(
        double update_hz,
        double filter);

    static Eigen::MatrixXd pseudoInverse(
        const Eigen::MatrixXd& matrix);

    void updateFilteredTargets();

private:
    Mat66 K_ = Mat66::Zero();
    Mat66 K_target_ = Mat66::Zero();

    Mat66 D_ = Mat66::Zero();
    Mat66 D_target_ = Mat66::Zero();

    Vec6 damping_factors_ =
        Vec6::Ones();

    double K_null_ = 0.0;
    double K_null_target_ = 0.0;

    double D_null_ = 0.0;
    double D_null_target_ = 0.0;

    double damping_null_factor_ = 1.0;

    Vec7 null_weights_ =
        Vec7::Ones();

    double null_max_torque_ = 0.0;

    JointSoftLimit joint_limit_{};

    Eigen::Vector3d p_d_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d p_d_target_ =
        Eigen::Vector3d::Zero();

    Eigen::Quaterniond R_d_ =
        Eigen::Quaterniond::Identity();

    Eigen::Quaterniond R_d_target_ =
        Eigen::Quaterniond::Identity();

    // Referencia de la rampa (entre el objetivo y pose_filter).
    Eigen::Vector3d p_ramp_ =
        Eigen::Vector3d::Zero();

    Eigen::Quaterniond R_ramp_ =
        Eigen::Quaterniond::Identity();

    double ramp_linear_m_s_ = 0.0;
    double ramp_angular_rad_s_ = 0.0;

    Vec7 q_null_ =
        Vec7::Zero();

    Vec7 q_null_target_ =
        Vec7::Zero();

    Vec6 wrench_ =
        Vec6::Zero();

    Vec6 wrench_target_ =
        Vec6::Zero();

    Vec7 tau_last_ =
        Vec7::Zero();

    double update_hz_ = 1000.0;

    double pose_filter_ = 0.1;
    double stiffness_filter_ = 0.1;
    double wrench_filter_ = 0.1;
    double nullspace_filter_ = 0.1;

    double delta_tau_max_ = 1.0;

    // Articulaciones de giro continuo (vueltas infinitas) del Kinova Gen3
    // 7 DoF: J1, J3, J5, J7 (guia de usuario, tabla 38). En ellas el error
    // del espacio nulo se toma por el camino mas corto, [-pi, pi].
    // J2, J4, J6 tienen limites (+-128.9, +-147.8, +-120.3 deg): su error se
    // deja sin envolver para no empujar a traves del limite.
    static constexpr bool continuous_joints_[7] =
        {true, false, true, false, true, false, true};
};

}  // namespace impedance
