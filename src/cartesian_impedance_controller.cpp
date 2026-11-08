#include "cartesian_impedance_controller.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace impedance
{

CartesianImpedanceController::CartesianImpedanceController()
{
    Vec6 k;
    k << 200, 200, 200, 20, 20, 20;
    setCartesianStiffness(k);
    setNullspaceStiffness(0);
    setDampingFactors(Vec6::Ones(), 1);
    K_ = K_target_;
    D_ = D_target_;
}

void CartesianImpedanceController::initialize(const Eigen::Vector3d& p, const Eigen::Quaterniond& R, const Vec7& q)
{
    p_d_ = p_d_target_ = p_ramp_ = p;
    R_d_ = R_d_target_ = R_ramp_ = R.normalized();
    q_null_ = q_null_target_ = q;
    tau_last_.setZero();

    // Arrancar con la rigidez/amortiguamiento configurados (setters llamados
    // antes de initialize), no con los 200/20 del constructor filtrados hacia
    // ellos. El filtro sigue suavizando los cambios posteriores.
    K_ = K_target_;
    D_ = D_target_;
    K_null_ = K_null_target_;
    D_null_ = D_null_target_;
}

void CartesianImpedanceController::setReferencePose(const Eigen::Vector3d& p, const Eigen::Quaterniond& R)
{
    p_d_target_ = p;
    R_d_target_ = R.normalized();
}

void CartesianImpedanceController::setNullspaceReference(const Vec7& q)
{
    q_null_target_ = q;
}

void CartesianImpedanceController::setCartesianStiffness(const Vec6& k)
{
    if ((k.array() < 0).any())
        throw std::invalid_argument("negative stiffness");

    K_target_.setZero();
    for (int i = 0; i < 6; ++i)
        K_target_(i, i) = k(i);

    for (int i = 0; i < 6; ++i)
        D_target_(i, i) = damping_factors_(i) * dampingRule(K_target_(i, i));
}

void CartesianImpedanceController::setCartesianStiffnessNow(const Vec6& k)
{
    setCartesianStiffness(k);
    K_ = K_target_;
    D_ = D_target_;
}

CartesianImpedanceController::Vec6 CartesianImpedanceController::nearTargetStiffness(
    const Vec6& base, double near, double start_m, double full_m, double dist_m)
{
    double s = (start_m - dist_m) / (start_m - full_m);
    s = std::isfinite(s) ? std::clamp(s, 0.0, 1.0) : 0.0;

    Vec6 k = base;
    for (int i = 0; i < 3; ++i)
        k(i) = std::max(base(i), base(i) + s * (near - base(i)));
    return k;
}

void CartesianImpedanceController::setNullspaceStiffness(double k)
{
    if (k < 0)
        throw std::invalid_argument("negative nullspace stiffness");

    K_null_target_ = k;
    D_null_target_ = damping_null_factor_ * dampingRule(k);
}

void CartesianImpedanceController::setNullspaceWeights(const Vec7& w)
{
    if (!w.allFinite() || (w.array() < 0).any())
        throw std::invalid_argument("bad nullspace weights");

    null_weights_ = w;
}

void CartesianImpedanceController::setNullspaceMaxTorque(double t)
{
    if (!std::isfinite(t) || t < 0)
        throw std::invalid_argument("bad nullspace max torque");

    null_max_torque_ = t;
}

void CartesianImpedanceController::setJointSoftLimit(const JointSoftLimit& l)
{
    if (l.joint < 0 || l.joint > 6 || !std::isfinite(l.min_rad) ||
        !std::isfinite(l.stiffness) || l.stiffness < 0 ||
        !std::isfinite(l.damping) || l.damping < 0 ||
        !std::isfinite(l.damping_ramp_rad) || l.damping_ramp_rad <= 0 ||
        !std::isfinite(l.max_torque) || l.max_torque <= 0)
        throw std::invalid_argument("bad joint soft limit");

    joint_limit_ = l;
}

void CartesianImpedanceController::setDampingFactors(const Vec6& d, double dn)
{
    if ((d.array() < 0).any() || dn < 0)
        throw std::invalid_argument("negative damping");

    damping_factors_ = d;
    damping_null_factor_ = dn;

    D_target_.setZero();
    for (int i = 0; i < 6; ++i)
        D_target_(i, i) = d(i) * dampingRule(K_target_(i, i));

    D_null_target_ = dn * dampingRule(K_null_target_);
}

void CartesianImpedanceController::setWrench(const Vec6& w)
{
    wrench_target_ = w;
}

void CartesianImpedanceController::setFiltering(double hz, double p, double k, double w, double n)
{
    auto ok = [](double x) { return x > 0 && x <= 1; };

    if (hz <= 0 || !ok(p) || !ok(k) || !ok(w) || !ok(n))
        throw std::invalid_argument("bad filter config");

    update_hz_ = hz;
    pose_filter_ = p;
    stiffness_filter_ = k;
    wrench_filter_ = w;
    nullspace_filter_ = n;
}

void CartesianImpedanceController::setReferenceSpeedLimit(double v, double w)
{
    if (!std::isfinite(v) || !std::isfinite(w) || v < 0 || w < 0)
        throw std::invalid_argument("bad reference speed limit");

    ramp_linear_m_s_ = v;
    ramp_angular_rad_s_ = w;
}

void CartesianImpedanceController::setMaxTorqueDelta(double d)
{
    if (d < 0)
        throw std::invalid_argument("negative delta_tau");

    delta_tau_max_ = d;
}

Eigen::Vector3d CartesianImpedanceController::orientationError(const Eigen::Quaterniond& des, Eigen::Quaterniond cur)
{
    if (des.coeffs().dot(cur.coeffs()) < 0)
        cur.coeffs() = -cur.coeffs();

    Eigen::Quaterniond qe = cur * des.inverse();
    Eigen::AngleAxisd aa(qe);

    if (!std::isfinite(aa.angle()) || std::abs(aa.angle()) < 1e-12)
        return Eigen::Vector3d::Zero();

    return aa.axis() * aa.angle();
}

double CartesianImpedanceController::dampingRule(double k)
{
    return 2 * std::sqrt(k);
}

double CartesianImpedanceController::filterStep(double hz, double f)
{
    if (f >= 1)
        return 1;

    f = std::clamp(f, 1e-9, 0.999999);
    double k = -1.0 / std::log(1 - f);
    return 1.0 / (k * hz + 1.0);
}

Eigen::MatrixXd CartesianImpedanceController::pseudoInverse(const Eigen::MatrixXd& M)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto s = svd.singularValues();

    Eigen::VectorXd inv = s;
    double mx = s.size() ? s.array().abs().maxCoeff() : 0;
    double tol = 1e-6 * std::max(M.rows(), M.cols()) * mx;

    for (Eigen::Index i = 0; i < s.size(); ++i)
        inv(i) = std::abs(s(i)) > tol ? 1.0 / s(i) : 0;

    return svd.matrixV() * inv.asDiagonal() * svd.matrixU().adjoint();
}

void CartesianImpedanceController::updateFilteredTargets()
{
    double ap = filterStep(update_hz_, pose_filter_);
    double ak = filterStep(update_hz_, stiffness_filter_);
    double aw = filterStep(update_hz_, wrench_filter_);
    double an = filterStep(update_hz_, nullspace_filter_);

    // Rampa: avanza hacia el objetivo lo que permiten las velocidades
    // maximas en este ciclo. La fraccion f es la misma para posicion y
    // orientacion, asi que ambas llegan a la vez y la posicion va en linea
    // recta a velocidad constante.
    if (ramp_linear_m_s_ > 0 && ramp_angular_rad_s_ > 0)
    {
        const double dt = 1.0 / update_hz_;
        const Eigen::Vector3d dp = p_d_target_ - p_ramp_;
        const double t_needed = std::max(dp.norm() / ramp_linear_m_s_,
                                         R_ramp_.angularDistance(R_d_target_) / ramp_angular_rad_s_);
        if (t_needed <= dt)
        {
            p_ramp_ = p_d_target_;
            R_ramp_ = R_d_target_;
        }
        else
        {
            const double f = dt / t_needed;
            p_ramp_ += f * dp;
            R_ramp_ = R_ramp_.slerp(f, R_d_target_).normalized();
        }
    }
    else
    {
        p_ramp_ = p_d_target_;
        R_ramp_ = R_d_target_;
    }

    p_d_ = (1 - ap) * p_d_ + ap * p_ramp_;
    R_d_ = R_d_.slerp(ap, R_ramp_);

    K_ = (1 - ak) * K_ + ak * K_target_;
    D_ = (1 - ak) * D_ + ak * D_target_;
    K_null_ = (1 - ak) * K_null_ + ak * K_null_target_;
    D_null_ = (1 - ak) * D_null_ + ak * D_null_target_;

    wrench_ = (1 - aw) * wrench_ + aw * wrench_target_;
    q_null_ = (1 - an) * q_null_ + an * q_null_target_;
}

CartesianImpedanceController::Output CartesianImpedanceController::compute(
    const Vec7& q, const Vec7& dq, const Eigen::Vector3d& p, Eigen::Quaterniond R, const Mat67& J)
{
    updateFilteredTargets();
    R.normalize();

    Output o;
    o.pose_error.head<3>() = p - p_d_;
    o.pose_error.tail<3>() = orientationError(R_d_, R);

    Vec6 xdot = J * dq;
    o.tau_spring = J.transpose() * (-K_ * o.pose_error);
    o.tau_task = J.transpose() * (-K_ * o.pose_error - D_ * xdot);

    Eigen::MatrixXd pinv = pseudoInverse(J.transpose());
    Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - J.transpose() * pinv;
    // Error articular del espacio nulo. En las articulaciones de giro continuo
    // se envuelve a [-pi, pi]: 359 deg y 1 deg difieren 2 deg, no 358 deg.
    Vec7 q_err = q_null_ - q;
    for (int i = 0; i < 7; ++i)
        if (continuous_joints_[i])
            q_err(i) = std::remainder(q_err(i), 2.0 * M_PI);

    o.tau_null = N * (K_null_ * null_weights_.cwiseProduct(q_err) - D_null_ * dq);

    if (null_max_torque_ > 0)
    {
        double mx = o.tau_null.cwiseAbs().maxCoeff();
        if (mx > null_max_torque_)
            o.tau_null *= null_max_torque_ / mx;
    }

    o.tau_ext = J.transpose() * wrench_;

    if (joint_limit_.enabled)
    {
        const int j = joint_limit_.joint;
        const double pen = joint_limit_.min_rad - q(j);
        if (pen > 0)
        {
            const double r = std::min(1.0, pen / joint_limit_.damping_ramp_rad);
            const double t = joint_limit_.stiffness * pen - r * joint_limit_.damping * dq(j);
            o.tau_limit(j) = std::clamp(t, 0.0, joint_limit_.max_torque);
        }
    }

    Vec7 req = o.tau_task + o.tau_null + o.tau_ext + o.tau_limit;
    Vec7 sat = tau_last_;
    for (int i = 0; i < 7; ++i)
        sat(i) += std::clamp(req(i) - tau_last_(i), -delta_tau_max_, delta_tau_max_);

    tau_last_ = sat;
    o.tau_impedance = sat;

    return o;
}

}  // namespace impedance