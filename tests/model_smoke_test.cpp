#include "Gen3_model.h"
#include "gen3_adapter.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

using namespace matrix;
using namespace KinovaGen3;

namespace
{

bool isFinite(const vec<7>& v)
{
    for (int i = 0; i < 7; ++i)
    {
        if (!std::isfinite(v[i][0]))
        {
            return false;
        }
    }

    return true;
}

bool isFinite(const mat<6, 7>& M)
{
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 7; ++c)
        {
            if (!std::isfinite(M[r][c]))
            {
                return false;
            }
        }
    }

    return true;
}

bool quaternionIsNormalized(const vec<7>& pose)
{
    const double qw = pose[3][0];
    const double qx = pose[4][0];
    const double qy = pose[5][0];
    const double qz = pose[6][0];

    const double norm =
        std::sqrt(
            qw * qw +
            qx * qx +
            qy * qy +
            qz * qz);

    std::cout
        << "Quaternion norm: "
        << norm
        << '\n';

    return std::abs(norm - 1.0) < 1e-3;
}

void printSeparator()
{
    std::cout
        << "\n========================================\n";
}

}  // namespace

int main()
{
    printSeparator();

    std::cout
        << "KINOVA GEN3 ANALYTIC MODEL SMOKE TEST\n";

    printSeparator();

    // ------------------------------------------------------------
    // 1. Test joint configuration
    //
    // IMPORTANT:
    // The model expects radians.
    // ------------------------------------------------------------
    constexpr float deg2rad =
        3.14159265358979323846f / 180.0f;

    vec<7> q =
    {
        10.0f * deg2rad,
        30.0f * deg2rad,
        -20.0f * deg2rad,
        45.0f * deg2rad,
        15.0f * deg2rad,
        -30.0f * deg2rad,
        20.0f * deg2rad
    };

    std::cout
        << "Testing configuration q [rad]:\n"
        << q
        << '\n';

    // ------------------------------------------------------------
    // 2. Forward kinematics / pose
    // ------------------------------------------------------------
    printSeparator();

    std::cout
        << "1. FORWARD KINEMATICS\n\n";

    const auto pose =
        GetPose(q);

    std::cout
        << "Pose [x y z qw qx qy qz]:\n"
        << pose
        << '\n';

    const bool pose_finite =
        isFinite(pose);

    const bool quat_normalized =
        quaternionIsNormalized(pose);

    // ------------------------------------------------------------
    // 3. Jacobian
    // ------------------------------------------------------------
    printSeparator();

    std::cout
        << "2. JACOBIAN\n\n";

    const auto J =
        GetJacobian(q);

    std::cout
        << "Jacobian 6x7:\n"
        << J
        << '\n';

    const bool jacobian_finite =
        isFinite(J);

    // ------------------------------------------------------------
    // 4. Gravity vector
    // ------------------------------------------------------------
    printSeparator();

    std::cout
        << "3. GRAVITY VECTOR\n\n";

    const auto gravity =
        GravityVector(q);

    std::cout
        << "g(q) [Nm]:\n"
        << gravity
        << '\n';

    const bool gravity_finite =
        isFinite(gravity);

    // ------------------------------------------------------------
    // 5. Model torque limits
    // ------------------------------------------------------------
    printSeparator();

    std::cout
        << "4. MODEL TORQUE LIMITS\n\n";

    std::cout
        << "maxTau [Nm]:\n"
        << maxTau
        << '\n';

    const bool max_tau_finite =
        isFinite(maxTau);

    // ------------------------------------------------------------
    // 6. Maximum current
    // ------------------------------------------------------------
    printSeparator();

    std::cout
        << "5. MOTOR CURRENT LIMITS\n\n";

    std::cout
        << "maxCurrent [A]:\n"
        << maxCurrent
        << '\n';

    const bool max_current_finite =
        isFinite(maxCurrent);

    // ------------------------------------------------------------
    // 7. Torque -> current conversion test
    //
    // If tau = g(q), then:
    //
    //     I = KTGR .* g(q)
    //
    // This is exactly the conversion used by the lab controller.
    // ------------------------------------------------------------
    printSeparator();

    std::cout
        << "6. GRAVITY TORQUE -> MOTOR CURRENT\n\n";

    const auto gravity_current =
        KTGR.hadamard(
            gravity);

    std::cout
        << "I_gravity [A]:\n"
        << gravity_current
        << '\n';

    bool current_inside_limits = true;

    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(gravity_current[i][0]) >
            maxCurrent[i][0])
        {
            current_inside_limits = false;

            std::cout
                << "WARNING: actuator "
                << i + 1
                << " gravity current exceeds maxCurrent.\n";
        }
    }

    // ------------------------------------------------------------
    // 8. Summary
    // ------------------------------------------------------------
    printSeparator();

    std::cout
        << "TEST SUMMARY\n\n";

    std::cout
        << "Pose finite:          "
        << (pose_finite ? "PASS" : "FAIL")
        << '\n';

    std::cout
        << "Quaternion normalized:"
        << (quat_normalized ? " PASS" : " FAIL")
        << '\n';

    std::cout
        << "Jacobian finite:      "
        << (jacobian_finite ? "PASS" : "FAIL")
        << '\n';

    std::cout
        << "Gravity finite:       "
        << (gravity_finite ? "PASS" : "FAIL")
        << '\n';

    std::cout
        << "maxTau finite:        "
        << (max_tau_finite ? "PASS" : "FAIL")
        << '\n';

    std::cout
        << "maxCurrent finite:    "
        << (max_current_finite ? "PASS" : "FAIL")
        << '\n';

    std::cout
        << "Gravity current safe: "
        << (current_inside_limits ? "PASS" : "FAIL")
        << '\n';

    printSeparator();

    const bool all_ok =
        pose_finite &&
        quat_normalized &&
        jacobian_finite &&
        gravity_finite &&
        max_tau_finite &&
        max_current_finite &&
        current_inside_limits;

    if (!all_ok)
    {
        std::cout
            << "SMOKE TEST FAILED.\n";

        return 1;
    }

    std::cout
        << "SMOKE TEST PASSED.\n";

    return 0;
}