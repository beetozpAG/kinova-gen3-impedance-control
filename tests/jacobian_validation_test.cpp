#include "Gen3_model.h"

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace matrix;
using namespace KinovaGen3;

int main()
{
    std::cout << "\n========================================\n";
    std::cout << "KINOVA GEN3 JACOBIAN VALIDATION TEST\n";
    std::cout << "========================================\n\n";

    constexpr float deg2rad =
        3.14159265358979323846f / 180.0f;

    // ========================================================
    // TEST CONFIGURATION
    // ========================================================

    mat<7, 1> q =
    {
         10.0f * deg2rad,
         30.0f * deg2rad,
        -20.0f * deg2rad,
         45.0f * deg2rad,
         15.0f * deg2rad,
        -30.0f * deg2rad,
         20.0f * deg2rad
    };

    // Finite difference perturbation
    constexpr float h = 1e-4f;

    // Maximum allowed error [m/rad]
    constexpr float tolerance = 1e-3f;

    std::cout << "Testing configuration q [rad]:\n";
    std::cout << q << "\n";

    // ========================================================
    // 1. ANALYTIC JACOBIAN
    // ========================================================

    mat<6, 7> J = GetJacobian(q);

    std::cout << "\n========================================\n";
    std::cout << "1. ANALYTIC TRANSLATIONAL JACOBIAN\n";
    std::cout << "========================================\n\n";

    for (std::size_t r = 0; r < 3; ++r)
    {
        for (std::size_t c = 0; c < 7; ++c)
        {
            std::cout
                << std::setw(14)
                << J[r][c]
                << " ";
        }

        std::cout << "\n";
    }

    // ========================================================
    // 2. NUMERICAL JACOBIAN
    // ========================================================

    mat<3, 7> J_numeric{};

    for (std::size_t i = 0; i < 7; ++i)
    {
        mat<7, 1> q_plus  = q;
        mat<7, 1> q_minus = q;

        q_plus[i][0]  += h;
        q_minus[i][0] -= h;

        const mat<7, 1> pose_plus =
            GetPose(q_plus);

        const mat<7, 1> pose_minus =
            GetPose(q_minus);

        // Only x, y, z
        for (std::size_t r = 0; r < 3; ++r)
        {
            J_numeric[r][i] =
                (
                    pose_plus[r][0]
                    -
                    pose_minus[r][0]
                )
                /
                (2.0f * h);
        }
    }

    std::cout << "\n========================================\n";
    std::cout << "2. NUMERICAL TRANSLATIONAL JACOBIAN\n";
    std::cout << "========================================\n\n";

    for (std::size_t r = 0; r < 3; ++r)
    {
        for (std::size_t c = 0; c < 7; ++c)
        {
            std::cout
                << std::setw(14)
                << J_numeric[r][c]
                << " ";
        }

        std::cout << "\n";
    }

    // ========================================================
    // 3. ERROR MATRIX
    // ========================================================

    mat<3, 7> error_matrix{};

    float max_error = 0.0f;
    bool passed = true;

    std::cout << "\n========================================\n";
    std::cout << "3. ABSOLUTE ERROR\n";
    std::cout << "========================================\n\n";

    for (std::size_t r = 0; r < 3; ++r)
    {
        for (std::size_t c = 0; c < 7; ++c)
        {
            const float error =
                std::abs(
                    J[r][c]
                    -
                    J_numeric[r][c]
                );

            error_matrix[r][c] = error;

            if (error > max_error)
            {
                max_error = error;
            }

            if (error > tolerance)
            {
                passed = false;
            }

            std::cout
                << std::setw(14)
                << error
                << " ";
        }

        std::cout << "\n";
    }

    // ========================================================
    // 4. ERROR PER JOINT
    // ========================================================

    std::cout << "\n========================================\n";
    std::cout << "4. MAX ERROR PER JOINT\n";
    std::cout << "========================================\n\n";

    for (std::size_t c = 0; c < 7; ++c)
    {
        float joint_max_error = 0.0f;

        for (std::size_t r = 0; r < 3; ++r)
        {
            if (error_matrix[r][c] > joint_max_error)
            {
                joint_max_error =
                    error_matrix[r][c];
            }
        }

        std::cout
            << "Joint "
            << c + 1
            << ": "
            << joint_max_error
            << "\n";
    }

    // ========================================================
    // 5. TEST SUMMARY
    // ========================================================

    std::cout << "\n========================================\n";
    std::cout << "TEST SUMMARY\n";
    std::cout << "========================================\n\n";

    std::cout
        << "Finite difference h: "
        << h
        << " rad\n";

    std::cout
        << "Tolerance:           "
        << tolerance
        << " m/rad\n";

    std::cout
        << "Maximum error:       "
        << max_error
        << " m/rad\n";

    std::cout << "\n";

    if (passed)
    {
        std::cout
            << "JACOBIAN VALIDATION PASSED.\n";

        std::cout
            << "Analytic translational Jacobian "
               "is consistent with FK.\n";

        std::cout
            << "========================================\n";

        return 0;
    }

    std::cout
        << "JACOBIAN VALIDATION FAILED.\n";

    std::cout
        << "Analytic Jacobian and FK differ "
           "more than the allowed tolerance.\n";

    std::cout
        << "========================================\n";

    return 1;
}