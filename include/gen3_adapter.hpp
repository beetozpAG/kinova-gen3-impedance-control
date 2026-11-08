#pragma once

#include "Gen3_model.h"
#include "Matrix.h"

#include <Eigen/Dense>

namespace gen3_adapter
{

using Vec7d = Eigen::Matrix<double, 7, 1>;
using Mat67d = Eigen::Matrix<double, 6, 7>;

struct CartesianPose
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

inline matrix::vec<7> wrapJointAngles(matrix::vec<7> q)
{
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;

    for (int i = 0; i < 7; ++i)
    {
        if (q[i][0] > PI)
        {
            q[i][0] -= TWO_PI;
        }
    }

    return q;
}

inline Vec7d toEigen(const matrix::vec<7>& vector)
{
    Vec7d result;

    for (int i = 0; i < 7; ++i)
    {
        result(i) =
            static_cast<double>(
                vector[i][0]);
    }

    return result;
}

inline matrix::vec<7> toMini(const Vec7d& vector)
{
    matrix::vec<7> result{};

    for (int i = 0; i < 7; ++i)
    {
        result[i][0] =
            static_cast<float>(
                vector(i));
    }

    return result;
}

inline Mat67d jacobian(const matrix::vec<7>& q)
{
    const auto J_model =
        KinovaGen3::GetJacobian(q);

    Mat67d J;

    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 7; ++col)
        {
            J(row, col) =
                static_cast<double>(
                    J_model[row][col]);
        }
    }

    return J;
}

inline CartesianPose pose(
    const matrix::vec<7>& q,
    const matrix::vec<7>& last_pose =
        matrix::vec<7>{})
{
    const auto model_pose =
        KinovaGen3::GetPose(
            q,
            last_pose);

    CartesianPose result;

    result.position <<
        model_pose[0][0],
        model_pose[1][0],
        model_pose[2][0];

    // GetPose:
    // [x, y, z, qw, qx, qy, qz]
    result.orientation =
        Eigen::Quaterniond(
            model_pose[3][0],
            model_pose[4][0],
            model_pose[5][0],
            model_pose[6][0]);

    result.orientation.normalize();

    return result;
}

inline matrix::vec<7> poseToMini(
    const CartesianPose& pose)
{
    matrix::vec<7> result{};

    result[0][0] =
        static_cast<float>(
            pose.position.x());

    result[1][0] =
        static_cast<float>(
            pose.position.y());

    result[2][0] =
        static_cast<float>(
            pose.position.z());

    result[3][0] =
        static_cast<float>(
            pose.orientation.w());

    result[4][0] =
        static_cast<float>(
            pose.orientation.x());

    result[5][0] =
        static_cast<float>(
            pose.orientation.y());

    result[6][0] =
        static_cast<float>(
            pose.orientation.z());

    return result;
}

}  // namespace gen3_adapter
