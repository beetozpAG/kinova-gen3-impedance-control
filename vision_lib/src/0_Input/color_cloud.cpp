// File:    color_cloud.cpp
// Module:  0_Input
// Purpose: Gives each point of the depth cloud the color of its pixel in the
//          color image. See color_cloud.h for the steps and their limits.
#include "0_Input/color_cloud.h"

#include <cmath>

#include "0_Input/depth_camera.h"
#include "0_Input/frames.h"

// DepthToColor: T_color_depth = T_int_color^-1 * T_int_depth is only a
// translation (frames.h).
DepthToColor::DepthToColor(const Intrinsics &color_intr)
    : intr_(color_intr),
      t_color_depth_((colorToInt().inverse() * depthToInt()).block<3, 1>(0, 3)),
      sign_(FLIP_XY ? -1.0 : 1.0) {}

// pixel: point in COLOR, then the inverse of x = sign * (z / fx) * (col - cx).
bool DepthToColor::pixel(const Eigen::Vector3d &p_depth, int &col, int &row, double &z) const {
  const Eigen::Vector3d q = p_depth + t_color_depth_;
  if (q.z() <= 0.0) {
    return false;
  }
  col = static_cast<int>(std::lround(intr_.cx + sign_ * intr_.fx * q.x() / q.z()));
  row = static_cast<int>(std::lround(intr_.cy + sign_ * intr_.fy * q.y() / q.z()));
  z = q.z();
  return true;
}

// toDepth: x = sign * (z / fx) * (col - cx), same for y; then COLOR -> DEPTH.
Eigen::Vector3d DepthToColor::toDepth(double col, double row, double z) const {
  const Eigen::Vector3d q(sign_ * (col - intr_.cx) * z / intr_.fx,
                          sign_ * (row - intr_.cy) * z / intr_.fy, z);
  return q - t_color_depth_;
}

// colorCloud: for each point, its pixel (DepthToColor), then the color of
// that pixel.
pcl::PointCloud<pcl::PointXYZRGB> colorCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                             const cv::Mat &color_image,
                                             const Intrinsics &color_intr)
{
    const DepthToColor to_color(color_intr);

    pcl::PointCloud<pcl::PointXYZRGB> colored;
    colored.resize(cloud.size());
    for (size_t i = 0; i < cloud.size(); i++)
    {
        const pcl::PointXYZ &p = cloud[i];
        pcl::PointXYZRGB &q = colored[i];
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.r = NO_COLOR_RGB[0];
        q.g = NO_COLOR_RGB[1];
        q.b = NO_COLOR_RGB[2];

        // 1-2. Pixel of the point in the color image.
        int col, row;
        double z;
        if (!to_color.pixel(Eigen::Vector3d(p.x, p.y, p.z), col, row, z) || col < 0 ||
            col >= color_image.cols || row < 0 || row >= color_image.rows)
        {
            continue;
        }

        // 3. Color of the pixel (OpenCV stores B, G, R).
        const cv::Vec3b &bgr = color_image.at<cv::Vec3b>(row, col);
        q.b = bgr[0];
        q.g = bgr[1];
        q.r = bgr[2];
    }
    return colored;
}
