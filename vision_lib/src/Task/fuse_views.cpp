// File:    fuse_views.cpp
// Module:  Task (block 4: fusion)
// Purpose: Joins the views of a sweep. See fuse_views.h.
#include "Task/fuse_views.h"

#include <cmath>
#include <iostream>
#include <unordered_set>

#include <pcl/io/pcd_io.h>

#include "0_Input/capture_file.h"
#include "0_Input/color_cloud.h"
#include "1_PreprocessCloud/crop_table.h"

// readSweepView: the same files as a capture of grasps_from_camera.
bool readSweepView(const std::string &folder, int k, SweepView &view) {
  const std::string name = folder + "/view_" + std::to_string(k);
  if (pcl::io::loadPCDFile(name + ".pcd", view.cloud) != 0) {
    std::cout << "Error: could not read " << name << ".pcd\n";
    return false;
  }
  double q_deg[7];
  if (!readPoseFile(name + "_pose.txt", view.tool, q_deg)) {
    std::cout << "Error: could not read a tool_pose line in " << name << "_pose.txt\n";
    return false;
  }
  if (!readColorCapture(name + "_pose.txt", view.color_intr, view.color_image)) {
    std::cout << "Error: no color image in " << name << "_pose.txt\n";
    return false;
  }
  return true;
}

// cellKey: the 3 mm cell of a point seen from above (XY of BASE).
static long long cellKey(const Eigen::Vector3d &p) {
  const long long i = std::llround(p.x() / FUSE_CELL);
  const long long j = std::llround(p.y() / FUSE_CELL);
  return i * 1000003LL + j;
}

// fuseViews: 4a for each view, 4b cell by cell, 4c back to DEPTH of view 0.
pcl::PointCloud<pcl::PointXYZRGB> fuseViews(const std::vector<SweepView> &views,
                                            double ee_offset_z, std::vector<size_t> &added) {
  pcl::PointCloud<pcl::PointXYZRGB> fused;
  added.assign(views.size(), 0);
  if (views.empty()) {
    return fused;
  }

  // T_base_depth of each view (frames.h).
  std::vector<Eigen::Matrix4d> T_base_depth(views.size());
  for (size_t k = 0; k < views.size(); k++) {
    T_base_depth[k] = intToBase(toolToBase(views[k].tool), ee_offset_z) * depthToInt();
  }
  const Eigen::Matrix4d T_depth0_base = T_base_depth[0].inverse();

  std::unordered_set<long long> filled;
  for (size_t k = 0; k < views.size(); k++) {
    // 4a. Above the table, colored with its own image.
    const pcl::PointCloud<pcl::PointXYZ> above = cropBelowTable(views[k].cloud, T_base_depth[k]);
    const pcl::PointCloud<pcl::PointXYZRGB> colored =
        colorCloud(above, views[k].color_image, views[k].color_intr);

    // 4b. View 0 whole; the other views only in empty cells. The cells of
    // a view are marked after it, so its own points do not block each other.
    std::vector<long long> new_cells;
    for (const pcl::PointXYZRGB &p : colored) {
      const Eigen::Vector3d base = (T_base_depth[k] * Eigen::Vector4d(p.x, p.y, p.z, 1.0)).head<3>();
      const long long key = cellKey(base);
      if (k > 0 && filled.count(key) > 0) {
        continue;
      }
      new_cells.push_back(key);

      // 4c. Back to the DEPTH frame of view 0.
      const Eigen::Vector3d d0 = (T_depth0_base * base.homogeneous()).head<3>();
      pcl::PointXYZRGB q = p;
      q.x = static_cast<float>(d0.x());
      q.y = static_cast<float>(d0.y());
      q.z = static_cast<float>(d0.z());
      fused.push_back(q);
      added[k]++;
    }
    filled.insert(new_cells.begin(), new_cells.end());
  }
  return fused;
}
