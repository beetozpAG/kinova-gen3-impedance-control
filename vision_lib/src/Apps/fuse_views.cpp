// fuse_views: block 4 of the task. Joins the views of a sweep (block 3,
// sweep_capture) into one colored cloud in the DEPTH frame of view 0
// (Task/fuse_views.h), and saves it so GPD can use it like a capture:
//   FOLDER/fused.pcd        the cloud (x y z rgb)
//   FOLDER/fused_pose.txt   copy of view_0_pose.txt (pose and color image of view 0)
// No robot needed.
//
// Usage (run from kinova_grasp/build/):
//   ./fuse_views FOLDER            e.g. ./fuse_views sweep_2410_200944
//   ./fuse_views FOLDER COLOR      also counts the points of COLOR
//                                  (cfg/plate_colors.cfg) in view 0 and fused
// GPD (block 5) must take the color of each point from fused.pcd, not from
// the image of view 0 (see Task/README.md, block 4): not done yet.
//
// Steps (sub-blocks of fuse_views.h):
//   1.  Read the views.
//   4a-4c. Fuse.
//   2.  Save, and with COLOR count the object points.

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include "Robot/Gen3_model.h"  // only for TOOL_OFFSET_Z_2F85 (0.120 m)
#include "0_Input/color_cloud.h"
#include "1_PreprocessCloud/crop_table.h"
#include "2_GetROI/color_mask.h"
#include "2_GetROI/largest_cluster.h"
#include "Task/fuse_views.h"

// countColor: number of points of the color, and of its largest group.
static void countColor(const pcl::PointCloud<pcl::PointXYZRGB> &colored, const ColorRange &range,
                       const char *label) {
  pcl::PointCloud<pcl::PointXYZ> xyz;
  pcl::copyPointCloud(colored, xyz);
  const std::vector<int> mask = colorMask(colored, range);
  printf("  %s: %zu points of the color\n", label, mask.size());
  largestCluster(xyz, mask);
}

int main(int argc, char *argv[]) {
  if (argc != 2 && argc != 3) {
    std::cout << "Usage: fuse_views FOLDER [COLOR]   (FOLDER from sweep_capture)\n"
              << "Example (from build/): ./fuse_views sweep_2410_200944 blue\n";
    return 1;
  }
  const std::string folder = argv[1];

  // ---------- 1. Read the views ----------
  std::vector<SweepView> views;
  for (int k = 0; std::filesystem::exists(folder + "/view_" + std::to_string(k) + ".pcd"); k++) {
    SweepView view;
    if (!readSweepView(folder, k, view)) {
      return 1;
    }
    views.push_back(view);
  }
  if (views.size() < 2) {
    std::cout << "Error: fewer than 2 views in " << folder << "/\n";
    return 1;
  }

  // ---------- 4a-4c. Fuse ----------
  std::vector<size_t> added;
  const pcl::PointCloud<pcl::PointXYZRGB> fused =
      fuseViews(views, KinovaGen3::TOOL_OFFSET_Z_2F85, added);

  // ---------- 2. Save ----------
  const std::string pcd_file = folder + "/fused.pcd";
  const std::string pose_file = folder + "/fused_pose.txt";
  if (pcl::io::savePCDFileBinary(pcd_file, fused) != 0) {
    std::cout << "Error: could not write " << pcd_file << "\n";
    return 1;
  }
  std::filesystem::copy_file(folder + "/view_0_pose.txt", pose_file,
                             std::filesystem::copy_options::overwrite_existing);

  std::cout << "\n========== FUSION ==========\n";
  for (size_t k = 0; k < views.size(); k++) {
    printf("  view_%zu: %zu points%s\n", k, added[k],
           k == 0 ? " (all, above the table)" : " added (cells empty until then)");
  }
  printf("  fused:  %zu points\n", fused.size());
  std::cout << "Saved: " << pcd_file << ", " << pose_file << "\n";

  if (argc == 3) {
    ColorRange range;
    if (readColorRange(PLATE_COLORS_FILE, argv[2], range)) {
      std::cout << "Points of " << argv[2] << ":\n";
      // View 0 above the table, as in the fusion.
      const Eigen::Matrix4d T_base_depth0 =
          intToBase(toolToBase(views[0].tool), KinovaGen3::TOOL_OFFSET_Z_2F85) * depthToInt();
      const pcl::PointCloud<pcl::PointXYZRGB> view0 =
          colorCloud(cropBelowTable(views[0].cloud, T_base_depth0), views[0].color_image,
                     views[0].color_intr);
      countColor(view0, range, "view_0 alone");
      countColor(fused, range, "fused");
    }
  }
  std::cout << "============================\n";
  return 0;
}
