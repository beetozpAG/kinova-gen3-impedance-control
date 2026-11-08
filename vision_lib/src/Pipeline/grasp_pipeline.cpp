// grasp_pipeline: GPD steps shared by grasps_from_file and
// grasps_from_camera.
#include "Pipeline/grasp_pipeline.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <gpd/candidate/hand_geometry.h>  // GPD: reads the hand geometry .cfg
#include <gpd/grasp_detector.h>           // GPD: GraspDetector (the whole pipeline)
#include <gpd/util/cloud.h>               // GPD: point cloud + camera position
#include <gpd/util/config_file.h>         // GPD: reads .cfg files

#include "2_GetROI/color_mask.h"
#include "2_GetROI/plate_edge.h"
#include "2_GetROI/plate_edge_image.h"
#include "3_Sample/contour_directions.h"
#include "3_Sample/contour_frames.h"
#include "6_SelectGrasp/grasp_to_ee.h"
#include "6_SelectGrasp/plate_center.h"

// Returns true if the file can be opened.
static bool fileExists(const std::string &file_name) {
  std::ifstream file(file_name);
  return file.good();
}

bool readGpdSetup(const std::string &config_filename, GpdSetup &setup) {
  if (!fileExists(config_filename)) {
    std::cout << "Error: config file not found: " << config_filename << "\n";
    return false;
  }
  gpd::util::ConfigFile config_file(config_filename);
  config_file.ExtractKeys();

  setup.hand_geometry_filename = config_file.getValueOfKeyAsString("hand_geometry_filename", "");
  if (!fileExists(setup.hand_geometry_filename)) {
    std::cout << "Error: hand geometry file not found: " << setup.hand_geometry_filename << "\n";
    return false;
  }
  const gpd::candidate::HandGeometry hand_geometry(setup.hand_geometry_filename);
  setup.hand_depth = hand_geometry.depth_;
  setup.hand = HandSize{hand_geometry.finger_width_, hand_geometry.outer_diameter_,
                        hand_geometry.depth_, hand_geometry.height_};

  // Same as atenpas detect_grasps.cpp:48-52.
  std::vector<double> camera_position =
      config_file.getValueOfKeyAsStdVectorDouble("camera_position", "0 0 0");
  setup.view_points.resize(3, 1);
  setup.view_points << camera_position[0], camera_position[1], camera_position[2];

  // Our key (GPD does not read it): frames from the outline of the object.
  setup.contour_frames = config_file.getValueOfKey<bool>("contour_frames", false);
  std::vector<double> direction =
      config_file.getValueOfKeyAsStdVectorDouble("direction", "0 0 1");
  setup.approach_direction = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
  // Our key (GPD does not read it): grasp nearest to the plate middle first.
  setup.select_plate_center = config_file.getValueOfKey<bool>("select_plate_center", false);
  // Our key (GPD does not read it): grasp nearest to the object middle first.
  setup.select_object_center = config_file.getValueOfKey<bool>("select_object_center", false);
  return true;
}

// Run GPD on a cloud (same as atenpas detect_grasps.cpp:69-83).
static bool runDetector(const std::string &config_filename, gpd::util::Cloud &cloud,
                        Grasps &grasps) {
  if (cloud.getCloudOriginal()->size() == 0) {
    std::cout << "Error: the point cloud is empty.\n";
    return false;
  }
  gpd::GraspDetector detector(config_filename);
  detector.preprocessPointCloud(cloud);
  grasps = detector.detectGrasps(cloud);
  return true;
}

bool detectGraspsInCloud(const std::string &config_filename,
                         const pcl::PointCloud<pcl::PointXYZ> &points,
                         const GpdSetup &setup, Grasps &grasps,
                         const Eigen::Matrix3Xd &samples) {
  // GPD works with colored points (PointXYZRGBA): copy x, y, z (no color).
  gpd::util::PointCloudRGB::Ptr points_rgb(new gpd::util::PointCloudRGB);
  pcl::copyPointCloud(points, *points_rgb);

  // All points come from one camera (camera 0): same as when GPD loads a
  // .pcd file (cloud.cpp:115).
  const Eigen::MatrixXi camera_source = Eigen::MatrixXi::Ones(1, points_rgb->size());
  gpd::util::Cloud cloud(points_rgb, camera_source, setup.view_points);

  // Given samples: GPD keeps them instead of choosing its own. It still takes
  // num_samples of them at random (cloud.cpp:350-357, 372-393) and drops
  // those outside the workspace (cloud.cpp:224-240).
  if (samples.cols() > 0) {
    cloud.setSamples(samples);
  }

  // Closing directions from the outline of the samples, seen along the
  // approach direction; cleared after GPD, so other runs keep the original
  // frames.
  std::vector<Eigen::Vector3d> directions;
  if (setup.contour_frames && samples.cols() > 0) {
    std::vector<double> widths;
    directions = contourDirections(samples, setup.approach_direction, widths);
    printf("Contour frames: %zu closing direction(s), width", directions.size());
    for (double w : widths) {
      printf(" %.1f", w * 1000.0);
    }
    printf(" mm%s\n", directions.empty() ? " (round outline: GPD frames)" : "");
  }
  if (setup.contour_frames && samples.cols() == 0) {
    std::cout << "Warning: contour_frames = 1 needs the object (--color); GPD uses its own\n"
                 "frames, with num_orientations of the cfg (set it back to 8).\n";
  }
  setContourDirections(directions);
  const bool ok = runDetector(config_filename, cloud, grasps);
  setContourDirections({});
  return ok;
}

std::string takeColorOption(int &argc, char *argv[]) {
  for (int i = 1; i + 1 < argc; i++) {
    if (std::string(argv[i]) == "--color") {
      const std::string name = argv[i + 1];
      for (int j = i; j + 2 < argc; j++) {
        argv[j] = argv[j + 2];
      }
      argc -= 2;
      return name;
    }
  }
  return "";
}

bool plateEdgeOfGrasps(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                       const Intrinsics &color_intr, const std::string &color_name,
                       const Eigen::Matrix3Xd &plate_points, const GpdSetup &setup,
                       Grasps &grasps, PlateEdge &edge) {
  ColorRange range;
  const bool from_image =
      readColorRange(PLATE_COLORS_FILE, color_name, range) &&
      measurePlateEdgeImage(cloud, color_image, color_intr, range, edge);
  if (!from_image) {
    std::cout << "Plate edge measured from the depth points instead.\n";
    if (!measurePlateEdge(plate_points, edge)) {
      return false;
    }
  }
  if (setup.select_plate_center) {
    sortByPlateCenter(grasps, edge);
    std::cout << "Grasps ordered by distance to the middle of the plate edge, nearest first\n"
                 "(select_plate_center = 1).\n";
  } else if (setup.select_object_center) {
    sortByObjectCenter(grasps, edge.center);
    std::cout << "Grasps ordered by distance to the middle of the object, nearest first\n"
                 "(select_object_center = 1).\n";
  }
  return true;
}

// printPlateEdgeOfGrasps: GPD gives the position of each grasp at the base
// of the fingers, in the middle between them; the hand comes across the
// edge, so its place along the edge is the same for any point of the hand.
void printPlateEdgeOfGrasps(const PlateEdge &edge, const Grasps &grasps,
                            const Eigen::Matrix4d &T_base_depth) {
  std::vector<Eigen::Vector3d> positions;
  for (size_t i = 0; i < grasps.size() && i < GRASPS_PRINTED; i++) {
    positions.push_back(grasps[i]->getPosition());
  }
  printPlateEdge(edge, T_base_depth, positions);
}

void printEEPoses(const Grasps &grasps, const GpdSetup &setup,
                  const Eigen::Matrix4d &T_base_int, const Eigen::Matrix4d &T_base_ee,
                  const Eigen::Vector3d &ee_angles_now_deg, double ee_offset_z,
                  const Eigen::Matrix3Xd &object_points,
                  const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  std::cout << "hand_depth = " << setup.hand_depth << " m (" << setup.hand_geometry_filename
            << ")\n";
  std::cout << "Number of grasps: " << grasps.size() << " (the first "
            << std::min(grasps.size(), GRASPS_PRINTED) << " printed)\n";

  for (size_t i = 0; i < grasps.size() && i < GRASPS_PRINTED; i++) {
    const gpd::candidate::Hand &g = *grasps[i];
    const CenteredGrasp centered = centerGrasp(g, object_points, cloud, setup.hand);
    const EEPose ee = graspToEE(g, centered.position, setup.hand_depth, T_base_int,
                                T_base_ee.block<3, 3>(0, 0), ee_angles_now_deg, ee_offset_z);
    // Gripper opening: width of the object between the fingers, measured by
    // GPD in the closing direction (hand_set.cpp:244-247). In % of the 2F-85
    // maximum opening, in both conventions of the Kinova user guide r12.0:
    // 0% = closed (p.155, p.157) and 0% = open (Actions page, p.175-176).
    // The gripper must open more than this to approach, then close past it.
    const double width = g.getGraspWidth();
    const double open_percent = 100.0 * width / GRIPPER_MAX_OPENING;
    std::cout << "\nGrasp " << i << "  (score " << g.getScore() << ")\n"
              << "  gripper opening:    " << width << " m = " << std::fixed
              << std::setprecision(0) << open_percent << "% open / "
              << 100.0 - open_percent << "% closed (2F-85 max "
              << GRIPPER_MAX_OPENING * 1000.0 << " mm)\n"
              << "  closing advance:    " << std::setprecision(1) << ee.tip_advance * 1000.0
              << " mm (finger tips move forward when closing; EE moved back this much)\n"
              << std::defaultfloat << std::setprecision(6)
              << "  1. pre-grasp (m):   x = " << ee.pregrasp_position.x()
              << "  y = " << ee.pregrasp_position.y() << "  z = " << ee.pregrasp_position.z()
              << "   (" << PREGRASP_DISTANCE * 100.0 << " cm back along the approach)\n"
              << "  2. EE position (m): x = " << ee.position.x()
              << "  y = " << ee.position.y() << "  z = " << ee.position.z() << "\n"
              << "  EE orientation (deg): tx = " << ee.theta_x << "  ty = " << ee.theta_y
              << "  tz = " << ee.theta_z << "   (same for 1 and 2)\n";
    if (centered.moved) {
      printf("  centered on the object: moved %.1f mm along the closing direction\n",
             centered.shift * 1000.0);
    } else if (object_points.cols() > 0) {
      printf("  not centered (a finger would hit a point there): GPD position\n");
    }
  }
  if (!grasps.empty()) {
    std::cout << "\nINT -> base of GPD fingers used: "
              << FINGERTIP_FROM_INT - setup.hand_depth << " m (" << FINGERTIP_FROM_INT
              << " - hand_depth)\n";
  }
  std::cout << "==========================================\n";
}
