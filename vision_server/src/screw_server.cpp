// screw_server (MODULAR): vision server for the task sequence of the
// impedance controller (Controlador_Impedancia, steps visionCapture /
// visionCapturePlate / visionCaptureHole).
//
// Version for kinova_grasp_NEW_VER (Antony, 2026-10-24): the same server and
// protocol as kinova_grasp_V2/src/screw_server.cpp, built on the new library
// (src/0_Input ... src/6_SelectGrasp). It lives in MODULAR/servidor_vision so
// Antony's repository is not changed; it is built in
// kinova_grasp_NEW_VER/build_modular (see ../README.md).
//
// What changed with the new library:
//   - GPD runs on the cloud without the points below the table
//     (1_PreprocessCloud/crop_table) and, by default, on a depth image
//     filtered over time (1_PreprocessCloud/depth_filter, about 1 s with the
//     robot still; --no-filter: one image, as before).
//   - Antony's procedure (2026-10-27), done here with the TOOL pose sent by
//     the controller (approach_object and grasps_from_camera read the robot's
//     own TOOL pose, which is stale while the controller runs):
//       1. "APPROACH ... PLATE|BOLT" = approach_object COLOR: centroid of the
//          region of the color, EE pose with the color camera APPROACH_HEIGHT
//          above it looking down; MOVE (the controller goes there and asks
//          again) or IN VIEW (the whole region is in the color image).
//       2. "CAPTURE ... PLATE|BOLT" = grasps_from_camera CFG --color COLOR
//          --filter: plate -> --plate-config (gpd_plate_params.cfg) and
//          --plate-color (blue); bolt -> CONFIG_FILE (gpd_params.cfg) and
//          --bolt-color (red). GPD only on the object of that color, each
//          grasp centered on it (6_SelectGrasp/center_grasp).
//     Before each color photo the color camera focuses (FOCUS NOW, as the
//     button of the web app; --no-focus to skip it).
//   - DEPTH -> INT comes from the camera calibration file (frames.h,
//     depthToInt); its values are the manual ones until a calibration.
//
// Usage (run from kinova_grasp_NEW_VER/build_modular/):
//   ./screw_server ../cfg/gpd_params.cfg [OPTIONS]                robot at 192.168.1.10
//   ./screw_server ../cfg/gpd_params.cfg <ip> [OPTIONS]           robot at another IP
//   ./screw_server ../cfg/gpd_params.cfg --pcd FILE.pcd [OPTIONS] test without robot or
//                                                                 camera: every request uses FILE
// OPTIONS: --plate-config FILE   --plate-color NAME (blue)   --bolt-color NAME (red)
//          --no-filter   --no-focus   --plate-length L (plate edge, m; 0.150)
//          --table-z Z (table top in BASE [m];
//          default TABLE_Z_BASE of 1_PreprocessCloud/crop_table.h)
//
// Protocol: text, one UDP datagram per message, port 5006 on 127.0.0.1.
//   controller -> server:  "PING"                          answer "PONG"
//   GUI -> server:         "COLORS plate bolt"             colors of the plate and the bolt
//                              (cfg/plate_colors.cfg) for the next requests, without
//                              restarting; answer "COLORS OK plate bolt" or
//                              "COLORS FAIL reason" (the colors do not change)
//   controller -> server:  "APPROACH seq x y z tx ty tz PLATE|BOLT"
//   server -> controller:  "APPROACH seq OK STATE x y z tx ty tz cx cy cz n"
//                              STATE = MOVE, INVIEW or ATTARGET (at the target
//                              but the region still touches the image border);
//                              EE target pose (m, deg) and centroid of the
//                              object in BASE [m], n points of the object
//                          "APPROACH seq FAIL reason"
//   controller -> server:  "CAPTURE seq x y z tx ty tz [PLATE|BOLT]"
//                              TOOL pose when the image is taken (m, deg x-y-z
//                              extrinsic); PLATE / BOLT: object and its color
//                              (see above). Without it: screws, whole cloud.
//   server -> controller:  "SCREW seq OK x y z tx ty tz px py pz score n EDGE ax ay az bx by bz"
//                              grasp EE position in BASE [m], its orientation
//                              (deg x-y-z extrinsic) and pre-grasp position [m];
//                              then the two ends of the plate edge at the grasp
//                              in BASE [m], or "NOEDGE" if there is no clear
//                              edge (screws) (see measureEdgeEnds)
//                          "SCREW seq FAIL reason"
//   controller -> server:  "CLOUD seq x y z tx ty tz"      only the point cloud, NO GPD
//                                                          (photo of the plate holes)
//   server -> controller:  "CLOUD seq OK name n HOLE x y z d k"
//                              saved name.pcd and name_pose.txt (hoyo_HHMMSS,
//                              n points); center of the SMALLEST plate hole in
//                              BASE [m] and its diameter [m], k holes found
//                              (see detectHoles), or "... NOHOLE"
//                          "CLOUD seq FAIL reason"
// The answer goes to the address the request came from.
//
// The TOOL pose comes from the controller, computed with its model from the
// measured joint angles (in low-level servoing the robot stops updating its
// own TOOL pose feedback), so this program does not use captureFromRobot. It
// connects only once at startup (RobotConnection) to read the camera
// parameters; the images come from the camera video streams.
// The program ONLY READS from the robot: it never moves it.
//
// GPD plot windows (plot_... in the config) close by themselves after
// GPD_PLOT_SECONDS (default 2 s; src/gpd_plot_2f85_auto.cpp).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/io/pcd_io.h>

#include <opencv2/imgcodecs.hpp>

#include "Robot/Gen3_model.h"  // only for TOOL_OFFSET_Z_2F85 (0.120 m)
#include "0_Input/capture_file.h"
#include "0_Input/color_camera.h"
#include "0_Input/depth_camera.h"
#include "0_Input/frames.h"
#include "0_Input/vision_module.h"
#include "1_PreprocessCloud/crop_table.h"
#include "1_PreprocessCloud/depth_filter.h"
#include "2_GetROI/color_mask.h"        // readColorRange (COLORS)
#include "2_GetROI/object_position.h"
#include "3_Sample/plate_samples.h"
#include "6_SelectGrasp/center_grasp.h"
#include "6_SelectGrasp/grasp_to_ee.h"
#include "Pipeline/grasp_pipeline.h"
#include "Robot/robot_connection.h"
#include "Task/approach_pose.h"

#include <DeviceManagerClientRpc.h>
#include <VisionConfigClientRpc.h>
#include <chrono>
#include <thread>

const int DEFAULT_PORT = 5006;  // --port N for a test next to a running server

volatile std::sig_atomic_t stop_requested = 0;

void signalHandler(int) { stop_requested = 1; }

// Depth limit of the hole photos (CLOUD) [m], as MAX_DEPTH of
// kinova_grasp_V2: without it the table fills the cloud and the plane fit of
// detectHoles could take the table instead of the plate (the new library
// keeps every depth and crops with the GPD workspace, which CLOUD does not use).
const double HOLE_MAX_DEPTH = 0.34;

// Read the parameters of both cameras once. The connection closes at the end
// of this function (Kortex allows one client of each service per session).
bool readIntrinsicsFromRobot(const std::string &ip_address, Intrinsics &intr,
                             Intrinsics &color_intr) {
  printf("Connecting to the robot at %s to read the camera parameters (read only)...\n",
         ip_address.c_str());
  RobotConnection robot(ip_address);
  if (!robot.isConnected() || !readIntrinsics(robot.router(), intr, color_intr)) {
    return false;
  }
  printf("Depth camera: %dx%d  fx = %.3f  fy = %.3f  cx = %.3f  cy = %.3f\n", intr.width,
         intr.height, intr.fx, intr.fy, intr.cx, intr.cy);
  return true;
}

// Split a line into words (spaces).
std::vector<std::string> words(const std::string &text) {
  std::vector<std::string> out;
  size_t i = 0;
  while (i < text.size()) {
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) i++;
    const size_t start = i;
    while (i < text.size() && !std::isspace(static_cast<unsigned char>(text[i]))) i++;
    if (i > start) out.push_back(text.substr(start, i - start));
  }
  return out;
}

// Number from text, independent of the locale. false if it is not a number.
template <typename T>
bool number(const std::string &text, T &out) {
  const auto r = std::from_chars(text.data(), text.data() + text.size(), out);
  return r.ec == std::errc() && r.ptr == text.data() + text.size();
}

// Number to text with "." always (independent of the locale).
std::string text(double value) {
  char buf[32];
  const auto r = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::fixed, 5);
  return std::string(buf, r.ptr);
}

// Name for a saved capture, from the current time: screw_HHMMSS (or
// prefix_HHMMSS).
std::string captureName(const std::string &prefix = "screw") {
  const std::time_t now = std::time(nullptr);
  char hhmmss[16];
  std::strftime(hhmmss, sizeof(hhmmss), "%H%M%S", std::localtime(&now));
  return prefix + "_" + hhmmss;
}

// Saves the cloud and the TOOL pose (same files as capture_pcd, without
// joint angles). false if the .pcd could not be written.
// Same _pose.txt format as kinova_grasp_V2 (the GUI reads it); with a color
// image (plate color), also name_color.png.
bool saveServerCapture(const std::string &name, const pcl::PointCloud<pcl::PointXYZ> &cloud,
                       const ToolPose &tool, const cv::Mat &color_image = cv::Mat(),
                       const Intrinsics *color_intr = nullptr) {
  if (pcl::io::savePCDFileBinary(name + ".pcd", cloud) != 0) {
    return false;
  }
  const bool has_color = !color_image.empty() && color_intr != nullptr &&
                         cv::imwrite(name + "_color.png", color_image);
  FILE *file = std::fopen((name + "_pose.txt").c_str(), "w");
  if (file != nullptr) {
    std::fprintf(file,
                 "# Robot state when %s.pcd was taken (screw_server, pose sent by the "
                 "controller).\n"
                 "# TOOL pose measured by the robot: x y z [m], theta_x theta_y theta_z "
                 "[deg, x-y-z extrinsic]\n"
                 "tool_pose = %s %s %s %s %s %s\n",
                 name.c_str(), text(tool.x).c_str(), text(tool.y).c_str(), text(tool.z).c_str(),
                 text(tool.theta_x).c_str(), text(tool.theta_y).c_str(),
                 text(tool.theta_z).c_str());
    if (has_color) {
      // Same lines as saveCapture (depth_camera.cpp): readColorCapture reads them.
      std::fprintf(file,
                   "# Color camera: width height fx fy cx cy [pixels], and its image\n"
                   "color_intrinsics = %d %d %.3f %.3f %.3f %.3f\ncolor_image = %s_color.png\n",
                   color_intr->width, color_intr->height, color_intr->fx, color_intr->fy,
                   color_intr->cx, color_intr->cy, name.c_str());
    }
    std::fclose(file);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Plate edge (2026-10-20). A standing plate seen from above: its top edge is
// a thin straight strip. Where along that edge does the best grasp hold it?
// The controller uses the two edge ends to place the plate in the fixture at
// the same position whatever the grasp point. Same algorithm and constants as
// Controlador_Impedancia/gui/modular_gui/plate_edge.py (keep both equal):
//   1. Edge band: points within EDGE_RADIUS (horizontal) of the grasp whose
//      height is within EDGE_BAND of the top (99.5 percentile).
//   2. Grasp point on the edge: where the gripper axis (approach) crosses
//      the edge height (a tilted gripper has its TCP off the edge).
//   3. Edge line: RANSAC in the horizontal plane with point pairs within
//      EDGE_SEED_R of that point, inliers within EDGE_SLAB; direction by PCA.
//   4. Walk the line from the grasp point both ways until a gap > EDGE_GAP:
//      those are the plate ends.
//   5. An end on the image border means the plate continues out of the image:
//      the answer is NOEDGE (the controller cannot center the plate).
// Only measures: it does not change the grasp.
const double EDGE_RADIUS = 0.30;
const double EDGE_BAND = 0.015;
const double EDGE_SLAB = 0.006;
const double EDGE_SEED_R = 0.03;
const double EDGE_GAP = 0.008;
const double EDGE_MAX_OFFSET = 0.015;
const int EDGE_RANSAC_ITERS = 300;
const int EDGE_MIN_POINTS = 40;
const double EDGE_BORDER_PX = 6.0;

struct EdgeEnds {
  bool ok = false;
  std::string why;
  Eigen::Vector3d end_a, end_b;  // edge ends in BASE [m] (at the edge height)
};

EdgeEnds measureEdgeEnds(const pcl::PointCloud<pcl::PointXYZ> &cloud_depth,
                         const Eigen::Matrix4d &T_base_depth, const EEPose &grasp,
                         const Intrinsics &intr_in) {
  EdgeEnds out;
  // Test mode (--pcd) has no camera parameters: the Gen3 depth camera values.
  Intrinsics intr = intr_in;
  if (intr.width <= 0) {
    intr.width = 480; intr.height = 270;
    intr.fx = 333.981; intr.fy = 333.981; intr.cx = 239.205; intr.cy = 141.683;
  }
  const Eigen::Matrix3d R_bd = T_base_depth.block<3, 3>(0, 0);
  const Eigen::Vector3d t_bd = T_base_depth.block<3, 1>(0, 3);
  std::vector<Eigen::Vector3d> P;
  P.reserve(cloud_depth.size());
  for (const auto &p : cloud_depth) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      P.push_back(R_bd * Eigen::Vector3d(p.x, p.y, p.z) + t_bd);
    }
  }
  const Eigen::Vector3d tcp = grasp.position;
  const Eigen::Vector3d y_side = grasp.rotation.col(1), z_app = grasp.rotation.col(2);

  // 1. edge band
  std::vector<double> zs;
  for (const auto &p : P) {
    if ((p.head<2>() - tcp.head<2>()).norm() < EDGE_RADIUS) zs.push_back(p.z());
  }
  if (static_cast<int>(zs.size()) < EDGE_MIN_POINTS) {
    out.why = "few points near the grasp";
    return out;
  }
  std::sort(zs.begin(), zs.end());
  const double z_top = zs[static_cast<size_t>(0.995 * (zs.size() - 1))];
  std::vector<Eigen::Vector3d> B;
  for (const auto &p : P) {
    if ((p.head<2>() - tcp.head<2>()).norm() < EDGE_RADIUS && p.z() > z_top - EDGE_BAND) B.push_back(p);
  }
  std::vector<double> bz;
  for (const auto &p : B) bz.push_back(p.z());
  std::nth_element(bz.begin(), bz.begin() + bz.size() / 2, bz.end());
  const double edge_z = bz[bz.size() / 2];

  // 2. grasp point on the edge
  Eigen::Vector3d g = tcp;
  if (std::abs(z_app.z()) > 0.3) g = tcp + z_app * (edge_z - tcp.z()) / z_app.z();

  // 3. RANSAC line (horizontal)
  std::vector<int> seeds;
  for (int i = 0; i < static_cast<int>(B.size()); i++) {
    if ((B[i].head<2>() - g.head<2>()).norm() < EDGE_SEED_R) seeds.push_back(i);
  }
  if (seeds.size() < 2) {
    out.why = "no plate edge next to the grasp";
    return out;
  }
  std::mt19937 rng(0);
  std::uniform_int_distribution<size_t> pick(0, seeds.size() - 1);
  std::vector<char> best_in;
  int best_n = 0;
  for (int it = 0; it < EDGE_RANSAC_ITERS; it++) {
    const size_t a = pick(rng), b = pick(rng);
    if (a == b) continue;
    const Eigen::Vector2d d = B[seeds[b]].head<2>() - B[seeds[a]].head<2>();
    const double L = d.norm();
    if (L < 0.01) continue;
    const Eigen::Vector2d nrm(-d.y() / L, d.x() / L);
    std::vector<char> in(B.size(), 0);
    int n = 0;
    for (size_t i = 0; i < B.size(); i++) {
      if (std::abs((B[i].head<2>() - B[seeds[a]].head<2>()).dot(nrm)) < EDGE_SLAB) {
        in[i] = 1;
        n++;
      }
    }
    if (n > best_n) {
      best_n = n;
      best_in.swap(in);
    }
  }
  if (best_n < EDGE_MIN_POINTS) {
    out.why = "no straight edge next to the grasp";
    return out;
  }
  std::vector<Eigen::Vector3d> E;
  for (size_t i = 0; i < B.size(); i++) {
    if (best_in[i]) E.push_back(B[i]);
  }
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto &p : E) mean += p.head<2>();
  mean /= E.size();
  Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
  for (const auto &p : E) C += (p.head<2>() - mean) * (p.head<2>() - mean).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
  Eigen::Vector3d u(es.eigenvectors()(0, 1), es.eigenvectors()(1, 1), 0.0);
  if (u.dot(y_side) < 0) u = -u;
  const Eigen::Vector3d nrm3(-u.y(), u.x(), 0.0);
  const double off = (g - Eigen::Vector3d(mean.x(), mean.y(), g.z())).dot(nrm3);
  if (std::abs(off) > EDGE_MAX_OFFSET) {
    out.why = "the grasp is " + text(std::abs(off) * 100) + " cm away from the edge";
    return out;
  }

  // 4. continuous run that contains the grasp point
  std::vector<double> s;
  for (const auto &p : E) s.push_back((p - g).dot(u));
  std::sort(s.begin(), s.end());
  if (s.front() > EDGE_GAP || s.back() < -EDGE_GAP) {
    out.why = "the grasp is off the plate edge";
    return out;
  }
  const size_t k0 = std::lower_bound(s.begin(), s.end(), 0.0) - s.begin();
  size_t lo = k0 > 0 ? k0 - 1 : 0, hi = std::min(k0, s.size() - 1);
  if (std::min(std::abs(s[lo]), std::abs(s[hi])) > EDGE_GAP) {
    out.why = "gap in the edge right at the grasp";
    return out;
  }
  while (lo > 0 && s[lo] - s[lo - 1] <= EDGE_GAP) lo--;
  while (hi + 1 < s.size() && s[hi + 1] - s[hi] <= EDGE_GAP) hi++;
  Eigen::Vector3d end_a = g + u * s[lo], end_b = g + u * s[hi];
  end_a.z() = end_b.z() = edge_z;

  // 5. ends on the image border -> the plate continues out of the image
  const Eigen::Matrix3d R_db = R_bd.transpose();
  auto atBorder = [&](const Eigen::Vector3d &p_base) {
    const Eigen::Vector3d c = R_db * (p_base - t_bd);
    if (c.z() <= 0) return true;
    const double col = intr.cx - intr.fx * c.x() / c.z();  // x = -(z/fx)(col - cx)
    const double row = intr.cy - intr.fy * c.y() / c.z();  // y = -(z/fy)(row - cy)
    return col < EDGE_BORDER_PX || col > intr.width - 1 - EDGE_BORDER_PX ||
           row < EDGE_BORDER_PX || row > intr.height - 1 - EDGE_BORDER_PX;
  };
  if (atBorder(end_a) || atBorder(end_b)) {
    out.why = "a plate end is on the image border (the plate continues out of the image)";
    return out;
  }
  out.ok = true;
  out.end_a = end_a;
  out.end_b = end_b;
  printf("Plate edge: %.1f cm (-y side %.1f cm, +y side %.1f cm from the grasp)\n",
         (end_b - end_a).norm() * 100, -s[lo] * 100, s[hi] * 100);
  return out;
}

// Table top in BASE [m] (--table-z). Antony's TABLE_Z_BASE was measured
// with the robot where it was on 2026-10-17; after the robot is moved the
// table is at another height (2026-10-27: -0.20) and the crop would drop the
// table AND the objects on it.
double g_table_z = TABLE_Z_BASE;

// Plate edge from the color image (Antony's measurePlateEdgeImage, 2026-10-29):
// accepted if its length is within [-PLATE_LENGTH_SHORT, +PLATE_LENGTH_LONG] of
// g_plate_length (--plate-length, m). Shorter = part of the plate is out of
// the image and the middle would be off by half of it. The color image edge
// reads a few mm long (blurred borders: 157 mm for a 150 mm plate).
double g_plate_length = 0.150;
const double PLATE_LENGTH_SHORT = 0.010;
const double PLATE_LENGTH_LONG = 0.015;

// Same as cropBelowTable (crop_table.cpp) with the table at g_table_z.
pcl::PointCloud<pcl::PointXYZ> cropTable(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                         const Eigen::Matrix4d &T_base_depth) {
  const double z_min = g_table_z - TABLE_MARGIN;
  pcl::PointCloud<pcl::PointXYZ> kept;
  for (const pcl::PointXYZ &p : cloud) {
    const double z_base = T_base_depth(2, 0) * p.x + T_base_depth(2, 1) * p.y +
                          T_base_depth(2, 2) * p.z + T_base_depth(2, 3);
    if (z_base > z_min) kept.push_back(p);
  }
  printf("Crop below the table (z in BASE > %.3f m): %zu of %zu points kept\n", z_min, kept.size(),
         cloud.size());
  if (cloud.size() > 1000 && kept.size() < cloud.size() / 5) {
    printf("  Warning: most of the cloud is below the table. Was the robot or the table moved?\n"
           "  Measure the table again and use --table-z (now %.3f m).\n", g_table_z);
  }
  return kept;
}

// What a CAPTURE request needs from main.
struct GraspConfig {
  std::string config_filename;  // GPD settings (screws or plate)
  const GpdSetup *setup;
  std::string color_name;       // plate color (cfg/plate_colors.cfg), "" = whole cloud
  bool plate = false;           // PLATE request: the edge ends go to the controller
};

// FOCUS NOW of the color camera (the button of the web app). Opens its own
// session (Kortex: one client of each service per session), only the vision
// module services: it does not touch the arm. Then waits FOCUS_WAIT_S for the
// lens. A failure only prints a warning (the photo is taken anyway).
const double FOCUS_WAIT_S = 1.0;
void focusColorCamera(const std::string &ip_address) {
  namespace k_api = Kinova::Api;
  try {
    RobotConnection robot(ip_address);
    if (!robot.isConnected()) {
      printf("Warning: FOCUS NOW not done (no connection)\n");
      return;
    }
    k_api::DeviceManager::DeviceManagerClient device_manager(&robot.router());
    k_api::VisionConfig::VisionConfigClient vision_config(&robot.router());
    uint32_t vision_id = 0;
    const auto devices = device_manager.ReadAllDevices();
    for (int i = 0; i < devices.device_handle_size(); i++) {
      if (devices.device_handle(i).device_type() == k_api::Common::DeviceTypes::VISION) {
        vision_id = devices.device_handle(i).device_identifier();
        break;
      }
    }
    if (vision_id == 0) {
      printf("Warning: FOCUS NOW not done (no vision module)\n");
      return;
    }
    k_api::VisionConfig::SensorFocusAction action;
    action.set_sensor(k_api::VisionConfig::SENSOR_COLOR);
    action.set_focus_action(k_api::VisionConfig::FOCUSACTION_FOCUS_NOW);
    vision_config.DoSensorFocusAction(action, vision_id);
    printf("Color camera: FOCUS NOW, waiting %.1f s\n", FOCUS_WAIT_S);
    std::this_thread::sleep_for(std::chrono::duration<double>(FOCUS_WAIT_S));
  } catch (const std::exception &e) {
    printf("Warning: FOCUS NOW failed (%s); the photo is taken anyway\n", e.what());
  }
}

// Depth image from the camera: filtered over time (about 1 s, robot still) or
// one image.
bool takeDepth(const std::string &ip_address, bool use_filter, cv::Mat &depth_image) {
  if (use_filter) {
    printf("Depth filtered for %.1f s (robot still)...\n", 3.0 / DEPTH_FILTER_W);
    return captureFilteredDepth(ip_address, DEPTH_FILTER_W, depth_image);
  }
  return captureDepthImage(ip_address, depth_image);
}

// One request: capture (or the --pcd cloud), GPD, EE pose of the best grasp
// and the plate edge. Returns the answer text (without "SCREW seq").
std::string handleCapture(const GraspConfig &gc, const std::string &ip_address,
                          const Intrinsics &intr, const Intrinsics &color_intr_in,
                          const std::string &pcd_filename, bool use_filter, bool use_focus,
                          const ToolPose &tool) {
  pcl::PointCloud<pcl::PointXYZ> raw;
  cv::Mat color_image;
  Intrinsics color_intr = color_intr_in;
  const bool want_color = !gc.color_name.empty();

  if (!pcd_filename.empty()) {
    // ---------- Test mode: saved cloud (and its color image, if any) ----------
    if (pcl::io::loadPCDFile(pcd_filename, raw) != 0 || raw.empty()) {
      return "FAIL could not read " + pcd_filename;
    }
    if (want_color) {
      std::string pose_file = pcd_filename;
      pose_file.replace(pose_file.size() - 4, 4, "_pose.txt");
      if (!readColorCapture(pose_file, color_intr, color_image)) {
        return "FAIL no color image for " + pcd_filename + " (needed by --plate-color)";
      }
    }
  } else {
    // ---------- 1. Depth image -> cloud (DEPTH frame) ----------
    if (want_color && use_focus) {
      focusColorCamera(ip_address);
    }
    cv::Mat depth_image;
    if (!takeDepth(ip_address, use_filter, depth_image)) {
      return "FAIL could not read the depth camera";
    }
    if (depth_image.cols != intr.width || depth_image.rows != intr.height) {
      return "FAIL image size does not match the camera parameters";
    }
    raw = depthToCloud(depth_image, intr);
    printf("Point cloud: %zu points\n", raw.size());
    if (raw.empty()) {
      return "FAIL empty point cloud (the camera measured no depth)";
    }
    if (want_color && !captureColorImage(ip_address, color_image)) {
      return "FAIL could not read the color camera";
    }

    // ---------- 2. Save it ----------
    const std::string name = captureName();
    if (saveServerCapture(name, raw, tool, color_image, &color_intr)) {
      printf("Saved %s.pcd and %s_pose.txt\n", name.c_str(), name.c_str());
    }
  }

  // ---------- 3. Frames, and the points below the table out (step 1.0) ----------
  const Eigen::Matrix4d T_base_ee = toolToBase(tool);
  const Eigen::Matrix4d T_base_int = intToBase(T_base_ee, KinovaGen3::TOOL_OFFSET_Z_2F85);
  const Eigen::Matrix4d T_base_depth = T_base_int * depthToInt();
  const pcl::PointCloud<pcl::PointXYZ> cloud = cropTable(raw, T_base_depth);

  // ---------- 4. GPD (only on the plate of one color with --plate-color) ----------
  Eigen::Matrix3Xd samples;
  if (want_color && !plateSamples(cloud, color_image, color_intr, gc.color_name, samples)) {
    return "FAIL no plate of color " + gc.color_name + " in the image";
  }
  Grasps grasps;
  if (!detectGraspsInCloud(gc.config_filename, cloud, *gc.setup, grasps, samples)) {
    return "FAIL could not run GPD";
  }
  if (grasps.empty()) {
    return "FAIL GPD found no grasps";
  }
  // Plate (select_plate_center = 1 in its cfg): the grasp nearest to the
  // middle of the plate edge first, as grasps_from_camera --color does
  // (Pipeline/grasp_pipeline.h, plateEdgeOfGrasps). If the edge cannot be
  // measured, the GPD order (best score first) is kept.
  // The same edge gives the edge ends for the controller (step 6).
  // Screw (select_object_center = 1 in its cfg): the grasp nearest to the middle
  // of the head first, along the hand axis. Same function; what it measures on a
  // screw is not a plate edge, so it does not reach step 6.
  PlateEdge plate_edge;
  bool has_plate_edge = false;
  if (want_color && gc.plate) {
    has_plate_edge = plateEdgeOfGrasps(cloud, color_image, color_intr, gc.color_name, samples,
                                       *gc.setup, grasps, plate_edge);
    if (!has_plate_edge) {
      printf("Plate edge not measured: grasps in GPD order (best score first)\n");
    }
  } else if (want_color && gc.setup->select_object_center) {
    PlateEdge object_middle;
    if (!plateEdgeOfGrasps(cloud, color_image, color_intr, gc.color_name, samples, *gc.setup,
                           grasps, object_middle)) {
      printf("Object middle not measured: grasps in GPD order (best score first)\n");
    }
  }

  // ---------- 5. EE poses in BASE (same as grasps_from_camera, step 7) ----------
  std::cout << "\n========== EE POSES w.r.t. BASE ==========\n"
            << "  EE now (m, deg): " << tool.x << " " << tool.y << " " << tool.z << "  "
            << tool.theta_x << " " << tool.theta_y << " " << tool.theta_z << "\n";
  const Eigen::Vector3d ee_angles_now(tool.theta_x, tool.theta_y, tool.theta_z);
  printEEPoses(grasps, *gc.setup, T_base_int, T_base_ee, ee_angles_now,
               KinovaGen3::TOOL_OFFSET_Z_2F85, samples, cloud);

  // Best grasp = first (nearest to the middle of the plate edge or of the screw
  // head; if not measured, GPD order, best score first); centered on the object
  // when its points are known (the same as printEEPoses does).
  Eigen::Vector3d grasp_position = grasps[0]->getPosition();
  if (samples.cols() > 0) {
    grasp_position = centerGrasp(*grasps[0], samples, cloud, gc.setup->hand).position;
  }
  const EEPose best = graspToEE(*grasps[0], grasp_position, gc.setup->hand_depth, T_base_int,
                                T_base_ee.block<3, 3>(0, 0), ee_angles_now,
                                KinovaGen3::TOOL_OFFSET_Z_2F85);
  const Eigen::Vector3d angles(best.theta_x, best.theta_y, best.theta_z);
  if (!best.position.allFinite() || !angles.allFinite() ||
      !best.pregrasp_position.allFinite()) {
    return "FAIL best grasp pose is not finite";
  }
  // ---------- 6. Plate edge (for the plate steps; ignored for screws) ----------
  // First the edge measured in the color image (Antony, step 4): its ends are
  // middle -+ axis * length / 2, if the length is the plate's. Otherwise the
  // edge from the depth points (measureEdgeEnds), which also checks the image
  // border.
  EdgeEnds edge;
  const double length_mm = plate_edge.length * 1000.0;
  if (has_plate_edge && plate_edge.length >= g_plate_length - PLATE_LENGTH_SHORT &&
      plate_edge.length <= g_plate_length + PLATE_LENGTH_LONG) {
    const Eigen::Vector3d half = 0.5 * plate_edge.length * plate_edge.axis;
    edge.ok = true;
    edge.end_a = (T_base_depth * (plate_edge.center - half).homogeneous()).head<3>();
    edge.end_b = (T_base_depth * (plate_edge.center + half).homogeneous()).head<3>();
    // Both ends at the height of the middle, as measureEdgeEnds gives them: the
    // z(s) line of the color edge tilts a few mm with noise, and the controller
    // would read that as the plate turned in the gripper.
    const double z_mid = 0.5 * (edge.end_a.z() + edge.end_b.z());
    edge.end_a.z() = edge.end_b.z() = z_mid;
    printf("Plate edge for the controller: color image, %.0f mm (plate %.0f mm)\n", length_mm,
           g_plate_length * 1000.0);
  } else {
    if (has_plate_edge) {
      printf("Plate edge in the color image: %.0f mm, not %.0f mm (-%.0f/+%.0f): part of the plate "
             "may be out of the image; edge from the depth points instead\n",
             length_mm, g_plate_length * 1000.0, PLATE_LENGTH_SHORT * 1000.0,
             PLATE_LENGTH_LONG * 1000.0);
    }
    edge = measureEdgeEnds(cloud, T_base_depth, best, intr);
  }
  std::string edge_text = " NOEDGE";
  if (edge.ok) {
    edge_text = " EDGE " + text(edge.end_a.x()) + " " + text(edge.end_a.y()) + " " +
                text(edge.end_a.z()) + " " + text(edge.end_b.x()) + " " + text(edge.end_b.y()) +
                " " + text(edge.end_b.z());
  } else {
    printf("Plate edge: not measured (%s)\n", edge.why.c_str());
  }
  return "OK " + text(best.position.x()) + " " + text(best.position.y()) + " " +
         text(best.position.z()) + " " + text(angles.x()) + " " + text(angles.y()) + " " +
         text(angles.z()) + " " + text(best.pregrasp_position.x()) + " " +
         text(best.pregrasp_position.y()) + " " + text(best.pregrasp_position.z()) + " " +
         text(grasps[0]->getScore()) + " " + std::to_string(grasps.size()) + edge_text;
}

// One APPROACH request = approach_object COLOR (Task/approach_pose.h), with
// the TOOL pose of the controller: capture (one depth image + color image),
// centroid of the region of the color, does it touch the image border, EE
// target. Returns the answer text (without "APPROACH seq").
std::string handleApproach(const std::string &color_name, const std::string &ip_address,
                           const Intrinsics &intr, const Intrinsics &color_intr_in,
                           const std::string &pcd_filename, bool use_focus, const ToolPose &tool) {
  if (color_name.empty()) {
    return "FAIL no color for this object (--plate-color / --bolt-color)";
  }
  pcl::PointCloud<pcl::PointXYZ> raw;
  cv::Mat color_image;
  Intrinsics color_intr = color_intr_in;
  if (!pcd_filename.empty()) {
    if (pcl::io::loadPCDFile(pcd_filename, raw) != 0 || raw.empty()) {
      return "FAIL could not read " + pcd_filename;
    }
    std::string pose_file = pcd_filename;
    pose_file.replace(pose_file.size() - 4, 4, "_pose.txt");
    if (!readColorCapture(pose_file, color_intr, color_image)) {
      return "FAIL no color image for " + pcd_filename;
    }
  } else {
    if (use_focus) {
      focusColorCamera(ip_address);
    }
    cv::Mat depth_image;
    if (!captureDepthImage(ip_address, depth_image)) {
      return "FAIL could not read the depth camera";
    }
    if (depth_image.cols != intr.width || depth_image.rows != intr.height) {
      return "FAIL image size does not match the camera parameters";
    }
    raw = depthToCloud(depth_image, intr);
    if (raw.empty()) {
      return "FAIL empty point cloud (the camera measured no depth)";
    }
    if (!captureColorImage(ip_address, color_image)) {
      return "FAIL could not read the color camera";
    }
    if (color_image.cols != color_intr.width || color_image.rows != color_intr.height) {
      return "FAIL color image size does not match the color camera parameters";
    }
    const std::string name = captureName("approach");
    if (saveServerCapture(name, raw, tool, color_image, &color_intr)) {
      printf("Saved %s.pcd, %s_color.png and %s_pose.txt\n", name.c_str(), name.c_str(), name.c_str());
    }
  }

  // ---------- 2. DEPTH frame in BASE, drop the points below the table ----------
  const Eigen::Matrix4d T_base_ee = toolToBase(tool);
  const Eigen::Matrix4d T_base_int = intToBase(T_base_ee, KinovaGen3::TOOL_OFFSET_Z_2F85);
  const Eigen::Matrix4d T_base_depth = T_base_int * depthToInt();
  const pcl::PointCloud<pcl::PointXYZ> cloud = cropTable(raw, T_base_depth);

  // ---------- 2a-2d (same as approach_object.cpp) ----------
  ObjectPosition object;
  if (!objectPosition(cloud, color_image, color_intr, color_name, T_base_depth, object)) {
    return "FAIL no " + color_name + " object in the image";
  }
  ImageBorders borders;
  if (!regionBorders(cloud, color_image, color_intr, color_name, borders)) {
    return "FAIL color " + color_name + " is not in the colors file";
  }
  const ToolPose target = approachPose(object.centroid, tool, color_intr, APPROACH_HEIGHT,
                                       KinovaGen3::TOOL_OFFSET_Z_2F85);
  double distance = 0.0;
  const bool at_target = atApproachPose(tool, target, distance);
  const char *state = !borders.any() ? "INVIEW" : (at_target ? "ATTARGET" : "MOVE");
  printf("\n========== APPROACH %s ==========\n", color_name.c_str());
  printf("  Centroid in BASE (m): %.4f %.4f %.4f  (%zu points)\n", object.centroid.x(),
         object.centroid.y(), object.centroid.z(), object.points);
  printf("  EE now   :  x = %.4f  y = %.4f  z = %.4f [m]   tx = %.1f  ty = %.1f  tz = %.1f [deg]\n",
         tool.x, tool.y, tool.z, tool.theta_x, tool.theta_y, tool.theta_z);
  printf("  EE target:  x = %.4f  y = %.4f  z = %.4f [m]   tx = %.1f  ty = %.1f  tz = %.1f [deg]\n",
         target.x, target.y, target.z, target.theta_x, target.theta_y, target.theta_z);
  printf("  Distance to the target: %.1f cm.%s%s%s%s%s\n", distance * 100.0,
         borders.any() ? " Region touches the image border:" : "", borders.left ? " left" : "",
         borders.right ? " right" : "", borders.top ? " top" : "", borders.bottom ? " bottom" : "");
  printf("  %s\n", !borders.any() ? "IN VIEW: the whole region is inside the color image"
                   : at_target    ? "AT TARGET, but the region still touches the border"
                                  : "MOVE: the controller goes to the EE target and asks again");
  return std::string("OK ") + state + " " + text(target.x) + " " + text(target.y) + " " +
         text(target.z) + " " + text(target.theta_x) + " " + text(target.theta_y) + " " +
         text(target.theta_z) + " " + text(object.centroid.x()) + " " + text(object.centroid.y()) +
         " " + text(object.centroid.z()) + " " + std::to_string(object.points);
}

// ---------------------------------------------------------------------------
// Plate holes (2026-10-21). Photo from above the plate in the fixture: the
// plate face is a flat surface and its holes are round gaps in it. The
// controller puts the screw in the SMALLEST hole. Same algorithm and
// constants as Controlador_Impedancia/gui/modular_gui/holes.py (keep both
// equal):
//   1. Plate plane: RANSAC on the cloud (normal within HOLE_PLANE_MAX_TILT of
//      vertical, inliers within HOLE_PLANE_TOL), refined with PCA.
//   2. Plate points (within HOLE_PLATE_BAND of the plane) on a HOLE_RES grid
//      on the plane (axes: base x projected, and n x e1). Closing of 1 cell
//      (dilate + erode, cross neighbours) to fill isolated gaps.
//   3. Holes: EMPTY regions (cross neighbours) that do not touch the grid
//      border, with the area of a circle between HOLE_MIN_DIAM and
//      HOLE_MAX_DIAM.
//   4. Rim: ring of 3 cells around the region; in 72 sectors from its center,
//      the nearest plate point. Least squares circle (Kasa). Rejected if less
//      than 90 % of the rim is seen, the residual is large or the area does
//      not match the circle.
const double HOLE_PLANE_TOL = 0.0015;
const double HOLE_PLANE_REFINE = 0.002;
const double HOLE_PLANE_MAX_TILT_DEG = 15.0;
const int HOLE_PLANE_ITERS = 300;
const double HOLE_PLATE_BAND = 0.003;
const double HOLE_RES = 0.001;
// The smallest real hole is 30 mm and reads 27 to 33 mm here; false holes (gaps in
// the cloud) read 13 to 24 mm and the controller goes to the smallest one.
const double HOLE_MIN_DIAM = 0.025;
// Empty region before the circle fit: smaller than the circle when the rim is noisy.
const double HOLE_MIN_REGION_DIAM = 0.010;
const double HOLE_MAX_DIAM = 0.050;
const int HOLE_RING_CELLS = 3;
const int HOLE_SECTORS = 72;
const int HOLE_MIN_SECTORS = 65;
const double HOLE_MAX_RMS_FRAC = 0.15;
const double HOLE_AREA_RATIO_MIN = 0.6, HOLE_AREA_RATIO_MAX = 1.3;

struct Hole {
  Eigen::Vector3d center;  // BASE [m], on the plate plane
  double diameter = 0.0;   // [m]
  double rms = 0.0;        // rim residual [m]
};

struct PlateHoles {
  std::string why;          // empty if at least one hole
  double tilt_deg = 0.0;    // plate plane vs horizontal
  std::vector<Hole> holes;  // smallest first
};

PlateHoles detectHoles(const pcl::PointCloud<pcl::PointXYZ> &cloud_depth,
                       const Eigen::Matrix4d &T_base_depth) {
  PlateHoles out;
  const Eigen::Matrix3d R_bd = T_base_depth.block<3, 3>(0, 0);
  const Eigen::Vector3d t_bd = T_base_depth.block<3, 1>(0, 3);
  std::vector<Eigen::Vector3d> P;
  P.reserve(cloud_depth.size());
  for (const auto &p : cloud_depth) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      P.push_back(R_bd * Eigen::Vector3d(p.x, p.y, p.z) + t_bd);
    }
  }
  if (P.size() < 100) {
    out.why = "few points";
    return out;
  }

  // 1. plate plane
  std::mt19937 rng(0);
  std::uniform_int_distribution<size_t> pick(0, P.size() - 1);
  const double max_tilt = std::cos(HOLE_PLANE_MAX_TILT_DEG * M_PI / 180.0);
  int best_n = 0;
  Eigen::Vector3d best_a, best_nrm;
  for (int it = 0; it < HOLE_PLANE_ITERS; it++) {
    const Eigen::Vector3d &a = P[pick(rng)], &b = P[pick(rng)], &c = P[pick(rng)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    if (n.norm() < 1e-9) continue;
    n.normalize();
    if (std::abs(n.z()) < max_tilt) continue;
    int k = 0;
    for (const auto &p : P) k += std::abs((p - a).dot(n)) < HOLE_PLANE_TOL;
    if (k > best_n) {
      best_n = k;
      best_a = a;
      best_nrm = n;
    }
  }
  if (best_n == 0) {
    out.why = "no flat plate face (within 15 deg of horizontal)";
    return out;
  }
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  int nin = 0;
  for (const auto &p : P) {
    if (std::abs((p - best_a).dot(best_nrm)) < HOLE_PLANE_REFINE) {
      c += p;
      nin++;
    }
  }
  c /= nin;
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (const auto &p : P) {
    if (std::abs((p - best_a).dot(best_nrm)) < HOLE_PLANE_REFINE) C += (p - c) * (p - c).transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
  Eigen::Vector3d n = es.eigenvectors().col(0);
  if (n.z() < 0) n = -n;
  out.tilt_deg = std::acos(std::min(1.0, std::abs(n.z()))) * 180.0 / M_PI;

  // 2. plate points on a grid
  Eigen::Vector3d e1 = Eigen::Vector3d::UnitX() - n * n.x();
  e1.normalize();
  const Eigen::Vector3d e2 = n.cross(e1);
  std::vector<Eigen::Vector2d> uv;
  for (const auto &p : P) {
    if (std::abs((p - c).dot(n)) < HOLE_PLATE_BAND) uv.emplace_back((p - c).dot(e1), (p - c).dot(e2));
  }
  Eigen::Vector2d mn(1e9, 1e9), mx(-1e9, -1e9);
  for (const auto &q : uv) {
    mn = mn.cwiseMin(q);
    mx = mx.cwiseMax(q);
  }
  const int H = static_cast<int>(std::floor((mx.x() - mn.x()) / HOLE_RES)) + 1;
  const int W = static_cast<int>(std::floor((mx.y() - mn.y()) / HOLE_RES)) + 1;
  std::vector<int> cell(uv.size());
  std::vector<char> occ(static_cast<size_t>(H) * W, 0);
  for (size_t i = 0; i < uv.size(); i++) {
    const int a = static_cast<int>(std::floor((uv[i].x() - mn.x()) / HOLE_RES));
    const int b = static_cast<int>(std::floor((uv[i].y() - mn.y()) / HOLE_RES));
    cell[i] = a * W + b;
    occ[cell[i]] = 1;
  }
  auto dilate = [&](const std::vector<char> &m) {
    std::vector<char> o = m;
    for (int i = 0; i < H; i++)
      for (int j = 0; j < W; j++)
        if (!m[i * W + j] && ((i > 0 && m[(i - 1) * W + j]) || (i < H - 1 && m[(i + 1) * W + j]) ||
                              (j > 0 && m[i * W + j - 1]) || (j < W - 1 && m[i * W + j + 1])))
          o[i * W + j] = 1;
    return o;
  };
  auto erode = [&](const std::vector<char> &m) {  // outside the grid = empty
    std::vector<char> o(m.size(), 0);
    for (int i = 1; i < H - 1; i++)
      for (int j = 1; j < W - 1; j++)
        o[i * W + j] = m[i * W + j] && m[(i - 1) * W + j] && m[(i + 1) * W + j] &&
                       m[i * W + j - 1] && m[i * W + j + 1];
    return o;
  };
  occ = erode(dilate(occ));

  // 3. empty regions
  const double min_cells = M_PI / 4 * std::pow(HOLE_MIN_REGION_DIAM / HOLE_RES, 2);
  const double max_cells = M_PI / 4 * std::pow(HOLE_MAX_DIAM / HOLE_RES, 2);
  std::vector<char> seen(occ.size(), 0);
  for (int start = 0; start < H * W; start++) {
    if (occ[start] || seen[start]) continue;
    std::vector<int> cells, stack{start};
    seen[start] = 1;
    bool border = false;
    while (!stack.empty()) {
      const int k = stack.back();
      stack.pop_back();
      cells.push_back(k);
      const int i = k / W, j = k % W;
      border = border || i == 0 || j == 0 || i == H - 1 || j == W - 1;
      const int nb[4][2] = {{i + 1, j}, {i - 1, j}, {i, j + 1}, {i, j - 1}};
      for (const auto &q : nb) {
        if (q[0] < 0 || q[0] >= H || q[1] < 0 || q[1] >= W) continue;
        const int kk = q[0] * W + q[1];
        if (!occ[kk] && !seen[kk]) {
          seen[kk] = 1;
          stack.push_back(kk);
        }
      }
    }
    if (border || cells.size() < min_cells || cells.size() > max_cells) continue;

    // 4. rim and circle
    std::vector<char> mask(occ.size(), 0);
    Eigen::Vector2d c0 = Eigen::Vector2d::Zero();
    for (int k : cells) {
      mask[k] = 1;
      c0 += Eigen::Vector2d(k / W, k % W);
    }
    c0 = (c0 / cells.size() + Eigen::Vector2d(0.5, 0.5)) * HOLE_RES + mn;
    std::vector<char> ring = mask;
    for (int r = 0; r < HOLE_RING_CELLS; r++) ring = dilate(ring);
    std::vector<double> best_r(HOLE_SECTORS, 1e9);
    std::vector<Eigen::Vector2d> rim(HOLE_SECTORS);
    for (size_t i = 0; i < uv.size(); i++) {
      if (!ring[cell[i]] || mask[cell[i]]) continue;
      const Eigen::Vector2d d = uv[i] - c0;
      int sct = static_cast<int>((std::atan2(d.y(), d.x()) + M_PI) / (2 * M_PI) * HOLE_SECTORS);
      sct = std::clamp(sct, 0, HOLE_SECTORS - 1);
      if (d.norm() < best_r[sct]) {
        best_r[sct] = d.norm();
        rim[sct] = uv[i];
      }
    }
    std::vector<Eigen::Vector2d> edge;
    for (int k = 0; k < HOLE_SECTORS; k++)
      if (best_r[k] < 1e8) edge.push_back(rim[k]);
    if (static_cast<int>(edge.size()) < HOLE_MIN_SECTORS) continue;
    Eigen::MatrixXd A(edge.size(), 3);
    Eigen::VectorXd bb(edge.size());
    for (size_t k = 0; k < edge.size(); k++) {
      A.row(k) << 2 * edge[k].x(), 2 * edge[k].y(), 1.0;
      bb(k) = edge[k].squaredNorm();
    }
    const Eigen::Vector3d sol = A.colPivHouseholderQr().solve(bb);
    const Eigen::Vector2d cc(sol(0), sol(1));
    const double R2 = sol(2) + cc.squaredNorm();
    if (!(R2 > 0)) continue;
    const double R = std::sqrt(R2);
    double ss = 0;
    for (const auto &q : edge) ss += std::pow((q - cc).norm() - R, 2);
    const double rms = std::sqrt(ss / edge.size());
    const double ratio = cells.size() * HOLE_RES * HOLE_RES / (M_PI * R2);
    if (2 * R < HOLE_MIN_DIAM || 2 * R > HOLE_MAX_DIAM || rms > HOLE_MAX_RMS_FRAC * R ||
        ratio < HOLE_AREA_RATIO_MIN || ratio > HOLE_AREA_RATIO_MAX)
      continue;
    out.holes.push_back({c + cc.x() * e1 + cc.y() * e2, 2 * R, rms});
  }
  std::sort(out.holes.begin(), out.holes.end(),
            [](const Hole &a, const Hole &b) { return a.diameter < b.diameter; });
  if (out.holes.empty()) out.why = "no round hole found";
  printf("Plate face: tilt %.2f deg. Holes (smallest first):\n", out.tilt_deg);
  for (const auto &h : out.holes) {
    printf("  hole at %.4f %.4f %.4f  diameter %.1f mm  rim rms %.2f mm\n", h.center.x(),
           h.center.y(), h.center.z(), h.diameter * 1000, h.rms * 1000);
  }
  return out;
}

// " HOLE x y z d n" (smallest hole, BASE [m], diameter [m], holes found) or " NOHOLE".
std::string holeText(const pcl::PointCloud<pcl::PointXYZ> &cloud, const ToolPose &tool) {
  const Eigen::Matrix4d T_base_depth =
      intToBase(toolToBase(tool), KinovaGen3::TOOL_OFFSET_Z_2F85) * depthToInt();
  const PlateHoles ph = detectHoles(cloud, T_base_depth);
  if (ph.holes.empty()) {
    printf("Holes: none (%s)\n", ph.why.c_str());
    return " NOHOLE";
  }
  const Hole &h = ph.holes.front();
  return " HOLE " + text(h.center.x()) + " " + text(h.center.y()) + " " + text(h.center.z()) +
         " " + text(h.diameter) + " " + std::to_string(ph.holes.size());
}

// One CLOUD request: capture (or the --pcd cloud), save it and find the
// plate holes. NO GPD. Returns the answer text (without "CLOUD seq").
std::string handleCloud(const std::string &ip_address, const Intrinsics &intr,
                        const std::string &pcd_filename, bool use_filter, const ToolPose &tool) {
  if (!pcd_filename.empty()) {
    // Test mode: nothing is saved, the answer names the test cloud.
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(pcd_filename, cloud) != 0 || cloud.empty()) {
      return "FAIL could not read " + pcd_filename;
    }
    std::string name = pcd_filename.substr(pcd_filename.find_last_of('/') + 1);
    if (name.size() > 4 && name.compare(name.size() - 4, 4, ".pcd") == 0) {
      name.resize(name.size() - 4);
    }
    return "OK " + name + " " + std::to_string(cloud.size()) + holeText(cloud, tool);
  }
  cv::Mat depth_image;
  if (!takeDepth(ip_address, use_filter, depth_image)) {
    return "FAIL could not read the depth camera";
  }
  if (depth_image.cols != intr.width || depth_image.rows != intr.height) {
    return "FAIL image size does not match the camera parameters";
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto &p : depthToCloud(depth_image, intr)) {
    if (p.z <= HOLE_MAX_DEPTH) cloud.push_back(p);
  }
  printf("Point cloud: %zu points (depth up to %.3f m)\n", cloud.size(), HOLE_MAX_DEPTH);
  if (cloud.empty()) {
    return "FAIL empty point cloud (nothing closer than HOLE_MAX_DEPTH)";
  }
  const std::string name = captureName("hoyo");
  if (!saveServerCapture(name, cloud, tool)) {
    return "FAIL could not save " + name + ".pcd";
  }
  printf("Saved %s.pcd and %s_pose.txt (no GPD)\n", name.c_str(), name.c_str());
  return "OK " + name + " " + std::to_string(cloud.size()) + holeText(cloud, tool);
}

// "--NAME VALUE" anywhere among the arguments: removes both and returns
// VALUE ("" if absent). "--NAME" alone (flag): returns "1".
std::string takeOption(int &argc, char *argv[], const std::string &name, bool flag = false) {
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) != name) continue;
    const int take = flag ? 1 : 2;
    if (i + take > argc) return "";
    const std::string value = flag ? "1" : argv[i + 1];
    for (int j = i; j + take < argc; j++) argv[j] = argv[j + take];
    argc -= take;
    return value;
  }
  return "";
}

int main(int argc, char *argv[]) {
  // ---------- Arguments ----------
  std::string plate_config = takeOption(argc, argv, "--plate-config");
  if (plate_config.empty()) plate_config = "../cfg/gpd_plate_params.cfg";
  std::string plate_color = takeOption(argc, argv, "--plate-color");
  if (plate_color.empty()) plate_color = "blue";
  std::string bolt_color = takeOption(argc, argv, "--bolt-color");
  if (bolt_color.empty()) bolt_color = "red";
  const bool use_filter = takeOption(argc, argv, "--no-filter", true).empty();
  const bool use_focus = takeOption(argc, argv, "--no-focus", true).empty();
  const std::string port_text = takeOption(argc, argv, "--port");
  const std::string table_text = takeOption(argc, argv, "--table-z");
  if (!table_text.empty()) {
    char *end = nullptr;
    g_table_z = std::strtod(table_text.c_str(), &end);
    if (end == table_text.c_str() || *end != '\0' || !std::isfinite(g_table_z) || g_table_z < -1.0 ||
        g_table_z > 0.5) {
      printf("Bad --table-z %s (table top in BASE, m)\n", table_text.c_str());
      return 1;
    }
  }
  const std::string length_text = takeOption(argc, argv, "--plate-length");
  if (!length_text.empty()) {
    char *end = nullptr;
    g_plate_length = std::strtod(length_text.c_str(), &end);
    if (end == length_text.c_str() || *end != '\0' || !std::isfinite(g_plate_length) ||
        g_plate_length < 0.05 || g_plate_length > 0.50) {
      printf("Bad --plate-length %s (plate edge length, m)\n", length_text.c_str());
      return 1;
    }
  }
  int SERVER_PORT = DEFAULT_PORT;
  if (!port_text.empty() && (!number(port_text, SERVER_PORT) || SERVER_PORT <= 0 || SERVER_PORT > 65535)) {
    printf("Bad --port %s\n", port_text.c_str());
    return 1;
  }
  if (argc < 2 || argc > 4 || (argc == 4 && std::string(argv[2]) != "--pcd") ||
      (argc == 3 && std::string(argv[2]) == "--pcd")) {
    std::cout << "Usage: screw_server CONFIG_FILE [OPTIONS]                (robot at 192.168.1.10)\n"
              << "       screw_server CONFIG_FILE <ip> [OPTIONS]           (robot at <ip>)\n"
              << "       screw_server CONFIG_FILE --pcd FILE.pcd [OPTIONS] (test, no robot)\n"
              << "OPTIONS: --plate-config FILE (default ../cfg/gpd_plate_params.cfg)\n"
              << "         --plate-color NAME  (cfg/plate_colors.cfg; default blue)\n"
              << "         --bolt-color NAME   (default red)\n"
              << "         --no-filter         (one depth image, no time filter)\n"
              << "         --no-focus          (no FOCUS NOW before the color photos)\n"
              << "         --plate-length L    (plate edge length [m]; default 0.150)\n"
              << "         --table-z Z         (table top in BASE [m]; default " << TABLE_Z_BASE << ")\n"
              << "Example (from build_modular/): ./screw_server ../cfg/gpd_params.cfg\n";
    return 1;
  }
  const std::string config_filename = argv[1];
  const bool test_mode = (argc == 4);
  const std::string pcd_filename = test_mode ? argv[3] : "";
  const std::string ip_address = (argc == 3) ? argv[2] : "192.168.1.10";

  struct sigaction sa {};
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // ---------- 1. GPD config: screws and plate ----------
  GpdSetup setup, plate_setup;
  if (!readGpdSetup(config_filename, setup) || !readGpdSetup(plate_config, plate_setup)) {
    return 1;
  }
  const GraspConfig screws{config_filename, &setup, ""};
  // Not const: COLORS changes their color while the server runs.
  GraspConfig bolt{config_filename, &setup, bolt_color};
  GraspConfig plate{plate_config, &plate_setup, plate_color, true};
  printf("Plate (PLATE): %s, color %s. Bolt (BOLT): %s, color %s. Depth filter: %s. "
         "Focus: %s. Table at z = %.3f m in BASE.\n",
         plate_config.c_str(), plate_color.c_str(), config_filename.c_str(), bolt_color.c_str(),
         use_filter ? "on" : "off", use_focus ? "on" : "off", g_table_z);

  // ---------- 2. Camera parameters (once) ----------
  Intrinsics intr{}, color_intr{};
  if (test_mode) {
    printf("TEST MODE: every request uses %s (no robot, no camera).\n", pcd_filename.c_str());
  } else if (!readIntrinsicsFromRobot(ip_address, intr, color_intr)) {
    return 1;
  }

  // ---------- 3. UDP socket on 127.0.0.1:5006 ----------
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(SERVER_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (fd < 0 || bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    printf("Could not open UDP port %d (is another screw_server running?)\n", SERVER_PORT);
    return 1;
  }
  // Wake up every 0.5 s to check Ctrl+C.
  timeval tv{0, 500000};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  printf("\nscrew_server ready on UDP 127.0.0.1:%d. Waiting for the controller "
         "(Ctrl+C to quit)...\n",
         SERVER_PORT);

  // ---------- 4. Requests ----------
  while (!stop_requested) {
    char buf[512];
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    const ssize_t n =
        recvfrom(fd, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr *>(&from), &from_len);
    if (n <= 0) {
      continue;  // timeout (check Ctrl+C) or error
    }
    const std::string request(buf, static_cast<size_t>(n));
    const std::vector<std::string> w = words(request);
    std::string answer;

    if (w.size() == 1 && w[0] == "PING") {
      answer = "PONG";
    } else if (!w.empty() && w[0] == "COLORS") {
      // New colors for the plate and the bolt (the GUI asks for them when the
      // sequence starts). Both must be in the colors file.
      ColorRange range;
      if (w.size() != 3) {
        answer = "COLORS FAIL use: COLORS plate bolt";
      } else if (!readColorRange(PLATE_COLORS_FILE, w[1], range)) {
        answer = "COLORS FAIL no color " + w[1] + " in " + PLATE_COLORS_FILE;
      } else if (!readColorRange(PLATE_COLORS_FILE, w[2], range)) {
        answer = "COLORS FAIL no color " + w[2] + " in " + PLATE_COLORS_FILE;
      } else {
        plate_color = plate.color_name = w[1];
        bolt_color = bolt.color_name = w[2];
        answer = "COLORS OK " + plate_color + " " + bolt_color;
        printf("\n===== Colors: plate %s, bolt %s =====\n", plate_color.c_str(), bolt_color.c_str());
      }
    } else if (!w.empty() && (w[0] == "CAPTURE" || w[0] == "CLOUD" || w[0] == "APPROACH")) {
      const bool cloud_only = (w[0] == "CLOUD");
      const bool approach = (w[0] == "APPROACH");
      uint32_t seq = 0;
      ToolPose tool{};
      double *values[6] = {&tool.x, &tool.y, &tool.z, &tool.theta_x, &tool.theta_y, &tool.theta_z};
      const std::string object = w.size() == 9 ? w[8] : "";
      const bool plate_photo = object == "PLATE";
      const bool bolt_photo = object == "BOLT";
      bool ok = (w.size() == 8 || (!cloud_only && (plate_photo || bolt_photo))) &&
                (!approach || plate_photo || bolt_photo) && number(w[1], seq) && seq != 0;
      for (int i = 0; ok && i < 6; i++) {
        ok = number(w[2 + i], *values[i]) && std::isfinite(*values[i]);
      }
      if (!ok) {
        printf("Bad request ignored: %s\n", request.c_str());
        continue;
      }
      printf("\n===== Request #%u%s: TOOL pose %s %s %s  %s %s %s =====\n", seq,
             approach ? (plate_photo ? " (approach the plate)" : " (approach the bolt)")
             : cloud_only ? " (cloud only, no GPD)"
             : plate_photo ? " (plate)" : (bolt_photo ? " (bolt)" : ""),
             text(tool.x).c_str(), text(tool.y).c_str(), text(tool.z).c_str(),
             text(tool.theta_x).c_str(), text(tool.theta_y).c_str(), text(tool.theta_z).c_str());
      std::string result;
      try {
        result = approach
                     ? handleApproach(plate_photo ? plate_color : bolt_color, ip_address, intr,
                                      color_intr, pcd_filename, use_focus, tool)
                 : cloud_only
                     ? handleCloud(ip_address, intr, pcd_filename, use_filter, tool)
                     : handleCapture(plate_photo ? plate : (bolt_photo ? bolt : screws), ip_address,
                                     intr, color_intr, pcd_filename, use_filter, use_focus, tool);
      } catch (const std::exception &e) {
        result = std::string("FAIL ") + e.what();
      }
      answer = (approach ? "APPROACH " : (cloud_only ? "CLOUD " : "SCREW ")) + std::to_string(seq) +
               " " + result;
    } else {
      printf("Unknown message ignored: %s\n", request.c_str());
      continue;
    }

    sendto(fd, answer.data(), answer.size(), 0, reinterpret_cast<sockaddr *>(&from), from_len);
    if (answer != "PONG") {
      printf("Answer sent: %s\n", answer.c_str());
    }
  }

  close(fd);
  printf("\nscrew_server stopped.\n");
  return 0;
}
