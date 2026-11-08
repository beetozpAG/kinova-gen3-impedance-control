# Build file for kinova_grasp.
#
# How to build (from the kinova_grasp folder):
#   cmake -S . -B build
#   cmake --build build -j$(nproc)
# Programs are created in build/. Run them from build/ (see cfg/gpd_params.cfg).

cmake_minimum_required(VERSION 3.16)
project(kinova_grasp CXX)

# Same C++ version that PCL 1.15 requires.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Write build/compile_commands.json: the exact include paths of each file,
# so the editor (VS Code IntelliSense) finds OpenCV, PCL, Eigen, Kortex and GPD.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Fix for the Ubuntu 26.04 VTK 9.5 error "JsonCpp::JsonCpp ... target was not found".
# The full explanation is in that file. It must run before PCL is searched
# (here and inside third_party/gpd), so it goes first.
include(third_party/gpd/cmake/jsoncpp_fix.cmake)

# GPD (copy in third_party/gpd).
# EXCLUDE_FROM_ALL: build only the GPD parts our programs use,
# not the ~10 example programs of atenpas.
add_subdirectory(third_party/gpd EXCLUDE_FROM_ALL)

# GPD plots (windows "Grasp candidates" and "Selected Grasps"): build GPD's
# plot library from our copy src/Plot/gpd_plot_2f85.cpp, which draws the Robotiq
# 2F-85, instead of third_party/gpd/src/gpd/util/plot.cpp (left unchanged).
set_property(TARGET gpd_plot PROPERTY SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Plot/gpd_plot_2f85.cpp)

# GPD local frames: build GPD's frame estimator from our copy
# src/3_Sample/gpd_frame_estimator_contour.cpp, which can turn the frames to
# the closing directions of the object (contour_frames = 1 in the cfg),
# instead of third_party/gpd/src/gpd/candidate/frame_estimator.cpp (left
# unchanged). It includes 3_Sample/contour_frames.h, so it needs src/.
set_property(TARGET gpd_frame_estimator PROPERTY SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/3_Sample/gpd_frame_estimator_contour.cpp)
target_include_directories(gpd_frame_estimator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Kinova Kortex API (robot SDK). Creates the target "kortex_api".
# It is downloaded during "cmake" (see third_party/kortex/CMakeLists.txt).
add_subdirectory(third_party/kortex)

# Our headers are included with their folder, starting from src/
# (e.g. "0_Input/depth_camera.h"). The folders follow the steps of
# Algorithm 1 of the GPD paper (ten Pas et al. 2017, p.3).
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# PCL: needed because GPD's headers use PCL types (point clouds).
find_package(PCL 1.15 REQUIRED)

# OpenCV (Ubuntu package, with GStreamer): reads the camera video streams
# (videoio), shows the color image in a window (highgui, view_cloud),
# saves and reads it as .png (imgcodecs) and converts colors to HSV
# (imgproc, 2_GetROI/color_mask), finds the ChArUco board (objdetect) and
# solves the camera pose and the hand-eye calibration (calib3d, Calibration/).
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui imgcodecs objdetect calib3d)

# Program: print the INT and TOOL poses in BASE for given joint angles
# (forward kinematics of src/Robot/Gen3_model.h, no robot connection).
add_executable(print_frames src/Apps/print_frames.cpp src/Robot/robot_connection.cpp src/0_Input/frames.cpp)

# kortex_api: Gen3_model.h and robot_connection.h use the Kortex API.
# Eigen3::Eigen: robot_connection.h includes frames.h, which uses Eigen
# (Eigen3 is found by find_package(PCL) above).
target_link_libraries(print_frames PRIVATE kortex_api Eigen3::Eigen)

# Program: detect grasps in a .pcd file and print the EE pose in BASE for
# each grasp (with the robot pose when the cloud was taken), or the grasps in
# the DEPTH frame (without it).
add_executable(grasps_from_file
  src/Apps/grasps_from_file.cpp
  src/3_Sample/plate_samples.cpp
  src/3_Sample/contour_directions.cpp
  src/2_GetROI/color_mask.cpp
  src/2_GetROI/plate_edge.cpp
  src/2_GetROI/plate_edge_image.cpp
  src/6_SelectGrasp/plate_center.cpp
  src/2_GetROI/largest_cluster.cpp
  src/0_Input/color_cloud.cpp
  src/0_Input/capture_file.cpp
  src/6_SelectGrasp/grasp_to_ee.cpp
  src/6_SelectGrasp/center_grasp.cpp
  src/0_Input/frames.cpp
  src/1_PreprocessCloud/crop_table.cpp
  src/Pipeline/grasp_pipeline.cpp
)
# Where the compiler finds the header files.
# GPD's CMakeLists does not export its include folder, so we add it here.
target_include_directories(grasps_from_file PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gpd/include
  ${PCL_INCLUDE_DIRS}
)
# gpd_grasp_detector = the GPD library (libgpd.so). It brings its own
# dependencies (PCL, OpenCV) with it.
target_link_libraries(grasps_from_file PRIVATE gpd_grasp_detector kortex_api ${OpenCV_LIBS})

# Program: capture a point cloud from the robot's camera, detect grasps with
# GPD and print the EE pose in BASE for each grasp (a capture +
# grasps_from_file in one run, using the TOOL pose measured by the robot).
add_executable(grasps_from_camera
  src/Apps/grasps_from_camera.cpp
  src/3_Sample/plate_samples.cpp
  src/3_Sample/contour_directions.cpp
  src/2_GetROI/color_mask.cpp
  src/2_GetROI/plate_edge.cpp
  src/2_GetROI/plate_edge_image.cpp
  src/6_SelectGrasp/plate_center.cpp
  src/2_GetROI/largest_cluster.cpp
  src/0_Input/color_cloud.cpp
  src/0_Input/depth_camera.cpp
  src/1_PreprocessCloud/depth_filter.cpp
  src/0_Input/vision_module.cpp
  src/0_Input/color_camera.cpp
  src/Robot/robot_connection.cpp
  src/Pipeline/grasp_pipeline.cpp
  src/6_SelectGrasp/grasp_to_ee.cpp
  src/6_SelectGrasp/center_grasp.cpp
  src/0_Input/frames.cpp
  src/1_PreprocessCloud/crop_table.cpp
)
target_include_directories(grasps_from_camera PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gpd/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(grasps_from_camera PRIVATE gpd_grasp_detector kortex_api ${OpenCV_LIBS})

# Program: find the object of one color with the robot's camera and print
# its centroid in BASE (a rough position to send the EE near it). No GPD.
add_executable(locate_object
  src/Apps/locate_object.cpp
  src/2_GetROI/object_position.cpp
  src/2_GetROI/color_mask.cpp
  src/2_GetROI/largest_cluster.cpp
  src/0_Input/color_cloud.cpp
  src/0_Input/capture_file.cpp
  src/0_Input/depth_camera.cpp
  src/1_PreprocessCloud/depth_filter.cpp
  src/0_Input/vision_module.cpp
  src/0_Input/color_camera.cpp
  src/Robot/robot_connection.cpp
  src/0_Input/frames.cpp
  src/1_PreprocessCloud/crop_table.cpp
)
target_include_directories(locate_object PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# PCL libraries used: io (.pcd files), search and kdtree (neighbors) and
# segmentation (largest_cluster).
target_link_libraries(locate_object PRIVATE kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_KDTREE_LIBRARY}
  ${PCL_SEGMENTATION_LIBRARY})

# Program (block 2 of the task): print the EE pose that puts the color camera
# above the object of one color. No GPD.
add_executable(approach_object
  src/Apps/approach_object.cpp
  src/Task/approach_pose.cpp
  src/2_GetROI/object_position.cpp
  src/2_GetROI/color_mask.cpp
  src/2_GetROI/largest_cluster.cpp
  src/0_Input/color_cloud.cpp
  src/0_Input/capture_file.cpp
  src/0_Input/depth_camera.cpp
  src/1_PreprocessCloud/depth_filter.cpp
  src/0_Input/vision_module.cpp
  src/0_Input/color_camera.cpp
  src/Robot/robot_connection.cpp
  src/0_Input/frames.cpp
  src/1_PreprocessCloud/crop_table.cpp
)
target_include_directories(approach_object PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# Same PCL libraries as locate_object.
target_link_libraries(approach_object PRIVATE kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_KDTREE_LIBRARY}
  ${PCL_SEGMENTATION_LIBRARY})

# Program (block 3 of the task): take the views of the sweep (wrist turned
# 0, 90, 180 deg, robot still at each one). No GPD.
add_executable(sweep_capture
  src/Apps/sweep_capture.cpp
  src/Task/sweep_views.cpp
  src/0_Input/depth_camera.cpp
  src/1_PreprocessCloud/depth_filter.cpp
  src/0_Input/vision_module.cpp
  src/0_Input/color_camera.cpp
  src/Robot/robot_connection.cpp
  src/0_Input/frames.cpp
)
target_include_directories(sweep_capture PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# PCL io: saveCapture writes the .pcd files.
target_link_libraries(sweep_capture PRIVATE kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY})

# Program (block 4 of the task): join the views of a sweep into one colored
# cloud (fused.pcd) for GPD. No robot, no GPD.
add_executable(fuse_views
  src/Apps/fuse_views.cpp
  src/Task/fuse_views.cpp
  src/0_Input/capture_file.cpp
  src/0_Input/color_cloud.cpp
  src/0_Input/frames.cpp
  src/1_PreprocessCloud/crop_table.cpp
  src/2_GetROI/color_mask.cpp
  src/2_GetROI/largest_cluster.cpp
)
target_include_directories(fuse_views PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# kortex_api only for its headers (Gen3_model.h, depth_camera.h); PCL io,
# search, kdtree and segmentation as locate_object.
target_link_libraries(fuse_views PRIVATE kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_KDTREE_LIBRARY}
  ${PCL_SEGMENTATION_LIBRARY})

# Program: show the depth camera point cloud in real time (same conversion as
# grasps_from_camera), and save the view shown with the key "s".
add_executable(view_cloud
  src/Apps/view_cloud.cpp
  src/0_Input/depth_camera.cpp
  src/1_PreprocessCloud/depth_filter.cpp
  src/0_Input/vision_module.cpp
  src/0_Input/color_camera.cpp
  src/0_Input/color_cloud.cpp
  src/2_GetROI/color_mask.cpp
  src/Robot/robot_connection.cpp
  src/0_Input/frames.cpp
)
# VTK header folder: same name as the VTK CMake folder found by PCL
# (VTK_DIR = .../cmake/vtk-9.5 -> headers in /usr/include/vtk-9.5).
get_filename_component(VTK_FOLDER_NAME ${VTK_DIR} NAME)
find_path(VTK_HEADERS_DIR vtkSmartPointer.h PATH_SUFFIXES ${VTK_FOLDER_NAME} REQUIRED)
target_include_directories(view_cloud PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${VTK_HEADERS_DIR})
# PCL visualization (the 3D window) needs VTK. PCL and VTK are linked as
# library FILES and the VTK headers are added by folder, because linking the
# PCL/VTK CMake targets brings "VTK::mpi", whose MPI dependency is not
# installed.
# VTK modules used by pcl_visualizer.h (found with "nm" on view_cloud.cpp.o):
#   CommonCore, CommonDataModel, CommonMath, RenderingCore, RenderingLOD, vtksys.
string(REPLACE "vtk-" "" VTK_FOLDER_VERSION ${VTK_FOLDER_NAME})  # "9.5"
set(VIEW_CLOUD_VTK_LIBRARIES "")
foreach(module CommonCore CommonDataModel CommonMath RenderingCore RenderingLOD sys)
  find_library(VTK_${module}_FILE vtk${module}-${VTK_FOLDER_VERSION} REQUIRED)
  list(APPEND VIEW_CLOUD_VTK_LIBRARIES ${VTK_${module}_FILE})
endforeach()
target_link_libraries(view_cloud PRIVATE kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY} ${PCL_VISUALIZATION_LIBRARY}
  ${VIEW_CLOUD_VTK_LIBRARIES})

# ---------- Camera calibration (src/Calibration/, blocks C0 to C4) ----------

# Program (block C0): write the ChArUco board to print (charuco_board.pdf).
# No robot.
add_executable(calib_board
  src/Apps/calib_board.cpp
  src/Calibration/charuco_board.cpp
)
target_include_directories(calib_board PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calib_board PRIVATE ${OpenCV_LIBS})

# Program (block C1): photograph the board from the poses of CALIB_POSES.
add_executable(calib_capture
  src/Apps/calib_capture.cpp
  src/Calibration/calib_poses.cpp
  src/Calibration/board_pose.cpp
  src/Calibration/charuco_board.cpp
  src/0_Input/depth_camera.cpp
  src/1_PreprocessCloud/depth_filter.cpp
  src/0_Input/vision_module.cpp
  src/0_Input/color_camera.cpp
  src/Robot/robot_connection.cpp
  src/0_Input/frames.cpp
)
target_include_directories(calib_capture PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# PCL io: saveCapture writes the .pcd files.
target_link_libraries(calib_capture PRIVATE kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY})
# Kortex has its own protobuf inside; objdetect (ArUco) loads the system one.
# Do not export the Kortex copy, or the system one calls it and crashes.
target_link_options(calib_capture PRIVATE "LINKER:--exclude-libs,ALL")

# Files of the hand-eye programs: the photos, the board in each one and the
# result files (Calibration/hand_eye_data.h). No robot.
set(HAND_EYE_SOURCES
  src/Calibration/hand_eye_data.cpp
  src/Calibration/board_pose.cpp
  src/Calibration/charuco_board.cpp
  src/0_Input/capture_file.cpp
  src/0_Input/frames.cpp
)

# Programs (blocks C3a, C3b, C3c): one hand-eye method each.
foreach(method tsai park daniilidis)
  add_executable(hand_eye_${method}
    src/Apps/hand_eye_${method}.cpp
    src/Calibration/hand_eye_${method}.cpp
    ${HAND_EYE_SOURCES}
  )
  target_include_directories(hand_eye_${method} PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  # kortex_api only for its headers (Gen3_model.h, depth_camera.h).
  target_link_libraries(hand_eye_${method} PRIVATE kortex_api ${OpenCV_LIBS})
endforeach()

# Program (block C4): compare the three methods and suggest the cfg.
add_executable(hand_eye_compare
  src/Apps/hand_eye_compare.cpp
  src/Calibration/compare_hand_eye.cpp
  ${HAND_EYE_SOURCES}
)
target_include_directories(hand_eye_compare PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hand_eye_compare PRIVATE kortex_api ${OpenCV_LIBS})
