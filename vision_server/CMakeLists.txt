# Servidor de vision del MODULAR (screw_server) sobre kinova_grasp_NEW_VER.
#
# No cambia el repositorio de Antony: toma sus fuentes y su copia de GPD de
# KG_DIR y compila solo screw_server. Compilar EN UNA CARPETA DENTRO DE KG_DIR
# (las rutas de cfg/gpd_params.cfg empiezan en "../"):
#   cmake -S ~/Escritorio/MODULAR/servidor_vision \
#         -B ~/Escritorio/kinova_grasp_NEW_VER/build_modular \
#         -DKORTEX_DIR=~/Escritorio/MODULAR/Controlador_Impedancia/build/_deps/kortex_api_rel-src
#   cmake --build ~/Escritorio/kinova_grasp_NEW_VER/build_modular -j8
cmake_minimum_required(VERSION 3.16)
project(servidor_vision_modular CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(KG_DIR "$ENV{HOME}/Escritorio/kinova_grasp_NEW_VER" CACHE PATH
    "kinova_grasp de Antony (fuentes y third_party/gpd)")

# Igual que el CMakeLists de KG_DIR (orden incluido).
include(${KG_DIR}/third_party/gpd/cmake/jsoncpp_fix.cmake)
add_subdirectory(${KG_DIR}/third_party/gpd ${CMAKE_BINARY_DIR}/gpd EXCLUDE_FROM_ALL)

# Graficas de GPD: la copia de Antony + cierre automatico de cada ventana
# (GPD_PLOT_SECONDS) con un evento de tecla "q" por X11.
set_property(TARGET gpd_plot PROPERTY SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gpd_plot_2f85_auto.cpp)
find_package(X11 REQUIRED)
target_link_libraries(gpd_plot ${X11_LIBRARIES})

# Marcos locales de GPD desde el contorno (contour_frames), como en KG_DIR.
set_property(TARGET gpd_frame_estimator PROPERTY SOURCES
  ${KG_DIR}/src/3_Sample/gpd_frame_estimator_contour.cpp)
target_include_directories(gpd_frame_estimator PRIVATE ${KG_DIR}/src)

add_subdirectory(${KG_DIR}/third_party/kortex ${CMAKE_BINARY_DIR}/kortex)
include_directories(${KG_DIR}/src)

# KG_DIR pide PCL 1.15 (Ubuntu 26.04); esta PC tiene 1.14 (Ubuntu 24.04).
find_package(PCL 1.14 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui imgcodecs)

add_executable(screw_server
  src/screw_server.cpp
  ${KG_DIR}/src/0_Input/capture_file.cpp
  ${KG_DIR}/src/0_Input/color_camera.cpp
  ${KG_DIR}/src/0_Input/color_cloud.cpp
  ${KG_DIR}/src/0_Input/depth_camera.cpp
  ${KG_DIR}/src/0_Input/frames.cpp
  ${KG_DIR}/src/0_Input/vision_module.cpp
  ${KG_DIR}/src/1_PreprocessCloud/crop_table.cpp
  ${KG_DIR}/src/1_PreprocessCloud/depth_filter.cpp
  ${KG_DIR}/src/2_GetROI/color_mask.cpp
  ${KG_DIR}/src/2_GetROI/largest_cluster.cpp
  ${KG_DIR}/src/2_GetROI/object_position.cpp
  ${KG_DIR}/src/2_GetROI/plate_edge.cpp
  ${KG_DIR}/src/2_GetROI/plate_edge_image.cpp
  ${KG_DIR}/src/3_Sample/contour_directions.cpp
  ${KG_DIR}/src/3_Sample/plate_samples.cpp
  ${KG_DIR}/src/6_SelectGrasp/center_grasp.cpp
  ${KG_DIR}/src/6_SelectGrasp/plate_center.cpp
  ${KG_DIR}/src/6_SelectGrasp/grasp_to_ee.cpp
  ${KG_DIR}/src/Pipeline/grasp_pipeline.cpp
  ${KG_DIR}/src/Robot/robot_connection.cpp
  ${KG_DIR}/src/Task/approach_pose.cpp
)
target_include_directories(screw_server PRIVATE
  ${KG_DIR}/third_party/gpd/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(screw_server PRIVATE gpd_grasp_detector kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY})

# grasps_from_file de Antony (sin cambios): la GUI lo usa para "Calcular
# agarre con GPD" sobre una captura guardada (screw_HHMMSS.pcd + _pose.txt).
add_executable(grasps_from_file
  ${KG_DIR}/src/Apps/grasps_from_file.cpp
  ${KG_DIR}/src/3_Sample/plate_samples.cpp
  ${KG_DIR}/src/3_Sample/contour_directions.cpp
  ${KG_DIR}/src/2_GetROI/color_mask.cpp
  ${KG_DIR}/src/2_GetROI/plate_edge.cpp
  ${KG_DIR}/src/2_GetROI/plate_edge_image.cpp
  ${KG_DIR}/src/2_GetROI/largest_cluster.cpp
  ${KG_DIR}/src/0_Input/color_cloud.cpp
  ${KG_DIR}/src/0_Input/capture_file.cpp
  ${KG_DIR}/src/6_SelectGrasp/grasp_to_ee.cpp
  ${KG_DIR}/src/6_SelectGrasp/center_grasp.cpp
  ${KG_DIR}/src/6_SelectGrasp/plate_center.cpp
  ${KG_DIR}/src/0_Input/frames.cpp
  ${KG_DIR}/src/1_PreprocessCloud/crop_table.cpp
  ${KG_DIR}/src/Pipeline/grasp_pipeline.cpp
)
target_include_directories(grasps_from_file PRIVATE
  ${KG_DIR}/third_party/gpd/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(grasps_from_file PRIVATE gpd_grasp_detector kortex_api ${OpenCV_LIBS})

# approach_object y grasps_from_camera de Antony (sin cambios), para el
# procedimiento a mano SIN el controlador (robot movido con la app web: leen la
# pose de la herramienta que reporta el robot). Con el controlador corriendo
# se usa APPROACH / CAPTURE del screw_server (la pose la manda el controlador).
add_executable(approach_object
  ${KG_DIR}/src/Apps/approach_object.cpp
  ${KG_DIR}/src/Task/approach_pose.cpp
  ${KG_DIR}/src/2_GetROI/object_position.cpp
  ${KG_DIR}/src/2_GetROI/color_mask.cpp
  ${KG_DIR}/src/2_GetROI/largest_cluster.cpp
  ${KG_DIR}/src/0_Input/color_cloud.cpp
  ${KG_DIR}/src/0_Input/capture_file.cpp
  ${KG_DIR}/src/0_Input/depth_camera.cpp
  ${KG_DIR}/src/1_PreprocessCloud/depth_filter.cpp
  ${KG_DIR}/src/0_Input/vision_module.cpp
  ${KG_DIR}/src/0_Input/color_camera.cpp
  ${KG_DIR}/src/Robot/robot_connection.cpp
  ${KG_DIR}/src/0_Input/frames.cpp
  ${KG_DIR}/src/1_PreprocessCloud/crop_table.cpp
)
target_include_directories(approach_object PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(approach_object PRIVATE kortex_api ${OpenCV_LIBS}
  ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_SEGMENTATION_LIBRARY}
  ${PCL_KDTREE_LIBRARY})

add_executable(grasps_from_camera
  ${KG_DIR}/src/Apps/grasps_from_camera.cpp
  ${KG_DIR}/src/3_Sample/plate_samples.cpp
  ${KG_DIR}/src/3_Sample/contour_directions.cpp
  ${KG_DIR}/src/2_GetROI/color_mask.cpp
  ${KG_DIR}/src/2_GetROI/plate_edge.cpp
  ${KG_DIR}/src/2_GetROI/plate_edge_image.cpp
  ${KG_DIR}/src/2_GetROI/largest_cluster.cpp
  ${KG_DIR}/src/0_Input/color_cloud.cpp
  ${KG_DIR}/src/0_Input/depth_camera.cpp
  ${KG_DIR}/src/1_PreprocessCloud/depth_filter.cpp
  ${KG_DIR}/src/0_Input/vision_module.cpp
  ${KG_DIR}/src/0_Input/color_camera.cpp
  ${KG_DIR}/src/Robot/robot_connection.cpp
  ${KG_DIR}/src/Pipeline/grasp_pipeline.cpp
  ${KG_DIR}/src/6_SelectGrasp/grasp_to_ee.cpp
  ${KG_DIR}/src/6_SelectGrasp/center_grasp.cpp
  ${KG_DIR}/src/6_SelectGrasp/plate_center.cpp
  ${KG_DIR}/src/0_Input/frames.cpp
  ${KG_DIR}/src/1_PreprocessCloud/crop_table.cpp
)
target_include_directories(grasps_from_camera PRIVATE
  ${KG_DIR}/third_party/gpd/include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(grasps_from_camera PRIVATE gpd_grasp_detector kortex_api ${OpenCV_LIBS})
