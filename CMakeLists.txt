cmake_minimum_required(VERSION 3.2)

project(Cartesian_Impedance_Gen3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler warnings

if (MSVC)

    add_compile_options(
        /W4
        /WX
    )

else()

    add_compile_options(
        -Wall
        -Wextra
        -pedantic
    )

endif()

# ------------------------------------------------------------
# External dependencies
# ------------------------------------------------------------

find_package(PkgConfig REQUIRED)

pkg_check_modules(
    GTK3
    REQUIRED
    IMPORTED_TARGET
    gtk+-3.0
)

find_package(Eigen3 REQUIRED)

# Custom CMake modules

list(
    APPEND
    CMAKE_MODULE_PATH
    "${CMAKE_SOURCE_DIR}/cmake"
)


add_subdirectory(
    Dependencies/Boost
)

add_subdirectory(
    Dependencies/Kinova
)

add_subdirectory(
    Matrix
)

add_subdirectory(
    KinovaRobot
)

add_subdirectory(
    DataLogging
)

# CARTESIAN IMPEDANCE CONTROLLER LIBRARY

add_library(
    cartesian_impedance

    src/cartesian_impedance_controller.cpp
)

target_include_directories(
    cartesian_impedance

    PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    cartesian_impedance

    PUBLIC

    matrix
    Eigen3::Eigen
)

# MAIN KINOVA IMPEDANCE EXECUTABLE

add_executable(
    impedance_gen3

    src/main_impedance.cpp
)

target_include_directories(
    impedance_gen3

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    impedance_gen3

    PRIVATE

    matrix
    kinova_robot
    kortex_api

    cartesian_impedance
    Eigen3::Eigen

    PkgConfig::GTK3
    data_logging
    Boost::numeric_odeint
)

# LOCAL OPERATIONAL PD (HARDWARE) + SHADOW IMPEDANCE

add_executable(
    op_space_local

    src/main_op_space_local.cpp
)

target_include_directories(
    op_space_local

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    op_space_local

    PRIVATE

    matrix
    kinova_robot
    kortex_api

    cartesian_impedance
    Eigen3::Eigen

    PkgConfig::GTK3
    Boost::numeric_odeint   # RK4 del observador de friccion
)

# ANALYTIC MODEL SMOKE TEST

add_executable(
    model_smoke_test

    tests/model_smoke_test.cpp
)

target_include_directories(
    model_smoke_test

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    model_smoke_test

    PRIVATE

    matrix
    Eigen3::Eigen
)

# JACOBIAN VALIDATION TEST

add_executable(
    jacobian_validation_test
    tests/jacobian_validation_test.cpp
)

target_include_directories(
    jacobian_validation_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    jacobian_validation_test
    PRIVATE
    matrix
)

# VISION (GPD) INTEGRATION: pipeline tests and test sender

add_executable(
    grasp_pipeline_test
    tests/grasp_pipeline_test.cpp
)

target_include_directories(
    grasp_pipeline_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    grasp_pipeline_test
    PRIVATE
    Eigen3::Eigen
)

add_executable(
    grasp_test_sender
    tools/grasp_test_sender.cpp
)

target_include_directories(
    grasp_test_sender
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# FASE 1 (FIXTURE Y PINZA): pruebas por separado

add_executable(
    fixture_test
    tools/fixture_test.cpp
)

target_include_directories(
    fixture_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(
    gripper_test
    tools/gripper_test.cpp
)

target_include_directories(
    gripper_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    gripper_test
    PRIVATE
    matrix
    kinova_robot
    kortex_api
)

# FASE 2: prueba de la secuencia de tarea (sin robot)

add_executable(
    task_sequence_test
    tests/task_sequence_test.cpp
)

target_include_directories(
    task_sequence_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    task_sequence_test
    PRIVATE
    Eigen3::Eigen
)

# FASE 3: prueba de la vision de tornillos (screw_server de kinova_grasp),
# sin robot

add_executable(
    vision_request_test
    tools/vision_request_test.cpp
)

target_include_directories(
    vision_request_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    vision_request_test
    PRIVATE
    Eigen3::Eigen
)

# Prueba del espacio nulo (postura del codo) con el Jacobiano del modelo,
# sin robot

add_executable(
    nullspace_test
    tests/nullspace_test.cpp
)

target_include_directories(
    nullspace_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    nullspace_test
    PRIVATE
    cartesian_impedance
    matrix
    Eigen3::Eigen
)

# Interfaz grafica (gui/): secuencia, poses y parametros en JSON (sin robot)

add_executable(
    task_info
    tools/task_info.cpp
)

target_include_directories(
    task_info
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    task_info
    PRIVATE
    Eigen3::Eigen
)
