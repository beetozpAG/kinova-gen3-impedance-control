cmake_minimum_required(VERSION 3.15)
project(LowLatencyList CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(low_latency_list src/block_list_node.cpp)
target_include_directories(low_latency_list PUBLIC include)


if(NOT DEFINED BUILD_TESTS)
	if (CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
		set(BUILD_TESTS TRUE)
	endif()
endif()

if(BUILD_TESTS)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/releases/download/v1.16.0/googletest-1.16.0.tar.gz)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(list_tests tests/block_list_node_tests.cpp tests/low_latency_list_tests.cpp)
target_link_libraries(list_tests GTest::gtest_main low_latency_list)

include(GoogleTest)
gtest_discover_tests(list_tests)
add_executable(benchmark_low_latency_list tests/benchmark.cpp)
target_link_libraries(benchmark_low_latency_list low_latency_list)
endif()

