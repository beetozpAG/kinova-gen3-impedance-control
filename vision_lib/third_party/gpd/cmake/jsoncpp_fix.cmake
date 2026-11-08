# KINOVA CHANGE: Fix for a CMake error on Ubuntu 26.04 (VTK 9.5).
#
# The error:
#   The link interface of target "VTK::jsoncpp" contains:
#     JsonCpp::JsonCpp
#   but the target was not found.
#
# Why it happens, step by step:
#   1. GPD asks for PCL: find_package(PCL ...) in CMakeLists.txt, line 10.
#   2. PCL loads VTK (used to display point clouds).
#   3. VTK 9.5 needs JsonCpp, but does not load it by itself.
#   4. CMake cannot find JsonCpp and stops.
#
# The fix: load JsonCpp first, before VTK asks for it.
# This is a problem of the Ubuntu VTK/PCL packages, not of GPD.
#
# Usage (from the gpd folder):
#   cmake -S . -B build -DCMAKE_PROJECT_INCLUDE=$PWD/cmake/jsoncpp_fix.cmake

find_package(jsoncpp CONFIG QUIET)
