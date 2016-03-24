cmake_minimum_required(VERSION 2.8.12)
project(com_motionreality_OSVR_SharedIMU)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# This looks for an osvrConfig.cmake file - most of the time it can be
# autodetected but you might need to create/extend CMAKE_PREFIX_PATH to include something like
# C:/Users/Ryan/Desktop/build/OSVR-Core-vc12 or
# C:/Users/Ryan/Downloads/OSVR-Core-Snapshot-v0.1-406-gaa55515-build54-vs12-32bit
# in the CMake GUI or command line.
# Or just set osvr_DIR after the first config fails
find_package(osvr REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories("${EIGEN3_INCLUDE_DIR}")

# This generates a header file, from the named json file, containing a string literal
# named com_osvr_example_selfcontained_json (not null terminated)
# The file must be added as a source file to some target (as below) to be generated.
osvr_convert_json(com_motionreality_OSVR_SharedIMU
    com_motionreality_OSVR_SharedIMU.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_motionreality_OSVR_SharedIMU_json.h")

# Be able to find our generated header file.
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/HIDAPI")

# This is just a helper function wrapping CMake's add_library command that
# sets up include dirs, libraries, and naming convention (no leading "lib")
# for an OSVR plugin. It also installs the plugin into the right directory.
# Pass as many source files as you need. See osvrAddPlugin.cmake for full docs.
osvr_add_plugin(NAME com_motionreality_OSVR_SharedIMU
    CPP # indicates we'd like to use the C++ wrapper
    SOURCES
    com_motionreality_OSVR_SharedIMU.cpp
    HIDAPI/hid.cpp
    HIDAPI/hidapi.h
    "${CMAKE_CURRENT_BINARY_DIR}/com_motionreality_OSVR_SharedIMU_json.h")

# If you use other libraries, find them and add a line like:
# target_link_libraries(com_osvr_example_selfcontained AnyOtherLibraries)