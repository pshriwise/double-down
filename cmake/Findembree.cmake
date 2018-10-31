

# Try to find MOAB
#
# Once done this will define
#
#  EMBREE_FOUND - system has MOAB
#  EMBREE_INCLUDE_DIRS - the MOAB include directory
#  EMBREE_LIBRARIES - Link these to use MOAB
#  EMBREE_DEFINITIONS - Compiler switches required for using MOAB

message(STATUS "Searching for embree...")

find_path(EMBREE_CMAKE_CONFIG NAMES embree-config.cmake
          HINTS ${EMBREE_ROOT}
          PATHS ENV LD_LIBRARY_PATH
          PATH_SUFFIXES lib Lib cmake cmake/embree-2.16.3
          NO_DEFAULT_PATH)
message(STATUS "Found EMBREE in ${EMBREE_CMAKE_CONFIG}")

include(${EMBREE_CMAKE_CONFIG}/embree-config.cmake)
