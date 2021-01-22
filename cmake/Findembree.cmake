

# Try to find MOAB
#
# Once done this will define
#
#  EMBREE_FOUND - system has MOAB
#  EMBREE_INCLUDE_DIRS - the MOAB include directory
#  EMBREE_LIBRARIES - Link these to use MOAB
#  EMBREE_DEFINITIONS - Compiler switches required for using MOAB

find_path(EMBREE_INCLUDE_DIR embree3/rtcore.h
  HINTS
  "${EMBREE_DIR}/include"
  "$ENV{EMBREE_ROOT}/include"
  "${EMBREE_DIR}/../include"
  "${EMBREE_DIR}/../../include"
  DOC "Embree headers path")

message(STATUS "Embree include dir: ${EMBREE_INCLUDE_DIR}")

if (EMBREE_INCLUDE_DIR AND EXISTS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_version.h" )
    file(STRINGS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_version.h" TMP REGEX "^#define RTC_VERSION_MAJOR.*$")
    string(REGEX MATCHALL "[0-9]+" MAJOR ${TMP})
    file(STRINGS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_version.h" TMP REGEX "^#define RTC_VERSION_MINOR.*$")
    string(REGEX MATCHALL "[0-9]+" MINOR ${TMP})
    file(STRINGS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_version.h" TMP REGEX "^#define RTC_VERSION_PATCH.*$")
    string(REGEX MATCHALL "[0-9]+" PATCH ${TMP})

    set (EMBREE_VERSION ${MAJOR}.${MINOR}.${PATCH})
endif()

if (NOT DEFINED EMBREE_VERSION AND EXISTS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_config.h" )
    file(STRINGS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_config.h" TMP REGEX "^#define RTC_VERSION_MAJOR.*$")
    string(REGEX MATCHALL "[0-9]+" MAJOR ${TMP})
    file(STRINGS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_config.h" TMP REGEX "^#define RTC_VERSION_MINOR.*$")
    string(REGEX MATCHALL "[0-9]+" MINOR ${TMP})
    file(STRINGS "${EMBREE_INCLUDE_DIR}/embree3/rtcore_config.h" TMP REGEX "^#define RTC_VERSION_PATCH.*$")
    string(REGEX MATCHALL "[0-9]+" PATCH ${TMP})

    set (EMBREE_VERSION ${MAJOR}.${MINOR}.${PATCH})
endif()

if (NOT DEFINED EMBREE_VERSION OR "${EMBREE_VERSION}" STREQUAL "..")
  message(FATAL_ERROR "Could not find Embree 3 in the install location: ${EMBREE_DIR}")
endif()

message(STATUS "Searching for embree ${EMBREE_VERSION}...")

find_path(EMBREE_CMAKE_CONFIG NAMES embree-config.cmake
  HINTS ${EMBREE_DIR}
  PATHS ENV LD_LIBRARY_PATH
  PATH_SUFFIXES lib Lib cmake cmake/embree-${EMBREE_VERSION} lib/cmake/embree-${EMBREE_VERSION}
  NO_DEFAULT_PATH)

if (NOT DEFINED EMBREE_CMAKE_CONFIG)
  message(FATAL_ERROR "Could not find the Embree CMake configuration file.")
endif()


message(STATUS "Found EMBREE in ${EMBREE_CMAKE_CONFIG}")

include(${EMBREE_CMAKE_CONFIG}/embree-config.cmake)
