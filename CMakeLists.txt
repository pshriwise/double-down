cmake_minimum_required(VERSION 2.8)
project( DBLDOWN )
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 11)

# Default to a release build
set(default_build_type "Release")
 
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -march=native -mavx2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -march=native -mavx2")

# MOAB
find_package( MOAB REQUIRED )
MESSAGE ( STATUS "Found MOAB Version: " ${MOAB_VERSION} )
MESSAGE ( STATUS "MOAB_LIBRARIES is " ${MOAB_LIBRARIES} )
MESSAGE ( STATUS "MOAB_INCLUDE_DIRS is " ${MOAB_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${MOAB_INCLUDE_DIRS} )

# Embree
FIND_PACKAGE ( embree 2.0 REQUIRED )

MESSAGE ( STATUS "Found Embree " ${EMBREE_VERSION} " at " ${EMBREE_PATH} )
MESSAGE ( STATUS "EMBREE_LIBRARY is " ${EMBREE_LIBRARY} )
MESSAGE ( STATUS "EMBREE_INCLUDE_DIRS is " ${EMBREE_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${EMBREE_INCLUDE_DIRS} )

set( DD_BINARY_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/bin" )

enable_testing()

add_subdirectory(apps)
add_subdirectory(test)

