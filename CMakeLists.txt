cmake_minimum_required(VERSION 3.4)
project( DBLDOWN )
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 14)

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
FIND_PACKAGE (embree REQUIRED)

if (NOT ${EMBREE_VERSION} VERSION_GREATER 3.6.0)
  message(FATAL_ERROR "Double-down requires Embree v3.6.1 or higher.")
endif()

# Set install locations
set( DD_BINARY_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/tools" )
set( DD_INCLUDE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/include" )
set( DD_LIBRARY_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/lib" )
set( DD_CMAKE_CONFIG_LOCATION "${DD_LIBRARY_INSTALL_LOCATION}/cmake")

MESSAGE ( STATUS "Found Embree " ${EMBREE_VERSION} " at " ${EMBREE_PATH} )
MESSAGE ( STATUS "EMBREE_LIBRARY is " ${EMBREE_LIBRARY} )
MESSAGE ( STATUS "EMBREE_INCLUDE_DIRS is " ${EMBREE_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${EMBREE_INCLUDE_DIRS} )

set(CMAKE_SKIP_BUILD_RPATH FALSE)

set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# add library
add_library( dd SHARED "src/RTI.cpp" "src/primitives.cpp" "src/MOABRay.cpp" "src/MOABDirectAccess.cpp")

target_include_directories( dd INTERFACE "${CMAKE_INSTALL_PREFIX}/include" ${EMBREE_INCLUDE_DIRS})
target_link_libraries( dd ${MOAB_LIBRARIES} ${EMBREE_LIBRARY} )
set_target_properties(dd PROPERTIES INSTALL_RPATH "${MOAB_LIBRARY_DIRS};${EMBREE_LIBRARY_DIR}")

install( TARGETS dd
         EXPORT ddTargets
         DESTINATION "${DD_LIBRARY_INSTALL_LOCATION}" )

install(EXPORT ddTargets DESTINATION "${DD_CMAKE_CONFIG_LOCATION}")
export( TARGETS dd FILE "ddTargets.cmake" )

# install headers
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/RTI.hpp" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/primitives.hpp" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/ray.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/MOABRay.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/MOABDirectAccess.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/Vec3da.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/constants.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/sys.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/Vec3fa.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/Vec3ba.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/Vec3.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install ( FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/AABB.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")


# Tests
enable_testing()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ddConfig.cmake.in" "${DD_CMAKE_CONFIG_LOCATION}/ddConfig.cmake")


add_subdirectory(tools)
add_subdirectory(test)
add_subdirectory(test_files)
