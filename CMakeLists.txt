cmake_minimum_required(VERSION 3.4)
project( DBLDOWN )
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 14)
set(FIND_LIBRARY_USE_LIB64_PATHS TRUE)

set(DOUBLE_DOWN_VERSION_MAJOR 1)
set(DOUBLE_DOWN_VERSION_MINOR 1)
set(DOUBLE_DOWN_VERSION_RELEASE 0)
set(DOUBLE_DOWN_VERSION ${DOUBLE_DOWN_VERSION_MAJOR}.${DOUBLE_DOWN_VERSION_MINOR}.${DOUBLE_DOWN_VERSION_RELEASE})

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -march=native")

IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64") # for desktop
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx2")
ENDIF()

# MOAB
find_package(MOAB REQUIRED HINTS ${MOAB_DIR})
MESSAGE ( STATUS "Found MOAB Version: " ${MOAB_VERSION} )
MESSAGE ( STATUS "MOAB_LIBRARIES is " ${MOAB_LIBRARIES} )
MESSAGE ( STATUS "MOAB_INCLUDE_DIRS is " ${MOAB_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${MOAB_INCLUDE_DIRS} )

# Embree
# this series of calls is needed to support the way that Embree indicates
# compatible versions in 3 and 4 (version 4 does it correctly, but 3 does not)
FIND_PACKAGE(embree 3.6.1 HINTS ${EMBREE_DIR} QUIET)
if (NOT ${embree_FOUND})
FIND_PACKAGE (embree REQUIRED HINTS ${EMBREE_DIR})
endif()

if (NOT ${EMBREE_VERSION} VERSION_GREATER 3.6.0)
  message(FATAL_ERROR "Double-down requires Embree v3.6.1 or higher.")
endif()

# set embree version

# Set install locations
set( DD_BINARY_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/tools" )
set( DD_INCLUDE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/include/double_down" )
set( DD_LIBRARY_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/lib" )
set( DD_CMAKE_CONFIG_LOCATION "${DD_LIBRARY_INSTALL_LOCATION}/cmake/dd")

MESSAGE ( STATUS "Found Embree " ${EMBREE_VERSION} " at " ${EMBREE_PATH} )
MESSAGE ( STATUS "EMBREE_LIBRARY is " ${EMBREE_LIBRARY} )
MESSAGE ( STATUS "EMBREE_INCLUDE_DIRS is " ${EMBREE_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${EMBREE_INCLUDE_DIRS} )

set(CMAKE_SKIP_BUILD_RPATH FALSE)

set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# add library
add_library(dd SHARED "src/RTI.cpp" "src/primitives.cpp" "src/MOABRay.cpp" "src/MOABDirectAccess.cpp")
target_include_directories(dd PRIVATE ${CMAKE_SOURCE_DIR}/include)
set_target_properties(dd PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(dd PROPERTIES INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib )

# pass precompile definition depending on embree version
if (${EMBREE_VERSION} VERSION_GREATER_EQUAL 4.0.0)
  target_compile_definitions(dd PUBLIC EMBREE4)
else()
  target_compile_definitions(dd PUBLIC EMBREE3)
endif()

target_include_directories( dd INTERFACE "${CMAKE_INSTALL_PREFIX}/include" ${EMBREE_INCLUDE_DIRS})
target_link_libraries( dd ${MOAB_LIBRARIES} ${EMBREE_LIBRARY} )
set_target_properties(dd PROPERTIES INSTALL_RPATH "${MOAB_LIBRARY_DIRS};${EMBREE_LIBRARY_DIR}")

install( TARGETS dd
         EXPORT ddTargets
         DESTINATION "${DD_LIBRARY_INSTALL_LOCATION}" )

install(EXPORT ddTargets DESTINATION "${DD_CMAKE_CONFIG_LOCATION}")
export( TARGETS dd FILE "ddTargets.cmake" )

find_package(Git)
# Set git SHA1 hash as a compile definition
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  RESULT_VARIABLE GIT_SHA1_SUCCESS
                  OUTPUT_VARIABLE GIT_SHA1
                  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(GIT_SHA1_SUCCESS EQUAL 0)
    target_compile_definitions(dd PRIVATE -DGIT_SHA1="${GIT_SHA1}")
  endif()
endif()

# install headers
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/RTI.hpp" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/primitives.hpp" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/ray.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/MOABRay.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/MOABDirectAccess.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/Vec3da.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/constants.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/sys.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/Vec3fa.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/Vec3ba.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/Vec3.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/embree3.hpp" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/embree4.hpp" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/embree_interface.hpp" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")

# Tests
enable_testing()

# Configure CMake and version files
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ddConfig.cmake.in" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ddConfig.cmake")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ddConfig.cmake" DESTINATION "${DD_CMAKE_CONFIG_LOCATION}")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ddConfigVersion.cmake.in" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ddConfigVersion.cmake")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ddConfigVersion.cmake" DESTINATION "${DD_CMAKE_CONFIG_LOCATION}")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/version.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/version.h")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/double_down/version.h" DESTINATION "${DD_INCLUDE_INSTALL_LOCATION}")

add_subdirectory(docs)
add_subdirectory(tools)
add_subdirectory(test)
add_subdirectory(test_files)
