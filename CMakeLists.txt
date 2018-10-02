
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(MOAB REQUIRED)

set(DD_BINARY_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/bin")

add_subdirectory(apps)
