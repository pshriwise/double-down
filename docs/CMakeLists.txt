

# Configure the documentation file

find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
else()
  message(WARNING "Doxygen is required to build the documentation for double-down.")
endif()