

add_executable(moab_rf
  moab_rf.cpp
  ../src/RTI.cpp
  ../src/primitives.cpp
  ../src/MOABRay.cpp
  ../src/MOABDirectAccess.cpp)
target_include_directories(moab_rf PUBLIC ${MOAB_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src/)
target_link_libraries(moab_rf ${MOAB_LIBRARIES})
target_link_libraries(moab_rf ${EMBREE_LIBRARY})
install(TARGETS moab_rf DESTINATION ${DD_BINARY_INSTALL_LOCATION})
