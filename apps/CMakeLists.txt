

add_executable(moab_rf moab_rf.cpp ../RTI.cpp)
target_include_directories(moab_rf PUBLIC ${MOAB_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(moab_rf ${MOAB_LIBRARIES})
target_link_libraries(moab_rf ${EMBREE_LIBRARY})
install(TARGETS moab_rf DESTINATION ${DD_BINARY_INSTALL_LOCATION})
