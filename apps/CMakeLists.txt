

add_executable(moab_rf moab_rf.cpp)
target_include_directories(moab_rf PUBLIC ${MOAB_INCLUDE_DIRS})
target_link_libraries(moab_rf ${MOAB_LIBRARIES})
install(TARGETS moab_rf DESTINATION ${DD_BINARY_INSTALL_LOCATION})
