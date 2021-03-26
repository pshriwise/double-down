
add_executable(ray_fire ray_fire)
target_include_directories(ray_fire PUBLIC ${MOAB_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/)
target_link_libraries(ray_fire ${MOAB_LIBRARIES} ${EMBREE_LIBRARY} dd)
target_link_libraries(ray_fire ${EMBREE_LIBRARY})
install(TARGETS ray_fire DESTINATION ${DD_BINARY_INSTALL_LOCATION})

add_executable(closest_to_location closest_to_location)
target_include_directories(closest_to_location PUBLIC ${MOAB_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/)
target_link_libraries(closest_to_location ${MOAB_LIBRARIES} ${EMBREE_LIBRARY} dd)
set_target_properties(closest_to_location PROPERTIES BUILD_RPATH "${MOAB_LIBRARY_DIRS}")
set_target_properties(closest_to_location PROPERTIES BUILD_RPATH_USE_LINK_PATH TRUE)

install(TARGETS closest_to_location DESTINATION ${DD_BINARY_INSTALL_LOCATION})

