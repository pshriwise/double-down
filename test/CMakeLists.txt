

add_executable(test_mb test_mb.cpp test_utils.cpp)
target_include_directories(test_mb PUBLIC ${MOAB_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src/)
target_link_libraries(test_mb ${MOAB_LIBRARIES})

add_executable(test_rf test_rf.cpp test_utils.cpp)
target_include_directories(test_rf PUBLIC ${MOAB_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src/)
target_link_libraries(test_rf ${MOAB_LIBRARIES} ${EMBREE_LIBRARY} dd)


add_test(NAME test_moab_write COMMAND test_mb)
add_test(NAME test_rti_init COMMAND test_rf)
