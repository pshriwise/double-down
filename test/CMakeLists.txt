

add_executable(test_mb test_mb.cpp test_utils.cpp)
target_include_directories(test_mb PUBLIC ${MOAB_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(test_mb ${MOAB_LIBRARIES})

add_test(NAME testy_test COMMAND test_mb)


