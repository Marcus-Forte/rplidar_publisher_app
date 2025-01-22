find_package(GTest REQUIRED)
include_directories(mocks)

add_executable(test_recorder src/test_recorder.cc)
target_link_libraries(test_recorder scan_recorder gtest_main gtest gmock)
gtest_discover_tests(test_recorder)

add_executable(test_server src/test_server.cc)
target_link_libraries(test_server sensors_server gtest_main gtest gmock)
gtest_discover_tests(test_server)

add_executable(draft draft.cc)
target_link_libraries(draft scan_recorder)