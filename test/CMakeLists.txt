cmake_minimum_required(VERSION 3.10)

add_subdirectory(googletest EXCLUDE_FROM_ALL)
# ensure gtest include directory is added before any of the
# system-wide include directories that might contain gtest
include_directories(BEFORE ${gtest_SOURCE_DIR}/include)

include(GoogleTest)

enable_testing()


# NOTE: We add ceres to tests, in case students use ceres::sin etc in camera model templates.

add_executable(test_ex0 src/test_ex0.cpp)
target_link_libraries(test_ex0 gtest gtest_main Ceres::ceres)

add_executable(test_ex1 src/test_ex1.cpp)
target_link_libraries(test_ex1 gtest gtest_main Ceres::ceres)

add_executable(test_ex2 src/test_ex2.cpp)
target_link_libraries(test_ex2 gtest gtest_main Ceres::ceres)

add_executable(test_ex3 src/test_ex3.cpp)
target_link_libraries(test_ex3 gtest gtest_main Ceres::ceres pangolin TBB::tbb OpenCV opengv)

add_executable(test_ex4 src/test_ex4.cpp)
target_link_libraries(test_ex4 gtest gtest_main Ceres::ceres opengv TBB::tbb)

add_executable(test_ex5 src/test_ex5.cpp)
target_link_libraries(test_ex5 gtest gtest_main Ceres::ceres opengv TBB::tbb)

add_executable(test_icp src/test_icp.cpp)
target_link_libraries(test_icp Ceres::ceres TBB::tbb ${PCL_LIBRARIES})

add_executable(test_icp_ceres src/test_icp_ceres.cpp)
target_link_libraries(test_icp_ceres Ceres::ceres TBB::tbb ${PCL_LIBRARIES})

add_executable(test_icp_flann src/test_icp_flann.cpp)
target_link_libraries(test_icp_flann Ceres::ceres TBB::tbb ${PCL_LIBRARIES})

add_executable(test_voxelization src/test_voxelization.cpp)
target_link_libraries(test_voxelization Ceres::ceres TBB::tbb ${PCL_LIBRARIES})

gtest_discover_tests(test_ex0 DISCOVERY_TIMEOUT 120)
gtest_discover_tests(test_ex1 DISCOVERY_TIMEOUT 120)
gtest_discover_tests(test_ex2 DISCOVERY_TIMEOUT 120)
add_test(NAME ex2_calibration_test COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test_ex2.sh WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
gtest_discover_tests(test_ex3 DISCOVERY_TIMEOUT 120)
gtest_discover_tests(test_ex4 DISCOVERY_TIMEOUT 120)
gtest_discover_tests(test_ex5 DISCOVERY_TIMEOUT 120)
