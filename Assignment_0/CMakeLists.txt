cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

find_package(Eigen3 CONFIG REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_executable (Transformation main.cpp)
target_link_libraries(Transformation PRIVATE Eigen3::Eigen)