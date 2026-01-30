cmake_minimum_required(VERSION 3.10)

project(BezierCurves)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)

add_executable(BezierCurves main.cpp)

target_link_libraries(BezierCurves PRIVATE ${OpenCV_LIBRARIES} Eigen3::Eigen )
