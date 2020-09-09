cmake_minimum_required(VERSION 3.1)
project(WATTMAN)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)
include_directories(
        ${PCL_INCLUDE_DIRS}
        )
add_executable(WATTMAN
        main.cpp
        point2plane_ICP.cpp
        )

target_link_libraries (WATTMAN
        ${PCL_LIBRARIES}
        )