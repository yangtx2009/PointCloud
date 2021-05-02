cmake_minimum_required(VERSION 3.0.0)
project(PointCloud VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(src)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(src)

add_executable(PointCloud
    main.cpp
)

target_link_libraries (PointCloud ${PCL_LIBRARIES} Libraries)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
