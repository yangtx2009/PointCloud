include("$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

cmake_minimum_required(VERSION 3.0.0)
project(PointCloud VERSION 0.1.0)

find_package(PCL 1.3 REQUIRED)  #  COMPONENTS common io
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include(CTest)
enable_testing()

add_executable(PointCloud main.cpp)

target_link_libraries(PointCloud ${PCL_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
