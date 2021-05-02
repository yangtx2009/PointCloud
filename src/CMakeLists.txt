cmake_minimum_required(VERSION 3.0.0)
project(Libraries VERSION 0.1.0)

if (NOT PCL_FOUND)
    find_package(PCL 1.10 REQUIRED)
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES example.cpp
            parser.cpp
            viewer.cpp     
            surface.cpp
            utils.cpp
)
set(HEADERS example.h
            parser.h
            viewer.h
            surface.h
            utils.h
)

add_library(Libraries
    ${SOURCES}
    ${HEADERS}
)

target_link_libraries (Libraries ${PCL_LIBRARIES})