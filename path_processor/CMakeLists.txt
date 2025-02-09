cmake_minimum_required(VERSION 3.10)
project(PathProcessor)

set(CMAKE_CXX_STANDARD 17)

# Append to SOURCE and HEADERS
set(SOURCE
   ${CMAKE_CURRENT_SOURCE_DIR}/Dijkstra.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/Path.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/PathProcessor.cpp
)

set(HEADERS
   ${CMAKE_CURRENT_SOURCE_DIR}/include/Dijkstra.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/Path.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/PathProcessor.h
)

# Create the library
add_library(${PROJECT_NAME}
    ${SOURCE} ${HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
