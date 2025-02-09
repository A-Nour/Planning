cmake_minimum_required(VERSION 3.5)

project(PathPlanner LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(path_processor)


find_package(Qt5 COMPONENTS Widgets REQUIRED)
include_directories(path_processor/include
                    include)

add_executable(${PROJECT_NAME}
main.cpp
mainwindow.cpp
mainwindow.h
ImageProcessor.cpp
ImageProcessor.h
)


target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Widgets PathProcessor)
