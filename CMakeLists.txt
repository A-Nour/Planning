cmake_minimum_required(VERSION 3.8)
project(Bachelor)

add_subdirectory(visualizer)
add_subdirectory(path_processor)

include_directories(path_processor/include)
include_directories(include)
add_executable(Bachelor ${SOURCE}
    ${HEADERS} include/Processing.h Processing.cpp include/config.h main.cpp)
target_link_libraries(Bachelor visualizer)

add_custom_command(
    TARGET Bachelor
    POST_BUILD COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        $<TARGET_FILE_DIR:Bachelor>/assets)





