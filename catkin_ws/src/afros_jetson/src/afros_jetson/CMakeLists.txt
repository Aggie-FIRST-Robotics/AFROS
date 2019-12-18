cmake_minimum_required(VERSION 2.8.3)

add_subdirectory(lcd_node)

set(
        ${PROJECT_NAME}_common_src
        common_lib.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_common_src})
