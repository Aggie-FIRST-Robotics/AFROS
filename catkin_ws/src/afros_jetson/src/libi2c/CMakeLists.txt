cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_libi2c_src
        i2c.c
)

add_library(${PROJECT_NAME}_libi2c ${${PROJECT_NAME}_libi2c_src})
