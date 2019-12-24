cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_libi2c_src
        i2c.c
)

add_library(${PROJECT_NAME}_libi2c STATIC ${${PROJECT_NAME}_libi2c_src})
set(
        ${PROJECT_NAME}_common_export_libs
        ${${PROJECT_NAME}_common_export_libs}
        ${PROJECT_NAME}_libi2c
        CACHE STRING "" FORCE
)
