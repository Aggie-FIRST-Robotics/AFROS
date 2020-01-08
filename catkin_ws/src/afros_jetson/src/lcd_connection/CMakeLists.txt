cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_lcd_connection_lib_src
        lcd_connection.cpp
)

add_library(${PROJECT_NAME}_lcd_connection_lib STATIC ${${PROJECT_NAME}_lcd_connection_lib_src})
add_dependencies(${PROJECT_NAME}_lcd_connection_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lcd_connection_lib ${PROJECT_NAME}_common_lib)
set(
        ${PROJECT_NAME}_common_export_libs
        ${${PROJECT_NAME}_common_export_libs}
        ${PROJECT_NAME}_lcd_connection_lib
        CACHE STRING "" FORCE
)
