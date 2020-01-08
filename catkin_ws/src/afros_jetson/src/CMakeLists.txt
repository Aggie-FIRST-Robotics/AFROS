cmake_minimum_required(VERSION 2.8.3)

set(${PROJECT_NAME}_common_export_libs ${catkin_LIBRARIES} CACHE STRING "" FORCE)
mark_as_advanced(${PROJECT_NAME}_export_libs)

add_subdirectory(lcd)
add_subdirectory(lcd_connection)
add_subdirectory(libi2c)

set(
        ${PROJECT_NAME}_common_src
        common_lib.cpp
)

add_library(${PROJECT_NAME}_common_lib STATIC ${${PROJECT_NAME}_common_src})
target_link_libraries(${PROJECT_NAME}_common_lib ${${PROJECT_NAME}_common_export_libs})
