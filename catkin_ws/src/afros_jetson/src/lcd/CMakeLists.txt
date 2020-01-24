cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_lcd_lib_src
        i2c_lcd_device.cpp
)

set(
        ${PROJECT_NAME}_lcd_src
        lcd_node.cpp
)

add_library(${PROJECT_NAME}_lcd_lib STATIC ${${PROJECT_NAME}_lcd_lib_src})
add_dependencies(${PROJECT_NAME}_lcd_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lcd_lib ${PROJECT_NAME}_common_lib)
set(
        ${PROJECT_NAME}_common_export_libs
        ${${PROJECT_NAME}_common_export_libs}
        ${PROJECT_NAME}_lcd_lib
        CACHE STRING "" FORCE
)

add_executable(${PROJECT_NAME}_lcd_node ${${PROJECT_NAME}_lcd_src})
set_target_properties(${PROJECT_NAME}_lcd_node PROPERTIES OUTPUT_NAME lcd_node PREFIX "")
add_dependencies(${PROJECT_NAME}_lcd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lcd_node ${PROJECT_NAME}_common_lib)