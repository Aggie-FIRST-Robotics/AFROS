cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_lcd_src
        lcd_node.cpp
        i2c_lcd_device.cpp
)

add_executable(${PROJECT_NAME}_lcd_node ${${PROJECT_NAME}_lcd_src} ${${PROJECT_NAME}_common_src})
set_target_properties(${PROJECT_NAME}_lcd_node PROPERTIES OUTPUT_NAME lcd_node PREFIX "")
add_dependencies(${PROJECT_NAME}_lcd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lcd_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_lcd_node ${PROJECT_NAME}_libi2c)
