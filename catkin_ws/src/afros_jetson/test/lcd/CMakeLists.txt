cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_lcd_test_src
        lcd_test.cpp
)

add_executable(${PROJECT_NAME}_lcd_test_node ${${PROJECT_NAME}_lcd_test_src})
set_target_properties(${PROJECT_NAME}_lcd_test_node PROPERTIES OUTPUT_NAME lcd_test_node PREFIX "")
add_dependencies(${PROJECT_NAME}_lcd_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lcd_test_node ${PROJECT_NAME}_common_lib)
