cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_state_machine_commander_src
        state_machine_commander.cpp
        state_machine_commander_callbacks.cpp
)

add_executable(${PROJECT_NAME}_state_machine_commander_node ${${PROJECT_NAME}_state_machine_commander_src} ${${PROJECT_NAME}_common_src})
set_target_properties(${PROJECT_NAME}_state_machine_commander_node PROPERTIES OUTPUT_NAME state_machine_commander_node PREFIX "")
add_dependencies(${PROJECT_NAME}_state_machine_commander_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_state_machine_commander_node ${catkin_LIBRARIES} ${PROJECT_NAME})
