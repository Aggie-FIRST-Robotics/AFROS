cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_connection_commander_lib_src
        connection_commander_callbacks.cpp
)

set(
        ${PROJECT_NAME}_connection_commander_src
        connection_commander.cpp
)

add_library(${PROJECT_NAME}_connection_commander_lib STATIC ${${PROJECT_NAME}_connection_commander_lib_src})
add_dependencies(${PROJECT_NAME}_connection_commander_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_connection_commander_lib ${PROJECT_NAME}_common_lib)
set(
        ${PROJECT_NAME}_common_export_libs
        ${${PROJECT_NAME}_common_export_libs}
        ${PROJECT_NAME}_connection_commander_lib
        CACHE STRING "" FORCE
)

add_executable(${PROJECT_NAME}_connection_commander_node ${${PROJECT_NAME}_connection_commander_src})
set_target_properties(${PROJECT_NAME}_connection_commander_node PROPERTIES OUTPUT_NAME connection_commander_node PREFIX "")
add_dependencies(${PROJECT_NAME}_connection_commander_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_connection_commander_node ${PROJECT_NAME}_common_lib)
