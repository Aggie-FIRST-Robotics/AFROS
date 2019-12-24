cmake_minimum_required(VERSION 2.8.3)

set(${PROJECT_NAME}_common_export_libs ${catkin_LIBRARIES} CACHE STRING "" FORCE)
mark_as_advanced(${PROJECT_NAME}_export_libs)

add_subdirectory(connection)
add_subdirectory(connection_commander)
add_subdirectory(device_id)
add_subdirectory(direct_control)
add_subdirectory(direct_control_commander)
add_subdirectory(main_commander)
add_subdirectory(state_machine)
add_subdirectory(state_machine_commander)

set(
        ${PROJECT_NAME}_common_src
        common_lib.cpp
)

add_library(${PROJECT_NAME}_common_lib STATIC ${${PROJECT_NAME}_common_src})
target_link_libraries(${PROJECT_NAME}_common_lib ${${PROJECT_NAME}_common_export_libs})
