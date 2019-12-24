cmake_minimum_required(VERSION 2.8.3)

add_subdirectory(lcd)

set(
        ${PROJECT_NAME}_test_src
        test.cpp
)

add_library(${PROJECT_NAME}_test_lib STATIC ${${PROJECT_NAME}_test_src})
target_link_libraries(${PROJECT_NAME}_test_lib ${catkin_LIBRARIES} ${PROJECT_NAME}_common_lib)
