#include "afros_core/common_lib.hpp"
#include "ros/ros.h"

#include "afros_jetson/node_names.hpp"

int main(int argc, char** argv){
    using namespace afros_jetson;
    afros_core::init_ros(argc, argv, LCD_NODE_NAME);
    ros::Rate rate{1};
    ros::AsyncSpinner spinner{3};

    spinner.start();

    while(ros::ok()){
        rate.sleep();
    }
    spinner.stop();
    afros_core::end_ros(LCD_NODE_NAME);
}
