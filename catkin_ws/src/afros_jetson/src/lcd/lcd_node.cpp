#include "afros_core/common_lib.hpp"
#include "ros/ros.h"

#include "afros_jetson/node_names.hpp"
#include "afros_jetson/lcd/i2c_lcd_device.hpp"
#include "afros_jetson/lcd_connection/lcd_connection.hpp"

static afros_jetson::i2c_lcd_device* lcd_device_ptr = nullptr;

int main(int argc, char** argv){
    using namespace afros_jetson;
    ros::NodeHandle node = afros_core::init_ros(argc, argv, LCD_NODE_NAME);
    lcd_connection connection{0, 0x27, node, 0.5, LCD_NODE_NAME};

    boost::array<std::string, 4> initial_text{};
    initial_text.at(1) = " Welcome to AFROS!! ";
    connection.set_data(initial_text);
    ROS_INFO("LCD Initialized");

    connection.main_function(node, 10);

    afros_core::end_ros(LCD_NODE_NAME);
}
