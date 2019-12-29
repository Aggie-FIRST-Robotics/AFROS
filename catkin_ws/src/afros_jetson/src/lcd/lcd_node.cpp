#include "afros_core/common_lib.hpp"
#include "ros/ros.h"

#include "afros_jetson/lcd_set.h"

#include "afros_jetson/node_names.hpp"
#include "afros_jetson/topics.hpp"
#include "afros_jetson/lcd_node/i2c_lcd_device.hpp"
#include "afros_jetson/lcd_node/lcd_message_utils.hpp"

void lcd_set_callback(const afros_jetson::lcd_set& message);

static afros_jetson::i2c_lcd_device* lcd_device_ptr = nullptr;

int main(int argc, char** argv){
    using namespace afros_jetson;
    ros::NodeHandle node = afros_core::init_ros(argc, argv, LCD_NODE_NAME);
    ros::Rate rate{10};
    i2c_lcd_device lcd_device{0, 0x27};
    lcd_device_ptr = &lcd_device;

    lcd_device.display_string(" Welcome to AFROS!! ", 2);
    ROS_INFO("LCD Initialized");

    auto lcd_set_sub = node.subscribe(topics::LCD_SET, 1, lcd_set_callback);
    ROS_INFO("Subscribing to %s", topics::LCD_SET);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    afros_core::end_ros(LCD_NODE_NAME);
}

void lcd_set_callback(const afros_jetson::lcd_set& message){
    ROS_INFO("LCD set");
    lcd_device_ptr->backlight(message.backlight_on);
    auto lines = afros_jetson::strings_from_lcd_set(message);
//    lcd_device_ptr->clear();
    for(uint8_t x = 0; x < 4; ++x){
        lcd_device_ptr->display_string(lines.at(x), x + 1);
    }
}
