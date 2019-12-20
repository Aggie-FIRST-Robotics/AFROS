#include "afros_core/common_lib.hpp"
#include "ros/ros.h"

#include "afros_jetson/lcd_set.h"

#include "afros_jetson/node_names.hpp"
#include "afros_jetson/topics.hpp"
#include "afros_jetson/lcd_node/i2c_lcd_device.hpp"

void lcd_set_callback(const afros_jetson::lcd_set& message);
static afros_jetson::i2c_lcd_device* lcd_device_ptr = nullptr;

int main(int argc, char** argv){
    using namespace afros_jetson;
    ros::NodeHandle node = afros_core::init_ros(argc, argv, LCD_NODE_NAME);
    ros::Rate rate{10};
    i2c_lcd_device lcd_device{0, 0x27};
    lcd_device_ptr = &lcd_device;

    lcd_device.display_string(" Welcome to AFROS!! ", 2);

    node.subscribe(topics::LCD_SET, 1, lcd_set_callback);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    afros_core::end_ros(LCD_NODE_NAME);
}

void lcd_set_callback(const afros_jetson::lcd_set& message){
    char line1_char[20];
    for(size_t x = 0; x < afros_jetson::lcd_set::_line1_type::size(); x++){
        line1_char[x] = message.line1.at(x);
    }
    std::string line1{line1_char};

    char line2_char[20];
    for(size_t x = 0; x < afros_jetson::lcd_set::_line2_type::size(); x++){
        line2_char[x] = message.line1.at(x);
    }
    std::string line2{line1_char};

    char line3_char[20];
    for(size_t x = 0; x < afros_jetson::lcd_set::_line3_type::size(); x++){
        line3_char[x] = message.line3.at(x);
    }
    std::string line3{line3_char};

    char line4_char[20];
    for(size_t x = 0; x < afros_jetson::lcd_set::_line4_type::size(); x++){
        line4_char[x] = message.line4.at(x);
    }
    std::string line4{line4_char};

    lcd_device_ptr->clear();
    lcd_device_ptr->display_string(line1, 1, 0);
    lcd_device_ptr->display_string(line2, 2, 0);
    lcd_device_ptr->display_string(line3, 3, 0);
    lcd_device_ptr->display_string(line4, 4, 0);
}
