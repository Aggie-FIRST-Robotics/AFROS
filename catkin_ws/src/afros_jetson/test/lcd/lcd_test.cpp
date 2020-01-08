#include "afros_core/common_lib.hpp"
#include "afros_jetson/topics.hpp"
#include "afros_jetson/lcd_set.h"
#include "afros_jetson/lcd/lcd_message_utils.hpp"

constexpr const char* NODE_ID = "lcd_test_node";

int main(int argc, char** argv){
    auto node = afros_core::init_ros(argc, argv, NODE_ID);
    ros::Rate rate{1};
    auto pub = node.advertise<afros_jetson::lcd_set>(afros_jetson::topics::LCD_SET, 1);
    ROS_INFO("Publishing to %s", afros_jetson::topics::LCD_SET);

    while(ros::ok()){
        uint32_t sec{};
        uint32_t nsec{};

        ros::ros_walltime(sec, nsec);

        ROS_INFO("Walltime, sec: %ui, nsec: %ui", sec, nsec);
        ROS_INFO("Subs: %ui", pub.getNumSubscribers());

        afros_jetson::lcd_set message{};
        afros_jetson::lcd_set_from_strings(message, "sec:", std::to_string(sec), "nsec:", std::to_string(nsec), false);
        message.backlight_on = true;
        pub.publish(message);

        ros::spinOnce();
        rate.sleep();
    }

    afros_core::end_ros(NODE_ID);
}
