#include "afros_jetson/lcd_connection/lcd_connection_access.hpp"
#include "afros_core/common_lib.hpp"

constexpr const char* NODE_ID = "lcd_test_node";

int main(int argc, char** argv){
    auto node = afros_core::init_ros(argc, argv, NODE_ID);

    afros_jetson::lcd_publication_access pub_access{node, NODE_ID};
    boost::function<void(const afros_jetson::lcd_data&)> callback{[](const afros_jetson::lcd_data& data) -> void{
        std::cout << "Lines:" << std::endl;
        for(auto& x : data){
            for(auto& y : x){
                std::cout << y;
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }};
//    afros_jetson::lcd_subscription_access sub_access{node, NODE_ID, 1, &callback};

    boost::function<void(const ros::SteadyTimerEvent&)> timer_callback{[&pub_access](const ros::SteadyTimerEvent& event) -> void{
        afros_jetson::lcd_data out{};
        uint32_t sec;
        uint32_t nsec;
        ros::ros_steadytime(sec, nsec);
        std::string sec_str{std::to_string(sec)};
        std::string nsec_str{std::to_string(nsec)};

        for(size_t x = 0; x < std::min(sec_str.size(), afros_jetson::lcd_data::value_type::size()); ++x){
            out.at(0).at(x) = sec_str.at(x);
        }
        for(size_t x = 0; x < std::min(nsec_str.size(), afros_jetson::lcd_data::value_type::size()); ++x){
            out.at(1).at(x) = nsec_str.at(x);
        }
        ROS_INFO("Setting time to %u : %u", sec, nsec);
        pub_access.publish(out);
    }};
    auto timer = node.createSteadyTimer(ros::WallDuration(1), timer_callback);

    ros::spin();
    afros_core::end_ros(NODE_ID);
}
