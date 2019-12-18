#ifndef AFROS_MAIN_COMMON_LIB_H
#define AFROS_MAIN_COMMON_LIB_H

#include "ros/ros.h"

#include "afros_core/error_val.hpp"

#include <boost/system/error_code.hpp>
#include <boost/asio.hpp>
#include <functional>

namespace std{
    template<>
    struct hash<boost::asio::ip::address>{
        size_t operator()(const boost::asio::ip::address& ip) const{
            return std::hash<std::string>{}(ip.to_string());
        }
    };
}

namespace afros_core{
    ros::NodeHandle init_ros(int argc, char** argv, const char* node_id);
    void end_ros(const char* node_id);

    void error(const std::exception& e, const char* node_id);

    void error(const boost::system::error_code& e, const char* node_id);

    constexpr const char* STATE_MACHINE_COMMANDER_NODE_NAME = "afros_core_state_machine_commander_node";
}

#endif //AFROS_MAIN_COMMON_LIB_H
