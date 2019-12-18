#include "afros_core/common_lib.h"

ros::NodeHandle afros_core::init_ros(int argc, char** argv, const char* node_id){
    ros::init(argc, argv, node_id);
    ros::NodeHandle node_handle{};
    ROS_INFO("%s initialized", node_id);
    return node_handle;
}

void afros_core::error(const std::exception& e, const char* node_id){
    ROS_ERROR("Error in %s: %s", node_id, e.what());
}

void afros_core::error(const boost::system::error_code& e, const char* node_id){
    ROS_ERROR("Error in %s: %s", node_id, e.message().c_str());
}

void afros_core::end_ros(const char* node_id){
    ROS_INFO("%s closed", node_id);
}
