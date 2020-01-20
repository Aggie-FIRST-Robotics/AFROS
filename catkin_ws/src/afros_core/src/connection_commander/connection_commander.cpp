#include <afros_core/connection_offline.h>
#include <afros_core/state_machine_commander/state_machine_commander_callbacks.hpp>
#include "afros_core/common_lib.hpp"

#include "afros_core/connection_commander/connection_commander_callbacks.hpp"

int main(int argc, char* argv[]){
    using namespace afros_core;
    auto node = afros_core::init_ros(argc, argv, CONNECTION_COMMANDER_NODE_NAME);

    std::unordered_map<std::string, connection_commander::status> connection_map{};
    auto* pub_sub = connection_commander::register_all(node, &connection_map);

    ros::spin();

    connection_commander::end();
    afros_core::end_ros(CONNECTION_COMMANDER_NODE_NAME);
}
