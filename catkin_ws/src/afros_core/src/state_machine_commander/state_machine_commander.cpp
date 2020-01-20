#include "afros_core/state_machine_offline.h"
#include "afros_core/common_lib.hpp"

#include "afros_core/state_machine_commander/state_machine_commander_callbacks.hpp"

#include <bondcpp/bond.h>

int main(int argc, char* argv[]){
    using namespace afros_core;
    auto node = init_ros(argc, argv, STATE_MACHINE_COMMANDER_NODE_NAME);

    std::unordered_map<std::string, state_machine_commander::status> state_machine_map{};
    auto* pub_sub = state_machine_commander::register_all(node, &state_machine_map);

    ros::spin();

    state_machine_commander::end();
    afros_core::end_ros(STATE_MACHINE_COMMANDER_NODE_NAME);
}
