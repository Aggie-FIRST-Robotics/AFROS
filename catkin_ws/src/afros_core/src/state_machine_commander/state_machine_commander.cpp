#include "afros_core/state_machine_offline.h"
#include "afros_core/common_lib.hpp"
#include "afros_core/registry.hpp"

#include "afros_core/state_machine_commander/state_machine_commander_callbacks.hpp"

#include <bondcpp/bond.h>

int main(int argc, char *argv[]) {
    using namespace afros_core;
    auto node = init_ros(argc, argv, STATE_MACHINE_COMMANDER_NODE_NAME);
    ros::Rate rate{1};

    registry<state_machine_commander::status> registry{};
    state_machine_commander::register_all(node, &registry);
    ros::AsyncSpinner spinner{3};

    spinner.start();
    while (ros::ok()) {
        std::vector<std::string> node_names{};
        ros::master::getNodes(node_names);
        registry_error error{};
        for (uint32_t x = 0; x < registry.get_size(); x++) {
            AFROS_CORE_ERROR_CHECK(entry, registry.get(x), error) {
                if (error != EMPTY_ENTRY) {
                    ROS_ERROR("Error going through registry: %i", error);
                }
                continue;
            }
            if (entry->get().bond->isBroken()) {
                state_machine_offline message{};
                message.name = entry->get().name;
                message.state_machine_id = x;
                state_machine_commander::pubs->offline.publish(message);
                registry.remove(x);
            }
        }
        rate.sleep();
    }
    spinner.stop();
    afros_core::end_ros(STATE_MACHINE_COMMANDER_NODE_NAME);
}
