#ifndef AFROS_CORE_STATE_MACHINE_COMMANDER_CALLBACKS_H
#define AFROS_CORE_STATE_MACHINE_COMMANDER_CALLBACKS_H

#include "afros_core/topics.h"
#include "afros_core/registry.h"
#include "ros/ros.h"

#include "afros_core/state_machine_broadcast.h"
#include "afros_core/state_machine_get_id_by_name.h"
#include "afros_core/state_machine_get_name_by_id.h"
#include "afros_core/state_machine_status.h"

namespace afros_core{
    namespace state_machine_commander{
        struct publishers{
            ros::Publisher order;
            ros::Publisher assign;
            ros::Publisher offline;
        };

        struct status{
            std::string name;
            bool is_enabled{};

            status() = default;
            status(std::string name); // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
            status(std::string name, bool is_enabled);

            bool operator==(const status& other);
        };

        static registry<status>* registry_ptr = nullptr;
        static publishers* pubs{};

        publishers* register_all(ros::NodeHandle& node, afros_core::registry<status>* registry);
        void broadcast_callback(const state_machine_broadcast& broadcast);
        bool get_id_by_name_callback(state_machine_get_id_by_name::Request& request,
                                     state_machine_get_id_by_name::Response& response);
        bool get_name_by_id_callback(state_machine_get_name_by_id::Request& request,
                                     state_machine_get_name_by_id::Response& response);
        bool status_callback(state_machine_status::Request& request, state_machine_status::Response& response);
    }
}
#endif //AFROS_CORE_STATE_MACHINE_COMMANDER_CALLBACKS_H
