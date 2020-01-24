#ifndef AFROS_CORE_STATE_MACHINE_COMMANDER_CALLBACKS_H
#define AFROS_CORE_STATE_MACHINE_COMMANDER_CALLBACKS_H

#include "afros_core/topics.hpp"
#include "ros/ros.h"

#include "afros_core/state_machine_broadcast.h"
#include "afros_core/state_machine_get_status.h"

#include <bondcpp/bond.h>
#include <unordered_map>

namespace afros_core{
    namespace state_machine_commander{
        struct pub_subs{
            ros::Publisher order_publisher;
            ros::Publisher offline_publisher;
            ros::Subscriber broadcast_subscriber;
            ros::ServiceServer status_service;
            ros::SteadyTimer bond_check_timer;
        };

        struct status{
            std::string name;
            bool is_enabled;
            bond::Bond* bond;

            status(const status& other) = delete;
            status(status&& other) noexcept;
            ~status();
            status& operator=(const status& other) = delete;
            status& operator=(status&& other) noexcept;

            status(std::string name, const std::string& bond_id);
            status(std::string name, const std::string& bond_id, bool is_enabled);
        };

        static std::unordered_map<std::string, status>* state_machine_map_ptr = nullptr;
        static pub_subs* pub_sub = nullptr;

        pub_subs* register_all(ros::NodeHandle& node, std::unordered_map<std::string, status>* state_machine_map);
        void end();

        void broadcast_callback(const state_machine_broadcast& broadcast);
        bool status_callback(state_machine_get_status::Request& request, state_machine_get_status::Response& response);
        void bond_check_callback(const ros::SteadyTimerEvent& event);
    }
}
#endif //AFROS_CORE_STATE_MACHINE_COMMANDER_CALLBACKS_H
