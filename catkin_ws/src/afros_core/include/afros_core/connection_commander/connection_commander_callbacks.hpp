#ifndef AFROS_CONNECTION_COMMANDER_CALLBACKS_HPP
#define AFROS_CONNECTION_COMMANDER_CALLBACKS_HPP

#include <bondcpp/bond.h>
#include <afros_core/connection_broadcast.h>
#include <afros_core/connection_get_status.h>
#include <unordered_map>

namespace afros_core{
    namespace connection_commander{
        struct pub_subs{
            ros::Publisher offline_publisher;
            ros::Subscriber broadcast_subscriber;
            ros::ServiceServer status_service;
            ros::SteadyTimer bond_check_timer;
        };

        struct status{
            std::string name;
            bond::Bond* bond;

            status(const status& other) = delete;
            status(status&& other) noexcept;
            ~status();
            status& operator=(const status& other) = delete;
            status& operator=(status&& other) noexcept;

            status(std::string name, const std::string& bond_id);
        };

        static std::unordered_map<std::string, status>* connection_map_ptr = nullptr;
        static pub_subs* pub_sub = nullptr;

        pub_subs* register_all(ros::NodeHandle& node, std::unordered_map<std::string, status>* connection_map);
        void end();

        void broadcast_callback(const connection_broadcast& broadcast);
        bool status_callback(connection_get_status::Request& request, connection_get_status::Response& response);
        void bond_check_callback(const ros::SteadyTimerEvent& event);
    }
}

#endif //AFROS_CONNECTION_COMMANDER_CALLBACKS_HPP
