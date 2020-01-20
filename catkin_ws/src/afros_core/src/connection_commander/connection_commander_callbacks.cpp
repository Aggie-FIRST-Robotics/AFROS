#include <afros_core/topics.hpp>
#include "afros_core/connection_commander/connection_commander_callbacks.hpp"
#include "afros_core/connection_offline.h"

namespace afros_core{
    namespace connection_commander{
        status::status(status&& other) noexcept : name(std::move(other.name)), bond(other.bond){
            other.bond = nullptr;
        }

        status::~status(){
            delete (bond);
        }

        status& status::operator=(status&& other) noexcept{
            if(this == &other){
                return *this;
            }
            delete (bond);
            name = other.name;
            bond = other.bond;
            other.bond = nullptr;
            return *this;
        }

        status::status(std::string name, const std::string& bond_id) : name(std::move(name)), bond(new bond::Bond{topics::CONNECTION_BOND_PREFIX + name, bond_id}){
            bond->start();
        }

        pub_subs* register_all(ros::NodeHandle& node, std::unordered_map<std::string, status>* connection_map){
            if(pub_sub != nullptr){
                return pub_sub;
            }

            if(connection_map == nullptr){
                ROS_ERROR("Cannot have nullptr for connection_map!");
                throw std::runtime_error{"Cannot have nullptr for connection_map!"};
            }
            connection_map_ptr = connection_map;

            pub_sub = new pub_subs{};
            pub_sub->offline_publisher = node.advertise<connection_offline>(topics::CONNECTION_OFFLINE, 50);
            pub_sub->broadcast_subscriber = node.subscribe(topics::CONNECTION_BROADCAST, 50, broadcast_callback);
            pub_sub->status_service = node.advertiseService(topics::CONNECTION_STATUS_SERVICE, status_callback);
            pub_sub->bond_check_timer = node.createSteadyTimer(ros::WallDuration{1}, bond_check_callback);

            return pub_sub;
        }

        void end(){
            delete (pub_sub);
        }

        void broadcast_callback(const connection_broadcast& broadcast){
            if(connection_map_ptr->find(broadcast.name) != connection_map_ptr->end()){
                ROS_ERROR("%s connection already registered and running!", broadcast.name.c_str());
                return;
            }

            auto result = connection_map_ptr->emplace(broadcast.name, status{broadcast.name, broadcast.bond_id});
            if(!result.second){
                ROS_ERROR("Could not insert connection %s with bond id %s", broadcast.name.c_str(), broadcast.bond_id.c_str());
                return;
            }
            result.first->second.bond->start();
        }

        bool status_callback(connection_get_status::Request& request, connection_get_status::Response& response){
            auto found = connection_map_ptr->find(request.connection_id);
            if(found == connection_map_ptr->end()){
                response.found = false;
                return true;
            }
            response.found = true;
            response.status.is_registered = true;
            return true;
        }

        void bond_check_callback(const ros::SteadyTimerEvent& event){
            std::vector<std::string> removes;
            for(auto& entry : *connection_map_ptr){
                if(entry.second.bond->isBroken()){
                    connection_offline message{};
                    message.connection_id = entry.first;
                    pub_sub->offline_publisher.publish(message);
                    removes.push_back(entry.first);
                }
            }

            for(auto& remove : removes){
                connection_map_ptr->erase(remove);
            }
        }
    }
}
