#include "afros_core/state_machine_commander/state_machine_commander_callbacks.hpp"

#include "afros_core/state_machine_order.h"
#include "afros_core/state_machine_offline.h"

namespace afros_core{
    namespace state_machine_commander{
        pub_subs* register_all(ros::NodeHandle& node, std::unordered_map<std::string, status>* state_machine_map){
            if(pub_sub != nullptr){
                return nullptr;
            }

            if(state_machine_map == nullptr){
                ROS_ERROR("Cannot have nullptr for connection_map!");
                throw std::runtime_error{"Cannot have nullptr for connection_map!"};
            }
            state_machine_map_ptr = state_machine_map;

            pub_sub = new pub_subs{};
            pub_sub->order_publisher = node.advertise<state_machine_order>(topics::STATE_MACHINE_ORDER, 50);
            pub_sub->offline_publisher = node.advertise<state_machine_offline>(topics::STATE_MACHINE_OFFLINE, 50);
            pub_sub->broadcast_subscriber = node.subscribe(topics::STATE_MACHINE_BROADCAST, 50, broadcast_callback);
            pub_sub->status_service = node.advertiseService(topics::STATE_MACHINE_STATUS_SERVICE, status_callback);
            pub_sub->bond_check_timer = node.createSteadyTimer(ros::WallDuration{1}, bond_check_callback);

            return pub_sub;
        }

        void broadcast_callback(const state_machine_broadcast& broadcast){
            if(state_machine_map_ptr->find(broadcast.name) != state_machine_map_ptr->end()){
                ROS_ERROR("%s connection already registered and running!", broadcast.name.c_str());
                return;
            }

            auto result = state_machine_map_ptr->emplace(broadcast.name, status{broadcast.name, broadcast.bond_id});
            if(!result.second){
                ROS_ERROR("Could not insert connection %s with bond id %s", broadcast.name.c_str(), broadcast.bond_id.c_str());
                return;
            }
            result.first->second.bond->start();
        }

        bool status_callback(state_machine_status::Request& request, state_machine_status::Response& response){
            auto found = state_machine_map_ptr->find(request.state_machine_name);
            if(found == state_machine_map_ptr->end()){
                response.is_registered = false;
                return true;
            }
            response.is_registered = true;
            response.is_enabled = found->second.is_enabled;
            return true;
        }

        void end(){
            delete (pub_sub);
        }

        void bond_check_callback(const ros::SteadyTimerEvent& event){
            std::vector<std::string> removes;
            for(auto& entry : *state_machine_map_ptr){
                if(entry.second.bond->isBroken()){
                    state_machine_offline message{};
                    message.state_machine_id = entry.first;
                    pub_sub->offline_publisher.publish(message);
                    removes.push_back(entry.first);
                }
            }

            for(auto& remove : removes){
                state_machine_map_ptr->erase(remove);
            }
        }

        status::status(std::string name, const std::string& bond_id) :
                name(std::move(name)), is_enabled(false),
                bond(new bond::Bond{topics::STATE_MACHINE_BOND + bond_id, bond_id}){
            bond->start();
        }

        status::status(std::string name, const std::string& bond_id, bool is_enabled) :
                name(std::move(name)), is_enabled(is_enabled),
                bond(new bond::Bond{topics::STATE_MACHINE_BOND + bond_id, bond_id}){
            bond->start();
        }

        status::status(status&& other) noexcept :
                name(std::move(other.name)), is_enabled(other.is_enabled), bond(other.bond){
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
            name = std::move(other.name);
            is_enabled = other.is_enabled;
            bond = other.bond;
            other.bond = nullptr;
        }
    }
}
