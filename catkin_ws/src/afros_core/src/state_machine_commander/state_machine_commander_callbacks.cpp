#include "afros_core/state_machine_commander/state_machine_commander_callbacks.hpp"

#include "afros_core/state_machine_order.h"
#include "afros_core/state_machine_assign.h"
#include "afros_core/state_machine_offline.h"

namespace afros_core{
    namespace state_machine_commander{
        publishers* register_all(ros::NodeHandle& node, registry<status>* registry){
            if(pubs != nullptr){
                return nullptr;
            }

            registry_ptr = registry;

            pubs = new publishers{};
            pubs->order = node.advertise<state_machine_order>(topics::STATE_MACHINE_ORDER, 50);
            pubs->assign = node.advertise<state_machine_assign>(topics::STATE_MACHINE_ASSIGN, 50);
            pubs->offline = node.advertise<state_machine_offline>(topics::STATE_MACHINE_OFFLINE, 50);

            node.subscribe(topics::STATE_MACHINE_BROADCAST, 50, broadcast_callback);

            node.advertiseService(topics::STATE_MACHINE_GET_ID_BY_NAME, get_id_by_name_callback);
            node.advertiseService(topics::STATE_MACHINE_GET_NAME_BY_ID, get_name_by_id_callback);
            node.advertiseService(topics::STATE_MACHINE_STATUS, status_callback);

            return pubs;
        }

        void broadcast_callback(const state_machine_broadcast& broadcast){
            ROS_INFO("Broadcast recieved form %s", broadcast.name.c_str());
            auto found = registry_ptr->find(broadcast.name);
            if(!found.is_error()){
                ROS_WARN("Double register of %s", broadcast.name.c_str());
                return;
            }

            registry_error error{};
            AFROS_CORE_ERROR_CHECK(index, registry_ptr->add(status{broadcast.name, broadcast.bond_id}), error){
                ROS_ERROR("Error assigning id for state machine %s: %i", broadcast.name.c_str(), error);
                return;
            }

            state_machine_assign out{};
            out.name = broadcast.name;
            out.bond_id = broadcast.bond_id;
            out.state_machine_id = *index;
            pubs->assign.publish(out);
            ROS_INFO("Assigned %s id %ui", broadcast.name.c_str(), *index);
        }

        bool get_id_by_name_callback(state_machine_get_id_by_name::Request& request,
                                     state_machine_get_id_by_name::Response& response){
            registry_error error{};
            AFROS_CORE_ERROR_CHECK(found_val, registry_ptr->find<std::string>(request.name), error){
                ROS_ERROR("State machine %s not found", request.name.c_str());
                return false;
            }
            AFROS_CORE_ERROR_CHECK(entry, registry_ptr->get(*found_val), error){
                ROS_ERROR("Cannot find entry that was found earlier! %s", request.name.c_str());
                return false;
            }
            response.is_enabled = entry->get().is_enabled;
            response.state_machine_id = *found_val;
            return true;
        }

        bool get_name_by_id_callback(state_machine_get_name_by_id::Request& request,
                                     state_machine_get_name_by_id::Response& response){
            registry_error error;
            AFROS_CORE_ERROR_CHECK(got, registry_ptr->get(request.state_machine_id), error){
                if(error == OUT_OF_BOUNDS){
                    ROS_ERROR("Request failed for index %ui, out of bounds", request.state_machine_id);
                } else if(error == EMPTY_ENTRY){
                    ROS_ERROR("Request failed for index %ui, empty entry", request.state_machine_id);
                } else{
                    ROS_ERROR("Request failed for index %ui", request.state_machine_id);
                }
                return false;
            }
            response.name = got->get().name;
            response.is_enabled = got->get().is_enabled;
            return true;
        }

        bool status_callback(state_machine_status::Request& request, state_machine_status::Response& response){
            registry_error error{};
            AFROS_CORE_ERROR_CHECK(state_machine, registry_ptr->get(request.state_machine_id), error){
                ROS_ERROR("Cannot find state machine %ui", request.state_machine_id);
                return false;
            }
            response.is_enabled = state_machine->get().is_enabled;
            return true;
        }

        bool status::operator==(const status& other){
            return name == other.name;
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

        bool status::operator==(const std::string& other){
            return name == other;
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
