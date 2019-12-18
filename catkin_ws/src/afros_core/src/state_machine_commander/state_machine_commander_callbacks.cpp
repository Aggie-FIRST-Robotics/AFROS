#include "afros_core/state_machine_commander/state_machine_commander_callbacks.hpp"

#include <utility>

#include "afros_core/state_machine_order.h"
#include "afros_core/state_machine_assign.h"
#include "afros_core/state_machine_offline.h"

afros_core::state_machine_commander::publishers*
afros_core::state_machine_commander::register_all(ros::NodeHandle& node, afros_core::registry<status>* registry){
    if(pubs != nullptr){
        return nullptr;
    }

    registry_ptr = registry;

    pubs = new publishers{};
    pubs->order = node.advertise<state_machine_order>(STATE_MACHINE_ORDER, 50);
    pubs->assign = node.advertise<state_machine_assign>(STATE_MACHINE_ASSIGN, 50);
    pubs->offline = node.advertise<state_machine_offline>(STATE_MACHINE_OFFLINE, 50);

    node.subscribe(STATE_MACHINE_BROADCAST, 50, broadcast_callback);

    node.advertiseService(STATE_MACHINE_GET_ID_BY_NAME, get_id_by_name_callback);
    node.advertiseService(STATE_MACHINE_GET_NAME_BY_ID, get_name_by_id_callback);
    node.advertiseService(STATE_MACHINE_STATUS, status_callback);

    return pubs;
}

void afros_core::state_machine_commander::broadcast_callback(const afros_core::state_machine_broadcast& broadcast){
    ROS_INFO("Broadcast recieved form %s", broadcast.name.c_str());
    auto found = registry_ptr->find(broadcast.name);
    if(!found.is_error()){
        ROS_WARN("Double register of %s", broadcast.name.c_str());
        return;
    }

    registry_error error{};
    AFROS_CORE_ERROR_CHECK(index, registry_ptr->add(broadcast.name), error){
        ROS_ERROR("Error assigning id for state machine %s: %i", broadcast.name.c_str(), error);
        return;
    }

    state_machine_assign out{};
    out.name = broadcast.name;
    out.random = broadcast.random;
    out.state_machine_id = *index;
    pubs->assign.publish(out);
    ROS_INFO("Assigned %s id %ui", broadcast.name.c_str(), *index);
}

bool afros_core::state_machine_commander::get_id_by_name_callback(
        afros_core::state_machine_get_id_by_name::Request& request,
        afros_core::state_machine_get_id_by_name::Response& response){
    registry_error error{};
    AFROS_CORE_ERROR_CHECK(found_val, registry_ptr->find(request.name), error){
        ROS_ERROR("State machine %s not found", request.name.c_str());
        return false;
    }
    AFROS_CORE_ERROR_CHECK(entry, registry_ptr->get(*found_val), error){
        ROS_ERROR("Cannot find entry that was found earlier! %s", request.name.c_str());
        return false;
    }
    response.is_enabled = entry->is_enabled;
    response.state_machine_id = *found_val;
    return true;
}

bool afros_core::state_machine_commander::get_name_by_id_callback(
        afros_core::state_machine_get_name_by_id::Request& request,
        afros_core::state_machine_get_name_by_id::Response& response){
    registry_error error;
    AFROS_CORE_ERROR_CHECK(got, registry_ptr->get(request.state_machine_id), error){
        if(error == OUT_OF_BOUNDS){
            ROS_ERROR("Request failed for index %ui, out of bounds", request.state_machine_id);
        }
        else if(error == EMPTY_ENTRY){
            ROS_ERROR("Request failed for index %ui, empty entry", request.state_machine_id);
        }
        else{
            ROS_ERROR("Request failed for index %ui", request.state_machine_id);
        }
        return false;
    }
    response.name = got->name;
    response.is_enabled = got->is_enabled;
    return true;
}

bool afros_core::state_machine_commander::status_callback(afros_core::state_machine_status::Request& request,
                                                          afros_core::state_machine_status::Response& response){
    registry_error error{};
    AFROS_CORE_ERROR_CHECK(state_machine, registry_ptr->get(request.state_machine_id), error){
        ROS_ERROR("Cannot find state machine %ui", request.state_machine_id);
        return false;
    }
    response.is_enabled = state_machine->is_enabled;
    return true;
}

bool afros_core::state_machine_commander::status::operator==(const afros_core::state_machine_commander::status& other){
    return name == other.name;
}

afros_core::state_machine_commander::status::status(std::string name) : name(std::move(name)), is_enabled(false){}

afros_core::state_machine_commander::status::status(std::string name, bool is_enabled) : name(std::move(name)),
                                                                                         is_enabled(is_enabled){}
