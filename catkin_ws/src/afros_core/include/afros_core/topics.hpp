#ifndef AFROS_CORE_TOPICS_H
#define AFROS_CORE_TOPICS_H

#include "string"

namespace afros_core{
    namespace topics{
        /*Messages*/
        //Connection
        constexpr const char* CONNECTION_BROADCAST = "/afros/connection/broadcast";
        constexpr const char* CONNECTION_OFFLINE = "/afros/connection/offline";
        constexpr const char* CONNECTION_STATUS_SERVICE = "/afros/connection/status";
        constexpr const char* CONNECTION_BOND_PREFIX = "/afros/connection/bond/";
        constexpr const char* CONNECTION_PREFIX = "/afros/connection/";
        constexpr const char* CONNECTION_DATA_SUFFIX = "/data";
        constexpr const char* CONNECTION_SUBSCRIBE_SUFFIX = "/subscribe";
        constexpr const char* CONNECTION_BOND_SUFFIX = "/bond";

        std::string get_connection_data_topic(const std::string& node_name, const std::string& name);
        std::string get_connection_bond_topic(const std::string& node_name, const std::string& name);
        std::string get_connection_subscribe_topic(const std::string& name);

        //State Machines
        constexpr const char* STATE_MACHINE_BROADCAST = "/afros/state_machine/broadcast";
        constexpr const char* STATE_MACHINE_ORDER = "/afros/state_machine/order";
        constexpr const char* STATE_MACHINE_ASSIGN = "/afros/state_machine/assign";
        constexpr const char* STATE_MACHINE_OFFLINE = "/afros/state_machine/offline";
        constexpr const char* STATE_MACHINE_BOND = "/afros/state_machine/bond/";

        /*Services*/
        //State Machines
        constexpr const char* STATE_MACHINE_GET_ID_BY_NAME = "/afros/state_machine/get_id_by_name";
        constexpr const char* STATE_MACHINE_GET_NAME_BY_ID = "/afros/state_machine/get_name_by_id";
        constexpr const char* STATE_MACHINE_STATUS_SERVICE = "/afros/state_machine/status";
    }
}

#endif //AFROS_CORE_TOPICS_H
