#ifndef AFROS_CORE_TOPICS_H
#define AFROS_CORE_TOPICS_H

namespace afros_core{
    /*Messages*/
    //State Machines
    constexpr const char* STATE_MACHINE_BROADCAST = "afros/state_machine/broadcast";
    constexpr const char* STATE_MACHINE_ORDER = "afros/state_machine/order";
    constexpr const char* STATE_MACHINE_ASSIGN = "afros/state_machine/assign";
    constexpr const char* STATE_MACHINE_OFFLINE = "afros/state_machine/offline";

    /*Services*/
    //State Machines
    constexpr const char* STATE_MACHINE_GET_ID_BY_NAME = "afros/state_machine/get_id_by_name";
    constexpr const char* STATE_MACHINE_GET_NAME_BY_ID = "afros/state_machine/get_name_by_id";
    constexpr const char* STATE_MACHINE_STATUS = "afros/state_machine/status";

}

#endif //AFROS_CORE_TOPICS_H
