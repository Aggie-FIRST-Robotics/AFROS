cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_message_files

        connection_broadcast.msg
        connection_offline.msg
        connection_pub_sub.msg
        connection_status.msg
        connection_subscribe.msg
        identifier.msg
        raw_data.msg
        state_machine_broadcast.msg
        state_machine_current_state.msg
        state_machine_offline.msg
        state_machine_order.msg

        PARENT_SCOPE
)