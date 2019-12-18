cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_service_files

        state_machine_get_current_state.srv
        state_machine_get_id_by_name.srv
        state_machine_get_name_by_id.srv
        state_machine_status.srv

        PARENT_SCOPE
)
