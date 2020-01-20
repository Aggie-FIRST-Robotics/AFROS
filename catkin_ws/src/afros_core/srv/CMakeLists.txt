cmake_minimum_required(VERSION 2.8.3)

set(
        ${PROJECT_NAME}_service_files

        connection_get_status.srv
        state_machine_get_status.srv

        PARENT_SCOPE
)
