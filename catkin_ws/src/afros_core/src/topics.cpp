#include "afros_core/topics.hpp"

namespace afros_core{
    namespace topics{
        std::string get_connection_data_topic(uint32_t id, const std::string& name){
            return CONNECTION_PREFIX + std::to_string(id) + "/" + name + CONNECTION_DATA_SUFFIX;
        }

        std::string get_connection_bond_topic(uint32_t id, const std::string& name){
            return CONNECTION_PREFIX + std::to_string(id) + "/" + name + CONNECTION_BOND_SUFFIX;
        }

        std::string get_connection_subscribe_topic(uint32_t id){
            return CONNECTION_PREFIX + std::to_string(id) + CONNECTION_SUBSCRIBE_SUFFIX;
        }
    }
}
