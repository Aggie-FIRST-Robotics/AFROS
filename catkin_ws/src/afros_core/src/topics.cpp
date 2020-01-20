#include "afros_core/topics.hpp"

namespace afros_core{
    namespace topics{
        std::string get_connection_data_topic(const std::string& node_name, const std::string& name){
            return CONNECTION_PREFIX + node_name + "/" + name + CONNECTION_DATA_SUFFIX;
        }

        std::string get_connection_bond_topic(const std::string& node_name, const std::string& name){
            return CONNECTION_PREFIX + node_name + "/" + name + CONNECTION_BOND_SUFFIX;
        }

        std::string get_connection_subscribe_topic(const std::string& name){
            return CONNECTION_PREFIX + name + CONNECTION_SUBSCRIBE_SUFFIX;
        }
    }
}
