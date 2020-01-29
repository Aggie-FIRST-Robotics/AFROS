#include "afros_core/access/publication_access.hpp"

#include "afros_core/topics.hpp"

namespace afros_core{
    void publication_access_intern::send_data(const raw_data& message_data){
        publisher.publish(message_data);
    }

    publication_access_intern::publication_access_intern(ros::NodeHandle& node, const publication_data& publication_data, uint32_t queue_size)
            : publisher(node.advertise<raw_data>(publication_data.get_topic(), queue_size)){}

    std::string publication_data::get_topic() const{
        return topics::get_connection_data_topic(connection_node_name, connection_publication_name);
    }
}
