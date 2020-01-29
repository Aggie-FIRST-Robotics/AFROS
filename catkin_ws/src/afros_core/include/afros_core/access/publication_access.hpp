#ifndef AFROS_PUBLICATION_ACCESS_HPP
#define AFROS_PUBLICATION_ACCESS_HPP

#include <ros/ros.h>
#include "afros_core/raw_data.h"

namespace afros_core{
    struct publication_data{
        std::string connection_node_name;
        std::string connection_publication_name;

        std::string get_topic() const;
    };

    class publication_access_intern{
        ros::Publisher publisher;

    protected:
        publication_access_intern(ros::NodeHandle& node, const publication_data& publication_data, uint32_t queue_size);

        void send_data(const raw_data& message_data);
    };

    template<typename T>
    class publication_access : public publication_access_intern{
        virtual raw_data pub_convert(const T& data) = 0;

    public:
        publication_access(ros::NodeHandle& node, const publication_data& publication_data, uint32_t queue_size);

        void publish(const T& data);
    };

    template<typename T>
    publication_access<T>::publication_access(ros::NodeHandle& node, const publication_data& publication_data, uint32_t queue_size) : publication_access_intern(node, publication_data, queue_size){}

    template<typename T>
    void publication_access<T>::publish(const T& data){
        publication_access_intern::send_data(pub_convert(data));
    }
}

#endif //AFROS_PUBLICATION_ACCESS_HPP
