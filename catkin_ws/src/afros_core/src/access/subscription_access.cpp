#include "afros_core/access/subscription_access.hpp"

#include <utility>
#include "afros_core/topics.hpp"
#include "afros_core/connection_subscribe.h"

namespace afros_core{
    std::string subscription_data::get_subscribe_topic() const{
        return topics::get_connection_subscribe_topic(connection_node_name);
    }

    std::string subscription_data::get_topic() const{
        return topics::get_connection_data_topic(connection_node_name, connection_subscription_name);
    }

    std::string subscription_data::get_bond_topic() const{
        return topics::get_connection_bond_topic(connection_node_name, connection_subscription_name);
    }

    subscription_access_intern::subscription_access_intern(ros::NodeHandle& node, const subscription_data& subscription_data, uint32_t queue_size, double frequency,
                                                           std::function<void(const raw_data&)> data_function) :
            data_function(std::move(data_function)), subscriber(node.subscribe<raw_data>(subscription_data.get_topic(), queue_size, this->data_function)), bond_id(generate_unique_id(node)),
            bond(subscription_data.get_bond_topic(), bond_id){
        auto subscribe_publisher = node.advertise<connection_subscribe>(subscription_data.get_subscribe_topic(), 1, false);
        connection_subscribe message{};
        message.bond_id = bond_id;
        message.frequency = frequency;
        message.subscription = subscription_data.connection_subscription_name;
        bond.start();
        ros::AsyncSpinner spinner{1};
        spinner.start();
        do{
            subscribe_publisher.publish(message);
        } while(!bond.waitUntilFormed(ros::Duration(1)));
        spinner.stop();
        ROS_INFO("Created bond with %s : %s", subscription_data.connection_node_name.c_str(), subscription_data.connection_subscription_name.c_str());
    }

    std::string subscription_access_intern::generate_unique_id(ros::NodeHandle& node){
        uint32_t sec = 0;
        uint32_t nsec = 0;
        ros::ros_steadytime(sec, nsec);
        std::string node_name;
        node.resolveName(node_name);
        return "access_" + node_name + "_" + std::to_string(sec) + "_" + std::to_string(nsec);
    }
}
