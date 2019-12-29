#include "afros_core/connection/connection.hpp"

#include "afros_core/common_lib.hpp"
#include "afros_core/topics.hpp"
#include "afros_core/connection_broadcast.h"

namespace afros_core {
    connection::connection() : subscriptions(), publishers() {}

    connection::connection(size_t subscriptions_size, size_t publishers_size) :
            subscriptions(subscriptions_size), publishers(publishers_size) {}

    void connection::main_function(int &argc, char **&argv, connection &connection, double frequency) {
        const char *name = connection.get_connection_name();
        auto node = init_ros(argc, argv, name);
        ros::Rate rate{frequency};

        auto broadcast_pub = node.advertise<connection_broadcast>(topics::CONNECTION_BROADCAST, 50);
        connection_broadcast broadcast{};
        broadcast.name = name;
        broadcast_pub.publish(broadcast);

        while (ros::ok()) {
            connection.main_loop();
            ros::spinOnce();
            rate.sleep();
        }

        end_ros(connection.get_connection_name());
    }
}
