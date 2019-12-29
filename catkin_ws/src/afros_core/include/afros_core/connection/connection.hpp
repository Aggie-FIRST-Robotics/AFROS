#ifndef AFROS_CONNECTION_HPP
#define AFROS_CONNECTION_HPP

#include "afros_core/raw_data.h"

#include <ros/ros.h>
#include <boost/function.hpp>
#include <vector>
#include <string>

namespace afros_core {
    struct subscription {
        std::string name;
        boost::function<afros_core::raw_data(void)> data_function;
        double max_update_rate;
    };

    struct publisher {
        std::string name;
        boost::function<void(afros_core::raw_data)> publish_function;
        double max_publish_rate;
    };

    class connection {
    protected:
        std::vector <subscription> subscriptions;
        std::vector <publisher> publishers;

    public:
        connection();

        connection(size_t subscriptions_size, size_t publishers_size);

        virtual void main_loop() = 0;

        virtual const char *get_connection_name() const = 0;

        static void main_function(int &argc, char **&argv, connection &connection, double frequency);

        static void main_function_async(connection &connection, double main_hz, uint32_t threads);
    };
}

#endif //AFROS_CONNECTION_HPP
