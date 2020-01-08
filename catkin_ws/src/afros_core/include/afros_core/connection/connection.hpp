#ifndef AFROS_CONNECTION_HPP
#define AFROS_CONNECTION_HPP

#include "afros_core/raw_data.h"
#include "afros_core/error_val.hpp"
#include "afros_core/nullable.hpp"
#include "bondcpp/bond.h"

#include <ros/ros.h>
#include <boost/function.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <boost/thread/shared_mutex.hpp>

namespace afros_core{
    struct subscription{
        std::string name;
        boost::function<afros_core::raw_data(void)> data_function;
        double max_frequency;
    };

    struct rated_bond{
        double frequency;
        bond::Bond* bond;

        rated_bond(const std::string& topic, const std::string& bond_id, double frequency);

        rated_bond(const rated_bond& other) = delete;
        rated_bond(rated_bond&& other) noexcept;
        ~rated_bond();
        rated_bond& operator=(const rated_bond& other) = delete;
        rated_bond& operator=(rated_bond&& other) noexcept;
    };

    struct internal_subscription{
        subscription& sub;
        ros::Publisher publisher;
        ros::SteadyTimer next_update;
        double current_rate;
        boost::mutex rated_bonds_mutex;
        std::vector<rated_bond> rated_bonds;

        explicit internal_subscription(subscription& sub);
    };

    struct publisher{
        std::string name;
        boost::function<void(const afros_core::raw_data&)> publish_function;
    };

    enum connection_error{
        ALREADY_STARTED,
    };

    class connection{
        bool started;
        uint32_t id;

        std::string node_name;

        std::vector<subscription> subscriptions;
        std::vector<publisher> publishers;

        std::string generate_unique_id();

    protected:
        error_val<nullptr_t, connection_error> add_subscription(const subscription& sub);
        error_val<nullptr_t, connection_error> add_publisher(const publisher& pub);

        error_val<nullptr_t, connection_error> set_name(const std::string& name);

    public:
        connection();
        connection(size_t subscriptions_size, size_t publishers_size);

        virtual void main_loop() = 0;

        /**
         * Function to run inside your connection node's main function
         * @param node from main function
         * @param argv from main function
         * @param connection connection representing this node
         * @param frequency rate for this node, should be faster than all subscriptions if not async
         * @param num_async_threads 0 if not async, or number of threads to run
         */
        void main_function(ros::NodeHandle& node, double frequency, uint32_t num_async_threads);
    };
}

#endif //AFROS_CONNECTION_HPP