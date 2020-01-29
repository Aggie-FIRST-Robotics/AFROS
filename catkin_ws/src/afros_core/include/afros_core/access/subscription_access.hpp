#ifndef AFROS_SUBSCRIPTION_ACCESS_HPP
#define AFROS_SUBSCRIPTION_ACCESS_HPP

#include <ros/ros.h>
#include <bondcpp/bond.h>
#include "afros_core/raw_data.h"

namespace afros_core{
    struct subscription_data{
        std::string connection_node_name;
        std::string connection_subscription_name;

        std::string get_subscribe_topic() const;
        std::string get_bond_topic() const;
        std::string get_topic() const;
    };

    class subscription_access_intern{
        ros::Subscriber subscriber;
        boost::function<void(const raw_data&)> data_function;
        std::string bond_id;
        bond::Bond bond;

        static std::string generate_unique_id(ros::NodeHandle& node);
    protected:
        subscription_access_intern(ros::NodeHandle& node, const subscription_data& subscription_data, uint32_t queue_size, double frequency,
                                   std::function<void(const raw_data&)> data_function);
    };

    template<typename T>
    class subscription_access : public subscription_access_intern{
        virtual T sub_convert(const raw_data& data) = 0;

    public:
        subscription_access(ros::NodeHandle& node, const subscription_data& subscription_data, uint32_t queue_size, double frequency, boost::function<void(const T&)>* callback);
    };

    template<typename T>
    subscription_access<T>::subscription_access(ros::NodeHandle& node, const subscription_data& subscription_data, uint32_t queue_size, double frequency,
                                                boost::function<void(const T&)>* callback) : subscription_access_intern(node, subscription_data, queue_size, frequency,
                                                                                                                        std::function<void(const raw_data&)>{
                                                                                                                                [this, callback](const raw_data& data) -> void{
                                                                                                                                    (*callback)(sub_convert(data));
                                                                                                                                }}){}
}

#endif //AFROS_SUBSCRIPTION_ACCESS_HPP
