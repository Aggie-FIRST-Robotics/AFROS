#include "afros_core/connection/connection.hpp"

#include "afros_core/common_lib.hpp"
#include "afros_core/topics.hpp"
#include "afros_core/connection_broadcast.h"
#include "afros_core/connection_assign.h"
#include "afros_core/connection_subscribe.h"

namespace afros_core{
    connection::connection() : subscriptions(), publishers(), started(false), id(){}

    connection::connection(size_t subscriptions_size, size_t publishers_size) : subscriptions(subscriptions_size), publishers(publishers_size), started(false), id(){}

    void connection::main_function(ros::NodeHandle& node, double frequency, uint32_t num_async_threads){
        //Lock off functions to create immutability
        started = true;

        ros::Rate rate{frequency};
        //Create async if requested
        ros::AsyncSpinner* spinner = nullptr;
        if(num_async_threads != 0){
            spinner = new ros::AsyncSpinner{num_async_threads};
            spinner->start();
        }

        std::string name{};
        node.resolveName(name);

        //Create bond with id bond_id
        std::string bond_id = generate_unique_id();
        bond::Bond bond(topics::CONNECTION_BOND, bond_id);

        //Broadcast bond id and what subs and pubs this connection has
        {
            auto broadcast_pub = node.advertise<connection_broadcast>(topics::CONNECTION_BROADCAST, 50);
            connection_broadcast broadcast{};

            broadcast.name = name;
            broadcast.bond_id = bond_id;
            for(auto& sub : subscriptions){
                connection_pub_sub pub_sub{};
                pub_sub.name = sub.name;
                pub_sub.parent_name = name;
                pub_sub.min_frequency = sub.max_frequency;
                broadcast.subscriptions.push_back(pub_sub);
            }
            for(auto& pub : publishers){
                connection_pub_sub pub_sub{};
                pub_sub.name = pub.name;
                pub_sub.parent_name = name;
                pub_sub.min_frequency = 0.0;
                broadcast.publications.push_back(pub_sub);
            }

            //Send broadcast until bond obtained
            bond.start();
            do{
                broadcast_pub.publish(broadcast);
            } while(!bond.waitUntilFormed(rate.expectedCycleTime()));
        }

        //Spin to find id assigned
        {
            bool has_id = false;
            boost::function<void(const connection_assign&)> assign_callback{[&bond_id, &name, &has_id, this](const connection_assign& message){
                if(!has_id && message.bond_id == bond_id){
                    id = message.connection_id;
                    has_id = true;
                    ROS_INFO("connection %s assigned id %ui", name.c_str(), message.connection_id);
                }
            }};
            auto assign_sub = node.subscribe<connection_assign>(topics::CONNECTION_ASSIGN, 10, assign_callback);
            while(!has_id){
                ros::spinOnce();
                rate.sleep();
            }
        }

        //Register all subs and pubs with rates
        auto* internal_data = reinterpret_cast<internal_subscription*>(malloc(sizeof(internal_subscription) * subscriptions.size()));
        std::unordered_map<std::string, internal_subscription&> subscribe_ables{subscriptions.size()};
        for(size_t x = 0; x < subscriptions.size(); ++x){
            auto& sub = subscriptions.at(x);
            auto* curr = new(internal_data + x)internal_subscription{sub};
            curr->publisher = node.advertise<raw_data>(topics::get_connection_data_topic(id, sub.name), 50);
            subscribe_ables.emplace(sub.name, *curr);
        }

        boost::function<void(const connection_subscribe&)> subscribe_callback{[&subscribe_ables, &name, &node, this](const connection_subscribe& message){
            auto found = subscribe_ables.find(message.subscription);
            if(found == subscribe_ables.end()){
                ROS_ERROR("Connection %s got subscribed for %s and wasn't found", name.c_str(), message.subscription.c_str());
                return;
            }
            boost::unique_lock<boost::mutex> lock{found->second.rated_bonds_mutex};
            double new_freq = std::min(found->second.sub.max_frequency, message.frequency);
            if(found->second.rated_bonds.empty() || new_freq > found->second.current_rate){
                auto& inter_data = found->second;
                boost::function<void(const ros::SteadyTimerEvent&)> timer_callback{[&inter_data](const ros::SteadyTimerEvent&){
                    inter_data.publisher.publish(inter_data.sub.data_function());
                }};
                found->second.next_update = node.createSteadyTimer(ros::WallDuration{1.0 / new_freq}, timer_callback);
                found->second.next_update.start();
                found->second.current_rate = new_freq;
            }
            found->second.rated_bonds.emplace_back(topics::get_connection_bond_topic(id, found->first), message.bond_id, new_freq);
            found->second.rated_bonds.back().bond->start();
        }};
        auto subscribe_subscription = node.subscribe<connection_subscribe>(topics::get_connection_subscribe_topic(id),50, subscribe_callback);

        //Register all publishers with callbacks
        std::vector<ros::Subscriber> publish_ables{publishers.size()};
        for(auto& pub : publishers){
            publish_ables.push_back(node.subscribe<raw_data>(topics::get_connection_data_topic(id, pub.name), 50, pub.publish_function));
        }

        //Do main loop until node ended or bond broken
        while(ros::ok() && !bond.isBroken()){
            main_loop();

            for(auto& sub : subscribe_ables){
                auto& rated_bonds = sub.second.rated_bonds;
                boost::unique_lock<boost::mutex> lock{sub.second.rated_bonds_mutex};
                if(!rated_bonds.empty()){
                    bool re_calc = false;
                    for(auto iter = rated_bonds.begin(); iter != rated_bonds.end();){
                        if(iter->bond->isBroken()){
                            rated_bonds.erase(iter);
                            re_calc = true;
                        }
                        else{
                            iter++;
                        }
                    }

                    if(re_calc && !rated_bonds.empty()){
                        double new_freq = 0.0;
                        for(auto& r_bond : rated_bonds){
                            if(r_bond.frequency > new_freq){
                                new_freq = r_bond.frequency;
                            }
                        }
                        boost::function<void(const ros::SteadyTimerEvent&)> timer_callback{[&sub](const ros::SteadyTimerEvent&){
                            sub.second.publisher.publish(sub.second.sub.data_function());
                        }};
                        sub.second.next_update = node.createSteadyTimer(ros::WallDuration{1.0 / new_freq}, timer_callback);
                        sub.second.next_update.start();
                        sub.second.current_rate = new_freq;
                    }
                    else{
                        sub.second.next_update.stop();
                    }
                }
            }

            if(spinner == nullptr){
                ros::spinOnce();
            }
            rate.sleep();
        }

        //Close spinner if that option was enabled
        if(spinner != nullptr){
            spinner->stop();
            delete (spinner);
        }
        delete (internal_data);
        end_ros(node_name.c_str());
        started = false;
    }

    error_val<nullptr_t, connection_error> connection::add_subscription(const subscription& sub){
        if(started){
            return error_val<nullptr_t, connection_error>{ALREADY_STARTED, false};
        }
        subscriptions.push_back(sub);
        return error_val<nullptr_t, connection_error>{};
    }

    error_val<nullptr_t, connection_error> connection::add_publisher(const publisher& pub){
        if(started){
            return error_val<nullptr_t, connection_error>{ALREADY_STARTED, false};
        }
        publishers.push_back(pub);
        return error_val<nullptr_t, connection_error>{};
    }

    error_val<nullptr_t, connection_error> connection::set_name(const std::string& name){
        if(started){
            return error_val<nullptr_t, connection_error>{ALREADY_STARTED, false};
        }
        node_name = name;
        return error_val<nullptr_t, connection_error>{};
    }

    std::string connection::generate_unique_id(){
        uint32_t sec = 0;
        uint32_t nsec = 0;
        ros::ros_steadytime(sec, nsec);
        return "connection-" + node_name + "-" + std::to_string(sec) + "." + std::to_string(nsec);
    }

    internal_subscription::internal_subscription(subscription& sub) : sub(sub), publisher(), next_update(), current_rate(sub.max_frequency), rated_bonds_mutex(), rated_bonds(){}

    rated_bond::rated_bond(const std::string& topic, const std::string& bond_id, double frequency) : bond(new bond::Bond{topic, bond_id}), frequency(frequency){}

    rated_bond::rated_bond(rated_bond&& other) noexcept : bond(other.bond), frequency(other.frequency){
        other.bond = nullptr;
    }

    rated_bond::~rated_bond(){
        delete(bond);
    }

    rated_bond& rated_bond::operator=(rated_bond&& other) noexcept {
        if(this == &other){
            return *this;
        }
        delete(bond);
        bond = other.bond;
        frequency = other.frequency;
        other.bond = nullptr;
    }
}
