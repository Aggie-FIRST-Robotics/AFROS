#include <afros_jetson/lcd_connection/lcd_connection.hpp>
#include "afros_jetson/lcd_connection/lcd_connection_access.hpp"
#include "afros_jetson/node_names.hpp"

namespace afros_jetson{
    afros_core::raw_data lcd_publication_access::pub_convert(const lcd_data& data){
        afros_core::raw_data out{};
        for(size_t x = 0; x < lcd_data::size(); ++x){
            out.strings.push_back(std::string{data.at(x).cbegin(), data.at(x).cend()});
        }
        return out;
    }

    lcd_publication_access::lcd_publication_access(ros::NodeHandle& node, const std::string& name) : afros_core::publication_access<lcd_data>(node, {LCD_NODE_NAME, lcd_con::LCD_PUBLISHER_NAME}, 1){}

    lcd_data lcd_subscription_access::sub_convert(const afros_core::raw_data& data){
        lcd_data out{};
        for(size_t x = 0; x < lcd_data::size(); ++x){
            for(size_t y = 0; y < lcd_data::value_type::size(); ++y){
                out.at(x).at(y) = data.strings.at(x).at(y);
            }
        }
        return out;
    }

    lcd_subscription_access::lcd_subscription_access(ros::NodeHandle& node, const std::string& name, double frequency, boost::function<void(const lcd_data&)>* callback)
            : afros_core::subscription_access<lcd_data>(node, {LCD_NODE_NAME, lcd_con::LCD_SUBSCRIBER_NAME}, 1, frequency, callback){

    }
}
