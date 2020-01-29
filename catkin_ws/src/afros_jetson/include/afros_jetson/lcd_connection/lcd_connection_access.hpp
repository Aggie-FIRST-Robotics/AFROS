#ifndef AFROS_LCD_CONNECTION_ACCESS_HPP
#define AFROS_LCD_CONNECTION_ACCESS_HPP

#include "afros_core/access/publication_access.hpp"
#include "afros_core/access/subscription_access.hpp"

namespace afros_jetson{
    typedef boost::array<boost::array<char, 20>, 4> lcd_data;

    class lcd_publication_access : public afros_core::publication_access<lcd_data>{
        afros_core::raw_data pub_convert(const lcd_data& data) override;

    public:
        lcd_publication_access(ros::NodeHandle& node, const std::string& name);
    };

    class lcd_subscription_access : public afros_core::subscription_access<lcd_data>{
        lcd_data sub_convert(const afros_core::raw_data& data) override;
    public:
        lcd_subscription_access(ros::NodeHandle& node, const std::string& name, double frequency, boost::function<void(const lcd_data&)>* callback);
    };
}

#endif //AFROS_LCD_CONNECTION_ACCESS_HPP
