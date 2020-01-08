#ifndef AFROS_LCD_CONNECTION_HPP
#define AFROS_LCD_CONNECTION_HPP

#include "afros_core/connection/connection.hpp"
#include "afros_core/connection/codeable.hpp"

#include "afros_jetson/lcd/i2c_lcd_device.hpp"

namespace afros_jetson{
    class lcd_data : public afros_core::codeable{
        boost::array<boost::array<uint8_t, 20>, 4> lines;
        bool changed;

        void decode_private(const afros_core::raw_data& data);

    public:
        lcd_data();
        explicit lcd_data(const boost::array<std::string, 4>& data);
        explicit lcd_data(const afros_core::raw_data& data);

        void decode(const afros_core::raw_data& data) override;
        afros_core::raw_data encode() override;

        boost::array<std::string, 4> as_strings();
        bool was_changed();
        void reset();

        void set_data(const boost::array<std::string, 4>& data);
    };

    class lcd_connection : public afros_core::connection{
        i2c_lcd_device lcd;
        lcd_data data;
        ros::SteadyTimer lcd_timer;

    public:
        lcd_connection(uint8_t bus_num, uint8_t address, ros::NodeHandle& node, double fps);
        void main_loop() override;

        void set_data(const boost::array<std::string, 4>& lines);
    };
}

#endif //AFROS_LCD_CONNECTION_HPP
