#ifndef AFROS_LCD_MESSAGE_UTILS_HPP
#define AFROS_LCD_MESSAGE_UTILS_HPP

#include "afros_jetson/lcd_set.h"
#include <boost/array.hpp>

namespace afros_jetson{
    void lcd_set_from_strings(lcd_set& message, const std::string& line1 = "", const std::string& line2 = "",
                              const std::string& line3 = "", const std::string& line4 = "", bool backlight_on = true);
    void lcd_set_from_strings(lcd_set& message, const boost::array<std::string, 4>& lines, bool backlight_on = true);
    boost::array<std::string, 4> strings_from_lcd_set(const lcd_set& message);
}

#endif //AFROS_LCD_MESSAGE_UTILS_HPP
