#include "afros_jetson/lcd_node/lcd_message_utils.hpp"

void afros_jetson::lcd_set_from_strings(lcd_set& message, const std::string& line1, const std::string& line2,
                                        const std::string& line3, const std::string& line4, bool backlight_on){
    for(size_t x = 0; x < std::min(line1.size(), lcd_set::_line1_type::size()); ++x){
        message.line1.at(x) = line1.at(x);
    }
    for(size_t x = 0; x < std::min(line2.size(), lcd_set::_line2_type::size()); ++x){
        message.line2.at(x) = line2.at(x);
    }
    for(size_t x = 0; x < std::min(line3.size(), lcd_set::_line3_type::size()); ++x){
        message.line3.at(x) = line3.at(x);
    }
    for(size_t x = 0; x < std::min(line4.size(), lcd_set::_line4_type::size()); ++x){
        message.line4.at(x) = line4.at(x);
    }
    message.backlight_on = backlight_on;
}

boost::array<std::string, 4> afros_jetson::strings_from_lcd_set(const afros_jetson::lcd_set& message){
    boost::array<std::string, 4> out{};
    char line_char[20];

    for(size_t x = 0; x < afros_jetson::lcd_set::_line1_type::size(); ++x){
        line_char[x] = message.line1.at(x);
    }
    out.at(0) = std::string{line_char};

    for(size_t x = 0; x < afros_jetson::lcd_set::_line2_type::size(); ++x){
        line_char[x] = message.line2.at(x);
    }
    out.at(1) = std::string{line_char};

    for(size_t x = 0; x < afros_jetson::lcd_set::_line3_type::size(); ++x){
        line_char[x] = message.line3.at(x);
    }
    out.at(2) = std::string{line_char};

    for(size_t x = 0; x < afros_jetson::lcd_set::_line4_type::size(); ++x){
        line_char[x] = message.line4.at(x);
    }
    out.at(3) = std::string{line_char};

    return out;
}

void afros_jetson::lcd_set_from_strings(lcd_set& message, const boost::array<std::string, 4>& lines, bool backlight_on){
    lcd_set_from_strings(message, lines.at(0), lines.at(1), lines.at(2), lines.at(3), backlight_on);
}
