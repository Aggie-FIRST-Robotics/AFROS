#include "afros_jetson/lcd_connection/lcd_connection.hpp"

afros_jetson::lcd_connection::lcd_connection(uint8_t bus_num, uint8_t address, ros::NodeHandle& node, double fps, const std::string& name) : connection(1, 1, name), lcd(bus_num, address), data(),
                                                                                                                                             lcd_timer(){
    afros_core::subscription lcd_subscription{};
    lcd_subscription.name = lcd_con::LCD_SUBSCRIBER_NAME;
    lcd_subscription.data_function = [this]() -> afros_core::raw_data{
        return data.encode();
    };
    lcd_subscription.max_frequency = 1.0;
    add_subscription(lcd_subscription);

    afros_core::publication lcd_publisher{};
    lcd_publisher.name = lcd_con::LCD_PUBLISHER_NAME;
    lcd_publisher.publish_function = [this](const afros_core::raw_data& message){
        data.decode(message);
    };
    add_publisher(lcd_publisher);

    boost::function<void(const ros::SteadyTimerEvent&)> timer_callback{[this](const ros::SteadyTimerEvent& event){
        ROS_INFO("Running data check");
        if(data.was_changed()){
            ROS_INFO("Data updating");
            auto strings = data.as_strings();
            for(uint8_t x = 0; x < 4; ++x){
                lcd.display_string(strings.at(x), x + 1);
            }
        }
        data.reset();
    }};
    lcd_timer = node.createSteadyTimer(ros::WallDuration{1.0 / fps}, timer_callback);
}

void afros_jetson::lcd_connection::main_loop(const ros::SteadyTimerEvent& event){}

void afros_jetson::lcd_connection::set_data(const boost::array<std::string, 4>& lines){
    data.set_data(lines);
    auto data_val = data.as_strings();
    for(int x = 0; x < 4; x++){
        ROS_INFO("Data line %i: %s", x, data_val.at(x).c_str());
    }
}

afros_jetson::lcd_internal_data::lcd_internal_data(const boost::array<std::string, 4>& data) : lines(), changed(false){
    set_data(data);
}

afros_jetson::lcd_internal_data::lcd_internal_data(const afros_core::raw_data& data) : lines(), changed(false){
    decode_private(data);
}

void afros_jetson::lcd_internal_data::decode(const afros_core::raw_data& data){
    decode_private(data);
}

void afros_jetson::lcd_internal_data::decode_private(const afros_core::raw_data& data){
    if(data.strings.size() != 4){
        ROS_ERROR("Invalid raw_data packet! strings size: %lu", data.strings.size());
        return;
    }
    for(auto& string : data.strings){
        if(string.size() != 20){
            ROS_ERROR("Invalid raw_data packet! string of length %lu found", string.size());
            return;
        }
    }
    for(uint8_t x = 0; x < 4; ++x){
        for(uint8_t y = 0; y < 20; ++y){
            lines.at(x).at(y) = data.strings.at(x).at(y);
        }
    }
    changed = true;
}

afros_core::raw_data afros_jetson::lcd_internal_data::encode(){
    afros_core::raw_data out{};
    for(auto& line : lines){
        out.strings.emplace_back(line.begin(), line.end());
    }
    return out;
}

boost::array<std::string, 4> afros_jetson::lcd_internal_data::as_strings(){
    boost::array<std::string, 4> out{};
    for(uint8_t x = 0; x < 4; ++x){
        char add[21];
        for(uint8_t y = 0; y < 20; y++){
            add[y] = lines.at(x).at(y) == 0 ? ' ' : lines.at(x).at(y);
        }
        add[20] = '\0';
        out.at(x) = std::string{add};
    }
    return out;
}

afros_jetson::lcd_internal_data::lcd_internal_data() : lines(), changed(false){}

bool afros_jetson::lcd_internal_data::was_changed(){
    return changed;
}

void afros_jetson::lcd_internal_data::reset(){
    changed = false;
}

void afros_jetson::lcd_internal_data::set_data(const boost::array<std::string, 4>& data){
    for(size_t x = 0; x < 4; ++x){
        for(uint8_t y = 0; y < std::min(20ul, data.at(x).size()); ++y){
            lines.at(x).at(y) = data.at(x).at(y);
        }
    }
    changed = true;
}
