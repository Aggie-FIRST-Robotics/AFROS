#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "hicpp-signed-bitwise"

#include <zconf.h>
#include <string>
#include <ros/ros.h>
#include "afros_jetson/lcd_node/i2c_lcd_device.hpp"

void afros_jetson::i2c_lcd_device::write(uint8_t data){
    i2c_ioctl_write(&device, 0, &data, 1);
    usleep(100);
}

afros_jetson::i2c_lcd_device::i2c_lcd_device(uint8_t bus_num, uint8_t address)
        : device(), backlight_state(afros_jetson::lcd::LCD_BACKLIGHT){
    memset(&device, 0, sizeof(device));

    const char* bus_name = (std::string("/dev/i2c-") + std::to_string(bus_num)).c_str();
    device.bus = i2c_open(bus_name);
    if(device.bus < 0){
        ROS_ERROR("Cannot open i2c bus %i", bus_num);
    }

    device.addr = address;
    device.iaddr_bytes = 0;
    device.page_bytes = 16;

    //Taken from script, some magic numbers involved but works
    using namespace lcd;
    lcd_write(0x03);
    lcd_write(0x03);
    lcd_write(0x03);
    lcd_write(0x02);

    lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE);
    lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
    lcd_write(LCD_CLEARDISPLAY);
    lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT);
    usleep(200000);
}

void afros_jetson::i2c_lcd_device::lcd_strobe(uint8_t data){
    write(data | lcd::En | backlight_state);
    usleep(500);
    write((data & ~lcd::En) | backlight_state);
    usleep(100);
}

void afros_jetson::i2c_lcd_device::lcd_write_four_bits(uint8_t data){
    write(data | backlight_state);
    lcd_strobe(data);
}

void afros_jetson::i2c_lcd_device::lcd_write(uint8_t cmd, uint8_t mode){
    lcd_write_four_bits(mode | (cmd & 0xF0u));
    lcd_write_four_bits(mode | ((cmd << 4u) & 0xF0u));
}

afros_jetson::i2c_lcd_device::~i2c_lcd_device(){
    i2c_close(device.bus);
}

void afros_jetson::i2c_lcd_device::write_char(char char_value, uint8_t mode){
    lcd_write_four_bits(mode | (char_value & 0xF0u));
    lcd_write_four_bits(mode | ((char_value << 4) & 0xF0u));
}

void afros_jetson::i2c_lcd_device::display_string(const std::string& string, uint8_t line, uint8_t pos){
    uint8_t pos_new = 0;
    if(line == 1){
        pos_new = pos;
    }
    else if(line == 2){
        pos_new = 0x40u + pos;
    }
    else if(line == 3){
        pos_new = 0x14u + pos;
    }
    else if(line == 4){
        pos_new = 0x54u + pos;
    }
    else{
        ROS_ERROR("Bad line num for lcd: %i", line);
        return;
    }

    lcd_write(0x80 + pos_new);

    for(char x : string){
        lcd_write(x, lcd::Rs);
    }
}

void afros_jetson::i2c_lcd_device::clear(){
    lcd_write(lcd::LCD_CLEARDISPLAY);
    lcd_write(lcd::LCD_RETURNHOME);
}

void afros_jetson::i2c_lcd_device::backlight(bool on){
    backlight_state = on ? lcd::LCD_BACKLIGHT : lcd::LCD_NOBACKLIGHT;
    write(backlight_state);
}

#pragma clang diagnostic pop