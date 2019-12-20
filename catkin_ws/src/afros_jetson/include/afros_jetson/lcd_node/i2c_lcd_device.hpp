#ifndef AFROS_I2C_LCD_DEVICE_HPP
#define AFROS_I2C_LCD_DEVICE_HPP

#include "libi2c/i2c.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace afros_jetson{
    namespace lcd{
        //commands
        constexpr uint8_t LCD_CLEARDISPLAY = 0x01u;
        constexpr uint8_t LCD_RETURNHOME = 0x02u;
        constexpr uint8_t LCD_ENTRYMODESET = 0x04u;
        constexpr uint8_t LCD_DISPLAYCONTROL = 0x08u;
        constexpr uint8_t LCD_CURSORSHIFT = 0x10u;
        constexpr uint8_t LCD_FUNCTIONSET = 0x20u;
        constexpr uint8_t LCD_SETCGRAMADDR = 0x40u;
        constexpr uint8_t LCD_SETDDRAMADDR = 0x80u;

        //flags for display entry mode
        constexpr uint8_t LCD_ENTRYRIGHT = 0x00u;
        constexpr uint8_t LCD_ENTRYLEFT = 0x02u;
        constexpr uint8_t LCD_ENTRYSHIFTINCREMENT = 0x01u;
        constexpr uint8_t LCD_ENTRYSHIFTDECREMENT = 0x00u;

        //flags for display on/off control
        constexpr uint8_t LCD_DISPLAYON = 0x04u;
        constexpr uint8_t LCD_DISPLAYOFF = 0x00u;
        constexpr uint8_t LCD_CURSORON = 0x02u;
        constexpr uint8_t LCD_CURSOROFF = 0x00u;
        constexpr uint8_t LCD_BLINKON = 0x01u;
        constexpr uint8_t LCD_BLINKOFF = 0x00u;

        //flags for display/cursor shift
        constexpr uint8_t LCD_DISPLAYMOVE = 0x08u;
        constexpr uint8_t LCD_CURSORMOVE = 0x00u;
        constexpr uint8_t LCD_MOVERIGHT = 0x04u;
        constexpr uint8_t LCD_MOVELEFT = 0x00u;

        //flags for function set
        constexpr uint8_t LCD_8BITMODE = 0x10u;
        constexpr uint8_t LCD_4BITMODE = 0x00u;
        constexpr uint8_t LCD_2LINE = 0x08u;
        constexpr uint8_t LCD_1LINE = 0x00u;
        constexpr uint8_t LCD_5x10DOTS = 0x04u;
        constexpr uint8_t LCD_5x8DOTS = 0x00u;

        //flags for backlight control
        constexpr uint8_t LCD_BACKLIGHT = 0x08u;
        constexpr uint8_t LCD_NOBACKLIGHT = 0x00u;

        constexpr uint8_t En = 0b00000100u;  //Enable bit
        constexpr uint8_t Rw = 0b00000010u;  //Read/Write bit
        constexpr uint8_t Rs = 0b00000001u;  //Register select bit
    }

    class i2c_lcd_device{
        i2c_device device;

        void write(uint8_t data);
        void lcd_write(uint8_t cmd, uint8_t mode = 0);
        void lcd_write_four_bits(uint8_t data);
        void lcd_strobe(uint8_t data);

        void write_char(char char_value, uint8_t mode = 1);

    public:
        i2c_lcd_device(uint8_t bus_num, uint8_t address);
        ~i2c_lcd_device();

        void display_string(const std::string& string, uint8_t line = 1, uint8_t pos = 0);
        void clear();
        void backlight(bool on);
    };
}

#endif //AFROS_I2C_LCD_DEVICE_HPP
