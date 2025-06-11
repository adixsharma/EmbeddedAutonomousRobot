/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: tcs_controller.c
 *
 * Description::
 * Code for controlling the TCS34725 RGB sensor
 **************************************************************/
#include <stdio.h>
#include <unistd.h>
#include "gpio_library/TCS34725/tcs_controller.h"

int rgb_write_16(i2c_data *i2c, uint8_t register_adr, const char value)
{
    char data[2];
    data[0] = register_adr | 0x80;
    data[1] = value;

    return i2c_write(data, 2, i2c);
}

int rgb_read_16(i2c_data *i2c, uint8_t register_adr, char *value)
{
    return i2c_read(register_adr | 0x80, value, 1, i2c);
}

int read_channel_16(i2c_data *i2c, uint8_t low_reg, uint16_t *out)
{
    char low = 0, high = 0;
    if (rgb_read_16(i2c, low_reg, &low) < 0)
        return -1;
    if (rgb_read_16(i2c, low_reg + 1, &high) < 0)
        return -1;

    *out = ((uint8_t)high << 8) | (uint8_t)low;
    return 0;
}

int rgb_init(i2c_data *i2c)
{
    i2c->address = 0x29;

    uint8_t power = ENABLE_POWER_ON | ENABLE_ADC;
    printf("DEBUG: Writing ENABLE register with value: 0x%02X\n", power);
    if (rgb_write_16(i2c, ENABLE, power) < 0)
    {
        printf("Failed to write ENABLE register.\n");
        return -1;
    }

    return 0;
}

int set_timing(i2c_data *i2c, uint16_t milliseconds)
{
    if (milliseconds > 700)
    {
        printf("Milliseconds too high, setting to 700.\n");
        milliseconds = 700;
    }

    uint8_t fast_time = 256 - (uint8_t)(milliseconds / 2.4);

    if (rgb_write_16(i2c, TIMER, fast_time) < 0)
    {
        printf("Failed to write TIMER.\n");
        return -1;
    }

    return 0;
}

int set_gain(i2c_data *i2c, uint8_t gain)
{
    if (gain != GAIN_1X && gain != GAIN_4X &&
        gain != GAIN_16X && gain != GAIN_60X)
    {
        printf("Invalid gain value.\n");
        return -1;
    }

    if (rgb_write_16(i2c, GAIN, gain) < 0)
    {
        printf("Failed to write GAIN.\n");
        return -1;
    }

    return 0;
}

rgb_color sense_color(i2c_data *i2c)
{
    rgb_color color = {.is_valid = -1};

    if (read_channel_16(i2c, CLEAR_DATA, &color.clear) < 0)
        return color;
    if (read_channel_16(i2c, RED_DATA, &color.red) < 0)
        return color;
    if (read_channel_16(i2c, GREEN_DATA, &color.green) < 0)
        return color;
    if (read_channel_16(i2c, BLUE_DATA, &color.blue) < 0)
        return color;

    color.is_valid = 1;
    return color;
}