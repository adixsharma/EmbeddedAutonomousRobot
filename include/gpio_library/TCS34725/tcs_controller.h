/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: tcs_controller.h
 *
 * Description::
 * The header file for the TCS34725 color sensor library.
 **************************************************************/
#pragma once

#include <stdint.h>
#include "gpio_library/core/i2c_access.h"

#define TCS_I2C_ACCESS 0x29

// Configuation Addresses
#define ENABLE 0x00

#define ENABLE_POWER_ON (1 << 0)
#define ENABLE_ADC (1 << 1)
#define ENABLE_WAIT (1 << 3)
#define ENABLE_INTERRUPT (1 << 4)

#define TIMER 0x01
#define INTERRUPT 0x0C

#define GAIN 0x0F

#define GAIN_1X ((0 << 0) | (0 << 0))
#define GAIN_4X ((0 << 1) | (1 << 0))
#define GAIN_16X ((0 << 0) | (1 << 1))
#define GAIN_60X ((0 << 1) | (1 << 1))

#define PART_ID 0x12

// Data Addresses
#define CLEAR_DATA 0x14
#define CLEAR_DATAH 0x15
#define RED_DATA 0x16
#define RED_DATAH 0x17
#define GREEN_DATA 0x18
#define GREEN_DATAH 0x19
#define BLUE_DATA 0x1A
#define BLUE_DATAH 0x1B

typedef struct rgb_color
{
    int is_valid;
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} rgb_color;

int rgb_init(i2c_data *i2c);

int set_timing(i2c_data *i2c, uint16_t milliseconds);

int set_gain(i2c_data *i2c, uint8_t gain);

rgb_color sense_color(i2c_data *i2c);