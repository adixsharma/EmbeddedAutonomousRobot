/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: motor_config.h
 *
 * Description::
 * The header file for the Waveshare motor driver hat library.
 **************************************************************/
#pragma once

#include <stdint.h>

#define UBYTE uint8_t
#define UWORD uint16_t

// Creates the gpio mmap
volatile unsigned int *motor_init(void);

// Initialize the GPIO pins
void motor_gpio_mode(UWORD pin, UWORD mode);

// Wrapper for gpio
void motor_write(UWORD pin, UBYTE value);
int motor_read(UWORD pin);
void motor_delay_ms(int milliseconds);

// Wrapper for i2c_access
int motor_i2c_open(UWORD address);
void motor_i2c_write_byte(uint8_t cmd, uint8_t value);
uint8_t motor_i2c_read_byte(UBYTE cmd);
uint16_t motor_i2c_read_word(UBYTE cmd);

// Cleans the driver struct and the gpio mmap
void motor_exit(void);
