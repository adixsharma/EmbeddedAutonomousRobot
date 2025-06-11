/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: pca9685.c
 *
 * Description::
 * Library for the pca9685 chip and related functions, used to
 * control the motor.
 **************************************************************/
#include "gpio_library/chips/pca9685.h"
#include <math.h>
#include <stdio.h>

static void pca9685_set_pwm(UBYTE channel, UWORD on, UWORD off)
{
    motor_i2c_write_byte(LED0_ON_L + 4 * channel, on & 0xFF);
    motor_i2c_write_byte(LED0_ON_H + 4 * channel, (on >> 8) & 0xFF);
    motor_i2c_write_byte(LED0_OFF_L + 4 * channel, off & 0xFF);
    motor_i2c_write_byte(LED0_OFF_H + 4 * channel, (off >> 8) & 0xFF);
}

volatile unsigned int *pca9685_init()
{
    volatile unsigned int *gpio = motor_init();
    if (gpio == NULL)
    {
        printf("Failed to open PCA9685!\n");
        return NULL;
    }

    // Reset and wait for motor
    motor_i2c_write_byte(MODE1, 0x00);
    motor_delay_ms(10);
    return gpio;
}

void pca9685_set_pwm_freq(UWORD frequency)
{
    frequency *= 0.9;
    double scale_level = 25000000.0;
    scale_level /= 4096.0;
    scale_level /= frequency;
    scale_level -= 1.0;

    UBYTE prescale = (UBYTE)(scale_level + 0.5);

    UBYTE oldmode = motor_i2c_read_byte(MODE1);
    UBYTE newmode = (oldmode & 0x7F) | 0x10;

    motor_i2c_write_byte(MODE1, newmode);
    motor_i2c_write_byte(PRESCALE, prescale);

    motor_delay_ms(5);

    motor_i2c_write_byte(MODE1, oldmode | 0x80);
}

void pca9685_set_pwm_dutycycle(UBYTE channel, UWORD pulse)
{
    if (pulse >= 100)
    {
        pca9685_set_pwm(channel, 0, 4095);
    }
    else
    {
        pca9685_set_pwm(channel, 0, pulse * (4096 / 100) - 1);
    }
}

void pca9685_set_level(UBYTE channel, UWORD value)
{
    if (value == 1)
        pca9685_set_pwm(channel, 0, 4095);
    else
        pca9685_set_pwm(channel, 0, 0);
}
