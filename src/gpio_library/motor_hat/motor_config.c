/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: motor_config.c
 *
 * Description::
 * Mini library for running proper delays and timers
 **************************************************************/
#include "gpio_library/core/pins.h"
#include "gpio_library/core/timer.h"
#include "gpio_library/core/i2c_access.h"
#include "gpio_library/motor_hat/motor_config.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>

#define DEFAULT_PIN 4

#define BCM2708_PERI_BASE 0x20000000
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)

#define I2C_ADDRESS 0x40

volatile unsigned int *gpio = NULL;
void *gpio_mmap = NULL;

i2c_data *i2c_driver = NULL;

volatile unsigned int *motor_init(void)
{

    MMap_Config config;
    config.protection = PROT_READ | PROT_WRITE;
    config.flags = MAP_SHARED;
    memcpy(config.path, "/dev/gpiomem", 13);
    config.location = GPIO_BASE;
    config.perms = O_RDWR | O_SYNC;

    gpio = init_io(config, &gpio_mmap);
    if (!gpio)
    {
        printf("Failed to map GPIO memory.\n");
        return NULL;
    }

    if (motor_i2c_open(I2C_ADDRESS) != 0)
    {
        printf("Failed to open I2C.\n");
        return NULL;
    }

    return gpio;
}

void motor_gpio_mode(UWORD pin, UWORD mode)
{
    if (mode == 0)
    {
        set_input(gpio, pin);
    }
    else
    {
        set_output(gpio, pin);
    }
}

void motor_write(UWORD pin, UBYTE value)
{
    write_pin(gpio, pin, value);
}

int motor_read(UWORD pin)
{
    return read_pin(gpio, pin);
}

void motor_delay_ms(int milliseconds)
{
    delay(0, milliseconds, 0);
}

int motor_i2c_open(UWORD address)
{
    i2c_driver = i2c_open_driver();
    if (!i2c_driver)
    {
        printf("Failed to get the driver!\n");
        return -1;
    }

    if (i2c_set_target_address(address) != 0)
    {
        printf("Failed to set address.\n");
        return -1;
    }

    i2c_driver->address = address;

    return 0;
}

void motor_i2c_write_byte(uint8_t cmd, uint8_t value)
{
    char buf[2] = {cmd, value};

    i2c_write(buf, 2, i2c_driver);
}

uint8_t motor_i2c_read_byte(UBYTE cmd)
{
    char buf[1] = {0};
    uint8_t ref = 0;
    i2c_read(cmd, buf, 1, i2c_driver);
    ref = (uint8_t)buf[0];
    return ref;
}

uint16_t motor_i2c_read_word(UBYTE cmd)
{
    char buf[2] = {0};
    i2c_read(cmd, buf, 2, i2c_driver);
    uint16_t ref = (uint16_t)buf[1] << 8;
    return (ref | buf[0]);
}

void motor_exit(void)
{
    clean(gpio_mmap);

    if (i2c_driver)
    {
        hardware_i2c_clean(i2c_driver);
    }

    i2c_driver = NULL;
}
