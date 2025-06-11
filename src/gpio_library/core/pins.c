/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: pins.c
 *
 * Description::
 * Mini Library For manipulating gpio pins
 **************************************************************/
#include "gpio_library/core/pins.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <errno.h>

#define INP_GPIO(g) *(gpio + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define OUT_GPIO(g) *(gpio + ((g) / 10)) |= (1 << (((g) % 10) * 3))

#define GPIO_SET *(gpio + 10 - (state * 3))
#define GET_GPIO(g) (*(gpio + 13) & (1 << g))

#define BLOCKSIZE (4 * 1024)

volatile unsigned int *init_io(MMap_Config config, void **gpio_map)
{
    int mem_fd = 0;

    if ((mem_fd = open(config.path, config.perms)) < 0)
    {
        printf("Failed to open file!\n");
        return NULL;
    }

    *gpio_map = mmap(
        NULL,
        BLOCKSIZE,
        config.protection,
        config.flags,
        mem_fd,
        config.location);

    close(mem_fd);

    if ((*gpio_map) == MAP_FAILED)
    {
        perror("mmap failed");
        return NULL;
    }

    return (volatile unsigned *)*gpio_map;
}

int set_output(volatile unsigned int *gpio, int pin)
{
    int g = 0;
    g = pin;
    INP_GPIO(g);
    OUT_GPIO(g);
    return 0;
}

int set_input(volatile unsigned int *gpio, int pin)
{
    int g = 0;
    g = pin;
    INP_GPIO(g);
    return 0;
}

int write_pins(volatile unsigned int *gpio, int *pins, int num_pins, int state)
{
    int g = 0;
    for (int i = 0; i < num_pins; i++)
    {
        g = pins[i];

        GPIO_SET = (1 << g);
    }

    return 0;
}

int write_pin(volatile unsigned int *gpio, int pin, int state)
{
    int g = 0;
    g = pin;

    GPIO_SET = (1 << g);

    return 0;
}

int *read_pins(volatile unsigned int *gpio, int *pins_val, int *pins, int num_pins)
{
    int g = 0;
    for (int i = 0; i < num_pins; i++)
    {
        g = pins[i];

        pins_val[i] = GET_GPIO(g);
    }

    return pins_val;
}

int read_pin(volatile unsigned int *gpio, int pin)
{
    int g = 0;
    int pin_val = -1;
    g = pin;

    pin_val = GET_GPIO(g);
    return pin_val;
}

int clean(void *gpio_mmap)
{
    munmap(gpio_mmap, BLOCKSIZE);
    return 0;
}