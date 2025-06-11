/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: i2c_access.c
 *
 * Description::
 * Connecting to the i2c driver via ioctl
 **************************************************************/
#include "gpio_library/core/i2c_access.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>

static i2c_data driver_data;
uint8_t isCreated = 0;

i2c_data *i2c_open_driver(void)
{
    if (isCreated != 0)
    {
        printf("File Descriptor already created!\n");
        printf("Only allowed one!\n");
        return NULL;
    }

    if ((driver_data.fd = open("/dev/i2c-1", O_RDWR)) < 0)
    {
        printf("Failed to open i2c device\n");
        return NULL;
    }

    isCreated = 1;

    return &driver_data;
}

int i2c_set_target_address(uint8_t address)
{
    if (ioctl(driver_data.fd, I2C_SLAVE, address) < 0)
    {
        printf("Failed to access bus.\n");
        return -1;
    }
    return 0;
}

int i2c_write(const char *buf, uint32_t len, i2c_data *data)
{
    write(data->fd, buf, len);
    return 0;
}

int i2c_read(uint8_t reg, char *buf, uint32_t len, i2c_data *data)
{
    uint8_t temp[1] = {reg};
    write(data->fd, temp, 1);
    read(data->fd, buf, len);
    return 0;
}

void hardware_i2c_clean(i2c_data *data)
{
    if (close(data->fd) != 0)
    {
        perror("Failed to close i2c device.\n");
    }

    data->fd = 0;
    isCreated = 0;
}

i2c_data duplicate_path()
{
    return driver_data;
}