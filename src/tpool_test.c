/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: tpool_test.c
 *
 * Description::
 * Test for thread pool implementation
 *
 * Reference:
 * Thread Pool in C | John's Blog
 * https://nachtimwald.com/2019/04/12/thread-pool-in-c/
 *
 * Used for educational purposes only.
 * All credits goes to the original author.
 **************************************************************/
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <setjmp.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

#include "gpio_library/core/pins.h"
#include "gpio_library/core/tpool.h"
#include "gpio_library/motor_hat/motor_config.h"
#include "gpio_library/core/i2c_access.h"
#include "gpio_library/core/timer.h"

// GPIO pins
#define SENSOR_A 2
#define SENSOR_B 3
#define SENSOR_C 4
#define SENSOR_D 14

#define BCM2708_PERI_BASE 0x20000000
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)

// Global variables
static int sensor_array[4] = {SENSOR_A, SENSOR_B, SENSOR_C, SENSOR_D};
static tpool_t *tm = NULL;
static volatile unsigned int *gpio = NULL;
static void *gpio_mmap = NULL;

void exit_handler(int sig)
{
    printf("\nCtrl + C detected...\n");
    tpool_wait(tm);
    printf("threads waited.\n");
    tpool_destroy(tm);
    printf("threads destroyed.\n");
    clean(gpio_mmap);
    exit(0);
}

void worker_function(void *arg)
{
    int sensor_pin = sensor_array[(intptr_t)arg];
    int val = read_pin(gpio, sensor_pin);
    printf("Pin#=%d, tid=%lu, ReadVal=%d\n", sensor_pin, pthread_self(), val);
    usleep(500000);
}

int main(int argc, char **argv)
{
    // Initialize GPIO pins
    MMap_Config config;
    config.protection = PROT_READ | PROT_WRITE;
    config.flags = MAP_SHARED;
    memcpy(config.path, "/dev/gpiomem", 13);
    config.location = GPIO_BASE;
    config.perms = O_RDWR | O_SYNC;

    gpio = init_io(config, &gpio_mmap);

    if (!gpio || set_input(gpio, SENSOR_A) != 0)
    {
        printf("Initialization failed!\n");
        return -1;
    }
    if (!gpio || set_input(gpio, SENSOR_B) != 0)
    {
        printf("Initialization failed!\n");
        return -1;
    }
    if (!gpio || set_input(gpio, SENSOR_C) != 0)
    {
        printf("Initialization failed!\n");
        return -1;
    }
    if (!gpio || set_input(gpio, SENSOR_D) != 0)
    {
        printf("Initialization failed!\n");
        return -1;
    }

    // Initialize thread pool
    tm = tpool_create(2);

    signal(SIGINT, exit_handler);

    int sensor_now = 0;
    while (1)
    {
        tpool_add_work(tm, worker_function, (void *)(intptr_t)sensor_now);
        sensor_now = (sensor_now + 1) % 4;
    }
    return 0;
}
