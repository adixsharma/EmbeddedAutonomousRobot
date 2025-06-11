/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: timer.c
 *
 * Description::
 * Mini library for running proper delays and timers
 **************************************************************/
#include "gpio_library/core/timer.h"

#define SECONDS 1000000
#define MILLISECONDS 1000

#define GETSEC() (now.tv_sec - start.tv_sec) * 1000000
#define GETNANO() (now.tv_nsec - start.tv_nsec) / 1000

void delay(uint64_t seconds, uint64_t milliseconds, uint64_t microseconds)
{

    uint64_t end_time = 0;
    end_time += seconds * SECONDS;
    end_time += milliseconds * MILLISECONDS;
    end_time += microseconds;

    struct timespec start, now;

    clock_gettime(CLOCK_MONOTONIC, &start);
    clock_gettime(CLOCK_MONOTONIC, &now);

    uint64_t elapsed_time = 0;

    // Check between deltaTime and given time
    while ((elapsed_time = GETSEC() + GETNANO()) < end_time)
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
    }
}

struct timespec get_time()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now;
}

double get_deltatime(struct timespec *start)
{
    struct timespec now;
    now = get_time();

    double delta_time = (now.tv_sec - start->tv_sec);

    delta_time += (now.tv_nsec - start->tv_nsec) / 1e9;
    *start = now;

    return delta_time;
}
