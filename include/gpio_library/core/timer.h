/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: timer.h
 *
 * Description::
 * A mini library for running proper delays and timers
 **************************************************************/
#pragma once

#include <stdint.h>
#include <time.h>

// Create an accurate delay
void delay(uint64_t seconds, uint64_t milliseconds, uint64_t microseconds);

// Recieve a time stamp to keep track of time
struct timespec get_time();

// Create deltatime frame based calcs
double get_deltatime(struct timespec *start);
