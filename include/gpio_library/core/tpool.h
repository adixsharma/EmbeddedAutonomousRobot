/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: tpool.h
 *
 * Description::
 * Code for thread pool related function. By utilizing thread
 * pools, we are able to cut the amout of threads for our car, saving running
 * resources
 *
 * Reference:
 * Thread Pool in C | John's Blog
 * https://nachtimwald.com/2019/04/12/thread-pool-in-c/
 *
 * Used for educational purposes only.
 * All credits goes to the original author.
 **************************************************************/
#ifndef __TPOOL_H__
#define __TPOOL_H__

#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>

struct tpool;
typedef struct tpool tpool_t;

// Function pointer type for thread pool tasks
typedef void (*thread_func_t)(void *arg);

// Represents a task to be executed by a worker thread
struct tpool_work
{
    thread_func_t func;      // Function pointer for the task
    void *arg;               // Argument for the task function
    struct tpool_work *next; // Pointer to the next task in the queue
};

typedef struct tpool_work tpool_work_t;

// Represents the thread pool
struct tpool
{
    tpool_work_t *work_first;    // Pointer to the first task in the queue
    tpool_work_t *work_last;     // Pointer to the last task in the queue
    pthread_mutex_t work_mutex;  // Mutex to protect access to the work queue and pool state
    pthread_cond_t work_cond;    // Condition variable to signal when new work is added
    pthread_cond_t working_cond; // Condition variable to signal when the pool becomes idle
    size_t working_cnt;          // Number of threads currently executing tasks
    size_t thread_cnt;           // Total number of worker threads in the pool
    bool stop;                   // Flag to signal threads to stop processing
};

tpool_t *tpool_create(size_t num);
void tpool_destroy(tpool_t *tm);

bool tpool_add_work(tpool_t *tm, thread_func_t func, void *arg);
void tpool_wait(tpool_t *tm);

#endif /* __TPOOL_H__ */