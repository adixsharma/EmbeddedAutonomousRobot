/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: tpool.c
 *
 * Description:: Code for thread pool related function. By utilizing thread
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
#include "gpio_library/core/tpool.h"
#include <stdlib.h>

static tpool_work_t *tpool_work_create(thread_func_t func, void *arg)
{
    tpool_work_t *work;

    if (func == NULL)
        return NULL;

    work = malloc(sizeof(*work));
    work->func = func;
    work->arg = arg;
    work->next = NULL;
    return work;
}

static void tpool_work_destroy(tpool_work_t *work)
{
    if (work == NULL)
        return;
    free(work);
}

static tpool_work_t *tpool_work_get(tpool_t *tm)
{
    tpool_work_t *work;

    if (tm == NULL)
        return NULL;

    work = tm->work_first;
    if (work == NULL)
        return NULL;

    if (work->next == NULL)
    {
        tm->work_first = NULL;
        tm->work_last = NULL;
    }
    else
    {
        tm->work_first = work->next;
    }

    return work;
}

static void *tpool_worker(void *arg)
{
    tpool_t *tm = arg;
    tpool_work_t *work;

    while (1)
    {
        // Lock the mutex to access the work queue and pool state
        pthread_mutex_lock(&(tm->work_mutex));

        // Wait for work if the queue is empty and the pool is not stopped
        while (tm->work_first == NULL && !tm->stop)
        {
            pthread_cond_wait(&(tm->work_cond), &(tm->work_mutex));
        }

        // If the pool is stopped, exit the loop
        if (tm->stop)
        {
            break; // Mutex is held, will be unlocked below
        }

        // Get a work item from the queue
        work = tpool_work_get(tm);
        tm->working_cnt++;                       // Increment count of active workers
        pthread_mutex_unlock(&(tm->work_mutex)); // Unlock mutex before executing work

        // Execute the task if work was successfully retrieved
        if (work != NULL)
        {
            work->func(work->arg);
            tpool_work_destroy(work); // Free the work item
        }

        // Lock the mutex again to update state
        pthread_mutex_lock(&(tm->work_mutex));
        tm->working_cnt--;
        if (!tm->stop && tm->working_cnt == 0 && tm->work_first == NULL)
            pthread_cond_signal(&(tm->working_cond));
        pthread_mutex_unlock(&(tm->work_mutex));
    }

    tm->thread_cnt--;
    pthread_cond_signal(&(tm->working_cond));
    pthread_mutex_unlock(&(tm->work_mutex));
    return NULL;
}

tpool_t *tpool_create(size_t num)
{
    tpool_t *tm;
    pthread_t thread;
    size_t i;

    if (num == 0)
        num = 2;

    tm = calloc(1, sizeof(*tm));
    tm->thread_cnt = num;

    pthread_mutex_init(&(tm->work_mutex), NULL);
    pthread_cond_init(&(tm->work_cond), NULL);
    pthread_cond_init(&(tm->working_cond), NULL);

    tm->work_first = NULL;
    tm->work_last = NULL;

    for (i = 0; i < num; i++)
    {
        pthread_create(&thread, NULL, tpool_worker, tm);
        pthread_detach(thread);
    }

    return tm;
}

void tpool_destroy(tpool_t *tm)
{
    tpool_work_t *work;
    tpool_work_t *work2;

    if (tm == NULL)
        return;

    pthread_mutex_lock(&(tm->work_mutex));
    work = tm->work_first;
    while (work != NULL)
    {
        work2 = work->next;
        tpool_work_destroy(work);
        work = work2;
    }
    tm->work_first = NULL;
    tm->stop = true;
    pthread_cond_broadcast(&(tm->work_cond));
    pthread_mutex_unlock(&(tm->work_mutex));

    tpool_wait(tm);

    pthread_mutex_destroy(&(tm->work_mutex));
    pthread_cond_destroy(&(tm->work_cond));
    pthread_cond_destroy(&(tm->working_cond));

    free(tm);
}

bool tpool_add_work(tpool_t *tm, thread_func_t func, void *arg)
{
    tpool_work_t *work;

    if (tm == NULL)
        return false;

    work = tpool_work_create(func, arg);
    if (work == NULL)
        return false;

    pthread_mutex_lock(&(tm->work_mutex));
    if (tm->work_first == NULL)
    {
        tm->work_first = work;
        tm->work_last = tm->work_first;
    }
    else
    {
        tm->work_last->next = work;
        tm->work_last = work;
    }

    pthread_cond_broadcast(&(tm->work_cond));
    pthread_mutex_unlock(&(tm->work_mutex));

    return true;
}

void tpool_wait(tpool_t *tm)
{
    if (tm == NULL)
        return;

    pthread_mutex_lock(&(tm->work_mutex));
    while (1)
    {
        if (tm->work_first != NULL || (!tm->stop && tm->working_cnt != 0) || (tm->stop && tm->thread_cnt != 0))
        {
            pthread_cond_wait(&(tm->working_cond), &(tm->work_mutex));
        }
        else
        {
            break;
        }
    }
    pthread_mutex_unlock(&(tm->work_mutex));
}
