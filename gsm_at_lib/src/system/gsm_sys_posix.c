/**
 * \file            gsm_sys_template.c
 * \brief           System dependant functions
 */

/*
 * Copyright (c) 2020 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         $_version_$
 */
#include "system/gsm_sys.h"
#include <math.h>

static gsm_sys_mutex_t sys_mutex;
#define NSEC_PER_SEC 1000000000

/**
 * \brief           Init system dependant parameters
 *
 * After this function is called,
 * all other system functions must be fully ready.
 *
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_init(void) {
    return gsm_sys_mutex_create(&sys_mutex);
}


/**
 * \brief           Convert milliseconds to timespec
 * \param[in]       msec: milliseconds value
 * \param[out]      ts: out timesepec value
 */
void
msec_to_timespec(unsigned long msec, struct timespec *ts)
{
    if (msec < 1000){
        ts->tv_sec = 0;
        ts->tv_nsec = msec * 1000000;
    }
    else {
        ts->tv_sec = msec / 1000;
        ts->tv_nsec = (msec - ts->tv_sec * 1000) * 1000000;
    }
}

/**
 * \brief           Convert timespec to ms
 * \param[in]       msec: timesepec value
 * \return          milliseconds value
 */
uint32_t
timespec_to_msec(struct timespec *ts)
{
    long ms;    //milliseconds
    time_t s;   //seconds
    s = ts->tv_sec;
    ms = round(ts->tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
    if (ms > 999) {
        s++;
        ms = 0;
    }
    return ((s * 1000) + ms);
}

/* Add a nanosecond value to a timespec
 *
 * \param r[out] result: a + b
 * \param a[in] base operand as timespec
 * \param b[in] operand in nanoseconds
 */
void
timespec_add_nsec(struct timespec *r, const struct timespec *a, int64_t b)
{
    r->tv_sec = a->tv_sec + (b / NSEC_PER_SEC);
    r->tv_nsec = a->tv_nsec + (b % NSEC_PER_SEC);

    if (r->tv_nsec >= NSEC_PER_SEC) {
        r->tv_sec++;
        r->tv_nsec -= NSEC_PER_SEC;
    } else if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

/* Add a millisecond value to a timespec
 *
 * \param r[out] result: a + b
 * \param a[in] base operand as timespec
 * \param ms[in] operand in milliseconds
 */
void
timespec_add_msec(struct timespec *r, const struct timespec *a, int64_t ms)
{
    timespec_add_nsec(r, a, ms * 1000000);
}

/* Subtract timespecs
 *
 * \param r[out] result: a - b
 * \param a[in] operand
 * \param b[in] operand
 */
void
timespec_sub(struct timespec *r,
         const struct timespec *a, const struct timespec *b)
{
    r->tv_sec = a->tv_sec - b->tv_sec;
    r->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (r->tv_nsec < 0) {
        r->tv_sec--;
        r->tv_nsec += NSEC_PER_SEC;
    }
}

/**
 * \brief           Get current time in units of milliseconds
 * \return          Current time in units of milliseconds
 */
uint32_t
gsm_sys_now(void) {
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);
    return timespec_to_msec(&spec);
}

/**
 * \brief           Protect middleware core
 *
 * Stack protection must support recursive mode.
 * This function may be called multiple times,
 * even if access has been granted before.
 *
 * \note            Most operating systems support recursive mutexes.
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_protect(void) {
    return gsm_sys_mutex_lock(&sys_mutex);
}

/**
 * \brief           Unprotect middleware core
 *
 * This function must follow number of calls of \ref gsm_sys_protect
 * and unlock access only when counter reached back zero.
 *
 * \note            Most operating systems support recursive mutexes.
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_unprotect(void) {
    return gsm_sys_mutex_unlock(&sys_mutex);
}

/**
 * \brief           Create new recursive mutex
 * \note            Recursive mutex has to be created as it may be locked multiple times before unlocked
 * \param[out]      p: Pointer to mutex structure to allocate
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mutex_create(gsm_sys_mutex_t* p) {
    pthread_mutexattr_t attr;
    int ret;

    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    ret = pthread_mutex_init(&p->mutex, &attr);
    p->is_valid = ret;
    return ret == 0;
}

/**
 * \brief           Delete recursive mutex from system
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mutex_delete(gsm_sys_mutex_t* p) {
    int ret ;
    if(! p->is_valid) {
        return 0;
    }

    ret = pthread_mutex_destroy(&p->mutex);
    p->is_valid = false;
    return ret == 0;
}

/**
 * \brief           Lock recursive mutex, wait forever to lock
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mutex_lock(gsm_sys_mutex_t* p) {
    if(! p->is_valid) {
        return 0;
    }
    return pthread_mutex_lock(&p->mutex) == 0;
}

/**
 * \brief           Unlock recursive mutex
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mutex_unlock(gsm_sys_mutex_t* p) {
    if(! p->is_valid) {
        return 0;
    }
    return pthread_mutex_unlock(&p->mutex) == 0;
}

/**
 * \brief           Check if mutex structure is valid system
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mutex_isvalid(gsm_sys_mutex_t* p) {
    return p != NULL && p->is_valid;
}

/**
 * \brief           Set recursive mutex structure as invalid
 * \param[in]       p: Pointer to mutex structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mutex_invalid(gsm_sys_mutex_t* p) {
    p->is_valid = false;
    return 1;
}

/**
 * \brief           Create a new binary semaphore and set initial state
 * \note            Semaphore may only have `1` token available
 * \param[out]      p: Pointer to semaphore structure to fill with result
 * \param[in]       cnt: Count indicating default semaphore state:
 *                     `0`: Take semaphore token immediately
 *                     `1`: Keep token available
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_sem_create(gsm_sys_sem_t* p, uint8_t cnt) {
    int shared_between_processes = 0;
    int result = sem_init(p, shared_between_processes, cnt);
    if(result == 0)
        p->is_valid = true;
    else
        p->valid = false;
    return result == 0;
}

/**
 * \brief           Delete binary semaphore
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_sem_delete(gsm_sys_sem_t* p) {
    if(! p->is_valid)
        ret 0;
    int result = sem_destroy(p);
    p->is_valid = false;
    return result == 0;
}

/**
 * \brief           Wait for semaphore to be available
 * \param[in]       p: Pointer to semaphore structure
 * \param[in]       timeout: Timeout to wait in milliseconds. When `0` is applied, wait forever
 * \return          Number of milliseconds waited for semaphore to become available or
 *                      \ref GSM_SYS_TIMEOUT if not available within given time
 */
uint32_t
gsm_sys_sem_wait(gsm_sys_sem_t* p, uint32_t timeout) {
    if(timeout == 0) {  //if no timeout
        return sem_wait(p);
    }

    //we have a timeout
    struct timespec before_call, deadline, after_call, elapsed;
    clock_gettime(CLOCK_MONOTONIC, &before_call);   //get time before call
    timespec_add_msec(&deadline, &before_call, timeout);    //compute deadline

    if( sem_timedwait(p, &deadline) != 0) {   //wait on semaphore
        return GSM_SYS_TIMEOUT; //if timeout
    }
    clock_gettime(CLOCK_MONOTONIC, &after_call);    //get time after call
    timespec_sub(&elapsed, &after_call, &before_call);
    return timespec_to_msec(&elapsed);
}

/**
 * \brief           Release semaphore
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_sem_release(gsm_sys_sem_t* p) {
    return sem_close(p) == 0 ? 1 : 0;
}

/**
 * \brief           Check if semaphore is valid
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_sem_isvalid(gsm_sys_sem_t* p) {
    return p != NULL;
}

/**
 * \brief           Invalid semaphore
 * \param[in]       p: Pointer to semaphore structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_sem_invalid(gsm_sys_sem_t* p) {
    return 1;
}

/**
 * \brief           Create a new message queue with entry type of `void *`
 * \param[out]      b: Pointer to message queue structure
 * \param[in]       size: Number of entries for message queue to hold
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mbox_create(gsm_sys_mbox_t* b, size_t size) {
    static unsigned int suffix = 0;
    pid_t pid;
    char mq_name[56];

    //build message queue name
    pid = getpid();
    sprintf(mq_name, "/%ld_%u", pid, suffix );
    ++suffix;   //can loop

    //create it
    struct mq_attr mqattr;
    mqattr.mq_maxmsg = size;
    mqattr.mq_msgsize = sizeof(void*);
    *b = mq_open(mq_name, O_RDWR | O_CREAT | O_EXCL, S_IRWXU, &mqattr);
    return *b != -1;
}

/**
 * \brief           Delete message queue
 * \param[in]       b: Pointer to message queue structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mbox_delete(gsm_sys_mbox_t* b) {
    if(mq_close(*b) == -1) {
        return 0;
    }
    if(mq_unlink(*b) == -1) {
        return 0;
    }
    return 1;
}

/**
 * \brief           Put a new entry to message queue and wait until memory available
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to entry to insert to message queue
 * \return          Time in units of milliseconds needed to put a message to queue
 */
uint32_t
gsm_sys_mbox_put(gsm_sys_mbox_t* b, void* m) {
    struct timespec before_call, after_call, elapsed;
    clock_gettime(CLOCK_MONOTONIC, &before_call);   //get time before call

    if( mq_send(*b, m, sizeof(m), 0) == -1){ 
        return GSM_SYS_TIMEOUT;
    }

    clock_gettime(CLOCK_MONOTONIC, &after_call);    //get time after call
    timespec_sub(&elapsed, &after_call, &before_call);
    return timespec_to_msec(&elapsed);
}

/**
 * \brief           Get a new entry from message queue with timeout
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to pointer to result to save value from message queue to
 * \param[in]       timeout: Maximal timeout to wait for new message. When `0` is applied, wait for unlimited time
 * \return          Time in units of milliseconds needed to put a message to queue
 *                      or \ref GSM_SYS_TIMEOUT if it was not successful
 */
uint32_t
gsm_sys_mbox_get(gsm_sys_mbox_t* b, void** m, uint32_t timeout) {
    struct timespec before_call, after_call, elapsed;
    ssize_t received_size;
    unsigned int received_prio;
    clock_gettime(CLOCK_MONOTONIC, &before_call);   //get time before call

    if(timeout == 0) {  //if no timeout
        received_size = mq_receive(*b, m, sizeof(void*), &received_prio);
    }
    else {
        timespec_add_msec(&deadline, &before_call, timeout);    //compute deadline
        received_size = mq_timedreceive(*b, m, sizeof(void*), &received_prio, &deadline);
    }
    clock_gettime(CLOCK_MONOTONIC, &after_call);    //get time after call

    if(received_size == -1) {
        return GSM_SYS_TIMEOUT;
    }
    timespec_sub(&elapsed, &after_call, &before_call);
    return timespec_to_msec(&elapsed);
}

/**
 * \brief           Put a new entry to message queue without timeout (now or fail)
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to message to save to queue
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mbox_putnow(gsm_sys_mbox_t* b, void* m) {
    struct timespec fake_deadline;
    memset(&fake_deadline, 0, sizeof(fake_deadline));
    return mq_timedsend(*b, m, sizeof(void*), 0, &fake_deadline) == 0 ? 1 : 0;
}

/**
 * \brief           Get an entry from message queue immediately
 * \param[in]       b: Pointer to message queue structure
 * \param[in]       m: Pointer to pointer to result to save value from message queue to
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mbox_getnow(gsm_sys_mbox_t* b, void** m) {
    struct timespec fake_deadline;
    unsigned int prio;
    memset(&fake_deadline, 0, sizeof(fake_deadline));
    return mq_timedreceive(*b, m, sizeof(void*), &prio, &fake_deadline) == -1 ? 0 : 1;
}

/**
 * \brief           Check if message queue is valid
 * \param[in]       b: Pointer to message queue structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mbox_isvalid(gsm_sys_mbox_t* b) {
    return b != NULL && *b != GSM_SYS_MBOX_NULL;
}

/**
 * \brief           Invalid message queue
 * \param[in]       b: Pointer to message queue structure
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_mbox_invalid(gsm_sys_mbox_t* b) {
    *b = GSM_SYS_MBOX_NULL;
    return 1;
}

/**
 * \brief           Create a new thread
 * \param[out]      t: Pointer to thread identifier if create was successful.
 *                     It may be set to `NULL`
 * \param[in]       name: Name of a new thread
 * \param[in]       thread_func: Thread function to use as thread body
 * \param[in]       arg: Thread function argument
 * \param[in]       stack_size: Size of thread stack in uints of bytes. If set to 0, reserve default stack size
 * \param[in]       prio: Thread priority
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_thread_create(gsm_sys_thread_t* t, const char* name, gsm_sys_thread_fn thread_func,
                          void* const arg, size_t stack_size, gsm_sys_thread_prio_t prio) {
    pthread_attr_t attr;
    if(pthread_attr_init(&attr) != 0) {
        return 0;
    }

    if(stack_size != 0 && pthread_attr_setstacksize(&attr, stack_size) != 0) {
        pthread_attr_destroy(&attr);
        return 0;
    }

    struct sched_param priority;
    priority.sched_priority = prio;
    if(pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0 || pthread_attr_setschedparam(&attr, &priority) != 0 ) {
        pthread_attr_destroy(&attr);
        return 0;
    }

    if(pthread_create(t, &attr, thread_func, arg) != 0) {
        pthread_attr_destroy(&attr);
        return 0;
    }

    pthread_attr_destroy(&attr);
    pthread_setname_np(*t, name);
    return 1;
}

/**
 * \brief           Terminate thread (shut it down and remove)
 * \param[in]       t: Pointer to thread handle to terminate.
 *                      If set to `NULL`, terminate current thread (thread from where function is called)
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_thread_terminate(gsm_sys_thread_t* t) {
    if(t == NULL)
        return 0;

    if(pthread_join(*t, NULL) != 0) {
        return 0;
    }

    return 1;
}

/**
 * \brief           Yield current thread
 * \return          `1` on success, `0` otherwise
 */
uint8_t
gsm_sys_thread_yield(void) {
    return sched_yield() == 0;
}
