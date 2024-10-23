/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

#include "lmic.h"
#include <stdbool.h>

#define TAG "oslmic"

// RUNTIME STATE
static struct
{
    osjob_t *scheduledjobs;
    osjob_t *runnablejobs;
} OS;

void os_init()
{
    memset(&OS, 0x00, sizeof(OS));
    hal_init();
    radio_init();
    LMIC_init();
}

ostime_t os_getTime()
{
    return hal_ticks();
}

static u1_t unlinkjob(osjob_t **pnext, osjob_t *job)
{
    for (; *pnext; pnext = &((*pnext)->next))
    {
        if (*pnext == job)
        { // unlink
            *pnext = job->next;
            return 1;
        }
    }
    return 0;
}

// clear scheduled job
void os_clearCallback(osjob_t *job)
{
    hal_disableIRQs();
    u1_t res = unlinkjob(&OS.scheduledjobs, job) || unlinkjob(&OS.runnablejobs, job);
    hal_enableIRQs();
#if LMIC_DEBUG_LEVEL > 1
    if (res)
         ESP_LOGI(TAG, "%lu: Cleared job %p", os_getTime(), job);
#endif
}

// schedule immediately runnable job
void os_setCallback(osjob_t *job, osjobcb_t cb)
{
    osjob_t **pnext;
    hal_disableIRQs();
    // remove if job was already queued
    os_clearCallback(job);
    // fill-in job
    job->func = cb;
    job->next = NULL;
    // add to end of run queue
    for (pnext = &OS.runnablejobs; *pnext; pnext = &((*pnext)->next))
        ;
    *pnext = job;
    hal_enableIRQs();
#if LMIC_DEBUG_LEVEL > 1
     ESP_LOGI(TAG, "%lu: Scheduled job %p, cb %p ASAP", os_getTime(), job, cb);
#endif
}

// schedule timed job
void os_setTimedCallback(osjob_t *job, ostime_t time, osjobcb_t cb)
{
    osjob_t **pnext;
    hal_disableIRQs();
    // remove if job was already queued
    os_clearCallback(job);
    // fill-in job
    job->deadline = time;
    job->func = cb;
    job->next = NULL;
    // insert into schedule
    for (pnext = &OS.scheduledjobs; *pnext; pnext = &((*pnext)->next))
    {
        if ((*pnext)->deadline - time > 0)
        { // (cmp diff, not abs!)
            // enqueue before next element and stop
            job->next = *pnext;
            break;
        }
    }
    *pnext = job;
    hal_enableIRQs();
#if LMIC_DEBUG_LEVEL > 1
    ostime_t now = os_getTime();
    ESP_LOGI(TAG, "%lu: Scheduled timed job %p, cb %p at %lu", now, job, cb, time);
    set_sleep_time(time);
    /* ostime_t diff = (time - hal_ticks());
    uint64_t diff_us = diff * 20; //(OSTICKS_PER_SEC / 10000)
    if (diff > 1000)
    {
        ESP_LOGI(TAG, "Sleeping til %lu for %lu,  %llu", time, diff, diff_us - 1000);
        set_sleep_time(diff_us - 1000);
        //esp_sleep_enable_timer_wakeup(diff_us);
        //esp_light_sleep_start();
        ESP_LOGI(TAG, "Woke up");
        esp_sleep_wakeup_cause_t wakeup_reason;

        wakeup_reason = esp_sleep_get_wakeup_cause();

        switch(wakeup_reason){
            case ESP_SLEEP_WAKEUP_EXT0 :  ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_IO"); break;
            case ESP_SLEEP_WAKEUP_EXT1 :  ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_CNTL"); break;
            case ESP_SLEEP_WAKEUP_TIMER :  ESP_LOGI(TAG, "Wakeup caused by timer"); break;
            case ESP_SLEEP_WAKEUP_TOUCHPAD :  ESP_LOGI(TAG, "Wakeup caused by touchpad"); break;
            case ESP_SLEEP_WAKEUP_ULP :  ESP_LOGI(TAG, "Wakeup caused by ULP program"); break;
            default :  ESP_LOGI(TAG, "Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
        }

    }*/

#endif
}

// execute jobs from timer and from run queue
void os_runloop()
{
    while (1)
    {
        os_runloop_once();
    }
}

void os_runloop_once()
{
#if LMIC_DEBUG_LEVEL > 1
    bool has_deadline = false;
#endif
    osjob_t *j = NULL;
    hal_disableIRQs();
    // check for runnable jobs
    if (OS.runnablejobs)
    {
        j = OS.runnablejobs;
        OS.runnablejobs = j->next;
    }
    else if (OS.scheduledjobs && hal_checkTimer(OS.scheduledjobs->deadline))
    { // check for expired timed jobs
        j = OS.scheduledjobs;
        OS.scheduledjobs = j->next;
#if LMIC_DEBUG_LEVEL > 1
        has_deadline = true;
#endif
    }
    else
    {                // nothing pending
        hal_sleep(); // wake by irq (timer already restarted)
    }
    hal_enableIRQs();
    if (j)
    { // run job callback
#if LMIC_DEBUG_LEVEL > 1
        ESP_LOGI(TAG, "%lu: Running job %p, cb %p, deadline %lu", os_getTime(), j, j->func, has_deadline ? j->deadline : 0);
#endif
        j->func(j);
    }
}
