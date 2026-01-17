/***************************************************************************
 * Copyright (c) 2024 Microsoft Corporation
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 **************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Utility                                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/*
 * FreeRTOS USBX RTOS abstraction types.
 * Only provides type definitions; macro mappings are handled in core headers.
 */
#ifndef UX_UTILITY_FREERTOS_H
#define UX_UTILITY_FREERTOS_H

/* FreeRTOS core includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"
#include "timers.h"


/* Thread type */
typedef struct UX_THREAD_STRUCT {
    TaskHandle_t handle;
    VOID (*thread_entry)(ULONG);
    ULONG thread_entry_parameter;
    ULONG thread_id; /* non-zero when created */
} FREERTOS_THREAD;

/* Semaphore type */
typedef struct UX_SEMAPHORE_STRUCT {
    SemaphoreHandle_t   handle;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticSemaphore_t  buffer;
#endif
    ULONG semaphore_id;
    ULONG semaphore_suspended_count;
    ULONG semaphore_count;
} FREERTOS_SEMAPHORE;

/* Mutex type */
typedef struct UX_MUTEX_STRUCT {
    SemaphoreHandle_t handle; /* FreeRTOS mutex */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticSemaphore_t  buffer;
#endif
    ULONG mutex_id;
} FREERTOS_MUTEX;

/* Event flags type */
typedef struct UX_EVENT_FLAGS_GROUP_STRUCT {
    EventGroupHandle_t handle;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticEventGroup_t  buffer;
#endif
    ULONG event_flags_group_id;
} FREERTOS_EVENT_FLAGS_GROUP;

/* Timer type */
typedef struct UX_TIMER_STRUCT {
    TimerHandle_t handle;
    VOID (*timer_function)(ULONG);
    ULONG timer_internal;
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticTimer_t buffer;
#endif
    ULONG timer_id;
} FREERTOS_TIMER;

#define UX_THREAD                               FREERTOS_THREAD
#define UX_SEMAPHORE                            FREERTOS_SEMAPHORE
#define UX_MUTEX                                FREERTOS_MUTEX
#define UX_EVENT_FLAGS_GROUP                    FREERTOS_EVENT_FLAGS_GROUP
#define UX_TIMER                                FREERTOS_TIMER

#ifndef UX_PERIODIC_RATE
#define UX_PERIODIC_RATE                        (configTICK_RATE_HZ)
#endif

#ifndef UX_ASSERT_FAIL
#define UX_ASSERT_FAIL                          for (;;) {vTaskDelay(portMAX_DELAY);}
#endif

#define _ux_system_semaphore_created(sem)       ((sem)->semaphore_id != UX_EMPTY)
#define _ux_system_thread_created(t)            ((t)->thread_id != UX_EMPTY)
#define _ux_system_semaphore_created(sem)       ((sem)->semaphore_id != UX_EMPTY)
#define _ux_system_event_flags_created(e)       ((e)->event_flags_group_id != UX_EMPTY)
#define _ux_system_thread_created(t)            ((t)->thread_id != UX_EMPTY)
#define _ux_system_semaphore_waiting(sem)       (uxSemaphoreGetCount((sem)->handle) != 0)

#define _ux_device_thread_entry(t)              ((t)->thread_entry)
#define _ux_device_semaphore_created(sem)       ((sem)->semaphore_id != UX_EMPTY)
#define _ux_device_semaphore_waiting(sem)       (uxSemaphoreGetCount((sem)->handle) != 0)

#define _ux_host_thread_created(thr)            ((thr)->thread_id != 0)
#define _ux_host_thread_entry(thr)              ((thr)->thread_entry)
#define _ux_host_semaphore_created(sem)         ((sem)->semaphore_id != 0)
#define _ux_host_semaphore_waiting(sem)         ((sem)->semaphore_count != 0)

#endif /* UX_UTILITY_FREERTOS_H */
