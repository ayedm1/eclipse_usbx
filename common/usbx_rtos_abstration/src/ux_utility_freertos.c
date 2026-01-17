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


/* Include necessary system files.  */

#define UX_SOURCE_CODE

#include "ux_api.h"

#if defined(UX_FREERTOS_BINDING)

/* Map ThreadX-style priority (lower = higher) to FreeRTOS (higher = higher).
   USBX commonly uses priorities like 2 and 20; FreeRTOS may have fewer levels,
   so we scale into [0..configMAX_PRIORITIES-1]. */
static UBaseType_t ux_priority_to_freertos(UINT priority)
{
#if (configMAX_PRIORITIES > 1)
    ULONG p = (priority > 31u) ? 31u : (ULONG)priority;
    return (UBaseType_t)(((31u - p) * (configMAX_PRIORITIES - 1u)) / 31u);
#else
    UX_PARAMETER_NOT_USED(priority);
    return (UBaseType_t)0;
#endif
}

/* Adapter to convert USBX timer callback to FreeRTOS signature. */
static VOID ux_timer_callback(TimerHandle_t xTimer)
{
    UX_TIMER *timer;

    timer = (UX_TIMER *)pvTimerGetTimerID(xTimer);

    if (timer && timer -> timer_function)
    {
        timer -> timer_function(timer -> timer_internal);
    }
}

/* Adapter to convert USBX thread entry to FreeRTOS task entry. */
static VOID ux_thread_entry_adapter(VOID *param)
{
    UX_THREAD *thread;

    thread = (UX_THREAD *)param;

    if (thread && thread -> thread_entry)
    {
        thread -> thread_entry(thread -> thread_entry_parameter);
    }

    /* Delete the task when the thread function exits. */
    vTaskDelete(NULL);
}

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_create                           PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates a thread for USBX.                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    thread_ptr                            Thread control block pointer  */
/*    name                                  Pointer to thread name string */
/*    entry_function                        Entry function of the thread  */
/*    entry_input                           32-bit input value to thread  */
/*    stack_start                           Pointer to start of stack     */
/*    stack_size                            Stack size in bytes           */
/*    priority                              Priority of thread (0-31)     */
/*    preempt_threshold                     Preemption threshold          */
/*    time_slice                            Thread time-slice value       */
/*    auto_start                            Automatic start selection     */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xTaskCreate                         FreeRTOS create thread function */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_thread_create(UX_THREAD *thread_ptr, CHAR *name,
                                VOID (*entry_function)(ULONG), ULONG entry_input,
                                VOID *stack_start, ULONG stack_size,
                                UINT priority, UINT preempt_threshold,
                                ULONG time_slice, UINT auto_start)
{

    if (thread_ptr == UX_NULL || entry_function == UX_NULL)
        return UX_ERROR;

    thread_ptr -> thread_entry = entry_function;
    thread_ptr -> thread_entry_parameter = entry_input;

    /* Create FreeRTOS task. Note: stack_start is ignored for dynamic create. */
    {
        uint16_t stack_depth = (uint16_t)(stack_size / sizeof(StackType_t));
        if (stack_depth == 0)
            stack_depth = 1;

    if (xTaskCreate(ux_thread_entry_adapter,
                    (const char *)name,
                    stack_depth,
                    (void *)thread_ptr,
                    ux_priority_to_freertos(priority),
                    (TaskHandle_t *)&thread_ptr -> handle) == pdPASS)
    {
        thread_ptr -> thread_id = 1;

        /* Respect auto_start: suspend if requested not to start. */
        if (auto_start == UX_DONT_START)
        {
            vTaskSuspend(thread_ptr -> handle);
        }

        return UX_SUCCESS;
    }

    }

    /* Return completion status.  */
    return UX_ERROR;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_delete                           PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deletes a thread for USBX.                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    thread_ptr                            Thread control block pointer  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    vTaskDelete                         FreeRTOS delete thread service  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_thread_delete(UX_THREAD *thread_ptr)
{

    if (thread_ptr == UX_NULL)
        return UX_ERROR;

    if ((thread_ptr != UX_NULL) && (thread_ptr -> handle != UX_NULL))
    {
        /* Call FreeRTOS to delete the USBX thread.  */
        vTaskDelete(thread_ptr -> handle);

        thread_ptr -> handle = UX_NULL;
        thread_ptr -> thread_id = 0;

        return UX_SUCCESS;
    }

    /* Return completion status.  */
    return UX_ERROR;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_identify                         PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function return a pointer to the calling thread.               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    tx_thread_identify                       ThreadX identify function  */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UX_THREAD *_ux_utility_thread_identify(VOID)
{


    /* If we're under interrupt, the thread returned by tx_thread_identify
        is the thread running prior to the ISR. Instead, we set it to null.  */
    return(UX_NULL);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_relinquish                       PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function helps the thread relinquish its control.              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    vTaskYield                      FreeRTOS relinquish thread function */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
VOID  _ux_utility_thread_relinquish(VOID)
{

    /* Call FreeRTOS to relinquish a USBX thread.  */
    taskYIELD();
}

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_resume                           PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function resumes a thread for USBX.                            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    thread_ptr                            Thread control block pointer  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    vTaskResume                         FreeRTOS resume thread function */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_thread_resume(UX_THREAD *thread_ptr)
{

    if (thread_ptr == UX_NULL || thread_ptr -> handle == UX_NULL)
        return UX_ERROR;

    /* Call FreeRTOS to resume USBX thread.  */
    vTaskResume(thread_ptr->handle);

    /* Return completion status.  */
    return(UX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_schedule_other                   PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function force the scheduling of all other threads.            */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    caller_priority                        Priority to restore.          */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*      vTaskYield                          FreeRTOS yield function       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_thread_schedule_other(UINT caller_priority)
{

    taskYIELD();

    /* Return completion status.  */
    return(UX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_sleep                            PORTABLE C      */
/*                                                           6.1.11       */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Mohamed AYED                                                        */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function causes the calling thread to sleep for the            */
/*    specified number of ticks.                                          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ticks                                 Number of ticks to sleep      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*      vTaskDelay                         FreeRTOS sleep function        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_thread_sleep(ULONG ticks)
{
    /* Call FreeRTOS sleep function.  */
    vTaskDelay((TickType_t)ticks);

    return(UX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_thread_suspend                          PORTABLE C      */
/*                                                           6.1.11       */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function suspends thread for USBX.                             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    thread_ptr                            Thread control block pointer  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    vTaskSuspend                        FreeRTOS suspend thread service */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_thread_suspend(UX_THREAD *thread_ptr)
{

    /* Call FreeRTOS to suspend USBX thread.  */
    if (thread_ptr == UX_NULL || thread_ptr -> handle == UX_NULL)
        return UX_ERROR;

    vTaskSuspend(thread_ptr->handle);

    /* Return completion status.  */
    return(UX_SUCCESS);
}

/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_semaphore_create                        PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates a semaphore.                                  */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    semaphore                             Semaphore to create           */
/*    semaphore_name                        Semaphore name                */
/*    initial_count                         Initial semaphore count       */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xSemaphoreCreateCounting              FreeRTOS semaphore create     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_semaphore_create(UX_SEMAPHORE *semaphore, CHAR *semaphore_name, UINT initial_count)
{

    UX_PARAMETER_NOT_USED(semaphore_name);

    /* Call FreeRTOS to create the semaphore.  */
    semaphore -> handle = xSemaphoreCreateCounting(0xFFFF, initial_count);

    if (semaphore -> handle)
    {
        semaphore -> semaphore_id = 1;

        return UX_SUCCESS;
    }

    /* Return completion status.  */
    return UX_ERROR;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_semaphore_delete                        PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deletes the specified semaphore.                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    semaphore                             Semaphore to delete           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    vSemaphoreDelete                   FreeRTOS semaphore delete        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_semaphore_delete(UX_SEMAPHORE *semaphore)
{

    if (semaphore && semaphore -> handle)
    {
        /* Call FreeRTOS Semaphore delete function.  */
        vSemaphoreDelete(semaphore -> handle);

        semaphore -> handle = UX_NULL;
        semaphore -> semaphore_id = 0;

        return UX_SUCCESS;
    }

    /* Return completion status.  */
    return UX_ERROR;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_semaphore_get                           PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function gets a semaphore signal.                              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    semaphore                             Semaphore to get signal from  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xTaskGetCurrentTaskHandle           FreeRTOS identify thread        */
/*    vTaskGetInfo                        FreeRTOS get thread info        */
/*    xSemaphoreTake                      FreeRTOS semaphore get          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_semaphore_get(UX_SEMAPHORE *semaphore, ULONG semaphore_signal)
{

UINT        status;

    if (semaphore == UX_NULL || semaphore -> handle == UX_NULL)
        return UX_ERROR;

    TickType_t wait_ticks = (semaphore_signal == UX_WAIT_FOREVER) ? portMAX_DELAY : (TickType_t)semaphore_signal;

    status = (xSemaphoreTake(semaphore -> handle, wait_ticks) == pdTRUE) ? UX_SUCCESS : UX_ERROR;

    /* Return completion status.  */
    return(status);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_semaphore_put                           PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sets a semaphore signal.                              */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    semaphore                             Semaphore to signal           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xSemaphoreGive                        FreeRTOS semaphore put        */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_semaphore_put(UX_SEMAPHORE *semaphore)
{

BaseType_t  xHigherPriorityTaskWoken = pdFALSE;
UINT        status;

    if (semaphore == UX_NULL)
        return UX_ERROR;

    /* Put a FreeRTOS semaphore.  */
    if (xPortIsInsideInterrupt())
    {
        status = xSemaphoreGiveFromISR((SemaphoreHandle_t)semaphore -> handle, &xHigherPriorityTaskWoken);

        if (status == pdPASS)
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    } else {
        status = xSemaphoreGive((SemaphoreHandle_t)semaphore -> handle);
    }

    /* Return completion status.  */
    return UX_ERROR;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_event_flags_create                      PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates a FreeRTOS group of flags                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    group_ptr                             Event flag control group      */
/*    name                                  Pointer to thread name string */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xEventGroupCreate                     FreeRTOS create event flag    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_event_flags_create(UX_EVENT_FLAGS_GROUP *group_ptr, CHAR *name)
{

    UX_PARAMETER_NOT_USED(name);

    /* Call FreeRTOS to create the event flags.  */
    group_ptr -> handle = xEventGroupCreate();

#if (configSUPPORT_STATIC_ALLOCATION == 1)
    group_ptr -> handle = xEventGroupCreateStatic(&group_ptr->buffer);
#else
    group_ptr -> handle = xEventGroupCreate();
#endif

    if (group_ptr -> handle != UX_NULL)
    {
        group_ptr -> event_flags_group_id = 1;

        return UX_SUCCESS;
    }
    else
    {
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_UTILITY, UX_EVENT_ERROR);

        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_EVENT_ERROR, group_ptr, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return UX_ERROR;
    }

    /* Return completion status.  */
//    return UX_ERROR;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_event_flags_delete                      PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates a FreeRTOS group of flags                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    group_ptr                             Event flag control group      */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    vEventGroupDelete                     FreeRTOS delete event flag    */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_event_flags_delete(UX_EVENT_FLAGS_GROUP *group_ptr)
{

    if ((group_ptr != UX_NULL) && (group_ptr -> handle != UX_NULL))
    {
        /* Call FreeRTOS to delete the event flags.  */
        vEventGroupDelete(group_ptr -> handle);

        group_ptr -> handle = UX_NULL;
        group_ptr -> event_flags_group_id = 0;

        return UX_SUCCESS;
    }

    /* Return completion status.  */
    return(UX_ERROR);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_event_flags_get                         PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function get event flags from event flag group                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    group_ptr                             Event flag control group      */
/*    requested_flags                       32 bits variable event flags  */
/*    get_option                            AND/OR/CLEAR ... options      */
/*    actual_flag_ptr                       where the flags are placed    */
/*    wait_option                           waiting option                */
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xEventGroupWaitBits                   FreeRTOS get event flag       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_event_flags_get(UX_EVENT_FLAGS_GROUP *group_ptr, ULONG requested_flags,
                                  UINT get_option, ULONG *actual_flags_ptr, ULONG wait_option)
{

EventBits_t bits;
BaseType_t clear;
BaseType_t wait_all;

    if (group_ptr == UX_NULL || group_ptr->handle == UX_NULL || actual_flags_ptr == UX_NULL)
        return UX_ERROR;

    clear = (get_option == UX_AND_CLEAR || get_option == UX_OR_CLEAR) ? pdTRUE : pdFALSE;
    wait_all = (get_option == UX_AND || get_option == UX_AND_CLEAR) ? pdTRUE : pdFALSE;

    bits = xEventGroupWaitBits(group_ptr->handle,
                               (EventBits_t)requested_flags,
                               clear,
                               wait_all,
                               (wait_option == UX_WAIT_FOREVER) ? portMAX_DELAY : (TickType_t)wait_option);

    if ((bits & requested_flags) == 0)
        return UX_ERROR;

    *actual_flags_ptr = (ULONG)bits;
    return UX_SUCCESS;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_event_flags_set                         PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function set event flags from event flag group                 */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    group_ptr                             Event flag control group      */
/*    flags_to_set                          32 bits variable event flags  */
/*    set_option                            set option                    */
/*                                                                        */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xEventGroupSetBits                    FreeRTOS set event flag       */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_event_flags_set(UX_EVENT_FLAGS_GROUP *group_ptr, ULONG flags_to_set, UINT set_option)
{

    UX_PARAMETER_NOT_USED(set_option);

    if (group_ptr == UX_NULL)
        return UX_ERROR;

    /* Call FreeRTOS to set the event flags.  */
    xEventGroupSetBits(group_ptr -> handle, flags_to_set);

    /* Return completion status.  */
    return(UX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_mutex_create                            PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates a protection mutex.                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    mutex                                 Pointer to mutex              */
/*    mutex_name                            Name of mutex                 */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xSemaphoreCreateMutex                 FreeRTOS mutex create         */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_mutex_create(UX_MUTEX *mutex, CHAR *mutex_name)
{

UINT    status;

    UX_PARAMETER_NOT_USED(mutex_name);

    if (mutex == UX_NULL)
        return UX_ERROR;

    /* Call FreeRTOS to create the Mutex object.  */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    mutex->handle = xSemaphoreCreateMutexStatic(&mutex->buffer);
#else
    mutex->handle = xSemaphoreCreateMutex();
#endif

    if (mutex -> handle != UX_NULL)
    {
        mutex -> mutex_id = 1;
        status = UX_SUCCESS;
    }
    else
    {
        status = UX_ERROR;
    }

    /* Return completion status.  */
    return(status);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_mutex_delete                            PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deletes a protection mutex.                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    mutex                                 Pointer to mutex              */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xSemaphoreCreateMutex                 FreeRTOS mutex create         */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_mutex_delete(UX_MUTEX *mutex)
{

    if ((mutex != UX_NULL) && (mutex -> handle != UX_NULL))
    {
        /* Call FreeRTOS to delete the Mutex object.  */
        vSemaphoreDelete(mutex -> handle);

        mutex -> handle = UX_NULL;
        mutex -> mutex_id = 0u;

        return UX_SUCCESS;
    }

    /* Return completion status.  */
    return(UX_ERROR);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_mutex_off                               PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function releases system protection.                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    Mutex                                                               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xSemaphoreGive                          FreeRTOS semaphore give     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
VOID  _ux_utility_mutex_off(UX_MUTEX *mutex)
{

    if (mutex == UX_NULL)
        return;

    /* Call FreeRTOS to release protection.  */
    xSemaphoreGive(mutex -> handle);

    /* Return to caller.  */
    return;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_mutex_on                                PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function gets system protection.                               */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    Mutex                                                               */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xSemaphoreTake                          FreeRTOS semaphore take     */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
VOID  _ux_utility_mutex_on(UX_MUTEX *mutex)
{

    /* Call FreeRTOS to get system mutex.  */
    xSemaphoreTake(mutex -> handle, portMAX_DELAY);

    /* Return to caller.  */
    return;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_timer_create                            PORTABLE C      */
/*                                                           6.1.11       */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function creates a timer.                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    timer                                 Pointer to timer              */
/*    timer_name                            Name of timer                 */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xTimerCreate                         FreeRTOS timer create          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_timer_create(UX_TIMER *timer, CHAR *timer_name, VOID (*expiration_function) (ULONG),
                               ULONG expiration_input, ULONG initial_ticks, ULONG reschedule_ticks,
                               UINT activation_flag)
{

    if (timer == UX_NULL)
        return UX_ERROR;

    BaseType_t auto_reload = (reschedule_ticks != 0) ? pdTRUE : pdFALSE;
    TickType_t period = (TickType_t)(auto_reload ? reschedule_ticks : initial_ticks);

    /* Store callback metadata */
    timer -> timer_function = expiration_function;
    timer -> timer_internal = expiration_input;

    /* Create FreeRTOS timer. */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    timer->handle = xTimerCreateStatic((const CHAR *)timer_name,
                                       period,
                                       auto_reload,
                                       (VOID *)timer,
                                       ux_timer_callback,
                                       &timer -> buffer);
#else
    timer->handle = xTimerCreate((const CHAR *)timer_name,
                                 period,
                                 auto_reload,
                                 (VOID *)timer,
                                 ux_timer_callback);
#endif

    if (timer -> handle == UX_NULL)
        return UX_ERROR;

    timer -> timer_id = 1;

    if (activation_flag == UX_AUTO_ACTIVATE)
    {
        if (xTimerStart(timer -> handle, 0) != pdPASS)
            return UX_ERROR;
    }

    /* Return completion status.  */
    return(UX_SUCCESS);
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_utility_timer_delete                            PORTABLE C      */
/*                                                           6.x          */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deletes a timer.                                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    timer                                 Pointer to timer              */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    xTimerDelete                          FreeRTOS timer delete         */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Components                                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/**************************************************************************/
UINT  _ux_utility_timer_delete(UX_TIMER *timer)
{

UINT    status;

    if (timer == UX_NULL)
        return UX_ERROR;

    if ((timer != UX_NULL) && (timer -> handle != UX_NULL))
    {
        /* Call FreeRTOS to delete the timer object.  */
        xTimerDelete(timer -> handle, 0);

        timer -> handle = UX_NULL;
        timer -> timer_id = 0;

        status = UX_SUCCESS;
    }
    else
    {
        status = UX_ERROR;
    }

    /* Return completion status.  */
    return(status);
}


/* Interrupt control for FreeRTOS. */
ALIGN_TYPE _ux_utility_interrupt_disable(VOID)
{
    /* Prefer mask-based APIs if the port provides them (safe from ISR). */
#if defined(portSET_INTERRUPT_MASK_FROM_ISR) && defined(portCLEAR_INTERRUPT_MASK_FROM_ISR)
    return (ALIGN_TYPE)portSET_INTERRUPT_MASK_FROM_ISR();
#else
    taskENTER_CRITICAL();
    return (ALIGN_TYPE)0;
#endif
}

VOID _ux_utility_interrupt_restore(ALIGN_TYPE flags)
{
#if defined(portSET_INTERRUPT_MASK_FROM_ISR) && defined(portCLEAR_INTERRUPT_MASK_FROM_ISR)
    portCLEAR_INTERRUPT_MASK_FROM_ISR((UBaseType_t)flags);
#else
    UX_PARAMETER_NOT_USED(flags);
    taskEXIT_CRITICAL();
#endif
}

/* Time get using FreeRTOS ticks. */
ULONG _ux_utility_time_get(VOID)
{
    return (ULONG)xTaskGetTickCount();
}

#endif /* UX_FREERTOS_BINDING */
