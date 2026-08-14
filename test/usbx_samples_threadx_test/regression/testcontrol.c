/* This is the test control routine for the USBX sample regression tests.
   Each sample is compiled as its own test binary (one test per executable).
   Under CTEST, the test array contains a single entry: test_application_define,
   which each test file defines under #ifdef CTEST.  */

#include "tx_api.h"
#include "ux_api.h"
#include <stdio.h>
#include <stdlib.h>

#define TEST_STACK_SIZE     6144

/* Define the test control thread and shared state.  */

TX_THREAD   test_control_thread;
TX_THREAD   test_thread;

ULONG       test_control_return_status;
ULONG       test_control_successful_tests;
ULONG       test_control_failed_tests;
ULONG       test_control_system_errors;

UCHAR      *test_free_memory_ptr;

VOID        (*test_isr_dispatch)(void);

UCHAR       test_control_thread_stack[TEST_STACK_SIZE];
UCHAR       tests_memory[64*1024];

/* Define the external preempt/system-state references used by test_control_return.  */

extern volatile UINT  _tx_thread_preempt_disable;
extern volatile ULONG _tx_thread_system_state;

/* Test entry type.  */

typedef struct TEST_ENTRY_STRUCT
{
    VOID (*test_entry)(void *);
} TEST_ENTRY;

/* Each binary defines test_application_define in its test source file.  */

void usbx_hid_mouse_demo_device_rtos_test_application_define(void *first_unused_memory);

/* Test array: one entry per binary — always dispatches test_application_define.
   (When compiled with -DCTEST each test file aliases its private function to
   test_application_define via the #ifdef CTEST guard in its own source.)  */

TEST_ENTRY test_control_tests[] =
{
    usbx_hid_mouse_demo_device_rtos_test_application_define,
    UX_NULL
};

/* Thread and return prototypes.  */

void test_control_thread_entry(ULONG thread_input);
void test_control_return(UINT status);

extern ULONG _tx_thread_created_count;

/* main — entry point provided by the test harness (EXTERNAL_MAIN must be defined
   in sample source files so they do not provide their own main).  */

void main(void)
{
    tx_kernel_enter();
}

/* tx_application_define — called by the ThreadX kernel during startup.  */

void tx_application_define(void *first_unused_memory)
{
    test_control_successful_tests = 0;
    test_control_failed_tests     = 0;
    test_control_system_errors    = 0;

    tx_thread_create(&test_control_thread, "test control thread",
                     test_control_thread_entry, 0,
                     test_control_thread_stack, TEST_STACK_SIZE,
                     17, 15, TX_NO_TIME_SLICE, TX_AUTO_START);

    test_free_memory_ptr = &tests_memory[0];
}

/* test_control_thread_entry — dispatches each test in test_control_tests[].  */

void test_control_thread_entry(ULONG thread_input)
{
    ULONG previous_failed;
    ULONG previous_thread_count;
    UINT  i;

    (void)thread_input;

    tx_thread_priority_change(&test_control_thread, 0, &i);

//    printf("**** USBX Samples Regression Test Suite ****\n\n");
//    printf("Version: %s  Data width: x%i\n\n", _ux_version_id, (int)(sizeof(void *) * 8));

    i = 0;
    while (test_control_tests[i].test_entry != UX_NULL)
    {
        previous_failed       = test_control_failed_tests;
        previous_thread_count = _tx_thread_created_count;

        (test_control_tests[i++].test_entry)(test_free_memory_ptr);

        /* If the test created threads, suspend until test_control_return wakes us.  */
        if (test_control_failed_tests == previous_failed &&
            _tx_thread_created_count  != previous_thread_count)
        {
            tx_thread_suspend(&test_control_thread);
        }
    }

    printf("\n**** Summary: Passed: %lu  Failed: %lu  System errors: %lu ****\n\n",
           test_control_successful_tests,
           test_control_failed_tests,
           test_control_system_errors);

    exit((int)test_control_failed_tests);
}

/* test_control_return — called by each test when it is done.  */

void test_control_return(UINT status)
{
    UINT old_posture = TX_INT_ENABLE;

    test_control_return_status = status;

    old_posture = tx_interrupt_control(TX_INT_ENABLE);

    if (status)
        test_control_failed_tests++;
    else
        test_control_successful_tests++;

    /* Basic sanity checks.  */
    if (_tx_thread_preempt_disable)
    {
        printf("  ***** SYSTEM ERROR ***** _tx_thread_preempt_disable non-zero!\n");
        test_control_system_errors++;
    }
    if (_tx_thread_system_state)
    {
        printf("  ***** SYSTEM ERROR ***** _tx_thread_system_state non-zero!\n");
        test_control_system_errors++;
    }
    if (old_posture == TX_INT_DISABLE)
    {
        printf("  ***** SYSTEM ERROR ***** test returned with interrupts disabled!\n");
        test_control_system_errors++;
    }

    tx_thread_resume(&test_control_thread);
}
