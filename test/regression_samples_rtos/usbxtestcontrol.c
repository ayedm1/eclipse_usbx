/* This is the test control routine of the USBX kernel.  All tests are dispatched from this routine.  */

#include "tx_api.h"
#include "ux_api.h"
#include <stdio.h>

#define TEST_STACK_SIZE         6144

/* Define the test control USBX objects...  */

UX_THREAD               test_control_thread;
UX_THREAD               test_thread;


/* Define the test control global variables.   */

ULONG           test_control_return_status;
ULONG           test_control_successful_tests;
ULONG           test_control_failed_tests;
ULONG           test_control_system_errors;



/* Remember the start of free memory.  */

UCHAR           *test_free_memory_ptr;


/* Define the function pointer for ISR dispatch.  */

VOID            (*test_isr_dispatch)(void);


UCHAR           test_control_thread_stack[TEST_STACK_SIZE];
UCHAR           tests_memory[1024*1024*1024];

/* Define the external reference for the preempt disable flag.  */

extern volatile UINT   _tx_thread_preempt_disable;
extern volatile ULONG  _tx_thread_system_state;


/* Define test entry pointer type.  */

typedef  struct TEST_ENTRY_STRUCT
{

VOID        (*test_entry)(void *);
} TEST_ENTRY;

/* CTest application define  */

void usbx_hid_mouse_demo_device_rtos_test_application_define(void *first_unused_memory);

/* Define the array of test entry points.  */
TEST_ENTRY  test_control_tests[] =
{
    usbx_hid_mouse_demo_device_rtos_test_application_define,
    UX_NULL,
};

/* Define thread prototypes.  */

void  test_control_thread_entry(ULONG thread_input);
void  test_control_return(UINT status);

/* Define necessary external references.  */

#ifdef __ghs
extern TX_MUTEX                 __ghLockMutex;
#endif

#ifdef EXTERNAL_EXIT
void external_exit(UINT code);
#endif

extern ULONG                    _tx_thread_created_count;

/* Define main entry point.  */
#ifndef EXTERNAL_MAIN
void main()
{

    /* Enter the USBX kernel.  */
    tx_kernel_enter();
}
#endif


/* Define what the initial system looks like.  */

void    tx_application_define(void *first_unused_memory)
{

    /* Initialize the test error/success counters.  */
    test_control_successful_tests =  0;
    test_control_failed_tests =      0;
    test_control_system_errors =     0;

    /* Create the test control thread.  */
    tx_thread_create(&test_control_thread, "test control thread", test_control_thread_entry, 0,
            test_control_thread_stack, TEST_STACK_SIZE,
            17, 15, TX_NO_TIME_SLICE, TX_AUTO_START);

    /* Remember the free memory pointer.  */
    test_free_memory_ptr =  &tests_memory[0];
}



/* Define the test control thread.  This thread is responsible for dispatching all of the
   tests in the USBX test suite.  */

void  test_control_thread_entry(ULONG thread_input)
{

ULONG   previous_test_control_failed_tests;
ULONG   previous_thread_created_count;
UINT    i;

    /* Raise the priority of the control thread to 0.  */
    tx_thread_priority_change(&test_control_thread, 0, &i);

    /* Print out banner.  */
    printf("********************** USBX Validation/Regression Test Suite *********************************\n\n");

    /* Print version id.  */
    printf("Version: %s Data width: x%i\n\n", _ux_version_id, (int)sizeof(void*) * 8);

    /* Print out the tests... */
    printf("Running validation/regression test:\n\n");

    /* Loop to process all tests...  */
    i =  0;
    while (test_control_tests[i].test_entry != TX_NULL)
    {

        /* Save the number of failed tests for comparison. */
        previous_test_control_failed_tests = test_control_failed_tests;

        /* Save previous thread count.  */
        previous_thread_created_count = _tx_thread_created_count;

        /* Dispatch the test.  */
        (test_control_tests[i++].test_entry)(test_free_memory_ptr);


        /* Did the test entry run successfully? */
        if (test_control_failed_tests == previous_test_control_failed_tests &&
            previous_thread_created_count != _tx_thread_created_count)

            /* Suspend control test to allow test to run.  */
            tx_thread_suspend(&test_control_thread);

   }

    /* Finished with all tests, print results and return!  */
    printf("**** Testing Complete ****\n");
    printf("**** Test Summary:  Tests Passed:  %lu   Tests Failed:  %lu    System Errors:   %lu\n",
        test_control_successful_tests, test_control_failed_tests, test_control_system_errors);

}

static VOID disable_test_actions(VOID)
{

}


void  test_control_return(UINT status)
{

UINT    old_posture =  TX_INT_ENABLE;

    disable_test_actions();

    /* Save the status in a global.  */
    test_control_return_status =  status;

    /* Ensure interrupts are enabled.  */
    old_posture =  tx_interrupt_control(TX_INT_ENABLE);

    /* Determine if it was successful or not.  */
    if (status)
        test_control_failed_tests++;
    else
        test_control_successful_tests++;

    /* Now check for system errors.  */

    /* Is preempt disable flag set?  */
    if (_tx_thread_preempt_disable)
    {

        /* System error - preempt disable should never be set inside of a thread!  */
        printf("    ***** SYSTEM ERROR ***** _tx_thread_preempt_disable is non-zero!\n");
        test_control_system_errors++;
    }

    /* Is system state set?  */
    if (_tx_thread_system_state)
    {

        /* System error - system state should never be set inside of a thread!  */
        printf("    ***** SYSTEM ERROR ***** _tx_thread_system_state is non-zero!\n");
        test_control_system_errors++;
    }

    /* Are interrupts disabled?  */
    if (old_posture == TX_INT_DISABLE)
    {

        /* System error - interrupts should always be enabled in our test threads!  */
        printf("    ***** SYSTEM ERROR ***** test returned with interrupts disabled!\n");
        test_control_system_errors++;
    }

    /* Resume the control thread to fully exit the test.  */
    tx_thread_resume(&test_control_thread);
}




