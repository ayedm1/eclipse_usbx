/* Direct SIL regression test for samples/demo_device_hid_mouse_rtos.c */
#include <stdio.h>
#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"

void  test_control_return(UINT status);
extern UINT ux_demo_device_hid_init(VOID);
/* Ensure device-only demo compiles within the test. */


void usbx_hid_mouse_demo_device_rtos_test_application_define(void *first_unused_memory)
{
    UX_PARAMETER_NOT_USED(first_unused_memory);

    UINT status;

    printf("Running HID Mouse Demo (direct) SIL Test............................ ");


    /* Initialize device side using the demo's API (creates its HID thread). */
    status = ux_demo_device_hid_init();
    
    if (status != UX_SUCCESS) 
    { 
        printf("Demo device init fail\n"); 
        test_control_return(1); 
    }

    /* Initialize host side. */
    status = ux_host_stack_initialize(UX_NULL);

    if (status != UX_SUCCESS)
    { 
        printf("Host init fail\n");
        test_control_return(1); 
    }

    status = ux_host_stack_class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);

    if (status != UX_SUCCESS)
    {
        printf("Host HID reg fail\n"); 
        test_control_return(1); 
    }

    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name, ux_host_class_hid_mouse_entry);

    if (status != UX_SUCCESS) 
    { 
        printf("Host mouse client reg fail\n"); 
        test_control_return(1); 
    }
}
