/* Direct SIL regression test for samples/demo_device_hid_mouse_rtos.c */
#include <stdio.h>
#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"

/* Ensure device-only demo compiles within the test. */


void usbx_hid_mouse_demo_device_rtos_test_application_define(void *first_unused_memory)
{
    UX_PARAMETER_NOT_USED(first_unused_memory);

    printf("Running HID Mouse Demo (direct) SIL Test............................ ");

}
