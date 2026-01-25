/* Direct SIL regression test for samples/demo_device_hid_mouse_rtos.c */

#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"

/* Ensure device-only demo compiles within the test. */
#define UX_DEVICE_SIDE_ONLY


static UX_HOST_CLASS_HID_MOUSE *mouse;

static VOID error_callback(UINT system_level, UINT system_context, UINT error_code)
{
    printf("Error: level %u, context %u, code 0x%x\n", system_level, system_context, error_code);
    test_control_return(1);
}

static UINT sleep_break_on_removed(VOID)
{
    UINT status;
    UX_HOST_CLASS *class;
    status = ux_host_stack_class_get(_ux_system_host_class_hid_name, &class);
    if (status == UX_SUCCESS)
    {
        UX_HOST_CLASS_HID *hid;
        status = ux_host_stack_class_instance_get(class, 0, (void **)&hid);
        if (status != UX_SUCCESS)
            return UX_TRUE;
    }
    return UX_FALSE;
}

static void tx_demo_thread_host_simulation_entry(ULONG arg)
{
    UX_PARAMETER_NOT_USED(arg);
    UINT status;
    SLONG prev_x = 0, prev_y = 0, prev_wheel = 0;
    SLONG cur_x, cur_y, cur_wheel;
    ULONG cur_buttons;

    /* Find HID mouse instance. */
    status = demo_class_hid_mouse_get();
    if (status != UX_SUCCESS) { printf("HID mouse get fail\n"); test_control_return(1); }

    mouse = (UX_HOST_CLASS_HID_MOUSE *)hid_client->ux_host_class_hid_client_local_instance;

    /* Read initial values. */
    ux_host_class_hid_mouse_buttons_get(mouse, &cur_buttons);
    ux_host_class_hid_mouse_position_get(mouse, &prev_x, &prev_y);
    ux_host_class_hid_mouse_wheel_get(mouse, &prev_wheel);

    /* Validate a sequence of demo-generated deltas (step size 3). */
    const INT checks = 24; /* quick but representative */
    const INT step = 3;
    for (INT i = 0; i < checks; i++)
    {
        tx_thread_sleep(10);
        ux_host_class_hid_mouse_buttons_get(mouse, &cur_buttons);
        ux_host_class_hid_mouse_position_get(mouse, &cur_x, &cur_y);
        ux_host_class_hid_mouse_wheel_get(mouse, &cur_wheel);

        SLONG dx = cur_x - prev_x;
        SLONG dy = cur_y - prev_y;

        /* Expect movement of exactly +/-3 on a single axis, buttons remain 0, wheel 0. */
        UX_TEST_ASSERT(cur_buttons == 0);
        UX_TEST_ASSERT(cur_wheel == 0);
        UX_TEST_ASSERT(((dx == step || dx == -step) && dy == 0) || (dx == 0 && (dy == step || dy == -step)));

        prev_x = cur_x;
        prev_y = cur_y;
        prev_wheel = cur_wheel;
    }

    /* Disconnect and cleanup. */
    ux_test_hcd_sim_host_disconnect();
    ux_test_breakable_sleep(50, sleep_break_on_removed);
    _ux_device_stack_disconnect();
    ux_device_stack_class_unregister(_ux_system_slave_class_hid_name, ux_device_class_hid_entry);
    _ux_device_stack_uninitialize();
    _ux_system_uninitialize();

    printf("SUCCESS!\n");
    test_control_return(0);
}

#ifdef CTEST
void test_application_define(void *first_unused_memory)
#else
void usbx_hid_mouse_demo_device_rtos_test_application_define(void *first_unused_memory)
#endif
{
    UX_PARAMETER_NOT_USED(first_unused_memory);

    UINT status;
    CHAR *stack_pointer;
    CHAR *memory_pointer;

    printf("Running HID Mouse Demo (direct) SIL Test............................ ");

    /* The demo initializes USBX system memory internally; we only set up host here. */

    /* Register the error callback early. */
    _ux_utility_error_callback_register(error_callback);

    /* Initialize device side using the demo's API (creates its HID thread). */
    status = ux_demo_device_hid_init();
    if (status != UX_SUCCESS) { printf("Demo device init fail\n"); test_control_return(1); }

    /* Initialize host side. */
    status = ux_host_stack_initialize(UX_NULL);
    if (status != UX_SUCCESS) { printf("Host init fail\n"); test_control_return(1); }

    status = ux_host_stack_class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);
    if (status != UX_SUCCESS) { printf("Host HID reg fail\n"); test_control_return(1); }

    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name, ux_host_class_hid_mouse_entry);
    if (status != UX_SUCCESS) { printf("Host mouse client reg fail\n"); test_control_return(1); }

    /* Use simulated HCD/ DCD. Demo uses ux_dcd_sim_slave_initialize via EXTERNAL_DCD_INITIALIZE. */
    status = ux_host_stack_hcd_register(_ux_system_host_hcd_simulator_name, ux_hcd_sim_host_initialize, 0, 0);
    if (status != UX_SUCCESS) { printf("HCD sim reg fail\n"); test_control_return(1); }

    /* Create host validation thread; memory for ThreadX stacks comes from test-common buffer. */
    stack_pointer = (CHAR *)usbx_memory;

    status = tx_thread_create(&tx_demo_thread_host_simulation, "host", tx_demo_thread_host_simulation_entry, 0,
                              stack_pointer, 1024, 20, 20, 1, TX_AUTO_START);
    if (status != TX_SUCCESS) { printf("Host thread fail\n"); test_control_return(1); }
}
