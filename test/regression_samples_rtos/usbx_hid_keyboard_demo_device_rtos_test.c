/* Regression test for samples/demo_device_hid_keyboard_rtos.c
 *
 * This test initialises the HID keyboard device demo (device side only) and
 * exercises it through the host-side HID stack using the USB simulator pair.
 * It verifies that:
 *   1. The device enumerates correctly on the host.
 *   2. The host HID keyboard client instance becomes live.
 *   3. Key events can be read from the device.
 */

#include <stdio.h>
#include "tx_api.h"
#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_keyboard.h"

/* Provided by the demo source when compiled with -DDEMO_TEST.
 * Calls ux_device_hid_init() (device stack, no ux_system_initialize),
 * registers the DCD, and starts the device HID worker thread.  */
extern UINT usbx_demo_device_hid_keyboard_init(VOID);

/* Declared in usbxtestcontrol.c */
void test_control_return(UINT status);

/* ---------------------------------------------------------------------------
 * Test-local state
 * --------------------------------------------------------------------------*/

#define TEST_STACK_SIZE     512
#define TEST_MEMORY_SIZE    (64 * 1024)

/* ---------------------------------------------------------------------------
 * Keyboard sequence verification constants — must mirror demo_device_hid_keyboard_rtos.c
 *   ux_device_hid_keyboard_send_character() cycles HID keycodes 4..29 (a..z),
 *   sending a press report then a release report for each key.
 *   The USBX host keyboard class translates keycode 4 -> 'a', ..., 29 -> 'z'.
 * --------------------------------------------------------------------------*/
#define KEYBOARD_FIRST_CHAR     'a'   /* first ASCII character in the demo stream */
#define KEYBOARD_NUM_KEYS       26u   /* total distinct keys: 'a' through 'z'     */

static UX_HOST_CLASS_HID           *hid_instance;
static UX_HOST_CLASS_HID_KEYBOARD  *hid_keyboard;

static TX_THREAD  host_sim_thread;
static ULONG      host_sim_stack[TEST_STACK_SIZE / sizeof(ULONG)];

static UCHAR      test_memory[TEST_MEMORY_SIZE];

/* Collected key-press events — static to keep them off the thread stack. */
static ULONG  g_key_events[KEYBOARD_NUM_KEYS];
static ULONG  g_mod_events[KEYBOARD_NUM_KEYS];

static UINT host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);
static void host_sim_thread_entry(ULONG thread_input);

/* ---------------------------------------------------------------------------
 * test_application_define  (dispatched by the test harness under CTEST)
 * --------------------------------------------------------------------------*/

void usbx_hid_keyboard_demo_device_rtos_test_application_define(void *first_unused_memory)
{
    UINT status;

    (void)first_unused_memory;

    printf("Running HID Keyboard Demo Device RTOS Test.............. ");

    /* -------------------------------------------------------------------
     * 1. Initialise the combined USBX system (host + device memory).
     *    The sample's ux_device_hid_init() skips this under DEMO_TEST.
     * ----------------------------------------------------------------- */
    status = ux_system_initialize(test_memory, TEST_MEMORY_SIZE, UX_NULL, 0);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (ux_system_initialize, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 2. Initialise the host stack and register the HID class + keyboard
     *    client before the device is connected.
     * ----------------------------------------------------------------- */
    status = ux_host_stack_initialize(host_event_callback);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (host stack init, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    status = ux_host_stack_class_register(_ux_system_host_class_hid_name,
                                          ux_host_class_hid_entry);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (HID class register, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_keyboard_name, ux_host_class_hid_keyboard_entry);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (keyboard client register, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 3. Initialise the device side via the demo's test-mode entry:
     *    - ux_device_stack_initialize()  (uses sample's USB descriptors)
     *    - register the HID keyboard class  (uses sample's callbacks)
     *    - ux_dcd_sim_slave_initialize() (DCD must be registered before HCD)
     *    - start the device HID worker thread
     * ----------------------------------------------------------------- */
    status = usbx_demo_device_hid_keyboard_init();
    if (status != UX_SUCCESS)
    {
        printf("FAILED (device init, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 4. Register the HCD simulator.  Because the DCD was already
     *    registered in step 3, this immediately establishes the USB link.
     * ----------------------------------------------------------------- */
    status = ux_host_stack_hcd_register(_ux_system_host_hcd_simulator_name,
                                        ux_hcd_sim_host_initialize, 0, 0);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (HCD register, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 5. Create the host simulation thread that verifies device behaviour.
     * ----------------------------------------------------------------- */
    status = ux_utility_thread_create(&host_sim_thread, "host sim thread", host_sim_thread_entry, 0,
                                      host_sim_stack, sizeof(host_sim_stack), 20, 20,
                                      UX_NO_TIME_SLICE, UX_AUTO_START);

    if (status != TX_SUCCESS)
    {
        printf("FAILED (host thread create, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
    }
}

/* ---------------------------------------------------------------------------
 * host_sim_thread_entry
 *
 * Collects key-press events sent by the keyboard demo and verifies they
 * spell out the alphabet 'a' through 'z' in order with no modifier.
 *
 * The demo's ux_device_hid_keyboard_send_character() cycles HID keycodes
 * 4..29, sending a press report then a release report for each key.
 * The USBX host keyboard class translates those to ASCII 'a'..'z'.
 * --------------------------------------------------------------------------*/

static void host_sim_thread_entry(ULONG thread_input)
{
UINT    status;
ULONG   key;
ULONG   modifier;
UINT    n;    /* number of press events collected  */
UINT    i;

    UX_PARAMETER_NOT_USED(thread_input);

    /* ----------------------------------------------------------------
     * Wait for the device to enumerate and the keyboard client to become live.
     * -------------------------------------------------------------- */
    while ((hid_instance == UX_NULL) ||
           (hid_keyboard == UX_NULL) ||
           (hid_keyboard->ux_host_class_hid_keyboard_state != (ULONG)UX_HOST_CLASS_INSTANCE_LIVE))
    {
        ux_utility_thread_sleep(10);
    }

    /* ----------------------------------------------------------------
     * Phase 1: collect press events.
     *
     * Poll every TEST_TICK_MS ms.  Only record non-zero key values
     * (key == 0 is a release event).  Stop once KEYBOARD_NUM_KEYS (26)
     * press events have been captured or the timeout expires.
     * -------------------------------------------------------------- */
    n  = 0;

    while (n < KEYBOARD_NUM_KEYS)
    {
        status = ux_host_class_hid_keyboard_key_get(hid_keyboard, &key, &modifier);

        if (status != UX_SUCCESS)
        {
            printf("FAILED (keys get error during collection, n=%u)\n", n);
            test_control_return(1);
            return;
        }

        g_key_events[n] = key;
        g_mod_events[n] = modifier;
        n++;

        ux_utility_thread_sleep(10);
    }

    /* ----------------------------------------------------------------
     * Phase 2: verify the collected sequence.
     *
     * Expected: key[i] = 'a' + i, modifier[i] = 1, for i in 0..25.
     * -------------------------------------------------------------- */
    if (n != KEYBOARD_NUM_KEYS)
    {
        printf("FAILED: expected %u keys, collected %u\n", KEYBOARD_NUM_KEYS, n);
        test_control_return(1);
        return;
    }

    for (i = 0; i < KEYBOARD_NUM_KEYS; i++)
    {
        ULONG expected_key = (ULONG)(KEYBOARD_FIRST_CHAR + i);

        if (g_key_events[i] != expected_key)
        {
            printf("FAILED: key[%u] = 0x%02lx, expected 0x%02lx ('%c')\n", i, (unsigned long)g_key_events[i],
                   (unsigned long)expected_key, (char)expected_key);

            test_control_return(1);
            return;
        }

        if (g_mod_events[i] != 1)
        {
            printf("FAILED: modifier[%u] = 0x%02lx, expected 0\n", i, (unsigned long)g_mod_events[i]);
            test_control_return(1);
            return;
        }
    }

    printf("SUCCESS!\n");
    test_control_return(0);
}

/* ---------------------------------------------------------------------------
 * host_event_callback — tracks HID class and keyboard client insertions.
 * --------------------------------------------------------------------------*/

static UINT host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance)
{
    UX_HOST_CLASS_HID_CLIENT *client = (UX_HOST_CLASS_HID_CLIENT *)current_instance;

    switch (event)
    {
    case UX_DEVICE_INSERTION:

        if (current_class -> ux_host_class_entry_function == ux_host_class_hid_entry)
        {
            if (hid_instance == UX_NULL)
                hid_instance = (UX_HOST_CLASS_HID *)current_instance;
        }
        break;

    case UX_DEVICE_REMOVAL:

        if ((VOID *)hid_instance == current_instance)
            hid_instance = UX_NULL;
        break;

    case UX_HID_CLIENT_INSERTION:

        if (client->ux_host_class_hid_client_handler == ux_host_class_hid_keyboard_entry)
        {
            if (hid_keyboard == UX_NULL)
                hid_keyboard = (UX_HOST_CLASS_HID_KEYBOARD *) client -> ux_host_class_hid_client_local_instance;
        }
        break;

    case UX_HID_CLIENT_REMOVAL:

        if ((VOID *)hid_keyboard == client -> ux_host_class_hid_client_local_instance)
            hid_keyboard = UX_NULL;
        break;

    default:
        break;
    }

    return UX_SUCCESS;
}
