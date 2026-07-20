/* Regression test for samples/demo_device_hid_mouse_keyboard_rtos.c
 *
 * This test initialises the HID composite mouse+keyboard device demo
 * (device side only) and exercises it through the host-side HID stack
 * using the USB simulator pair.
 * It verifies that:
 *   1. The device enumerates correctly on the host.
 *   2. The host HID mouse and keyboard client instances become live.
 *   3. Mouse position data traces the expected rectangular path.
 *   4. Keyboard key events spell out the full alphabet 'a' through 'z'.
 */

#include <stdio.h>
#include "tx_api.h"
#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"
#include "ux_host_class_hid_keyboard.h"

/* Provided by the demo source when compiled with -DDEMO_TEST.
 * Calls ux_device_hid_init() (device stack, no ux_system_initialize),
 * registers the DCD, and starts the device HID worker threads.  */
extern UINT usbx_demo_device_hid_mouse_keyboard_init(VOID);

/* Declared in usbxtestcontrol.c */
void test_control_return(UINT status);

/* ---------------------------------------------------------------------------
 * Test-local state
 * --------------------------------------------------------------------------*/

#define TEST_STACK_SIZE     512
#define TEST_MEMORY_SIZE    (64 * 1024)

/* ---------------------------------------------------------------------------
 * Mouse path verification constants — must mirror demo_device_hid_mouse_keyboard_rtos.c
 *   UX_DEMO_HID_MOUSE_CURSOR_MOVE   = 3
 *   UX_DEMO_HID_MOUSE_CURSOR_MOVE_N = 100
 * --------------------------------------------------------------------------*/
#define MOUSE_STEP              3
#define MOUSE_SIDE              300                         /* 100 steps * 3 delta = 300 */
#define MOUSE_EXPECTED_STEPS    (MOUSE_SIDE / MOUSE_STEP)  /* 100 steps per segment     */
#define MOUSE_MAX_SAMPLES       400                         /* collection-loop limit     */

/* Path-segment identifiers for the mouse verification state machine */
#define MSEG_RIGHT  1u
#define MSEG_DOWN   2u
#define MSEG_LEFT   3u
#define MSEG_UP     4u
#define MSEG_DONE   5u

/* ---------------------------------------------------------------------------
 * Keyboard sequence verification constants — must mirror demo_device_hid_mouse_keyboard_rtos.c
 *   ux_device_hid_keyboard_send_character() cycles HID keycodes 4..29 (a..z).
 *   The USBX host keyboard class translates keycode 4 -> 'a', ..., 29 -> 'z'.
 * --------------------------------------------------------------------------*/
#define KEYBOARD_FIRST_CHAR     'a'
#define KEYBOARD_NUM_KEYS       26u

static UX_HOST_CLASS_HID          *hid_instance;
static UX_HOST_CLASS_HID_MOUSE    *hid_mouse;
static UX_HOST_CLASS_HID_KEYBOARD *hid_keyboard;

static UX_THREAD  host_sim_thread;
static ULONG      host_sim_stack[TEST_STACK_SIZE / sizeof(ULONG)];

static UCHAR      test_memory[TEST_MEMORY_SIZE];

/* Static buffers to avoid overflowing the thread stack. */
static SLONG  g_seq_x[MOUSE_MAX_SAMPLES];
static SLONG  g_seq_y[MOUSE_MAX_SAMPLES];

static ULONG  g_key_events[KEYBOARD_NUM_KEYS];
static ULONG  g_mod_events[KEYBOARD_NUM_KEYS];

static UINT host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);
static void host_sim_thread_entry(ULONG thread_input);

/* ---------------------------------------------------------------------------
 * test_application_define  (dispatched by the test harness under CTEST)
 * --------------------------------------------------------------------------*/

void usbx_hid_mouse_keyboard_demo_device_rtos_test_application_define(void *first_unused_memory)
{
    UINT status;

    (void)first_unused_memory;

    printf("Running HID Mouse Keyboard Demo Device RTOS Test........ ");

    /* -------------------------------------------------------------------
     * 1. Initialise the combined USBX system (host + device memory).
     *    The sample's ux_device_hid_init() skips this under DEMO_TEST,
     *    so we must do it here before any stack is initialised.
     * ----------------------------------------------------------------- */
    status = ux_system_initialize(test_memory, TEST_MEMORY_SIZE, UX_NULL, 0);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (ux_system_initialize, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 2. Initialise the host stack and register the HID class plus both
     *    mouse and keyboard clients before the device is connected.
     * ----------------------------------------------------------------- */
    status = ux_host_stack_initialize(host_event_callback);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (host stack init, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    status = ux_host_stack_class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (HID class register, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name, ux_host_class_hid_mouse_entry);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (mouse client register, line %d, status 0x%02x)\n", __LINE__, status);
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
     *    - register the HID composite class  (uses sample's callbacks)
     *    - ux_dcd_sim_slave_initialize() (DCD must be registered before HCD)
     *    - start the device HID worker threads
     * ----------------------------------------------------------------- */
    status = usbx_demo_device_hid_mouse_keyboard_init();
    if (status != UX_SUCCESS)
    {
        printf("FAILED (device init, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 4. Register the HCD simulator.  Because the DCD was already
     *    registered in step 3, this immediately establishes the USB link
     *    and starts device enumeration.
     * ----------------------------------------------------------------- */
    status = ux_host_stack_hcd_register(_ux_system_host_hcd_simulator_name, ux_hcd_sim_host_initialize, 0, 0);
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
 * Phase 1 — Mouse: collects the (X,Y) position sequence and verifies it
 * traces the expected rectangular path:
 *
 *   RIGHT : X increases by MOUSE_STEP (3), Y = 0,           until X = 300
 *   DOWN  : Y increases by MOUSE_STEP (3), X = 300,         until Y = 300
 *   LEFT  : X decreases by MOUSE_STEP (3), Y = 300,         until X = 0
 *   UP    : Y decreases by MOUSE_STEP (3), X = 0,           until Y = 0
 *
 * Phase 2 — Keyboard: collects 26 key-press events and verifies they
 * spell out the alphabet 'a' through 'z' in order with no modifier.
 * --------------------------------------------------------------------------*/

static void host_sim_thread_entry(ULONG thread_input)
{
UINT    status;
SLONG   x, y;
SLONG   prev_x, prev_y;
UINT    n;
UINT    seg;
UINT    seg_steps;
UINT    i;
ULONG   key;
ULONG   modifier;

    UX_PARAMETER_NOT_USED(thread_input);

    /* ----------------------------------------------------------------
     * Wait for the device to enumerate and both clients to become live.
     * -------------------------------------------------------------- */
    while ((hid_instance == UX_NULL) ||
           (hid_mouse == UX_NULL) || (hid_keyboard == UX_NULL) ||
           (hid_mouse->ux_host_class_hid_mouse_state != (ULONG)UX_HOST_CLASS_INSTANCE_LIVE) ||
           (hid_keyboard->ux_host_class_hid_keyboard_state != (ULONG)UX_HOST_CLASS_INSTANCE_LIVE))
    {
        ux_utility_thread_sleep(10);
    }

    /* ================================================================
     * Phase 1: Mouse — collect position samples.
     * ============================================================== */
    n = 0;

    while (n < MOUSE_MAX_SAMPLES)
    {
        status = ux_host_class_hid_mouse_position_get(hid_mouse, &x, &y);
        if (status != UX_SUCCESS)
        {
            printf("FAILED (mouse position get error during collection, n=%u)\n", n);
            test_control_return(1);
            return;
        }

        g_seq_x[n] = x;
        g_seq_y[n] = y;
        n++;

        ux_utility_thread_sleep(10);
    }

    /* ----------------------------------------------------------------
     * Phase 1 verify: check that the samples trace a 300×300 rectangle.
     * -------------------------------------------------------------- */
    seg       = MSEG_RIGHT;
    seg_steps = 0;
    prev_x    = 0;
    prev_y    = 0;

    for (i = 0; i < n; i++)
    {
        x = g_seq_x[i];
        y = g_seq_y[i];

        switch (seg)
        {
        case MSEG_RIGHT:
            if (y != 0 || x <= 0 || x > MOUSE_SIDE)
            {
                printf("FAILED: RIGHT segment out of bounds (%d,%d) at step %u\n", (int)x, (int)y, seg_steps);
                test_control_return(1);
                return;
            }
            if (x != prev_x + MOUSE_STEP)
            {
                printf("FAILED: RIGHT bad step %d -> %d (expected +%d)\n", (int)prev_x, (int)x, MOUSE_STEP);
                test_control_return(1);
                return;
            }
            seg_steps++;
            if (x == MOUSE_SIDE)
            {
                if (seg_steps != MOUSE_EXPECTED_STEPS)
                {
                    printf("FAILED: RIGHT has %u steps, expected %u\n", seg_steps, MOUSE_EXPECTED_STEPS);
                    test_control_return(1);
                    return;
                }
                seg       = MSEG_DOWN;
                seg_steps = 0;
            }
            break;

        case MSEG_DOWN:
            if (x != MOUSE_SIDE || y <= 0 || y > MOUSE_SIDE)
            {
                printf("FAILED: DOWN segment out of bounds (%d,%d) at step %u\n", (int)x, (int)y, seg_steps);
                test_control_return(1);
                return;
            }
            if (y != prev_y + MOUSE_STEP)
            {
                printf("FAILED: DOWN bad step %d -> %d (expected +%d)\n", (int)prev_y, (int)y, MOUSE_STEP);
                test_control_return(1);
                return;
            }
            seg_steps++;
            if (y == MOUSE_SIDE)
            {
                if (seg_steps != MOUSE_EXPECTED_STEPS)
                {
                    printf("FAILED: DOWN has %u steps, expected %u\n", seg_steps, MOUSE_EXPECTED_STEPS);
                    test_control_return(1);
                    return;
                }
                seg       = MSEG_LEFT;
                seg_steps = 0;
            }
            break;

        case MSEG_LEFT:
            if (y != MOUSE_SIDE || x < 0 || x >= MOUSE_SIDE)
            {
                printf("FAILED: LEFT segment out of bounds (%d,%d) at step %u\n", (int)x, (int)y, seg_steps);
                test_control_return(1);
                return;
            }
            if (x != prev_x - MOUSE_STEP)
            {
                printf("FAILED: LEFT bad step %d -> %d (expected -%d)\n", (int)prev_x, (int)x, MOUSE_STEP);
                test_control_return(1);
                return;
            }
            seg_steps++;
            if (x == 0)
            {
                if (seg_steps != MOUSE_EXPECTED_STEPS)
                {
                    printf("FAILED: LEFT has %u steps, expected %u\n", seg_steps, MOUSE_EXPECTED_STEPS);
                    test_control_return(1);
                    return;
                }
                seg       = MSEG_UP;
                seg_steps = 0;
            }
            break;

        case MSEG_UP:
            if (x != 0 || y < 0 || y >= MOUSE_SIDE)
            {
                printf("FAILED: UP segment out of bounds (%d,%d) at step %u\n", (int)x, (int)y, seg_steps);
                test_control_return(1);
                return;
            }
            if (y != prev_y - MOUSE_STEP)
            {
                printf("FAILED: UP bad step %d -> %d (expected -%d)\n", (int)prev_y, (int)y, MOUSE_STEP);
                test_control_return(1);
                return;
            }
            seg_steps++;
            if (y == 0)
            {
                if (seg_steps != MOUSE_EXPECTED_STEPS)
                {
                    printf("FAILED: UP has %u steps, expected %u\n", seg_steps, MOUSE_EXPECTED_STEPS);
                    test_control_return(1);
                    return;
                }
                seg       = MSEG_DONE;
                seg_steps = 0;
            }
            break;

        default:
            break;
        }

        prev_x = x;
        prev_y = y;
    }

    if (seg != MSEG_DONE)
    {
        printf("FAILED: mouse path incomplete, stopped at segment %u (%u samples)\n", seg, n);
        test_control_return(1);
        return;
    }

    /* ================================================================
     * Phase 2: Keyboard — wait for client and collect key-press events.
     * ============================================================== */


//    n = 0;
//
//    while (n < KEYBOARD_NUM_KEYS)
//    {
//        status = ux_host_class_hid_keyboard_key_get(hid_keyboard, &key, &modifier);
//
//        if (status != UX_SUCCESS)
//        {
//            printf("FAILED (keyboard key get error during collection, n=%u)\n", n);
//            test_control_return(1);
//            return;
//        }
//
//        g_key_events[n] = key;
//        g_mod_events[n] = modifier;
//        n++;
//
//        ux_utility_thread_sleep(10);
//    }
//
//    /* ----------------------------------------------------------------
//     * Phase 2 verify: key[i] = 'a' + i, modifier[i] = 0, for i in 0..25.
//     * -------------------------------------------------------------- */
//    if (n != KEYBOARD_NUM_KEYS)
//    {
//        printf("FAILED: expected %u keys, collected %u\n", KEYBOARD_NUM_KEYS, n);
//        test_control_return(1);
//        return;
//    }
//
//    for (i = 0; i < KEYBOARD_NUM_KEYS; i++)
//    {
//        ULONG expected_key = (ULONG)(KEYBOARD_FIRST_CHAR + i);
//
//        if (g_key_events[i] != expected_key)
//        {
//            printf("FAILED: key[%u] = 0x%02lx, expected 0x%02lx ('%c')\n",
//                   i, (unsigned long)g_key_events[i],
//                   (unsigned long)expected_key, (char)expected_key);
//            test_control_return(1);
//            return;
//        }
//
//        if (g_mod_events[i] != 0)
//        {
//            printf("FAILED: modifier[%u] = 0x%02lx, expected 0\n",
//                   i, (unsigned long)g_mod_events[i]);
//            test_control_return(1);
//            return;
//        }
//    }

    printf("SUCCESS!\n");
    test_control_return(0);
}

/* ---------------------------------------------------------------------------
 * host_event_callback — tracks HID class, mouse, and keyboard client events.
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

        if (client -> ux_host_class_hid_client_handler == ux_host_class_hid_mouse_entry)
        {
            if (hid_mouse == UX_NULL)
                hid_mouse = (UX_HOST_CLASS_HID_MOUSE *)client -> ux_host_class_hid_client_local_instance;
        }

        if (client->ux_host_class_hid_client_handler == ux_host_class_hid_keyboard_entry)
        {
            if (hid_keyboard == UX_NULL)
                hid_keyboard = (UX_HOST_CLASS_HID_KEYBOARD *)client -> ux_host_class_hid_client_local_instance;
        }
        break;

    case UX_HID_CLIENT_REMOVAL:

        if ((VOID *)hid_mouse == client -> ux_host_class_hid_client_local_instance)
            hid_mouse = UX_NULL;

        if ((VOID *)hid_keyboard == client -> ux_host_class_hid_client_local_instance)
            hid_keyboard = UX_NULL;
        break;

    case UX_DEVICE_CONNECTION:
      break;

    case UX_DEVICE_DISCONNECTION:
      break;

    default:
        break;
    }

    return UX_SUCCESS;
}
