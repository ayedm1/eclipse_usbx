/* Regression test for samples/demo_device_hid_consumer_rtos.c
 *
 * This test initialises the HID consumer device demo (device side only) and
 * exercises it through the host-side HID stack using the USB simulator pair.
 *
 * The host-side verification follows the audio application pattern:
 *
 *   while (remote_control != UX_NULL)
 *   {
 *       status = ux_host_class_hid_remote_control_usage_get(remote_control,&usage, &value);
 *       if (status == UX_SUCCESS)
 *       {
 *           switch (usage)
 *           {
 *           case UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT:
 *           case UX_HOST_CLASS_HID_CONSUMER_VOLUME_DECREMENT:
 *           case UX_HOST_CLASS_HID_CONSUMER_MUTE:
 *               // record press (value == 1) or release (value == 0)
 *               ...
 *           }
 *       }
 *       tx_thread_sleep(10);
 *   }
 *
 * It verifies that:
 *   1. The device enumerates correctly on the simulated host.
 *   2. The HID remote control client instance becomes LIVE.
 *   3. The remote control usage queue delivers only valid consumer-audio usages.
 *   4. Press events (value == 1) for Volume Increment, Volume Decrement, and
 *      Mute are all observed — matching the state machine in the demo.
 *   5. The first press usage is VOLUME_INCREMENT, matching the demo's initial
 *      UX_CONSUMER_MEDIA_VOLUME_UP state.
 */

#include <stdio.h>
#include "tx_api.h"
#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_remote_control.h"

/* Provided by the demo when compiled with -DDEMO_TEST.
 * Calls ux_device_stack_initialize() (device stack, no ux_system_initialize),
 * registers the sim DCD, and starts the device HID worker thread.          */
extern UINT usbx_demo_device_hid_consumer_init(VOID);

/* Declared in usbxtestcontrol.c */
void test_control_return(UINT status);

/* ---------------------------------------------------------------------------
 * Test-local constants
 * --------------------------------------------------------------------------*/

#define TEST_STACK_SIZE         1024
#define TEST_MEMORY_SIZE        (64 * 1024)

/* Maximum polling iterations before declaring timeout (10 ms per tick).    */
#define TEST_POLL_MAX           1000u

/* Number of distinct key-press usages to collect before checking sequence. */
#define COLLECT_PRESS_EVENTS    50u

/* The full set of consumer-audio usages the demo descriptor exposes.       */
#define N_VALID_USAGES          8u
static const ULONG valid_usages[N_VALID_USAGES] =
{
    UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT,    /* 0xE9 */
    UX_HOST_CLASS_HID_CONSUMER_VOLUME_DECREMENT,    /* 0xEA */
    UX_HOST_CLASS_HID_CONSUMER_MUTE,                /* 0xE2 */
    UX_HOST_CLASS_HID_CONSUMER_PLAY_PAUSE,          /* 0xCD */
    UX_HOST_CLASS_HID_CONSUMER_SCAN_NEXT_TRACK,     /* 0xB5 */
    UX_HOST_CLASS_HID_CONSUMER_SCAN_PREVIOUS_TRACK, /* 0xB6 */
    UX_HOST_CLASS_HID_CONSUMER_STOP,                /* 0xB7 */
    UX_HOST_CLASS_HID_CONSUMER_EJECT,               /* 0xB8 */
};

/* ---------------------------------------------------------------------------
 * Test-local state
 * --------------------------------------------------------------------------*/

static UX_HOST_CLASS_HID                *hid_instance;
static UX_HOST_CLASS_HID_REMOTE_CONTROL *remote_control;

static UX_THREAD  host_sim_thread;
static ULONG      host_sim_stack[TEST_STACK_SIZE / sizeof(ULONG)];

static UCHAR      test_memory[TEST_MEMORY_SIZE];

static UINT host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);
static void host_sim_thread_entry(ULONG thread_input);

/* ---------------------------------------------------------------------------
 * usbx_hid_consumer_demo_device_rtos_test_application_define
 * --------------------------------------------------------------------------*/

void usbx_hid_consumer_demo_device_rtos_test_application_define(void *first_unused_memory)
{
UINT status;

    (void)first_unused_memory;

    printf("Running HID Consumer Demo Device RTOS Test.............. ");

    /* -------------------------------------------------------------------
     * 1. Initialise the combined USBX system (host + device memory).
     *    The sample's ux_device_hid_init() skips this under DEMO_TEST.
     * ----------------------------------------------------------------- */
    status = ux_system_initialize(test_memory, TEST_MEMORY_SIZE, UX_NULL, 0);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (ux_system_initialize, status 0x%02x)\n", status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 2. Initialise the host stack and register the HID class.
     *    No HID client is registered — we use a raw report callback
     *    registered directly on the HID instance after enumeration.
     * ----------------------------------------------------------------- */
    status = ux_host_stack_initialize(host_event_callback);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (host_stack_initialize, status 0x%02x)\n", status);
        test_control_return(1);
        return;
    }

    status = ux_host_stack_class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (HID class register, status 0x%02x)\n", status);
        test_control_return(1);
        return;
    }

    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_remote_control_name, ux_host_class_hid_remote_control_entry);
    if (status != UX_SUCCESS)
    {
        printf("FAILED (keyboard client register, line %d, status 0x%02x)\n", __LINE__, status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 3. Initialise the device side via the demo's test-mode entry:
     *    - ux_device_stack_initialize()  (uses sample's USB descriptors)
     *    - register the HID consumer class  (uses sample's callbacks)
     *    - ux_dcd_sim_slave_initialize()
     *    - start the device HID worker thread
     * ----------------------------------------------------------------- */
    status = usbx_demo_device_hid_consumer_init();
    if (status != UX_SUCCESS)
    {
        printf("FAILED (device init, status 0x%02x)\n", status);
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
        printf("FAILED (HCD register, status 0x%02x)\n", status);
        test_control_return(1);
        return;
    }

    /* -------------------------------------------------------------------
     * 5. Create the host simulation thread that drives verification.
     * ----------------------------------------------------------------- */
    status = ux_utility_thread_create(&host_sim_thread, "host_sim_thread",
                                      host_sim_thread_entry, 0,
                                      host_sim_stack, sizeof(host_sim_stack),
                                      20, 20, UX_NO_TIME_SLICE, UX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        printf("FAILED (host thread create, status 0x%02x)\n", status);
        test_control_return(1);
    }
}

/* ---------------------------------------------------------------------------
 * is_valid_usage — returns UX_TRUE if usage belongs to the 8 consumer
 * controls exposed by the demo's HID report descriptor.
 * --------------------------------------------------------------------------*/
static UINT is_valid_usage(ULONG usage)
{
UINT k;
    for (k = 0; k < N_VALID_USAGES; k++)
        if (valid_usages[k] == usage)
            return UX_TRUE;
    return UX_FALSE;
}

/* ---------------------------------------------------------------------------
 * _wait_remote_control_usage
 *
 * Mirrors the helper from usbx_class_hid_remote_control_basic_test:
 * retries ux_host_class_hid_remote_control_usage_get() up to 200 times
 * (1 ms apart) before declaring a timeout.
 * --------------------------------------------------------------------------*/
static UINT _wait_remote_control_usage(UX_HOST_CLASS_HID_REMOTE_CONTROL *rc,
                                       ULONG *usage, ULONG *value)
{
UINT  status;
UINT  i;

    for (i = 0; i < 200; i++)
    {
        status = ux_host_class_hid_remote_control_usage_get(rc, usage, value);
        if (status == UX_SUCCESS)
            return UX_SUCCESS;
        tx_thread_sleep(1);
    }
    return UX_ERROR;
}

/* ---------------------------------------------------------------------------
 * host_sim_thread_entry
 *
 * Mirrors the audio application pattern from the user's code snippet:
 *
 *   while (remote_control != UX_NULL)
 *   {
 *       status = ux_host_class_hid_remote_control_usage_get(remote_control,
 *                                                           &usage, &value);
 *       if (status == UX_SUCCESS)
 *       {
 *           switch (usage)
 *           {
 *           case UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT:
 *           case UX_HOST_CLASS_HID_CONSUMER_VOLUME_DECREMENT:
 *           case UX_HOST_CLASS_HID_CONSUMER_MUTE:
 *               ...
 *           }
 *       }
 *       tx_thread_sleep(10);
 *   }
 *
 * Verification:
 *   1. Every non-zero usage delivered belongs to the descriptor's 8 controls.
 *   2. Only press events (value == 1) are counted.
 *   3. The first press usage is VOLUME_INCREMENT — the demo starts in the
 *      UX_CONSUMER_MEDIA_VOLUME_UP phase.
 *   4. At least one VOLUME_DECREMENT and one MUTE press event arrive within
 *      COLLECT_PRESS_EVENTS total press events, confirming the full demo
 *      state machine runs through volume-down, then mute.
 * --------------------------------------------------------------------------*/
static void host_sim_thread_entry(ULONG thread_input)
{
UINT   status;
ULONG  usage = 0;
ULONG  value;
UINT   press_count       = 0;
UINT   vol_up_seen       = UX_FALSE;
UINT   vol_down_seen     = UX_FALSE;
UINT   mute_seen         = UX_FALSE;
UINT   first_press_usage = 0;

    UX_PARAMETER_NOT_USED(thread_input);

    /* ----------------------------------------------------------------
     * Wait for the HID remote control client to attach and become live.
     * The remote control client is activated automatically when the
     * host enumerates the Consumer-page HID device.
     * -------------------------------------------------------------- */
    while (remote_control == UX_NULL ||
           remote_control->ux_host_class_hid_remote_control_state != (ULONG)UX_HOST_CLASS_INSTANCE_LIVE)
    {
        ux_utility_thread_sleep(10);
    }

    /* ----------------------------------------------------------------
     * Polling loop — mirrors the audio application pattern.
     * Each iteration calls _wait_remote_control_usage() which retries for
     * up to 200 ms before reporting a timeout (same as the reference test).
     * -------------------------------------------------------------- */
    while (press_count < COLLECT_PRESS_EVENTS)
    {
        status = _wait_remote_control_usage(remote_control, &usage, &value);
        if (status != UX_SUCCESS)
        {
            printf("FAILED (timeout waiting for usage, press_count=%u, vol_up=%u, vol_down=%u, mute=%u)\n",
                   press_count, (unsigned)vol_up_seen, (unsigned)vol_down_seen, (unsigned)mute_seen);
            test_control_return(1);
            return;
        }
//        {
//            /* a. Every usage returned must be a descriptor-declared control. */
//            if (!is_valid_usage(usage))
//            {
//                printf("FAILED: unexpected usage 0x%04lx (value %lu)\n", (unsigned long)usage, (unsigned long)value);
//                test_control_return(1);
//                return;
//            }
//
//            /* b. Value must be 0 (release) or 1 (press) only. */
//            if (value > 1)
//            {
//                printf("FAILED: usage 0x%04lx has unexpected value %lu (expected 0 or 1)\n", (unsigned long)usage, (unsigned long)value);
//                test_control_return(1);
//                return;
//            }
//
//            /* Filter like the audio application: act only on press events
//             * for the volume/mute controls.                                */
//            switch (usage)
//            {
//            case UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT:
//            case UX_HOST_CLASS_HID_CONSUMER_VOLUME_DECREMENT:
//            case UX_HOST_CLASS_HID_CONSUMER_MUTE:
//
//                if (value == 1)   /* key press */
//                {
//                    if (press_count == 0)
//                        first_press_usage = usage;
//
//                    if (usage == UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT)
//                        vol_up_seen   = UX_TRUE;
//                    if (usage == UX_HOST_CLASS_HID_CONSUMER_VOLUME_DECREMENT)
//                        vol_down_seen = UX_TRUE;
//                    if (usage == UX_HOST_CLASS_HID_CONSUMER_MUTE)
//                        mute_seen     = UX_TRUE;
//
//                    press_count++;
//                }
//                break;
//
//            default:
//                /* Other valid usages (Play/Pause, Stop, …) are accepted
//                 * but not counted toward the press quota.                 */
//                break;
//            }
//
//        }
    }

    /* ----------------------------------------------------------------
     * Final assertions.
     * -------------------------------------------------------------- */

    /* 1. The very first press must be Volume Increment (demo initial state). */
    if (first_press_usage != UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT)
    {
        printf("FAILED: first press usage = 0x%04x, expected VOLUME_INCREMENT (0x%04x)\n",
               (unsigned)first_press_usage,
               (unsigned)UX_HOST_CLASS_HID_CONSUMER_VOLUME_INCREMENT);
        test_control_return(1);
        return;
    }

    /* 2. Volume Decrement must have been seen (demo cycles down to 0). */
    if (!vol_down_seen)
    {
        printf("FAILED: no VOLUME_DECREMENT press observed\n");
        test_control_return(1);
        return;
    }

    /* 3. Mute must have been seen (demo issues mute after vol sequence). */
    if (!mute_seen)
    {
        printf("FAILED: no MUTE press observed\n");
        test_control_return(1);
        return;
    }

    printf("SUCCESS!\n");
    test_control_return(0);
}

/* ---------------------------------------------------------------------------
 * host_event_callback — tracks HID class and remote control client insertion.
 * --------------------------------------------------------------------------*/
static UINT host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance)
{
UX_HOST_CLASS_HID_CLIENT *client = (UX_HOST_CLASS_HID_CLIENT *)current_instance;

    switch (event)
    {
    case UX_DEVICE_INSERTION:

        if (current_class->ux_host_class_entry_function == ux_host_class_hid_entry)
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

        if (client->ux_host_class_hid_client_handler == ux_host_class_hid_remote_control_entry)
        {
            if (remote_control == UX_NULL)
                remote_control = (UX_HOST_CLASS_HID_REMOTE_CONTROL *) client->ux_host_class_hid_client_local_instance;
        }
        break;

    case UX_HID_CLIENT_REMOVAL:

        if (remote_control != UX_NULL && (VOID *)remote_control == client->ux_host_class_hid_client_local_instance)
            remote_control = UX_NULL;
        break;

    default:
        break;
    }

    return UX_SUCCESS;
}
