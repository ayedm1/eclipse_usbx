/* Regression test for samples/demo_device_hid_mouse_rtos.c
 *
 * This test initialises the HID mouse device demo (device side only) and
 * exercises it through the host-side HID stack using the USB simulator pair.
 * It verifies that:
 *   1. The device enumerates correctly on the host.
 *   2. The host HID mouse client instance becomes live.
 *   3. Mouse position data can be read from the device.
 */

#include <stdio.h>
#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"

/* Provided by the demo source when compiled with -DDEMO_TEST.
 * Calls ux_device_hid_init() (device stack, no ux_system_initialize),
 * registers the DCD, and starts the device HID worker thread.  */
extern UINT usbx_demo_device_hid_mouse_init(VOID);

/* Declared in usbxtestcontrol.c */
void test_control_return(UINT status);

/* ---------------------------------------------------------------------------
 * Test-local state
 * --------------------------------------------------------------------------*/

#define TEST_STACK_SIZE     512
#define TEST_MEMORY_SIZE    (64 * 1024)

/* ---------------------------------------------------------------------------
 * Square-path verification constants — must mirror demo_device_hid_mouse_rtos.c
 *   UX_DEMO_HID_MOUSE_CURSOR_MOVE   = 3
 *   UX_DEMO_HID_MOUSE_CURSOR_MOVE_N = 30  (but buffer is set before the switch,
 *                                          so only N-1 = 29 delta steps reach
 *                                          the host per straight segment)
 * --------------------------------------------------------------------------*/
#define MOUSE_STEP              3                           /* delta per HID report      */
#define MOUSE_SIDE              90                          /* max absolute coordinate   */
#define MOUSE_EXPECTED_STEPS    (MOUSE_SIDE / MOUSE_STEP)  /* 30 steps per segment      */
#define MOUSE_MAX_SAMPLES       120                         /* collection-loop limit     */

/* Path-segment identifiers for the verification state machine */
#define MSEG_INIT   0u   /* waiting for first sample                          */
#define MSEG_RIGHT  1u   /* X increasing, Y == 0                              */
#define MSEG_DOWN   2u   /* Y increasing, X == MOUSE_SIDE                     */
#define MSEG_LEFT   3u   /* X decreasing, Y == MOUSE_SIDE                     */
#define MSEG_UP     4u   /* Y decreasing, X == 0                              */
#define MSEG_DONE   5u   /* path complete, only (0,0) expected                */

static UX_HOST_CLASS_HID        *hid_instance;
static UX_HOST_CLASS_HID_MOUSE  *hid_mouse;

static UX_THREAD  host_sim_thread;
static ULONG      host_sim_stack[TEST_STACK_SIZE / sizeof(ULONG)];

static UCHAR      test_memory[TEST_MEMORY_SIZE];

/* Deduplicated position sequence — declared here to avoid overflowing the
 * thread stack (TEST_STACK_SIZE = 512 bytes).                               */
static SLONG  g_seq_x[MOUSE_MAX_SAMPLES];
static SLONG  g_seq_y[MOUSE_MAX_SAMPLES];

static UINT host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);
static void host_sim_thread_entry(ULONG thread_input);

/* ---------------------------------------------------------------------------
 * test_application_define  (dispatched by the test harness under CTEST)
 * --------------------------------------------------------------------------*/

void usbx_hid_mouse_demo_device_rtos_test_application_define(void *first_unused_memory)
{
    UINT status;

    (void)first_unused_memory;

    printf("Running HID Mouse Demo Device RTOS Test................. ");

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
     * 2. Initialise the host stack and register the HID class + mouse
     *    client before the device is connected.
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

    /* -------------------------------------------------------------------
     * 3. Initialise the device side via the demo's test-mode entry:
     *    - ux_device_stack_initialize()  (uses sample's USB descriptors)
     *    - register the HID mouse class  (uses sample's callbacks)
     *    - ux_dcd_sim_slave_initialize() (DCD must be registered before HCD)
     *    - start the device HID worker thread
     * ----------------------------------------------------------------- */
    status = usbx_demo_device_hid_mouse_init();
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
 * Collects the (X,Y) position sequence reported by the HID mouse device
 * and then verifies that it traces the expected 90×90 square path in four
 * straight segments:
 *
 *   RIGHT : X increases by MOUSE_STEP (3), Y = 0,          until X = 90
 *   DOWN  : Y increases by MOUSE_STEP (3), X = 90,         until Y = 90
 *   LEFT  : X decreases by MOUSE_STEP (3), Y = 90,         until X = 0
 *   UP    : Y decreases by MOUSE_STEP (3), X = 0,          until Y = 0
 *
 * Each segment has exactly MOUSE_EXPECTED_STEPS = 30 movement steps.
 * --------------------------------------------------------------------------*/

static void host_sim_thread_entry(ULONG thread_input)
{
UINT    status;
SLONG   x, y;
SLONG   prev_x, prev_y; /* previous position for step-size check                */
UINT    n;              /* number of samples collected                          */
UINT    seg;            /* current verification segment (MSEG_*)                */
UINT    seg_steps;      /* movement-step count within the current segment       */
UINT    i;

    UX_PARAMETER_NOT_USED(thread_input);

    /* ----------------------------------------------------------------
     * Wait for the device to enumerate and the mouse client to become live.
     * -------------------------------------------------------------- */
    while ((hid_instance == UX_NULL) ||
           (hid_mouse == UX_NULL) ||
           (hid_mouse -> ux_host_class_hid_mouse_state != (ULONG)UX_HOST_CLASS_INSTANCE_LIVE))
    {
        ux_utility_thread_sleep(10);
    }

    /* ----------------------------------------------------------------
     * Phase 1: collect the position sequence.
     *
     * Poll every 10 ms and record every sample until MOUSE_MAX_SAMPLES
     * (120) readings have been captured — one full 90×90 square.
     * -------------------------------------------------------------- */
    n        = 0;

    while (n < MOUSE_MAX_SAMPLES)
    {
        status = ux_host_class_hid_mouse_position_get(hid_mouse, &x, &y);
        if (status != UX_SUCCESS)
        {
            printf("FAILED (position get error during collection, n=%u)\n", n);
            test_control_return(1);
            return;
        }

        /* Record the position sample. */
        g_seq_x[n] = x;
        g_seq_y[n] = y;
        n++;

        ux_utility_thread_sleep(10);
    }

    /* ----------------------------------------------------------------
     * Phase 2: verify the collected sequence with a linear state machine.
     *
     * The deduplication in Phase 1 collapses the "corner pause" (where
     * the device emits a zero-delta report on direction change) into the
     * single corner point that is already the last sample of the previous
     * segment.  The verification therefore sees clean transitions:
     *
     *   i=0  .. i=29  : (3,0)..(90,0)   — MSEG_RIGHT  (30 steps)
     *   i=30 .. i=59  : (90,3)..(90,90) — MSEG_DOWN   (30 steps)
     *   i=60 .. i=89  : (87,90)..(0,90) — MSEG_LEFT   (30 steps)
     *   i=90 .. i=119 : (0,87)..(0,0)   — MSEG_UP     (30 steps)
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
        /* ---- moving right: X increases by MOUSE_STEP, Y must be 0 ---- */
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

        /* ---- moving down: Y increases by MOUSE_STEP, X must be MOUSE_SIDE ---- */
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
                    printf("FAILED: DOWN has %u steps, expected %u\n",
                           seg_steps, MOUSE_EXPECTED_STEPS);
                    test_control_return(1);
                    return;
                }
                seg       = MSEG_LEFT;
                seg_steps = 0;
            }
            break;

        /* ---- moving left: X decreases by MOUSE_STEP, Y must be MOUSE_SIDE ---- */
        case MSEG_LEFT:
            if (y != MOUSE_SIDE || x < 0 || x >= MOUSE_SIDE)
            {
                printf("FAILED: LEFT segment out of bounds (%d,%d) at step %u\n", (int)x, (int)y, seg_steps);
                test_control_return(1);
                return;
            }
            if (x != prev_x - MOUSE_STEP)
            {
                printf("FAILED: LEFT bad step %d -> %d (expected -%d)\n",
                       (int)prev_x, (int)x, MOUSE_STEP);
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

        /* ---- moving up: Y decreases by MOUSE_STEP, X must be 0 ---- */
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
        printf("FAILED: path incomplete, stopped at segment %u (%u samples)\n",
               seg, n);
        test_control_return(1);
        return;
    }

    printf("SUCCESS!\n");
    test_control_return(0);
}

/* ---------------------------------------------------------------------------
 * host_event_callback — tracks HID class and mouse client insertions.
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
        break;

    case UX_HID_CLIENT_REMOVAL:

        if ((VOID *)hid_mouse == client -> ux_host_class_hid_client_local_instance)
            hid_mouse = UX_NULL;
        break;

    default:
        break;
    }

    return UX_SUCCESS;
}
