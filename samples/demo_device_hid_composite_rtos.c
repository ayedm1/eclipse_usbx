/***************************************************************************
 * Copyright (c) 2025-present Eclipse ThreadX Contributors
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 *
 * SPDX-License-Identifier: MIT
 **************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** Overview                                                              */
/**                                                                       */
/**  This sample demonstrates USBX in device mode, enumerating as a       */
/**  composite USB HID device that exposes both a keyboard and a mouse    */
/**  through a single HID interface using multiple Report IDs.            */
/**                                                                       */
/**  Two ThreadX threads are created:                                     */
/**   - ux_demo_thread       : registers the device-controller driver     */
/**                            (real DCD or simulator for testing).       */
/**   - ux_device_hid_thread : waits for host enumeration, then first     */
/**                            moves the cursor in a rectangular pattern  */
/**                            (right → down → left → up, 30 steps each,  */
/**                            3 units per step) using Report ID 2, and   */
/**                            then cycles through the letters 'a'–'z'    */
/**                            (keycodes 0x04–0x1D) using Report ID 1.    */
/**                                                                       */
/**  The HID report descriptor contains two top-level collections:        */
/**   - Report ID 1 (Keyboard): 8 modifier bits, 1 reserved byte,         */
/**                              6 keycode slots.                         */
/**   - Report ID 2 (Mouse)   : 3 buttons, relative X/Y/Wheel axes.       */
/**  Full-speed and high-speed device descriptors are both provided;      */
/**  the stack selects the appropriate one at enumeration time.           */
/**  AUTHOR                                                               */
/**                                                                       */
/**                                                                       */
/**   Mohamed AYED                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#include "ux_api.h"
#include "ux_device_class_hid.h"


#if (UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH < 9)
#error HID Keyboard event buffer length must be more then 9
#endif

/************************************************************************************************/
/**  Define constants                                                                           */
/************************************************************************************************/
#define UX_DEVICE_MEMORY_STACK_SIZE     (7*1024)

#define UX_DEMO_THREAD_STACK_SIZE       (512)

#define HID_REPORT_ID_KEYBOARD          0x01U
#define HID_REPORT_ID_MOUSE             0x02U

#define UX_DEMO_HID_MOUSE_CURSOR_MOVE   3
#define UX_DEMO_HID_MOUSE_CURSOR_MOVE_N 30

#define UX_MOUSE_CURSOR_MOVE_RIGHT      0x01
#define UX_MOUSE_CURSOR_MOVE_DOWN       0x02
#define UX_MOUSE_CURSOR_MOVE_LEFT       0x03
#define UX_MOUSE_CURSOR_MOVE_UP         0x04
#define UX_MOUSE_CURSOR_MOVE_DONE       0x05

#define UX_HID_NUM_LOCK_MASK            0x01
#define UX_HID_CAPS_LOCK_MASK           0x02

#define UX_KEYBOARD_SEND_CHAR_DONE      0x01

#define UX_DEMO_MOUSE                   0x00
#define UX_DEMO_KEYBOARD                0x01
#define UX_DEMO_DONE                    0x03

/************************************************************************************************/
/**  Demo device class demo callbacks function prototypes                                       */
/************************************************************************************************/
static VOID ux_demo_device_hid_instance_activate(VOID *hid_instance);
static VOID ux_demo_device_hid_instance_deactivate(VOID *hid_instance);
static UINT ux_demo_device_hid_callback(UX_SLAVE_CLASS_HID *hid_instance, UX_SLAVE_CLASS_HID_EVENT *hid_event);
static UINT ux_demo_device_hid_get_callback(UX_SLAVE_CLASS_HID *hid_instance, UX_SLAVE_CLASS_HID_EVENT *hid_event);

#ifndef DEMO_TEST
/************************************************************************************************/
/**  usbx application initialization with RTOS                                                  */
/************************************************************************************************/
VOID tx_application_define(VOID *first_unused_memory);
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  usbx device hid instance                                                                   */
/************************************************************************************************/
static UX_SLAVE_CLASS_HID *hid_mouse;
static UX_SLAVE_CLASS_HID *hid_keyboard;

/************************************************************************************************/
/**  Thread object                                                                              */
/************************************************************************************************/
static UX_THREAD ux_demo_thread;
static ULONG ux_demo_thread_stack[UX_DEMO_THREAD_STACK_SIZE / sizeof(ULONG)];
static ULONG ux_demo_thread_size = UX_DEMO_THREAD_STACK_SIZE;
static VOID ux_demo_thread_entry(ULONG thread_input);

static UX_THREAD ux_device_hid_thread;
static ULONG ux_device_hid_thread_stack[UX_DEMO_THREAD_STACK_SIZE / sizeof(ULONG)];
static ULONG ux_device_hid_thread_size = UX_DEMO_THREAD_STACK_SIZE;
static VOID ux_device_hid_thread_entry(ULONG thread_input);

/************************************************************************************************/
/**  usbx demo callback prototype                                                               */
/************************************************************************************************/
static VOID ux_demo_error_callback(UINT system_level, UINT system_context, UINT error_code);
static UINT ux_demo_device_change_function(ULONG device_state);

/************************************************************************************************/
/**  Demo function prototypes                                                                   */
/************************************************************************************************/
UINT usbx_demo_device_hid_mouse_keyboard_init(VOID);
UINT usbx_demo_device_hid_mouse_keyboard_uninit(VOID);
static UINT ux_device_hid_init(VOID);
static UINT ux_device_hid_uninit(VOID);
static UINT ux_device_hid_mouse_cursor_move(UX_SLAVE_CLASS_HID *device_hid);
static UINT ux_device_hid_keyboard_send_character(UX_SLAVE_CLASS_HID *device_hid);

/************************************************************************************************/
/**  Demo variables                                                                             */
/************************************************************************************************/
#ifndef DEMO_TEST
static CHAR ux_system_memory_pool[UX_DEVICE_MEMORY_STACK_SIZE];
#endif
static ULONG num_lock_flag  = UX_FALSE;
static ULONG caps_lock_flag = UX_FALSE;

/************************************************************************************************/
/**  usbx demo extern function prototypes                                                       */
/************************************************************************************************/
#ifndef EXTERNAL_MAIN
extern int board_setup(void);
#endif /* EXTERNAL_MAIN */

#ifndef DEMO_TEST
extern int usb_device_dcd_initialize(void *param);
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  USB descriptors                                                                            */
/************************************************************************************************/

/* USB HID Report descriptor Length */
#define UX_HID_REPORT_DESCRIPTOR_LENGTH  sizeof(hid_report_descriptor)

/* USB HID Report descriptor */
static unsigned char hid_report_descriptor[] = {

    // Report ID 1: Keyboard
    0x05, 0x01,                         // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                         // USAGE (Keyboard)
    0xA1, 0x01,                         // COLLECTION (Application)
    0x85, HID_REPORT_ID_KEYBOARD,       //   REPORT_ID (1)
    0x05, 0x07,                         //   USAGE_PAGE (Keyboard)
    0x19, 0xE0,                         //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xE7,                         //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                         //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                         //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                         //   REPORT_SIZE (1)
    0x95, 0x08,                         //   REPORT_COUNT (8)
    0x81, 0x02,                         //   INPUT (Data,Var,Abs) ; Modifier byte
    0x95, 0x01,                         //   REPORT_COUNT (1)
    0x75, 0x08,                         //   REPORT_SIZE (8)
    0x81, 0x03,                         //   INPUT (Const,Var,Abs) ; Reserved
    0x95, 0x06,                         //   REPORT_COUNT (6)
    0x75, 0x08,                         //   REPORT_SIZE (8)
    0x15, 0x00,                         //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                         //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                         //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                         //   USAGE_MINIMUM (0)
    0x29, 0x65,                         //   USAGE_MAXIMUM (101)
    0x81, 0x00,                         //   INPUT (Data,Array)
    0xC0,                               // END_COLLECTION

    // Report ID 2: Mouse
    0x05, 0x01,                         // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                         // USAGE (Mouse)
    0xA1, 0x01,                         // COLLECTION (Application)
    0x85, HID_REPORT_ID_MOUSE,          //   REPORT_ID (2)
    0x09, 0x01,                         //   USAGE (Pointer)
    0xA1, 0x00,                         //   COLLECTION (Physical)
    0x05, 0x09,                         //     USAGE_PAGE (Button)
    0x19, 0x01,                         //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                         //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                         //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                         //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                         //     REPORT_COUNT (3)
    0x75, 0x01,                         //     REPORT_SIZE (1)
    0x81, 0x02,                         //     INPUT (Data,Var,Abs)
    0x95, 0x01,                         //     REPORT_COUNT (1)
    0x75, 0x05,                         //     REPORT_SIZE (5)
    0x81, 0x03,                         //     INPUT (Const,Var,Abs)
    0x05, 0x01,                         //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                         //     USAGE (X)
    0x09, 0x31,                         //     USAGE (Y)
    0x09, 0x38,                         //     USAGE (Wheel)
    0x15, 0x81,                         //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                         //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                         //     REPORT_SIZE (8)
    0x95, 0x03,                         //     REPORT_COUNT (3)
    0x81, 0x06,                         //     INPUT (Data,Var,Rel)
    0xC0,                               //   END_COLLECTION
    0xC0                                // END_COLLECTION
};


/* USB High Speed Device Descriptor Length */
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED  sizeof(device_framework_high_speed)

/* USB High Speed Device Descriptor */
static unsigned char device_framework_high_speed[] = {

    /* Device descriptor */
    0x12,           /* bLength */
    0x01,           /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x03,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize0 */
    0x0A, 0x1A,     /* idVendor */
    0x40, 0x58,     /* idProduct */
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacturer */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01,           /* bNumConfigurations */

    /* Device Qualifier descriptor */
    0x0A,           /* bLength */
    0x06,           /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x03,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00,           /* bReserved */

    /* Configuration descriptor */
    0x09,           /* bLength */
    0x02,           /* bDescriptorType */
    0x22, 0x00,     /* wTotalLength */
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x05,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x32,           /* bMaxPower */

    /* HID Interface descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x06,           /* iInterface */

    /* HID Descriptor */
    0x09,           /* bLength */
    0x21,           /* bDescriptorType */
    0x10, 0x01,     /* bcdHID */
    0x21,           /* bCountryCode */
    0x01,           /* bNumDescriptors */
    0x22,           /* bReportDescriptorType */
    UX_W0(UX_HID_REPORT_DESCRIPTOR_LENGTH),  /* wReportDescriptorLength */
    UX_W1(UX_HID_REPORT_DESCRIPTOR_LENGTH),

    /* HID Endpoint IN Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x81,           /* bEndpointAddress */
    0x03,           /* bmAttributes */
    0x10, 0x00,     /* wMaxPacketSize */
    0x08            /* bInterval */
};

/* USB Full Speed Device Descriptor Length */
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED  sizeof(device_framework_full_speed)

/* USB Full Speed Device Descriptor */
static unsigned char device_framework_full_speed[] = {

    /* Device descriptor */
    0x12,           /* bLength */
    0x01,           /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x03,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x08,           /* bMaxPacketSize0 */
    0x0A, 0x1A,     /* idVendor */
    0x40, 0x58,     /* idProduct */
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacturer */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01,           /* bNumConfigurations */

    /* Configuration descriptor */
    0x09,           /* bLength */
    0x02,           /* bDescriptorType */
    0x22, 0x00,     /* wTotalLength */
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x04,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x32,           /* bMaxPower */

    /* HID Interface descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x01,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x06,           /* iInterface */

    /* HID Descriptor */
    0x09,           /* bLength */
    0x21,           /* bDescriptorType */
    0x10, 0x01,     /* bcdHID */
    0x21,           /* bCountryCode */
    0x01,           /* bNumDescriptors */
    0x22,           /* bReportDescriptorType */
    UX_W0(UX_HID_REPORT_DESCRIPTOR_LENGTH),  /* wReportDescriptorLength */
    UX_W1(UX_HID_REPORT_DESCRIPTOR_LENGTH),

    /* HID Endpoint IN Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x81,           /* bEndpointAddress */
    0x03,           /* bmAttributes */
    0x10, 0x00,     /* wMaxPacketSize */
    0x08            /* bInterval */
};

/* USB String Device Framework Length */
#define DEVICE_FRAMEWORK_LENGTH_STRING  sizeof(device_framework_string)

/* USB String Device Framework */
static unsigned char device_framework_string[] = {

    /* iManufacturer string descriptor */
    0x09, 0x04,     /* Language ID */
    0x01,           /* String Index */
    0x0F,           /* String Length */
    'E', 'c', 'l', 'i', 'p', 's', 'e', ' ', 'T', 'h', 'r', 'e', 'a', 'd', 'x',

    /* iProduct string descriptor */
    0x09, 0x04,     /* Language ID */
    0x02,           /* String Index */
    0x04,           /* String Length */
    'U', 'S', 'B', 'X',

    /* iSerialNumber string descriptor */
    0x09, 0x04,     /* Language ID */
    0x03,           /* String Index */
    0x0E,           /* String Length */
    'U', 'S', 'B', 'D', 'E', 'V', 'I', 'C', 'E', '0', '0', '0', '0', '1',

    /* iConfiguration string descriptor */
    0x09, 0x04,     /* Language ID */
    0x04,           /* String Index */
    0x0A,           /* String Length */
    'F', 'U', 'L', 'L', ' ', 'S', 'P', 'E', 'E', 'D',

    /* iConfiguration string descriptor */
    0x09, 0x04,     /* Language ID */
    0x05,           /* String Index */
    0x0A,           /* String Length */
    'H', 'I', 'G', 'H', ' ', 'S', 'P', 'E', 'E', 'D',

    /* iInterface HID string descriptor */
    0x09, 0x04,     /* Language ID */
    0x04,           /* String Index */
    0x0E,           /* String Length */
    'm', 'o', 'u', 's', 'e', ' ', 'k', 'e', 'y', 'b', 'o', 'a', 'r', 'd'
};

/* USB Language ID Framework Length */
#define DEVICE_FRAMEWORK_LENGTH_LANGUAGE_ID  sizeof(device_framework_language_id)

/* USB Language ID Framework */
static unsigned char device_framework_language_id[] = {
    0x09, 0x04      /* Language ID (0x0409 = English (US)) */
};

#ifndef EXTERNAL_MAIN
/************************************************************************************************/
/**  main                                                                                       */
/**                                                                                             */
/**  Perform board-level setup and then transfer control to the ThreadX kernel so the RTOS      */
/**  sample can start its threads.                                                              */
/**                                                                                             */
/************************************************************************************************/
int main(void)
{
    /* Initialize the board.  */
    board_setup();

    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}
#endif /* EXTERNAL_MAIN */

#ifndef DEMO_TEST
/************************************************************************************************/
/**  tx_application_define                                                                      */
/**                                                                                             */
/**  ThreadX application entry used to start the USBX demo once the kernel has been initialized.*/
/**                                                                                             */
/************************************************************************************************/
VOID tx_application_define(VOID *first_unused_memory)
{
    UX_PARAMETER_NOT_USED(first_unused_memory);

    usbx_demo_device_hid_mouse_keyboard_init();
}
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  usbx_demo_device_hid_mouse_keyboard_init                                                   */
/**                                                                                             */
/**  Create the demo threads and initialize the USBX HID device stack used by this sample.      */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_hid_mouse_keyboard_init(VOID)
{

UINT    status;

    /* Create the main demo thread.  */
    status = ux_utility_thread_create(&ux_demo_thread, "usbx_demo_app_thread_entry",
                                      ux_demo_thread_entry, 0, ux_demo_thread_stack,
                                      ux_demo_thread_size, 20, 20, 1, UX_AUTO_START);

    if (status != UX_SUCCESS)
        return status;

    /* Create the hid demo thread.  */
    status = ux_utility_thread_create(&ux_device_hid_thread, "usbx_hid_app_thread_entry",
                                      ux_device_hid_thread_entry, 0, ux_device_hid_thread_stack,
                                      ux_device_hid_thread_size, 20, 20, UX_NO_TIME_SLICE, UX_AUTO_START);

    if (status != UX_SUCCESS)
        return status;

    status = ux_device_hid_init();

    if (status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_device_hid_init                                                                         */
/**                                                                                             */
/**  Initialize USBX device resources, install the device stack and register the HID mouse and  */
/**  keyboard class instance with its callbacks.                                                */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_device_hid_init(VOID)
{

UINT                            status;
UX_SLAVE_CLASS_HID_PARAMETER    hid_parameter = {UX_NULL};

#ifndef DEMO_TEST
    /* Initialize USBX Memory.  */
    status = ux_system_initialize(ux_system_memory_pool, UX_DEVICE_MEMORY_STACK_SIZE, UX_NULL, 0);

    if (status != UX_SUCCESS)
        return status;
#endif /* DEMO_TEST */

    /* Install the device portion of USBX.  */
    status =  ux_device_stack_initialize(device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                         device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                         device_framework_string, DEVICE_FRAMEWORK_LENGTH_STRING,
                                         device_framework_language_id, DEVICE_FRAMEWORK_LENGTH_LANGUAGE_ID,
                                         ux_demo_device_change_function);

    if (status != UX_SUCCESS)
        return status;

    /* Initialize the hid keyboard class parameters for the device.  */
    hid_parameter.ux_slave_class_hid_instance_activate         = ux_demo_device_hid_instance_activate;
    hid_parameter.ux_slave_class_hid_instance_deactivate       = ux_demo_device_hid_instance_deactivate;
    hid_parameter.ux_device_class_hid_parameter_report_address = hid_report_descriptor;
    hid_parameter.ux_device_class_hid_parameter_report_length  = UX_HID_REPORT_DESCRIPTOR_LENGTH;
    hid_parameter.ux_device_class_hid_parameter_report_id      = UX_TRUE;
    hid_parameter.ux_device_class_hid_parameter_callback       = ux_demo_device_hid_callback;
    hid_parameter.ux_device_class_hid_parameter_get_callback   = ux_demo_device_hid_get_callback;

    /* Initialize the device hid class. The class is connected with interface 0 on configuration 1.  */
    status = ux_device_stack_class_register(_ux_system_slave_class_hid_name, ux_device_class_hid_entry,
                                            1, 0, (VOID *)&hid_parameter);

    if (status != UX_SUCCESS)
        return status;

    /* Register error callback.  */
    ux_utility_error_callback_register(ux_demo_error_callback);

    return status;
}

/************************************************************************************************/
/**  usbx_demo_device_hid_mouse_keyboard_uninit                                                 */
/**                                                                                             */
/**  Stop the demo worker threads created by                                                    */
/**  usbx_demo_device_hid_mouse_keyboard_init so the RTOS sample can be shut down cleanly.      */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_hid_mouse_keyboard_uninit(VOID)
{

UINT    status;

    /* Delete the main demo thread.  */
    status = ux_utility_thread_delete(&ux_demo_thread);

    /* Delete the hid demo thread.  */
    status = ux_utility_thread_delete(&ux_device_hid_thread);

    if (status != UX_SUCCESS)
        return status;

    status = ux_device_hid_uninit();

    if (status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_device_hid_uninit                                                                       */
/**                                                                                             */
/**  Tear down the USBX device stack and unregister the HID mouse+keyboard class.               */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_device_hid_uninit(VOID)
{

UINT    status;

    /* Uninitialize USBX Memory.  */
    status = ux_device_stack_uninitialize();

    if (status != UX_SUCCESS)
        return status;

    /* Uninitialize the device hid class.  */
    status = ux_device_stack_class_unregister(_ux_system_slave_class_hid_name, ux_device_class_hid_entry);

    if (status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_demo_device_hid_instance_activate                                                       */
/**                                                                                             */
/**  Store the HID class instance pointer when the device becomes active so the worker thread   */
/**  can start sending reports.                                                                 */
/**                                                                                             */
/************************************************************************************************/
static VOID ux_demo_device_hid_instance_activate(VOID *hid_instance)
{
    if (hid == UX_NULL)
        hid = (UX_SLAVE_CLASS_HID*) hid_instance;
}

/************************************************************************************************/
/**  ux_demo_device_hid_instance_deactivate                                                     */
/**                                                                                             */
/**  Clear the cached HID class instance pointer when the device is deactivated or disconnected.*/
/**                                                                                             */
/************************************************************************************************/
static VOID ux_demo_device_hid_instance_deactivate(VOID *hid_instance)
{
    if (hid_instance == (VOID *)hid)
        hid = UX_NULL;
}

/************************************************************************************************/
/**  ux_demo_device_hid_callback                                                                */
/**                                                                                             */
/**  Process HID OUTPUT reports sent by the host. Updates num_lock_flag when the Num Lock bit   */
/**  is set in the LED byte and caps_lock_flag when the Caps Lock bit is set.                   */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_demo_device_hid_callback(UX_SLAVE_CLASS_HID *hid_instance, UX_SLAVE_CLASS_HID_EVENT *hid_event)
{
    UX_PARAMETER_NOT_USED(hid_instance);

    /* There was an event.  Analyze it.  Is it NUM LOCK ? */
    if ((hid_event -> ux_device_class_hid_event_buffer[0] & UX_HID_NUM_LOCK_MASK) && (num_lock_flag == UX_FALSE) &&
        (hid_event -> ux_device_class_hid_event_report_type == UX_DEVICE_CLASS_HID_REPORT_TYPE_OUTPUT))

        /* Set the Num lock flag.  */
        num_lock_flag = UX_TRUE;
    else
        /* Reset the Num lock flag.  */
        num_lock_flag = UX_FALSE;

    /* There was an event.  Analyze it.  Is it CAPS LOCK ? */
    if ((hid_event -> ux_device_class_hid_event_buffer[0] & UX_HID_CAPS_LOCK_MASK) && (caps_lock_flag == UX_FALSE) &&
        (hid_event -> ux_device_class_hid_event_report_type == UX_DEVICE_CLASS_HID_REPORT_TYPE_OUTPUT))
        /* Set the Caps lock flag.  */
        caps_lock_flag = UX_TRUE;
    else
        /* Reset the Caps lock flag.  */
        caps_lock_flag = UX_FALSE;

    return UX_SUCCESS;
}

/************************************************************************************************/
/**  ux_demo_device_hid_get_callback                                                            */
/**                                                                                             */
/**  Respond to HID GET-style requests that require application data.                           */
/**  The mouse+keyboard demo has no dynamic data to return, so the request completes            */
/**  successfully without modifying the event.                                                  */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_demo_device_hid_get_callback(UX_SLAVE_CLASS_HID *hid_instance, UX_SLAVE_CLASS_HID_EVENT *hid_event)
{
    UX_PARAMETER_NOT_USED(hid_instance);
    UX_PARAMETER_NOT_USED(hid_event);

    return UX_SUCCESS;
}

/************************************************************************************************/
/**  ux_demo_thread_entry                                                                       */
/**                                                                                             */
/**  Register the device controller used by the sample, either the simulator DCD for tests or   */
/**  the board-specific DCD provided by the platform.                                           */
/**                                                                                             */
/************************************************************************************************/
static VOID ux_demo_thread_entry(ULONG thread_input)
{

    UX_PARAMETER_NOT_USED(thread_input);

#ifndef DEMO_TEST

    /* Register the USB device controllers available in this system */
    usb_device_dcd_initialize(UX_NULL);
#else /* DEMO_TEST */

    /* Register the USB device simulator controllers for testing */
    ux_dcd_sim_slave_initialize();
#endif /* DEMO_TEST */

}

/************************************************************************************************/
/**  ux_device_hid_thread_entry                                                                 */
/**                                                                                             */
/**  Wait for host enumeration, then run the two-phase demo sequence: first move the mouse      */
/**  cursor through a full rectangle using ux_device_hid_mouse_cursor_move, then send all 26   */
/**  keyboard letters using ux_device_hid_keyboard_send_character, sleeping 10 ms between each */
/**  report. The thread exits once both phases complete.                                        */
/**                                                                                             */
/************************************************************************************************/
static VOID ux_device_hid_thread_entry(ULONG thread_input)
{

UINT    demo_state = UX_DEMO_MOUSE;


    UX_PARAMETER_NOT_USED(thread_input);

    /* Check if the device state already configured.  */
    while ((hid == UX_NULL) && (UX_SLAVE_DEVICE_CHECK_STATE(UX_DEVICE_CONFIGURED) == UX_FALSE))
    {
        /* Sleep thread for 10ms.  */
        ux_utility_thread_sleep(10);
    }

    while (1)
    {
        switch(demo_state)
        {

            case UX_DEMO_MOUSE:

                /* Move cursor */
                if (ux_device_hid_mouse_cursor_move(hid) == UX_MOUSE_CURSOR_MOVE_DONE)
                    demo_state = UX_DEMO_KEYBOARD;

                break;

            case UX_DEMO_KEYBOARD:

                /* keyboard send lowercase character */
                if (ux_device_hid_keyboard_send_character(hid) == UX_KEYBOARD_SEND_CHAR_DONE)
                    demo_state = UX_DEMO_DONE;

                break;

            default:

                ux_utility_thread_sleep(10);

                break;
        }


        if (demo_state == UX_DEMO_DONE)
            break;

        /* Sleep thread for 10ms.  */
        ux_utility_thread_sleep(10);
    }
}

/************************************************************************************************/
/**  ux_device_hid_mouse_cursor_move                                                            */
/**                                                                                             */
/**  Build and send one HID input report that moves the mouse cursor along a rectangular path   */
/**  by updating the relative X/Y fields and advancing the current direction state.             */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_device_hid_mouse_cursor_move(UX_SLAVE_CLASS_HID *device_hid)
{

UINT                        status;
UX_SLAVE_CLASS_HID_EVENT    device_hid_event;
static CHAR                 mouse_x;
static CHAR                 mouse_y;
static UCHAR                mouse_move_count;
static UCHAR                mouse_move_dir = UX_MOUSE_CURSOR_MOVE_RIGHT;

    /* Reset the HID event structure.  */
    ux_utility_memory_set(&device_hid_event, 0, sizeof(UX_SLAVE_CLASS_HID_EVENT));

    /* Move cursor.  */
    switch(mouse_move_dir)
    {
        case UX_MOUSE_CURSOR_MOVE_RIGHT:  /* +x.  */

            mouse_x = (CHAR)UX_DEMO_HID_MOUSE_CURSOR_MOVE;
            mouse_y = 0;
            mouse_move_count ++;

            if (mouse_move_count >= UX_DEMO_HID_MOUSE_CURSOR_MOVE_N)
            {
                mouse_move_count = 0;
                mouse_move_dir = UX_MOUSE_CURSOR_MOVE_DOWN;
            }

            break;

        case UX_MOUSE_CURSOR_MOVE_DOWN:  /* +y.  */

            mouse_x = 0;
            mouse_y = (CHAR)UX_DEMO_HID_MOUSE_CURSOR_MOVE;
            mouse_move_count ++;

            if (mouse_move_count >= UX_DEMO_HID_MOUSE_CURSOR_MOVE_N)
            {

                mouse_move_count = 0;
                mouse_move_dir = UX_MOUSE_CURSOR_MOVE_LEFT;
            }
            break;

        case UX_MOUSE_CURSOR_MOVE_LEFT:  /* -x. */

            mouse_x = (CHAR)(-UX_DEMO_HID_MOUSE_CURSOR_MOVE);
            mouse_y = 0;
            mouse_move_count ++;

            if (mouse_move_count >= UX_DEMO_HID_MOUSE_CURSOR_MOVE_N)
            {
                mouse_move_count = 0;
                mouse_move_dir = UX_MOUSE_CURSOR_MOVE_UP;
            }

            break;

        case UX_MOUSE_CURSOR_MOVE_UP:  /* -y. */

            mouse_x = 0;
            mouse_y = (CHAR)(-UX_DEMO_HID_MOUSE_CURSOR_MOVE);
            mouse_move_count ++;

            if (mouse_move_count >= UX_DEMO_HID_MOUSE_CURSOR_MOVE_N)
            {
                mouse_move_count = 0;
                mouse_move_dir = UX_MOUSE_CURSOR_MOVE_DONE;
            }

            break;

        case UX_MOUSE_CURSOR_MOVE_DONE:

            mouse_x = 0;
            mouse_y = 0;

            break;

        default:

            ux_utility_memory_set(&device_hid_event, 0, sizeof(UX_SLAVE_CLASS_HID_EVENT));

            break;
    }

    device_hid_event.ux_device_class_hid_event_report_id = HID_REPORT_ID_MOUSE;
    device_hid_event.ux_device_class_hid_event_report_type = UX_DEVICE_CLASS_HID_REPORT_TYPE_INPUT;
    device_hid_event.ux_device_class_hid_event_length = 4;
    device_hid_event.ux_device_class_hid_event_buffer[0] = 0;           /* ...R|M|L  */
    device_hid_event.ux_device_class_hid_event_buffer[1] = mouse_x;     /* X         */
    device_hid_event.ux_device_class_hid_event_buffer[2] = mouse_y;     /* Y         */
    device_hid_event.ux_device_class_hid_event_buffer[3] = 0;           /* Wheel     */

    status = ux_device_class_hid_event_set(device_hid, &device_hid_event);

    if (status != UX_SUCCESS)
        return UX_ERROR;


    return mouse_move_dir;
}

/************************************************************************************************/
/**  ux_device_hid_keyboard_send_character                                                      */
/**                                                                                             */
/**  Build and send one HID keyboard report sequence that presses and releases the next key     */
/**  in the demo stream, then advances through the alphabet until all characters have been sent.*/
/**                                                                                             */
/************************************************************************************************/
static UINT ux_device_hid_keyboard_send_character(UX_SLAVE_CLASS_HID *device_hid)
{

UINT                        status = UX_SUCCESS;
UX_SLAVE_CLASS_HID_EVENT    device_hid_event;
static UCHAR                key = 4;

    /* Reset the HID event structure.  */
    ux_utility_memory_set(&device_hid_event, 0, sizeof(UX_SLAVE_CLASS_HID_EVENT));

    /* Then insert a key into the keyboard event.  Length is fixed to 8.  */
    device_hid_event.ux_device_class_hid_event_report_id = HID_REPORT_ID_KEYBOARD;
    device_hid_event.ux_device_class_hid_event_report_type = UX_DEVICE_CLASS_HID_REPORT_TYPE_INPUT;
    device_hid_event.ux_device_class_hid_event_length = 8;
    device_hid_event.ux_device_class_hid_event_buffer[0] = 0;     /* 0x02: Left Shift modifier */
    device_hid_event.ux_device_class_hid_event_buffer[1] = 0;
    device_hid_event.ux_device_class_hid_event_buffer[2] = key;   /* key */
    device_hid_event.ux_device_class_hid_event_buffer[3] = 0;
    device_hid_event.ux_device_class_hid_event_buffer[4] = 0;
    device_hid_event.ux_device_class_hid_event_buffer[5] = 0;
    device_hid_event.ux_device_class_hid_event_buffer[6] = 0;
    device_hid_event.ux_device_class_hid_event_buffer[7] = 0;

    /* Set the keyboard event.  */
    status = ux_device_class_hid_event_set(device_hid, &device_hid_event);

    if (status != UX_SUCCESS)
        return status;

    /* Next event has the key depressed.  */
    device_hid_event.ux_device_class_hid_event_buffer[2] = 0;

    /* Set the keyboard event.  */
    status = ux_device_class_hid_event_set(device_hid, &device_hid_event);

    if (status != UX_SUCCESS)
        return status;

    /* Are we at the end of alphabet ?  */
    if (key != (0x04 + 25))
        key++;
    else
        status = UX_KEYBOARD_SEND_CHAR_DONE;

    return status;
}

/************************************************************************************************/
/**  ux_demo_device_change_function                                                             */
/**                                                                                             */
/**  Device-state change callback registered with ux_device_stack_initialize. The stack calls   */
/**  this function whenever the USB connection state transitions:                               */
/**   - UX_DEVICE_ATTACHED : VBUS detected; host has started enumeration.                       */
/**   - UX_DEVICE_REMOVED  : VBUS lost or host disconnected; hid will be cleared by             */
/**                          ux_demo_device_hid_instance_deactivate shortly after.              */
/**  This sample takes no action on state changes; extend the cases to add application-level    */
/**  power management or safe-state handling as needed.                                         */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_demo_device_change_function(ULONG device_state)
{

UINT status = UX_SUCCESS;

    switch (device_state)
    {
        case UX_DEVICE_ATTACHED:
            break;

        case UX_DEVICE_REMOVED:
            break;

        default:
            break;
    }

  return status;
}

/************************************************************************************************/
/**  ux_demo_error_callback                                                                     */
/**                                                                                             */
/**  Print USBX error details to aid debugging when the stack reports runtime failures.         */
/**                                                                                             */
/************************************************************************************************/
static VOID ux_demo_error_callback(UINT system_level, UINT system_context, UINT error_code)
{
    /*
     * Refer to ux_api.h. For example,
     * UX_SYSTEM_LEVEL_INTERRUPT, UX_SYSTEM_CONTEXT_DCD, UX_DEVICE_HANDLE_UNKNOWN
     */
    printf("USBX error: system level(%d), context(%d), error code(0x%x)\r\n", system_level, system_context, error_code);
}
