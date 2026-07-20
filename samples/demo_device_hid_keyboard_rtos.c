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
/**  This sample demonstrates USBX in device mode, enumerating as a USB   */
/**  HID keyboard (Boot-protocol keyboard, HID class 0x03, subclass 0x01, */
/**  protocol 0x01) on the host.                                          */
/**                                                                       */
/**  Two ThreadX threads are created:                                     */
/**   - ux_demo_thread       : registers the device-controller driver     */
/**                            (real DCD or simulator for testing).       */
/**   - ux_device_hid_thread : waits for host enumeration, then sends     */
/**                            HID input reports every 10 ms that cycle   */
/**                            through the letters 'a' to 'z' (USB        */
/**                            keycodes 0x04 to 0x1D), pressing and       */
/**                            releasing each key in turn.                */
/**                                                                       */
/**  The HID report descriptor exposes 8 modifier bits, 1 reserved byte, */
/**  5 LED output bits (Num Lock, Caps Lock, Scroll Lock, Compose, Kana), */
/**  and 6 simultaneous keycode slots. Full-speed and high-speed device   */
/**  descriptors are both provided; the stack selects the appropriate one */
/**  at enumeration time.                                                 */
/**                                                                       */
/**                                                                       */
/**  AUTHOR                                                               */
/**                                                                       */
/**   Mohamed AYED                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#include "ux_api.h"
#include "ux_device_class_hid.h"


#if (UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH < 8)
#error HID keyboard event buffer length must be more then 8.
#endif

/************************************************************************************************/
/**  Define constants                                                                           */
/************************************************************************************************/
#define UX_DEVICE_MEMORY_STACK_SIZE     (7*1024)
#define UX_DEMO_THREAD_STACK_SIZE       (512)

#define UX_HID_NUM_LOCK_MASK            0x01
#define UX_HID_CAPS_LOCK_MASK           0x02

#define UX_KEYBOARD_SEND_CHAR_DONE      0x01

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
/**  usbx device hid keyboard instance                                                          */
/************************************************************************************************/
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
UINT usbx_demo_device_hid_keyboard_init(VOID);
UINT usbx_demo_device_hid_keyboard_uninit(VOID);
static UINT ux_device_hid_init(VOID);
static UINT ux_device_hid_uninit(VOID);
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

    0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,  // USAGE (Keyboard)
    0xA1, 0x01,  // COLLECTION (Application)
    0x05, 0x07,  //   USAGE_PAGE (Keyboard/Keypad)
    0x19, 0xE0,  //   USAGE_MINIMUM (Left Control)
    0x29, 0xE7,  //   USAGE_MAXIMUM (Right GUI)
    0x15, 0x00,  //   LOGICAL_MINIMUM (Logical Min (0))
    0x25, 0x01,  //   LOGICAL_MAXIMUM (Logical Max (1))
    0x75, 0x01,  //   REPORT_SIZE (1 bit)
    0x95, 0x08,  //   REPORT_COUNT (8 modifiers)
    0x81, 0x02,  //   INPUT (Data, Variable, Absolute)
    0x75, 0x08,  //   REPORT_SIZE (8 bits)
    0x95, 0x01,  //   REPORT_COUNT (1 byte)
    0x81, 0x01,  //   INPUT (Constant (Reserved))
    0x05, 0x08,  //   USAGE_PAGE (LED)
    0x19, 0x01,  //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,  //   USAGE_MAXIMUM (Kana)
    0x75, 0x01,  //   REPORT_SIZE (1 bit)
    0x95, 0x05,  //   REPORT_COUNT (5 LEDs)
    0x91, 0x02,  //   OUTPUT (Data, Variable, Absolute)
    0x75, 0x03,  //   REPORT_SIZE (3 bits)
    0x95, 0x01,  //   REPORT_COUNT (1 report)
    0x91, 0x01,  //   OUTPUT (Constant (Padding))
    0x05, 0x07,  //   USAGE_PAGE (Keyboard/Keypad)
    0x19, 0x00,  //   USAGE_MINIMUM (No Event)
    0x29, 0x65,  //   USAGE_MAXIMUM (Keyboard Application)
    0x15, 0x00,  //   LOGICAL_MINIMUM (Logical Min (0))
    0x25, 0x65,  //   LOGICAL_MAXIMUM (Logical Max (101))
    0x75, 0x08,  //   REPORT_SIZE (8 bits)
    0x95, 0x06,  //   REPORT_COUNT (6 keycodes)
    0x81, 0x00,  //   INPUT (Data, Array)
    0xC0         // END_COLLECTION
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
    0x40, 0x57,     /* idProduct */
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
    0x01,           /* bInterfaceProtocol */
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
    0x08, 0x00,     /* wMaxPacketSize */
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
    0x40, 0x57,     /* idProduct */
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
    0x01,           /* bInterfaceProtocol */
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
    0x08, 0x00,     /* wMaxPacketSize */
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
    0x06,           /* String Index */
    0x07,           /* String Length */
    'K', 'e', 'y', 'b', 'o', 'r', 'd'
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

    usbx_demo_device_hid_keyboard_init();
}
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  usbx_demo_device_hid_keyboard_init                                                         */
/**                                                                                             */
/**  Create the demo threads and initialize the USBX HID device stack used by this sample.      */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_hid_keyboard_init(VOID)
{

UINT    status;

    /* Create the main demo thread.  */
    status = ux_utility_thread_create(&ux_demo_thread, "usbx_demo_app_thread_entry",
                                      ux_demo_thread_entry, 0, ux_demo_thread_stack,
                                      ux_demo_thread_size, 20, 20, 1, UX_AUTO_START);

    if(status != UX_SUCCESS)
        return status;

    /* Create the hid demo thread.  */
    status = ux_utility_thread_create(&ux_device_hid_thread, "usbx_hid_app_thread_entry",
                                      ux_device_hid_thread_entry, 0, ux_device_hid_thread_stack,
                                      ux_device_hid_thread_size, 20, 20, UX_NO_TIME_SLICE, UX_AUTO_START);

    if(status != UX_SUCCESS)
        return status;

    status = ux_device_hid_init();

    if(status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_device_hid_init                                                                         */
/**                                                                                             */
/**  Initialize USBX device resources, install the device stack and register the HID keyboard   */
/**  class instance with its callbacks.                                                         */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_device_hid_init(VOID)
{

UINT                            status;
UX_SLAVE_CLASS_HID_PARAMETER    hid_keyboard_parameter = {UX_NULL};

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
    hid_keyboard_parameter.ux_slave_class_hid_instance_activate         = ux_demo_device_hid_instance_activate;
    hid_keyboard_parameter.ux_slave_class_hid_instance_deactivate       = ux_demo_device_hid_instance_deactivate;
    hid_keyboard_parameter.ux_device_class_hid_parameter_report_address = hid_report_descriptor;
    hid_keyboard_parameter.ux_device_class_hid_parameter_report_length  = UX_HID_REPORT_DESCRIPTOR_LENGTH;
    hid_keyboard_parameter.ux_device_class_hid_parameter_report_id      = UX_FALSE;
    hid_keyboard_parameter.ux_device_class_hid_parameter_callback       = ux_demo_device_hid_callback;
    hid_keyboard_parameter.ux_device_class_hid_parameter_get_callback   = ux_demo_device_hid_get_callback;

    /* Initialize the device hid class. The class is connected with interface 0 on configuration 1.  */
    status = ux_device_stack_class_register(_ux_system_slave_class_hid_name, ux_device_class_hid_entry,
                                            1, 0, (VOID *)&hid_keyboard_parameter);

    if (status != UX_SUCCESS)
        return status;

    /* Register error callback.  */
    ux_utility_error_callback_register(ux_demo_error_callback);

    return status;
}

/************************************************************************************************/
/**  usbx_demo_device_hid_keyboard_uninit                                                       */
/**                                                                                             */
/**  Stop the demo worker threads created by                                                    */
/**  usbx_demo_device_hid_keyboard_init so the RTOS sample can be shut down cleanly.            */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_hid_keyboard_uninit(VOID)
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
/**  Unregister the HID keyboard class and tear down the USBX device stack.                     */
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
    if (hid_keyboard == UX_NULL)
        hid_keyboard = (UX_SLAVE_CLASS_HID*) hid_instance;
}

/************************************************************************************************/
/**  ux_demo_device_hid_instance_deactivate                                                     */
/**                                                                                             */
/**  Clear the cached HID class instance pointer when the device is deactivated or disconnected.*/
/**                                                                                             */
/************************************************************************************************/
static VOID ux_demo_device_hid_instance_deactivate(VOID *hid_instance)
{
    if (hid_instance == (VOID *)hid_keyboard)
        hid_keyboard = UX_NULL;
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
/**  The keyboard demo has no dynamic data to return, so the request completes successfully     */
/**  without modifying the event.                                                               */
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
/**  Wait for host enumeration, then call ux_device_hid_keyboard_send_character repeatedly,     */
/**  sleeping 10 ms between each report, until all 26 letters have been sent. The thread exits  */
/**  once the full sequence is complete.                                                        */
/**                                                                                             */
/************************************************************************************************/
static VOID ux_device_hid_thread_entry(ULONG thread_input)
{

    UX_PARAMETER_NOT_USED(thread_input);

    /* Check if the device state already configured.  */
    while ((hid_keyboard == UX_NULL) && (UX_SLAVE_DEVICE_CHECK_STATE(UX_DEVICE_CONFIGURED) == UX_FALSE))
    {
        /* Sleep thread for 10ms.  */
        ux_utility_thread_sleep(10);
    }

    while (1)
    {
        if (ux_device_hid_keyboard_send_character(hid_keyboard) == UX_KEYBOARD_SEND_CHAR_DONE)
            break;

        /* Sleep thread for 10ms.  */
        ux_utility_thread_sleep(10);
    }
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
    device_hid_event.ux_device_class_hid_event_report_id = 0;
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
/**   - UX_DEVICE_REMOVED  : VBUS lost or host disconnected, hid_keyboard will be cleared by    */
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
