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
/**                                                                       */
/**                                                                       */
/** Note                                                                  */
/**                                                                       */
/**  This demonstration is not optimized, to optimize application user    */
/**  sould configuer related class flag in ux_user.h and adjust           */
/**  UX_DEVICE_MEMORY_STACK_SIZE                                          */
/**                                                                       */
/**                                                                       */
/**  AUTHOR                                                               */
/**                                                                       */
/**   Mohamed AYED                                                        */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

#ifndef UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT
#error  UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT must be defined for this sample
#endif

/************************************************************************************************/
/**  Define constants                                                                           */
/************************************************************************************************/
#define UX_DEVICE_MEMORY_STACK_SIZE     (12*1024)
#define UX_DEMO_THREAD_STACK_SIZE       (1*1024)

#define UX_DEMO_BUFFER_SIZE             512

/************************************************************************************************/
/**  Demo device class demo callbacks function prototypes                                       */
/************************************************************************************************/
static VOID ux_demo_device_cdc_acm_instance_activate(VOID *cdc_acm_instance);
static VOID ux_demo_device_cdc_acm_instance_deactivate(VOID *cdc_acm_instance);
static VOID ux_demo_device_cdc_acm_instance_parameter_chage(VOID *cdc_acm_instance);

/************************************************************************************************/
/**  usbx application initialization with RTOS                                                  */
/************************************************************************************************/
#ifndef DEMO_TEST
VOID tx_application_define(VOID *first_unused_memory);
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  usbx device cdc acm instance                                                               */
/************************************************************************************************/
static UX_SLAVE_CLASS_CDC_ACM *cdc_acm;

/************************************************************************************************/
/**  Thread object                                                                              */
/************************************************************************************************/
static UX_THREAD ux_demo_thread;
static ULONG ux_demo_thread_stack[UX_DEMO_THREAD_STACK_SIZE / sizeof(ULONG)];
static ULONG ux_demo_thread_size = UX_DEMO_THREAD_STACK_SIZE;
static VOID ux_demo_thread_entry(ULONG thread_input);

static UX_THREAD ux_device_cdc_acm_thread;
static ULONG ux_device_cdc_acm_thread_stack[UX_DEMO_THREAD_STACK_SIZE / sizeof(ULONG)];
static ULONG ux_device_cdc_acm_thread_size = UX_DEMO_THREAD_STACK_SIZE;
static VOID ux_device_cdc_acm_thread_entry(ULONG thread_input);

/************************************************************************************************/
/**  usbx demo callback prototype                                                               */
/************************************************************************************************/
static VOID ux_demo_error_callback(UINT system_level, UINT system_context, UINT error_code);

/************************************************************************************************/
/**  Demo function prototypes                                                                   */
/************************************************************************************************/
UINT usbx_demo_device_cdc_acm_init(VOID);
UINT usbx_demo_device_cdc_acm_uninit(VOID);
static UINT ux_device_cdc_acm_init(VOID);
static UINT ux_device_cdc_acm_uninit(VOID);

/************************************************************************************************/
/**  Demo variables                                                                             */
/************************************************************************************************/
#ifndef DEMO_TEST
static CHAR ux_system_memory_pool[UX_DEVICE_MEMORY_STACK_SIZE];
#endif

static UCHAR cdc_acm_buffer[UX_DEMO_BUFFER_SIZE];

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

/* USB High Speed Device Descriptor Length */
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED  sizeof(device_framework_high_speed)

/* USB High Speed Device Descriptor */
static unsigned char device_framework_high_speed[] = {

    /* Device descriptor */
    0x12,           /* bLength */
    0x01,           /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x02,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize0 */
    0x0A, 0x1A,     /* idVendor */
    0x40, 0x62,     /* idProduct */
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacturer */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01,           /* bNumConfigurations */

    /* Device Qualifier descriptor */
    0x0A,           /* bLength */
    0x06,           /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x02,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00,           /* bReserved */

    /* Configuration descriptor */
    0x09,           /* bLength */
    0x02,           /* bDescriptorType */
    0x4B, 0x00,     /* wTotalLength */
    0x02,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x05,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x32,           /* bMaxPower */

    /* CDC ACM Interface Association descriptor */
    0x08,           /* bLength */
    0x0B,           /* bDescriptorType */
    0x00,           /* bFirstInterface */
    0x02,           /* bInterfaceCount */
    0x02,           /* bFunctionClass */
    0x02,           /* bFunctionSubClass */
    0x01,           /* bFunctionProtocol */
    0x00,           /* iFunction */

    /* CDC ACM Control Interface descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x02,           /* bInterfaceClass */
    0x02,           /* bInterfaceSubClass */
    0x01,           /* bInterfaceProtocol */
    0x06,           /* iInterface */

    /* CDC ACM Header Functional Descriptor */
    0x05,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x00,           /* bDescriptorSubtype */
    0x10, 0x01,     /* bcdCDC */

    /* CDC ACM Call Management Functional Descriptor */
    0x05,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x01,           /* bDescriptorSubtype */
    0x00,           /* bmCapabilities */
    0x01,           /* bDataInterface */

    /* CDC ACM Abstract Control Management Functional Descriptor */
    0x04,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x02,           /* bDescriptorSubtype */
    0x02,           /* bmCapabilities */

    /* CDC ACM Union Functional Descriptor (1 slave interface) */
    0x05,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x06,           /* bDescriptorSubtype */
    0x00,           /* bMasterInterface */
    0x01,           /* bSlaveInterface0 */

    /* CDC ACM Endpoint NOTIFY Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x81,           /* bEndpointAddress */
    0x03,           /* bmAttributes */
    0x40, 0x00,     /* wMaxPacketSize */
    0x16,           /* bInterval */

    /* CDC ACM Data Interface Descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x01,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x0A,           /* bInterfaceClass */
    0x00,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x07,           /* iInterface */

    /* CDC ACM Endpoint DATA Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x82,           /* bEndpointAddress */
    0x02,           /* bmAttributes */
    0x00, 0x02,     /* wMaxPacketSize */
    0x00,           /* bInterval */

    /* CDC ACM Endpoint DATA Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x02,           /* bEndpointAddress */
    0x02,           /* bmAttributes */
    0x00, 0x02,     /* wMaxPacketSize */
    0x00            /* bInterval */
};

/* USB Full Speed Device Descriptor Length */
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED  sizeof(device_framework_full_speed)

/* USB Full Speed Device Descriptor */
static unsigned char device_framework_full_speed[] = {

    /* Device descriptor */
    0x12,           /* bLength */
    0x01,           /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x02,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x08,           /* bMaxPacketSize0 */
    0x0A, 0x1A,     /* idVendor */
    0x40, 0x62,     /* idProduct */
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacturer */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01,           /* bNumConfigurations */

    /* Configuration descriptor */
    0x09,           /* bLength */
    0x02,           /* bDescriptorType */
    0x4B, 0x00,     /* wTotalLength */
    0x02,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x04,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x32,           /* bMaxPower */

    /* CDC ACM Interface Association descriptor */
    0x08,           /* bLength */
    0x0B,           /* bDescriptorType */
    0x00,           /* bFirstInterface */
    0x02,           /* bInterfaceCount */
    0x02,           /* bFunctionClass */
    0x02,           /* bFunctionSubClass */
    0x01,           /* bFunctionProtocol */
    0x00,           /* iFunction */

    /* CDC ACM Control Interface descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x02,           /* bInterfaceClass */
    0x02,           /* bInterfaceSubClass */
    0x01,           /* bInterfaceProtocol */
    0x06,           /* iInterface */

    /* CDC ACM Header Functional Descriptor */
    0x05,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x00,           /* bDescriptorSubtype */
    0x10, 0x01,     /* bcdCDC */

    /* CDC ACM Call Management Functional Descriptor */
    0x05,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x01,           /* bDescriptorSubtype */
    0x00,           /* bmCapabilities */
    0x01,           /* bDataInterface */

    /* CDC ACM Abstract Control Management Functional Descriptor */
    0x04,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x02,           /* bDescriptorSubtype */
    0x02,           /* bmCapabilities */

    /* CDC ACM Union Functional Descriptor (1 slave interface) */
    0x05,           /* bFunctionLength */
    0x24,           /* bDescriptorType */
    0x06,           /* bDescriptorSubtype */
    0x00,           /* bMasterInterface */
    0x01,           /* bSlaveInterface0 */

    /* CDC ACM Endpoint NOTIFY Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x81,           /* bEndpointAddress */
    0x03,           /* bmAttributes */
    0x08, 0x00,     /* wMaxPacketSize */
    0x16,           /* bInterval */

    /* CDC ACM Data Interface Descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x01,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x0A,           /* bInterfaceClass */
    0x00,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x07,           /* iInterface */

    /* CDC ACM Endpoint DATA Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x82,           /* bEndpointAddress */
    0x02,           /* bmAttributes */
    0x40, 0x00,     /* wMaxPacketSize */
    0x00,           /* bInterval */

    /* CDC ACM Endpoint DATA Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x02,           /* bEndpointAddress */
    0x02,           /* bmAttributes */
    0x40, 0x00,     /* wMaxPacketSize */
    0x00            /* bInterval */
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

    /* iInterface CDC ACM string descriptor */
    0x09, 0x04,     /* Language ID */
    0x06,           /* String Index */
    0x0F,           /* String Length */
    'C', 'D', 'C', ' ', 'A', 'C', 'M', ' ', 'C', 'O', 'N', 'T', 'R', 'O', 'L',

    /* iInterface CDC ACM string descriptor */
    0x09, 0x04,     /* Language ID */
    0x07,           /* String Index */
    0x0C,           /* String Length */
    'C', 'D', 'C', ' ', 'A', 'C', 'M', ' ', 'D', 'A', 'T', 'A'
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
/**  Perform board-level setup and then transfer control to the hreadX kernel so the RTOS       */
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

    usbx_demo_device_cdc_acm_init();
}
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  usbx_demo_device_cdc_acm_init                                                              */
/**                                                                                             */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_cdc_acm_init(VOID)
{
UINT    status;

    /* Create the main demo thread.  */
    status = ux_utility_thread_create(&ux_demo_thread, "usbx_demo_app_thread_entry",
                                      ux_demo_thread_entry, 0, ux_demo_thread_stack,
                                      ux_demo_thread_size, 20, 20, 1, UX_AUTO_START);

    if(status != UX_SUCCESS)
        return status;

    /* Create the cdc acm demo thread.  */
    status = ux_utility_thread_create(&ux_device_cdc_acm_thread, "usbx_cdc_acm_app_thread_entry",
                                      ux_device_cdc_acm_thread_entry, 0, ux_device_cdc_acm_thread_stack,
                                      ux_device_cdc_acm_thread_size, 20, 20, UX_NO_TIME_SLICE, UX_AUTO_START);

    if(status != UX_SUCCESS)
        return status;

    status = ux_device_cdc_acm_init();

    if(status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_device_cdc_acm_init                                                                     */
/**                                                                                             */
/**  Initialize USBX device resources, install the device stack, and register the CDC ACM class */
/**  instance with its callbacks.                                                               */
/************************************************************************************************/
static UINT ux_device_cdc_acm_init(VOID)
{

UINT                                    status;
UX_SLAVE_CLASS_CDC_ACM_PARAMETER        cdc_acm_parameter = {UX_NULL};

#ifndef DEMO_TEST
    /* Initialize USBX Memory.  */
    status = ux_system_initialize(ux_system_memory_pool, UX_DEVICE_MEMORY_STACK_SIZE, UX_NULL, 0);

    if(status != UX_SUCCESS)
        return status;
#endif /* DEMO_TEST */

    /* Install the device portion of USBX.  */
    status =  ux_device_stack_initialize(device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                         device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                         device_framework_string, DEVICE_FRAMEWORK_LENGTH_STRING,
                                         device_framework_language_id, DEVICE_FRAMEWORK_LENGTH_LANGUAGE_ID,
                                         UX_NULL);

    if(status != UX_SUCCESS)
        return status;

    /* Initialize the cdc acm class parameters for the device */
    cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate   = ux_demo_device_cdc_acm_instance_activate;
    cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = ux_demo_device_cdc_acm_instance_deactivate;
    cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change    = ux_demo_device_cdc_acm_instance_parameter_chage;

    /* Initialize the device cdc acm class. The class is connected with interface 0 on configuration 1. */
    status = ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry,
                                            1, 0, (VOID *)&cdc_acm_parameter);

    if(status != UX_SUCCESS)
        return status;

    /* Register error callback.  */
    ux_utility_error_callback_register(ux_demo_error_callback);

    return status;
}

/************************************************************************************************/
/**  usbx_demo_device_cdc_acm_uninit                                                            */
/**                                                                                             */
/**  Stop the demo worker threads created by                                                    */
/**  usbx_demo_device_cdc_acm_init so the RTOS sample can be shut down cleanly.                 */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_cdc_acm_uninit(VOID)
{

UINT    status;

    /* Delete the main demo thread.  */
    status = ux_utility_thread_delete(&ux_demo_thread);

    if(status != UX_SUCCESS)
        return status;

    /* Delete the cdc acm demo thread.  */
    status = ux_utility_thread_delete(&ux_device_cdc_acm_thread);

    if(status != UX_SUCCESS)
        return status;

    status = ux_device_cdc_acm_uninit();

    if(status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_device_cdc_acm_uninit                                                                   */
/**                                                                                             */
/**  Tear down the CDC ACM device stack and delete the threads created for this RTOS sample.    */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_device_cdc_acm_uninit(VOID)
{

UINT    status;

    /* Uninitialize USBX Memory.  */
    status = ux_device_stack_uninitialize();

    if(status != UX_SUCCESS)
        return status;

    /* Uninitialize the device cdc acm class.  */
    status = ux_device_stack_class_unregister(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry);

    if(status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_demo_device_cdc_acm_instance_activate                                                   */
/************************************************************************************************/
static VOID ux_demo_device_cdc_acm_instance_activate(VOID *cdc_acm_instance)
{
    if (cdc_acm == UX_NULL)
        cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;
}

/************************************************************************************************/
/**  ux_demo_device_cdc_acm_instance_deactivate                                                 */
/************************************************************************************************/
static VOID ux_demo_device_cdc_acm_instance_deactivate(VOID *cdc_acm_instance)
{
    if (cdc_acm_instance == (VOID *)cdc_acm)
        cdc_acm = UX_NULL;
}

/************************************************************************************************/
/**  ux_demo_device_cdc_acm_instance_parameter_chage                                            */
/************************************************************************************************/
static VOID ux_demo_device_cdc_acm_instance_parameter_chage(VOID *cdc_acm_instance)
{
    UX_PARAMETER_NOT_USED(cdc_acm_instance);
}

/************************************************************************************************/
/**  ux_demo_thread_entry                                                                       */
/**                                                                                             */
/**  Register the device controller used by the sample, either the simulator DCD for tests or   */
/**  the board-specific DCD provided by the platform.                                           */
/**                                                                                             */
/************************************************************************************************/
VOID ux_demo_thread_entry(ULONG thread_input)
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
/**  ux_device_cdc_acm_thread_entry                                                             */
/**                                                                                             */
/**  Poll the device state and submit CDC ACM while the device is configured.                   */
/**  When disconnected, the thread sleeps until the host enumerates the device again.           */
/**                                                                                             */
/************************************************************************************************/
static VOID ux_device_cdc_acm_thread_entry(ULONG thread_input)
{

UINT    status;
ULONG   requested_length;
ULONG   actual_length;
UCHAR   *buffer = cdc_acm_buffer;

    UX_PARAMETER_NOT_USED(thread_input);

    /* Check if the device state already configured.  */
    while ((cdc_acm == UX_NULL) && (UX_SLAVE_DEVICE_CHECK_STATE(UX_DEVICE_CONFIGURED) == UX_FALSE))
    {
        /* Sleep thread for 10ms.  */
        ux_utility_thread_sleep(10);
    }

    /* Echo back when connected.  */
    while(1)
    {

        /* Request exactly one packet (64 or 512).  */
        requested_length = cdc_acm -> ux_slave_class_cdc_acm_interface ->
          ux_slave_interface_first_endpoint ->
            ux_slave_endpoint_descriptor.wMaxPacketSize;

        /* Read from CDC class.  */
        status = ux_device_class_cdc_acm_read(cdc_acm, cdc_acm_buffer, requested_length, &actual_length);

        if (status != UX_SUCCESS)
            continue;

        /* Just echo back.  */
        /* Check the status.  If OK, we will write to the CDC instance.  */
        status = ux_device_class_cdc_acm_write(cdc_acm, cdc_acm_buffer, actual_length, &actual_length);

        /* Check for CR/LF.  */
        if (buffer[actual_length - 1] == '\r')
        {

            /* Copy LF value into user buffer.  */
            ux_utility_memory_copy(buffer, "\n",  1);

            /* And send it again.  */
            status = ux_device_class_cdc_acm_write(cdc_acm, buffer, 1, &actual_length);

        }
        else
        {

            /* Send ZLP if it's full packet.  */
            if ((actual_length % requested_length) == 0)
                status = ux_device_class_cdc_acm_write(cdc_acm, buffer, 0, &actual_length);
        }
    }
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
