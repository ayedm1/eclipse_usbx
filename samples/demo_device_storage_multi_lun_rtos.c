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
#include "ux_device_class_storage.h"

#ifndef UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT
#error  UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT must be defined for this sample
#endif

#if UX_MAX_SLAVE_LUN != 2
#error UX_MAX_SLAVE_LUN must be 2 for this sample
#endif

/************************************************************************************************/
/**  Define constants                                                                           */
/************************************************************************************************/
#define UX_DEVICE_MEMORY_STACK_SIZE     (8*1024)

#define UX_DEMO_THREAD_STACK_SIZE       (512)

#define BUILD_FILE_SYSTEM

#ifndef RAM_DISK_SIZE
#define RAM_DISK_SIZE                   (1024 * 90) /* Must > 20K for windows.  */
#endif

#define RAM_DISK_N_LBA                  (RAM_DISK_SIZE / 512)
#define RAM_DISK_LAST_LBA               (RAM_DISK_N_LBA - 1)
#define RAM_DISK_BLOCK_LENGTH           512

/* Write Caching support.  */
#define RAM_DISK_WRITE_CACHING          UX_FALSE

/************************************************************************************************/
/**  Demo device class demo callbacks function prototypes                                       */
/************************************************************************************************/
static VOID ux_demo_device_storage_instance_activate(VOID *storage_instance);
static VOID ux_demo_device_storage_instance_deactivate(VOID *storage_instance);
static UINT ux_demo_device_storage_media_read(VOID *storage_instance, ULONG lun, UCHAR * data_pointer, ULONG number_blocks,
                                              ULONG lba, ULONG *media_status);
static UINT ux_demo_device_storage_media_write(VOID *storage_instance, ULONG lun, UCHAR * data_pointer, ULONG number_blocks,
                                               ULONG lba, ULONG *media_status);
static UINT ux_demo_device_storage_media_status(VOID *storage_instance, ULONG lun, ULONG media_id, ULONG *media_status);
static UINT ux_demo_device_storage_media_flush(VOID *storage, ULONG lun, ULONG number_blocks, ULONG lba, ULONG *media_status);
static UINT ux_demo_device_storage_media_notification(VOID *storage,  ULONG lun, ULONG media_id, ULONG notification_class,
                                               UCHAR **media_notification, ULONG *media_notification_length);

/************************************************************************************************/
/**  usbx application initialization with RTOS                                                  */
/************************************************************************************************/
#ifndef DEMO_TEST
VOID tx_application_define(VOID *first_unused_memory);
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  usbx device storage instance                                                               */
/************************************************************************************************/
static UX_SLAVE_CLASS_STORAGE *storage;

/************************************************************************************************/
/**  Thread object                                                                              */
/************************************************************************************************/
static UX_THREAD ux_demo_thread;
static ULONG ux_demo_thread_stack[UX_DEMO_THREAD_STACK_SIZE / sizeof(ULONG)];
static ULONG ux_demo_thread_size = UX_DEMO_THREAD_STACK_SIZE;
static VOID ux_demo_thread_entry(ULONG thread_input);

/************************************************************************************************/
/**  usbx demo callback prototype                                                               */
/************************************************************************************************/
static VOID ux_demo_error_callback(UINT system_level, UINT system_context, UINT error_code);

/************************************************************************************************/
/**  Demo function prototypes                                                                   */
/************************************************************************************************/
UINT usbx_demo_device_storage_multi_lun_init(VOID);
UINT usbx_demo_device_storage_multi_lun_uninit(VOID);
static UINT ux_device_storage_init(VOID);
static UINT ux_device_storage_uninit(VOID);
static VOID ux_storage_disk_init(VOID);

/************************************************************************************************/
/**  Demo variables                                                                             */
/************************************************************************************************/
#ifndef DEMO_TEST
static CHAR ux_system_memory_pool[UX_DEVICE_MEMORY_STACK_SIZE];
#endif

static UCHAR ram_disk1_memory[RAM_DISK_SIZE];
static UCHAR ram_disk2_memory[RAM_DISK_SIZE];

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
    0x08,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize0 */
    0x0A, 0x1A,     /* idVendor */
    0x40, 0x61,     /* idProduct */
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacturer */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01,           /* bNumConfigurations */

    /* Device Qualifier descriptor */
    0x0A,           /* bLength */
    0x06,           /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x08,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00,           /* bReserved */

    /* Configuration descriptor */
    0x09,           /* bLength */
    0x02,           /* bDescriptorType */
    0x20, 0x00,     /* wTotalLength */
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x32,           /* bMaxPower */

    /* MSC Interface descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x08,           /* bInterfaceClass */
    0x06,           /* bInterfaceSubClass */
    0x50,           /* bInterfaceProtocol */
    0x06,           /* iInterface */

    /* MSC Endpoint IN Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x81,           /* bEndpointAddress */
    0x02,           /* bmAttributes */
    0x00, 0x02,     /* wMaxPacketSize */
    0x00,           /* bInterval */

    /* MSC Endpoint OUT Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x01,           /* bEndpointAddress */
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
    0x08,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x08,           /* bMaxPacketSize0 */
    0x0A, 0x1A,     /* idVendor */
    0x40, 0x61,     /* idProduct */
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacturer */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01,           /* bNumConfigurations */

    /* Configuration descriptor */
    0x09,           /* bLength */
    0x02,           /* bDescriptorType */
    0x20, 0x00,     /* wTotalLength */
    0x01,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x32,           /* bMaxPower */

    /* MSC Interface descriptor */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x08,           /* bInterfaceClass */
    0x06,           /* bInterfaceSubClass */
    0x50,           /* bInterfaceProtocol */
    0x06,           /* iInterface */

    /* MSC Endpoint IN Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x81,           /* bEndpointAddress */
    0x02,           /* bmAttributes */
    0x40, 0x00,     /* wMaxPacketSize */
    0x00,           /* bInterval */

    /* MSC Endpoint OUT Descriptor */
    0x07,           /* bLength */
    0x05,           /* bDescriptorType */
    0x01,           /* bEndpointAddress */
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

    /* iInterface Storage string descriptor */
    0x09, 0x04,     /* Language ID */
    0x06,           /* String Index */
    0x0F,           /* String Length */
    'R', 'e', 'm', 'o', 'v', 'a', 'b', 'l', 'e', ' ', 'd', 'r', 'i', 'v', 'e'
};

/* USB Language ID Framework Length */
#define DEVICE_FRAMEWORK_LENGTH_LANGUAGE_ID  sizeof(device_framework_language_id)

/* USB Language ID Framework */
static unsigned char device_framework_language_id[] = {
    0x09, 0x04      /* Language ID (0x0409 = English (US)) */
};

/************************************************************************************************/
/**  File System                                                                                */
/************************************************************************************************/
#ifdef BUILD_FILE_SYSTEM

static UCHAR sector_0[] = {
    0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30, 0x00,
    0x02, 0x01, 0x06, 0x00, 0x02, 0x00, 0x02,

    /*0x13: small sectors*/
    0xF0, 0x00,
    0xF8, 0x01, 0x00,
    0x01, 0x00, 0x01,

    /*0x20: large sectors*/
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x29,

    /*0x27: serial number*/0x7E, 0x0D, 0x2E, 0xF0,
    0x4E, 0x4F, 0x20, 0x4E, 0x41,
    0x4D, 0x45, 0x20, 0x20, 0x20, 0x20, 0x46, 0x41, 0x54, 0x31, 0x32, 0x20,
    0x20, 0x20, 0x33, 0xC9, 0x8E, 0xD1, 0xBC, 0xF0, 0x7B, 0x8E, 0xD9, 0xB8,
    0x00, 0x20, 0x8E, 0xC0, 0xFC, 0xBD, 0x00, 0x7C, 0x38, 0x4E, 0x24, 0x7D,
    0x24, 0x8B, 0xC1, 0x99, 0xE8, 0x3C, 0x01, 0x72, 0x1C, 0x83, 0xEB, 0x3A,
    0x66, 0xA1, 0x1C, 0x7C, 0x26, 0x66, 0x3B, 0x07, 0x26, 0x8A, 0x57, 0xFC,
    0x75, 0x06, 0x80, 0xCA, 0x02, 0x88, 0x56, 0x02, 0x80, 0xC3, 0x10, 0x73,
    0xEB, 0x33, 0xC9, 0x8A, 0x46, 0x10, 0x98, 0xF7, 0x66, 0x16, 0x03, 0x46,
    0x1C, 0x13, 0x56, 0x1E, 0x03, 0x46, 0x0E, 0x13, 0xD1, 0x8B, 0x76, 0x11,
    0x60, 0x89, 0x46, 0xFC, 0x89, 0x56, 0xFE, 0xB8, 0x20, 0x00, 0xF7, 0xE6,
    0x8B, 0x5E, 0x0B, 0x03, 0xC3, 0x48, 0xF7, 0xF3, 0x01, 0x46, 0xFC, 0x11,
    0x4E, 0xFE, 0x61, 0xBF, 0x00, 0x00, 0xE8, 0xE6, 0x00, 0x72, 0x39, 0x26,
    0x38, 0x2D, 0x74, 0x17, 0x60, 0xB1, 0x0B, 0xBE, 0xA1, 0x7D, 0xF3, 0xA6,
    0x61, 0x74, 0x32, 0x4E, 0x74, 0x09, 0x83, 0xC7, 0x20, 0x3B, 0xFB, 0x72,
    0xE6, 0xEB, 0xDC, 0xA0, 0xFB, 0x7D, 0xB4, 0x7D, 0x8B, 0xF0, 0xAC, 0x98,
    0x40, 0x74, 0x0C, 0x48, 0x74, 0x13, 0xB4, 0x0E, 0xBB, 0x07, 0x00, 0xCD,
    0x10, 0xEB, 0xEF, 0xA0, 0xFD, 0x7D, 0xEB, 0xE6, 0xA0, 0xFC, 0x7D, 0xEB,
    0xE1, 0xCD, 0x16, 0xCD, 0x19, 0x26, 0x8B, 0x55, 0x1A, 0x52, 0xB0, 0x01,
    0xBB, 0x00, 0x00, 0xE8, 0x3B, 0x00, 0x72, 0xE8, 0x5B, 0x8A, 0x56, 0x24,
    0xBE, 0x0B, 0x7C, 0x8B, 0xFC, 0xC7, 0x46, 0xF0, 0x3D, 0x7D, 0xC7, 0x46,
    0xF4, 0x29, 0x7D, 0x8C, 0xD9, 0x89, 0x4E, 0xF2, 0x89, 0x4E, 0xF6, 0xC6,
    0x06, 0x96, 0x7D, 0xCB, 0xEA, 0x03, 0x00, 0x00, 0x20, 0x0F, 0xB6, 0xC8,
    0x66, 0x8B, 0x46, 0xF8, 0x66, 0x03, 0x46, 0x1C, 0x66, 0x8B, 0xD0, 0x66,
    0xC1, 0xEA, 0x10, 0xEB, 0x5E, 0x0F, 0xB6, 0xC8, 0x4A, 0x4A, 0x8A, 0x46,
    0x0D, 0x32, 0xE4, 0xF7, 0xE2, 0x03, 0x46, 0xFC, 0x13, 0x56, 0xFE, 0xEB,
    0x4A, 0x52, 0x50, 0x06, 0x53, 0x6A, 0x01, 0x6A, 0x10, 0x91, 0x8B, 0x46,
    0x18, 0x96, 0x92, 0x33, 0xD2, 0xF7, 0xF6, 0x91, 0xF7, 0xF6, 0x42, 0x87,
    0xCA, 0xF7, 0x76, 0x1A, 0x8A, 0xF2, 0x8A, 0xE8, 0xC0, 0xCC, 0x02, 0x0A,
    0xCC, 0xB8, 0x01, 0x02, 0x80, 0x7E, 0x02, 0x0E, 0x75, 0x04, 0xB4, 0x42,
    0x8B, 0xF4, 0x8A, 0x56, 0x24, 0xCD, 0x13, 0x61, 0x61, 0x72, 0x0B, 0x40,
    0x75, 0x01, 0x42, 0x03, 0x5E, 0x0B, 0x49, 0x75, 0x06, 0xF8, 0xC3, 0x41,
    0xBB, 0x00, 0x00, 0x60, 0x66, 0x6A, 0x00, 0xEB, 0xB0, 0x42, 0x4F, 0x4F,
    0x54, 0x4D, 0x47, 0x52, 0x20, 0x20, 0x20, 0x20, 0x0D, 0x0A, 0x52, 0x65,
    0x6D, 0x6F, 0x76, 0x65, 0x20, 0x64, 0x69, 0x73, 0x6B, 0x73, 0x20, 0x6F,
    0x72, 0x20, 0x6F, 0x74, 0x68, 0x65, 0x72, 0x20, 0x6D, 0x65, 0x64, 0x69,
    0x61, 0x2E, 0xFF, 0x0D, 0x0A, 0x44, 0x69, 0x73, 0x6B, 0x20, 0x65, 0x72,
    0x72, 0x6F, 0x72, 0xFF, 0x0D, 0x0A, 0x50, 0x72, 0x65, 0x73, 0x73, 0x20,
    0x61, 0x6E, 0x79, 0x20, 0x6B, 0x65, 0x79, 0x20, 0x74, 0x6F, 0x20, 0x72,
    0x65, 0x73, 0x74, 0x61, 0x72, 0x74, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xAC, 0xCB, 0xD8, 0x55, 0xAA
};

static UCHAR sector_6_7[] = {
    0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F,
};

static const UCHAR sector_8[] = {
    0x54, 0x45, 0x53, 0x54, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x08,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x92,
    0x81, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x20, 0x00, 0x49,
    0x00, 0x6E, 0x00, 0x66, 0x00, 0x6F, 0x00, 0x0F, 0x00, 0x72, 0x72, 0x00,
    0x6D, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00, 0x6F, 0x00, 0x00, 0x00,
    0x6E, 0x00, 0x00, 0x00, 0x01, 0x53, 0x00, 0x79, 0x00, 0x73, 0x00, 0x74,
    0x00, 0x65, 0x00, 0x0F, 0x00, 0x72, 0x6D, 0x00, 0x20, 0x00, 0x56, 0x00,
    0x6F, 0x00, 0x6C, 0x00, 0x75, 0x00, 0x00, 0x00, 0x6D, 0x00, 0x65, 0x00,
    0x53, 0x59, 0x53, 0x54, 0x45, 0x4D, 0x7E, 0x31, 0x20, 0x20, 0x20, 0x16,
    0x00, 0xB5, 0x25, 0x92, 0x81, 0x52, 0x81, 0x52, 0x00, 0x00, 0x26, 0x92,
    0x81, 0x52, 0x02,
};

/* Per-LUN volume labels (boot sector offset 0x2B, 11 bytes). */
static const UCHAR volume_label_lun1[11] = { 'T','E','S','T','1',' ',' ',' ',' ',' ',' ' };
static const UCHAR volume_label_lun2[11] = { 'T','E','S','T','2',' ',' ',' ',' ',' ',' ' };
#endif /* BUILD_FILE_SYSTEM */

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

    usbx_demo_device_storage_multi_lun_init();
}
#endif /* DEMO_TEST */

/************************************************************************************************/
/**  usbx_demo_device_storage_multi_lun_init                                                    */
/**                                                                                             */
/**  Create the demo threads and initialize the USBX Storage device stack used by this sample.  */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_storage_multi_lun_init(VOID)
{
UINT    status;

    /* Create the main demo thread.  */
    status = ux_utility_thread_create(&ux_demo_thread, "usbx_demo_app_thread_entry",
                                      ux_demo_thread_entry, 0, ux_demo_thread_stack,
                                      ux_demo_thread_size, 20, 20, 1, UX_AUTO_START);

    if(status != UX_SUCCESS)
        return status;

    status = ux_device_storage_init();

    if(status != UX_SUCCESS)
        return status;

    return status;
}


/************************************************************************************************/
/**  ux_device_storage_init                                                                     */
/**                                                                                             */
/**  Initialize USBX device resources, install the device stack,  and register the Storage      */
/**  class instance with its callbacks.                                                         */
/************************************************************************************************/
static UINT ux_device_storage_init(VOID)
{
UINT                               status;
UX_SLAVE_CLASS_STORAGE_PARAMETER   storage_parameter = {UX_NULL};


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

    /* Store the number of LUN in this device storage instance.  */
    storage_parameter.ux_slave_class_storage_instance_activate = ux_demo_device_storage_instance_activate;
    storage_parameter.ux_slave_class_storage_instance_deactivate = ux_demo_device_storage_instance_deactivate;
    storage_parameter.ux_slave_class_storage_parameter_number_lun = 2;
    storage_parameter.ux_slave_class_storage_parameter_vendor_id = (UCHAR*) "Eclipse ";
    storage_parameter.ux_slave_class_storage_parameter_product_id = (UCHAR*) "USBX storage";
    storage_parameter.ux_slave_class_storage_parameter_product_rev = (UCHAR*) "2000";
    storage_parameter.ux_slave_class_storage_parameter_product_serial = (UCHAR*) "001";

    /* Initialize the storage class parameters for reading/writing to the Flash Disk.  */
    /* LUN-1 configuration */
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_last_lba       = RAM_DISK_LAST_LBA;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_block_length   = RAM_DISK_BLOCK_LENGTH;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_type           = 0;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_removable_flag = 0x80;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_read_only_flag = UX_FALSE;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_read           = ux_demo_device_storage_media_read;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_write          = ux_demo_device_storage_media_write;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_status         = ux_demo_device_storage_media_status;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_flush          = RAM_DISK_WRITE_CACHING ? ux_demo_device_storage_media_flush : UX_NULL;
    storage_parameter.ux_slave_class_storage_parameter_lun[0].ux_slave_class_storage_media_notification   = ux_demo_device_storage_media_notification;

    /* LUN2 configuration */
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_last_lba       = RAM_DISK_LAST_LBA;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_block_length   = RAM_DISK_BLOCK_LENGTH;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_type           = 0;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_removable_flag = 0x80;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_read_only_flag = UX_FALSE;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_read           = ux_demo_device_storage_media_read;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_write          = ux_demo_device_storage_media_write;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_status         = ux_demo_device_storage_media_status;
    storage_parameter.ux_slave_class_storage_parameter_lun[1].ux_slave_class_storage_media_flush          = RAM_DISK_WRITE_CACHING ? ux_demo_device_storage_media_flush : UX_NULL;

    /* Initialize the device storage class. The class is connected with interface 0 on configuration 1. */
    status = ux_device_stack_class_register(_ux_system_slave_class_storage_name, _ux_device_class_storage_entry,
                                            1, 0, (VOID *)&storage_parameter);

    if(status != UX_SUCCESS)
        return status;

    /* Register error callback.  */
    ux_utility_error_callback_register(ux_demo_error_callback);

    return status;
}

/************************************************************************************************/
/**  usbx_demo_device_storage_multi_lun_uninit                                                  */
/**                                                                                             */
/**  Stop the demo worker threads created by                                                    */
/**  usbx_demo_device_storage_multi_lun_uninit so the  RTOS sample can be shut down cleanly.    */
/**                                                                                             */
/************************************************************************************************/
UINT usbx_demo_device_storage_multi_lun_uninit(VOID)
{

UINT    status;

    /* Delete the main demo thread.  */
    status = ux_utility_thread_delete(&ux_demo_thread);

    if(status != UX_SUCCESS)
        return status;

    status = ux_device_storage_uninit();

    if(status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_device_storage_uninit                                                                   */
/**                                                                                             */
/**  Tear down the storage device stack and delete the threads created for this RTOS sample.    */
/**                                                                                             */
/************************************************************************************************/
static UINT ux_device_storage_uninit(VOID)
{

UINT    status;

    /* Uninitialize USBX Memory.  */
    status = ux_device_stack_uninitialize();

    if(status != UX_SUCCESS)
        return status;

    /* Uninitialize the device storage class.  */
    status = ux_device_stack_class_unregister(_ux_system_slave_class_storage_name, ux_device_class_storage_entry);

    if(status != UX_SUCCESS)
        return status;

    return status;
}

/************************************************************************************************/
/**  ux_demo_device_storage_instance_activate                                                   */
/************************************************************************************************/
static VOID ux_demo_device_storage_instance_activate(VOID *storage_instance)
{
    if (storage == UX_NULL)
        storage = (UX_SLAVE_CLASS_STORAGE*) storage_instance;
}

/************************************************************************************************/
/**  ux_demo_device_storage_instance_deactivate                                                 */
/************************************************************************************************/
static VOID ux_demo_device_storage_instance_deactivate(VOID *storage_instance)
{
    if (storage_instance == (VOID *)storage)
        storage = UX_NULL;
}

/************************************************************************************************/
/**  ux_demo_device_storage_media_read                                                          */
/************************************************************************************************/
static UINT ux_demo_device_storage_media_read(VOID *storage, ULONG lun, UCHAR *data_pointer, ULONG number_blocks,
                                       ULONG lba, ULONG *media_status)
{

UINT status             = UX_SUCCESS;
UCHAR *memory_pointer   = UX_NULL;

    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(media_status);

    if (lun == 0)
        memory_pointer = ram_disk1_memory;
    else if (lun == 1)
        memory_pointer = ram_disk2_memory;
    else
        return UX_ERROR;

    ux_utility_memory_copy(data_pointer, memory_pointer + lba * RAM_DISK_BLOCK_LENGTH,
                           number_blocks * RAM_DISK_BLOCK_LENGTH);

    return(status);
}

/************************************************************************************************/
/**  ux_demo_device_storage_media_write                                                         */
/************************************************************************************************/
static UINT ux_demo_device_storage_media_write(VOID *storage, ULONG lun, UCHAR *data_pointer, ULONG number_blocks,
                                               ULONG lba, ULONG *media_status)
{

UINT status             = UX_SUCCESS;
UCHAR *memory_pointer   = UX_NULL;

    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(media_status);

    if (lun == 0)
        memory_pointer = ram_disk1_memory;
    else if (lun == 1)
        memory_pointer = ram_disk2_memory;
    else
        return UX_ERROR;

    ux_utility_memory_copy(memory_pointer + lba * RAM_DISK_BLOCK_LENGTH, data_pointer,
                           number_blocks * RAM_DISK_BLOCK_LENGTH);

    return(status);
}

/************************************************************************************************/
/**  ux_demo_device_storage_media_status                                                        */
/************************************************************************************************/
static UINT ux_demo_device_storage_media_status(VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status)
{
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(media_id);
    UX_PARAMETER_NOT_USED(media_status);

    return(UX_SUCCESS);
}

/************************************************************************************************/
/**  ux_demo_device_storage_media_flush                                                         */
/************************************************************************************************/
static UINT ux_demo_device_storage_media_flush(VOID *storage, ULONG lun, ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(number_blocks);
    UX_PARAMETER_NOT_USED(lba);
    UX_PARAMETER_NOT_USED(media_status);

    return(UX_SUCCESS);
}

/************************************************************************************************/
/**  ux_demo_device_storage_media_notification                                                  */
/************************************************************************************************/
static UINT ux_demo_device_storage_media_notification(VOID *storage,  ULONG lun, ULONG media_id, ULONG notification_class,
                                                      UCHAR **media_notification, ULONG *media_notification_length)
{

    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(media_id);
    UX_PARAMETER_NOT_USED(notification_class);
    UX_PARAMETER_NOT_USED(media_notification);
    UX_PARAMETER_NOT_USED(media_notification_length);

    return(UX_SUCCESS);
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

    ux_storage_disk_init();
}

/************************************************************************************************/
/**  usbx_storage_disk_init                                                                     */
/************************************************************************************************/

static VOID ux_storage_disk_init(VOID)
{
    ux_utility_memory_set(ram_disk1_memory, 0, RAM_DISK_SIZE);
    ux_utility_memory_set(ram_disk2_memory, 0, RAM_DISK_SIZE);

#ifdef BUILD_FILE_SYSTEM

    ux_utility_memory_copy(ram_disk1_memory + 0 * 512, (void*)sector_0, sizeof(sector_0));
    ux_utility_memory_copy(ram_disk2_memory + 0 * 512, (void*)sector_0, sizeof(sector_0));

    /* Set unique volume labels per LUN in boot sector (offset 0x2B). */
    ux_utility_memory_copy(ram_disk1_memory + 0x2B, (void*)volume_label_lun1, sizeof(volume_label_lun1));
    ux_utility_memory_copy(ram_disk2_memory + 0x2B, (void*)volume_label_lun2, sizeof(volume_label_lun2));

    /* Update number of sectors.  */
    if (RAM_DISK_N_LBA >= 0x100)
    {
        ux_utility_short_put(ram_disk1_memory + 0x13, 0);
        ux_utility_long_put(ram_disk1_memory + 0x20, RAM_DISK_N_LBA);
        ux_utility_short_put(ram_disk2_memory + 0x13, 0);
        ux_utility_long_put(ram_disk2_memory + 0x20, RAM_DISK_N_LBA);
    }
    else
    {
        ux_utility_short_put(ram_disk1_memory + 0x13, RAM_DISK_N_LBA);
        ux_utility_long_put(ram_disk1_memory + 0x20, 0);
        ux_utility_short_put(ram_disk2_memory + 0x13, RAM_DISK_N_LBA);
        ux_utility_long_put(ram_disk2_memory + 0x20, 0);
    }

    ux_utility_memory_copy(ram_disk1_memory + 6 * 512, (void*)sector_6_7, sizeof(sector_6_7));
    ux_utility_memory_copy(ram_disk1_memory + 7 * 512, (void*)sector_6_7, sizeof(sector_6_7));
    ux_utility_memory_copy(ram_disk1_memory + 8 * 512, (void*)sector_8, sizeof(sector_8));

    ux_utility_memory_copy(ram_disk2_memory + 6 * 512, (void*)sector_6_7, sizeof(sector_6_7));
    ux_utility_memory_copy(ram_disk2_memory + 7 * 512, (void*)sector_6_7, sizeof(sector_6_7));
    ux_utility_memory_copy(ram_disk2_memory + 8 * 512, (void*)sector_8, sizeof(sector_8));

    /* Root directory volume label entry (first entry, 32 bytes). */
    ux_utility_memory_set(ram_disk1_memory + 8 * 512, 0, 32);
    ux_utility_memory_copy(ram_disk1_memory + 8 * 512, (void*)volume_label_lun1, sizeof(volume_label_lun1));
    *(ram_disk1_memory + 8 * 512 + 0x0B) = 0x08; /* Attribute: Volume Label */

    ux_utility_memory_set(ram_disk2_memory + 8 * 512, 0, 32);
    ux_utility_memory_copy(ram_disk2_memory + 8 * 512, (void*)volume_label_lun2, sizeof(volume_label_lun2));
    *(ram_disk2_memory + 8 * 512 + 0x0B) = 0x08; /* Attribute: Volume Label */

#endif /* BUILD_FILE_SYSTEM */
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
