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

#ifndef UX_DEVICE_SIDE_ONLY
#error UX_DEVICE_SIDE_ONLY must be defined
#endif

#ifndef UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT
#error UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT must be defined for this sample
#endif

#ifndef UX_STANDALONE
#warning UX_STANDALONE must be define for this sample.
#endif

#if UX_PERIODIC_RATE != 1000
#warning UX_PERIODIC_RATE should be 1000 for 1ms tick.
#endif

#define BUILD_FILE_SYSTEM

/**************************************************/
/**  Define constants                             */
/**************************************************/
#define UX_DEVICE_MEMORY_STACK_SIZE             (10*1024)

#define UX_DEMO_STORAGE_DEVICE_VID              0x070A
#define UX_DEMO_STORAGE_DEVICE_PID              0x4030

#define UX_DEMO_BCD_USB                         0x0200
#define UX_DEMO_MAX_EP0_SIZE                    0x40U
#define UX_DEMO_STORAGE_CONFIG_DESC_SIZE        0x20U

#define UX_DEMO_STORAGE_EPIN_ADDR               0x81U
#define UX_DEMO_STORAGE_EPOUT_ADDR              0x01U
#define UX_DEMO_STORAGE_FS_EPIN_SIZE            0x40U
#define UX_DEMO_STORAGE_FS_EPOUT_SIZE           0x40U
#define UX_DEMO_STORAGE_HS_EPIN_SIZE            0x200U
#define UX_DEMO_STORAGE_HS_EPOUT_SIZE           0x200U

/**************************************************/
/**  usbx device storage demo callbacks           */
/**************************************************/
VOID ux_demo_device_storage_instance_activate(VOID *storage_instance);
VOID ux_demo_device_storage_instance_deactivate(VOID *storage_instance);
UINT ux_demo_device_storage_media_read(VOID *storage_instance, ULONG lun, UCHAR * data_pointer, ULONG number_blocks,
                                       ULONG lba, ULONG *media_status);
UINT ux_demo_device_storage_media_write(VOID *storage_instance, ULONG lun, UCHAR * data_pointer, ULONG number_blocks,
                                        ULONG lba, ULONG *media_status);
UINT ux_demo_device_storage_media_status(VOID *storage_instance, ULONG lun, ULONG media_id, ULONG *media_status);
UINT ux_demo_device_storage_media_flush(VOID *storage, ULONG lun, ULONG number_blocks, ULONG lba, ULONG *media_status);

/**************************************************/
/**  usbx device storage demo                     */
/**************************************************/
#ifndef RAM_DISK_SIZE
#define RAM_DISK_SIZE           (1024 * 90) /* Must > 20K for windows.  */
#endif

#define RAM_DISK_N_LBA          (RAM_DISK_SIZE / 512)
#define RAM_DISK_LAST_LBA       (RAM_DISK_N_LBA - 1)
#define RAM_DISK_BLOCK_LENGTH   512

#define BUILD_FILE_SYSTEM

/* Write Caching support.  */
#define RAM_DISK_WRITE_CACHING  UX_FALSE

UCHAR ram_disk_memory[RAM_DISK_SIZE];
static VOID usbx_storage_disk_init(VOID);

/**************************************************/
/**  usbx device storage instance                 */
/**************************************************/
UX_SLAVE_CLASS_STORAGE *storage;

/**************************************************/
/**  usbx callback error                          */
/**************************************************/
static VOID ux_demo_error_callback(UINT system_level, UINT system_context, UINT error_code);

VOID ux_application_define(VOID);

static CHAR ux_system_memory_pool[UX_DEVICE_MEMORY_STACK_SIZE];

#ifndef EXTERNAL_MAIN
extern int board_setup(void);
#endif /* EXTERNAL_MAIN */
extern int usb_device_dcd_initialize(void *param);

/**************************************************/
/**  USB Report descriptor                        */
/**************************************************/

#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED sizeof(ux_demo_device_framework_full_speed)

UCHAR ux_demo_device_framework_full_speed[] = {
    /* Device descriptor */
    0x12,                       /* bLength */
    0x01,                       /* bDescriptorType */
    UX_W0(UX_DEMO_BCD_USB), UX_W1(UX_DEMO_BCD_USB), /* bcdUSB */
    0x00,                       /* bDeviceClass : 0x00 : Interface-defined */
    0x00,                       /* bDeviceSubClass : 0x00 : Reset */
    0x00,                       /* bDeviceProtocol : 0x00 : Reset */
    UX_DEMO_MAX_EP0_SIZE,       /* bMaxPacketSize0 */
    UX_W0(UX_DEMO_STORAGE_DEVICE_VID), UX_W1(UX_DEMO_STORAGE_DEVICE_VID), /* idVendor */
    UX_W0(UX_DEMO_STORAGE_DEVICE_PID), UX_W1(UX_DEMO_STORAGE_DEVICE_PID), /* idProduct */
    0x00, 0x00,                 /* bcdDevice */
    0x01,                       /* iManufacturer */
    0x02,                       /* iProduct */
    0x03,                       /* iSerialNumber */
    0x01,                       /* bNumConfigurations */

    /* Configuration descriptor */
    0x09,                       /* bLength */
    0x02,                       /* bDescriptorType */
    UX_W0(UX_DEMO_STORAGE_CONFIG_DESC_SIZE), UX_W1(UX_DEMO_STORAGE_CONFIG_DESC_SIZE), /* wTotalLength */
    0x01,                       /* bNumInterfaces */
    0x01,                       /* bConfigurationValue */
    0x04,                       /* iConfiguration */
    0xC0,                       /* bmAttributes */
                                /* D6 : 0x1 : Self-powered */
                                /* D5, Remote Wakeup : 0x0 : Not supported */
    0x32,                       /* bMaxPower : 50 : 100mA */

    /* Interface descriptor */
    0x09,                       /* bLength */
    0x04,                       /* bDescriptorType */
    0x00,                       /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x02,                       /* bNumEndpoints */
    0x08,                       /* bInterfaceClass : 0x08 : Mass Storage */
    0x06,                       /* bInterfaceSubClass : SCSI transparent*/
    0x50,                       /* nInterfaceProtocol */
    0x06,                       /* iInterface */

    /* Endpoint descriptor (Bulk In) */
    0x07,                       /* bLength */
    0x05,                       /* bDescriptorType */
    UX_DEMO_STORAGE_EPIN_ADDR,  /* bEndpointAddress */
    0x02,                       /* bmAttributes */
                                  /* D1..0, Transfer Type : 0x2 : Bulk */
                                  /* D3..2, Synchronization Type : 0x0 : No Synchronization */
                                  /* D5..4, Usage Type : 0x0 : Data endpoint */
    UX_W0(UX_DEMO_STORAGE_FS_EPIN_SIZE), UX_W1(UX_DEMO_STORAGE_FS_EPIN_SIZE),  /* wMaxPacketSize */
                                  /* D10..0, Max Packet Size */
                                  /* D12..11, Additional transactions : 0x00 */
    0x00,                        /* bInterval */

    /* Endpoint descriptor (Bulk Out) */
    0x07,                       /* bLength */
    0x05,                       /* bDescriptorType */
    UX_DEMO_STORAGE_EPOUT_ADDR, /* bEndpointAddress */
    0x02,                       /* bmAttributes */
                                  /* D1..0, Transfer Type : 0x2 : Bulk */
                                  /* D3..2, Synchronization Type : 0x0 : No Synchronization */
                                  /* D5..4, Usage Type : 0x0 : Data endpoint */
    UX_W0(UX_DEMO_STORAGE_FS_EPOUT_SIZE), UX_W1(UX_DEMO_STORAGE_FS_EPOUT_SIZE),  /* wMaxPacketSize */
                                  /* D10..0, Max Packet Size */
                                  /* D12..11, Additional transactions : 0x00 */
    0x00                        /* bInterval */
};

#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED sizeof(ux_demo_device_framework_high_speed)
UCHAR ux_demo_device_framework_high_speed[] = {
    /* Device descriptor */
    0x12,                       /* bLength */
    0x01,                       /* bDescriptorType */
    UX_W0(UX_DEMO_BCD_USB), UX_W1(UX_DEMO_BCD_USB), /* bcdUSB */
    0x00,                       /* bDeviceClass : 0x00 : Interface-defined */
    0x00,                       /* bDeviceSubClass : 0x00 : Reset */
    0x00,                       /* bDeviceProtocol : 0x00 : Reset */
    UX_DEMO_MAX_EP0_SIZE,       /* bMaxPacketSize0 */
    UX_W0(UX_DEMO_STORAGE_DEVICE_VID), UX_W1(UX_DEMO_STORAGE_DEVICE_VID), /* idVendor */
    UX_W0(UX_DEMO_STORAGE_DEVICE_PID), UX_W1(UX_DEMO_STORAGE_DEVICE_PID), /* idProduct */
    0x01, 0x00,                 /* bcdDevice */
    0x01,                       /* iManufacturer */
    0x02,                       /* iProduct */
    0x03,                       /* iSerialNumber */
    0x01,                       /* bNumConfigurations */

    /* Device qualifier descriptor */
    0x0A,                       /* bLength */
    0x06,                       /* bDescriptorType */
    UX_W0(UX_DEMO_BCD_USB), UX_W1(UX_DEMO_BCD_USB), /* bcdUSB */
    0x00,                       /* bDeviceClass : 0x00 : Interface-defined */
    0x00,                       /* bDeviceSubClass : 0x00 : Reset */
    0x00,                       /* bDeviceProtocol : 0x00 : Reset */
    UX_DEMO_MAX_EP0_SIZE,       /* bMaxPacketSize0 */
    0x01,                       /* bNumConfigurations */
    0x00,                       /* bReserved */

    /* Configuration descriptor */
    0x09,                       /* bLength */
    0x02,                       /* bDescriptorType */
    UX_W0(UX_DEMO_STORAGE_CONFIG_DESC_SIZE), UX_W1(UX_DEMO_STORAGE_CONFIG_DESC_SIZE), /* wTotalLength */
    0x01,                       /* bNumInterfaces */
    0x01,                       /* bConfigurationValue */
    0x05,                       /* iConfiguration */
    0xC0,                       /* bmAttributes */
                                    /* D6 : 0x1 : Self-powered */
                                    /* D5, Remote Wakeup : 0x0 : Not supported */
    0x19,                       /* bMaxPower : 50 : 100mA */

    /* Interface descriptor */
    0x09,                       /* bLength */
    0x04,                       /* bDescriptorType */
    0x00,                       /* bInterfaceNumber */
    0x00,                       /* bAlternateSetting */
    0x02,                       /* bNumEndpoints */
    0x08,                       /* bInterfaceClass : 0x08 : Mass Storage */
    0x06,                       /* bInterfaceSubClass : SCSI transparent*/
    0x50,                       /* nInterfaceProtocol */
    0x06,                       /* iInterface */

    /* Endpoint descriptor (Bulk In) */
    0x07,                       /* bLength */
    0x05,                       /* bDescriptorType */
    UX_DEMO_STORAGE_EPIN_ADDR,  /* bEndpointAddress */
    0x02,                       /* bmAttributes */
                                  /* D1..0, Transfer Type : 0x2 : Bulk */
                                  /* D3..2, Synchronization Type : 0x0 : No Synchronization */
                                  /* D5..4, Usage Type : 0x0 : Data endpoint */
    UX_W0(UX_DEMO_STORAGE_HS_EPIN_SIZE), UX_W1(UX_DEMO_STORAGE_HS_EPIN_SIZE),  /* wMaxPacketSize */
                                  /* D10..0, Max Packet Size */
                                  /* D12..11, Additional transactions : 0x00 */
    0x00,                        /* bInterval */

    /* Endpoint descriptor (Bulk Out) */
    0x07,                       /* bLength */
    0x05,                       /* bDescriptorType */
    UX_DEMO_STORAGE_EPOUT_ADDR, /* bEndpointAddress */
    0x02,                       /* bmAttributes */
                                    /* D1..0, Transfer Type : 0x2 : Bulk */
                                    /* D3..2, Synchronization Type : 0x0 : No Synchronization */
                                    /* D5..4, Usage Type : 0x0 : Data endpoint */
    UX_W0(UX_DEMO_STORAGE_HS_EPOUT_SIZE), UX_W1(UX_DEMO_STORAGE_HS_EPOUT_SIZE),  /* wMaxPacketSize */
                                    /* D10..0, Max Packet Size */
                                    /* D12..11, Additional transactions : 0x00 */
    0x00                        /* bInterval */
};


/* String Device Framework :
   Byte 0 and 1 : Word containing the language ID : 0x0904 for US
   Byte 2       : Byte containing the index of the descriptor
   Byte 3       : Byte containing the length of the descriptor string
*/
#define STRING_FRAMEWORK_LENGTH sizeof(ux_demo_string_framework)
UCHAR ux_demo_string_framework[] = {

    /* iManufacturer string descriptor : Index 1 */
    0x09, 0x04, 0x01, 12,
    'U', 'S', 'B', 'X', ' ', 'e', 'c', 'l', 'i', 'p', 's', 'e',

    /* iProduct string descriptor : Index 2 */
    0x09, 0x04, 0x02, 20,
    'S', 'T', 'O', 'R', 'A', 'G', 'E', ' ', 'R', 'A', 'M', 'D', 'I', 'S', 'K', ' ', 'D', 'e', 'm', 'o',

    /* iSerialNumber Number string descriptor : Index 3 */
    0x09, 0x04, 0x03, 13,
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1',

    /* iConfiguration string descriptor : Index 4 */
    0x09, 0x04, 0x04, 10,
    'F', 'U', 'L', 'L', ' ', 'S', 'P', 'E', 'E', 'D',

    /* iConfiguration string descriptor : Index 5 */
    0x09, 0x04, 0x05, 10,
    'H', 'I', 'G', 'H', ' ', 'S', 'P', 'E', 'E', 'D',

    /* iInterface string descriptor : Index 6 */
    0x09, 0x04, 0x06, 15,
    'R', 'e', 'm', 'o', 'v', 'a', 'b', 'l', 'e', ' ', 'D', 'r', 'i', 'v', 'e'
};


/* Multiple languages are supported on the device, to add  a language besides english,
   the unicode language code must be appended to the ux_demo_language_id_framework array and the length
   adjusted accordingly.
*/
#define LANGUAGE_ID_FRAMEWORK_LENGTH sizeof(ux_demo_language_id_framework)
UCHAR ux_demo_language_id_framework[] = {
    /* English. */
    0x09, 0x04
};

#ifdef BUILD_FILE_SYSTEM
static UCHAR sector_0[] = {
	0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30, 0x00,
	0x02, 0x01, 0x06, 0x00, 0x02, 0x00, 0x02,
    /*0x13: small sectors*/0xF0, 0x00,
    0xF8, 0x01, 0x00,
	0x01, 0x00, 0x01,
    /*0x20: large sectors*/0x00, 0x00, 0x00, 0x00,
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
#endif /* BUILD_FILE_SYSTEM */

#ifndef EXTERNAL_MAIN
int main(void)
{
    /* Initialize the board.  */
    board_setup();

    ux_application_define();

    while (1)
    {
        ux_system_tasks_run();
    }
}
#endif /* EXTERNAL_MAIN */

VOID ux_application_define(VOID)
{
CHAR                               *memory_pointer;
UINT                               status;
UX_SLAVE_CLASS_STORAGE_PARAMETER   storage_parameter;
UX_SLAVE_CLASS_STORAGE_LUN         *lun;

    /* Use static memory block.  */
    memory_pointer = ux_system_memory_pool;

    /* Initialize USBX Memory */
    status = ux_system_initialize(memory_pointer, UX_DEVICE_MEMORY_STACK_SIZE, UX_NULL, 0);

    if(status != UX_SUCCESS)
        return;

    /* Install the device portion of USBX.  */
    status =  ux_device_stack_initialize(ux_demo_device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                         ux_demo_device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                         ux_demo_string_framework, STRING_FRAMEWORK_LENGTH,
                                         ux_demo_language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH,
                                         UX_NULL);

    if(status != UX_SUCCESS)
        return;

    /* Store the number of LUN in this device storage instance.  */
    storage_parameter.ux_slave_class_storage_instance_activate = ux_demo_device_storage_instance_activate;
    storage_parameter.ux_slave_class_storage_instance_deactivate = ux_demo_device_storage_instance_deactivate;
    storage_parameter.ux_slave_class_storage_parameter_number_lun = 1;

     /* Initialize the storage class parameters for reading/writing to the Flash Disk.  */
    lun = &storage_parameter.ux_slave_class_storage_parameter_lun[0];
    lun -> ux_slave_class_storage_media_last_lba = RAM_DISK_LAST_LBA;
    lun -> ux_slave_class_storage_media_block_length = RAM_DISK_BLOCK_LENGTH;
    lun -> ux_slave_class_storage_media_type = 0;
    lun -> ux_slave_class_storage_media_removable_flag = 0x80;
    lun -> ux_slave_class_storage_media_read_only_flag = UX_FALSE;
    lun -> ux_slave_class_storage_media_read = ux_demo_device_storage_media_read;
    lun -> ux_slave_class_storage_media_write = ux_demo_device_storage_media_write;
    lun -> ux_slave_class_storage_media_status = ux_demo_device_storage_media_status;
    lun -> ux_slave_class_storage_media_flush = RAM_DISK_WRITE_CACHING ? ux_demo_device_storage_media_flush : UX_NULL;

    /* Initialize the device storage class. The class is connected with interface 0 on configuration 1. */
    status = ux_device_stack_class_register(_ux_system_slave_class_storage_name, _ux_device_class_storage_entry,
                                            1, 0, (VOID *)&storage_parameter);

    if(status != UX_SUCCESS)
        return;

    /* Register error callback */
    ux_utility_error_callback_register(ux_demo_error_callback);

    /* Register the USB device controllers available in this system.  */
    usb_device_dcd_initialize(UX_NULL);

    usbx_storage_disk_init();
}

/********************************************************************/
/**  ux_demo_device_storage_instance_activate                       */
/********************************************************************/
VOID ux_demo_device_storage_instance_activate(VOID *storage_instance)
{
    if (storage == UX_NULL)
        storage = (UX_SLAVE_CLASS_STORAGE*) storage_instance;
}

/********************************************************************/
/**  ux_demo_device_storage_instance_deactivate                     */
/********************************************************************/
VOID ux_demo_device_storage_instance_deactivate(VOID *storage_instance)
{
    if (storage_instance == (VOID *)storage)
        storage = UX_NULL;
}

/********************************************************************/
/**  ux_demo_device_storage_media_read                               */
/********************************************************************/
UINT ux_demo_device_storage_media_read(VOID *storage, ULONG lun, UCHAR *data_pointer, ULONG number_blocks,
                                       ULONG lba, ULONG *media_status)
{
UINT status = UX_SUCCESS;

    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(media_status);

    ux_utility_memory_copy(data_pointer,
                           ram_disk_memory + lba * RAM_DISK_BLOCK_LENGTH,
                           number_blocks * RAM_DISK_BLOCK_LENGTH);

    /* Time consuming operation returns state status.  */
    if(status == UX_SUCCESS)
        return(UX_STATE_NEXT);

    return(UX_STATE_ERROR);
}

/********************************************************************/
/**  ux_demo_device_storage_media_write                             */
/********************************************************************/
UINT ux_demo_device_storage_media_write(VOID *storage, ULONG lun, UCHAR *data_pointer, ULONG number_blocks,
                                        ULONG lba, ULONG *media_status)
{
UINT status =  UX_SUCCESS;

    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(media_status);

    ux_utility_memory_copy(ram_disk_memory + lba * RAM_DISK_BLOCK_LENGTH,
                           data_pointer,
                           number_blocks * RAM_DISK_BLOCK_LENGTH);

    /* Time consuming operation returns state status.  */
    if(status == UX_SUCCESS)
        return(UX_STATE_NEXT);

    return(UX_STATE_ERROR);
}

/********************************************************************/
/**  ux_demo_device_storage_media_status                            */
/********************************************************************/
UINT ux_demo_device_storage_media_status(VOID *storage, ULONG lun, ULONG media_id, ULONG *media_status)
{
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(media_id);
    UX_PARAMETER_NOT_USED(media_status);

    return(UX_SUCCESS);
}

/********************************************************************/
/**  ux_demo_device_storage_media_flush                             */
/********************************************************************/
UINT ux_demo_device_storage_media_flush(VOID *storage, ULONG lun, ULONG number_blocks, ULONG lba, ULONG *media_status)
{
    UX_PARAMETER_NOT_USED(storage);
    UX_PARAMETER_NOT_USED(lun);
    UX_PARAMETER_NOT_USED(number_blocks);
    UX_PARAMETER_NOT_USED(lba);
    UX_PARAMETER_NOT_USED(media_status);

    /* The ATA drive never fails. This is just for usbx only !!!! */

    /* Time consuming operation returns state status.  */
    return(UX_STATE_NEXT);
}

/********************************************************************/
/**  usbx_storage_disk_init                                         */
/********************************************************************/
static VOID usbx_storage_disk_init(VOID)
{
    ux_utility_memory_set(ram_disk_memory, 0, RAM_DISK_SIZE);

#ifdef BUILD_FILE_SYSTEM

    ux_utility_memory_copy(ram_disk_memory + 0 * 512, (void*)sector_0, sizeof(sector_0));

    /* Update number of sectors.  */
    if (RAM_DISK_N_LBA >= 0x100)
    {
        ux_utility_short_put(ram_disk_memory + 0x13, 0);
        ux_utility_long_put(ram_disk_memory + 0x20, RAM_DISK_N_LBA);
    }
    else
    {
        ux_utility_short_put(ram_disk_memory + 0x13, RAM_DISK_N_LBA);
        ux_utility_long_put(ram_disk_memory + 0x20, 0);
    }

    ux_utility_memory_copy(ram_disk_memory + 6 * 512, (void*)sector_6_7, sizeof(sector_6_7));
    ux_utility_memory_copy(ram_disk_memory + 7 * 512, (void*)sector_6_7, sizeof(sector_6_7));
    ux_utility_memory_copy(ram_disk_memory + 8 * 512, (void*)sector_8, sizeof(sector_8));

#endif /* BUILD_FILE_SYSTEM */
}

static VOID ux_demo_error_callback(UINT system_level, UINT system_context, UINT error_code)
{
    /*
     * Refer to ux_api.h. For example,
     * UX_SYSTEM_LEVEL_INTERRUPT, UX_SYSTEM_CONTEXT_DCD, UX_DEVICE_HANDLE_UNKNOWN
     */
    printf("USBX error: system level(%d), context(%d), error code(0x%x)\r\n", system_level, system_context, error_code);
}
