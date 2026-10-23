const optionalEndpointState = {};
const audioAcEpState = {};
let usedStringIndexes = new Map();
let customStringDescriptors = [];
let customStringNextId = 1;

const STRING_LANGUAGE_OPTIONS = [
    { id: 'lang-en-us', code: 0x0409, name: 'English (US)' },
    { id: 'lang-en-uk', code: 0x0809, name: 'English (UK)' },
    { id: 'lang-fr', code: 0x040C, name: 'French' },
    { id: 'lang-de', code: 0x0407, name: 'German' },
    { id: 'lang-es', code: 0x0C0A, name: 'Spanish' },
    { id: 'lang-it', code: 0x0410, name: 'Italian' },
    { id: 'lang-ja', code: 0x0411, name: 'Japanese' },
    { id: 'lang-zh-cn', code: 0x0804, name: 'Chinese (Simplified)' }
];

const STRING_FIELD_GROUPS = [
    {
        wrapperId: 'manufacturer-string-group',
        title: 'Manufacturer String (iManufacturer)',
        indexFieldId: 'iManufacturer',
        stringFieldId: 'manufacturer',
        languageFieldId: 'manufacturerLanguageIdDisplay',
        fixedIndex: '1',
        stringHelp: 'Manufacturer name'
    },
    {
        wrapperId: 'product-string-group',
        title: 'Product String (iProduct)',
        indexFieldId: 'iProduct',
        stringFieldId: 'product',
        languageFieldId: 'productLanguageIdDisplay',
        fixedIndex: '2',
        stringHelp: 'Product name'
    },
    {
        wrapperId: 'serial-string-group',
        title: 'Serial Number String (iSerialNumber)',
        indexFieldId: 'iSerialNumber',
        stringFieldId: 'serialNumber',
        languageFieldId: 'serialNumberLanguageIdDisplay',
        fixedIndex: '3',
        stringHelp: 'Serial number'
    }
];

const CONFIGURATION_STRING_FIELD_GROUPS = [
    {
        wrapperId: 'configuration-fs-string-group',
        title: 'Configuration String - Full Speed (iConfiguration)',
        indexFieldId: 'iConfiguration',
        stringFieldId: 'configurationStringFS',
        languageFieldId: 'configurationFsLanguageIdDisplay',
        stringHelp: 'Full Speed configuration string'
    },
    {
        wrapperId: 'configuration-hs-string-group',
        title: 'Configuration String - High Speed (iConfiguration)',
        indexFieldId: 'iConfigurationHS',
        stringFieldId: 'configurationStringHS',
        languageFieldId: 'configurationHsLanguageIdDisplay',
        stringHelp: 'High Speed configuration string',
        conditionalField: 'speed-high',
        conditionalOperator: 'checked'
    }
];

const CLASS_ORDER_DEFAULT = [
    'hid',
    'msc',
    'dfu',
    'printer',
    'video',
    'mtp',
    'ptp',
    'cdc',
    'rndis',
    'ecm',
    'audio',
    'audio2'
];

const CLASS_CARD_CONFIG = {
    hid: { cardId: 'hidDescriptorCard', collapseId: 'collapseHID', title: 'HID Configuration', checkboxId: 'class-hid' },
    msc: { cardId: 'massStorageDescriptorCard', collapseId: 'collapseMSC', title: 'Mass Storage Configuration', checkboxId: 'class-msc' },
    dfu: { cardId: 'dfuDescriptorCard', collapseId: 'collapseDFU', title: 'DFU Configuration', checkboxId: 'class-dfu' },
    printer: { cardId: 'printerDescriptorCard', collapseId: 'collapsePrinter', title: 'Printer Configuration', checkboxId: 'class-printer' },
    video: { cardId: 'videoDescriptorCard', collapseId: 'collapseVideo', title: 'Video Configuration', checkboxId: 'class-video' },
    mtp: { cardId: 'mtpDescriptorCard', collapseId: 'collapseMTP', title: 'MTP Configuration', checkboxId: 'class-mtp' },
    ptp: { cardId: 'ptpDescriptorCard', collapseId: 'collapsePTP', title: 'PTP Configuration', checkboxId: 'class-ptp' },
    cdc: { cardId: 'cdcDescriptorCard', collapseId: 'collapseCDC', title: 'CDC ACM Configuration', checkboxId: 'class-cdc' },
    rndis: { cardId: 'cdcRndisDescriptorCard', collapseId: 'collapseRNDIS', title: 'CDC RNDIS Configuration', checkboxId: 'class-rndis' },
    ecm: { cardId: 'cdcEcmDescriptorCard', collapseId: 'collapseECM', title: 'CDC ECM Configuration', checkboxId: 'class-ecm' },
    audio: { cardId: 'audioDescriptorCard', collapseId: 'collapseAudio', title: 'Audio 1.0 Configuration', checkboxId: 'class-audio' },
    audio2: { cardId: 'audio2DescriptorCard', collapseId: 'collapseAudio2', title: 'Audio 2.0 Configuration', checkboxId: 'class-audio2' }
};

let classCardOrder = CLASS_ORDER_DEFAULT.slice();

const PAGE_SCHEMA = {
    fields: [
        {
            id: 'bcdUSB',
            label: 'USB Specification Version (bcdUSB)',
            help: 'USB Specification Release Number in\nBinary-Coded Decimal (i.e., 2.10 is 210H).\nThis field identifies the release of the USB\nSpecification with which the device and its\ndescriptors are compliant.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0110', text: 'USB 1.1 (0x0110)' },
                { value: '0x0200', text: 'USB 2.0 (0x0200)' }
            ]
        },
        {
            id: 'bDeviceClass',
            label: 'Device Class (bDeviceClass)',
            help: 'Class code (assigned by the USB-IF).\nIf this field is reset to zero, each interface\nwithin a configuration specifies its own\nclass information and the various\ninterfaces operate independently.\nIf this field is set to a value between 1 and\nFEH, the device supports different class\nspecifications on different interfaces and\nthe interfaces may not operate\nindependently.  This value identifies the\nclass definition used for the aggregate\ninterfaces.\nIf this field is set to FFH, the device class\nis vendor-specific',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x00',
            options: [
                { value: '0x00', text: 'Use Interface Descriptors (0x00)' },
                { value: '0x01', text: 'Audio (0x01)' },
                { value: '0x02', text: 'CDC - Communications (0x02)' },
                { value: '0x03', text: 'HID - Human Interface (0x03)' },
                { value: '0x05', text: 'Physical (0x05)' },
                { value: '0x06', text: 'Image (0x06)' },
                { value: '0x07', text: 'Printer (0x07)' },
                { value: '0x08', text: 'Mass Storage (0x08)' },
                { value: '0x09', text: 'Hub (0x09)' },
                { value: '0x0A', text: 'CDC-Data (0x0A)' },
                { value: '0x0B', text: 'Smart Card (0x0B)' },
                { value: '0x0D', text: 'Content Security (0x0D)' },
                { value: '0x0E', text: 'Video (0x0E)' },
                { value: '0x0F', text: 'Personal Healthcare (0x0F)' },
                { value: '0xDC', text: 'Diagnostic Device (0xDC)' },
                { value: '0xE0', text: 'Wireless Controller (0xE0)' },
                { value: '0xEF', text: 'Miscellaneous (0xEF)' },
                { value: '0xFE', text: 'Application Specific (0xFE)' },
                { value: '0xFF', text: 'Vendor Specific (0xFF)' }
            ]
        },
        {
            id: 'bDeviceSubClass',
            label: 'Device SubClass (bDeviceSubClass)',
            help: 'Subclass code (assigned by the USB-IF).\nThese codes are qualified by the value of\nthe bDeviceClass field.\nIf the bDeviceClass field is reset to zero,\nthis field must also be reset to zero.\nIf the bDeviceClass field is not set to FFH,\nall values are reserved for assignment by\nthe USB-IF.',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x00',
            placeholder: '0x00'
        },
        {
            id: 'bDeviceProtocol',
            label: 'Device Protocol (bDeviceProtocol)',
            help: 'Protocol code (assigned by the USB-IF).\nThese codes are qualified by the value of\nthe bDeviceClass and the bDeviceSubClass fields.  If a device\nsupports class-specific protocols on a\ndevice basis as opposed to an interface\nbasis, this code identifies the protocols \nthat the device uses as defined by the \nspecification of the device class. \nIf this field is reset to zero, the device \ndoes not use class-specific protocols on a \ndevice basis.  However, it may use class \nspecific protocols on an interface basis. \nIf this field is set to FFH, the device uses a \nvendor-specific protocol on a device basis.',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x00',
            placeholder: '0x00'
        },
        {
            id: 'bMaxPacketSize0',
            label: 'Max Packet Size EP0 - Full Speed (bMaxPacketSize0)',
            help: 'Maximum packet size for endpoint zero\n(only 8, 16, 32, or 64 are valid)',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x08',
            options: [
                { value: '0x08', text: '8 bytes' },
                { value: '0x10', text: '16 bytes' },
                { value: '0x20', text: '32 bytes' },
                { value: '0x40', text: '64 bytes' }
            ]
        },
        {
            id: 'bMaxPacketSize0HS',
            label: 'Max Packet Size EP0 - High Speed (bMaxPacketSize0)',
            help: 'Maximum packet size for endpoint zero\n(only 8, 16, 32, or 64 are valid)',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x40',
            conditionalField: 'speed-high',
            conditionalOperator: 'checked',
            options: [
                { value: '0x08', text: '8 bytes' },
                { value: '0x10', text: '16 bytes' },
                { value: '0x20', text: '32 bytes' },
                { value: '0x40', text: '64 bytes' }
            ]
        },
        {
            id: 'idVendor',
            label: 'Vendor ID (idVendor)',
            help: 'Vendor ID (assigned by the USB-IF)',
            type: 'text',
            format: 'hex',
            bytes: 2,
            default: '0x1A0A',
            placeholder: '0x1A0A'
        },
        {
            id: 'idProduct',
            label: 'Product ID (idProduct)',
            help: 'Product ID (assigned by the manufacturer)',
            type: 'text',
            format: 'hex',
            bytes: 2,
            default: '0x5740',
            placeholder: '0x5740'
        },
        {
            id: 'bcdDevice',
            label: 'Device Release Number (bcdDevice)',
            help: 'Device release number in binary-coded decimal',
            type: 'text',
            format: 'hex',
            bytes: 2,
            default: '0x0100',
            placeholder: '0x0100'
        },
        {
            id: 'iManufacturer',
            label: 'Manufacturer String Index (iManufacturer)',
            help: '0 = no string',
            type: 'number',
            format: 'number',
            default: 1,
            min: 0,
            max: 255
        },
        {
            id: 'manufacturer',
            label: 'Manufacturer String',
            help: 'Manufacturer name',
            type: 'text',
            format: 'string',
            default: 'Eclipse Threadx',
            placeholder: 'Manufacturer name',
            conditionalField: 'iManufacturer',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        },
        {
            id: 'iProduct',
            label: 'Product String Index (iProduct)',
            help: '0 = no string',
            type: 'number',
            format: 'number',
            default: 2,
            min: 0,
            max: 255
        },
        {
            id: 'product',
            label: 'Product String',
            help: 'Product name',
            type: 'text',
            format: 'string',
            default: 'USBX',
            placeholder: 'Product name',
            conditionalField: 'iProduct',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        },
        {
            id: 'iSerialNumber',
            label: 'Serial Number String Index (iSerialNumber)',
            help: '0 = no string',
            type: 'number',
            format: 'number',
            default: 3,
            min: 0,
            max: 255
        },
        {
            id: 'serialNumber',
            label: 'Serial Number String',
            help: 'Serial number',
            type: 'text',
            format: 'string',
            default: 'USBDEVICE00001',
            placeholder: 'Serial number',
            conditionalField: 'iSerialNumber',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        },
        {
            id: 'bNumConfigurations',
            label: 'Number of Configurations (bNumConfigurations)',
            help: 'Number of possible configurations.',
            type: 'number',
            format: 'number',
            default: 1,
            min: 1,
            max: 255
        }
    ],
    descriptorComments: [
        'bLength',
        'bDescriptorType',
        'bcdUSB LSB',
        'bcdUSB MSB',
        'bDeviceClass',
        'bDeviceSubClass',
        'bDeviceProtocol',
        'bMaxPacketSize0',
        'idVendor LSB',
        'idVendor MSB',
        'idProduct LSB',
        'idProduct MSB',
        'bcdDevice LSB',
        'bcdDevice MSB',
        'iManufacturer',
        'iProduct',
        'iSerialNumber',
        'bNumConfigurations'
    ],
    readableMaps: {
        usbVersions: {
            272: '1.1',
            512: '2.0',
            768: '3.0',
            784: '3.1',
            800: '3.2'
        },
        deviceClasses: {
            0: 'Use Interface Descriptors',
            1: 'Audio',
            2: 'CDC - Communications',
            3: 'HID - Human Interface',
            8: 'Mass Storage',
            9: 'Hub',
            255: 'Vendor Specific'
        }
    }
};

const CONFIGURATION_SCHEMA = {
    fields: [
        {
            id: 'iConfiguration',
            label: 'Configuration String Index - Full Speed (iConfiguration)',
            help: 'Index of string descriptor, 0 = no string',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        },
        {
            id: 'configurationStringFS',
            label: 'Configuration String - Full Speed',
            help: 'Full Speed configuration string',
            type: 'text',
            format: 'string',
            default: 'FULL SPEED',
            placeholder: 'FULL SPEED',
            conditionalField: 'iConfiguration',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        },
        {
            id: 'iConfigurationHS',
            label: 'Configuration String Index - High Speed (iConfiguration)',
            help: 'Index of string descriptor, 0 = no string',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255,
            conditionalField: 'speed-high',
            conditionalOperator: 'checked'
        },
        {
            id: 'configurationStringHS',
            label: 'Configuration String - High Speed',
            help: 'High Speed configuration string',
            type: 'text',
            format: 'string',
            default: 'HIGH SPEED',
            placeholder: 'HIGH SPEED',
            conditionalField: 'speed-high',
            conditionalOperator: 'checked',
            conditionalFieldSecondary: 'iConfigurationHS',
            conditionalOperatorSecondary: 'greaterThan',
            conditionalValueSecondary: 0
        },
        {
            id: 'bmAttributes',
            label: 'Attributes (bmAttributes)',
            help: 'A device configuration that uses power from\nthe bus and a local source reports a non-zero\nvalue in bMaxPower to indicate the amount of\nbus power required and sets D6.  The actual\npower source at runtime may be determined\nusing the GetStatus(DEVICE) request (see Section 9.4.5).',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0xC0',
            options: [
                { value: '0x80', text: 'Bus-powered (0x80)' },
                { value: '0xC0', text: 'Self-powered (0xC0)' },
                { value: '0xA0', text: 'Bus-powered + Remote Wakeup (0xA0)' },
                { value: '0xE0', text: 'Self-powered + Remote Wakeup (0xE0)' }
            ]
        },
        {
            id: 'bMaxPower',
            label: 'Max Power (bMaxPower)',
            help: 'Maximum power consumption of the USB\ndevice from the bus in this specific\nconfiguration when the device is fully\noperational.  Expressed in 2 mA units\n(i.e., 50 = 100 mA).\nNote:  A device configuration reports whether\nthe configuration is bus-powered or self\npowered.  Device status reports whether the\ndevice is currently self-powered.  If a device is\ndisconnected from its external power source, it\nupdates device status to indicate that it is no\nlonger self-powered.\nA device may not increase its power draw\nfrom the bus, when it loses its external power\nsource, beyond the amount reported by its\nconfiguration.\nIf a device can continue to operate when\ndisconnected from its external power source, it\ncontinues to do so.  If the device cannot\ncontinue to operate, it fails operations it can\nno longer support.  The USB System Software\nmay determine the cause of the failure by\nchecking the status and noting the loss of the\ndevice’s power source.',
            type: 'number',
            format: 'number',
            default: 50,
            min: 0,
            max: 255
        }
    ]
};

const INTERFACE_SCHEMA = {
    fields: [
        {
            id: 'bInterfaceSubClass',
            label: 'Interface SubClass (bInterfaceSubClass)',
            help: 'Subclass code.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x00', text: 'No subclass' },
                { value: '0x01', text: 'Boot Interface subclass' }
            ]
        },
        {
            id: 'bInterfaceProtocol',
            label: 'Interface Protocol (bInterfaceProtocol)',
            help: 'Protocol code.',
            type: 'select',
            format: 'hex',
            default: '0x00',
            options: [
                { value: '0x00', text: 'None' },
                { value: '0x01', text: 'Keyboard' },
                { value: '0x02', text: 'Mouse' }
            ]
        },
        {
            id: 'iInterface',
            label: 'Interface String Index (iInterface)',
            help: 'Index of string descriptor, 0 = no string',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        },
        {
            id: 'interfaceString',
            label: 'Interface String',
            help: 'String descriptor text for this interface (used when iInterface != 0)',
            type: 'text',
            format: 'string',
            default: 'custom',
            placeholder: 'e.g., HID Interface',
            conditionalField: 'iInterface',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        }
    ]
};

const HID_DESC_SCHEMA = {
    fields: [
        {
            id: 'hidBcdHID',
            label: 'HID Specification Version (bcdHID)',
            help: 'Numeric expression identifying the HID Class Specification release.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0110',
            options: [
                { value: '0x0100', text: 'HID 1.0 (0x0100)' },
                { value: '0x0110', text: 'HID 1.1 (0x0110)' },
                { value: '0x0111', text: 'HID 1.11 (0x0111)' }
            ]
        },
        {
            id: 'hidBCountryCode',
            label: 'Country Code (bCountryCode)',
            help: 'Numeric expression identifying country code of the localized hardware.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x21',
            options: [
                { value: '0x00', text: 'Not localized (0x00)' },
                { value: '0x21', text: 'United States (0x21)' },
                { value: '0x08', text: 'British (0x08)' },
                { value: '0x0C', text: 'French (0x0C)' },
                { value: '0x10', text: 'German (0x10)' },
                { value: '0x14', text: 'Italian (0x14)' }
            ]
        },
        {
            id: 'hidBNumDescriptors',
            label: 'Number of Class Descriptors (bNumDescriptors)',
            help: 'Usually 1 for Report descriptor',
            type: 'number',
            format: 'number',
            default: 1,
            min: 1,
            max: 255
        }
    ]
};

const MASS_STORAGE_DESC_SCHEMA = {
    fields: [
        {
            id: 'mscBInterfaceSubClass',
            label: 'Interface SubClass (bInterfaceSubClass)',
            help: 'Mass Storage subclass code identifies the command set used.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x06',
            options: [
                { value: '0x01', text: 'RBC - Reduced Block Commands (0x01)' },
                { value: '0x02', text: 'SFF-8020i/MMC-2 (ATAPI) (0x02)' },
                { value: '0x03', text: 'QIC-157 (Tape) (0x03)' },
                { value: '0x04', text: 'UFI (Floppy) (0x04)' },
                { value: '0x05', text: 'SFF-8070i (ATAPI removable) (0x05)' },
                { value: '0x06', text: 'SCSI transparent command set (0x06)' },
                { value: '0x07', text: 'LSD FS (0x07)' },
                { value: '0x08', text: 'IEEE 1667 (0x08)' }
            ]
        },
        {
            id: 'mscBInterfaceProtocol',
            label: 'Interface Protocol (bInterfaceProtocol)',
            help: 'Mass Storage protocol defines the transport mechanism.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x50',
            options: [
                { value: '0x00', text: 'CBI (Control/Bulk/Interrupt) with command completion interrupt (0x00)' },
                { value: '0x01', text: 'CBI (Control/Bulk/Interrupt) without command completion interrupt (0x01)' },
                { value: '0x50', text: 'BBB - Bulk-Only Transport (0x50)' },
                { value: '0x62', text: 'UAS - USB Attached SCSI (0x62)' }
            ]
        },
        {
            id: 'mscMaxLUN',
            label: 'Maximum LUN (Logical Unit Number)',
            help: 'Maximum Logical Unit Number. The value of the maximum LUN is 0 to 15 (0x0F). This value is returned in response to GET_MAX_LUN request.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 15
        }
    ]
};

const DFU_DESC_SCHEMA = {
    fields: [
        {
            id: 'dfuBInterfaceSubClass',
            label: 'DFU Interface SubClass (bInterfaceSubClass)',
            help: 'DFU subclass code. Use 0x01 for DFU devices.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x01', text: 'Device Firmware Upgrade (0x01)' }
            ]
        },
        {
            id: 'dfuBInterfaceProtocol',
            label: 'DFU Interface Protocol (bInterfaceProtocol)',
            help: 'DFU protocol for runtime or DFU mode.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            options: [
                { value: '0x01', text: 'Runtime protocol (0x01)' },
                { value: '0x02', text: 'DFU mode protocol (0x02)' }
            ]
        },
        {
            id: 'dfuBmAttributes',
            label: 'DFU Attributes (bmAttributes)',
            help: 'DFU functional descriptor attributes bitmap.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x0B',
            options: [
                { value: '0x01', text: 'WillDetach (0x01)' },
                { value: '0x03', text: 'WillDetach + ManifestationTolerant (0x03)' },
                { value: '0x07', text: 'WillDetach + ManifestationTolerant + CanUpload (0x07)' },
                { value: '0x0B', text: 'WillDetach + CanUpload + CanDnload (0x0B)' },
                { value: '0x0F', text: 'All common flags set (0x0F)' }
            ]
        },
        {
            id: 'dfuDetachTimeout',
            label: 'Detach Timeout (wDetachTimeOut)',
            help: 'Maximum time in milliseconds for detach processing.',
            type: 'number',
            format: 'number',
            default: 1000,
            min: 0,
            max: 65535
        },
        {
            id: 'dfuTransferSize',
            label: 'Transfer Size (wTransferSize)',
            help: 'Maximum number of bytes per control-write transaction.',
            type: 'number',
            format: 'number',
            default: 1024,
            min: 1,
            max: 65535
        },
        {
            id: 'dfuBcdVersion',
            label: 'DFU Specification Release (bcdDFUVersion)',
            help: 'DFU specification version in BCD.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0110',
            options: [
                { value: '0x0101', text: 'DFU 1.0 (0x0101)' },
                { value: '0x0110', text: 'DFU 1.1 (0x0110)' }
            ]
        },
        {
            id: 'dfuIInterface',
            label: 'Interface String Index (iInterface)',
            help: 'String index for this DFU interface.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        },
        {
            id: 'dfuInterfaceString',
            label: 'Interface String',
            help: 'String descriptor text for this DFU interface (used when iInterface != 0)',
            type: 'text',
            format: 'string',
            default: '',
            placeholder: 'e.g., DFU Interface',
            conditionalField: 'dfuIInterface',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        }
    ]
};

const PRINTER_DESC_SCHEMA = {
    fields: [
        {
            id: 'printerBInterfaceSubClass',
            label: 'Printer SubClass (bInterfaceSubClass)',
            help: 'Printer subclass code. Use 0x01 for printer class devices.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x01', text: 'Printer Class SubClass (0x01)' }
            ]
        },
        {
            id: 'printerBInterfaceProtocol',
            label: 'Printer Protocol (bInterfaceProtocol)',
            help: 'Printer transport protocol.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            options: [
                { value: '0x01', text: 'Unidirectional (0x01)' },
                { value: '0x02', text: 'Bidirectional (0x02)' },
                { value: '0x03', text: 'IEEE 1284.4 Compatible (0x03)' }
            ]
        },
        {
            id: 'printerIInterface',
            label: 'Interface String Index (iInterface)',
            help: 'String descriptor index for this printer interface.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        },
        {
            id: 'printerInterfaceString',
            label: 'Interface String',
            help: 'String descriptor text for this printer interface (used when iInterface != 0)',
            type: 'text',
            format: 'string',
            default: '',
            placeholder: 'e.g., Printer Interface',
            conditionalField: 'printerIInterface',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        }
    ]
};

const VIDEO_DESC_SCHEMA = {
    fields: [
        {
            id: 'videoBInterfaceSubClass',
            label: 'Video SubClass (bInterfaceSubClass)',
            help: 'Video interface subclass code.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            options: [
                { value: '0x01', text: 'Video Control (0x01)' },
                { value: '0x02', text: 'Video Streaming (0x02)' },
                { value: '0x03', text: 'Video Interface Collection (0x03)' }
            ]
        },
        {
            id: 'videoBInterfaceProtocol',
            label: 'Video Protocol (bInterfaceProtocol)',
            help: 'Video interface protocol code.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x00',
            options: [
                { value: '0x00', text: 'Undefined/None (0x00)' },
                { value: '0x01', text: 'Protocol 15 (0x01)' }
            ]
        },
        {
            id: 'videoIInterface',
            label: 'Interface String Index (iInterface)',
            help: 'String descriptor index for this video interface.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        },
        {
            id: 'videoInterfaceString',
            label: 'Interface String',
            help: 'String descriptor text for this video interface (used when iInterface != 0)',
            type: 'text',
            format: 'string',
            default: '',
            placeholder: 'e.g., Video Interface',
            conditionalField: 'videoIInterface',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        }
    ]
};

const MTP_DESC_SCHEMA = {
    fields: [
        {
            id: 'mtpBInterfaceSubClass',
            label: 'MTP SubClass (bInterfaceSubClass)',
            help: 'Imaging subclass for Still Image Capture devices.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x01', text: 'Still Image Capture Device (0x01)' }
            ]
        },
        {
            id: 'mtpBInterfaceProtocol',
            label: 'MTP Protocol (bInterfaceProtocol)',
            help: 'PTP protocol value commonly used for MTP transport.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x00', text: 'No protocol (0x00)' },
                { value: '0x01', text: 'Picture Transfer Protocol (0x01)' }
            ]
        },
        {
            id: 'mtpIInterface',
            label: 'Interface String Index (iInterface)',
            help: 'String descriptor index for this MTP interface.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        },
        {
            id: 'mtpInterfaceString',
            label: 'Interface String',
            help: 'String descriptor text for this MTP interface (used when iInterface != 0)',
            type: 'text',
            format: 'string',
            default: '',
            placeholder: 'e.g., MTP Interface',
            conditionalField: 'mtpIInterface',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        }
    ]
};

const PTP_DESC_SCHEMA = {
    fields: [
        {
            id: 'ptpBInterfaceSubClass',
            label: 'PTP SubClass (bInterfaceSubClass)',
            help: 'Imaging subclass for Still Image Capture devices.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x01', text: 'Still Image Capture Device (0x01)' }
            ]
        },
        {
            id: 'ptpBInterfaceProtocol',
            label: 'PTP Protocol (bInterfaceProtocol)',
            help: 'Picture Transfer Protocol transport selector.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x00', text: 'No protocol (0x00)' },
                { value: '0x01', text: 'Picture Transfer Protocol (0x01)' }
            ]
        },
        {
            id: 'ptpIInterface',
            label: 'Interface String Index (iInterface)',
            help: 'String descriptor index for this PTP interface.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        },
        {
            id: 'ptpInterfaceString',
            label: 'Interface String',
            help: 'String descriptor text for this PTP interface (used when iInterface != 0)',
            type: 'text',
            format: 'string',
            default: '',
            placeholder: 'e.g., PTP Interface',
            conditionalField: 'ptpIInterface',
            conditionalOperator: 'greaterThan',
            conditionalValue: 0
        }
    ]
};

const CDC_ACM_DESC_SCHEMA = {
    fields: [
        {
            id: 'cdcBInterfaceSubClass',
            label: 'Communication Interface SubClass (bInterfaceSubClass)',
            help: 'CDC subclass code forCommunication Class Interface.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            options: [
                { value: '0x00', text: 'Reserved (0x00)' },
                { value: '0x01', text: 'Direct Line Control Model (0x01)' },
                { value: '0x02', text: 'Abstract Control Model (0x02)' },
                { value: '0x03', text: 'Telephone Control Model (0x03)' },
                { value: '0x04', text: 'Multi-Channel Control Model (0x04)' },
                { value: '0x05', text: 'CAPI Control Model (0x05)' },
                { value: '0x06', text: 'Ethernet Networking Control Model (0x06)' },
                { value: '0x07', text: 'ATM Networking Control Model (0x07)' }
            ]
        },
        {
            id: 'cdcBInterfaceProtocol',
            label: 'Communication Interface Protocol (bInterfaceProtocol)',
            help: 'Protocol code for Communication Class Interface.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            options: [
                { value: '0x00', text: 'No class specific protocol (0x00)' },
                { value: '0x01', text: 'AT Commands: V.250 etc (0x01)' },
                { value: '0x02', text: 'AT Commands defined by PCCA-101 (0x02)' },
                { value: '0x03', text: 'AT Commands defined by PCCA-101 & Annex O (0x03)' },
                { value: '0x04', text: 'AT Commands defined by GSM 07.07 (0x04)' },
                { value: '0x05', text: 'AT Commands defined by 3GPP 27.007 (0x05)' },
                { value: '0x06', text: 'AT Commands defined by TIA for CDMA (0x06)' },
                { value: '0xFE', text: 'External Protocol (0xFE)' },
                { value: '0xFF', text: 'Vendor-specific (0xFF)' }
            ]
        },
        {
            id: 'cdcBcdCDC',
            label: 'CDC Specification Release (bcdCDC)',
            help: 'CDC specification version in BCD.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0110',
            options: [
                { value: '0x0110', text: 'CDC 1.1 (0x0110)' },
                { value: '0x0120', text: 'CDC 1.2 (0x0120)' }
            ]
        },
        {
            id: 'cdcBmCapabilities',
            label: 'ACM Capabilities (bmCapabilities)',
            help: 'Bitmap of supported ACM features.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            options: [
                { value: '0x00', text: 'No capabilities (0x00)' },
                { value: '0x01', text: 'Device supports comm features (0x01)' },
                { value: '0x02', text: 'Device supports line coding/serial state (0x02)' },
                { value: '0x03', text: 'Device supports comm features + line coding (0x03)' },
                { value: '0x04', text: 'Device supports send break (0x04)' },
                { value: '0x06', text: 'Device supports line coding + send break (0x06)' },
                { value: '0x0F', text: 'All capabilities (0x0F)' }
            ]
        },
        {
            id: 'cdcDataInterfaceNumber',
            label: 'Data Interface Number',
            help: 'Interface number of Data Class interface.',
            type: 'number',
            format: 'number',
            default: 1,
            min: 0,
            max: 255
        }
    ]
};

const CDC_RNDIS_DESC_SCHEMA = {
    fields: [
        {
            id: 'rndisBcdCDC',
            label: 'CDC Specification Release (bcdCDC)',
            help: 'CDC specification version in BCD.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0110',
            options: [
                { value: '0x0110', text: 'CDC 1.1 (0x0110)' },
                { value: '0x0120', text: 'CDC 1.2 (0x0120)' }
            ]
        },
        {
            id: 'rndisCallMgmtCapabilities',
            label: 'Call Management Capabilities (bmCapabilities)',
            help: 'Call management capabilities bitmap.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x00',
            options: [
                { value: '0x00', text: 'No call management (0x00)' },
                { value: '0x01', text: 'Handles call mgmt itself (0x01)' },
                { value: '0x02', text: 'Uses data interface for call mgmt (0x02)' },
                { value: '0x03', text: 'Both capabilities (0x03)' }
            ]
        },
        {
            id: 'rndisAcmCapabilities',
            label: 'ACM Capabilities (bmCapabilities)',
            help: 'ACM capabilities bitmap for RNDIS control interface.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x00',
            options: [
                { value: '0x00', text: 'No ACM capabilities (0x00)' },
                { value: '0x02', text: 'Line coding/serial state (0x02)' },
                { value: '0x06', text: 'Line coding + send break (0x06)' }
            ]
        }
    ]
};

const CDC_ECM_DESC_SCHEMA = {
    fields: [
        {
            id: 'ecmBcdCDC',
            label: 'CDC Specification Release (bcdCDC)',
            help: 'CDC specification version in BCD.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0120',
            options: [
                { value: '0x0110', text: 'CDC 1.1 (0x0110)' },
                { value: '0x0120', text: 'CDC 1.2 (0x0120)' }
            ]
        },
        {
            id: 'ecmMacStringIndex',
            label: 'MAC String Index (iMACAddress)',
            help: 'String descriptor index for the MAC address string.',
            type: 'number',
            format: 'number',
            default: 4,
            min: 0,
            max: 255
        },
        {
            id: 'ecmMaxSegmentSize',
            label: 'Maximum Segment Size (wMaxSegmentSize)',
            help: 'Maximum Ethernet segment size in bytes.',
            type: 'number',
            format: 'number',
            default: 1514,
            min: 64,
            max: 65535
        },
        {
            id: 'ecmNumMcFilters',
            label: 'Number of Multicast Filters (wNumberMCFilters)',
            help: 'Number of multicast filters supported by the device.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 65535
        },
        {
            id: 'ecmNumPowerFilters',
            label: 'Number of Power Filters (bNumberPowerFilters)',
            help: 'Number of pattern filters for wakeup.',
            type: 'number',
            format: 'number',
            default: 0,
            min: 0,
            max: 255
        }
    ]
};

const AUDIO10_DESC_SCHEMA = {
    fields: [
        {
            id: 'audioTerminalLink',
            label: 'Terminal Link (bTerminalLink)',
            help: 'Terminal ID for this AudioStreaming interface. Used as the Input Terminal bTerminalID in the AC topology.',
            type: 'number',
            format: 'number',
            default: 1,
            min: 1,
            max: 253
        },
        {
            id: 'audioInputTerminalType',
            label: 'Input Terminal Type (wTerminalType)',
            help: 'Terminal type for the Input Terminal in the Audio Control topology (USB Audio 1.0 Terminal Types spec).',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0101',
            options: [
                { value: '0x0100', text: 'USB Undefined (0x0100)' },
                { value: '0x0101', text: 'USB Streaming (0x0101)' },
                { value: '0x01FF', text: 'USB Vendor Specific (0x01FF)' },
                { value: '0x0200', text: 'Input Undefined (0x0200)' },
                { value: '0x0201', text: 'Microphone (0x0201)' },
                { value: '0x0202', text: 'Desktop Microphone (0x0202)' },
                { value: '0x0203', text: 'Personal Microphone (0x0203)' },
                { value: '0x0204', text: 'Omni-Directional Microphone (0x0204)' },
                { value: '0x0205', text: 'Microphone Array (0x0205)' },
                { value: '0x0206', text: 'Processing Microphone Array (0x0206)' },
                { value: '0x0400', text: 'Bi-dir Undefined (0x0400)' },
                { value: '0x0401', text: 'Handset (0x0401)' },
                { value: '0x0402', text: 'Headset (0x0402)' },
                { value: '0x0403', text: 'Speakerphone no echo reduction (0x0403)' },
                { value: '0x0404', text: 'Echo-suppressing Speakerphone (0x0404)' },
                { value: '0x0405', text: 'Echo-canceling Speakerphone (0x0405)' },
                { value: '0x0500', text: 'Telephony Undefined (0x0500)' },
                { value: '0x0501', text: 'Phone Line (0x0501)' },
                { value: '0x0502', text: 'Telephone (0x0502)' },
                { value: '0x0503', text: 'Down Line Phone (0x0503)' },
                { value: '0x0600', text: 'External Undefined (0x0600)' },
                { value: '0x0601', text: 'Analog Connector (0x0601)' },
                { value: '0x0602', text: 'Digital Audio Interface (0x0602)' },
                { value: '0x0603', text: 'Line Connector (0x0603)' },
                { value: '0x0604', text: 'Legacy Audio Connector (0x0604)' },
                { value: '0x0605', text: 'S/PDIF Interface (0x0605)' },
                { value: '0x0606', text: '1394 DA Stream (0x0606)' },
                { value: '0x0607', text: '1394 DV Stream Soundtrack (0x0607)' },
                { value: '0x0700', text: 'Embedded Undefined (0x0700)' },
                { value: '0x0701', text: 'Level Calibration Noise Source (0x0701)' },
                { value: '0x0703', text: 'CD Player (0x0703)' },
                { value: '0x0704', text: 'DAT (0x0704)' },
                { value: '0x0705', text: 'DCC (0x0705)' },
                { value: '0x0706', text: 'MiniDisk (0x0706)' },
                { value: '0x0707', text: 'Analog Tape (0x0707)' },
                { value: '0x0708', text: 'Phonograph (0x0708)' },
                { value: '0x0709', text: 'VCR Audio (0x0709)' },
                { value: '0x070A', text: 'Video Disc Audio (0x070A)' },
                { value: '0x070B', text: 'DVD Audio (0x070B)' },
                { value: '0x070C', text: 'TV Tuner Audio (0x070C)' },
                { value: '0x070D', text: 'Satellite Receiver Audio (0x070D)' },
                { value: '0x070E', text: 'Cable Tuner Audio (0x070E)' },
                { value: '0x070F', text: 'DSS Audio (0x070F)' },
                { value: '0x0710', text: 'Radio Receiver (0x0710)' },
                { value: '0x0711', text: 'Radio Transmitter (0x0711)' },
                { value: '0x0712', text: 'Multi-track Recorder (0x0712)' },
                { value: '0x0713', text: 'Synthesizer (0x0713)' }
            ]
        },
        {
            id: 'audioOutputTerminalType',
            label: 'Output Terminal Type (wTerminalType)',
            help: 'Terminal type for the Output Terminal in the Audio Control topology (USB Audio 1.0 Terminal Types spec).',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0301',
            options: [
                { value: '0x0100', text: 'USB Undefined (0x0100)' },
                { value: '0x0101', text: 'USB Streaming (0x0101)' },
                { value: '0x01FF', text: 'USB Vendor Specific (0x01FF)' },
                { value: '0x0300', text: 'Output Undefined (0x0300)' },
                { value: '0x0301', text: 'Speaker (0x0301)' },
                { value: '0x0302', text: 'Headphones (0x0302)' },
                { value: '0x0303', text: 'Head Mounted Display Audio (0x0303)' },
                { value: '0x0304', text: 'Desktop Speaker (0x0304)' },
                { value: '0x0305', text: 'Room Speaker (0x0305)' },
                { value: '0x0306', text: 'Communication Speaker (0x0306)' },
                { value: '0x0307', text: 'Low Freq Effects Speaker (0x0307)' },
                { value: '0x0400', text: 'Bi-dir Undefined (0x0400)' },
                { value: '0x0401', text: 'Handset (0x0401)' },
                { value: '0x0402', text: 'Headset (0x0402)' },
                { value: '0x0403', text: 'Speakerphone no echo reduction (0x0403)' },
                { value: '0x0404', text: 'Echo-suppressing Speakerphone (0x0404)' },
                { value: '0x0405', text: 'Echo-canceling Speakerphone (0x0405)' },
                { value: '0x0500', text: 'Telephony Undefined (0x0500)' },
                { value: '0x0501', text: 'Phone Line (0x0501)' },
                { value: '0x0502', text: 'Telephone (0x0502)' },
                { value: '0x0503', text: 'Down Line Phone (0x0503)' },
                { value: '0x0600', text: 'External Undefined (0x0600)' },
                { value: '0x0601', text: 'Analog Connector (0x0601)' },
                { value: '0x0602', text: 'Digital Audio Interface (0x0602)' },
                { value: '0x0603', text: 'Line Connector (0x0603)' },
                { value: '0x0604', text: 'Legacy Audio Connector (0x0604)' },
                { value: '0x0605', text: 'S/PDIF Interface (0x0605)' },
                { value: '0x0606', text: '1394 DA Stream (0x0606)' },
                { value: '0x0607', text: '1394 DV Stream Soundtrack (0x0607)' },
                { value: '0x0700', text: 'Embedded Undefined (0x0700)' },
                { value: '0x0701', text: 'Level Calibration Noise Source (0x0701)' },
                { value: '0x0703', text: 'CD Player (0x0703)' },
                { value: '0x0704', text: 'DAT (0x0704)' },
                { value: '0x0705', text: 'DCC (0x0705)' },
                { value: '0x0706', text: 'MiniDisk (0x0706)' },
                { value: '0x0707', text: 'Analog Tape (0x0707)' },
                { value: '0x0708', text: 'Phonograph (0x0708)' },
                { value: '0x0709', text: 'VCR Audio (0x0709)' },
                { value: '0x070A', text: 'Video Disc Audio (0x070A)' },
                { value: '0x070B', text: 'DVD Audio (0x070B)' },
                { value: '0x070C', text: 'TV Tuner Audio (0x070C)' },
                { value: '0x070D', text: 'Satellite Receiver Audio (0x070D)' },
                { value: '0x070E', text: 'Cable Tuner Audio (0x070E)' },
                { value: '0x070F', text: 'DSS Audio (0x070F)' },
                { value: '0x0710', text: 'Radio Receiver (0x0710)' },
                { value: '0x0711', text: 'Radio Transmitter (0x0711)' },
                { value: '0x0712', text: 'Multi-track Recorder (0x0712)' },
                { value: '0x0713', text: 'Synthesizer (0x0713)' }
            ]
        },
        {
            id: 'audioFuMasterControls',
            label: 'Feature Unit — Master Channel Controls (bmaControls[0])',
            help: 'Controls enabled on the master (mix) channel. Bits 0–7 require bControlSize=1; selecting Bass Boost (bit 8) or Loudness (bit 9) automatically sets bControlSize=2.',
            type: 'multiselect',
            default: [0, 1],
            options: [
                { value: -1, text: 'None (0x00)' },
                { value: 0, text: 'Mute' },
                { value: 1, text: 'Volume' },
                { value: 2, text: 'Bass' },
                { value: 3, text: 'Mid' },
                { value: 4, text: 'Treble' },
                { value: 5, text: 'Graphic EQ' },
                { value: 6, text: 'Auto Gain' },
                { value: 7, text: 'Delay' },
                { value: 8, text: 'Bass Boost' },
                { value: 9, text: 'Loudness' }
            ]
        },
        {
            id: 'audioFuChannelControls',
            label: 'Feature Unit — Per-Channel Controls (bmaControls[1..N])',
            help: 'Controls enabled per logical channel (same bitmask applied uniformly to bmaControls[1] through bmaControls[N]). Bits 0–7 require bControlSize=1; selecting Bass Boost (bit 8) or Loudness (bit 9) automatically sets bControlSize=2.',
            type: 'multiselect',
            default: [-1],
            options: [
                { value: -1, text: 'None (0x00)' },
                { value: 0, text: 'Mute' },
                { value: 1, text: 'Volume' },
                { value: 2, text: 'Bass' },
                { value: 3, text: 'Mid' },
                { value: 4, text: 'Treble' },
                { value: 5, text: 'Graphic EQ' },
                { value: 6, text: 'Auto Gain' },
                { value: 7, text: 'Delay' },
                { value: 8, text: 'Bass Boost' },
                { value: 9, text: 'Loudness' }
            ]
        },
        {
            id: 'audioFormatTag',
            label: 'Format Tag (wFormatTag)',
            help: 'Audio data format used by this interface (USB Audio Class 1.0 Data Formats specification).',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0001',
            options: [
                { value: '0x0001', text: 'PCM (0x0001)' },
                { value: '0x0002', text: 'PCM8 (0x0002)' },
                { value: '0x0003', text: 'IEEE_FLOAT (0x0003)' },
                { value: '0x0004', text: 'ALAW (0x0004)' },
                { value: '0x0005', text: 'MULAW (0x0005)' },
                { value: '0x1001', text: 'MPEG (0x1001)' },
                { value: '0x1002', text: 'AC-3 (0x1002)' },
                { value: '0x2001', text: 'IEC1937_AC-3 (0x2001)' },
                { value: '0x2002', text: 'IEC1937_MPEG-1_Layer1 (0x2002)' },
                { value: '0x2003', text: 'IEC1937_MPEG-1_Layer2/3 (0x2003)' },
                { value: '0x2004', text: 'IEC1937_MPEG-2_EXT (0x2004)' },
                { value: '0x2005', text: 'IEC1937_MPEG-2_Layer1_LS (0x2005)' },
                { value: '0x2006', text: 'IEC1937_MPEG-2_Layer2/3_LS (0x2006)' }
            ]
        },

        {
            id: 'audioNumChannels',
            label: 'Number of Channels (bNrChannels)',
            help: 'Number of logical audio channels in the stream (Type I / Type III only).',
            type: 'number',
            format: 'number',
            default: 2,
            min: 1,
            max: 8,
            showForFormatTypes: ['0x01', '0x03']
        },
        {
            id: 'audioBitResolution',
            label: 'Bit Resolution (bBitResolution)',
            help: 'Number of effectively used bits per audio subframe (Type I / Type III only).',
            type: 'select',
            format: 'number',
            bytes: 1,
            default: 16,
            options: [
                { value: 8,  text: '8-bits per sample' },
                { value: 16, text: '16-bits per sample' },
                { value: 24, text: '24-bits per sample' },
                { value: 32, text: '32-bits per sample' }
            ],
            showForFormatTypes: ['0x01', '0x03']
        },
        {
            id: 'audioMaxBitRate',
            label: 'Max Bit Rate – wMaxBitRate (kbit/s)',
            help: 'Maximum bit rate in kbit/s that this interface can produce or consume (Type II only). E.g. 128 for MPEG Layer-3 at 128 kbps.',
            type: 'number',
            format: 'number',
            default: 128,
            min: 1,
            max: 65535,
            showForFormatTypes: ['0x02']
        },
        {
            id: 'audioSamplesPerFrame',
            label: 'Samples Per Frame – wSamplesPerFrame',
            help: 'Number of PCM audio samples contained in one encoded audio frame (Type II only). E.g. 1 for AC-3, 384 for MPEG Layer-1.',
            type: 'number',
            format: 'number',
            default: 1,
            min: 1,
            max: 65535,
            showForFormatTypes: ['0x02']
        },
        {
            id: 'audioSampleRate',
            label: 'Sample Rate (tSamFreq)',
            help: 'One or more sampling frequencies supported by the AudioStreaming interface.',
            type: 'multiselect',
            default: [48000],
            options: [
                { value: 8000,  text: '8 KHz' },
                { value: 16000, text: '16 KHz' },
                { value: 32000, text: '32 KHz' },
                { value: 44100, text: '44.1 KHz' },
                { value: 48000, text: '48 KHz' },
                { value: 96000, text: '96 KHz' }
            ]
        }
    ]
};

const AUDIO20_DESC_SCHEMA = {
    fields: [
        {
            id: 'audio20ClockSourceId',
            label: 'Clock Source ID (bClockID)',
            help: 'Entity ID of the UAC2 clock source descriptor.',
            type: 'number',
            format: 'number',
            default: 16,
            min: 1,
            max: 255
        },
        {
            id: 'audio20TerminalLink',
            label: 'Terminal Link (bTerminalLink)',
            help: 'Terminal ID used by AudioStreaming interface.',
            type: 'number',
            format: 'number',
            default: 1,
            min: 1,
            max: 255
        },
        {
            id: 'audio20NumChannels',
            label: 'Number of Channels (bNrChannels)',
            help: 'Number of logical channels in the audio stream.',
            type: 'number',
            format: 'number',
            default: 2,
            min: 1,
            max: 8
        },
        {
            id: 'audio20SubslotSize',
            label: 'Subslot Size (bSubslotSize)',
            help: 'Number of bytes occupied by one audio subslot.',
            type: 'select',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            options: [
                { value: '0x01', text: '1 byte (0x01)' },
                { value: '0x02', text: '2 bytes (0x02)' },
                { value: '0x03', text: '3 bytes (0x03)' },
                { value: '0x04', text: '4 bytes (0x04)' }
            ]
        },
        {
            id: 'audio20BitResolution',
            label: 'Bit Resolution (bBitResolution)',
            help: 'Number of effectively used bits per sample.',
            type: 'number',
            format: 'number',
            default: 16,
            min: 1,
            max: 32
        },
        {
            id: 'audio20SampleRate',
            label: 'Sample Rate (Hz)',
            help: 'Nominal sample rate used to compute packet sizes.',
            type: 'number',
            format: 'number',
            default: 48000,
            min: 8000,
            max: 384000
        }
    ]
};

const ENDPOINT_SCHEMA = {
    fields: [
        {
            id: 'endpointBEndpointAddress',
            label: 'Endpoint Address (bEndpointAddress)',
            help: 'e.g., 0x81 for IN endpoint 1',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'endpointWMaxPacketSize',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size this endpoint is capable of sending or receiving when this configuration is selected. For interrupt endpoints, this value is used to reserve the bus time in the schedule, required for the per frame data payloads. Smaller data payloads may be sent, but will terminate the transfer and thus require intervention to restart. (e.g., 0x0008 for 8 bytes)',
            type: 'text',
            format: 'hex',
            bytes: 2,
            default: '0x0008',
            placeholder: '0x0008'
        },
        {
            id: 'endpointBInterval',
            label: 'Polling Interval (bInterval)',
            help: 'Interval for polling endpoint for data transfers. Expressed in milliseconds. (e.g., 0x08 for 8ms)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x08',
            placeholder: '0x08'
        }
    ]
};

const templates = {};

function registerHandlebarsHelpers() {
    Handlebars.registerHelper('ifEq', function (a, b, options) {
        return a === b ? options.fn(this) : options.inverse(this);
    });

    Handlebars.registerHelper('ifIn', function (item, arr, options) {
        return Array.isArray(arr) && arr.includes(item) ? options.fn(this) : options.inverse(this);
    });

    Handlebars.registerHelper('formatCodeField', function (hex, comma) {
        const value = `${String(hex || '')}${comma ? ',' : ''}`;
        return value.padEnd(16, ' ');
    });
}

function compileTemplates() {
    templates.field = Handlebars.compile(document.getElementById('field-template').innerHTML);
    templates.cArray = Handlebars.compile(document.getElementById('c-array-template').innerHTML);
    templates.hex = Handlebars.compile(document.getElementById('hex-template').innerHTML);
    templates.readable = Handlebars.compile(document.getElementById('readable-template').innerHTML);
}

function toHex(value, bytes) {
    const hex = value.toString(16).toUpperCase().padStart(bytes * 2, '0');
    return '0x' + hex;
}

const FORMAT_TYPE_FROM_TAG = {
    '0x0001': '0x01', '0x0002': '0x01', '0x0003': '0x01', '0x0004': '0x01', '0x0005': '0x01',
    '0x1001': '0x02', '0x1002': '0x02',
    '0x2001': '0x03', '0x2002': '0x03', '0x2003': '0x03', '0x2004': '0x03', '0x2005': '0x03', '0x2006': '0x03'
};

const FORMAT_TYPE_NAMES = {
    '0x01': 'Type I  – PCM, PCM8, IEEE_FLOAT, ALAW, MULAW',
    '0x02': 'Type II – MPEG, AC-3 (encoded)',
    '0x03': 'Type III – IEC 60958 pass-through'
};

function getFormatTypeFromTag(tagHex) {
    const normalised = String(tagHex).trim().toLowerCase().replace(/^0x/, '');
    const lookup = '0x' + normalised.toUpperCase().padStart(4, '0');
    return FORMAT_TYPE_FROM_TAG[lookup] || '0x01';
}

// wChannelConfig bitmap (USB Audio 1.0 spec Table 3-6), keyed by bNrChannels.
// Bit assignments: B0=L, B1=R, B2=C, B3=LFE, B4=LS, B5=RS, B6=LC, B7=RC, B8=S, B9=SL, B10=SR, B11=T
const CHANNEL_CONFIG_FROM_COUNT = {
    1: 0x0000, // Mono – non-predefined; host relies on bNrChannels
    2: 0x0003, // L + R
    3: 0x0007, // L + R + C
    4: 0x0033, // L + R + LS + RS
    5: 0x0037, // L + R + C + LS + RS
    6: 0x003F, // L + R + C + LFE + LS + RS  (5.1)
    7: 0x013F, // L + R + C + LFE + LS + RS + S  (6.1)
    8: 0x063F  // L + R + C + LFE + LS + RS + SL + SR  (7.1)
};

const CHANNEL_CONFIG_LABELS = {
    1: 'Mono (non-predefined)',
    2: 'L + R',
    3: 'L + R + C',
    4: 'L + R + LS + RS',
    5: 'L + R + C + LS + RS',
    6: 'L + R + C + LFE + LS + RS  (5.1)',
    7: 'L + R + C + LFE + LS + RS + S  (6.1)',
    8: 'L + R + C + LFE + LS + RS + SL + SR  (7.1)'
};

function getChannelConfigFromCount(n) {
    const count = Math.max(1, Math.min(8, parseInt(n, 10) || 2));
    const raw = CHANNEL_CONFIG_FROM_COUNT[count] !== undefined ? CHANNEL_CONFIG_FROM_COUNT[count] : 0x0003;
    return raw;
}

function parseHexValue(rawValue, bytes, fieldId) {
    const cleaned = String(rawValue).trim().replace(/^0x/i, '');
    if (!/^[0-9a-fA-F]+$/.test(cleaned)) {
        throw new Error(fieldId + ' must be a hexadecimal value');
    }

    const parsed = parseInt(cleaned, 16);
    const maxValue = (1 << (bytes * 8)) - 1;
    if (parsed < 0 || parsed > maxValue) {
        throw new Error(fieldId + ' must fit in ' + (bytes * 8) + ' bits');
    }

    return parsed;
}

function parseNumberValue(rawValue, field) {
    const parsed = Number.parseInt(rawValue, 10);
    if (Number.isNaN(parsed)) {
        throw new Error(field.id + ' must be a number');
    }

    if (typeof field.min === 'number' && parsed < field.min) {
        throw new Error(field.id + ' must be >= ' + field.min);
    }

    if (typeof field.max === 'number' && parsed > field.max) {
        throw new Error(field.id + ' must be <= ' + field.max);
    }

    return parsed;
}

function getCurrentClassOrder() {
    return classCardOrder.slice();
}

function getEnabledClassOrder() {
    return classCardOrder.filter((className) => {
        const config = CLASS_CARD_CONFIG[className];
        const checkbox = config ? document.getElementById(config.checkboxId) : null;
        return !!(checkbox && checkbox.checked);
    });
}

function applyClassCardOrder() {
    const container = document.getElementById('class-config-cards');
    if (!container) {
        return;
    }

    classCardOrder.forEach((className) => {
        const config = CLASS_CARD_CONFIG[className];
        const card = config ? document.getElementById(config.cardId) : null;
        if (card) {
            container.appendChild(card);
        }
    });
}

function updateClassReorderButtonsState() {
    const enabledOrder = getEnabledClassOrder();
    const hasMultipleEnabled = enabledOrder.length > 1;

    classCardOrder.forEach((className) => {
        const card = document.getElementById(CLASS_CARD_CONFIG[className].cardId);
        if (!card) {
            return;
        }

        const upBtn = card.querySelector('.class-reorder-up');
        const downBtn = card.querySelector('.class-reorder-down');
        if (!upBtn || !downBtn) {
            return;
        }

        const enabledIndex = enabledOrder.indexOf(className);
        const isEnabled = enabledIndex !== -1;

        upBtn.disabled = !hasMultipleEnabled || !isEnabled || enabledIndex === 0;
        downBtn.disabled = !hasMultipleEnabled || !isEnabled || enabledIndex === enabledOrder.length - 1;
    });
}

function moveClassCard(className, direction) {
    const enabledOrder = getEnabledClassOrder();
    const enabledIndex = enabledOrder.indexOf(className);
    if (enabledIndex === -1) {
        return;
    }

    const targetEnabledIndex = enabledIndex + direction;
    if (targetEnabledIndex < 0 || targetEnabledIndex >= enabledOrder.length) {
        return;
    }

    const swapClassName = enabledOrder[targetEnabledIndex];
    const currentIndex = classCardOrder.indexOf(className);
    const swapIndex = classCardOrder.indexOf(swapClassName);
    if (currentIndex === -1 || swapIndex === -1) {
        return;
    }

    classCardOrder[currentIndex] = swapClassName;
    classCardOrder[swapIndex] = className;

    applyClassCardOrder();
    updateClassReorderButtonsState();
    generateDescriptor();
}

function injectClassCardReorderControls() {
    classCardOrder.forEach((className) => {
        const config = CLASS_CARD_CONFIG[className];
        const card = document.getElementById(config.cardId);
        if (!card) {
            return;
        }

        const header = card.querySelector('.card-header');
        const heading = header ? header.querySelector('h2.mb-0') : null;
        const titleButton = heading ? heading.querySelector('.btn-collapse') : null;
        if (!header || !heading || !titleButton) {
            return;
        }

        if (header.querySelector('.class-reorder-actions')) {
            return;
        }

        heading.classList.add('class-card-header');

        const titleWrap = document.createElement('div');
        titleWrap.className = 'class-title-wrap';
        heading.insertBefore(titleWrap, titleButton);
        titleWrap.appendChild(titleButton);

        const actions = document.createElement('div');
        actions.className = 'class-reorder-actions';
        actions.innerHTML = `
            <button type="button" class="class-reorder-button class-reorder-up" title="Move up" aria-label="Move ${config.title} up" onclick="moveClassCard('${className}', -1)">
                <span class="class-reorder-icon" aria-hidden="true">&#x25B2;</span><span class="sr-only">Move up</span>
            </button>
            <button type="button" class="class-reorder-button class-reorder-down" title="Move down" aria-label="Move ${config.title} down" onclick="moveClassCard('${className}', 1)">
                <span class="class-reorder-icon" aria-hidden="true">&#x25BC;</span><span class="sr-only">Move down</span>
            </button>
        `;
        heading.appendChild(actions);
    });

    applyClassCardOrder();
    updateClassReorderButtonsState();
}

function setClassConfigCardExpanded(className, expanded) {
    const config = CLASS_CARD_CONFIG[className];
    if (!config) {
        return;
    }

    const card = document.getElementById(config.cardId);
    const collapse = document.getElementById(config.collapseId);
    const button = card ? card.querySelector('.btn-collapse') : null;

    if (collapse) {
        collapse.classList.toggle('show', expanded);
    }

    if (button) {
        button.classList.toggle('collapsed', !expanded);
        button.setAttribute('aria-expanded', expanded ? 'true' : 'false');
    }
}

function renderForm() {
    const fieldsForTemplate = PAGE_SCHEMA.fields.map((field) => ({
        ...field,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    document.getElementById('dynamic-form-fields').innerHTML = html;
    renderStringFieldGroups();
    enforceFixedStringIndexes();

    // Render configuration fields
    const configFieldsForTemplate = CONFIGURATION_SCHEMA.fields.map((field) => ({
        ...field,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const configHtml = templates.field({ fields: configFieldsForTemplate });
    document.getElementById('configuration-form-fields').innerHTML = configHtml;
    renderConfigurationStringFieldGroups();

    // Update conditional field visibility after all form fields are rendered.
    updateConditionalFieldsVisibility();
    updateStringLanguageFields();

    // Add event listeners for conditional fields.
    setupConditionalFieldListeners();
    setupLanguageFieldListeners();
}

function createInfoIcon(helpText) {
    return `<span class="info-icon" title="${helpText}" data-tooltip="${helpText}">ℹ</span>`;
}

function setFormGroupLabel(group, labelText, helpText) {
    const label = group ? group.querySelector('label') : null;
    if (!label) {
        return;
    }

    label.innerHTML = `${labelText} ${createInfoIcon(helpText)}`;
}

function getPrimaryStringLanguage() {
    for (const language of STRING_LANGUAGE_OPTIONS) {
        const checkbox = document.getElementById(language.id);
        if (checkbox && checkbox.checked) {
            return language;
        }
    }

    return STRING_LANGUAGE_OPTIONS[0];
}

function getEnabledStringLanguages() {
    return STRING_LANGUAGE_OPTIONS.filter((language) => {
        const checkbox = document.getElementById(language.id);
        return checkbox && checkbox.checked;
    });
}

function getSelectedLanguageForGroup(groupConfig) {
    const selector = document.getElementById(groupConfig.languageFieldId);
    const enabledLanguages = getEnabledStringLanguages();
    const fallbackLanguage = enabledLanguages[0] || getPrimaryStringLanguage();

    if (!selector) {
        return fallbackLanguage;
    }

    const selectedCode = Number.parseInt(selector.value, 16);
    const selectedLanguage = enabledLanguages.find((language) => language.code === selectedCode);
    return selectedLanguage || fallbackLanguage;
}

function getSelectedLanguageForSelector(selectorId) {
    const selector = document.getElementById(selectorId);
    const enabledLanguages = getEnabledStringLanguages();
    const fallbackLanguage = enabledLanguages[0] || getPrimaryStringLanguage();

    if (!selector) {
        return fallbackLanguage;
    }

    const selectedCode = Number.parseInt(selector.value, 16);
    const selectedLanguage = enabledLanguages.find((language) => language.code === selectedCode);
    return selectedLanguage || fallbackLanguage;
}

function updateInterfaceLanguageSelectors() {
    const enabledLanguages = getEnabledStringLanguages();
    const primaryLanguage = enabledLanguages[0] || getPrimaryStringLanguage();
    const hasMultipleLanguages = enabledLanguages.length > 1;

    document.querySelectorAll('.interface-language-selector').forEach((selector) => {
        const previousValue = selector.value;
        selector.innerHTML = '';

        enabledLanguages.forEach((language) => {
            const option = document.createElement('option');
            option.value = toHex(language.code, 2);
            option.textContent = `${toHex(language.code, 2)} (${language.name})`;
            selector.appendChild(option);
        });

        if (selector.options.length === 0) {
            const option = document.createElement('option');
            option.value = toHex(primaryLanguage.code, 2);
            option.textContent = `${toHex(primaryLanguage.code, 2)} (${primaryLanguage.name})`;
            selector.appendChild(option);
        }

        const hasPreviousOption = Array.from(selector.options).some((option) => option.value === previousValue);
        selector.value = hasPreviousOption ? previousValue : toHex(primaryLanguage.code, 2);

        if (!selector.dataset.lockedByIndex) {
            selector.disabled = !hasMultipleLanguages;
        }
        selector.title = getSelectedLanguageForSelector(selector.id).name;
    });
}

function renderStringFieldGroups() {
    STRING_FIELD_GROUPS.forEach((groupConfig) => renderStringFieldGroup(groupConfig));
}

function renderStringFieldGroup(groupConfig) {
    const indexInput = document.getElementById(groupConfig.indexFieldId);
    const stringInput = document.getElementById(groupConfig.stringFieldId);
    const formContainer = document.getElementById('dynamic-form-fields');

    if (!indexInput || !stringInput || !formContainer) {
        return;
    }

    const indexGroup = indexInput.closest('.form-group');
    const stringGroup = stringInput.closest('.form-group');

    if (!indexGroup || !stringGroup) {
        return;
    }

    const wrapper = document.createElement('div');
    wrapper.className = 'string-field-group';
    wrapper.id = groupConfig.wrapperId;
    wrapper.innerHTML = `
        <div class="string-field-group-title">${groupConfig.title}</div>
        <div class="string-field-grid">
            <div class="form-group string-language-group">
                <label for="${groupConfig.languageFieldId}">
                    Language ID ${createInfoIcon('Select language when multiple language IDs are enabled')}
                </label>
                <select id="${groupConfig.languageFieldId}"></select>
            </div>
        </div>
    `;

    const grid = wrapper.querySelector('.string-field-grid');
    formContainer.insertBefore(wrapper, indexGroup);
    grid.insertBefore(indexGroup, grid.firstChild);
    grid.insertBefore(stringGroup, grid.querySelector('.string-language-group'));

    setFormGroupLabel(indexGroup, 'Index', '0 = no string');
    setFormGroupLabel(stringGroup, 'String', groupConfig.stringHelp);

    indexInput.value = groupConfig.fixedIndex;
    indexInput.readOnly = true;
}

function renderConfigurationStringFieldGroups() {
    CONFIGURATION_STRING_FIELD_GROUPS.forEach((groupConfig) => {
        const indexInput = document.getElementById(groupConfig.indexFieldId);
        const stringInput = document.getElementById(groupConfig.stringFieldId);
        const formContainer = document.getElementById('configuration-form-fields');

        if (!indexInput || !stringInput || !formContainer) {
            return;
        }

        const indexGroup = indexInput.closest('.form-group');
        const stringGroup = stringInput.closest('.form-group');

        if (!indexGroup || !stringGroup) {
            return;
        }

        const wrapper = document.createElement('div');
        wrapper.className = 'string-field-group';
        wrapper.id = groupConfig.wrapperId;
        wrapper.innerHTML = `
            <div class="string-field-group-title">${groupConfig.title}</div>
            <div class="string-field-grid">
                <div class="form-group string-language-group">
                    <label for="${groupConfig.languageFieldId}">
                        Language ID ${createInfoIcon('Select language when multiple language IDs are enabled')}
                    </label>
                    <select id="${groupConfig.languageFieldId}"></select>
                </div>
            </div>
        `;

        const grid = wrapper.querySelector('.string-field-grid');
        formContainer.insertBefore(wrapper, indexGroup);
        grid.insertBefore(indexGroup, grid.firstChild);
        grid.insertBefore(stringGroup, grid.querySelector('.string-language-group'));

        setFormGroupLabel(indexGroup, 'Index', '0 = no string');
        setFormGroupLabel(stringGroup, 'String', groupConfig.stringHelp);
    });
}

function enforceFixedStringIndexes() {
    STRING_FIELD_GROUPS.forEach((groupConfig) => {
        const indexInput = document.getElementById(groupConfig.indexFieldId);
        if (!indexInput) {
            return;
        }

        indexInput.value = groupConfig.fixedIndex;
        indexInput.readOnly = true;
    });
}

function updateStringLanguageFields() {
    const enabledLanguages = getEnabledStringLanguages();
    const primaryLanguage = enabledLanguages[0] || getPrimaryStringLanguage();
    const hasMultipleLanguages = enabledLanguages.length > 1;

    STRING_FIELD_GROUPS.concat(CONFIGURATION_STRING_FIELD_GROUPS).forEach((groupConfig) => {
        const selector = document.getElementById(groupConfig.languageFieldId);
        if (!selector) {
            return;
        }

        const previousValue = selector.value;
        selector.innerHTML = '';

        enabledLanguages.forEach((language) => {
            const option = document.createElement('option');
            option.value = toHex(language.code, 2);
            option.textContent = `${toHex(language.code, 2)} (${language.name})`;
            selector.appendChild(option);
        });

        if (selector.options.length === 0) {
            const option = document.createElement('option');
            option.value = toHex(primaryLanguage.code, 2);
            option.textContent = `${toHex(primaryLanguage.code, 2)} (${primaryLanguage.name})`;
            selector.appendChild(option);
        }

        const hasPreviousOption = Array.from(selector.options).some((option) => option.value === previousValue);
        selector.value = hasPreviousOption ? previousValue : toHex(primaryLanguage.code, 2);
        selector.disabled = !hasMultipleLanguages;
        selector.title = getSelectedLanguageForGroup(groupConfig).name;
    });

    updateInterfaceLanguageSelectors();
    updateCustomStringLanguageSelectors();

    // Re-apply index-driven lock state for configuration string controls.
    updateConditionalFieldsVisibility();
}

function setupLanguageFieldListeners() {
    STRING_LANGUAGE_OPTIONS.forEach((language) => {
        const element = document.getElementById(language.id);
        if (element && !element.dataset.languageDisplayBound) {
            element.addEventListener('change', updateStringLanguageFields);
            element.dataset.languageDisplayBound = 'true';
        }
    });
}

// ─── Custom String Descriptor Card ───────────────────────────────────────────

function addCustomStringDescriptor() {
    // Suggest next unused index (after 1=mfr, 2=product, 3=serial)
    const used = new Set([1, 2, 3]);
    customStringDescriptors.forEach((d) => used.add(d.index));
    let nextIdx = 4;
    while (used.has(nextIdx)) nextIdx++;

    const primaryLang = (getEnabledStringLanguages()[0] || getPrimaryStringLanguage());
    customStringDescriptors.push({
        id: customStringNextId++,
        index: nextIdx,
        text: '',
        languageId: toHex(primaryLang.code, 2)
    });
    renderCustomStringDescriptorCard();
    generateDescriptor();
}

function removeCustomStringDescriptor(id) {
    customStringDescriptors = customStringDescriptors.filter((d) => d.id !== id);
    renderCustomStringDescriptorCard();
    generateDescriptor();
}

function renderCustomStringDescriptorCard() {
    const container = document.getElementById('custom-string-descriptors-list');
    if (!container) return;

    if (customStringDescriptors.length === 0) {
        container.innerHTML = '';
        return;
    }

    const enabledLanguages = getEnabledStringLanguages();
    const primaryLanguage = enabledLanguages[0] || getPrimaryStringLanguage();
    const langList = enabledLanguages.length ? enabledLanguages : [primaryLanguage];
    const hasMultiple = langList.length > 1;

    container.innerHTML = customStringDescriptors.map((desc) => {
        const langOptions = langList
            .map((lang) => {
                const val = toHex(lang.code, 2);
                const selected = desc.languageId === val ? ' selected' : '';
                return `<option value="${val}"${selected}>${val} (${lang.name})</option>`;
            })
            .join('');

        return `
        <div id="custom-str-card-${desc.id}" style="margin-bottom: 12px;">
            <div class="card" style="margin-bottom: 0;">
                <div class="card-header">
                    <h3 class="mb-0" style="display: flex; align-items: center; justify-content: space-between; gap: 8px;">
                        <button class="btn-collapse collapsed" type="button"
                            onclick="toggleCollapse('collapseCustomStr${desc.id}', this)"
                            aria-expanded="false"
                            id="custom-str-title-${desc.id}"
                            style="flex: 1; text-align: left; font-size: 0.85em;">
                            String Descriptor ${desc.index}
                        </button>
                        <button class="btn btn-sm" type="button"
                            onclick="removeCustomStringDescriptor(${desc.id})"
                            style="flex: 0 0 auto; margin: 6px 8px 6px 0; padding: 4px 8px; font-size: 0.75em;">
                            remove
                        </button>
                    </h3>
                </div>
                <div id="collapseCustomStr${desc.id}" class="collapse">
                    <div class="card-body">
                        <div class="string-field-grid">
                            <div class="form-group">
                                <label for="custom-str-index-${desc.id}">Index ${createInfoIcon('String descriptor index (must be unique)')}</label>
                                <input type="number" id="custom-str-index-${desc.id}" value="${desc.index}" min="1" max="255"
                                    onchange="syncCustomStringDescriptor(${desc.id})" oninput="syncCustomStringDescriptor(${desc.id})">
                            </div>
                            <div class="form-group">
                                <label for="custom-str-text-${desc.id}">String ${createInfoIcon('String content for this descriptor')}</label>
                                <input type="text" id="custom-str-text-${desc.id}" value="${desc.text}" placeholder="e.g., My String"
                                    onchange="syncCustomStringDescriptor(${desc.id})" oninput="syncCustomStringDescriptor(${desc.id})">
                            </div>
                            <div class="form-group string-language-group">
                                <label for="custom-str-lang-${desc.id}">Language ID ${createInfoIcon('Select language when multiple language IDs are enabled')}</label>
                                <select id="custom-str-lang-${desc.id}" ${hasMultiple ? '' : 'disabled'}
                                    onchange="syncCustomStringDescriptor(${desc.id})">${langOptions}</select>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>`;
    }).join('');
}

function syncCustomStringDescriptor(id) {
    const desc = customStringDescriptors.find((d) => d.id === id);
    if (!desc) return;

    const indexEl = document.getElementById(`custom-str-index-${id}`);
    const textEl = document.getElementById(`custom-str-text-${id}`);
    const langEl = document.getElementById(`custom-str-lang-${id}`);
    const titleEl = document.getElementById(`custom-str-title-${id}`);

    if (indexEl) desc.index = parseInt(indexEl.value, 10) || 1;
    if (textEl) desc.text = textEl.value;
    if (langEl) desc.languageId = langEl.value;

    // Keep card header title in sync with current index
    if (titleEl) titleEl.textContent = `String Descriptor ${desc.index}`;

    generateDescriptor();
}

function updateCustomStringLanguageSelectors() {
    const enabledLanguages = getEnabledStringLanguages();
    const primaryLanguage = enabledLanguages[0] || getPrimaryStringLanguage();
    const langList = enabledLanguages.length ? enabledLanguages : [primaryLanguage];
    const hasMultiple = langList.length > 1;

    customStringDescriptors.forEach((desc) => {
        const selector = document.getElementById(`custom-str-lang-${desc.id}`);
        if (!selector) return;

        const previousValue = selector.value;
        selector.innerHTML = langList
            .map((lang) => {
                const val = toHex(lang.code, 2);
                return `<option value="${val}">${val} (${lang.name})</option>`;
            })
            .join('');

        const hasPrev = Array.from(selector.options).some((opt) => opt.value === previousValue);
        selector.value = hasPrev ? previousValue : toHex(primaryLanguage.code, 2);
        selector.disabled = !hasMultiple;
        desc.languageId = selector.value;
    });
}

function getCustomStringDescriptorValues() {
    return customStringDescriptors.map((desc) => {
        const indexEl = document.getElementById(`custom-str-index-${desc.id}`);
        const textEl = document.getElementById(`custom-str-text-${desc.id}`);
        const langEl = document.getElementById(`custom-str-lang-${desc.id}`);
        return {
            id: desc.id,
            index: indexEl ? (parseInt(indexEl.value, 10) || desc.index) : desc.index,
            text: textEl ? textEl.value : desc.text,
            languageId: langEl ? langEl.value : desc.languageId
        };
    });
}

function updateConditionalFieldsVisibility() {
    const allFields = PAGE_SCHEMA.fields.concat(CONFIGURATION_SCHEMA.fields);

    const evaluateCondition = (fieldId, operator, expectedValue) => {
        const conditionalElement = document.getElementById(fieldId);
        if (!conditionalElement) {
            return false;
        }

        if (operator === 'checked') {
            return conditionalElement.checked === true;
        }

        const conditionalValue = parseFloat(conditionalElement.value) || 0;
        if (operator === 'greaterThan') {
            return conditionalValue > expectedValue;
        }

        if (operator === 'equals') {
            return conditionalValue === expectedValue;
        }

        return false;
    };

    allFields.forEach((field) => {
        if (field.conditionalField) {
            const fieldElement = document.getElementById(field.id);

            if (fieldElement) {
                const formGroup = fieldElement.closest('.form-group');
                if (formGroup) {
                    const primaryCondition = evaluateCondition(
                        field.conditionalField,
                        field.conditionalOperator,
                        field.conditionalValue
                    );
                    const secondaryCondition = field.conditionalFieldSecondary
                        ? evaluateCondition(
                            field.conditionalFieldSecondary,
                            field.conditionalOperatorSecondary,
                            field.conditionalValueSecondary
                        )
                        : true;
                    const shouldShow = primaryCondition && secondaryCondition;

                    formGroup.style.display = shouldShow ? 'block' : 'none';
                }
            }
        }
    });

    STRING_FIELD_GROUPS.forEach((groupConfig) => {
        const languageGroup = document.querySelector(`#${groupConfig.wrapperId} .string-language-group`);
        const stringGroup = document.getElementById(groupConfig.stringFieldId)?.closest('.form-group');
        if (languageGroup && stringGroup) {
            languageGroup.style.display = stringGroup.style.display === 'none' ? 'none' : 'block';
        }
    });

    CONFIGURATION_STRING_FIELD_GROUPS.forEach((groupConfig) => {
        const wrapper = document.getElementById(groupConfig.wrapperId);
        if (!wrapper) {
            return;
        }

        const isWrapperVisible = groupConfig.conditionalField
            ? evaluateCondition(groupConfig.conditionalField, groupConfig.conditionalOperator, groupConfig.conditionalValue)
            : true;
        wrapper.style.display = isWrapperVisible ? '' : 'none';

        const indexInput = document.getElementById(groupConfig.indexFieldId);
        const stringInput = document.getElementById(groupConfig.stringFieldId);
        const indexGroup = indexInput ? indexInput.closest('.form-group') : null;
        const stringGroup = stringInput ? stringInput.closest('.form-group') : null;
        const indexValue = indexInput ? (parseFloat(indexInput.value) || 0) : 0;
        const isEditable = isWrapperVisible && indexValue !== 0;

        if (indexGroup) {
            indexGroup.style.display = 'block';
        }

        if (stringGroup) {
            stringGroup.style.display = 'block';
        }

        if (stringInput) {
            stringInput.disabled = !isEditable;
        }

        const languageGroup = wrapper.querySelector('.string-language-group');
        if (languageGroup) {
            languageGroup.style.display = 'block';
            const languageSelector = languageGroup.querySelector('select');
            if (languageSelector) {
                languageSelector.disabled = !isEditable;
            }
        }
    });
}

// Helper function to update conditional visibility for dynamically created class fields
function updateClassConditionalFieldsVisibility(instanceNum, schema, prefix) {
    const evaluateCondition = (fieldId, operator, expectedValue) => {
        const conditionalElement = document.getElementById(fieldId);
        if (!conditionalElement) {
            return false;
        }

        if (operator === 'checked') {
            return conditionalElement.checked === true;
        }

        const conditionalValue = parseFloat(conditionalElement.value) || 0;
        if (operator === 'greaterThan') {
            return conditionalValue > expectedValue;
        }

        if (operator === 'equals') {
            return conditionalValue === expectedValue;
        }

        return false;
    };

    schema.fields.forEach((field) => {
        if (field.conditionalField) {
            const dynamicFieldId = `${prefix}-${instanceNum}-${field.id}`;
            const dynamicConditionalFieldId = `${prefix}-${instanceNum}-${field.conditionalField}`;
            const fieldElement = document.getElementById(dynamicFieldId);

            if (fieldElement) {
                const formGroup = fieldElement.closest('.form-group');
                if (formGroup) {
                    const primaryCondition = evaluateCondition(
                        dynamicConditionalFieldId,
                        field.conditionalOperator,
                        field.conditionalValue
                    );
                    const secondaryCondition = field.conditionalFieldSecondary
                        ? evaluateCondition(
                            `${prefix}-${instanceNum}-${field.conditionalFieldSecondary}`,
                            field.conditionalOperatorSecondary,
                            field.conditionalValueSecondary
                        )
                        : true;
                    const shouldShow = primaryCondition && secondaryCondition;

                    formGroup.style.display = shouldShow ? 'block' : 'none';
                }
            }
        }
    });
}

function setupConditionalFieldListeners() {
    // Find all fields that control conditional visibility
    const conditionalControlFields = new Set();
    const allFields = PAGE_SCHEMA.fields.concat(CONFIGURATION_SCHEMA.fields);

    allFields.forEach((field) => {
        if (field.conditionalField) {
            conditionalControlFields.add(field.conditionalField);
        }
        if (field.conditionalFieldSecondary) {
            conditionalControlFields.add(field.conditionalFieldSecondary);
        }
    });

    // Add change listeners to control fields
    conditionalControlFields.forEach((fieldId) => {
        const element = document.getElementById(fieldId);
        if (element) {
            element.addEventListener('change', updateConditionalFieldsVisibility);
            element.addEventListener('input', updateConditionalFieldsVisibility);
        }
    });
}

function readFormValues() {
    const values = {};

    PAGE_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(field.id);
        if (!element) {
            // Field might be hidden, use default value
            if (field.format === 'string') {
                values[field.id] = field.default || '';
            } else if (field.format === 'hex') {
                // Parse default hex value
                const defaultValue = field.default || '0x00';
                values[field.id] = parseHexValue(defaultValue, field.bytes, field.id);
            } else {
                values[field.id] = field.default || 0;
            }
            return;
        }

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function getClassInterfaceCount(classCheckboxId, classCountId, interfacesPerInstance = 1) {
    const checkbox = document.getElementById(classCheckboxId);
    if (!checkbox || !checkbox.checked) {
        return 0;
    }

    const countInput = document.getElementById(classCountId);
    const instanceCount = Math.max(1, parseInt(countInput && countInput.value, 10) || 1);
    return instanceCount * interfacesPerInstance;
}

function calculateTotalInterfaces() {
    const classInterfaceConfig = [
        { checkboxId: 'class-hid', countId: 'class-num-hid', perInstance: 1 },
        { checkboxId: 'class-msc', countId: 'class-num-msc', perInstance: 1 },
        { checkboxId: 'class-dfu', countId: 'class-num-dfu', perInstance: 1 },
        { checkboxId: 'class-printer', countId: 'class-num-printer', perInstance: 1 },
        { checkboxId: 'class-video', countId: 'class-num-video', perInstance: 1 },
        { checkboxId: 'class-mtp', countId: 'class-num-mtp', perInstance: 1 },
        { checkboxId: 'class-ptp', countId: 'class-num-ptp', perInstance: 1 },
        { checkboxId: 'class-cdc', countId: 'class-num-cdc', perInstance: 2 },
        { checkboxId: 'class-rndis', countId: 'class-num-rndis', perInstance: 2 },
        { checkboxId: 'class-ecm', countId: 'class-num-ecm', perInstance: 2 },
        { checkboxId: 'class-audio', countId: 'class-num-audio', perInstance: 2 },
        { checkboxId: 'class-audio2', countId: 'class-num-audio2', perInstance: 2 }
    ];

    const total = classInterfaceConfig.reduce((sum, config) => {
        return sum + getClassInterfaceCount(config.checkboxId, config.countId, config.perInstance);
    }, 0);

    return Math.min(255, total);
}

function readConfigurationValues() {
    const values = {};

    CONFIGURATION_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(field.id);
        if (!element) {
            // Field might be hidden, use default value
            if (field.format === 'hex') {
                const defaultValue = field.default || '0x00';
                values[field.id] = parseHexValue(defaultValue, field.bytes, field.id);
            } else {
                values[field.id] = field.default || 0;
            }
            return;
        }

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    // bNumInterfaces is derived from enabled class instances and not user-editable.
    values.bNumInterfaces = calculateTotalInterfaces();

    return values;
}

function validateUniqueStringIndexes(values, configValues) {
    const indexOwners = new Map();
    const registerIndex = (indexValue, ownerName) => {
        const index = Number.parseInt(indexValue, 10);
        if (!Number.isFinite(index) || index <= 0) {
            return;
        }

        const existingOwner = indexOwners.get(index);
        if (existingOwner) {
            throw new Error(`String index ${index} is already used by ${existingOwner}. It cannot be reused for ${ownerName}.`);
        }

        indexOwners.set(index, ownerName);
    };

    registerIndex(values.iManufacturer, 'iManufacturer');
    registerIndex(values.iProduct, 'iProduct');
    registerIndex(values.iSerialNumber, 'iSerialNumber');
    registerIndex(configValues.iConfiguration, 'iConfiguration (Full Speed)');

    const hsEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    if (hsEnabled) {
        registerIndex(configValues.iConfigurationHS, 'iConfiguration (High Speed)');
    }

    const registerClassIndexes = (classCheckboxId, classCountId, readValuesFn, indexKey, ownerPrefix) => {
        const checkbox = document.getElementById(classCheckboxId);
        if (!checkbox || !checkbox.checked) {
            return;
        }

        const countInput = document.getElementById(classCountId);
        const classCount = Math.max(1, Number.parseInt(countInput && countInput.value, 10) || 1);
        for (let i = 1; i <= classCount; i++) {
            const classValues = readValuesFn(i);
            registerIndex(classValues[indexKey], `${ownerPrefix}-${i}`);
        }
    };

    registerClassIndexes('class-hid', 'class-num-hid', readInterfaceDescriptorValues, 'iInterface', 'iInterface HID');
    registerClassIndexes('class-dfu', 'class-num-dfu', readDFUConfigValues, 'dfuIInterface', 'iInterface DFU');
    registerClassIndexes('class-printer', 'class-num-printer', readPrinterConfigValues, 'printerIInterface', 'iInterface Printer');
    registerClassIndexes('class-video', 'class-num-video', readVideoConfigValues, 'videoIInterface', 'iInterface Video');
    registerClassIndexes('class-mtp', 'class-num-mtp', readMTPConfigValues, 'mtpIInterface', 'iInterface MTP');
    registerClassIndexes('class-ptp', 'class-num-ptp', readPTPConfigValues, 'ptpIInterface', 'iInterface PTP');
    registerClassIndexes('class-ecm', 'class-num-ecm', readECMConfigValues, 'ecmMacStringIndex', 'iMACAddress ECM');
    registerClassIndexes('class-audio', 'class-num-audio', readAudioConfigValues, 'audioAcIInterface', 'iInterface Audio AC');
    registerClassIndexes('class-audio', 'class-num-audio', readAudioConfigValues, 'audioAsIInterface', 'iInterface Audio AS');

    getCustomStringDescriptorValues().forEach((csd, idx) => {
        if (csd.index >= 1) {
            registerIndex(csd.index, `Custom String Descriptor ${idx + 1}`);
        }
    });

    usedStringIndexes = indexOwners;
}

function buildDescriptor(values) {
    return [
        18,
        0x01,
        values.bcdUSB & 0xFF,
        (values.bcdUSB >> 8) & 0xFF,
        values.bDeviceClass,
        values.bDeviceSubClass,
        values.bDeviceProtocol,
        values.bMaxPacketSize0,
        values.idVendor & 0xFF,
        (values.idVendor >> 8) & 0xFF,
        values.idProduct & 0xFF,
        (values.idProduct >> 8) & 0xFF,
        values.bcdDevice & 0xFF,
        (values.bcdDevice >> 8) & 0xFF,
        values.iManufacturer,
        values.iProduct,
        values.iSerialNumber,
        values.bNumConfigurations
    ];
}

function buildConfigDescriptor(configValues, isHighSpeed = false) {
    const iConfig = isHighSpeed ? (configValues.iConfigurationHS || configValues.iConfiguration) : configValues.iConfiguration;
    return [
        9,  // bLength
        0x02,  // bDescriptorType (Configuration)
        configValues.wTotalLength & 0xFF,  // wTotalLength LSB
        (configValues.wTotalLength >> 8) & 0xFF,  // wTotalLength MSB
        configValues.bNumInterfaces,  // bNumInterfaces
        0x01,  // bConfigurationValue
        iConfig,  // iConfiguration
        configValues.bmAttributes,  // bmAttributes
        configValues.bMaxPower  // bMaxPower
    ];
}

function updateConfigurationTotalLength(descriptor, configStartIndex) {
    if (!Array.isArray(descriptor) || descriptor.length < configStartIndex + 4) {
        return 0;
    }

    const totalLength = Math.max(0, descriptor.length - configStartIndex);
    descriptor[configStartIndex + 2] = totalLength & 0xFF;
    descriptor[configStartIndex + 3] = (totalLength >> 8) & 0xFF;
    return totalLength;
}

function appendIadDescriptor(descriptor, commentsList, tag, firstInterface, interfaceCount, functionClass, functionSubClass, functionProtocol, iFunction = 0x00) {
    const iadDescriptor = [
        0x08,  // bLength
        0x0B,  // bDescriptorType (IAD)
        firstInterface & 0xFF,  // bFirstInterface
        interfaceCount & 0xFF,  // bInterfaceCount
        functionClass & 0xFF,  // bFunctionClass
        functionSubClass & 0xFF,  // bFunctionSubClass
        functionProtocol & 0xFF,  // bFunctionProtocol
        iFunction & 0xFF   // iFunction
    ];

    descriptor.push(...iadDescriptor);
    [
        'bLength',
        'bDescriptorType',
        'bFirstInterface',
        'bInterfaceCount',
        'bFunctionClass',
        'bFunctionSubClass',
        'bFunctionProtocol',
        'iFunction'
    ].forEach((comment) => commentsList.push(`[${tag}] ${comment}`));
}

function appendCDCNetworkingDescriptors(descriptor, commentsList, options) {
    const {
        instanceIndex,
        commInterfaceNum,
        dataInterfaceNum,
        subclass,
        protocol,
        bcdCdc,
        callMgmtCaps,
        acmCaps,
        notifyEndpoint,
        notifyMaxPacket,
        notifyInterval,
        bulkInEndpoint,
        bulkInMaxPacket,
        bulkInInterval,
        bulkOutEndpoint,
        bulkOutMaxPacket,
        bulkOutInterval,
        tagPrefix,
        includeCallMgmt,
        includeAcm,
        includeEthernet,
        ethernet
    } = options;

    const commInterfaceDescriptor = [
        0x09,
        0x04,
        commInterfaceNum,
        0x00,
        0x01,
        0x02,
        subclass,
        protocol,
        0x00
    ];
    descriptor.push(...commInterfaceDescriptor);
    ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
        .forEach((comment) => commentsList.push(`[${tagPrefix}-COMM-IF-${instanceIndex}] ${comment}`));

    const headerDescriptor = [
        0x05,
        0x24,
        0x00,
        bcdCdc & 0xFF,
        (bcdCdc >> 8) & 0xFF
    ];
    descriptor.push(...headerDescriptor);
    ['bFunctionLength', 'bDescriptorType', 'bDescriptorSubtype', 'bcdCDC LSB', 'bcdCDC MSB']
        .forEach((comment) => commentsList.push(`[${tagPrefix}-HEADER-${instanceIndex}] ${comment}`));

    if (includeCallMgmt) {
        const callMgmtDescriptor = [
            0x05,
            0x24,
            0x01,
            callMgmtCaps,
            dataInterfaceNum
        ];
        descriptor.push(...callMgmtDescriptor);
        ['bFunctionLength', 'bDescriptorType', 'bDescriptorSubtype', 'bmCapabilities', 'bDataInterface']
            .forEach((comment) => commentsList.push(`[${tagPrefix}-CALL-MGMT-${instanceIndex}] ${comment}`));
    }

    if (includeAcm) {
        const acmDescriptor = [
            0x04,
            0x24,
            0x02,
            acmCaps
        ];
        descriptor.push(...acmDescriptor);
        ['bFunctionLength', 'bDescriptorType', 'bDescriptorSubtype', 'bmCapabilities']
            .forEach((comment) => commentsList.push(`[${tagPrefix}-ACM-${instanceIndex}] ${comment}`));
    }

    if (includeEthernet && ethernet) {
        const ethernetDescriptor = [
            0x0D,
            0x24,
            0x0F,
            ethernet.macStringIndex,
            0x00,
            0x00,
            0x00,
            0x00,
            ethernet.maxSegmentSize & 0xFF,
            (ethernet.maxSegmentSize >> 8) & 0xFF,
            ethernet.numMcFilters & 0xFF,
            (ethernet.numMcFilters >> 8) & 0xFF,
            ethernet.numPowerFilters
        ];
        descriptor.push(...ethernetDescriptor);
        ['bFunctionLength', 'bDescriptorType', 'bDescriptorSubtype', 'iMACAddress', 'bmEthernetStatistics[0]', 'bmEthernetStatistics[1]', 'bmEthernetStatistics[2]', 'bmEthernetStatistics[3]', 'wMaxSegmentSize LSB', 'wMaxSegmentSize MSB', 'wNumberMCFilters LSB', 'wNumberMCFilters MSB', 'bNumberPowerFilters']
            .forEach((comment) => commentsList.push(`[${tagPrefix}-ETH-${instanceIndex}] ${comment}`));
    }

    const unionDescriptor = [
        0x05,
        0x24,
        0x06,
        commInterfaceNum,
        dataInterfaceNum
    ];
    descriptor.push(...unionDescriptor);
    ['bFunctionLength', 'bDescriptorType', 'bDescriptorSubtype', 'bMasterInterface', 'bSlaveInterface0']
        .forEach((comment) => commentsList.push(`[${tagPrefix}-UNION-${instanceIndex}] ${comment}`));

    const notifyDescriptor = [
        0x07,
        0x05,
        notifyEndpoint,
        0x03,
        notifyMaxPacket & 0xFF,
        (notifyMaxPacket >> 8) & 0xFF,
        notifyInterval
    ];
    descriptor.push(...notifyDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[${tagPrefix}-EP-NOTIFY-${instanceIndex}] ${comment}`));

    const dataInterfaceDescriptor = [
        0x09,
        0x04,
        dataInterfaceNum,
        0x00,
        0x02,
        0x0A,
        0x00,
        0x00,
        0x00
    ];
    descriptor.push(...dataInterfaceDescriptor);
    ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
        .forEach((comment) => commentsList.push(`[${tagPrefix}-DATA-IF-${instanceIndex}] ${comment}`));

    const bulkInDescriptor = [
        0x07,
        0x05,
        bulkInEndpoint,
        0x02,
        bulkInMaxPacket & 0xFF,
        (bulkInMaxPacket >> 8) & 0xFF,
        bulkInInterval
    ];
    descriptor.push(...bulkInDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[${tagPrefix}-EP-IN-${instanceIndex}] ${comment}`));

    const bulkOutDescriptor = [
        0x07,
        0x05,
        bulkOutEndpoint,
        0x02,
        bulkOutMaxPacket & 0xFF,
        (bulkOutMaxPacket >> 8) & 0xFF,
        bulkOutInterval
    ];
    descriptor.push(...bulkOutDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[${tagPrefix}-EP-OUT-${instanceIndex}] ${comment}`));
}

function appendVideoDescriptorsFromValues(descriptor, commentsList, options) {
    const {
        instanceIndex,
        interfaceNumber,
        configValues,
        bulkInValues,
        bulkOutValues,
        inEndpointKey,
        inPacketKey,
        inIntervalKey,
        outEndpointKey,
        outPacketKey,
        outIntervalKey,
        defaultInEndpoint,
        defaultOutEndpoint,
        defaultPacket,
        defaultInterval
    } = options;

    const interfaceDescriptor = [
        0x09,
        0x04,
        interfaceNumber,
        0x00,
        0x02,
        0x0E,
        parseHexValue(configValues.videoBInterfaceSubClass || '0x02', 1, 'videoBInterfaceSubClass'),
        parseHexValue(configValues.videoBInterfaceProtocol || '0x00', 1, 'videoBInterfaceProtocol'),
        parseNumberValue(configValues.videoIInterface || 0, { id: 'videoIInterface', min: 0, max: 255, default: 0 })
    ];
    descriptor.push(...interfaceDescriptor);
    ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
        .forEach((comment) => commentsList.push(`[VID-IF-${instanceIndex}] ${comment}`));

    const inEndpoint = parseHexValue(bulkInValues[inEndpointKey] || defaultInEndpoint, 1, inEndpointKey);
    const inMaxPacket = parseHexValue(bulkInValues[inPacketKey] || defaultPacket, 2, inPacketKey);
    const inInterval = parseHexValue(bulkInValues[inIntervalKey] || defaultInterval, 1, inIntervalKey);
    const bulkInDescriptor = [0x07, 0x05, inEndpoint, 0x02, inMaxPacket & 0xFF, (inMaxPacket >> 8) & 0xFF, inInterval];
    descriptor.push(...bulkInDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[VID-EP-IN-${instanceIndex}] ${comment}`));

    const outEndpoint = parseHexValue(bulkOutValues[outEndpointKey] || defaultOutEndpoint, 1, outEndpointKey);
    const outMaxPacket = parseHexValue(bulkOutValues[outPacketKey] || defaultPacket, 2, outPacketKey);
    const outInterval = parseHexValue(bulkOutValues[outIntervalKey] || defaultInterval, 1, outIntervalKey);
    const bulkOutDescriptor = [0x07, 0x05, outEndpoint, 0x02, outMaxPacket & 0xFF, (outMaxPacket >> 8) & 0xFF, outInterval];
    descriptor.push(...bulkOutDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[VID-EP-OUT-${instanceIndex}] ${comment}`));
}

function appendMTPDescriptorsFromValues(descriptor, commentsList, options) {
    const {
        instanceIndex,
        interfaceNumber,
        configValues,
        bulkInValues,
        bulkOutValues,
        inEndpointKey,
        inPacketKey,
        inIntervalKey,
        outEndpointKey,
        outPacketKey,
        outIntervalKey,
        defaultInEndpoint,
        defaultOutEndpoint,
        defaultPacket,
        defaultInterval
    } = options;

    const interfaceDescriptor = [
        0x09,
        0x04,
        interfaceNumber,
        0x00,
        0x02,
        0x06,
        parseHexValue(configValues.mtpBInterfaceSubClass || '0x01', 1, 'mtpBInterfaceSubClass'),
        parseHexValue(configValues.mtpBInterfaceProtocol || '0x01', 1, 'mtpBInterfaceProtocol'),
        parseNumberValue(configValues.mtpIInterface || 0, { id: 'mtpIInterface', min: 0, max: 255, default: 0 })
    ];
    descriptor.push(...interfaceDescriptor);
    ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
        .forEach((comment) => commentsList.push(`[MTP-IF-${instanceIndex}] ${comment}`));

    const inEndpoint = parseHexValue(bulkInValues[inEndpointKey] || defaultInEndpoint, 1, inEndpointKey);
    const inMaxPacket = parseHexValue(bulkInValues[inPacketKey] || defaultPacket, 2, inPacketKey);
    const inInterval = parseHexValue(bulkInValues[inIntervalKey] || defaultInterval, 1, inIntervalKey);
    const bulkInDescriptor = [0x07, 0x05, inEndpoint, 0x02, inMaxPacket & 0xFF, (inMaxPacket >> 8) & 0xFF, inInterval];
    descriptor.push(...bulkInDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[MTP-EP-IN-${instanceIndex}] ${comment}`));

    const outEndpoint = parseHexValue(bulkOutValues[outEndpointKey] || defaultOutEndpoint, 1, outEndpointKey);
    const outMaxPacket = parseHexValue(bulkOutValues[outPacketKey] || defaultPacket, 2, outPacketKey);
    const outInterval = parseHexValue(bulkOutValues[outIntervalKey] || defaultInterval, 1, outIntervalKey);
    const bulkOutDescriptor = [0x07, 0x05, outEndpoint, 0x02, outMaxPacket & 0xFF, (outMaxPacket >> 8) & 0xFF, outInterval];
    descriptor.push(...bulkOutDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[MTP-EP-OUT-${instanceIndex}] ${comment}`));
}

function appendPTPDescriptorsFromValues(descriptor, commentsList, options) {
    const {
        instanceIndex,
        interfaceNumber,
        configValues,
        bulkInValues,
        bulkOutValues,
        inEndpointKey,
        inPacketKey,
        inIntervalKey,
        outEndpointKey,
        outPacketKey,
        outIntervalKey,
        defaultInEndpoint,
        defaultOutEndpoint,
        defaultPacket,
        defaultInterval
    } = options;

    const interfaceDescriptor = [
        0x09,
        0x04,
        interfaceNumber,
        0x00,
        0x02,
        0x06,
        parseHexValue(configValues.ptpBInterfaceSubClass || '0x01', 1, 'ptpBInterfaceSubClass'),
        parseHexValue(configValues.ptpBInterfaceProtocol || '0x01', 1, 'ptpBInterfaceProtocol'),
        parseNumberValue(configValues.ptpIInterface || 0, { id: 'ptpIInterface', min: 0, max: 255, default: 0 })
    ];
    descriptor.push(...interfaceDescriptor);
    ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
        .forEach((comment) => commentsList.push(`[PTP-IF-${instanceIndex}] ${comment}`));

    const inEndpoint = parseHexValue(bulkInValues[inEndpointKey] || defaultInEndpoint, 1, inEndpointKey);
    const inMaxPacket = parseHexValue(bulkInValues[inPacketKey] || defaultPacket, 2, inPacketKey);
    const inInterval = parseHexValue(bulkInValues[inIntervalKey] || defaultInterval, 1, inIntervalKey);
    const bulkInDescriptor = [0x07, 0x05, inEndpoint, 0x02, inMaxPacket & 0xFF, (inMaxPacket >> 8) & 0xFF, inInterval];
    descriptor.push(...bulkInDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[PTP-EP-IN-${instanceIndex}] ${comment}`));

    const outEndpoint = parseHexValue(bulkOutValues[outEndpointKey] || defaultOutEndpoint, 1, outEndpointKey);
    const outMaxPacket = parseHexValue(bulkOutValues[outPacketKey] || defaultPacket, 2, outPacketKey);
    const outInterval = parseHexValue(bulkOutValues[outIntervalKey] || defaultInterval, 1, outIntervalKey);
    const bulkOutDescriptor = [0x07, 0x05, outEndpoint, 0x02, outMaxPacket & 0xFF, (outMaxPacket >> 8) & 0xFF, outInterval];
    descriptor.push(...bulkOutDescriptor);
    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
        .forEach((comment) => commentsList.push(`[PTP-EP-OUT-${instanceIndex}] ${comment}`));
}

// Returns sorted, filtered list of active string descriptors (same data as device_framework_string).
// Each entry: { index, text, name, languageGroup?, languageSelectorId? }
function buildActiveStringDescriptors(values, configValues) {
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);

    const stringDescriptors = [
        {
            index: values.iManufacturer,
            text: values.manufacturer,
            name: 'Manufacturer',
            minIndex: 1,
            enabled: true,
            languageGroup: STRING_FIELD_GROUPS.find((g) => g.indexFieldId === 'iManufacturer')
        },
        {
            index: values.iProduct,
            text: values.product,
            name: 'Product',
            minIndex: 1,
            enabled: true,
            languageGroup: STRING_FIELD_GROUPS.find((g) => g.indexFieldId === 'iProduct')
        },
        {
            index: values.iSerialNumber,
            text: values.serialNumber,
            name: 'Serial Number',
            minIndex: 1,
            enabled: true,
            languageGroup: STRING_FIELD_GROUPS.find((g) => g.indexFieldId === 'iSerialNumber')
        },
        {
            index: configValues.iConfiguration,
            text: configValues.configurationStringFS,
            name: 'Configuration',
            minIndex: 1,
            enabled: true,
            languageGroup: CONFIGURATION_STRING_FIELD_GROUPS.find((g) => g.indexFieldId === 'iConfiguration')
        },
        {
            index: configValues.iConfigurationHS,
            text: configValues.configurationStringHS,
            name: 'Configuration HS',
            minIndex: 1,
            enabled: isHighSpeedEnabled,
            languageGroup: CONFIGURATION_STRING_FIELD_GROUPS.find((g) => g.indexFieldId === 'iConfigurationHS')
        }
    ];

    const appendIfActive = (index, text, name, languageSelectorId) => {
        if (index && index > 0 && text) {
            stringDescriptors.push({ index, text, name, minIndex: 1, enabled: true, languageSelectorId });
        }
    };

    const hidCb = document.getElementById('class-hid');
    if (hidCb && hidCb.checked) {
        const n = parseInt((document.getElementById('class-num-hid') || {}).value) || 1;
        for (let i = 1; i <= n; i++) {
            const v = readInterfaceDescriptorValues(i);
            appendIfActive(v.iInterface, v.interfaceString, `Interface HID-${i}`, `interface-${i}-languageIdDisplay`);
        }
    }

    const dfuCb = document.getElementById('class-dfu');
    if (dfuCb && dfuCb.checked) {
        const n = parseInt((document.getElementById('class-num-dfu') || {}).value) || 1;
        for (let i = 1; i <= n; i++) {
            const v = readDFUConfigValues(i);
            appendIfActive(v.dfuIInterface, v.dfuInterfaceString, `Interface DFU-${i}`);
        }
    }

    const printerCb = document.getElementById('class-printer');
    if (printerCb && printerCb.checked) {
        const n = parseInt((document.getElementById('class-num-printer') || {}).value) || 1;
        for (let i = 1; i <= n; i++) {
            const v = readPrinterConfigValues(i);
            appendIfActive(v.printerIInterface, v.printerInterfaceString, `Interface Printer-${i}`);
        }
    }

    const videoCb = document.getElementById('class-video');
    if (videoCb && videoCb.checked) {
        const n = parseInt((document.getElementById('class-num-video') || {}).value) || 1;
        for (let i = 1; i <= n; i++) {
            const v = readVideoConfigValues(i);
            appendIfActive(v.videoIInterface, v.videoInterfaceString, `Interface Video-${i}`);
        }
    }

    const mtpCb = document.getElementById('class-mtp');
    if (mtpCb && mtpCb.checked) {
        const n = parseInt((document.getElementById('class-num-mtp') || {}).value) || 1;
        for (let i = 1; i <= n; i++) {
            const v = readMTPConfigValues(i);
            appendIfActive(v.mtpIInterface, v.mtpInterfaceString, `Interface MTP-${i}`);
        }
    }

    const ptpCb = document.getElementById('class-ptp');
    if (ptpCb && ptpCb.checked) {
        const n = parseInt((document.getElementById('class-num-ptp') || {}).value) || 1;
        for (let i = 1; i <= n; i++) {
            const v = readPTPConfigValues(i);
            appendIfActive(v.ptpIInterface, v.ptpInterfaceString, `Interface PTP-${i}`);
        }
    }

    const audioCb = document.getElementById('class-audio');
    if (audioCb && audioCb.checked) {
        const n = parseInt((document.getElementById('class-num-audio') || {}).value) || 1;
        for (let i = 1; i <= n; i++) {
            const v = readAudioConfigValues(i);
            appendIfActive(v.audioAcIInterface, v.audioAcInterfaceString, `Interface Audio AC-${i}`, `audio-ac-${i}-languageIdDisplay`);
            appendIfActive(v.audioAsIInterface, v.audioAsInterfaceString, `Interface Audio AS-${i}`, `audio-as-${i}-languageIdDisplay`);
        }
    }

    getCustomStringDescriptorValues().forEach((csd, idx) => {
        if (csd.index >= 1 && csd.text && csd.text.length > 0) {
            stringDescriptors.push({
                index: csd.index,
                text: csd.text,
                name: `Custom String ${idx + 1}`,
                minIndex: 1,
                enabled: true,
                languageSelectorId: `custom-str-lang-${csd.id}`
            });
        }
    });

    stringDescriptors.sort((a, b) => {
        const ia = (a.index && a.index > 0) ? a.index : Infinity;
        const ib = (b.index && b.index > 0) ? b.index : Infinity;
        return ia - ib;
    });

    return stringDescriptors.filter((desc) => {
        const minIdx = typeof desc.minIndex === 'number' ? desc.minIndex : 1;
        return desc.enabled && desc.index >= minIdx && !!desc.text && desc.text.length > 0;
    });
}

// Returns the flat byte array for device_framework_language_id.
function buildLanguageIdBytes() {
    const bytes = [];
    let hasAny = false;
    STRING_LANGUAGE_OPTIONS.forEach((lang) => {
        const cb = document.getElementById(lang.id);
        if (cb && cb.checked) {
            bytes.push(lang.code & 0xFF);
            bytes.push((lang.code >> 8) & 0xFF);
            hasAny = true;
        }
    });
    if (!hasAny) {
        // Default: English US 0x0409
        bytes.push(0x09);
        bytes.push(0x04);
    }
    return bytes;
}

// Returns the flat byte array for device_framework_string.
function buildStringFrameworkBytes(values, configValues) {
    const active = buildActiveStringDescriptors(values, configValues);
    const bytes = [];
    active.forEach((desc) => {
        const lang = desc.languageGroup
            ? getSelectedLanguageForGroup(desc.languageGroup)
            : (desc.languageSelectorId
                ? getSelectedLanguageForSelector(desc.languageSelectorId)
                : getPrimaryStringLanguage());
        bytes.push(lang.code & 0xFF);
        bytes.push((lang.code >> 8) & 0xFF);
        bytes.push(desc.index);
        bytes.push(desc.text.length);
        for (let i = 0; i < desc.text.length; i++) {
            bytes.push(desc.text.charCodeAt(i));
        }
    });
    return bytes;
}

function renderCArray(descriptor, customComments) {
    const comments = customComments || PAGE_SCHEMA.descriptorComments;
    const rows = [];
    let lastSectionKey = '';
    const hidClassNumberInput = document.getElementById('class-num-hid');
    const hidClassNumber = Math.max(1, parseInt(hidClassNumberInput && hidClassNumberInput.value, 10) || 1);

    const getHidInterfaceSectionTitle = (instance) => {
        return hidClassNumber === 1
            ? 'HID Interface descriptor'
            : `HID-${instance} Interface descriptor`;
    };

    const getClassInstanceCount = (inputId) => {
        const input = document.getElementById(inputId);
        return Math.max(1, parseInt(input && input.value, 10) || 1);
    };

    const classInstanceCounts = {
        'HID': getClassInstanceCount('class-num-hid'),
        'MSC': getClassInstanceCount('class-num-msc'),
        'DFU': getClassInstanceCount('class-num-dfu'),
        'Printer': getClassInstanceCount('class-num-printer'),
        'Video': getClassInstanceCount('class-num-video'),
        'MTP': getClassInstanceCount('class-num-mtp'),
        'PTP': getClassInstanceCount('class-num-ptp'),
        'CDC ACM': getClassInstanceCount('class-num-cdc'),
        'CDC RNDIS': getClassInstanceCount('class-num-rndis'),
        'CDC ECM': getClassInstanceCount('class-num-ecm'),
        'Audio 1.0': getClassInstanceCount('class-num-audio'),
        'Audio 2.0': getClassInstanceCount('class-num-audio2')
    };

    const escapeRegex = (text) => text.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');

    const normalizeClassSectionTitle = (title) => {
        let normalized = title;

        Object.entries(classInstanceCounts).forEach(([className, instanceCount]) => {
            if (instanceCount !== 1) {
                return;
            }

            const pattern = new RegExp(`${escapeRegex(className)}-1\\b`, 'g');
            normalized = normalized.replace(pattern, className);
        });

        return normalized;
    };

    const mergeLsbMsbRows = (inputRows) => {
        const mergedRows = [];

        for (let i = 0; i < inputRows.length; i++) {
            const current = inputRows[i];
            const next = inputRows[i + 1];
            const currentMatch = String(current.comment || '').match(/^(.*)\sLSB$/);
            const nextMatch = next ? String(next.comment || '').match(/^(.*)\sMSB$/) : null;

            if (currentMatch && nextMatch && currentMatch[1] === nextMatch[1]) {
                mergedRows.push({
                    hex: `${current.hex}, ${next.hex}`,
                    comment: currentMatch[1],
                    comma: next.comma,
                    sectionHeader: current.sectionHeader || next.sectionHeader || ''
                });
                i++;
                continue;
            }

            mergedRows.push(current);
        }

        return mergedRows;
    };

    const getSectionInfo = (index, comment) => {
        if (index < 18) {
            return { key: 'device', title: 'Device descriptor' };
        }

        // Check for [CONFIG] tag
        if (String(comment).startsWith('[CONFIG]')) {
            return { key: 'config', title: 'Configuration descriptor' };
        }

        const cdcIadMatch = String(comment).match(/^\[CDC-IAD-(\d+)\]/);
        if (cdcIadMatch) {
            const instance = cdcIadMatch[1];
            return { key: `CDC-IAD-${instance}`, title: `CDC ACM-${instance} Interface Association descriptor` };
        }

        const rndisIadMatch = String(comment).match(/^\[RNDIS-IAD-(\d+)\]/);
        if (rndisIadMatch) {
            const instance = rndisIadMatch[1];
            return { key: `RNDIS-IAD-${instance}`, title: `CDC RNDIS-${instance} Interface Association descriptor` };
        }

        const ecmIadMatch = String(comment).match(/^\[ECM-IAD-(\d+)\]/);
        if (ecmIadMatch) {
            const instance = ecmIadMatch[1];
            return { key: `ECM-IAD-${instance}`, title: `CDC ECM-${instance} Interface Association descriptor` };
        }

        const audioIadMatch = String(comment).match(/^\[AUDIO-IAD-(\d+)\]/);
        if (audioIadMatch) {
            const instance = audioIadMatch[1];
            return { key: `AUDIO-IAD-${instance}`, title: `Audio 1.0-${instance} Interface Association descriptor` };
        }

        const audio2IadMatch = String(comment).match(/^\[AUDIO2-IAD-(\d+)\]/);
        if (audio2IadMatch) {
            const instance = audio2IadMatch[1];
            return { key: `AUDIO2-IAD-${instance}`, title: `Audio 2.0-${instance} Interface Association descriptor` };
        }

        const epOutMatch = String(comment).match(/^\[EP-OUT-(\d+)\]/);
        if (epOutMatch) {
            const instance = epOutMatch[1];
            return { key: `EP-OUT-${instance}`, title: `HID-${instance} Endpoint OUT Descriptor` };
        }

        // Check for Mass Storage tags
        const mscEpInMatch = String(comment).match(/^\[MSC-EP-IN-(\d+)\]/);
        if (mscEpInMatch) {
            const instance = mscEpInMatch[1];
            return { key: `MSC-EP-IN-${instance}`, title: `MSC-${instance} Endpoint IN Descriptor` };
        }

        const mscEpOutMatch = String(comment).match(/^\[MSC-EP-OUT-(\d+)\]/);
        if (mscEpOutMatch) {
            const instance = mscEpOutMatch[1];
            return { key: `MSC-EP-OUT-${instance}`, title: `MSC-${instance} Endpoint OUT Descriptor` };
        }

        const mscIfMatch = String(comment).match(/^\[MSC-IF-(\d+)\]/);
        if (mscIfMatch) {
            const instance = mscIfMatch[1];
            return { key: `MSC-IF-${instance}`, title: `MSC-${instance} Interface descriptor` };
        }

        const dfuIfMatch = String(comment).match(/^\[DFU-IF-(\d+)\]/);
        if (dfuIfMatch) {
            const instance = dfuIfMatch[1];
            return { key: `DFU-IF-${instance}`, title: `DFU-${instance} Interface descriptor` };
        }

        const dfuFuncMatch = String(comment).match(/^\[DFU-FUNC-(\d+)\]/);
        if (dfuFuncMatch) {
            const instance = dfuFuncMatch[1];
            return { key: `DFU-FUNC-${instance}`, title: `DFU-${instance} Functional descriptor` };
        }

        const printerIfMatch = String(comment).match(/^\[PRN-IF-(\d+)\]/);
        if (printerIfMatch) {
            const instance = printerIfMatch[1];
            return { key: `PRN-IF-${instance}`, title: `Printer-${instance} Interface descriptor` };
        }

        const printerEpInMatch = String(comment).match(/^\[PRN-EP-IN-(\d+)\]/);
        if (printerEpInMatch) {
            const instance = printerEpInMatch[1];
            return { key: `PRN-EP-IN-${instance}`, title: `Printer-${instance} Endpoint IN Descriptor` };
        }

        const printerEpOutMatch = String(comment).match(/^\[PRN-EP-OUT-(\d+)\]/);
        if (printerEpOutMatch) {
            const instance = printerEpOutMatch[1];
            return { key: `PRN-EP-OUT-${instance}`, title: `Printer-${instance} Endpoint OUT Descriptor` };
        }

        const videoIfMatch = String(comment).match(/^\[VID-IF-(\d+)\]/);
        if (videoIfMatch) {
            const instance = videoIfMatch[1];
            return { key: `VID-IF-${instance}`, title: `Video-${instance} Interface descriptor` };
        }

        const videoEpInMatch = String(comment).match(/^\[VID-EP-IN-(\d+)\]/);
        if (videoEpInMatch) {
            const instance = videoEpInMatch[1];
            return { key: `VID-EP-IN-${instance}`, title: `Video-${instance} Endpoint IN Descriptor` };
        }

        const videoEpOutMatch = String(comment).match(/^\[VID-EP-OUT-(\d+)\]/);
        if (videoEpOutMatch) {
            const instance = videoEpOutMatch[1];
            return { key: `VID-EP-OUT-${instance}`, title: `Video-${instance} Endpoint OUT Descriptor` };
        }

        const mtpIfMatch = String(comment).match(/^\[MTP-IF-(\d+)\]/);
        if (mtpIfMatch) {
            const instance = mtpIfMatch[1];
            return { key: `MTP-IF-${instance}`, title: `MTP-${instance} Interface descriptor` };
        }

        const mtpEpInMatch = String(comment).match(/^\[MTP-EP-IN-(\d+)\]/);
        if (mtpEpInMatch) {
            const instance = mtpEpInMatch[1];
            return { key: `MTP-EP-IN-${instance}`, title: `MTP-${instance} Endpoint IN Descriptor` };
        }

        const mtpEpOutMatch = String(comment).match(/^\[MTP-EP-OUT-(\d+)\]/);
        if (mtpEpOutMatch) {
            const instance = mtpEpOutMatch[1];
            return { key: `MTP-EP-OUT-${instance}`, title: `MTP-${instance} Endpoint OUT Descriptor` };
        }

        const ptpIfMatch = String(comment).match(/^\[PTP-IF-(\d+)\]/);
        if (ptpIfMatch) {
            const instance = ptpIfMatch[1];
            return { key: `PTP-IF-${instance}`, title: `PTP-${instance} Interface descriptor` };
        }

        const ptpEpInMatch = String(comment).match(/^\[PTP-EP-IN-(\d+)\]/);
        if (ptpEpInMatch) {
            const instance = ptpEpInMatch[1];
            return { key: `PTP-EP-IN-${instance}`, title: `PTP-${instance} Endpoint IN Descriptor` };
        }

        const ptpEpOutMatch = String(comment).match(/^\[PTP-EP-OUT-(\d+)\]/);
        if (ptpEpOutMatch) {
            const instance = ptpEpOutMatch[1];
            return { key: `PTP-EP-OUT-${instance}`, title: `PTP-${instance} Endpoint OUT Descriptor` };
        }

        // Check for CDC ACM tags
        const cdcCommIfMatch = String(comment).match(/^\[CDC-COMM-IF-(\d+)\]/);
        if (cdcCommIfMatch) {
            const instance = cdcCommIfMatch[1];
            return { key: `CDC-COMM-IF-${instance}`, title: `CDC ACM-${instance} Control Interface descriptor` };
        }

        const cdcHeaderMatch = String(comment).match(/^\[CDC-HEADER-(\d+)\]/);
        if (cdcHeaderMatch) {
            const instance = cdcHeaderMatch[1];
            return { key: `CDC-HEADER-${instance}`, title: `CDC ACM-${instance} Header Functional Descriptor` };
        }

        const cdcCallMgmtMatch = String(comment).match(/^\[CDC-CALL-MGMT-(\d+)\]/);
        if (cdcCallMgmtMatch) {
            const instance = cdcCallMgmtMatch[1];
            return { key: `CDC-CALL-MGMT-${instance}`, title: `CDC ACM-${instance} Call Management Functional Descriptor` };
        }

        const cdcAcmMatch = String(comment).match(/^\[CDC-ACM-(\d+)\]/);
        if (cdcAcmMatch) {
            const instance = cdcAcmMatch[1];
            return { key: `CDC-ACM-${instance}`, title: `CDC ACM-${instance} Abstract Control Management Functional Descriptor` };
        }

        const cdcUnionMatch = String(comment).match(/^\[CDC-UNION-(\d+)\]/);
        if (cdcUnionMatch) {
            const instance = cdcUnionMatch[1];
            return { key: `CDC-UNION-${instance}`, title: `CDC ACM-${instance} Union Functional Descriptor (1 slave interface)` };
        }

        const cdcEpNotifyMatch = String(comment).match(/^\[CDC-EP-NOTIFY-(\d+)\]/);
        if (cdcEpNotifyMatch) {
            const instance = cdcEpNotifyMatch[1];
            return { key: `CDC-EP-NOTIFY-${instance}`, title: `CDC ACM-${instance} Endpoint NOTIFY Descriptor` };
        }

        const cdcDataIfMatch = String(comment).match(/^\[CDC-DATA-IF-(\d+)\]/);
        if (cdcDataIfMatch) {
            const instance = cdcDataIfMatch[1];
            return { key: `CDC-DATA-IF-${instance}`, title: `CDC ACM-${instance} Data Interface Descriptor` };
        }

        const cdcEpInMatch = String(comment).match(/^\[CDC-EP-IN-(\d+)\]/);
        if (cdcEpInMatch) {
            const instance = cdcEpInMatch[1];
            return { key: `CDC-EP-IN-${instance}`, title: `CDC ACM-${instance} Endpoint DATA Descriptor` };
        }

        const cdcEpOutMatch = String(comment).match(/^\[CDC-EP-OUT-(\d+)\]/);
        if (cdcEpOutMatch) {
            const instance = cdcEpOutMatch[1];
            return { key: `CDC-EP-OUT-${instance}`, title: `CDC ACM-${instance} Endpoint DATA Descriptor` };
        }

        const rndisCommIfMatch = String(comment).match(/^\[RNDIS-COMM-IF-(\d+)\]/);
        if (rndisCommIfMatch) {
            const instance = rndisCommIfMatch[1];
            return { key: `RNDIS-COMM-IF-${instance}`, title: `CDC RNDIS-${instance} Control Interface descriptor` };
        }

        const rndisHeaderMatch = String(comment).match(/^\[RNDIS-HEADER-(\d+)\]/);
        if (rndisHeaderMatch) {
            const instance = rndisHeaderMatch[1];
            return { key: `RNDIS-HEADER-${instance}`, title: `CDC RNDIS-${instance} Header Functional Descriptor` };
        }

        const rndisCallMgmtMatch = String(comment).match(/^\[RNDIS-CALL-MGMT-(\d+)\]/);
        if (rndisCallMgmtMatch) {
            const instance = rndisCallMgmtMatch[1];
            return { key: `RNDIS-CALL-MGMT-${instance}`, title: `CDC RNDIS-${instance} Call Management Functional Descriptor` };
        }

        const rndisAcmMatch = String(comment).match(/^\[RNDIS-ACM-(\d+)\]/);
        if (rndisAcmMatch) {
            const instance = rndisAcmMatch[1];
            return { key: `RNDIS-ACM-${instance}`, title: `CDC RNDIS-${instance} ACM Functional Descriptor` };
        }

        const rndisUnionMatch = String(comment).match(/^\[RNDIS-UNION-(\d+)\]/);
        if (rndisUnionMatch) {
            const instance = rndisUnionMatch[1];
            return { key: `RNDIS-UNION-${instance}`, title: `CDC RNDIS-${instance} Union Functional Descriptor` };
        }

        const rndisNotifyMatch = String(comment).match(/^\[RNDIS-EP-NOTIFY-(\d+)\]/);
        if (rndisNotifyMatch) {
            const instance = rndisNotifyMatch[1];
            return { key: `RNDIS-EP-NOTIFY-${instance}`, title: `CDC RNDIS-${instance} Endpoint NOTIFY Descriptor` };
        }

        const rndisDataIfMatch = String(comment).match(/^\[RNDIS-DATA-IF-(\d+)\]/);
        if (rndisDataIfMatch) {
            const instance = rndisDataIfMatch[1];
            return { key: `RNDIS-DATA-IF-${instance}`, title: `CDC RNDIS-${instance} Data Interface Descriptor` };
        }

        const rndisEpInMatch = String(comment).match(/^\[RNDIS-EP-IN-(\d+)\]/);
        if (rndisEpInMatch) {
            const instance = rndisEpInMatch[1];
            return { key: `RNDIS-EP-IN-${instance}`, title: `CDC RNDIS-${instance} Endpoint DATA Descriptor` };
        }

        const rndisEpOutMatch = String(comment).match(/^\[RNDIS-EP-OUT-(\d+)\]/);
        if (rndisEpOutMatch) {
            const instance = rndisEpOutMatch[1];
            return { key: `RNDIS-EP-OUT-${instance}`, title: `CDC RNDIS-${instance} Endpoint DATA Descriptor` };
        }

        const ecmCommIfMatch = String(comment).match(/^\[ECM-COMM-IF-(\d+)\]/);
        if (ecmCommIfMatch) {
            const instance = ecmCommIfMatch[1];
            return { key: `ECM-COMM-IF-${instance}`, title: `CDC ECM-${instance} Control Interface descriptor` };
        }

        const ecmHeaderMatch = String(comment).match(/^\[ECM-HEADER-(\d+)\]/);
        if (ecmHeaderMatch) {
            const instance = ecmHeaderMatch[1];
            return { key: `ECM-HEADER-${instance}`, title: `CDC ECM-${instance} Header Functional Descriptor` };
        }

        const ecmEthMatch = String(comment).match(/^\[ECM-ETH-(\d+)\]/);
        if (ecmEthMatch) {
            const instance = ecmEthMatch[1];
            return { key: `ECM-ETH-${instance}`, title: `CDC ECM-${instance} Ethernet Functional Descriptor` };
        }

        const ecmUnionMatch = String(comment).match(/^\[ECM-UNION-(\d+)\]/);
        if (ecmUnionMatch) {
            const instance = ecmUnionMatch[1];
            return { key: `ECM-UNION-${instance}`, title: `CDC ECM-${instance} Union Functional Descriptor` };
        }

        const ecmNotifyMatch = String(comment).match(/^\[ECM-EP-NOTIFY-(\d+)\]/);
        if (ecmNotifyMatch) {
            const instance = ecmNotifyMatch[1];
            return { key: `ECM-EP-NOTIFY-${instance}`, title: `CDC ECM-${instance} Endpoint NOTIFY Descriptor` };
        }

        const ecmDataIfMatch = String(comment).match(/^\[ECM-DATA-IF-(\d+)\]/);
        if (ecmDataIfMatch) {
            const instance = ecmDataIfMatch[1];
            return { key: `ECM-DATA-IF-${instance}`, title: `CDC ECM-${instance} Data Interface Descriptor` };
        }

        const ecmEpInMatch = String(comment).match(/^\[ECM-EP-IN-(\d+)\]/);
        if (ecmEpInMatch) {
            const instance = ecmEpInMatch[1];
            return { key: `ECM-EP-IN-${instance}`, title: `CDC ECM-${instance} Endpoint DATA Descriptor` };
        }

        const ecmEpOutMatch = String(comment).match(/^\[ECM-EP-OUT-(\d+)\]/);
        if (ecmEpOutMatch) {
            const instance = ecmEpOutMatch[1];
            return { key: `ECM-EP-OUT-${instance}`, title: `CDC ECM-${instance} Endpoint DATA Descriptor` };
        }

        // Check for Audio 1.0 tags
        const audioAcIfMatch = String(comment).match(/^\[AUDIO-AC-IF-(\d+)\]/);
        if (audioAcIfMatch) {
            const instance = audioAcIfMatch[1];
            return { key: `AUDIO-AC-IF-${instance}`, title: `Audio 1.0-${instance} Control Interface descriptor` };
        }

        const audioAcCsMatch = String(comment).match(/^\[AUDIO-AC-CS-(\d+)\]/);
        if (audioAcCsMatch) {
            const instance = audioAcCsMatch[1];
            return { key: `AUDIO-AC-CS-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Interface Descriptor` };
        }

        const audioAcItMatch = String(comment).match(/^\[AUDIO-AC-IT-(\d+)\]/);
        if (audioAcItMatch) {
            const instance = audioAcItMatch[1];
            return { key: `AUDIO-AC-IT-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Input Terminal descriptor` };
        }

        const audioAcFuMatch = String(comment).match(/^\[AUDIO-AC-FU-(\d+)\]/);
        if (audioAcFuMatch) {
            const instance = audioAcFuMatch[1];
            return { key: `AUDIO-AC-FU-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Feature Unit descriptor` };
        }

        const audioAcOtMatch = String(comment).match(/^\[AUDIO-AC-OT-(\d+)\]/);
        if (audioAcOtMatch) {
            const instance = audioAcOtMatch[1];
            return { key: `AUDIO-AC-OT-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Output Terminal descriptor` };
        }

        const audioAcEpMatch = String(comment).match(/^\[AUDIO-AC-EP-(\d+)\]/);
        if (audioAcEpMatch) {
            const instance = audioAcEpMatch[1];
            return { key: `AUDIO-AC-EP-${instance}`, title: `Audio 1.0-${instance} AC Interrupt Endpoint Descriptor` };
        }

        const audioAsIf0Match = String(comment).match(/^\[AUDIO-AS-IF0-(\d+)\]/);
        if (audioAsIf0Match) {
            const instance = audioAsIf0Match[1];
            return { key: `AUDIO-AS-IF0-${instance}`, title: `Audio 1.0-${instance} Streaming Interface descriptor (Alt 0)` };
        }

        const audioAsIf1Match = String(comment).match(/^\[AUDIO-AS-IF1-(\d+)\]/);
        if (audioAsIf1Match) {
            const instance = audioAsIf1Match[1];
            return { key: `AUDIO-AS-IF1-${instance}`, title: `Audio 1.0-${instance} Streaming Interface descriptor (Alt 1)` };
        }

        const audioAsCsGenMatch = String(comment).match(/^\[AUDIO-AS-CS-GEN-(\d+)\]/);
        if (audioAsCsGenMatch) {
            const instance = audioAsCsGenMatch[1];
            return { key: `AUDIO-AS-CS-GEN-${instance}`, title: `Audio 1.0-${instance} Class-Specific AS General Descriptor` };
        }

        const audioAsCsFmtMatch = String(comment).match(/^\[AUDIO-AS-CS-FMT-(\d+)\]/);
        if (audioAsCsFmtMatch) {
            const instance = audioAsCsFmtMatch[1];
            return { key: `AUDIO-AS-CS-FMT-${instance}`, title: `Audio 1.0-${instance} Type I Format Descriptor` };
        }

        const audioEpMatch = String(comment).match(/^\[AUDIO-EP-(\d+)\]/);
        if (audioEpMatch) {
            const instance = audioEpMatch[1];
            return { key: `AUDIO-EP-${instance}`, title: `Audio 1.0-${instance} Isochronous Endpoint Descriptor` };
        }

        const audioCsEpMatch = String(comment).match(/^\[AUDIO-CS-EP-(\d+)\]/);
        if (audioCsEpMatch) {
            const instance = audioCsEpMatch[1];
            return { key: `AUDIO-CS-EP-${instance}`, title: `Audio 1.0-${instance} Class-Specific Iso Endpoint Descriptor` };
        }

        const audioFbEpMatch = String(comment).match(/^\[AUDIO-FB-EP-(\d+)\]/);
        if (audioFbEpMatch) {
            const instance = audioFbEpMatch[1];
            return { key: `AUDIO-FB-EP-${instance}`, title: `Audio 1.0-${instance} Feedback Endpoint Descriptor` };
        }

        // Check for Audio 2.0 tags
        const audio2AcIfMatch = String(comment).match(/^\[AUDIO2-AC-IF-(\d+)\]/);
        if (audio2AcIfMatch) {
            const instance = audio2AcIfMatch[1];
            return { key: `AUDIO2-AC-IF-${instance}`, title: `Audio 2.0-${instance} Control Interface descriptor` };
        }

        const audio2AcHeaderMatch = String(comment).match(/^\[AUDIO2-AC-HDR-(\d+)\]/);
        if (audio2AcHeaderMatch) {
            const instance = audio2AcHeaderMatch[1];
            return { key: `AUDIO2-AC-HDR-${instance}`, title: `Audio 2.0-${instance} Class-Specific AC Header Descriptor` };
        }

        const audio2ClockMatch = String(comment).match(/^\[AUDIO2-CLOCK-(\d+)\]/);
        if (audio2ClockMatch) {
            const instance = audio2ClockMatch[1];
            return { key: `AUDIO2-CLOCK-${instance}`, title: `Audio 2.0-${instance} Clock Source Descriptor` };
        }

        const audio2InTermMatch = String(comment).match(/^\[AUDIO2-INTERM-(\d+)\]/);
        if (audio2InTermMatch) {
            const instance = audio2InTermMatch[1];
            return { key: `AUDIO2-INTERM-${instance}`, title: `Audio 2.0-${instance} Input Terminal Descriptor` };
        }

        const audio2OutTermMatch = String(comment).match(/^\[AUDIO2-OUTTERM-(\d+)\]/);
        if (audio2OutTermMatch) {
            const instance = audio2OutTermMatch[1];
            return { key: `AUDIO2-OUTTERM-${instance}`, title: `Audio 2.0-${instance} Output Terminal Descriptor` };
        }

        const audio2AsIf0Match = String(comment).match(/^\[AUDIO2-AS-IF0-(\d+)\]/);
        if (audio2AsIf0Match) {
            const instance = audio2AsIf0Match[1];
            return { key: `AUDIO2-AS-IF0-${instance}`, title: `Audio 2.0-${instance} Streaming Interface descriptor (Alt 0)` };
        }

        const audio2AsIf1Match = String(comment).match(/^\[AUDIO2-AS-IF1-(\d+)\]/);
        if (audio2AsIf1Match) {
            const instance = audio2AsIf1Match[1];
            return { key: `AUDIO2-AS-IF1-${instance}`, title: `Audio 2.0-${instance} Streaming Interface descriptor (Alt 1)` };
        }

        const audio2AsGenMatch = String(comment).match(/^\[AUDIO2-AS-GEN-(\d+)\]/);
        if (audio2AsGenMatch) {
            const instance = audio2AsGenMatch[1];
            return { key: `AUDIO2-AS-GEN-${instance}`, title: `Audio 2.0-${instance} Class-Specific AS General Descriptor` };
        }

        const audio2FmtMatch = String(comment).match(/^\[AUDIO2-AS-FMT-(\d+)\]/);
        if (audio2FmtMatch) {
            const instance = audio2FmtMatch[1];
            return { key: `AUDIO2-AS-FMT-${instance}`, title: `Audio 2.0-${instance} Type I Format Descriptor` };
        }

        const audio2EpMatch = String(comment).match(/^\[AUDIO2-EP-(\d+)\]/);
        if (audio2EpMatch) {
            const instance = audio2EpMatch[1];
            return { key: `AUDIO2-EP-${instance}`, title: `Audio 2.0-${instance} Isochronous Endpoint Descriptor` };
        }

        const audio2CsEpMatch = String(comment).match(/^\[AUDIO2-CS-EP-(\d+)\]/);
        if (audio2CsEpMatch) {
            const instance = audio2CsEpMatch[1];
            return { key: `AUDIO2-CS-EP-${instance}`, title: `Audio 2.0-${instance} Class-Specific Iso Endpoint Descriptor` };
        }

        const match = String(comment).match(/^\[(IF|HID|EP)-(\d+)\]/);
        if (!match) {
            return { key: 'other', title: 'Additional descriptor data' };
        }

        const kind = match[1];
        const instance = match[2];

        if (kind === 'IF') {
            return { key: `IF-${instance}`, title: getHidInterfaceSectionTitle(instance) };
        }

        if (kind === 'HID') {
            return { key: `HID-${instance}`, title: `HID-${instance} Descriptor` };
        }

        return { key: `EP-${instance}`, title: `HID-${instance} Endpoint IN Descriptor` };
    };

    descriptor.forEach((byte, index) => {
        const comment = comments[index] || `Byte ${index}`;
        const sectionInfo = getSectionInfo(index, comment);
        const sectionTitle = normalizeClassSectionTitle(sectionInfo.title);
        const sectionHeader = sectionInfo.key !== lastSectionKey ? sectionTitle : '';
        const displayComment = String(comment).replace(/^\[(IF|HID|EP|EP-OUT|CONFIG|MSC-IF|MSC-EP-IN|MSC-EP-OUT|DFU-IF|DFU-FUNC|PRN-IF|PRN-EP-IN|PRN-EP-OUT|VID-IF|VID-EP-IN|VID-EP-OUT|MTP-IF|MTP-EP-IN|MTP-EP-OUT|PTP-IF|PTP-EP-IN|PTP-EP-OUT|CDC-IAD|CDC-COMM-IF|CDC-HEADER|CDC-CALL-MGMT|CDC-ACM|CDC-UNION|CDC-EP-NOTIFY|CDC-DATA-IF|CDC-EP-IN|CDC-EP-OUT|RNDIS-IAD|RNDIS-COMM-IF|RNDIS-HEADER|RNDIS-CALL-MGMT|RNDIS-ACM|RNDIS-UNION|RNDIS-EP-NOTIFY|RNDIS-DATA-IF|RNDIS-EP-IN|RNDIS-EP-OUT|ECM-IAD|ECM-COMM-IF|ECM-HEADER|ECM-ETH|ECM-UNION|ECM-EP-NOTIFY|ECM-DATA-IF|ECM-EP-IN|ECM-EP-OUT|AUDIO-IAD|AUDIO-AC-IF|AUDIO-AC-CS|AUDIO-AC-IT|AUDIO-AC-FU|AUDIO-AC-OT|AUDIO-AC-EP|AUDIO-AS-IF0|AUDIO-AS-IF1|AUDIO-AS-CS-GEN|AUDIO-AS-CS-FMT|AUDIO-EP|AUDIO-CS-EP|AUDIO-FB-EP|AUDIO2-IAD|AUDIO2-AC-IF|AUDIO2-AC-HDR|AUDIO2-CLOCK|AUDIO2-INTERM|AUDIO2-OUTTERM|AUDIO2-AS-IF0|AUDIO2-AS-IF1|AUDIO2-AS-GEN|AUDIO2-AS-FMT|AUDIO2-EP|AUDIO2-CS-EP)-?\d*\]\s*/, '');

        rows.push({
            hex: toHex(byte, 1),
            comment: displayComment,
            comma: index < descriptor.length - 1,
            sectionHeader: sectionHeader
        });

        lastSectionKey = sectionInfo.key;
    });

    // Check if high speed is enabled
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let hsRows = null;

    if (isHighSpeedEnabled) {
        // Generate high speed descriptor
        const hsDescriptor = buildHighSpeedDescriptor();

        if (hsDescriptor && hsDescriptor.descriptor && hsDescriptor.comments) {
            hsRows = [];
            let hsLastSectionKey = '';

            const getSectionInfoHS = (index, comment) => {
                if (index < 18) {
                    return { key: 'device', title: 'Device descriptor' };
                }

                if (index >= 18 && index < 28) {
                    return { key: 'qualifier', title: 'Device Qualifier descriptor' };
                }

                if (String(comment).startsWith('[CONFIG]')) {
                    return { key: 'config', title: 'Configuration descriptor' };
                }

                const cdcIadMatch = String(comment).match(/^\[CDC-IAD-(\d+)\]/);
                if (cdcIadMatch) {
                    const instance = cdcIadMatch[1];
                    return { key: `CDC-IAD-${instance}`, title: `CDC ACM-${instance} Interface Association descriptor` };
                }

                const rndisIadMatch = String(comment).match(/^\[RNDIS-IAD-(\d+)\]/);
                if (rndisIadMatch) {
                    const instance = rndisIadMatch[1];
                    return { key: `RNDIS-IAD-${instance}`, title: `CDC RNDIS-${instance} Interface Association descriptor` };
                }

                const ecmIadMatch = String(comment).match(/^\[ECM-IAD-(\d+)\]/);
                if (ecmIadMatch) {
                    const instance = ecmIadMatch[1];
                    return { key: `ECM-IAD-${instance}`, title: `CDC ECM-${instance} Interface Association descriptor` };
                }

                const audioIadMatch = String(comment).match(/^\[AUDIO-IAD-(\d+)\]/);
                if (audioIadMatch) {
                    const instance = audioIadMatch[1];
                    return { key: `AUDIO-IAD-${instance}`, title: `Audio 1.0-${instance} Interface Association descriptor` };
                }

                const audio2IadMatch = String(comment).match(/^\[AUDIO2-IAD-(\d+)\]/);
                if (audio2IadMatch) {
                    const instance = audio2IadMatch[1];
                    return { key: `AUDIO2-IAD-${instance}`, title: `Audio 2.0-${instance} Interface Association descriptor` };
                }

                const epOutMatch = String(comment).match(/^\[EP-OUT-(\d+)\]/);
                if (epOutMatch) {
                    const instance = epOutMatch[1];
                    return { key: `EP-OUT-${instance}`, title: `HID-${instance} Endpoint OUT Descriptor` };
                }

                // Check for Mass Storage tags
                const mscEpInMatch = String(comment).match(/^\[MSC-EP-IN-(\d+)\]/);
                if (mscEpInMatch) {
                    const instance = mscEpInMatch[1];
                    return { key: `MSC-EP-IN-${instance}`, title: `MSC-${instance} Endpoint IN Descriptor` };
                }

                const mscEpOutMatch = String(comment).match(/^\[MSC-EP-OUT-(\d+)\]/);
                if (mscEpOutMatch) {
                    const instance = mscEpOutMatch[1];
                    return { key: `MSC-EP-OUT-${instance}`, title: `MSC-${instance} Endpoint OUT Descriptor` };
                }

                const mscIfMatch = String(comment).match(/^\[MSC-IF-(\d+)\]/);
                if (mscIfMatch) {
                    const instance = mscIfMatch[1];
                    return { key: `MSC-IF-${instance}`, title: `MSC-${instance} Interface descriptor` };
                }

                const dfuIfMatch = String(comment).match(/^\[DFU-IF-(\d+)\]/);
                if (dfuIfMatch) {
                    const instance = dfuIfMatch[1];
                    return { key: `DFU-IF-${instance}`, title: `DFU-${instance} Interface descriptor` };
                }

                const dfuFuncMatch = String(comment).match(/^\[DFU-FUNC-(\d+)\]/);
                if (dfuFuncMatch) {
                    const instance = dfuFuncMatch[1];
                    return { key: `DFU-FUNC-${instance}`, title: `DFU-${instance} Functional descriptor` };
                }

                const printerIfMatch = String(comment).match(/^\[PRN-IF-(\d+)\]/);
                if (printerIfMatch) {
                    const instance = printerIfMatch[1];
                    return { key: `PRN-IF-${instance}`, title: `Printer-${instance} Interface descriptor` };
                }

                const printerEpInMatch = String(comment).match(/^\[PRN-EP-IN-(\d+)\]/);
                if (printerEpInMatch) {
                    const instance = printerEpInMatch[1];
                    return { key: `PRN-EP-IN-${instance}`, title: `Printer-${instance} Endpoint IN Descriptor` };
                }

                const printerEpOutMatch = String(comment).match(/^\[PRN-EP-OUT-(\d+)\]/);
                if (printerEpOutMatch) {
                    const instance = printerEpOutMatch[1];
                    return { key: `PRN-EP-OUT-${instance}`, title: `Printer-${instance} Endpoint OUT Descriptor` };
                }

                const videoIfMatch = String(comment).match(/^\[VID-IF-(\d+)\]/);
                if (videoIfMatch) {
                    const instance = videoIfMatch[1];
                    return { key: `VID-IF-${instance}`, title: `Video-${instance} Interface descriptor` };
                }

                const videoEpInMatch = String(comment).match(/^\[VID-EP-IN-(\d+)\]/);
                if (videoEpInMatch) {
                    const instance = videoEpInMatch[1];
                    return { key: `VID-EP-IN-${instance}`, title: `Video-${instance} Endpoint IN Descriptor` };
                }

                const videoEpOutMatch = String(comment).match(/^\[VID-EP-OUT-(\d+)\]/);
                if (videoEpOutMatch) {
                    const instance = videoEpOutMatch[1];
                    return { key: `VID-EP-OUT-${instance}`, title: `Video-${instance} Endpoint OUT Descriptor` };
                }

                const mtpIfMatch = String(comment).match(/^\[MTP-IF-(\d+)\]/);
                if (mtpIfMatch) {
                    const instance = mtpIfMatch[1];
                    return { key: `MTP-IF-${instance}`, title: `MTP-${instance} Interface descriptor` };
                }

                const mtpEpInMatch = String(comment).match(/^\[MTP-EP-IN-(\d+)\]/);
                if (mtpEpInMatch) {
                    const instance = mtpEpInMatch[1];
                    return { key: `MTP-EP-IN-${instance}`, title: `MTP-${instance} Endpoint IN Descriptor` };
                }

                const mtpEpOutMatch = String(comment).match(/^\[MTP-EP-OUT-(\d+)\]/);
                if (mtpEpOutMatch) {
                    const instance = mtpEpOutMatch[1];
                    return { key: `MTP-EP-OUT-${instance}`, title: `MTP-${instance} Endpoint OUT Descriptor` };
                }

                const ptpIfMatch = String(comment).match(/^\[PTP-IF-(\d+)\]/);
                if (ptpIfMatch) {
                    const instance = ptpIfMatch[1];
                    return { key: `PTP-IF-${instance}`, title: `PTP-${instance} Interface descriptor` };
                }

                const ptpEpInMatch = String(comment).match(/^\[PTP-EP-IN-(\d+)\]/);
                if (ptpEpInMatch) {
                    const instance = ptpEpInMatch[1];
                    return { key: `PTP-EP-IN-${instance}`, title: `PTP-${instance} Endpoint IN Descriptor` };
                }

                const ptpEpOutMatch = String(comment).match(/^\[PTP-EP-OUT-(\d+)\]/);
                if (ptpEpOutMatch) {
                    const instance = ptpEpOutMatch[1];
                    return { key: `PTP-EP-OUT-${instance}`, title: `PTP-${instance} Endpoint OUT Descriptor` };
                }

                // Check for CDC ACM tags
                const cdcCommIfMatch = String(comment).match(/^\[CDC-COMM-IF-(\d+)\]/);
                if (cdcCommIfMatch) {
                    const instance = cdcCommIfMatch[1];
                    return { key: `CDC-COMM-IF-${instance}`, title: `CDC ACM-${instance} Control Interface descriptor` };
                }

                const cdcHeaderMatch = String(comment).match(/^\[CDC-HEADER-(\d+)\]/);
                if (cdcHeaderMatch) {
                    const instance = cdcHeaderMatch[1];
                    return { key: `CDC-HEADER-${instance}`, title: `CDC ACM-${instance} Header Functional Descriptor` };
                }

                const cdcCallMgmtMatch = String(comment).match(/^\[CDC-CALL-MGMT-(\d+)\]/);
                if (cdcCallMgmtMatch) {
                    const instance = cdcCallMgmtMatch[1];
                    return { key: `CDC-CALL-MGMT-${instance}`, title: `CDC ACM-${instance} Call Management Functional Descriptor` };
                }

                const cdcAcmMatch = String(comment).match(/^\[CDC-ACM-(\d+)\]/);
                if (cdcAcmMatch) {
                    const instance = cdcAcmMatch[1];
                    return { key: `CDC-ACM-${instance}`, title: `CDC ACM-${instance} Abstract Control Management Functional Descriptor` };
                }

                const cdcUnionMatch = String(comment).match(/^\[CDC-UNION-(\d+)\]/);
                if (cdcUnionMatch) {
                    const instance = cdcUnionMatch[1];
                    return { key: `CDC-UNION-${instance}`, title: `CDC ACM-${instance} Union Functional Descriptor (1 slave interface)` };
                }

                const cdcEpNotifyMatch = String(comment).match(/^\[CDC-EP-NOTIFY-(\d+)\]/);
                if (cdcEpNotifyMatch) {
                    const instance = cdcEpNotifyMatch[1];
                    return { key: `CDC-EP-NOTIFY-${instance}`, title: `CDC ACM-${instance} Endpoint NOTIFY Descriptor` };
                }

                const cdcDataIfMatch = String(comment).match(/^\[CDC-DATA-IF-(\d+)\]/);
                if (cdcDataIfMatch) {
                    const instance = cdcDataIfMatch[1];
                    return { key: `CDC-DATA-IF-${instance}`, title: `CDC ACM-${instance} Data Interface Descriptor` };
                }

                const cdcEpInMatch = String(comment).match(/^\[CDC-EP-IN-(\d+)\]/);
                if (cdcEpInMatch) {
                    const instance = cdcEpInMatch[1];
                    return { key: `CDC-EP-IN-${instance}`, title: `CDC ACM-${instance} Endpoint DATA Descriptor` };
                }

                const cdcEpOutMatch = String(comment).match(/^\[CDC-EP-OUT-(\d+)\]/);
                if (cdcEpOutMatch) {
                    const instance = cdcEpOutMatch[1];
                    return { key: `CDC-EP-OUT-${instance}`, title: `CDC ACM-${instance} Endpoint DATA Descriptor` };
                }

                const rndisCommIfMatch = String(comment).match(/^\[RNDIS-COMM-IF-(\d+)\]/);
                if (rndisCommIfMatch) {
                    const instance = rndisCommIfMatch[1];
                    return { key: `RNDIS-COMM-IF-${instance}`, title: `CDC RNDIS-${instance} Control Interface descriptor` };
                }

                const rndisHeaderMatch = String(comment).match(/^\[RNDIS-HEADER-(\d+)\]/);
                if (rndisHeaderMatch) {
                    const instance = rndisHeaderMatch[1];
                    return { key: `RNDIS-HEADER-${instance}`, title: `CDC RNDIS-${instance} Header Functional Descriptor` };
                }

                const rndisCallMgmtMatch = String(comment).match(/^\[RNDIS-CALL-MGMT-(\d+)\]/);
                if (rndisCallMgmtMatch) {
                    const instance = rndisCallMgmtMatch[1];
                    return { key: `RNDIS-CALL-MGMT-${instance}`, title: `CDC RNDIS-${instance} Call Management Functional Descriptor` };
                }

                const rndisAcmMatch = String(comment).match(/^\[RNDIS-ACM-(\d+)\]/);
                if (rndisAcmMatch) {
                    const instance = rndisAcmMatch[1];
                    return { key: `RNDIS-ACM-${instance}`, title: `CDC RNDIS-${instance} ACM Functional Descriptor` };
                }

                const rndisUnionMatch = String(comment).match(/^\[RNDIS-UNION-(\d+)\]/);
                if (rndisUnionMatch) {
                    const instance = rndisUnionMatch[1];
                    return { key: `RNDIS-UNION-${instance}`, title: `CDC RNDIS-${instance} Union Functional Descriptor` };
                }

                const rndisNotifyMatch = String(comment).match(/^\[RNDIS-EP-NOTIFY-(\d+)\]/);
                if (rndisNotifyMatch) {
                    const instance = rndisNotifyMatch[1];
                    return { key: `RNDIS-EP-NOTIFY-${instance}`, title: `CDC RNDIS-${instance} Endpoint NOTIFY Descriptor` };
                }

                const rndisDataIfMatch = String(comment).match(/^\[RNDIS-DATA-IF-(\d+)\]/);
                if (rndisDataIfMatch) {
                    const instance = rndisDataIfMatch[1];
                    return { key: `RNDIS-DATA-IF-${instance}`, title: `CDC RNDIS-${instance} Data Interface Descriptor` };
                }

                const rndisEpInMatch = String(comment).match(/^\[RNDIS-EP-IN-(\d+)\]/);
                if (rndisEpInMatch) {
                    const instance = rndisEpInMatch[1];
                    return { key: `RNDIS-EP-IN-${instance}`, title: `CDC RNDIS-${instance} Endpoint DATA Descriptor` };
                }

                const rndisEpOutMatch = String(comment).match(/^\[RNDIS-EP-OUT-(\d+)\]/);
                if (rndisEpOutMatch) {
                    const instance = rndisEpOutMatch[1];
                    return { key: `RNDIS-EP-OUT-${instance}`, title: `CDC RNDIS-${instance} Endpoint DATA Descriptor` };
                }

                const ecmCommIfMatch = String(comment).match(/^\[ECM-COMM-IF-(\d+)\]/);
                if (ecmCommIfMatch) {
                    const instance = ecmCommIfMatch[1];
                    return { key: `ECM-COMM-IF-${instance}`, title: `CDC ECM-${instance} Control Interface descriptor` };
                }

                const ecmHeaderMatch = String(comment).match(/^\[ECM-HEADER-(\d+)\]/);
                if (ecmHeaderMatch) {
                    const instance = ecmHeaderMatch[1];
                    return { key: `ECM-HEADER-${instance}`, title: `CDC ECM-${instance} Header Functional Descriptor` };
                }

                const ecmEthMatch = String(comment).match(/^\[ECM-ETH-(\d+)\]/);
                if (ecmEthMatch) {
                    const instance = ecmEthMatch[1];
                    return { key: `ECM-ETH-${instance}`, title: `CDC ECM-${instance} Ethernet Functional Descriptor` };
                }

                const ecmUnionMatch = String(comment).match(/^\[ECM-UNION-(\d+)\]/);
                if (ecmUnionMatch) {
                    const instance = ecmUnionMatch[1];
                    return { key: `ECM-UNION-${instance}`, title: `CDC ECM-${instance} Union Functional Descriptor` };
                }

                const ecmNotifyMatch = String(comment).match(/^\[ECM-EP-NOTIFY-(\d+)\]/);
                if (ecmNotifyMatch) {
                    const instance = ecmNotifyMatch[1];
                    return { key: `ECM-EP-NOTIFY-${instance}`, title: `CDC ECM-${instance} Endpoint NOTIFY Descriptor` };
                }

                const ecmDataIfMatch = String(comment).match(/^\[ECM-DATA-IF-(\d+)\]/);
                if (ecmDataIfMatch) {
                    const instance = ecmDataIfMatch[1];
                    return { key: `ECM-DATA-IF-${instance}`, title: `CDC ECM-${instance} Data Interface Descriptor` };
                }

                const ecmEpInMatch = String(comment).match(/^\[ECM-EP-IN-(\d+)\]/);
                if (ecmEpInMatch) {
                    const instance = ecmEpInMatch[1];
                    return { key: `ECM-EP-IN-${instance}`, title: `CDC ECM-${instance} Endpoint DATA Descriptor` };
                }

                const ecmEpOutMatch = String(comment).match(/^\[ECM-EP-OUT-(\d+)\]/);
                if (ecmEpOutMatch) {
                    const instance = ecmEpOutMatch[1];
                    return { key: `ECM-EP-OUT-${instance}`, title: `CDC ECM-${instance} Endpoint DATA Descriptor` };
                }

                // Check for Audio 1.0 tags
                const audioAcIfMatch = String(comment).match(/^\[AUDIO-AC-IF-(\d+)\]/);
                if (audioAcIfMatch) {
                    const instance = audioAcIfMatch[1];
                    return { key: `AUDIO-AC-IF-${instance}`, title: `Audio 1.0-${instance} Control Interface descriptor` };
                }

                const audioAcCsMatch = String(comment).match(/^\[AUDIO-AC-CS-(\d+)\]/);
                if (audioAcCsMatch) {
                    const instance = audioAcCsMatch[1];
                    return { key: `AUDIO-AC-CS-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Interface Descriptor` };
                }

                const audioAcItMatch = String(comment).match(/^\[AUDIO-AC-IT-(\d+)\]/);
                if (audioAcItMatch) {
                    const instance = audioAcItMatch[1];
                    return { key: `AUDIO-AC-IT-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Input Terminal descriptor` };
                }

                const audioAcFuMatch = String(comment).match(/^\[AUDIO-AC-FU-(\d+)\]/);
                if (audioAcFuMatch) {
                    const instance = audioAcFuMatch[1];
                    return { key: `AUDIO-AC-FU-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Feature Unit descriptor` };
                }

                const audioAcOtMatch = String(comment).match(/^\[AUDIO-AC-OT-(\d+)\]/);
                if (audioAcOtMatch) {
                    const instance = audioAcOtMatch[1];
                    return { key: `AUDIO-AC-OT-${instance}`, title: `Audio 1.0-${instance} Class-Specific AC Output Terminal descriptor` };
                }

                const audioAcEpMatch = String(comment).match(/^\[AUDIO-AC-EP-(\d+)\]/);
                if (audioAcEpMatch) {
                    const instance = audioAcEpMatch[1];
                    return { key: `AUDIO-AC-EP-${instance}`, title: `Audio 1.0-${instance} AC Interrupt Endpoint Descriptor` };
                }

                const audioAsIf0Match = String(comment).match(/^\[AUDIO-AS-IF0-(\d+)\]/);
                if (audioAsIf0Match) {
                    const instance = audioAsIf0Match[1];
                    return { key: `AUDIO-AS-IF0-${instance}`, title: `Audio 1.0-${instance} Streaming Interface descriptor (Alt 0)` };
                }

                const audioAsIf1Match = String(comment).match(/^\[AUDIO-AS-IF1-(\d+)\]/);
                if (audioAsIf1Match) {
                    const instance = audioAsIf1Match[1];
                    return { key: `AUDIO-AS-IF1-${instance}`, title: `Audio 1.0-${instance} Streaming Interface descriptor (Alt 1)` };
                }

                const audioAsCsGenMatch = String(comment).match(/^\[AUDIO-AS-CS-GEN-(\d+)\]/);
                if (audioAsCsGenMatch) {
                    const instance = audioAsCsGenMatch[1];
                    return { key: `AUDIO-AS-CS-GEN-${instance}`, title: `Audio 1.0-${instance} Class-Specific AS General Descriptor` };
                }

                const audioAsCsFmtMatch = String(comment).match(/^\[AUDIO-AS-CS-FMT-(\d+)\]/);
                if (audioAsCsFmtMatch) {
                    const instance = audioAsCsFmtMatch[1];
                    return { key: `AUDIO-AS-CS-FMT-${instance}`, title: `Audio 1.0-${instance} Type I Format Descriptor` };
                }

                const audioEpMatch = String(comment).match(/^\[AUDIO-EP-(\d+)\]/);
                if (audioEpMatch) {
                    const instance = audioEpMatch[1];
                    return { key: `AUDIO-EP-${instance}`, title: `Audio 1.0-${instance} Isochronous Endpoint Descriptor` };
                }

                const audioCsEpMatch = String(comment).match(/^\[AUDIO-CS-EP-(\d+)\]/);
                if (audioCsEpMatch) {
                    const instance = audioCsEpMatch[1];
                    return { key: `AUDIO-CS-EP-${instance}`, title: `Audio 1.0-${instance} Class-Specific Iso Endpoint Descriptor` };
                }

                const audioFbEpMatchHS = String(comment).match(/^\[AUDIO-FB-EP-(\d+)\]/);
                if (audioFbEpMatchHS) {
                    const instance = audioFbEpMatchHS[1];
                    return { key: `AUDIO-FB-EP-${instance}`, title: `Audio 1.0-${instance} Feedback Endpoint Descriptor` };
                }

                // Check for Audio 2.0 tags
                const audio2AcIfMatch = String(comment).match(/^\[AUDIO2-AC-IF-(\d+)\]/);
                if (audio2AcIfMatch) {
                    const instance = audio2AcIfMatch[1];
                    return { key: `AUDIO2-AC-IF-${instance}`, title: `Audio 2.0-${instance} Control Interface descriptor` };
                }

                const audio2AcHeaderMatch = String(comment).match(/^\[AUDIO2-AC-HDR-(\d+)\]/);
                if (audio2AcHeaderMatch) {
                    const instance = audio2AcHeaderMatch[1];
                    return { key: `AUDIO2-AC-HDR-${instance}`, title: `Audio 2.0-${instance} Class-Specific AC Header Descriptor` };
                }

                const audio2ClockMatch = String(comment).match(/^\[AUDIO2-CLOCK-(\d+)\]/);
                if (audio2ClockMatch) {
                    const instance = audio2ClockMatch[1];
                    return { key: `AUDIO2-CLOCK-${instance}`, title: `Audio 2.0-${instance} Clock Source Descriptor` };
                }

                const audio2InTermMatch = String(comment).match(/^\[AUDIO2-INTERM-(\d+)\]/);
                if (audio2InTermMatch) {
                    const instance = audio2InTermMatch[1];
                    return { key: `AUDIO2-INTERM-${instance}`, title: `Audio 2.0-${instance} Input Terminal Descriptor` };
                }

                const audio2OutTermMatch = String(comment).match(/^\[AUDIO2-OUTTERM-(\d+)\]/);
                if (audio2OutTermMatch) {
                    const instance = audio2OutTermMatch[1];
                    return { key: `AUDIO2-OUTTERM-${instance}`, title: `Audio 2.0-${instance} Output Terminal Descriptor` };
                }

                const audio2AsIf0Match = String(comment).match(/^\[AUDIO2-AS-IF0-(\d+)\]/);
                if (audio2AsIf0Match) {
                    const instance = audio2AsIf0Match[1];
                    return { key: `AUDIO2-AS-IF0-${instance}`, title: `Audio 2.0-${instance} Streaming Interface descriptor (Alt 0)` };
                }

                const audio2AsIf1Match = String(comment).match(/^\[AUDIO2-AS-IF1-(\d+)\]/);
                if (audio2AsIf1Match) {
                    const instance = audio2AsIf1Match[1];
                    return { key: `AUDIO2-AS-IF1-${instance}`, title: `Audio 2.0-${instance} Streaming Interface descriptor (Alt 1)` };
                }

                const audio2AsGenMatch = String(comment).match(/^\[AUDIO2-AS-GEN-(\d+)\]/);
                if (audio2AsGenMatch) {
                    const instance = audio2AsGenMatch[1];
                    return { key: `AUDIO2-AS-GEN-${instance}`, title: `Audio 2.0-${instance} Class-Specific AS General Descriptor` };
                }

                const audio2FmtMatch = String(comment).match(/^\[AUDIO2-AS-FMT-(\d+)\]/);
                if (audio2FmtMatch) {
                    const instance = audio2FmtMatch[1];
                    return { key: `AUDIO2-AS-FMT-${instance}`, title: `Audio 2.0-${instance} Type I Format Descriptor` };
                }

                const audio2EpMatch = String(comment).match(/^\[AUDIO2-EP-(\d+)\]/);
                if (audio2EpMatch) {
                    const instance = audio2EpMatch[1];
                    return { key: `AUDIO2-EP-${instance}`, title: `Audio 2.0-${instance} Isochronous Endpoint Descriptor` };
                }

                const audio2CsEpMatch = String(comment).match(/^\[AUDIO2-CS-EP-(\d+)\]/);
                if (audio2CsEpMatch) {
                    const instance = audio2CsEpMatch[1];
                    return { key: `AUDIO2-CS-EP-${instance}`, title: `Audio 2.0-${instance} Class-Specific Iso Endpoint Descriptor` };
                }

                const match = String(comment).match(/^\[(IF|HID|EP)-(\d+)\]/);
                if (!match) {
                    return { key: 'other', title: 'Additional descriptor data' };
                }

                const kind = match[1];
                const instance = match[2];

                if (kind === 'IF') {
                    return { key: `IF-${instance}`, title: getHidInterfaceSectionTitle(instance) };
                }

                if (kind === 'HID') {
                    return { key: `HID-${instance}`, title: `HID-${instance} Descriptor` };
                }

                return { key: `EP-${instance}`, title: `HID-${instance} Endpoint IN Descriptor` };
            };

            hsDescriptor.descriptor.forEach((byte, index) => {
                const comment = hsDescriptor.comments[index] || `Byte ${index}`;
                const sectionInfo = getSectionInfoHS(index, comment);
                const sectionTitle = normalizeClassSectionTitle(sectionInfo.title);
                const sectionHeader = sectionInfo.key !== hsLastSectionKey ? sectionTitle : '';
                const displayComment = String(comment).replace(/^\[(IF|HID|EP|EP-OUT|CONFIG|QUALIFIER|MSC-IF|MSC-EP-IN|MSC-EP-OUT|DFU-IF|DFU-FUNC|PRN-IF|PRN-EP-IN|PRN-EP-OUT|VID-IF|VID-EP-IN|VID-EP-OUT|MTP-IF|MTP-EP-IN|MTP-EP-OUT|PTP-IF|PTP-EP-IN|PTP-EP-OUT|CDC-IAD|CDC-COMM-IF|CDC-HEADER|CDC-CALL-MGMT|CDC-ACM|CDC-UNION|CDC-EP-NOTIFY|CDC-DATA-IF|CDC-EP-IN|CDC-EP-OUT|RNDIS-IAD|RNDIS-COMM-IF|RNDIS-HEADER|RNDIS-CALL-MGMT|RNDIS-ACM|RNDIS-UNION|RNDIS-EP-NOTIFY|RNDIS-DATA-IF|RNDIS-EP-IN|RNDIS-EP-OUT|ECM-IAD|ECM-COMM-IF|ECM-HEADER|ECM-ETH|ECM-UNION|ECM-EP-NOTIFY|ECM-DATA-IF|ECM-EP-IN|ECM-EP-OUT|AUDIO-IAD|AUDIO-AC-IF|AUDIO-AC-CS|AUDIO-AC-IT|AUDIO-AC-FU|AUDIO-AC-OT|AUDIO-AC-EP|AUDIO-AS-IF0|AUDIO-AS-IF1|AUDIO-AS-CS-GEN|AUDIO-AS-CS-FMT|AUDIO-EP|AUDIO-CS-EP|AUDIO-FB-EP|AUDIO2-IAD|AUDIO2-AC-IF|AUDIO2-AC-HDR|AUDIO2-CLOCK|AUDIO2-INTERM|AUDIO2-OUTTERM|AUDIO2-AS-IF0|AUDIO2-AS-IF1|AUDIO2-AS-GEN|AUDIO2-AS-FMT|AUDIO2-EP|AUDIO2-CS-EP)-?\d*\]\s*/, '');

                hsRows.push({
                    hex: toHex(byte, 1),
                    comment: displayComment,
                    comma: index < hsDescriptor.descriptor.length - 1,
                    sectionHeader: sectionHeader
                });

                hsLastSectionKey = sectionInfo.key;
            });
        }
    }

    const values = readFormValues();
    const configValues = readConfigurationValues();
    const stringRows = [];

    // String descriptor format:
    // Byte 0-1: Language ID (0x09, 0x04 for US English)
    // Byte 2: String index
    // Byte 3: String length
    // Byte 4+: String characters

    const stringDescriptors = [
        {
            index: values.iManufacturer,
            text: values.manufacturer,
            name: 'Manufacturer',
            minIndex: 1,
            enabled: true,
            languageGroup: STRING_FIELD_GROUPS.find((group) => group.indexFieldId === 'iManufacturer')
        },
        {
            index: values.iProduct,
            text: values.product,
            name: 'Product',
            minIndex: 1,
            enabled: true,
            languageGroup: STRING_FIELD_GROUPS.find((group) => group.indexFieldId === 'iProduct')
        },
        {
            index: values.iSerialNumber,
            text: values.serialNumber,
            name: 'Serial Number',
            minIndex: 1,
            enabled: true,
            languageGroup: STRING_FIELD_GROUPS.find((group) => group.indexFieldId === 'iSerialNumber')
        },
        { index: configValues.iConfiguration, text: configValues.configurationStringFS, name: 'Configuration', minIndex: 1, enabled: true,
            languageGroup: CONFIGURATION_STRING_FIELD_GROUPS.find((g) => g.indexFieldId === 'iConfiguration') },
        { index: configValues.iConfigurationHS, text: configValues.configurationStringHS, name: 'Configuration', minIndex: 1, enabled: isHighSpeedEnabled,
            languageGroup: CONFIGURATION_STRING_FIELD_GROUPS.find((g) => g.indexFieldId === 'iConfigurationHS') }
    ];

    // Collect interface strings from enabled USB classes and append to device_framework_string.
    const appendInterfaceString = (index, text, name, languageSelectorId) => {
        if (index && index > 0 && text) {
            stringDescriptors.push({
                index: index,
                text: text,
                name: name,
                minIndex: 1,
                enabled: true,
                languageSelectorId: languageSelectorId
            });
        }
    };

    const hidCheckbox = document.getElementById('class-hid');
    if (hidCheckbox && hidCheckbox.checked) {
        const hidNumberInput = document.getElementById('class-num-hid');
        const classNumber = parseInt(hidNumberInput.value) || 1;
        for (let i = 1; i <= classNumber; i++) {
            const interfaceValues = readInterfaceDescriptorValues(i);
            appendInterfaceString(
                interfaceValues.iInterface,
                interfaceValues.interfaceString,
                `Interface HID-${i}`,
                `interface-${i}-languageIdDisplay`
            );
        }
    }

    const dfuCheckbox = document.getElementById('class-dfu');
    if (dfuCheckbox && dfuCheckbox.checked) {
        const dfuNumberInput = document.getElementById('class-num-dfu');
        const classNumber = parseInt(dfuNumberInput.value) || 1;
        for (let i = 1; i <= classNumber; i++) {
            const dfuConfigValues = readDFUConfigValues(i);
            appendInterfaceString(dfuConfigValues.dfuIInterface, dfuConfigValues.dfuInterfaceString, `Interface DFU-${i}`);
        }
    }

    const printerCheckbox = document.getElementById('class-printer');
    if (printerCheckbox && printerCheckbox.checked) {
        const printerNumberInput = document.getElementById('class-num-printer');
        const classNumber = parseInt(printerNumberInput.value) || 1;
        for (let i = 1; i <= classNumber; i++) {
            const printerConfigValues = readPrinterConfigValues(i);
            appendInterfaceString(printerConfigValues.printerIInterface, printerConfigValues.printerInterfaceString, `Interface Printer-${i}`);
        }
    }

    const videoCheckbox = document.getElementById('class-video');
    if (videoCheckbox && videoCheckbox.checked) {
        const videoNumberInput = document.getElementById('class-num-video');
        const classNumber = parseInt(videoNumberInput.value) || 1;
        for (let i = 1; i <= classNumber; i++) {
            const videoConfigValues = readVideoConfigValues(i);
            appendInterfaceString(videoConfigValues.videoIInterface, videoConfigValues.videoInterfaceString, `Interface Video-${i}`);
        }
    }

    const mtpCheckbox = document.getElementById('class-mtp');
    if (mtpCheckbox && mtpCheckbox.checked) {
        const mtpNumberInput = document.getElementById('class-num-mtp');
        const classNumber = parseInt(mtpNumberInput.value) || 1;
        for (let i = 1; i <= classNumber; i++) {
            const mtpConfigValues = readMTPConfigValues(i);
            appendInterfaceString(mtpConfigValues.mtpIInterface, mtpConfigValues.mtpInterfaceString, `Interface MTP-${i}`);
        }
    }

    const ptpCheckbox = document.getElementById('class-ptp');
    if (ptpCheckbox && ptpCheckbox.checked) {
        const ptpNumberInput = document.getElementById('class-num-ptp');
        const classNumber = parseInt(ptpNumberInput.value) || 1;
        for (let i = 1; i <= classNumber; i++) {
            const ptpConfigValues = readPTPConfigValues(i);
            appendInterfaceString(ptpConfigValues.ptpIInterface, ptpConfigValues.ptpInterfaceString, `Interface PTP-${i}`);
        }
    }

    const audioCheckboxC = document.getElementById('class-audio');
    if (audioCheckboxC && audioCheckboxC.checked) {
        const audioNumberInputC = document.getElementById('class-num-audio');
        const classNumber = parseInt(audioNumberInputC && audioNumberInputC.value) || 1;
        for (let i = 1; i <= classNumber; i++) {
            const audioVals = readAudioConfigValues(i);
            appendInterfaceString(audioVals.audioAcIInterface, audioVals.audioAcInterfaceString, `Interface Audio AC-${i}`, `audio-ac-${i}-languageIdDisplay`);
            appendInterfaceString(audioVals.audioAsIInterface, audioVals.audioAsInterfaceString, `Interface Audio AS-${i}`, `audio-as-${i}-languageIdDisplay`);
        }
    }

    // Append user-defined custom string descriptors
    getCustomStringDescriptorValues().forEach((csd, idx) => {
        if (csd.index >= 1 && csd.text && csd.text.length > 0) {
            stringDescriptors.push({
                index: csd.index,
                text: csd.text,
                name: `Custom String ${idx + 1}`,
                minIndex: 1,
                enabled: true,
                languageSelectorId: `custom-str-lang-${csd.id}`
            });
        }
    });

    // Sort by string index for consistent ordering in the output
    stringDescriptors.sort((a, b) => {
        const ia = (a.index && a.index > 0) ? a.index : Infinity;
        const ib = (b.index && b.index > 0) ? b.index : Infinity;
        return ia - ib;
    });

    const isStringDescriptorActive = (desc) => {
        const minIndex = typeof desc.minIndex === 'number' ? desc.minIndex : 1;
        return desc.enabled && desc.index >= minIndex && !!desc.text && desc.text.length > 0;
    };

    stringDescriptors.forEach((desc, descIdx) => {
        if (isStringDescriptorActive(desc)) {
            const isLastDescriptor = (descIdx === stringDescriptors.length - 1) ||
                                    !stringDescriptors.slice(descIdx + 1).some((d) => isStringDescriptorActive(d));
            const descriptorHeaderName = desc.name.replace(/\s+/g, '');
            let sectionHeaderLabel = `i${descriptorHeaderName} string descriptor`;
            const hidNameMatch = String(desc.name).match(/^Interface HID-(\d+)$/);
            if (hidNameMatch) {
                const hidInstance = hidNameMatch[1];
                sectionHeaderLabel = hidClassNumber === 1
                    ? 'iInterface HID string descriptor'
                    : `iInterface HID-${hidInstance} string descriptor`;
            }
            const customStringMatch = String(desc.name).match(/^Custom String (\d+)$/);
            if (customStringMatch) {
                sectionHeaderLabel = `Custom string descriptor ${customStringMatch[1]} (index ${desc.index})`;
            }
            const audioAcNameMatch = String(desc.name).match(/^Interface Audio AC-(\d+)$/);
            if (audioAcNameMatch) {
                const audioAcInstance = audioAcNameMatch[1];
                const audioAcClassNumber = parseInt((document.getElementById('class-num-audio') || {}).value) || 1;
                sectionHeaderLabel = audioAcClassNumber === 1
                    ? 'iInterface Audio Control string descriptor'
                    : `iInterface Audio Control-${audioAcInstance} string descriptor`;
            }
            const audioAsNameMatch = String(desc.name).match(/^Interface Audio AS-(\d+)$/);
            if (audioAsNameMatch) {
                const audioAsInstance = audioAsNameMatch[1];
                const audioAsClassNumber = parseInt((document.getElementById('class-num-audio') || {}).value) || 1;
                sectionHeaderLabel = audioAsClassNumber === 1
                    ? 'iInterface Audio Streaming string descriptor'
                    : `iInterface Audio Streaming-${audioAsInstance} string descriptor`;
            }
            const selectedLanguage = desc.languageGroup
                ? getSelectedLanguageForGroup(desc.languageGroup)
                : (desc.languageSelectorId
                    ? getSelectedLanguageForSelector(desc.languageSelectorId)
                    : getPrimaryStringLanguage());
            const languageIdLsb = toHex(selectedLanguage.code & 0xFF, 1);
            const languageIdMsb = toHex((selectedLanguage.code >> 8) & 0xFF, 1);

            // String descriptor marker and Language ID on the same line
            stringRows.push({
                sectionHeader: sectionHeaderLabel,
                hex: `${languageIdLsb}, ${languageIdMsb}`,
                comment: 'Language ID',
                comma: true
            });

            // String index
            stringRows.push({
                hex: toHex(desc.index, 1),
                comment: 'String Index',
                comma: true
            });

            // String length
            stringRows.push({
                hex: toHex(desc.text.length, 1),
                comment: 'String Length',
                comma: true
            });

            // String characters - all on one line
            let charHexValues = [];
            for (let i = 0; i < desc.text.length; i++) {
                charHexValues.push(`'${desc.text[i]}'`);
            }

            stringRows.push({
                hex: charHexValues.join(', '),
                comment: '',
                comma: !isLastDescriptor
            });

            // Add empty line after each descriptor (except the last one)
            if (!isLastDescriptor) {
                stringRows.push({
                    hex: '',
                    comment: '',
                    comma: false,
                    emptyLine: true
                });
            }
        }
    });

    // Generate Language ID array from enabled languages
    const langIdRows = [];

    STRING_LANGUAGE_OPTIONS.forEach((lang, index) => {
        const checkbox = document.getElementById(lang.id);
        if (checkbox && checkbox.checked) {
            const lsb = lang.code & 0xFF;
            const msb = (lang.code >> 8) & 0xFF;
            const isLast = (index === STRING_LANGUAGE_OPTIONS.length - 1 &&
                          !STRING_LANGUAGE_OPTIONS.slice(index + 1).some(l => {
                              const cb = document.getElementById(l.id);
                              return cb && cb.checked;
                          }));

            langIdRows.push({
                hex: `${toHex(lsb, 1)}, ${toHex(msb, 1)}`,
                comment: `Language ID (0x${lang.code.toString(16).toUpperCase().padStart(4, '0')} = ${lang.name})`,
                comma: !isLast
            });
        }
    });

    // If no languages enabled, default to English (US)
    if (langIdRows.length === 0) {
        langIdRows.push({ hex: '0x09, 0x04', comment: 'Language ID (0x0409 = English US)', comma: false });
    } else {
        // Fix the last comma flag
        langIdRows[langIdRows.length - 1].comma = false;
    }

    const mergedRows = mergeLsbMsbRows(rows);
    const mergedHsRows = hsRows ? mergeLsbMsbRows(hsRows) : hsRows;

    // Calculate array sizes by counting hex values
    const calculateArraySize = (rowsArray) => {
        return rowsArray.reduce((total, row) => {
            if (row.hex) {
                // Count the number of hex values (separated by commas)
                const hexCount = row.hex.split(',').length;
                return total + hexCount;
            }
            return total;
        }, 0);
    };

    const fsSize = calculateArraySize(mergedRows);
    const hsSize = mergedHsRows ? calculateArraySize(mergedHsRows) : 0;
    const stringSize = calculateArraySize(stringRows);
    const langIdSize = calculateArraySize(langIdRows);

    document.getElementById('c-array-output').innerHTML = templates.cArray({
        rows: mergedRows,
        hsRows: mergedHsRows,
        stringRows: stringRows,
        langIdRows: langIdRows,
        fsSize: fsSize,
        hsSize: hsSize,
        stringSize: stringSize,
        langIdSize: langIdSize
    });
}

function buildHighSpeedDescriptor() {
    try {
        const values = readFormValues();
        const configValues = readConfigurationValues();

        // Build high-speed device descriptor using HS bMaxPacketSize0
        const hsPacketSize = values.bMaxPacketSize0HS || values.bMaxPacketSize0;
        const hsDescriptor = [
            18,
            0x01,
            values.bcdUSB & 0xFF,
            (values.bcdUSB >> 8) & 0xFF,
            values.bDeviceClass,
            values.bDeviceSubClass,
            values.bDeviceProtocol,
            hsPacketSize,
            values.idVendor & 0xFF,
            (values.idVendor >> 8) & 0xFF,
            values.idProduct & 0xFF,
            (values.idProduct >> 8) & 0xFF,
            values.bcdDevice & 0xFF,
            (values.bcdDevice >> 8) & 0xFF,
            values.iManufacturer,
            values.iProduct,
            values.iSerialNumber,
            values.bNumConfigurations
        ];
        let descriptor = hsDescriptor;
        let commentsList = PAGE_SCHEMA.descriptorComments.slice();

        // Add Device Qualifier Descriptor (10 bytes)
        const qualifierDescriptor = [
            10,     // bLength
            0x06,   // bDescriptorType (Device Qualifier)
            0x00,   // bcdUSB LSB (USB 2.0)
            0x02,   // bcdUSB MSB
            0x00,   // bDeviceClass
            0x00,   // bDeviceSubClass
            0x00,   // bDeviceProtocol
            hsPacketSize,   // bMaxPacketSize0 (use HS value)
            0x01,   // bNumConfigurations
            0x00    // bReserved
        ];
        descriptor = descriptor.concat(qualifierDescriptor);

        // Add comments for Device Qualifier descriptor
        const qualifierComments = [
            'bLength',
            'bDescriptorType',
            'bcdUSB LSB',
            'bcdUSB MSB',
            'bDeviceClass',
            'bDeviceSubClass',
            'bDeviceProtocol',
            'bMaxPacketSize0',
            'bNumConfigurations',
            'bReserved'
        ];
        qualifierComments.forEach((comment) => {
            commentsList.push(`[QUALIFIER] ${comment}`);
        });

        // Add Configuration Descriptor
        const configDescriptor = buildConfigDescriptor(configValues, true);
        descriptor = descriptor.concat(configDescriptor);

        // Add comments for Configuration descriptor
        const configComments = [
            'bLength',
            'bDescriptorType',
            'wTotalLength LSB',
            'wTotalLength MSB',
            'bNumInterfaces',
            'bConfigurationValue',
            'iConfiguration',
            'bmAttributes',
            'bMaxPower'
        ];
        configComments.forEach((comment) => {
            commentsList.push(`[CONFIG] ${comment}`);
        });

        const classInterfacePerInstance = {
            hid: 1,
            msc: 1,
            dfu: 1,
            printer: 1,
            video: 1,
            mtp: 1,
            ptp: 1,
            cdc: 2,
            rndis: 2,
            ecm: 2,
            audio: 2,
            audio2: 2
        };

        const getEnabledClassInstances = (className) => {
            const checkbox = document.getElementById(`class-${className}`);
            if (!checkbox || !checkbox.checked) {
                return 0;
            }

            const countInput = document.getElementById(`class-num-${className}`);
            return Math.max(1, parseInt(countInput && countInput.value, 10) || 1);
        };

        const getClassInterfaceOffset = (targetClassName) => {
            let offset = 0;
            for (const className of getCurrentClassOrder()) {
                if (className === targetClassName) {
                    break;
                }

                offset += getEnabledClassInstances(className) * (classInterfacePerInstance[className] || 1);
            }

            return offset;
        };

        const classSegments = new Map();
        const beginClassSegment = () => ({ descriptorStart: descriptor.length, commentsStart: commentsList.length });
        const finalizeClassSegment = (className, segmentStart) => {
            const descriptorPart = descriptor.slice(segmentStart.descriptorStart);
            const commentsPart = commentsList.slice(segmentStart.commentsStart);

            descriptor.length = segmentStart.descriptorStart;
            commentsList.length = segmentStart.commentsStart;

            if (descriptorPart.length > 0) {
                classSegments.set(className, { descriptorPart, commentsPart });
            }
        };

        // Generate Interface and HID descriptors if HID is enabled
        const hidCheckbox = document.getElementById('class-hid');
        if (hidCheckbox && hidCheckbox.checked) {
            const hidNumberInput = document.getElementById('class-num-hid');
            const classNumber = parseInt(hidNumberInput.value) || 1;

            for (let i = 1; i <= classNumber; i++) {
                // Read form values using high-speed endpoint values
                const interfaceValues = readInterfaceDescriptorValues(i);
                interfaceValues.bInterfaceNumber = getClassInterfaceOffset('hid') + (i - 1);
                const hidDetailsValues = readHIDDetailsValues(i);
                const endpointValues = readEndpointInDescriptorValuesHighSpeed(i);
                const endpointOutValues = optionalEndpointState[i] && isProtocolNone(i) ? readEndpointOutDescriptorValuesHighSpeed(i) : null;

                if (endpointOutValues && Object.keys(endpointOutValues).length > 0) {
                    interfaceValues.bNumEndpoints = 2;
                }

                // Only generate if at least one set of values was provided
                if (Object.keys(interfaceValues).length > 0 || Object.keys(hidDetailsValues).length > 0 || Object.keys(endpointValues).length > 0) {
                    // Add Interface Descriptor
                    if (Object.keys(interfaceValues).length > 0) {
                        const interfaceDescriptor = buildInterfaceDescriptorFromValues(interfaceValues);
                        descriptor = descriptor.concat(interfaceDescriptor);

                        const interfaceComments = [
                            'bLength',
                            'bDescriptorType',
                            'bInterfaceNumber',
                            'bAlternateSetting',
                            'bNumEndpoints',
                            'bInterfaceClass',
                            'bInterfaceSubClass',
                            'bInterfaceProtocol',
                            'iInterface'
                        ];
                        interfaceComments.forEach((comment) => {
                            commentsList.push(`[IF-${i}] ${comment}`);
                        });
                    }

                    // Add HID Descriptor
                    if (Object.keys(hidDetailsValues).length > 0) {
                        const hidDescriptor = buildHIDDetailsDescriptorFromValues(hidDetailsValues);
                        descriptor = descriptor.concat(hidDescriptor);

                        const hidComments = [
                            'bLength',
                            'bDescriptorType',
                            'bcdHID LSB',
                            'bcdHID MSB',
                            'bCountryCode',
                            'bNumDescriptors',
                            'bReportDescriptorType',
                            'wReportDescriptorLength LSB',
                            'wReportDescriptorLength MSB'
                        ];
                        hidComments.forEach((comment) => {
                            commentsList.push(`[HID-${i}] ${comment}`);
                        });
                    }

                    // Add Endpoint IN Descriptor (high-speed values)
                    if (Object.keys(endpointValues).length > 0) {
                        const endpointDescriptor = buildEndpointInDescriptorFromValues(endpointValues);
                        descriptor = descriptor.concat(endpointDescriptor);

                        const endpointComments = [
                            'bLength',
                            'bDescriptorType',
                            'bEndpointAddress',
                            'bmAttributes',
                            'wMaxPacketSize LSB',
                            'wMaxPacketSize MSB',
                            'bInterval'
                        ];
                        endpointComments.forEach((comment) => {
                            commentsList.push(`[EP-${i}] ${comment}`);
                        });
                    }

                    // Add Endpoint OUT Descriptor (high-speed values) if present
                    if (endpointOutValues && Object.keys(endpointOutValues).length > 0) {
                        const endpointOutDescriptor = buildEndpointInDescriptorFromValues(endpointOutValues);
                        descriptor = descriptor.concat(endpointOutDescriptor);

                        const endpointOutComments = [
                            'bLength',
                            'bDescriptorType',
                            'bEndpointAddress',
                            'bmAttributes',
                            'wMaxPacketSize LSB',
                            'wMaxPacketSize MSB',
                            'bInterval'
                        ];
                        endpointOutComments.forEach((comment) => {
                            commentsList.push(`[EP-OUT-${i}] ${comment}`);
                        });
                    }
                }
            }
        }

        // Generate Mass Storage descriptors if MSC is enabled (High Speed)
        const mscCheckbox = document.getElementById('class-msc');
        if (mscCheckbox && mscCheckbox.checked) {
            const mscNumberInput = document.getElementById('class-num-msc');
            const classNumber = parseInt(mscNumberInput.value) || 1;

            for (let i = 1; i <= classNumber; i++) {
                // Read Mass Storage configuration values (High Speed)
                const mscConfigValues = readMassStorageConfigValues(i);
                const bulkInValuesHS = readMassStorageBulkInValuesHS(i);
                const bulkOutValuesHS = readMassStorageBulkOutValuesHS(i);

                // Build Interface Descriptor for Mass Storage
                const interfaceDescriptor = [
                    0x09,  // bLength
                    0x04,  // bDescriptorType (Interface)
                    i - 1 + (hidCheckbox && hidCheckbox.checked ? parseInt(document.getElementById('class-num-hid').value) || 1 : 0), // bInterfaceNumber
                    0x00,  // bAlternateSetting
                    0x02,  // bNumEndpoints (Bulk IN + Bulk OUT)
                    0x08,  // bInterfaceClass (Mass Storage)
                    parseHexValue(mscConfigValues.mscBInterfaceSubClass || '0x06', 1, 'mscBInterfaceSubClass'),  // bInterfaceSubClass
                    parseHexValue(mscConfigValues.mscBInterfaceProtocol || '0x50', 1, 'mscBInterfaceProtocol'),  // bInterfaceProtocol
                    0x00   // iInterface
                ];
                descriptor = descriptor.concat(interfaceDescriptor);

                // Add comments for Interface descriptor
                const interfaceComments = [
                    'bLength',
                    'bDescriptorType',
                    'bInterfaceNumber',
                    'bAlternateSetting',
                    'bNumEndpoints',
                    'bInterfaceClass',
                    'bInterfaceSubClass',
                    'bInterfaceProtocol',
                    'iInterface'
                ];
                interfaceComments.forEach((comment) => {
                    commentsList.push(`[MSC-IF-${i}] ${comment}`);
                });

                // Build Bulk IN Endpoint Descriptor (High Speed)
                const bulkInEndpoint = parseHexValue(bulkInValuesHS.mscBulkInEndpointHS || '0x81', 1, 'bulkInEndpoint');
                const bulkInMaxPacket = parseHexValue(bulkInValuesHS.mscBulkInMaxPacketSizeHS || '0x0200', 2, 'bulkInMaxPacket');
                const bulkInInterval = parseHexValue(bulkInValuesHS.mscBulkInIntervalHS || '0', 1, 'bulkInInterval');
                const bulkInDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkInEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkInMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkInMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkInInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkInDescriptor);

                const bulkInComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkInComments.forEach((comment) => {
                    commentsList.push(`[MSC-EP-IN-${i}] ${comment}`);
                });

                // Build Bulk OUT Endpoint Descriptor (High Speed)
                const bulkOutEndpoint = parseHexValue(bulkOutValuesHS.mscBulkOutEndpointHS || '0x01', 1, 'bulkOutEndpoint');
                const bulkOutMaxPacket = parseHexValue(bulkOutValuesHS.mscBulkOutMaxPacketSizeHS || '0x0200', 2, 'bulkOutMaxPacket');
                const bulkOutInterval = parseHexValue(bulkOutValuesHS.mscBulkOutIntervalHS || '0', 1, 'bulkOutInterval');
                const bulkOutDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkOutEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkOutMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkOutMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkOutInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkOutDescriptor);

                const bulkOutComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkOutComments.forEach((comment) => {
                    commentsList.push(`[MSC-EP-OUT-${i}] ${comment}`);
                });
            }
        }

        // Generate DFU descriptors if DFU is enabled (High Speed)
        const dfuCheckbox = document.getElementById('class-dfu');
        if (dfuCheckbox && dfuCheckbox.checked) {
            const dfuNumberInput = document.getElementById('class-num-dfu');
            const classNumber = parseInt(dfuNumberInput.value) || 1;

            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }

            for (let i = 1; i <= classNumber; i++) {
                const dfuConfigValues = readDFUConfigValues(i);

                const interfaceDescriptor = [
                    0x09,
                    0x04,
                    interfaceOffset + (i - 1),
                    0x00,
                    0x00,
                    0xFE,
                    parseHexValue(dfuConfigValues.dfuBInterfaceSubClass || '0x01', 1, 'dfuBInterfaceSubClass'),
                    parseHexValue(dfuConfigValues.dfuBInterfaceProtocol || '0x02', 1, 'dfuBInterfaceProtocol'),
                    parseNumberValue(dfuConfigValues.dfuIInterface || 0, { id: 'dfuIInterface', min: 0, max: 255, default: 0 })
                ];
                descriptor = descriptor.concat(interfaceDescriptor);

                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[DFU-IF-${i}] ${comment}`));

                const bcdDFUVersion = parseHexValue(dfuConfigValues.dfuBcdVersion || '0x0110', 2, 'dfuBcdVersion');
                const detachTimeout = parseNumberValue(dfuConfigValues.dfuDetachTimeout || 1000, { id: 'dfuDetachTimeout', min: 0, max: 65535, default: 1000 });
                const transferSize = parseNumberValue(dfuConfigValues.dfuTransferSize || 1024, { id: 'dfuTransferSize', min: 1, max: 65535, default: 1024 });
                const functionalDescriptor = [
                    0x09,
                    0x21,
                    parseHexValue(dfuConfigValues.dfuBmAttributes || '0x0B', 1, 'dfuBmAttributes'),
                    detachTimeout & 0xFF,
                    (detachTimeout >> 8) & 0xFF,
                    transferSize & 0xFF,
                    (transferSize >> 8) & 0xFF,
                    bcdDFUVersion & 0xFF,
                    (bcdDFUVersion >> 8) & 0xFF
                ];
                descriptor = descriptor.concat(functionalDescriptor);

                ['bLength', 'bDescriptorType', 'bmAttributes', 'wDetachTimeOut LSB', 'wDetachTimeOut MSB', 'wTransferSize LSB', 'wTransferSize MSB', 'bcdDFUVersion LSB', 'bcdDFUVersion MSB']
                    .forEach((comment) => commentsList.push(`[DFU-FUNC-${i}] ${comment}`));
            }
        }

        // Generate Printer descriptors if Printer is enabled (High Speed)
        const printerCheckbox = document.getElementById('class-printer');
        if (printerCheckbox && printerCheckbox.checked) {
            const printerNumberInput = document.getElementById('class-num-printer');
            const classNumber = parseInt(printerNumberInput.value) || 1;

            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }

            for (let i = 1; i <= classNumber; i++) {
                const printerConfigValues = readPrinterConfigValues(i);
                const bulkInValuesHS = readPrinterBulkInValuesHS(i);
                const bulkOutValuesHS = readPrinterBulkOutValuesHS(i);

                const interfaceDescriptor = [
                    0x09,
                    0x04,
                    interfaceOffset + (i - 1),
                    0x00,
                    0x02,
                    0x07,
                    parseHexValue(printerConfigValues.printerBInterfaceSubClass || '0x01', 1, 'printerBInterfaceSubClass'),
                    parseHexValue(printerConfigValues.printerBInterfaceProtocol || '0x02', 1, 'printerBInterfaceProtocol'),
                    parseNumberValue(printerConfigValues.printerIInterface || 0, { id: 'printerIInterface', min: 0, max: 255, default: 0 })
                ];
                descriptor = descriptor.concat(interfaceDescriptor);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[PRN-IF-${i}] ${comment}`));

                const bulkInEndpoint = parseHexValue(bulkInValuesHS.printerBulkInEndpointHS || '0x81', 1, 'printerBulkInEndpointHS');
                const bulkInMaxPacket = parseHexValue(bulkInValuesHS.printerBulkInMaxPacketSizeHS || '0x0200', 2, 'printerBulkInMaxPacketSizeHS');
                const bulkInInterval = parseHexValue(bulkInValuesHS.printerBulkInIntervalHS || '0', 1, 'printerBulkInIntervalHS');
                const bulkInDescriptor = [0x07, 0x05, bulkInEndpoint, 0x02, bulkInMaxPacket & 0xFF, (bulkInMaxPacket >> 8) & 0xFF, bulkInInterval];
                descriptor = descriptor.concat(bulkInDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
                    .forEach((comment) => commentsList.push(`[PRN-EP-IN-${i}] ${comment}`));

                const bulkOutEndpoint = parseHexValue(bulkOutValuesHS.printerBulkOutEndpointHS || '0x01', 1, 'printerBulkOutEndpointHS');
                const bulkOutMaxPacket = parseHexValue(bulkOutValuesHS.printerBulkOutMaxPacketSizeHS || '0x0200', 2, 'printerBulkOutMaxPacketSizeHS');
                const bulkOutInterval = parseHexValue(bulkOutValuesHS.printerBulkOutIntervalHS || '0', 1, 'printerBulkOutIntervalHS');
                const bulkOutDescriptor = [0x07, 0x05, bulkOutEndpoint, 0x02, bulkOutMaxPacket & 0xFF, (bulkOutMaxPacket >> 8) & 0xFF, bulkOutInterval];
                descriptor = descriptor.concat(bulkOutDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
                    .forEach((comment) => commentsList.push(`[PRN-EP-OUT-${i}] ${comment}`));
            }
        }

        // Generate Video descriptors if Video is enabled (High Speed)
        const videoCheckbox = document.getElementById('class-video');
        if (videoCheckbox && videoCheckbox.checked) {
            const videoNumberInput = document.getElementById('class-num-video');
            const classNumber = parseInt(videoNumberInput.value) || 1;

            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }

            for (let i = 1; i <= classNumber; i++) {
                const videoConfigValues = readVideoConfigValues(i);
                const bulkInValuesHS = readVideoBulkInValuesHS(i);
                const bulkOutValuesHS = readVideoBulkOutValuesHS(i);

                appendVideoDescriptorsFromValues(descriptor, commentsList, {
                    instanceIndex: i,
                    interfaceNumber: interfaceOffset + (i - 1),
                    configValues: videoConfigValues,
                    bulkInValues: bulkInValuesHS,
                    bulkOutValues: bulkOutValuesHS,
                    inEndpointKey: 'videoBulkInEndpointHS',
                    inPacketKey: 'videoBulkInMaxPacketSizeHS',
                    inIntervalKey: 'videoBulkInIntervalHS',
                    outEndpointKey: 'videoBulkOutEndpointHS',
                    outPacketKey: 'videoBulkOutMaxPacketSizeHS',
                    outIntervalKey: 'videoBulkOutIntervalHS',
                    defaultInEndpoint: '0x83',
                    defaultOutEndpoint: '0x03',
                    defaultPacket: '0x0200',
                    defaultInterval: '0'
                });
            }
        }

        // Generate MTP descriptors if MTP is enabled (High Speed)
        const mtpCheckbox = document.getElementById('class-mtp');
        if (mtpCheckbox && mtpCheckbox.checked) {
            const mtpNumberInput = document.getElementById('class-num-mtp');
            const classNumber = parseInt(mtpNumberInput.value) || 1;

            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }
            if (videoCheckbox && videoCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-video').value) || 1;
            }

            for (let i = 1; i <= classNumber; i++) {
                const mtpConfigValues = readMTPConfigValues(i);
                const bulkInValuesHS = readMTPBulkInValuesHS(i);
                const bulkOutValuesHS = readMTPBulkOutValuesHS(i);

                appendMTPDescriptorsFromValues(descriptor, commentsList, {
                    instanceIndex: i,
                    interfaceNumber: interfaceOffset + (i - 1),
                    configValues: mtpConfigValues,
                    bulkInValues: bulkInValuesHS,
                    bulkOutValues: bulkOutValuesHS,
                    inEndpointKey: 'mtpBulkInEndpointHS',
                    inPacketKey: 'mtpBulkInMaxPacketSizeHS',
                    inIntervalKey: 'mtpBulkInIntervalHS',
                    outEndpointKey: 'mtpBulkOutEndpointHS',
                    outPacketKey: 'mtpBulkOutMaxPacketSizeHS',
                    outIntervalKey: 'mtpBulkOutIntervalHS',
                    defaultInEndpoint: '0x84',
                    defaultOutEndpoint: '0x04',
                    defaultPacket: '0x0200',
                    defaultInterval: '0'
                });
            }
        }

        // Generate PTP descriptors if PTP is enabled (High Speed)
        const ptpCheckbox = document.getElementById('class-ptp');
        if (ptpCheckbox && ptpCheckbox.checked) {
            const ptpNumberInput = document.getElementById('class-num-ptp');
            const classNumber = parseInt(ptpNumberInput.value) || 1;

            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }
            if (videoCheckbox && videoCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-video').value) || 1;
            }
            if (mtpCheckbox && mtpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-mtp').value) || 1;
            }

            for (let i = 1; i <= classNumber; i++) {
                const ptpConfigValues = readPTPConfigValues(i);
                const bulkInValuesHS = readPTPBulkInValuesHS(i);
                const bulkOutValuesHS = readPTPBulkOutValuesHS(i);

                appendPTPDescriptorsFromValues(descriptor, commentsList, {
                    instanceIndex: i,
                    interfaceNumber: interfaceOffset + (i - 1),
                    configValues: ptpConfigValues,
                    bulkInValues: bulkInValuesHS,
                    bulkOutValues: bulkOutValuesHS,
                    inEndpointKey: 'ptpBulkInEndpointHS',
                    inPacketKey: 'ptpBulkInMaxPacketSizeHS',
                    inIntervalKey: 'ptpBulkInIntervalHS',
                    outEndpointKey: 'ptpBulkOutEndpointHS',
                    outPacketKey: 'ptpBulkOutMaxPacketSizeHS',
                    outIntervalKey: 'ptpBulkOutIntervalHS',
                    defaultInEndpoint: '0x85',
                    defaultOutEndpoint: '0x05',
                    defaultPacket: '0x0200',
                    defaultInterval: '0'
                });
            }
        }

        // Generate CDC ACM descriptors if CDC is enabled (High Speed)
        const cdcCheckbox = document.getElementById('class-cdc');
        if (cdcCheckbox && cdcCheckbox.checked) {
            const cdcNumberInput = document.getElementById('class-num-cdc');
            const classNumber = parseInt(cdcNumberInput.value) || 1;

            // Calculate interface offset based on previously added classes
            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }
            if (videoCheckbox && videoCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-video').value) || 1;
            }
            if (mtpCheckbox && mtpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-mtp').value) || 1;
            }
            if (ptpCheckbox && ptpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-ptp').value) || 1;
            }

            for (let i = 1; i <= classNumber; i++) {
                // Read CDC ACM configuration values
                const cdcConfigValues = readCDCConfigValues(i);
                const notifyValuesHS = readCDCNotifyValuesHS(i);
                const bulkInValuesHS = readCDCBulkInValuesHS(i);
                const bulkOutValuesHS = readCDCBulkOutValuesHS(i);

                const commInterfaceNum = interfaceOffset + (i - 1) * 2;
                const dataInterfaceNum = commInterfaceNum + 1;

                appendIadDescriptor(descriptor, commentsList, `CDC-IAD-${i}`, commInterfaceNum, 2,
                    0x02,
                    parseHexValue(cdcConfigValues.cdcBInterfaceSubClass || '0x02', 1, 'cdcBInterfaceSubClass'),
                    parseHexValue(cdcConfigValues.cdcBInterfaceProtocol || '0x01', 1, 'cdcBInterfaceProtocol'));

                // Build Communication Interface Descriptor
                const commInterfaceDescriptor = [
                    0x09,  // bLength
                    0x04,  // bDescriptorType (Interface)
                    commInterfaceNum,  // bInterfaceNumber
                    0x00,  // bAlternateSetting
                    0x01,  // bNumEndpoints (Notification endpoint only)
                    0x02,  // bInterfaceClass (Communications)
                    parseHexValue(cdcConfigValues.cdcBInterfaceSubClass || '0x02', 1, 'cdcBInterfaceSubClass'),  // bInterfaceSubClass
                    parseHexValue(cdcConfigValues.cdcBInterfaceProtocol || '0x01', 1, 'cdcBInterfaceProtocol'),  // bInterfaceProtocol
                    0x00   // iInterface
                ];
                descriptor = descriptor.concat(commInterfaceDescriptor);

                const commInterfaceComments = [
                    'bLength',
                    'bDescriptorType',
                    'bInterfaceNumber',
                    'bAlternateSetting',
                    'bNumEndpoints',
                    'bInterfaceClass',
                    'bInterfaceSubClass',
                    'bInterfaceProtocol',
                    'iInterface'
                ];
                commInterfaceComments.forEach((comment) => {
                    commentsList.push(`[CDC-COMM-IF-${i}] ${comment}`);
                });

                // CDC Header Functional Descriptor
                const cdcBcdCDC = parseHexValue(cdcConfigValues.cdcBcdCDC || '0x0110', 2, 'cdcBcdCDC');
                const headerDescriptor = [
                    0x05,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x00,  // bDescriptorSubtype (Header)
                    cdcBcdCDC & 0xFF,  // bcdCDC LSB
                    (cdcBcdCDC >> 8) & 0xFF   // bcdCDC MSB
                ];
                descriptor = descriptor.concat(headerDescriptor);

                const headerComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bcdCDC LSB',
                    'bcdCDC MSB'
                ];
                headerComments.forEach((comment) => {
                    commentsList.push(`[CDC-HEADER-${i}] ${comment}`);
                });

                // CDC Call Management Functional Descriptor
                const callMgmtDescriptor = [
                    0x05,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x01,  // bDescriptorSubtype (Call Management)
                    0x00,  // bmCapabilities (no call management)
                    dataInterfaceNum   // bDataInterface
                ];
                descriptor = descriptor.concat(callMgmtDescriptor);

                const callMgmtComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bmCapabilities',
                    'bDataInterface'
                ];
                callMgmtComments.forEach((comment) => {
                    commentsList.push(`[CDC-CALL-MGMT-${i}] ${comment}`);
                });

                // CDC ACM Functional Descriptor
                const acmDescriptor = [
                    0x04,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x02,  // bDescriptorSubtype (ACM)
                    parseHexValue(cdcConfigValues.cdcBmCapabilities || '0x02', 1, 'cdcBmCapabilities')  // bmCapabilities
                ];
                descriptor = descriptor.concat(acmDescriptor);

                const acmComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bmCapabilities'
                ];
                acmComments.forEach((comment) => {
                    commentsList.push(`[CDC-ACM-${i}] ${comment}`);
                });

                // CDC Union Functional Descriptor
                const unionDescriptor = [
                    0x05,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x06,  // bDescriptorSubtype (Union)
                    commInterfaceNum,  // bMasterInterface
                    dataInterfaceNum   // bSlaveInterface0
                ];
                descriptor = descriptor.concat(unionDescriptor);

                const unionComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bMasterInterface',
                    'bSlaveInterface0'
                ];
                unionComments.forEach((comment) => {
                    commentsList.push(`[CDC-UNION-${i}] ${comment}`);
                });

                // Notification Endpoint Descriptor (Interrupt IN) - High Speed
                const notifyEndpoint = parseHexValue(notifyValuesHS.cdcNotifyEndpointHS || '0x81', 1, 'notifyEndpoint');
                const notifyMaxPacket = parseHexValue(notifyValuesHS.cdcNotifyMaxPacketSizeHS || '0x0040', 2, 'notifyMaxPacket');
                const notifyInterval = parseHexValue(notifyValuesHS.cdcNotifyIntervalHS || '16', 1, 'notifyInterval');
                const notifyDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    notifyEndpoint,  // bEndpointAddress
                    0x03,  // bmAttributes (Interrupt)
                    notifyMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (notifyMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    notifyInterval   // bInterval
                ];
                descriptor = descriptor.concat(notifyDescriptor);

                const notifyComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                notifyComments.forEach((comment) => {
                    commentsList.push(`[CDC-EP-NOTIFY-${i}] ${comment}`);
                });

                // Build Data Interface Descriptor
                const dataInterfaceDescriptor = [
                    0x09,  // bLength
                    0x04,  // bDescriptorType (Interface)
                    dataInterfaceNum,  // bInterfaceNumber
                    0x00,  // bAlternateSetting
                    0x02,  // bNumEndpoints (Bulk IN + Bulk OUT)
                    0x0A,  // bInterfaceClass (CDC Data)
                    0x00,  // bInterfaceSubClass
                    0x00,  // bInterfaceProtocol
                    0x00   // iInterface
                ];
                descriptor = descriptor.concat(dataInterfaceDescriptor);

                const dataInterfaceComments = [
                    'bLength',
                    'bDescriptorType',
                    'bInterfaceNumber',
                    'bAlternateSetting',
                    'bNumEndpoints',
                    'bInterfaceClass',
                    'bInterfaceSubClass',
                    'bInterfaceProtocol',
                    'iInterface'
                ];
                dataInterfaceComments.forEach((comment) => {
                    commentsList.push(`[CDC-DATA-IF-${i}] ${comment}`);
                });

                // Data Bulk IN Endpoint Descriptor - High Speed
                const bulkInEndpoint = parseHexValue(bulkInValuesHS.cdcBulkInEndpointHS || '0x82', 1, 'bulkInEndpoint');
                const bulkInMaxPacket = parseHexValue(bulkInValuesHS.cdcBulkInMaxPacketSizeHS || '0x0200', 2, 'bulkInMaxPacket');
                const bulkInInterval = parseHexValue(bulkInValuesHS.cdcBulkInIntervalHS || '0', 1, 'bulkInInterval');
                const bulkInDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkInEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkInMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkInMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkInInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkInDescriptor);

                const bulkInComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkInComments.forEach((comment) => {
                    commentsList.push(`[CDC-EP-IN-${i}] ${comment}`);
                });

                // Data Bulk OUT Endpoint Descriptor - High Speed
                const bulkOutEndpoint = parseHexValue(bulkOutValuesHS.cdcBulkOutEndpointHS || '0x02', 1, 'bulkOutEndpoint');
                const bulkOutMaxPacket = parseHexValue(bulkOutValuesHS.cdcBulkOutMaxPacketSizeHS || '0x0200', 2, 'bulkOutMaxPacket');
                const bulkOutInterval = parseHexValue(bulkOutValuesHS.cdcBulkOutIntervalHS || '0', 1, 'bulkOutInterval');
                const bulkOutDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkOutEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkOutMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkOutMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkOutInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkOutDescriptor);

                const bulkOutComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkOutComments.forEach((comment) => {
                    commentsList.push(`[CDC-EP-OUT-${i}] ${comment}`);
                });
            }
        }

        // Generate CDC RNDIS descriptors if enabled (High Speed)
        const rndisCheckbox = document.getElementById('class-rndis');
        if (rndisCheckbox && rndisCheckbox.checked) {
            const rndisNumberInput = document.getElementById('class-num-rndis');
            const classNumber = parseInt(rndisNumberInput.value) || 1;

            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }
            if (videoCheckbox && videoCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-video').value) || 1;
            }
            if (mtpCheckbox && mtpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-mtp').value) || 1;
            }
            if (ptpCheckbox && ptpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-ptp').value) || 1;
            }
            if (cdcCheckbox && cdcCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-cdc').value) || 1) * 2;
            }

            for (let i = 1; i <= classNumber; i++) {
                const rndisConfig = readRNDISConfigValues(i);
                const notifyValuesHS = readRNDISNotifyValuesHS(i);
                const bulkInValuesHS = readRNDISBulkInValuesHS(i);
                const bulkOutValuesHS = readRNDISBulkOutValuesHS(i);

                const commInterfaceNum = interfaceOffset + (i - 1) * 2;
                const dataInterfaceNum = commInterfaceNum + 1;

                appendIadDescriptor(descriptor, commentsList, `RNDIS-IAD-${i}`, commInterfaceNum, 2, 0x02, 0x02, 0xFF);

                appendCDCNetworkingDescriptors(descriptor, commentsList, {
                    instanceIndex: i,
                    commInterfaceNum,
                    dataInterfaceNum,
                    subclass: 0x02,
                    protocol: 0xFF,
                    bcdCdc: parseHexValue(rndisConfig.rndisBcdCDC || '0x0110', 2, 'rndisBcdCDC'),
                    callMgmtCaps: parseHexValue(rndisConfig.rndisCallMgmtCapabilities || '0x00', 1, 'rndisCallMgmtCapabilities'),
                    acmCaps: parseHexValue(rndisConfig.rndisAcmCapabilities || '0x00', 1, 'rndisAcmCapabilities'),
                    notifyEndpoint: parseHexValue(notifyValuesHS.notifyEndpointHS || '0x81', 1, 'rndisNotifyEndpointHS'),
                    notifyMaxPacket: parseHexValue(notifyValuesHS.notifyMaxPacketSizeHS || '0x0040', 2, 'rndisNotifyMaxPacketSizeHS'),
                    notifyInterval: parseHexValue(notifyValuesHS.notifyIntervalHS || '16', 1, 'rndisNotifyIntervalHS'),
                    bulkInEndpoint: parseHexValue(bulkInValuesHS.bulkInEndpointHS || '0x82', 1, 'rndisBulkInEndpointHS'),
                    bulkInMaxPacket: parseHexValue(bulkInValuesHS.bulkInMaxPacketSizeHS || '0x0200', 2, 'rndisBulkInMaxPacketSizeHS'),
                    bulkInInterval: parseHexValue(bulkInValuesHS.bulkInIntervalHS || '0', 1, 'rndisBulkInIntervalHS'),
                    bulkOutEndpoint: parseHexValue(bulkOutValuesHS.bulkOutEndpointHS || '0x02', 1, 'rndisBulkOutEndpointHS'),
                    bulkOutMaxPacket: parseHexValue(bulkOutValuesHS.bulkOutMaxPacketSizeHS || '0x0200', 2, 'rndisBulkOutMaxPacketSizeHS'),
                    bulkOutInterval: parseHexValue(bulkOutValuesHS.bulkOutIntervalHS || '0', 1, 'rndisBulkOutIntervalHS'),
                    tagPrefix: 'RNDIS',
                    includeCallMgmt: true,
                    includeAcm: true,
                    includeEthernet: false
                });
            }
        }

        // Generate CDC ECM descriptors if enabled (High Speed)
        const ecmCheckbox = document.getElementById('class-ecm');
        if (ecmCheckbox && ecmCheckbox.checked) {
            const ecmNumberInput = document.getElementById('class-num-ecm');
            const classNumber = parseInt(ecmNumberInput.value) || 1;

            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }
            if (videoCheckbox && videoCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-video').value) || 1;
            }
            if (mtpCheckbox && mtpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-mtp').value) || 1;
            }
            if (ptpCheckbox && ptpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-ptp').value) || 1;
            }
            if (cdcCheckbox && cdcCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-cdc').value) || 1) * 2;
            }
            if (rndisCheckbox && rndisCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-rndis').value) || 1) * 2;
            }

            for (let i = 1; i <= classNumber; i++) {
                const ecmConfig = readECMConfigValues(i);
                const notifyValuesHS = readECMNotifyValuesHS(i);
                const bulkInValuesHS = readECMBulkInValuesHS(i);
                const bulkOutValuesHS = readECMBulkOutValuesHS(i);

                const commInterfaceNum = interfaceOffset + (i - 1) * 2;
                const dataInterfaceNum = commInterfaceNum + 1;

                appendIadDescriptor(descriptor, commentsList, `ECM-IAD-${i}`, commInterfaceNum, 2, 0x02, 0x06, 0x00);

                appendCDCNetworkingDescriptors(descriptor, commentsList, {
                    instanceIndex: i,
                    commInterfaceNum,
                    dataInterfaceNum,
                    subclass: 0x06,
                    protocol: 0x00,
                    bcdCdc: parseHexValue(ecmConfig.ecmBcdCDC || '0x0120', 2, 'ecmBcdCDC'),
                    callMgmtCaps: 0x00,
                    acmCaps: 0x00,
                    notifyEndpoint: parseHexValue(notifyValuesHS.notifyEndpointHS || '0x81', 1, 'ecmNotifyEndpointHS'),
                    notifyMaxPacket: parseHexValue(notifyValuesHS.notifyMaxPacketSizeHS || '0x0040', 2, 'ecmNotifyMaxPacketSizeHS'),
                    notifyInterval: parseHexValue(notifyValuesHS.notifyIntervalHS || '16', 1, 'ecmNotifyIntervalHS'),
                    bulkInEndpoint: parseHexValue(bulkInValuesHS.bulkInEndpointHS || '0x82', 1, 'ecmBulkInEndpointHS'),
                    bulkInMaxPacket: parseHexValue(bulkInValuesHS.bulkInMaxPacketSizeHS || '0x0200', 2, 'ecmBulkInMaxPacketSizeHS'),
                    bulkInInterval: parseHexValue(bulkInValuesHS.bulkInIntervalHS || '0', 1, 'ecmBulkInIntervalHS'),
                    bulkOutEndpoint: parseHexValue(bulkOutValuesHS.bulkOutEndpointHS || '0x02', 1, 'ecmBulkOutEndpointHS'),
                    bulkOutMaxPacket: parseHexValue(bulkOutValuesHS.bulkOutMaxPacketSizeHS || '0x0200', 2, 'ecmBulkOutMaxPacketSizeHS'),
                    bulkOutInterval: parseHexValue(bulkOutValuesHS.bulkOutIntervalHS || '0', 1, 'ecmBulkOutIntervalHS'),
                    tagPrefix: 'ECM',
                    includeCallMgmt: false,
                    includeAcm: false,
                    includeEthernet: true,
                    ethernet: {
                        macStringIndex: ecmConfig.ecmMacStringIndex,
                        maxSegmentSize: ecmConfig.ecmMaxSegmentSize,
                        numMcFilters: ecmConfig.ecmNumMcFilters,
                        numPowerFilters: ecmConfig.ecmNumPowerFilters
                    }
                });
            }
        }

        // Generate Audio 1.0 descriptors if Audio is enabled (High Speed)
        const audioCheckbox = document.getElementById('class-audio');
        if (audioCheckbox && audioCheckbox.checked) {
            const audioNumberInput = document.getElementById('class-num-audio');
            const classNumber = parseInt(audioNumberInput.value) || 1;

            // Audio comes after HID + MSC + CDC + RNDIS + ECM (each CDC variant uses 2 interfaces).
            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }
            if (videoCheckbox && videoCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-video').value) || 1;
            }
            if (mtpCheckbox && mtpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-mtp').value) || 1;
            }
            if (ptpCheckbox && ptpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-ptp').value) || 1;
            }
            if (cdcCheckbox && cdcCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-cdc').value) || 1) * 2;
            }
            if (rndisCheckbox && rndisCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-rndis').value) || 1) * 2;
            }
            if (ecmCheckbox && ecmCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-ecm').value) || 1) * 2;
            }

            for (let i = 1; i <= classNumber; i++) {
                const audioConfig = readAudioConfigValues(i);
                const audioEpHS = readAudioEndpointValuesHS(i);

                const acInterfaceNum = interfaceOffset + (i - 1) * 2;
                const asInterfaceNum = acInterfaceNum + 1;
                const acSubClass = parseHexValue(audioConfig.audioControlSubClass || '0x01', 1, 'audioControlSubClass');
                const asSubClass = parseHexValue(audioConfig.audioStreamingSubClass || '0x02', 1, 'audioStreamingSubClass');

                appendIadDescriptor(descriptor, commentsList, `AUDIO-IAD-${i}`, acInterfaceNum, 2, 0x01, acSubClass, 0x00);
                const terminalLink = parseNumberValue(audioConfig.audioTerminalLink || 1, { id: 'audioTerminalLink', min: 1, max: 253, default: 1 });
                const inputTerminalType  = parseHexValue(audioConfig.audioInputTerminalType  || '0x0101', 2, 'audioInputTerminalType');
                const outputTerminalType = parseHexValue(audioConfig.audioOutputTerminalType || '0x0301', 2, 'audioOutputTerminalType');
                const formatTag = parseHexValue(audioConfig.audioFormatTag || '0x0001', 2, 'audioFormatTag');
                const formatType = parseHexValue(audioConfig.audioFormatType, 1, 'audioFormatType');
                const numChannels = parseNumberValue(audioConfig.audioNumChannels || 2, { id: 'audioNumChannels', min: 1, max: 8, default: 2 });
                const bitResolution = parseNumberValue(audioConfig.audioBitResolution || 16, { id: 'audioBitResolution', min: 1, max: 32, default: 16 });
                const subframeSize = Math.ceil(bitResolution / 8);
                const maxBitRate = parseNumberValue(audioConfig.audioMaxBitRate || 128, { id: 'audioMaxBitRate', min: 1, max: 65535, default: 128 });
                const samplesPerFrame = parseNumberValue(audioConfig.audioSamplesPerFrame || 1, { id: 'audioSamplesPerFrame', min: 1, max: 65535, default: 1 });
                const sampleRates = Array.isArray(audioConfig.audioSampleRate) && audioConfig.audioSampleRate.length > 0
                    ? audioConfig.audioSampleRate : [48000];

                // AC topology IDs: IT=terminalLink, FU=terminalLink+2, OT=terminalLink+1
                const fuId = terminalLink + 2;
                const otId = terminalLink + 1;
                const channelConfig = getChannelConfigFromCount(numChannels);

                // Pre-compute bControlSize: 2 if Bass Boost (bit8) or Loudness (bit9) selected
                const _masterCtrlRaw = Array.isArray(audioConfig.audioFuMasterControls) ? audioConfig.audioFuMasterControls : [0, 1];
                const _channelCtrlRaw = Array.isArray(audioConfig.audioFuChannelControls) ? audioConfig.audioFuChannelControls : [];
                const bControlSize = (_masterCtrlRaw.some((b) => b >= 8) || _channelCtrlRaw.some((b) => b >= 8)) ? 2 : 1;
                // FU bLength = 7 + (bNrChannels + 1) * bControlSize
                const fuLength = 7 + (numChannels + 1) * bControlSize;
                // wTotalLength = CS header(9) + IT(12) + FU(fuLength) + OT(9)
                const acTotalLength = 9 + 12 + fuLength + 9;
                const audioAcIInterface = parseNumberValue(audioConfig.audioAcIInterface || 0, { id: 'audioAcIInterface', min: 0, max: 255, default: 0 });
                const acEpEnabled   = !!audioConfig.audioAcEpEnabled;
                const acEpAddress   = acEpEnabled ? parseHexValue(audioConfig.audioAcEpAddressHS   || '0x83', 1, 'audioAcEpAddressHS')   : 0;
                const acEpMaxPacket = acEpEnabled ? parseNumberValue(audioConfig.audioAcEpMaxPacketHS || 2,   { id: 'audioAcEpMaxPacketHS', min: 1, max: 1024, default: 2 })   : 0;
                const acEpInterval  = acEpEnabled ? parseNumberValue(audioConfig.audioAcEpIntervalHS  || 6,   { id: 'audioAcEpIntervalHS',  min: 1, max: 16,  default: 6 })  : 0;

                const acInterfaceDescriptor = [
                    0x09, 0x04, acInterfaceNum, 0x00, acEpEnabled ? 0x01 : 0x00, 0x01, acSubClass, 0x00, audioAcIInterface
                ];
                descriptor = descriptor.concat(acInterfaceDescriptor);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-IF-${i}] ${comment}`));

                const acClassSpecificDescriptor = [
                    0x09, 0x24, 0x01, 0x00, 0x01,
                    acTotalLength & 0xFF, (acTotalLength >> 8) & 0xFF,
                    0x01, asInterfaceNum
                ];
                descriptor = descriptor.concat(acClassSpecificDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bcdADC LSB', 'bcdADC MSB', 'wTotalLength LSB', 'wTotalLength MSB', 'bInCollection', 'baInterfaceNr(1)']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-CS-${i}] ${comment}`));

                // Input Terminal Descriptor (UAC1 Table 4-3, 12 bytes)
                const acInputTerminalDesc = [
                    0x0C, 0x24, 0x02,
                    terminalLink,
                    inputTerminalType & 0xFF, (inputTerminalType >> 8) & 0xFF,
                    0x00,
                    numChannels,
                    channelConfig & 0xFF, (channelConfig >> 8) & 0xFF,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(acInputTerminalDesc);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bNrChannels', 'wChannelConfig LSB', 'wChannelConfig MSB', 'iChannelNames', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-IT-${i}] ${comment}`));

                // Feature Unit Descriptor (UAC1 Table 4-6)
                const masterControlBits = _masterCtrlRaw.filter((b) => b >= 0);
                const channelControlBits = _channelCtrlRaw.filter((b) => b >= 0);
                const masterLsb = masterControlBits.filter((b) => b < 8).reduce((acc, b) => acc | (1 << b), 0);
                const masterMsb = masterControlBits.filter((b) => b >= 8).reduce((acc, b) => acc | (1 << (b - 8)), 0);
                const channelLsb = channelControlBits.filter((b) => b < 8).reduce((acc, b) => acc | (1 << b), 0);
                const channelMsb = channelControlBits.filter((b) => b >= 8).reduce((acc, b) => acc | (1 << (b - 8)), 0);
                const fuBmaControls = [];
                if (bControlSize === 1) {
                    fuBmaControls.push(masterLsb);
                    for (let ch = 0; ch < numChannels; ch++) fuBmaControls.push(channelLsb);
                } else {
                    fuBmaControls.push(masterLsb, masterMsb);
                    for (let ch = 0; ch < numChannels; ch++) fuBmaControls.push(channelLsb, channelMsb);
                }
                const acFeatureUnitDesc = [
                    fuLength, 0x24, 0x06,
                    fuId,
                    terminalLink,
                    bControlSize,
                    ...fuBmaControls,
                    0x00
                ];
                descriptor = descriptor.concat(acFeatureUnitDesc);
                const fuComments = ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bUnitID', 'bSourceID', 'bControlSize'];
                for (let ch = 0; ch <= numChannels; ch++) {
                    if (bControlSize === 1) { fuComments.push(`bmaControls(${ch})`); }
                    else { fuComments.push(`bmaControls(${ch}) LSB`, `bmaControls(${ch}) MSB`); }
                }
                fuComments.push('iFeature');
                fuComments.forEach((comment) => commentsList.push(`[AUDIO-AC-FU-${i}] ${comment}`));

                // Output Terminal Descriptor (UAC1 Table 4-4, 9 bytes)
                const acOutputTerminalDesc = [
                    0x09, 0x24, 0x03,
                    otId,
                    outputTerminalType & 0xFF, (outputTerminalType >> 8) & 0xFF,
                    0x00,
                    fuId,
                    0x00
                ];
                descriptor = descriptor.concat(acOutputTerminalDesc);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bSourceID', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-OT-${i}] ${comment}`));

                if (acEpEnabled) {
                    const acEpDescriptor = [
                        0x09, 0x05, acEpAddress, 0x03,
                        acEpMaxPacket & 0xFF, (acEpMaxPacket >> 8) & 0xFF,
                        acEpInterval, 0x00, 0x00
                    ];
                    descriptor = descriptor.concat(acEpDescriptor);
                    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval', 'bRefresh', 'bSynchAddress']
                        .forEach((comment) => commentsList.push(`[AUDIO-AC-EP-${i}] ${comment}`));
                }

                const audioAsIInterface = parseNumberValue(audioConfig.audioAsIInterface || 0, { id: 'audioAsIInterface', min: 0, max: 255, default: 0 });

                const asInterfaceAlt0 = [
                    0x09, 0x04, asInterfaceNum, 0x00, 0x00, 0x01, asSubClass, 0x00, audioAsIInterface
                ];
                descriptor = descriptor.concat(asInterfaceAlt0);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO-AS-IF0-${i}] ${comment}`));

                const audioSyncTypeHS = parseInt(audioEpHS.audioSyncType, 16) || 0x01;
                const hasFeedbackEpHS = audioSyncTypeHS === 0x05;
                const asInterfaceAlt1 = [
                    0x09, 0x04, asInterfaceNum, 0x01, hasFeedbackEpHS ? 0x02 : 0x01, 0x01, asSubClass, 0x00, audioAsIInterface
                ];
                descriptor = descriptor.concat(asInterfaceAlt1);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO-AS-IF1-${i}] ${comment}`));

                const asGeneralDescriptor = [
                    0x07, 0x24, 0x01, terminalLink, 0x01, formatTag & 0xFF, (formatTag >> 8) & 0xFF
                ];
                descriptor = descriptor.concat(asGeneralDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalLink', 'bDelay', 'wFormatTag LSB', 'wFormatTag MSB']
                    .forEach((comment) => commentsList.push(`[AUDIO-AS-CS-GEN-${i}] ${comment}`));

                const samFreqBytesHS = [];
                const samFreqCommentsHS = [];
                sampleRates.forEach((rate, idx) => {
                    samFreqBytesHS.push(rate & 0xFF, (rate >> 8) & 0xFF, (rate >> 16) & 0xFF);
                    samFreqCommentsHS.push(`tSamFreq[${idx}] byte0`, `tSamFreq[${idx}] byte1`, `tSamFreq[${idx}] byte2`);
                });

                let formatTypeDescriptor;
                let formatTypeComments;
                if (formatType === 0x02) {
                    // Type II: wMaxBitRate + wSamplesPerFrame instead of channel/resolution fields
                    formatTypeDescriptor = [
                        0x09 + sampleRates.length * 3,
                        0x24, 0x02, 0x02,
                        maxBitRate & 0xFF, (maxBitRate >> 8) & 0xFF,
                        samplesPerFrame & 0xFF, (samplesPerFrame >> 8) & 0xFF,
                        sampleRates.length,
                        ...samFreqBytesHS
                    ];
                    formatTypeComments = ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bFormatType', 'wMaxBitRate LSB', 'wMaxBitRate MSB', 'wSamplesPerFrame LSB', 'wSamplesPerFrame MSB', 'bSamFreqType', ...samFreqCommentsHS];
                } else {
                    // Type I or Type III: bNrChannels + bSubframeSize + bBitResolution
                    formatTypeDescriptor = [
                        0x08 + sampleRates.length * 3,
                        0x24, 0x02, formatType,
                        numChannels, subframeSize, bitResolution,
                        sampleRates.length,
                        ...samFreqBytesHS
                    ];
                    formatTypeComments = ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bFormatType', 'bNrChannels', 'bSubframeSize', 'bBitResolution', 'bSamFreqType', ...samFreqCommentsHS];
                }
                descriptor = descriptor.concat(formatTypeDescriptor);
                formatTypeComments.forEach((comment) => commentsList.push(`[AUDIO-AS-CS-FMT-${i}] ${comment}`));

                const epAddress = parseHexValue(audioEpHS.audioEndpointAddressHS || '0x81', 1, 'audioEndpointAddressHS');
                const epMaxPacket = parseInt(audioEpHS.audioMaxPacketSizeHS, 10) || 192;
                const epInterval = parseHexValue(audioEpHS.audioIntervalHS || '1', 1, 'audioIntervalHS');
                const fbEpAddressHS   = hasFeedbackEpHS ? parseHexValue(audioEpHS.audioFbEpAddressHS || '0x01', 1, 'audioFbEpAddressHS') : 0;
                const fbEpMaxPacketHS = hasFeedbackEpHS ? (parseInt(audioEpHS.audioFbEpMaxPacketHS, 10) || 4) : 0;
                const fbEpIntervalHS  = hasFeedbackEpHS ? (parseInt(audioEpHS.audioFbEpIntervalHS,  10) || 4) : 0;
                const isocEndpointDescriptor = [
                    0x09, 0x05, epAddress, audioSyncTypeHS & 0xFF,
                    epMaxPacket & 0xFF, (epMaxPacket >> 8) & 0xFF,
                    epInterval, 0x00, hasFeedbackEpHS ? fbEpAddressHS : 0x00
                ];
                descriptor = descriptor.concat(isocEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval', 'bRefresh', 'bSynchAddress']
                    .forEach((comment) => commentsList.push(`[AUDIO-EP-${i}] ${comment}`));

                const csIsoEndpointDescriptor = [
                    0x07, 0x25, 0x01,
                    sampleRates.length > 1 ? 0x01 : 0x00,
                    0x00, 0x00, 0x00
                ];
                descriptor = descriptor.concat(csIsoEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bmAttributes', 'bLockDelayUnits', 'wLockDelay LSB', 'wLockDelay MSB']
                    .forEach((comment) => commentsList.push(`[AUDIO-CS-EP-${i}] ${comment}`));

                if (hasFeedbackEpHS) {
                    const fbEpDescriptorHS = [
                        0x09, 0x05, fbEpAddressHS, 0x11,
                        fbEpMaxPacketHS & 0xFF, (fbEpMaxPacketHS >> 8) & 0xFF,
                        fbEpIntervalHS, 0x00, 0x00
                    ];
                    descriptor = descriptor.concat(fbEpDescriptorHS);
                    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval', 'bRefresh', 'bSynchAddress']
                        .forEach((comment) => commentsList.push(`[AUDIO-FB-EP-${i}] ${comment}`));
                }
            }
        }

        // Generate Audio 2.0 descriptors if Audio 2.0 is enabled (High Speed)
        const audio2Checkbox = document.getElementById('class-audio2');
        if (audio2Checkbox && audio2Checkbox.checked) {
            const audio2NumberInput = document.getElementById('class-num-audio2');
            const classNumber = parseInt(audio2NumberInput.value) || 1;

            // Audio 2.0 comes after HID + MSC + CDC + RNDIS + ECM + Audio 1.0.
            let interfaceOffset = 0;
            if (hidCheckbox && hidCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-hid').value) || 1;
            }
            if (mscCheckbox && mscCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-msc').value) || 1;
            }
            if (dfuCheckbox && dfuCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-dfu').value) || 1;
            }
            if (printerCheckbox && printerCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-printer').value) || 1;
            }
            if (videoCheckbox && videoCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-video').value) || 1;
            }
            if (mtpCheckbox && mtpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-mtp').value) || 1;
            }
            if (ptpCheckbox && ptpCheckbox.checked) {
                interfaceOffset += parseInt(document.getElementById('class-num-ptp').value) || 1;
            }
            if (cdcCheckbox && cdcCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-cdc').value) || 1) * 2;
            }
            if (rndisCheckbox && rndisCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-rndis').value) || 1) * 2;
            }
            if (ecmCheckbox && ecmCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-ecm').value) || 1) * 2;
            }
            if (audioCheckbox && audioCheckbox.checked) {
                interfaceOffset += (parseInt(document.getElementById('class-num-audio').value) || 1) * 2;
            }

            for (let i = 1; i <= classNumber; i++) {
                const audio2Config = readAudio2ConfigValues(i);
                const audio2EpHS = readAudio2EndpointValuesHS(i);

                const acInterfaceNum = interfaceOffset + (i - 1) * 2;
                const asInterfaceNum = acInterfaceNum + 1;
                const acSubClass = parseHexValue(audio2Config.audio20ControlSubClass || '0x01', 1, 'audio20ControlSubClass');
                const asSubClass = parseHexValue(audio2Config.audio20StreamingSubClass || '0x02', 1, 'audio20StreamingSubClass');

                appendIadDescriptor(descriptor, commentsList, `AUDIO2-IAD-${i}`, acInterfaceNum, 2, 0x01, acSubClass, 0x20);
                const clockSourceId = parseNumberValue(audio2Config.audio20ClockSourceId || 16, { id: 'audio20ClockSourceId', min: 1, max: 255, default: 16 });
                const terminalLink = parseNumberValue(audio2Config.audio20TerminalLink || 1, { id: 'audio20TerminalLink', min: 1, max: 255, default: 1 });
                const numChannels = parseNumberValue(audio2Config.audio20NumChannels || 2, { id: 'audio20NumChannels', min: 1, max: 8, default: 2 });
                const subslotSize = parseHexValue(audio2Config.audio20SubslotSize || '0x02', 1, 'audio20SubslotSize');
                const bitResolution = parseNumberValue(audio2Config.audio20BitResolution || 16, { id: 'audio20BitResolution', min: 1, max: 32, default: 16 });

                // Standard AC Interface Descriptor (UAC2 protocol 0x20)
                const acInterfaceDescriptor = [
                    0x09,
                    0x04,
                    acInterfaceNum,
                    0x00,
                    0x00,
                    0x01,
                    acSubClass,
                    0x20,
                    0x00
                ];
                descriptor = descriptor.concat(acInterfaceDescriptor);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AC-IF-${i}] ${comment}`));

                // Class-specific AC Header Descriptor (UAC2)
                const acHeaderDescriptor = [
                    0x09,
                    0x24,
                    0x01,
                    0x00,
                    0x02,
                    0x25,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(acHeaderDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bcdADC LSB', 'bcdADC MSB', 'wTotalLength LSB', 'wTotalLength MSB', 'bmControls LSB', 'bmControls MSB']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AC-HDR-${i}] ${comment}`));

                const clockSourceDescriptor = [
                    0x08,
                    0x24,
                    0x0A,
                    clockSourceId,
                    0x03,
                    0x07,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(clockSourceDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bClockID', 'bmAttributes', 'bmControls', 'bAssocTerminal', 'iClockSource']
                    .forEach((comment) => commentsList.push(`[AUDIO2-CLOCK-${i}] ${comment}`));

                const inputTerminalDescriptor = [
                    0x11,
                    0x24,
                    0x02,
                    terminalLink,
                    0x01,
                    0x01,
                    0x00,
                    clockSourceId,
                    numChannels,
                    0x03,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(inputTerminalDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bCSourceID', 'bNrChannels', 'bmChannelConfig B0', 'bmChannelConfig B1', 'bmChannelConfig B2', 'bmChannelConfig B3', 'iChannelNames', 'bmControls LSB', 'bmControls MSB', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO2-INTERM-${i}] ${comment}`));

                const outputTerminalDescriptor = [
                    0x0C,
                    0x24,
                    0x03,
                    terminalLink + 1,
                    0x01,
                    0x03,
                    0x00,
                    terminalLink,
                    clockSourceId,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(outputTerminalDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bSourceID', 'bCSourceID', 'bmControls LSB', 'bmControls MSB', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO2-OUTTERM-${i}] ${comment}`));

                const asInterfaceAlt0 = [
                    0x09,
                    0x04,
                    asInterfaceNum,
                    0x00,
                    0x00,
                    0x01,
                    asSubClass,
                    0x20,
                    0x00
                ];
                descriptor = descriptor.concat(asInterfaceAlt0);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-IF0-${i}] ${comment}`));

                const asInterfaceAlt1 = [
                    0x09,
                    0x04,
                    asInterfaceNum,
                    0x01,
                    0x01,
                    0x01,
                    asSubClass,
                    0x20,
                    0x00
                ];
                descriptor = descriptor.concat(asInterfaceAlt1);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-IF1-${i}] ${comment}`));

                const asGeneralDescriptor = [
                    0x10,
                    0x24,
                    0x01,
                    terminalLink,
                    0x00,
                    0x00,
                    0x01,
                    0x00,
                    0x00,
                    0x00,
                    numChannels,
                    0x03,
                    0x00,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(asGeneralDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalLink', 'bmControls LSB', 'bmControls MSB', 'bFormatType', 'bmFormats B0', 'bmFormats B1', 'bmFormats B2', 'bNrChannels', 'bmChannelConfig B0', 'bmChannelConfig B1', 'bmChannelConfig B2', 'bmChannelConfig B3', 'iChannelNames']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-GEN-${i}] ${comment}`));

                const formatTypeDescriptor = [
                    0x06,
                    0x24,
                    0x02,
                    subslotSize,
                    bitResolution,
                    0x00
                ];
                descriptor = descriptor.concat(formatTypeDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bSubslotSize', 'bBitResolution', 'bReserved']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-FMT-${i}] ${comment}`));

                const epAddress = parseHexValue(audio2EpHS.audio2EndpointAddressHS || '0x81', 1, 'audio2EndpointAddressHS');
                const epMaxPacket = parseHexValue(audio2EpHS.audio2MaxPacketSizeHS || '0x0200', 2, 'audio2MaxPacketSizeHS');
                const epInterval = parseHexValue(audio2EpHS.audio2IntervalHS || '1', 1, 'audio2IntervalHS');
                const isocEndpointDescriptor = [
                    0x07,
                    0x05,
                    epAddress,
                    0x05,
                    epMaxPacket & 0xFF,
                    (epMaxPacket >> 8) & 0xFF,
                    epInterval
                ];
                descriptor = descriptor.concat(isocEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
                    .forEach((comment) => commentsList.push(`[AUDIO2-EP-${i}] ${comment}`));

                const csIsoEndpointDescriptor = [
                    0x08,
                    0x25,
                    0x01,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(csIsoEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bmAttributes', 'bmControls LSB', 'bmControls MSB', 'bLockDelayUnits', 'wLockDelay']
                    .forEach((comment) => commentsList.push(`[AUDIO2-CS-EP-${i}] ${comment}`));
            }
        }

        // Configuration descriptor starts right after device (18) + qualifier (10).
        updateConfigurationTotalLength(descriptor, 28);

        return { descriptor, comments: commentsList };
    } catch (error) {
        console.error('Error generating high-speed descriptor: ' + error.message);
        return null;
    }
}

function getClassInstanceCount(countInputId) {
    const countInput = document.getElementById(countInputId);
    return Math.max(1, parseInt(countInput && countInput.value, 10) || 1);
}

function collectConfiguredParameters(formValues, configValues) {
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    const languageConfig = [
        { checkboxId: 'lang-en-us', code: '0x0409', name: 'English (US)' },
        { checkboxId: 'lang-en-uk', code: '0x0809', name: 'English (UK)' },
        { checkboxId: 'lang-fr', code: '0x040C', name: 'French' },
        { checkboxId: 'lang-de', code: '0x0407', name: 'German' },
        { checkboxId: 'lang-es', code: '0x0C0A', name: 'Spanish' },
        { checkboxId: 'lang-it', code: '0x0410', name: 'Italian' },
        { checkboxId: 'lang-ja', code: '0x0411', name: 'Japanese' },
        { checkboxId: 'lang-zh-cn', code: '0x0804', name: 'Chinese (Simplified)' }
    ];

    const enabledLanguages = languageConfig
        .filter((lang) => {
            const checkbox = document.getElementById(lang.checkboxId);
            return !!(checkbox && checkbox.checked);
        })
        .map((lang) => ({ code: lang.code, name: lang.name }));

    const jsonOutput = {
        device: {
            idVendor: toHex(formValues.idVendor, 2),
            idProduct: toHex(formValues.idProduct, 2),
            manufacturer: formValues.manufacturer || '',
            product: formValues.product || '',
            serialNumber: formValues.serialNumber || '',
            bcdDevice: toHex(formValues.bcdDevice, 2),
            bcdUSB: toHex(formValues.bcdUSB, 2),
            bDeviceClass: toHex(formValues.bDeviceClass, 1),
            bDeviceSubClass: toHex(formValues.bDeviceSubClass, 1),
            bDeviceProtocol: toHex(formValues.bDeviceProtocol, 1),
            bMaxPacketSize0: formValues.bMaxPacketSize0,
            iManufacturer: formValues.iManufacturer,
            iProduct: formValues.iProduct,
            iSerialNumber: formValues.iSerialNumber,
            bNumConfigurations: formValues.bNumConfigurations
        },
        configuration: {
            wTotalLength: configValues.wTotalLength,
            bNumInterfaces: configValues.bNumInterfaces,
            bConfigurationValue: '0x01',
            iConfiguration: configValues.iConfiguration,
            configurationString: configValues.configurationString || '',
            bmAttributes: toHex(configValues.bmAttributes, 1),
            bMaxPower: configValues.bMaxPower,
            highSpeed: isHighSpeedEnabled
                ? {
                    iConfigurationHS: configValues.iConfigurationHS,
                    configurationStringHS: configValues.configurationStringHS || ''
                }
                : null
        },
        languageIds: enabledLanguages,
        speeds: {
            fullSpeed: true,
            highSpeed: isHighSpeedEnabled
        },
        classOrder: getEnabledClassOrder(),
        customStrings: getCustomStringDescriptorValues().map((csd) => ({
            index: csd.index,
            text: csd.text,
            languageId: csd.languageId
        })),
        classes: {}
    };

    const classDefinitions = [
        {
            key: 'hid',
            checkboxId: 'class-hid',
            countId: 'class-num-hid',
            buildInstance: (i) => ({
                interfaceDescriptor: readInterfaceDescriptorValues(i),
                hidDescriptor: readHIDDetailsValues(i),
                endpointInFS: readEndpointInDescriptorValues(i),
                endpointOutFS: optionalEndpointState[i] && isProtocolNone(i) ? readEndpointOutDescriptorValues(i) : null,
                endpointInHS: isHighSpeedEnabled ? readEndpointInDescriptorValuesHighSpeed(i) : null,
                endpointOutHS: isHighSpeedEnabled && optionalEndpointState[i] && isProtocolNone(i) ? readEndpointOutDescriptorValuesHighSpeed(i) : null,
                optionalOutEndpointEnabled: !!optionalEndpointState[i]
            })
        },
        {
            key: 'massStorage',
            checkboxId: 'class-msc',
            countId: 'class-num-msc',
            buildInstance: (i) => ({
                interfaceConfig: readMassStorageConfigValues(i),
                bulkInFS: readMassStorageBulkInValuesFS(i),
                bulkOutFS: readMassStorageBulkOutValuesFS(i),
                bulkInHS: isHighSpeedEnabled ? readMassStorageBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readMassStorageBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'dfu',
            checkboxId: 'class-dfu',
            countId: 'class-num-dfu',
            buildInstance: (i) => ({
                interfaceConfig: readDFUConfigValues(i)
            })
        },
        {
            key: 'printer',
            checkboxId: 'class-printer',
            countId: 'class-num-printer',
            buildInstance: (i) => ({
                interfaceConfig: readPrinterConfigValues(i),
                bulkInFS: readPrinterBulkInValuesFS(i),
                bulkOutFS: readPrinterBulkOutValuesFS(i),
                bulkInHS: isHighSpeedEnabled ? readPrinterBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readPrinterBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'video',
            checkboxId: 'class-video',
            countId: 'class-num-video',
            buildInstance: (i) => ({
                interfaceConfig: readVideoConfigValues(i),
                bulkInFS: readVideoBulkInValuesFS(i),
                bulkOutFS: readVideoBulkOutValuesFS(i),
                bulkInHS: isHighSpeedEnabled ? readVideoBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readVideoBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'mtp',
            checkboxId: 'class-mtp',
            countId: 'class-num-mtp',
            buildInstance: (i) => ({
                interfaceConfig: readMTPConfigValues(i),
                bulkInFS: readMTPBulkInValuesFS(i),
                bulkOutFS: readMTPBulkOutValuesFS(i),
                bulkInHS: isHighSpeedEnabled ? readMTPBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readMTPBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'ptp',
            checkboxId: 'class-ptp',
            countId: 'class-num-ptp',
            buildInstance: (i) => ({
                interfaceConfig: readPTPConfigValues(i),
                bulkInFS: readPTPBulkInValuesFS(i),
                bulkOutFS: readPTPBulkOutValuesFS(i),
                bulkInHS: isHighSpeedEnabled ? readPTPBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readPTPBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'cdcAcm',
            checkboxId: 'class-cdc',
            countId: 'class-num-cdc',
            buildInstance: (i) => ({
                interfaceConfig: readCDCConfigValues(i),
                notifyFS: readCDCNotifyValuesFS(i),
                bulkInFS: readCDCBulkInValuesFS(i),
                bulkOutFS: readCDCBulkOutValuesFS(i),
                notifyHS: isHighSpeedEnabled ? readCDCNotifyValuesHS(i) : null,
                bulkInHS: isHighSpeedEnabled ? readCDCBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readCDCBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'rndis',
            checkboxId: 'class-rndis',
            countId: 'class-num-rndis',
            buildInstance: (i) => ({
                interfaceConfig: readRNDISConfigValues(i),
                notifyFS: readRNDISNotifyValuesFS(i),
                bulkInFS: readRNDISBulkInValuesFS(i),
                bulkOutFS: readRNDISBulkOutValuesFS(i),
                notifyHS: isHighSpeedEnabled ? readRNDISNotifyValuesHS(i) : null,
                bulkInHS: isHighSpeedEnabled ? readRNDISBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readRNDISBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'ecm',
            checkboxId: 'class-ecm',
            countId: 'class-num-ecm',
            buildInstance: (i) => ({
                interfaceConfig: readECMConfigValues(i),
                notifyFS: readECMNotifyValuesFS(i),
                bulkInFS: readECMBulkInValuesFS(i),
                bulkOutFS: readECMBulkOutValuesFS(i),
                notifyHS: isHighSpeedEnabled ? readECMNotifyValuesHS(i) : null,
                bulkInHS: isHighSpeedEnabled ? readECMBulkInValuesHS(i) : null,
                bulkOutHS: isHighSpeedEnabled ? readECMBulkOutValuesHS(i) : null
            })
        },
        {
            key: 'audio10',
            checkboxId: 'class-audio',
            countId: 'class-num-audio',
            buildInstance: (i) => ({
                interfaceConfig: readAudioConfigValues(i),
                endpointFS: readAudioEndpointValuesFS(i),
                endpointHS: isHighSpeedEnabled ? readAudioEndpointValuesHS(i) : null
            })
        },
        {
            key: 'audio20',
            checkboxId: 'class-audio2',
            countId: 'class-num-audio2',
            buildInstance: (i) => ({
                interfaceConfig: readAudio2ConfigValues(i),
                endpointFS: readAudio2EndpointValuesFS(i),
                endpointHS: isHighSpeedEnabled ? readAudio2EndpointValuesHS(i) : null
            })
        }
    ];

    const classOrderIndex = new Map(classCardOrder.map((className, index) => [className, index]));
    const jsonClassToUiClass = {
        hid: 'hid',
        massStorage: 'msc',
        dfu: 'dfu',
        printer: 'printer',
        video: 'video',
        mtp: 'mtp',
        ptp: 'ptp',
        cdcAcm: 'cdc',
        rndis: 'rndis',
        ecm: 'ecm',
        audio10: 'audio',
        audio20: 'audio2'
    };

    classDefinitions.sort((a, b) => {
        const aIndex = classOrderIndex.get(jsonClassToUiClass[a.key]) ?? Number.MAX_SAFE_INTEGER;
        const bIndex = classOrderIndex.get(jsonClassToUiClass[b.key]) ?? Number.MAX_SAFE_INTEGER;
        return aIndex - bIndex;
    });

    classDefinitions.forEach((classDef) => {
        const checkbox = document.getElementById(classDef.checkboxId);
        if (!checkbox || !checkbox.checked) {
            return;
        }

        const classCount = getClassInstanceCount(classDef.countId);
        jsonOutput.classes[classDef.key] = {
            instances: Array.from({ length: classCount }, (_, index) => classDef.buildInstance(index + 1))
        };
    });

    return jsonOutput;
}

function renderJSONArray(formValues, configValues) {
    const jsonOutput = collectConfiguredParameters(formValues, configValues);

    // Pretty print JSON with syntax highlighting
    const jsonString = JSON.stringify(jsonOutput, null, 2);
    const highlighted = jsonString
        .replace(/("[\w]+"): /g, '<span class="descriptor-keyword">$1</span>: ')
        .replace(/: (".*?")/g, ': <span class="descriptor-value">$1</span>')
        .replace(/: (0x[0-9A-F]+)/g, ': <span class="descriptor-value">$1</span>')
        .replace(/: (\d+)/g, ': <span class="descriptor-value">$1</span>');

    document.getElementById('json-array-output').innerHTML = highlighted;
}

function buildHexRows(byteArray) {
    const rows = [];
    for (let i = 0; i < byteArray.length; i += 8) {
        rows.push({
            offset: toHex(i, 2),
            bytes: byteArray.slice(i, i + 8).map((b) => toHex(b, 1))
        });
    }
    return rows;
}

function renderHexDump(descriptor) {
    const values = readFormValues();
    const configValues = readConfigurationValues();
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);

    // Full Speed
    const fsRows = buildHexRows(descriptor);

    // High Speed
    let hsRows = null;
    let hsSize = 0;
    if (isHighSpeedEnabled) {
        const hsResult = buildHighSpeedDescriptor();
        if (hsResult && hsResult.descriptor && hsResult.descriptor.length > 0) {
            hsRows = buildHexRows(hsResult.descriptor);
            hsSize = hsResult.descriptor.length;
        }
    }

    // String
    const stringBytes = buildStringFrameworkBytes(values, configValues);
    const stringHexRows = stringBytes.length > 0 ? buildHexRows(stringBytes) : null;

    // Language ID
    const langIdBytes = buildLanguageIdBytes();
    const langIdHexRows = langIdBytes.length > 0 ? buildHexRows(langIdBytes) : null;

    document.getElementById('hex-output').innerHTML = templates.hex({
        rows: fsRows,
        size: descriptor.length,
        hsRows: hsRows,
        hsSize: hsSize,
        stringHexRows: stringHexRows,
        stringSize: stringBytes.length,
        langIdHexRows: langIdHexRows,
        langIdSize: langIdBytes.length
    });
}

// Helper: push all enabled USB class descriptors into the readable rows array.
// className: 'hid'|'msc'|'dfu'|'printer'|'video'|'mtp'|'ptp'|'cdc'|'rndis'|'ecm'|'audio'|'audio2'
// isHS: true for high-speed variant (use HS endpoint read functions)
function pushReadableClassRows(rows, className, ifOffset, isHS) {
    const cb = document.getElementById(`class-${className}`);
    if (!cb || !cb.checked) return;
    const n = parseInt((document.getElementById(`class-num-${className}`) || {}).value) || 1;
    const speed = isHS ? 'High Speed' : 'Full Speed';

    const pushEp = (label, addr, attrs, pkt, interval) => {
        rows.push({ label: `  bEndpointAddress:`, value: addr });
        rows.push({ label: `  bmAttributes:`, value: attrs });
        rows.push({ label: `  wMaxPacketSize:`, value: pkt });
        rows.push({ label: `  bInterval:`, value: String(interval) });
    };

    const epBlock = (label, ep, isHex) => {
        const addr = isHex ? ep.addr : toHex(ep.addr, 1);
        const pktRaw = isHex ? parseHexValue(ep.pkt || '0x0040', 2, 'pkt') : ep.pkt;
        rows.push({ label: '', value: '' });
        rows.push({ label: label, value: '' });
        rows.push({ label: '  bLength:', value: '7 bytes' });
        rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
        rows.push({ label: '  bEndpointAddress:', value: isHex ? toHex(parseHexValue(ep.addr || '0x81', 1, 'addr'), 1) : toHex(ep.addr, 1) });
        rows.push({ label: '  bmAttributes:', value: toHex(ep.attrs, 1) });
        const pktVal = isHex ? parseHexValue(ep.pkt || '0x0040', 2, 'pkt') : ep.pkt;
        rows.push({ label: '  wMaxPacketSize:', value: toHex(pktVal, 2) + ' (' + pktVal + ' bytes)' });
        rows.push({ label: '  bInterval:', value: String(isHex ? parseHexValue(ep.interval || '0', 1, 'int') : ep.interval) });
    };

    const ifRow = (num, altSetting, numEp, cls, sub, proto, iIf) => {
        rows.push({ label: '  bLength:', value: '9 bytes' });
        rows.push({ label: '  bDescriptorType:', value: '0x04 (Interface)' });
        rows.push({ label: '  bInterfaceNumber:', value: toHex(num, 1) });
        rows.push({ label: '  bAlternateSetting:', value: toHex(altSetting, 1) });
        rows.push({ label: '  bNumEndpoints:', value: String(numEp) });
        rows.push({ label: '  bInterfaceClass:', value: toHex(cls, 1) });
        rows.push({ label: '  bInterfaceSubClass:', value: toHex(sub, 1) });
        rows.push({ label: '  bInterfaceProtocol:', value: toHex(proto, 1) });
        rows.push({ label: '  iInterface:', value: String(iIf) });
    };

    const iadRow = (firstIf, count, cls, sub, proto) => {
        rows.push({ label: '', value: '' });
        rows.push({ label: 'IAD (Interface Association):', value: '' });
        rows.push({ label: '  bLength:', value: '8 bytes' });
        rows.push({ label: '  bDescriptorType:', value: '0x0B (IAD)' });
        rows.push({ label: '  bFirstInterface:', value: toHex(firstIf, 1) });
        rows.push({ label: '  bInterfaceCount:', value: String(count) });
        rows.push({ label: '  bFunctionClass:', value: toHex(cls, 1) });
        rows.push({ label: '  bFunctionSubClass:', value: toHex(sub, 1) });
        rows.push({ label: '  bFunctionProtocol:', value: toHex(proto, 1) });
        rows.push({ label: '  iFunction:', value: '0x00' });
    };

    const bulkEpFromHex = (label, addrRaw, pktRaw, intervalRaw, defAddr, defPkt, defInt) => {
        const addr = parseHexValue(addrRaw || defAddr, 1, 'addr');
        const pkt = parseHexValue(pktRaw || defPkt, 2, 'pkt');
        const interval = parseHexValue(intervalRaw || defInt, 1, 'int');
        rows.push({ label: '', value: '' });
        rows.push({ label: label, value: '' });
        rows.push({ label: '  bLength:', value: '7 bytes' });
        rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
        rows.push({ label: '  bEndpointAddress:', value: toHex(addr, 1) });
        rows.push({ label: '  bmAttributes:', value: '0x02 (Bulk)' });
        rows.push({ label: '  wMaxPacketSize:', value: toHex(pkt, 2) + ' (' + pkt + ' bytes)' });
        rows.push({ label: '  bInterval:', value: String(interval) });
    };

    const isoEpFromHex = (label, addrRaw, pktRaw, intervalRaw, defAddr, defPkt, defInt, bmAttrs, syncName, synchAddr) => {
        const addr = parseHexValue(addrRaw || defAddr, 1, 'addr');
        const pkt = parseHexValue(pktRaw || defPkt, 2, 'pkt');
        const interval = parseHexValue(intervalRaw || defInt, 1, 'int');
        const isUAC1 = synchAddr !== undefined;
        rows.push({ label: '', value: '' });
        rows.push({ label: label, value: '' });
        rows.push({ label: '  bLength:', value: isUAC1 ? '9 bytes' : '7 bytes' });
        rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
        rows.push({ label: '  bEndpointAddress:', value: toHex(addr, 1) });
        rows.push({ label: '  bmAttributes:', value: toHex(bmAttrs || 0x01, 1) + ' (Isochronous' + (syncName ? ', ' + syncName : '') + ')' });
        rows.push({ label: '  wMaxPacketSize:', value: toHex(pkt, 2) + ' (' + pkt + ' bytes)' });
        rows.push({ label: '  bInterval:', value: String(interval) });
        if (isUAC1) {
            rows.push({ label: '  bRefresh:', value: '0x00' });
            rows.push({ label: '  bSynchAddress:', value: toHex(synchAddr, 1) });
        }
    };

    if (className === 'hid') {
        const classIPR2 = { hid: 1, msc: 1, dfu: 1, printer: 1, video: 1, mtp: 1, ptp: 1, cdc: 2, rndis: 2, ecm: 2, audio: 2, audio2: 2 };
        for (let i = 1; i <= n; i++) {
            const ifVals = readInterfaceDescriptorValues(i);
            ifVals.bInterfaceNumber = ifOffset + (i - 1);
            const hidVals = readHIDDetailsValues(i);
            const epIn = isHS ? readEndpointInDescriptorValuesHighSpeed(i) : readEndpointInDescriptorValues(i);
            const epOut = optionalEndpointState[i] && isProtocolNone(i)
                ? (isHS ? readEndpointOutDescriptorValuesHighSpeed(i) : readEndpointOutDescriptorValues(i)) : null;

            if (Object.keys(ifVals).length > 0 || Object.keys(hidVals).length > 0 || Object.keys(epIn).length > 0) {
                rows.push({ label: '', value: '' });
                rows.push({ label: `═══ HID Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
                if (Object.keys(ifVals).length > 0) {
                    rows.push({ label: '', value: '' });
                    rows.push({ label: 'Interface Descriptor:', value: '' });
                    ifRow(ifVals.bInterfaceNumber, ifVals.bAlternateSetting,
                        epOut && Object.keys(epOut).length > 0 ? 2 : ifVals.bNumEndpoints,
                        ifVals.bInterfaceClass, ifVals.bInterfaceSubClass, ifVals.bInterfaceProtocol, ifVals.iInterface);
                }
                if (Object.keys(hidVals).length > 0) {
                    rows.push({ label: '', value: '' });
                    rows.push({ label: 'HID Descriptor:', value: '' });
                    rows.push({ label: '  bLength:', value: '9 bytes' });
                    rows.push({ label: '  bDescriptorType:', value: '0x21 (HID)' });
                    rows.push({ label: '  bcdHID:', value: toHex(hidVals.hidBcdHID, 2) });
                    rows.push({ label: '  bCountryCode:', value: toHex(hidVals.hidBCountryCode, 1) });
                    rows.push({ label: '  bNumDescriptors:', value: String(hidVals.hidBNumDescriptors) });
                    rows.push({ label: '  bReportDescriptorType:', value: '0x22 (Report)' });
                    rows.push({ label: '  wReportDescriptorLength:', value: toHex(0x0100, 2) });
                }
                if (Object.keys(epIn).length > 0) {
                    rows.push({ label: '', value: '' });
                    rows.push({ label: 'Endpoint IN Descriptor:', value: '' });
                    rows.push({ label: '  bLength:', value: '7 bytes' });
                    rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
                    rows.push({ label: '  bEndpointAddress:', value: toHex(epIn.endpointBEndpointAddress, 1) });
                    rows.push({ label: '  bmAttributes:', value: toHex(epIn.endpointBmAttributes, 1) });
                    rows.push({ label: '  wMaxPacketSize:', value: toHex(epIn.endpointWMaxPacketSize, 2) + ' (' + epIn.endpointWMaxPacketSize + ' bytes)' });
                    rows.push({ label: '  bInterval:', value: String(epIn.endpointBInterval) });
                }
                if (epOut && Object.keys(epOut).length > 0) {
                    rows.push({ label: '', value: '' });
                    rows.push({ label: 'Endpoint OUT Descriptor:', value: '' });
                    rows.push({ label: '  bLength:', value: '7 bytes' });
                    rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
                    rows.push({ label: '  bEndpointAddress:', value: toHex(epOut.endpointBEndpointAddress, 1) });
                    rows.push({ label: '  bmAttributes:', value: toHex(epOut.endpointBmAttributes, 1) });
                    rows.push({ label: '  wMaxPacketSize:', value: toHex(epOut.endpointWMaxPacketSize, 2) + ' (' + epOut.endpointWMaxPacketSize + ' bytes)' });
                    rows.push({ label: '  bInterval:', value: String(epOut.endpointBInterval) });
                }
            }
        }
        return;
    }

    if (className === 'msc') {
        for (let i = 1; i <= n; i++) {
            const cfg = readMassStorageConfigValues(i);
            const epIn = isHS ? readMassStorageBulkInValuesHS(i) : readMassStorageBulkInValuesFS(i);
            const epOut = isHS ? readMassStorageBulkOutValuesHS(i) : readMassStorageBulkOutValuesFS(i);
            const subCls = parseHexValue(cfg.mscBInterfaceSubClass || '0x06', 1, 'mscBInterfaceSubClass');
            const proto = parseHexValue(cfg.mscBInterfaceProtocol || '0x50', 1, 'mscBInterfaceProtocol');

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ MSC (Mass Storage) Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Interface Descriptor:', value: '' });
            ifRow(ifOffset + (i - 1), 0x00, 2, 0x08, subCls, proto, 0x00);

            const inKey = isHS ? 'mscBulkInEndpointHS' : 'mscBulkInEndpointFS';
            const inPktKey = isHS ? 'mscBulkInMaxPacketSizeHS' : 'mscBulkInMaxPacketSizeFS';
            const inIntKey = isHS ? 'mscBulkInIntervalHS' : 'mscBulkInIntervalFS';
            const outKey = isHS ? 'mscBulkOutEndpointHS' : 'mscBulkOutEndpointFS';
            const outPktKey = isHS ? 'mscBulkOutMaxPacketSizeHS' : 'mscBulkOutMaxPacketSizeFS';
            const outIntKey = isHS ? 'mscBulkOutIntervalHS' : 'mscBulkOutIntervalFS';
            const defPkt = isHS ? '0x0200' : '0x0040';

            bulkEpFromHex('Endpoint IN Descriptor:', epIn[inKey], epIn[inPktKey], epIn[inIntKey], '0x81', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor:', epOut[outKey], epOut[outPktKey], epOut[outIntKey], '0x01', defPkt, '0');
        }
        return;
    }

    if (className === 'dfu') {
        for (let i = 1; i <= n; i++) {
            const cfg = readDFUConfigValues(i);
            const subCls = parseHexValue(cfg.dfuBInterfaceSubClass || '0x01', 1, 'dfuBInterfaceSubClass');
            const proto = parseHexValue(cfg.dfuBInterfaceProtocol || '0x02', 1, 'dfuBInterfaceProtocol');
            const iIf = parseNumberValue(cfg.dfuIInterface || 0, { id: 'dfuIInterface', min: 0, max: 255, default: 0 });
            const bmAttrs = parseHexValue(cfg.dfuBmAttributes || '0x0B', 1, 'dfuBmAttributes');
            const detach = parseNumberValue(cfg.dfuDetachTimeout || 1000, { id: 'dfuDetachTimeout', min: 0, max: 65535, default: 1000 });
            const xfSize = parseNumberValue(cfg.dfuTransferSize || 1024, { id: 'dfuTransferSize', min: 1, max: 65535, default: 1024 });
            const bcd = parseHexValue(cfg.dfuBcdVersion || '0x0110', 2, 'dfuBcdVersion');

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ DFU Instance ${i} ═══`, value: '', isHeader: true });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Interface Descriptor:', value: '' });
            ifRow(ifOffset + (i - 1), 0x00, 0x00, 0xFE, subCls, proto, iIf);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'DFU Functional Descriptor:', value: '' });
            rows.push({ label: '  bLength:', value: '9 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x21 (DFU Functional)' });
            rows.push({ label: '  bmAttributes:', value: toHex(bmAttrs, 1) });
            rows.push({ label: '  wDetachTimeout:', value: String(detach) + ' ms' });
            rows.push({ label: '  wTransferSize:', value: String(xfSize) + ' bytes' });
            rows.push({ label: '  bcdDFUVersion:', value: toHex(bcd, 2) });
        }
        return;
    }

    if (className === 'printer') {
        for (let i = 1; i <= n; i++) {
            const cfg = readPrinterConfigValues(i);
            const epIn  = isHS ? readPrinterBulkInValuesHS(i)  : readPrinterBulkInValuesFS(i);
            const epOut = isHS ? readPrinterBulkOutValuesHS(i) : readPrinterBulkOutValuesFS(i);
            const subCls = parseHexValue(cfg.printerBInterfaceSubClass || '0x01', 1, 'printerBInterfaceSubClass');
            const proto = parseHexValue(cfg.printerBInterfaceProtocol || '0x02', 1, 'printerBInterfaceProtocol');
            const iIf = parseNumberValue(cfg.printerIInterface || 0, { id: 'printerIInterface', min: 0, max: 255, default: 0 });

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ Printer Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Interface Descriptor:', value: '' });
            ifRow(ifOffset + (i - 1), 0x00, 2, 0x07, subCls, proto, iIf);

            const sfx = isHS ? 'HS' : 'FS';
            const defPkt = isHS ? '0x0200' : '0x0040';
            bulkEpFromHex('Endpoint IN Descriptor:', epIn[`printerBulkInEndpoint${sfx}`], epIn[`printerBulkInMaxPacketSize${sfx}`], epIn[`printerBulkInInterval${sfx}`], '0x81', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor:', epOut[`printerBulkOutEndpoint${sfx}`], epOut[`printerBulkOutMaxPacketSize${sfx}`], epOut[`printerBulkOutInterval${sfx}`], '0x01', defPkt, '0');
        }
        return;
    }

    if (className === 'video') {
        for (let i = 1; i <= n; i++) {
            const cfg = readVideoConfigValues(i);
            const epIn  = isHS ? readVideoBulkInValuesHS(i)  : readVideoBulkInValuesFS(i);
            const epOut = isHS ? readVideoBulkOutValuesHS(i) : readVideoBulkOutValuesFS(i);
            const subCls = parseHexValue(cfg.videoBInterfaceSubClass || '0x02', 1, 'videoBInterfaceSubClass');
            const proto = parseHexValue(cfg.videoBInterfaceProtocol || '0x00', 1, 'videoBInterfaceProtocol');
            const iIf = parseNumberValue(cfg.videoIInterface || 0, { id: 'videoIInterface', min: 0, max: 255, default: 0 });

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ Video (UVC) Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Interface Descriptor:', value: '' });
            ifRow(ifOffset + (i - 1), 0x00, 2, 0x0E, subCls, proto, iIf);

            const sfx = isHS ? 'HS' : 'FS';
            const defPkt = isHS ? '0x0200' : '0x0040';
            bulkEpFromHex('Endpoint IN Descriptor:', epIn[`videoBulkInEndpoint${sfx}`], epIn[`videoBulkInMaxPacketSize${sfx}`], epIn[`videoBulkInInterval${sfx}`], '0x83', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor:', epOut[`videoBulkOutEndpoint${sfx}`], epOut[`videoBulkOutMaxPacketSize${sfx}`], epOut[`videoBulkOutInterval${sfx}`], '0x03', defPkt, '0');
        }
        return;
    }

    if (className === 'mtp') {
        for (let i = 1; i <= n; i++) {
            const cfg = readMTPConfigValues(i);
            const epIn  = isHS ? readMTPBulkInValuesHS(i)  : readMTPBulkInValuesFS(i);
            const epOut = isHS ? readMTPBulkOutValuesHS(i) : readMTPBulkOutValuesFS(i);
            const subCls = parseHexValue(cfg.mtpBInterfaceSubClass || '0x01', 1, 'mtpBInterfaceSubClass');
            const proto = parseHexValue(cfg.mtpBInterfaceProtocol || '0x01', 1, 'mtpBInterfaceProtocol');
            const iIf = parseNumberValue(cfg.mtpIInterface || 0, { id: 'mtpIInterface', min: 0, max: 255, default: 0 });

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ MTP Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Interface Descriptor:', value: '' });
            ifRow(ifOffset + (i - 1), 0x00, 2, 0x06, subCls, proto, iIf);

            const sfx = isHS ? 'HS' : 'FS';
            const defPkt = isHS ? '0x0200' : '0x0040';
            bulkEpFromHex('Endpoint IN Descriptor:', epIn[`mtpBulkInEndpoint${sfx}`], epIn[`mtpBulkInMaxPacketSize${sfx}`], epIn[`mtpBulkInInterval${sfx}`], '0x84', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor:', epOut[`mtpBulkOutEndpoint${sfx}`], epOut[`mtpBulkOutMaxPacketSize${sfx}`], epOut[`mtpBulkOutInterval${sfx}`], '0x04', defPkt, '0');
        }
        return;
    }

    if (className === 'ptp') {
        for (let i = 1; i <= n; i++) {
            const cfg = readPTPConfigValues(i);
            const epIn  = isHS ? readPTPBulkInValuesHS(i)  : readPTPBulkInValuesFS(i);
            const epOut = isHS ? readPTPBulkOutValuesHS(i) : readPTPBulkOutValuesFS(i);
            const subCls = parseHexValue(cfg.ptpBInterfaceSubClass || '0x01', 1, 'ptpBInterfaceSubClass');
            const proto = parseHexValue(cfg.ptpBInterfaceProtocol || '0x01', 1, 'ptpBInterfaceProtocol');
            const iIf = parseNumberValue(cfg.ptpIInterface || 0, { id: 'ptpIInterface', min: 0, max: 255, default: 0 });

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ PTP Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Interface Descriptor:', value: '' });
            ifRow(ifOffset + (i - 1), 0x00, 2, 0x06, subCls, proto, iIf);

            const sfx = isHS ? 'HS' : 'FS';
            const defPkt = isHS ? '0x0200' : '0x0040';
            bulkEpFromHex('Endpoint IN Descriptor:', epIn[`ptpBulkInEndpoint${sfx}`], epIn[`ptpBulkInMaxPacketSize${sfx}`], epIn[`ptpBulkInInterval${sfx}`], '0x85', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor:', epOut[`ptpBulkOutEndpoint${sfx}`], epOut[`ptpBulkOutMaxPacketSize${sfx}`], epOut[`ptpBulkOutInterval${sfx}`], '0x05', defPkt, '0');
        }
        return;
    }

    if (className === 'cdc') {
        for (let i = 1; i <= n; i++) {
            const cfg = readCDCConfigValues(i);
            const notify = isHS ? readCDCNotifyValuesHS(i)  : readCDCNotifyValuesFS(i);
            const bkIn   = isHS ? readCDCBulkInValuesHS(i)  : readCDCBulkInValuesFS(i);
            const bkOut  = isHS ? readCDCBulkOutValuesHS(i) : readCDCBulkOutValuesFS(i);
            const subCls  = parseHexValue(cfg.cdcBInterfaceSubClass  || '0x02', 1, 'cdcBInterfaceSubClass');
            const proto   = parseHexValue(cfg.cdcBInterfaceProtocol  || '0x01', 1, 'cdcBInterfaceProtocol');
            const bmCaps  = parseHexValue(cfg.cdcBmCapabilities      || '0x02', 1, 'cdcBmCapabilities');
            const bcdCDC  = parseHexValue(cfg.cdcBcdCDC              || '0x0110', 2, 'cdcBcdCDC');
            const commIf  = ifOffset + (i - 1) * 2;
            const dataIf  = commIf + 1;
            const sfx = isHS ? 'HS' : 'FS';
            const defPkt  = isHS ? '0x0200' : '0x0040';

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ CDC ACM Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            iadRow(commIf, 2, 0x02, subCls, proto);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Communication Interface Descriptor:', value: '' });
            ifRow(commIf, 0x00, 1, 0x02, subCls, proto, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Header Functional Descriptor:', value: '' });
            rows.push({ label: '  bFunctionLength:', value: '5 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x24 (CS_INTERFACE)' });
            rows.push({ label: '  bDescriptorSubtype:', value: '0x00 (Header)' });
            rows.push({ label: '  bcdCDC:', value: toHex(bcdCDC, 2) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Call Management Functional Descriptor:', value: '' });
            rows.push({ label: '  bFunctionLength:', value: '5 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x24 (CS_INTERFACE)' });
            rows.push({ label: '  bDescriptorSubtype:', value: '0x01 (Call Management)' });
            rows.push({ label: '  bmCapabilities:', value: '0x00' });
            rows.push({ label: '  bDataInterface:', value: String(dataIf) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC ACM Functional Descriptor:', value: '' });
            rows.push({ label: '  bFunctionLength:', value: '4 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x24 (CS_INTERFACE)' });
            rows.push({ label: '  bDescriptorSubtype:', value: '0x02 (ACM)' });
            rows.push({ label: '  bmCapabilities:', value: toHex(bmCaps, 1) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Union Functional Descriptor:', value: '' });
            rows.push({ label: '  bFunctionLength:', value: '5 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x24 (CS_INTERFACE)' });
            rows.push({ label: '  bDescriptorSubtype:', value: '0x06 (Union)' });
            rows.push({ label: '  bMasterInterface:', value: String(commIf) });
            rows.push({ label: '  bSlaveInterface0:', value: String(dataIf) });

            const notifyEp  = parseHexValue(notify[`cdcNotifyEndpoint${sfx}`]     || '0x81', 1, 'notifyEp');
            const notifyPkt = parseHexValue(notify[`cdcNotifyMaxPacketSize${sfx}`] || '0x0008', 2, 'notifyPkt');
            const notifyInt = parseHexValue(notify[`cdcNotifyInterval${sfx}`]      || '16', 1, 'notifyInt');
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Notification Endpoint Descriptor (Interrupt IN):', value: '' });
            rows.push({ label: '  bLength:', value: '7 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
            rows.push({ label: '  bEndpointAddress:', value: toHex(notifyEp, 1) });
            rows.push({ label: '  bmAttributes:', value: '0x03 (Interrupt)' });
            rows.push({ label: '  wMaxPacketSize:', value: toHex(notifyPkt, 2) + ' (' + notifyPkt + ' bytes)' });
            rows.push({ label: '  bInterval:', value: String(notifyInt) });

            rows.push({ label: '', value: '' });
            rows.push({ label: 'Data Interface Descriptor:', value: '' });
            ifRow(dataIf, 0x00, 2, 0x0A, 0x00, 0x00, 0x00);

            bulkEpFromHex('Endpoint IN Descriptor (Bulk):', bkIn[`cdcBulkInEndpoint${sfx}`], bkIn[`cdcBulkInMaxPacketSize${sfx}`], bkIn[`cdcBulkInInterval${sfx}`], '0x82', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor (Bulk):', bkOut[`cdcBulkOutEndpoint${sfx}`], bkOut[`cdcBulkOutMaxPacketSize${sfx}`], bkOut[`cdcBulkOutInterval${sfx}`], '0x02', defPkt, '0');
        }
        return;
    }

    if (className === 'rndis') {
        for (let i = 1; i <= n; i++) {
            const cfg    = readRNDISConfigValues(i);
            const notify = isHS ? readRNDISNotifyValuesHS(i)  : readRNDISNotifyValuesFS(i);
            const bkIn   = isHS ? readRNDISBulkInValuesHS(i)  : readRNDISBulkInValuesFS(i);
            const bkOut  = isHS ? readRNDISBulkOutValuesHS(i) : readRNDISBulkOutValuesFS(i);
            const bcdCDC = parseHexValue(cfg.rndisBcdCDC || '0x0110', 2, 'rndisBcdCDC');
            const callMgmt = parseHexValue(cfg.rndisCallMgmtCapabilities || '0x00', 1, 'rndisCallMgmt');
            const acmCaps  = parseHexValue(cfg.rndisAcmCapabilities || '0x00', 1, 'rndisAcm');
            const commIf = ifOffset + (i - 1) * 2;
            const dataIf = commIf + 1;
            const sfx = isHS ? 'HS' : 'FS';
            const defPkt = isHS ? '0x0200' : '0x0040';

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ CDC RNDIS Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            iadRow(commIf, 2, 0x02, 0x02, 0xFF);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Communication Interface Descriptor:', value: '' });
            ifRow(commIf, 0x00, 1, 0x02, 0x02, 0xFF, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Header Functional Descriptor:', value: '' });
            rows.push({ label: '  bcdCDC:', value: toHex(bcdCDC, 2) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Call Management Functional Descriptor:', value: '' });
            rows.push({ label: '  bmCapabilities:', value: toHex(callMgmt, 1) });
            rows.push({ label: '  bDataInterface:', value: String(dataIf) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC ACM Functional Descriptor:', value: '' });
            rows.push({ label: '  bmCapabilities:', value: toHex(acmCaps, 1) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Union Functional Descriptor:', value: '' });
            rows.push({ label: '  bMasterInterface:', value: String(commIf) });
            rows.push({ label: '  bSlaveInterface0:', value: String(dataIf) });

            const notifyEp  = parseHexValue(notify.notifyEndpoint  || notify[`notifyEndpoint${sfx}`]     || '0x81', 1, 'notifyEp');
            const notifyPkt = parseHexValue(notify.notifyMaxPacketSize || notify[`notifyMaxPacketSize${sfx}`] || '0x0008', 2, 'notifyPkt');
            const notifyInt = parseHexValue(notify.notifyInterval   || notify[`notifyInterval${sfx}`]    || '16', 1, 'notifyInt');
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Notification Endpoint Descriptor (Interrupt IN):', value: '' });
            rows.push({ label: '  bLength:', value: '7 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
            rows.push({ label: '  bEndpointAddress:', value: toHex(notifyEp, 1) });
            rows.push({ label: '  bmAttributes:', value: '0x03 (Interrupt)' });
            rows.push({ label: '  wMaxPacketSize:', value: toHex(notifyPkt, 2) + ' (' + notifyPkt + ' bytes)' });
            rows.push({ label: '  bInterval:', value: String(notifyInt) });

            rows.push({ label: '', value: '' });
            rows.push({ label: 'Data Interface Descriptor:', value: '' });
            ifRow(dataIf, 0x00, 2, 0x0A, 0x00, 0x00, 0x00);

            bulkEpFromHex('Endpoint IN Descriptor (Bulk):', bkIn.bulkInEndpointFS || bkIn.bulkInEndpointHS, bkIn.bulkInMaxPacketSizeFS || bkIn.bulkInMaxPacketSizeHS, bkIn.bulkInIntervalFS || bkIn.bulkInIntervalHS, '0x82', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor (Bulk):', bkOut.bulkOutEndpointFS || bkOut.bulkOutEndpointHS, bkOut.bulkOutMaxPacketSizeFS || bkOut.bulkOutMaxPacketSizeHS, bkOut.bulkOutIntervalFS || bkOut.bulkOutIntervalHS, '0x02', defPkt, '0');
        }
        return;
    }

    if (className === 'ecm') {
        for (let i = 1; i <= n; i++) {
            const cfg    = readECMConfigValues(i);
            const notify = isHS ? readECMNotifyValuesHS(i)  : readECMNotifyValuesFS(i);
            const bkIn   = isHS ? readECMBulkInValuesHS(i)  : readECMBulkInValuesFS(i);
            const bkOut  = isHS ? readECMBulkOutValuesHS(i) : readECMBulkOutValuesFS(i);
            const bcdCDC = parseHexValue(cfg.ecmBcdCDC || '0x0120', 2, 'ecmBcdCDC');
            const commIf = ifOffset + (i - 1) * 2;
            const dataIf = commIf + 1;
            const defPkt = isHS ? '0x0200' : '0x0040';

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ CDC ECM Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            iadRow(commIf, 2, 0x02, 0x06, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Communication Interface Descriptor:', value: '' });
            ifRow(commIf, 0x00, 1, 0x02, 0x06, 0x00, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Header Functional Descriptor:', value: '' });
            rows.push({ label: '  bcdCDC:', value: toHex(bcdCDC, 2) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'ECM Ethernet Functional Descriptor:', value: '' });
            rows.push({ label: '  iMACAddress:', value: String(cfg.ecmMacStringIndex || 4) });
            rows.push({ label: '  wMaxSegmentSize:', value: String(cfg.ecmMaxSegmentSize || 1514) + ' bytes' });
            rows.push({ label: '  wNumberMCFilters:', value: String(cfg.ecmNumMcFilters || 0) });
            rows.push({ label: '  bNumberPowerFilters:', value: String(cfg.ecmNumPowerFilters || 0) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CDC Union Functional Descriptor:', value: '' });
            rows.push({ label: '  bMasterInterface:', value: String(commIf) });
            rows.push({ label: '  bSlaveInterface0:', value: String(dataIf) });

            const notifyEp  = parseHexValue(notify.notifyEndpointFS || notify.notifyEndpointHS || '0x81', 1, 'notifyEp');
            const notifyPkt = parseHexValue(notify.notifyMaxPacketSizeFS || notify.notifyMaxPacketSizeHS || '0x0008', 2, 'notifyPkt');
            const notifyInt = parseHexValue(notify.notifyIntervalFS || notify.notifyIntervalHS || '16', 1, 'notifyInt');
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Notification Endpoint Descriptor (Interrupt IN):', value: '' });
            rows.push({ label: '  bLength:', value: '7 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
            rows.push({ label: '  bEndpointAddress:', value: toHex(notifyEp, 1) });
            rows.push({ label: '  bmAttributes:', value: '0x03 (Interrupt)' });
            rows.push({ label: '  wMaxPacketSize:', value: toHex(notifyPkt, 2) + ' (' + notifyPkt + ' bytes)' });
            rows.push({ label: '  bInterval:', value: String(notifyInt) });

            rows.push({ label: '', value: '' });
            rows.push({ label: 'Data Interface Descriptor:', value: '' });
            ifRow(dataIf, 0x00, 2, 0x0A, 0x00, 0x00, 0x00);

            bulkEpFromHex('Endpoint IN Descriptor (Bulk):', bkIn.bulkInEndpointFS || bkIn.bulkInEndpointHS, bkIn.bulkInMaxPacketSizeFS || bkIn.bulkInMaxPacketSizeHS, bkIn.bulkInIntervalFS || bkIn.bulkInIntervalHS, '0x82', defPkt, '0');
            bulkEpFromHex('Endpoint OUT Descriptor (Bulk):', bkOut.bulkOutEndpointFS || bkOut.bulkOutEndpointHS, bkOut.bulkOutMaxPacketSizeFS || bkOut.bulkOutMaxPacketSizeHS, bkOut.bulkOutIntervalFS || bkOut.bulkOutIntervalHS, '0x02', defPkt, '0');
        }
        return;
    }

    if (className === 'audio') {
        for (let i = 1; i <= n; i++) {
            const cfg   = readAudioConfigValues(i);
            const epVals = isHS ? readAudioEndpointValuesHS(i) : readAudioEndpointValuesFS(i);
            const acSubClass = parseHexValue(cfg.audioControlSubClass   || '0x01', 1, 'audioControlSubClass');
            const asSubClass = parseHexValue(cfg.audioStreamingSubClass || '0x02', 1, 'audioStreamingSubClass');
            const terminalLink = parseNumberValue(cfg.audioTerminalLink || 1, { id: 'audioTerminalLink', min: 1, max: 253, default: 1 });
            const inputTerminalType  = parseHexValue(cfg.audioInputTerminalType  || '0x0101', 2, 'audioInputTerminalType');
            const outputTerminalType = parseHexValue(cfg.audioOutputTerminalType || '0x0301', 2, 'audioOutputTerminalType');
            const formatTag    = parseHexValue(cfg.audioFormatTag || '0x0001', 2, 'audioFormatTag');
            const formatType   = parseHexValue(cfg.audioFormatType, 1, 'audioFormatType');
            const numChannels  = parseNumberValue(cfg.audioNumChannels  || 2, { id: 'audioNumChannels', min: 1, max: 8, default: 2 });
            const bitRes       = parseNumberValue(cfg.audioBitResolution || 16, { id: 'audioBitResolution', min: 1, max: 32, default: 16 });
            const subframeSize = Math.ceil(bitRes / 8);
            const maxBitRate   = parseNumberValue(cfg.audioMaxBitRate || 128, { id: 'audioMaxBitRate', min: 1, max: 65535, default: 128 });
            const samplesPerFrame = parseNumberValue(cfg.audioSamplesPerFrame || 1, { id: 'audioSamplesPerFrame', min: 1, max: 65535, default: 1 });
            const sampleRates  = Array.isArray(cfg.audioSampleRate) && cfg.audioSampleRate.length > 0
                ? cfg.audioSampleRate : [48000];
            const channelConfig = getChannelConfigFromCount(numChannels);
            const fuId = terminalLink + 2;
            const otId = terminalLink + 1;
            const sumBControlSize = ((Array.isArray(cfg.audioFuMasterControls) ? cfg.audioFuMasterControls : [0, 1]).some((b) => b >= 8) || (Array.isArray(cfg.audioFuChannelControls) ? cfg.audioFuChannelControls : []).some((b) => b >= 8)) ? 2 : 1;
            const fuLength = 7 + (numChannels + 1) * sumBControlSize;
            const acTotalLength = 9 + 12 + fuLength + 9;
            const acIf = ifOffset + (i - 1) * 2;
            const asIf = acIf + 1;
            const sfx  = isHS ? 'HS' : 'FS';
            const defPkt = isHS ? '0x00C0' : '0x00C0';

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ Audio 1.0 Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            iadRow(acIf, 2, 0x01, acSubClass, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Audio Control Interface Descriptor (Alt 0):', value: '' });
            const audioAcIInterface = cfg.audioAcIInterface || 0;
            const audioAcEpEnabled = !!cfg.audioAcEpEnabled;
            ifRow(acIf, 0x00, audioAcEpEnabled ? 1 : 0, 0x01, acSubClass, 0x00, audioAcIInterface);
            if (audioAcIInterface > 0) rows.push({ label: '  iInterface:', value: String(audioAcIInterface) + '  ("' + (cfg.audioAcInterfaceString || '') + '")' });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'AC CS Interface Header Descriptor:', value: '' });
            rows.push({ label: '  bcdADC:', value: '0x0100  (UAC 1.0)' });
            rows.push({ label: '  wTotalLength:', value: String(acTotalLength) + ' bytes' });
            rows.push({ label: '  baInterfaceNr:', value: String(asIf) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Input Terminal Descriptor (IT):', value: '' });
            rows.push({ label: '  bTerminalID:', value: String(terminalLink) });
            rows.push({ label: '  wTerminalType:', value: '0x' + inputTerminalType.toString(16).toUpperCase().padStart(4, '0') });
            rows.push({ label: '  bNrChannels:', value: String(numChannels) });
            rows.push({ label: '  wChannelConfig:', value: '0x' + channelConfig.toString(16).toUpperCase().padStart(4, '0') + '  (' + (CHANNEL_CONFIG_LABELS[Math.max(1, Math.min(8, numChannels))] || '') + ')' });
            rows.push({ label: '', value: '' });
            const masterCtrlBits = (Array.isArray(cfg.audioFuMasterControls) ? cfg.audioFuMasterControls : [0, 1]).filter((b) => b >= 0);
            const channelCtrlBits = (Array.isArray(cfg.audioFuChannelControls) ? cfg.audioFuChannelControls : []).filter((b) => b >= 0);
            const fuControlNames = ['Mute', 'Volume', 'Bass', 'Mid', 'Treble', 'Graphic EQ', 'Auto Gain', 'Delay', 'Bass Boost', 'Loudness'];
            const masterLsbSum = masterCtrlBits.filter((b) => b < 8).reduce((acc, b) => acc | (1 << b), 0);
            const masterMsbSum = masterCtrlBits.filter((b) => b >= 8).reduce((acc, b) => acc | (1 << (b - 8)), 0);
            const channelLsbSum = channelCtrlBits.filter((b) => b < 8).reduce((acc, b) => acc | (1 << b), 0);
            const channelMsbSum = channelCtrlBits.filter((b) => b >= 8).reduce((acc, b) => acc | (1 << (b - 8)), 0);
            const masterLabel = masterCtrlBits.length > 0 ? masterCtrlBits.map((b) => fuControlNames[b] || `Bit${b}`).join(' + ') : 'none';
            const channelLabel = channelCtrlBits.length > 0 ? channelCtrlBits.map((b) => fuControlNames[b] || `Bit${b}`).join(' + ') : 'none';
            const masterHex = sumBControlSize === 1 ? '0x' + masterLsbSum.toString(16).toUpperCase().padStart(2, '0') : '[0x' + masterLsbSum.toString(16).toUpperCase().padStart(2, '0') + ', 0x' + masterMsbSum.toString(16).toUpperCase().padStart(2, '0') + ']';
            const channelHex = sumBControlSize === 1 ? '0x' + channelLsbSum.toString(16).toUpperCase().padStart(2, '0') : '[0x' + channelLsbSum.toString(16).toUpperCase().padStart(2, '0') + ', 0x' + channelMsbSum.toString(16).toUpperCase().padStart(2, '0') + ']';
            rows.push({ label: 'Feature Unit Descriptor (FU):', value: '' });
            rows.push({ label: '  bUnitID:', value: String(fuId) });
            rows.push({ label: '  bSourceID:', value: String(terminalLink) });
            rows.push({ label: '  bControlSize:', value: String(sumBControlSize) });
            rows.push({ label: '  bmaControls(0):', value: masterHex + '  (' + masterLabel + ')' });
            rows.push({ label: '  bmaControls(1..N):', value: channelHex + '  (' + channelLabel + ')' });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Output Terminal Descriptor (OT):', value: '' });
            rows.push({ label: '  bTerminalID:', value: String(otId) });
            rows.push({ label: '  wTerminalType:', value: '0x' + outputTerminalType.toString(16).toUpperCase().padStart(4, '0') });
            rows.push({ label: '  bSourceID:', value: String(fuId) });
            rows.push({ label: '', value: '' });
            if (audioAcEpEnabled) {
                const acEpAddr = isHS ? (cfg.audioAcEpAddressHS || '0x83') : (cfg.audioAcEpAddressFS || '0x83');
                const acEpPkt  = isHS ? (cfg.audioAcEpMaxPacketHS || 2)     : (cfg.audioAcEpMaxPacketFS || 2);
                const acEpIntvl = isHS ? (cfg.audioAcEpIntervalHS || 6)     : (cfg.audioAcEpIntervalFS || 16);
                rows.push({ label: 'AC Interrupt Endpoint Descriptor:', value: '' });
                rows.push({ label: '  bEndpointAddress:', value: '0x' + ((parseInt(acEpAddr, 16) || 0x83) & 0xFF).toString(16).toUpperCase().padStart(2, '0') });
                rows.push({ label: '  bmAttributes:', value: '0x03  (Interrupt)' });
                rows.push({ label: '  wMaxPacketSize:', value: String(acEpPkt) + ' bytes' });
                rows.push({ label: '  bInterval:', value: String(acEpIntvl) + (isHS ? ' (×125 µs)' : ' ms') });
                rows.push({ label: '', value: '' });
            }
            rows.push({ label: 'Audio Streaming Interface (Alt 0, zero bandwidth):', value: '' });
            const audioAsIInterface = cfg.audioAsIInterface || 0;
            ifRow(asIf, 0x00, 0, 0x01, asSubClass, 0x00, audioAsIInterface);
            if (audioAsIInterface > 0) rows.push({ label: '  iInterface:', value: String(audioAsIInterface) + '  ("' + (cfg.audioAsInterfaceString || '') + '")' });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Audio Streaming Interface (Alt 1, operational):', value: '' });
            ifRow(asIf, 0x01, 1, 0x01, asSubClass, 0x00, audioAsIInterface);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'AS CS Interface General Descriptor:', value: '' });
            rows.push({ label: '  bTerminalLink:', value: String(terminalLink) });
            rows.push({ label: '  wFormatTag:', value: '0x' + formatTag.toString(16).toUpperCase().padStart(4, '0') });
            rows.push({ label: '', value: '' });
            const fmtTypeNames = { 0x01: 'Type I', 0x02: 'Type II', 0x03: 'Type III' };
            rows.push({ label: 'AS Format Type Descriptor:', value: '' });
            rows.push({ label: '  bFormatType:', value: toHex(formatType, 1) + ' (' + (fmtTypeNames[formatType] || 'Unknown') + ')' });
            if (formatType === 0x02) {
                rows.push({ label: '  wMaxBitRate:', value: String(maxBitRate) + ' kbit/s' });
                rows.push({ label: '  wSamplesPerFrame:', value: String(samplesPerFrame) });
            } else {
                rows.push({ label: '  bNrChannels:', value: String(numChannels) });
                const chanCfgHex = '0x' + channelConfig.toString(16).toUpperCase().padStart(4, '0');
                const chanCfgLabel = CHANNEL_CONFIG_LABELS[Math.max(1, Math.min(8, numChannels))] || '';
                rows.push({ label: '  wChannelConfig:', value: chanCfgHex + (chanCfgLabel ? '  (' + chanCfgLabel + ')' : '') });
                rows.push({ label: '  bSubframeSize:', value: toHex(subframeSize, 1) });
                rows.push({ label: '  bBitResolution:', value: String(bitRes) });
            }
            rows.push({ label: '  tSamFreq:', value: sampleRates.map((r) => r + ' Hz').join(', ') });

            const _audioPktNum = parseInt(epVals[`audioMaxPacketSize${sfx}`], 10) || 192;
            const _audioPktHex = '0x' + _audioPktNum.toString(16).toUpperCase().padStart(4, '0');
            const audioSyncTypeSum = parseInt(epVals.audioSyncType, 16) || 0x01;
            const hasFeedbackEpSum = audioSyncTypeSum === 0x05;
            const AUDIO_SYNC_NAMES = { 0x01: 'No Sync', 0x05: 'Asynchronous', 0x09: 'Adaptive', 0x0D: 'Synchronous' };
            const syncNameSum = AUDIO_SYNC_NAMES[audioSyncTypeSum] || 'Isochronous';
            const fbEpAddrSumRaw = hasFeedbackEpSum ? parseHexValue(epVals[`audioFbEpAddress${sfx}`] || '0x01', 1, 'addr') : 0x00;
            isoEpFromHex('Isochronous Endpoint Descriptor:', epVals[`audioEndpointAddress${sfx}`], _audioPktHex, epVals[`audioInterval${sfx}`], '0x81', defPkt, '1', audioSyncTypeSum, syncNameSum, hasFeedbackEpSum ? fbEpAddrSumRaw : 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CS Isochronous Endpoint Descriptor (Audio):', value: '' });
            rows.push({ label: '  bLength:', value: '7 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x25 (CS_ENDPOINT)' });
            rows.push({ label: '  bDescriptorSubtype:', value: '0x01' });
            if (hasFeedbackEpSum) {
                const fbPktSum   = isHS ? (epVals.audioFbEpMaxPacketHS || 4) : (epVals.audioFbEpMaxPacketFS || 3);
                const fbIntvlSum = isHS ? (epVals.audioFbEpIntervalHS  || 4) : (epVals.audioFbEpIntervalFS  || 1);
                rows.push({ label: '', value: '' });
                rows.push({ label: 'Feedback Endpoint Descriptor:', value: '' });
                rows.push({ label: '  bLength:', value: '9 bytes' });
                rows.push({ label: '  bDescriptorType:', value: '0x05 (Endpoint)' });
                rows.push({ label: '  bEndpointAddress:', value: toHex(fbEpAddrSumRaw, 1) });
                rows.push({ label: '  bmAttributes:', value: '0x11 (Isochronous, Feedback)' });
                rows.push({ label: '  wMaxPacketSize:', value: String(fbPktSum) + ' bytes' });
                rows.push({ label: '  bInterval:', value: String(fbIntvlSum) });
                rows.push({ label: '  bRefresh:', value: '0x00' });
                rows.push({ label: '  bSynchAddress:', value: '0x00' });
            }
        }
        return;
    }

    if (className === 'audio2') {
        for (let i = 1; i <= n; i++) {
            const cfg   = readAudio2ConfigValues(i);
            const epVals = isHS ? readAudio2EndpointValuesHS(i) : readAudio2EndpointValuesFS(i);
            const acSubClass = parseHexValue(cfg.audio20ControlSubClass   || '0x01', 1, 'audio20ControlSubClass');
            const asSubClass = parseHexValue(cfg.audio20StreamingSubClass || '0x02', 1, 'audio20StreamingSubClass');
            const clockSrcId  = parseNumberValue(cfg.audio20ClockSourceId || 16, { id: 'audio20ClockSourceId', min: 1, max: 255, default: 16 });
            const terminalLink = parseNumberValue(cfg.audio20TerminalLink || 1, { id: 'audio20TerminalLink', min: 1, max: 255, default: 1 });
            const numChannels  = parseNumberValue(cfg.audio20NumChannels  || 2, { id: 'audio20NumChannels', min: 1, max: 8, default: 2 });
            const subslotSize  = parseHexValue(cfg.audio20SubslotSize     || '0x02', 1, 'audio20SubslotSize');
            const bitRes       = parseNumberValue(cfg.audio20BitResolution || 16, { id: 'audio20BitResolution', min: 1, max: 32, default: 16 });
            const acIf = ifOffset + (i - 1) * 2;
            const asIf = acIf + 1;
            const sfx  = isHS ? 'HS' : 'FS';
            const defPkt = '0x00C0';

            rows.push({ label: '', value: '' });
            rows.push({ label: `═══ Audio 2.0 Instance ${i}${isHS ? ' (High Speed)' : ''} ═══`, value: '', isHeader: true });
            iadRow(acIf, 2, 0x01, acSubClass, 0x20);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Audio Control Interface Descriptor:', value: '' });
            ifRow(acIf, 0x00, 0, 0x01, acSubClass, 0x20, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'AC CS Interface Header Descriptor:', value: '' });
            rows.push({ label: '  bcdADC:', value: '0x0200' });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Clock Source Descriptor:', value: '' });
            rows.push({ label: '  bClockID:', value: String(clockSrcId) });
            rows.push({ label: '  bmAttributes:', value: '0x03 (Internal, SoF-synced)' });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Input Terminal Descriptor:', value: '' });
            rows.push({ label: '  bTerminalID:', value: String(terminalLink) });
            rows.push({ label: '  wTerminalType:', value: '0x0101 (USB streaming)' });
            rows.push({ label: '  bCSourceID:', value: String(clockSrcId) });
            rows.push({ label: '  bNrChannels:', value: String(numChannels) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Output Terminal Descriptor:', value: '' });
            rows.push({ label: '  bTerminalID:', value: String(terminalLink + 1) });
            rows.push({ label: '  wTerminalType:', value: '0x0301 (Speaker)' });
            rows.push({ label: '  bCSourceID:', value: String(clockSrcId) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Audio Streaming Interface (Alt 0, zero bandwidth):', value: '' });
            ifRow(asIf, 0x00, 0, 0x01, asSubClass, 0x20, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'Audio Streaming Interface (Alt 1, operational):', value: '' });
            ifRow(asIf, 0x01, 1, 0x01, asSubClass, 0x20, 0x00);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'AS CS Interface General Descriptor:', value: '' });
            rows.push({ label: '  bTerminalLink:', value: String(terminalLink) });
            rows.push({ label: '', value: '' });
            rows.push({ label: 'AS Type II Format Descriptor:', value: '' });
            rows.push({ label: '  bSubslotSize:', value: toHex(subslotSize, 1) });
            rows.push({ label: '  bBitResolution:', value: String(bitRes) });

            isoEpFromHex('Isochronous Endpoint Descriptor:', epVals[`audio2EndpointAddress${sfx}`], epVals[`audio2MaxPacketSize${sfx}`], epVals[`audio2Interval${sfx}`], '0x81', defPkt, '1', 0x05);
            rows.push({ label: '', value: '' });
            rows.push({ label: 'CS Isochronous Endpoint Descriptor (Audio 2.0):', value: '' });
            rows.push({ label: '  bLength:', value: '8 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x25 (CS_ENDPOINT)' });
            rows.push({ label: '  bDescriptorSubtype:', value: '0x01' });
        }
        return;
    }
}

// Returns the interface offset for a class in the current class order.
function getReadableClassIfOffset(className) {
    const classIPR = { hid: 1, msc: 1, dfu: 1, printer: 1, video: 1, mtp: 1, ptp: 1, cdc: 2, rndis: 2, ecm: 2, audio: 2, audio2: 2 };
    let offset = 0;
    for (const cn of getCurrentClassOrder()) {
        if (cn === className) break;
        const cb = document.getElementById(`class-${cn}`);
        if (cb && cb.checked) {
            const inp = document.getElementById(`class-num-${cn}`);
            offset += Math.max(1, parseInt(inp && inp.value, 10) || 1) * (classIPR[cn] || 1);
        }
    }
    return offset;
}

function renderReadable(values, configValues, fullDescriptor) {
    const usbVersion = PAGE_SCHEMA.readableMaps.usbVersions[values.bcdUSB] || 'Custom';
    const deviceClass = PAGE_SCHEMA.readableMaps.deviceClasses[values.bDeviceClass] || 'Unknown';
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    const fsSize = fullDescriptor ? fullDescriptor.length : 0;

    const rows = [
        { label: `device_framework_full_speed  (${fsSize} bytes)`, value: '', isHeader: true },
        { label: '', value: '' },
        { label: '═══ Device Descriptor ═══', value: '', isHeader: true },
        { label: 'Descriptor Length:', value: '18 bytes (Device Descriptor)' },
        { label: 'Descriptor Type:', value: '0x01 (Device)' },
        { label: 'USB Version:', value: 'USB ' + usbVersion + ' (' + toHex(values.bcdUSB, 2) + ')' },
        { label: 'Device Class:', value: toHex(values.bDeviceClass, 1) + ' (' + deviceClass + ')' },
        { label: 'Device SubClass:', value: toHex(values.bDeviceSubClass, 1) },
        { label: 'Device Protocol:', value: toHex(values.bDeviceProtocol, 1) },
        { label: 'Max Packet Size (EP0):', value: String(values.bMaxPacketSize0) + ' bytes' },
        { label: 'Vendor ID:', value: toHex(values.idVendor, 2) },
        { label: 'Product ID:', value: toHex(values.idProduct, 2) },
        { label: 'Device Release:', value: toHex(values.bcdDevice, 2) },
        { label: 'Manufacturer String:', value: 'Index ' + values.iManufacturer },
        { label: 'Product String:', value: 'Index ' + values.iProduct },
        { label: 'Serial Number String:', value: 'Index ' + values.iSerialNumber },
        { label: 'Num Configurations:', value: String(values.bNumConfigurations) }
    ];

    // Add Configuration Descriptor info
    if (configValues) {
        rows.push({ label: '', value: '' });
        rows.push({ label: '═══ Configuration Descriptor ═══', value: '', isHeader: true });
        rows.push({ label: 'Descriptor Length:', value: '9 bytes (Configuration Descriptor)' });
        rows.push({ label: 'Descriptor Type:', value: '0x02 (Configuration)' });
        rows.push({ label: 'Total Length:', value: toHex(configValues.wTotalLength, 2) + ' (' + configValues.wTotalLength + ' bytes)' });
        rows.push({ label: 'Num Interfaces:', value: String(configValues.bNumInterfaces) });
        rows.push({ label: 'Configuration Value:', value: '0x01 (fixed)' });
        rows.push({ label: 'Configuration String:', value: 'Index ' + configValues.iConfiguration });
        rows.push({ label: 'Attributes:', value: toHex(configValues.bmAttributes, 1) + ' (' + (configValues.bmAttributes & 0x40 ? 'Self-powered' : 'Bus-powered') + ')' });
        rows.push({ label: 'Max Power:', value: String(configValues.bMaxPower) + ' units (' + (configValues.bMaxPower * 2) + 'mA)' });
    }

    // Add class descriptors (FS) in user-defined class order
    for (const className of getCurrentClassOrder()) {
        pushReadableClassRows(rows, className, getReadableClassIfOffset(className), false);
    }

    // device_framework_high_speed section
    if (isHighSpeedEnabled) {
        const hsResult = buildHighSpeedDescriptor();
        if (hsResult && hsResult.descriptor && hsResult.descriptor.length > 0) {
            const hsBytes = hsResult.descriptor;
            const hsPacketSize = values.bMaxPacketSize0HS || values.bMaxPacketSize0;
            const hsConfigWTotal = hsBytes.length > 31 ? (hsBytes[30] | (hsBytes[31] << 8)) : 0;

            rows.push({ label: '', value: '' });
            rows.push({ label: `device_framework_high_speed  (${hsBytes.length} bytes)`, value: '', isHeader: true });

            rows.push({ label: '', value: '' });
            rows.push({ label: '═══ Device Descriptor (High Speed) ═══', value: '', isHeader: true });
            rows.push({ label: '  bLength:', value: '18 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x01 (Device)' });
            rows.push({ label: '  bcdUSB:', value: 'USB ' + usbVersion + ' (' + toHex(values.bcdUSB, 2) + ')' });
            rows.push({ label: '  bDeviceClass:', value: toHex(values.bDeviceClass, 1) + ' (' + deviceClass + ')' });
            rows.push({ label: '  bDeviceSubClass:', value: toHex(values.bDeviceSubClass, 1) });
            rows.push({ label: '  bDeviceProtocol:', value: toHex(values.bDeviceProtocol, 1) });
            rows.push({ label: '  bMaxPacketSize0:', value: String(hsPacketSize) + ' bytes' });
            rows.push({ label: '  idVendor:', value: toHex(values.idVendor, 2) });
            rows.push({ label: '  idProduct:', value: toHex(values.idProduct, 2) });
            rows.push({ label: '  bcdDevice:', value: toHex(values.bcdDevice, 2) });
            rows.push({ label: '  iManufacturer:', value: 'Index ' + values.iManufacturer });
            rows.push({ label: '  iProduct:', value: 'Index ' + values.iProduct });
            rows.push({ label: '  iSerialNumber:', value: 'Index ' + values.iSerialNumber });
            rows.push({ label: '  bNumConfigurations:', value: String(values.bNumConfigurations) });

            rows.push({ label: '', value: '' });
            rows.push({ label: '═══ Device Qualifier Descriptor ═══', value: '', isHeader: true });
            rows.push({ label: '  bLength:', value: '10 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x06 (Device Qualifier)' });
            rows.push({ label: '  bcdUSB:', value: '0x0200 (USB 2.0)' });
            rows.push({ label: '  bDeviceClass:', value: '0x00' });
            rows.push({ label: '  bDeviceSubClass:', value: '0x00' });
            rows.push({ label: '  bDeviceProtocol:', value: '0x00' });
            rows.push({ label: '  bMaxPacketSize0:', value: String(hsPacketSize) + ' bytes' });
            rows.push({ label: '  bNumConfigurations:', value: '1' });
            rows.push({ label: '  bReserved:', value: '0x00' });

            rows.push({ label: '', value: '' });
            rows.push({ label: '═══ Configuration Descriptor (High Speed) ═══', value: '', isHeader: true });
            rows.push({ label: '  bLength:', value: '9 bytes' });
            rows.push({ label: '  bDescriptorType:', value: '0x02 (Configuration)' });
            rows.push({ label: '  wTotalLength:', value: toHex(hsConfigWTotal, 2) + ' (' + hsConfigWTotal + ' bytes)' });
            rows.push({ label: '  bNumInterfaces:', value: String(configValues.bNumInterfaces) });
            rows.push({ label: '  bConfigurationValue:', value: '0x01 (fixed)' });
            rows.push({ label: '  iConfiguration:', value: 'Index ' + (configValues.iConfigurationHS || 0) });
            rows.push({ label: '  bmAttributes:', value: toHex(configValues.bmAttributes, 1) + ' (' + (configValues.bmAttributes & 0x40 ? 'Self-powered' : 'Bus-powered') + ')' });
            rows.push({ label: '  bMaxPower:', value: String(configValues.bMaxPower) + ' units (' + (configValues.bMaxPower * 2) + 'mA)' });

            // All HS class instances in user-defined order
            for (const className of getCurrentClassOrder()) {
                pushReadableClassRows(rows, className, getReadableClassIfOffset(className), true);
            }
        }
    }

    // Add String Descriptors section
    const activeStrings = buildActiveStringDescriptors(values, configValues);
    const strBytes = buildStringFrameworkBytes(values, configValues);
    if (activeStrings.length > 0) {
        rows.push({ label: '', value: '' });
        rows.push({ label: `device_framework_string  (${strBytes.length} bytes)`, value: '', isHeader: true });

        const enabledLangs = getEnabledStringLanguages();
        const primaryLang = getPrimaryStringLanguage();
        const displayLangs = enabledLangs.length > 0 ? enabledLangs : [primaryLang];
        rows.push({ label: '', value: '' });
        rows.push({ label: '═══ String Descriptors ═══', value: '', isHeader: true });
        rows.push({ label: '  Language IDs:', value: displayLangs.map((l) => `0x${l.code.toString(16).toUpperCase().padStart(4, '0')} (${l.name})`).join(', ') });

        activeStrings.forEach((desc) => {
            const lang = desc.languageGroup
                ? getSelectedLanguageForGroup(desc.languageGroup)
                : (desc.languageSelectorId
                    ? getSelectedLanguageForSelector(desc.languageSelectorId)
                    : getPrimaryStringLanguage());
            rows.push({ label: `  [${desc.index}] ${desc.name}:`, value: `"${desc.text}"  (lang ${toHex(lang.code, 2)})` });
        });
    }

    // Add Language ID section
    const langIdBytes = buildLanguageIdBytes();
    if (langIdBytes.length > 0) {
        rows.push({ label: '', value: '' });
        rows.push({ label: `device_framework_language_id  (${langIdBytes.length} bytes)`, value: '', isHeader: true });
        rows.push({ label: '', value: '' });
        rows.push({ label: '═══ Language ID Descriptor ═══', value: '', isHeader: true });
        for (let i = 0; i < langIdBytes.length; i += 2) {
            const code = langIdBytes[i] | (langIdBytes[i + 1] << 8);
            const lang = STRING_LANGUAGE_OPTIONS.find((l) => l.code === code);
            const name = lang ? lang.name : 'Unknown';
            rows.push({ label: `  Language ID ${i / 2 + 1}:`, value: `0x${code.toString(16).toUpperCase().padStart(4, '0')} (${name})` });
        }
    }

    // Summary
    rows.push({ label: '', value: '' });
    rows.push({ label: '═══ Summary ═══', value: '', isHeader: true });
    if (fullDescriptor && fullDescriptor.length > 0) {
        rows.push({ label: '  device_framework_full_speed:', value: fullDescriptor.length + ' bytes' });
    }
    if (isHighSpeedEnabled) {
        const hsRes = buildHighSpeedDescriptor();
        if (hsRes && hsRes.descriptor) rows.push({ label: '  device_framework_high_speed:', value: hsRes.descriptor.length + ' bytes' });
    }
    if (strBytes.length > 0) rows.push({ label: '  device_framework_string:', value: strBytes.length + ' bytes' });
    if (langIdBytes.length > 0) rows.push({ label: '  device_framework_language_id:', value: langIdBytes.length + ' bytes' });

    document.getElementById('readable-output').innerHTML = templates.readable({ rows: rows });
}

function generateDescriptor() {
    try {
        const values = readFormValues();
        const configValues = readConfigurationValues();
        validateUniqueStringIndexes(values, configValues);
        let descriptor = buildDescriptor(values);
        let commentsList = PAGE_SCHEMA.descriptorComments.slice(); // Copy base comments

        // Add Configuration Descriptor
        const configDescriptor = buildConfigDescriptor(configValues);
        descriptor = descriptor.concat(configDescriptor);

        // Add comments for Configuration descriptor
        const configComments = [
            'bLength',
            'bDescriptorType',
            'wTotalLength LSB',
            'wTotalLength MSB',
            'bNumInterfaces',
            'bConfigurationValue',
            'iConfiguration',
            'bmAttributes',
            'bMaxPower'
        ];
        configComments.forEach((comment) => {
            commentsList.push(`[CONFIG] ${comment}`);
        });

        const classInterfacePerInstance = {
            hid: 1,
            msc: 1,
            dfu: 1,
            printer: 1,
            video: 1,
            mtp: 1,
            ptp: 1,
            cdc: 2,
            rndis: 2,
            ecm: 2,
            audio: 2,
            audio2: 2
        };

        const getEnabledClassInstances = (className) => {
            const checkbox = document.getElementById(`class-${className}`);
            if (!checkbox || !checkbox.checked) {
                return 0;
            }

            const countInput = document.getElementById(`class-num-${className}`);
            return Math.max(1, parseInt(countInput && countInput.value, 10) || 1);
        };

        const getClassInterfaceOffset = (targetClassName) => {
            let offset = 0;
            for (const className of getCurrentClassOrder()) {
                if (className === targetClassName) {
                    break;
                }

                offset += getEnabledClassInstances(className) * (classInterfacePerInstance[className] || 1);
            }

            return offset;
        };

        const classSegments = new Map();
        const beginClassSegment = () => ({ descriptorStart: descriptor.length, commentsStart: commentsList.length });
        const finalizeClassSegment = (className, segmentStart) => {
            const descriptorPart = descriptor.slice(segmentStart.descriptorStart);
            const commentsPart = commentsList.slice(segmentStart.commentsStart);

            descriptor.length = segmentStart.descriptorStart;
            commentsList.length = segmentStart.commentsStart;

            if (descriptorPart.length > 0) {
                classSegments.set(className, { descriptorPart, commentsPart });
            }
        };

        const hidSegmentStart = beginClassSegment();
        // Generate Interface and HID descriptors if HID is enabled and append to device descriptor
        const hidCheckbox = document.getElementById('class-hid');
        if (hidCheckbox && hidCheckbox.checked) {
            const hidNumberInput = document.getElementById('class-num-hid');
            const classNumber = parseInt(hidNumberInput.value) || 1;

            for (let i = 1; i <= classNumber; i++) {
                // Read form values from the three descriptor types
                const interfaceValues = readInterfaceDescriptorValues(i);
                interfaceValues.bInterfaceNumber = getClassInterfaceOffset('hid') + (i - 1);
                const hidDetailsValues = readHIDDetailsValues(i);
                const endpointValues = readEndpointInDescriptorValues(i);
                const endpointOutValues = optionalEndpointState[i] && isProtocolNone(i) ? readEndpointOutDescriptorValues(i) : null;

                if (endpointOutValues && Object.keys(endpointOutValues).length > 0) {
                    interfaceValues.bNumEndpoints = 2;
                }

                // Only generate if at least one set of values was provided
                if (Object.keys(interfaceValues).length > 0 || Object.keys(hidDetailsValues).length > 0 || Object.keys(endpointValues).length > 0) {
                    // Add Interface Descriptor
                    if (Object.keys(interfaceValues).length > 0) {
                        const interfaceDescriptor = buildInterfaceDescriptorFromValues(interfaceValues);
                        descriptor = descriptor.concat(interfaceDescriptor);

                        // Add comments for Interface descriptor
                        const interfaceComments = [
                            'bLength',
                            'bDescriptorType',
                            'bInterfaceNumber',
                            'bAlternateSetting',
                            'bNumEndpoints',
                            'bInterfaceClass',
                            'bInterfaceSubClass',
                            'bInterfaceProtocol',
                            'iInterface'
                        ];
                        interfaceComments.forEach((comment) => {
                            commentsList.push(`[IF-${i}] ${comment}`);
                        });
                    }

                    // Add HID Descriptor
                    if (Object.keys(hidDetailsValues).length > 0) {
                        const hidDescriptor = buildHIDDetailsDescriptorFromValues(hidDetailsValues);
                        descriptor = descriptor.concat(hidDescriptor);

                        // Add comments for HID descriptor
                        const hidComments = [
                            'bLength',
                            'bDescriptorType',
                            'bcdHID LSB',
                            'bcdHID MSB',
                            'bCountryCode',
                            'bNumDescriptors',
                            'bReportDescriptorType',
                            'wReportDescriptorLength LSB',
                            'wReportDescriptorLength MSB'
                        ];
                        hidComments.forEach((comment) => {
                            commentsList.push(`[HID-${i}] ${comment}`);
                        });
                    }

                    // Add Endpoint IN Descriptor
                    if (Object.keys(endpointValues).length > 0) {
                        const endpointDescriptor = buildEndpointInDescriptorFromValues(endpointValues);
                        descriptor = descriptor.concat(endpointDescriptor);

                        const endpointComments = [
                            'bLength',
                            'bDescriptorType',
                            'bEndpointAddress',
                            'bmAttributes',
                            'wMaxPacketSize LSB',
                            'wMaxPacketSize MSB',
                            'bInterval'
                        ];
                        endpointComments.forEach((comment) => {
                            commentsList.push(`[EP-${i}] ${comment}`);
                        });
                    }

                    if (endpointOutValues && Object.keys(endpointOutValues).length > 0) {
                        const endpointOutDescriptor = buildEndpointInDescriptorFromValues(endpointOutValues);
                        descriptor = descriptor.concat(endpointOutDescriptor);

                        const endpointOutComments = [
                            'bLength',
                            'bDescriptorType',
                            'bEndpointAddress',
                            'bmAttributes',
                            'wMaxPacketSize LSB',
                            'wMaxPacketSize MSB',
                            'bInterval'
                        ];
                        endpointOutComments.forEach((comment) => {
                            commentsList.push(`[EP-OUT-${i}] ${comment}`);
                        });
                    }
                }
            }
        }
        finalizeClassSegment('hid', hidSegmentStart);

        const mscSegmentStart = beginClassSegment();
        // Generate Mass Storage descriptors if MSC is enabled and append to device descriptor
        const mscCheckbox = document.getElementById('class-msc');
        if (mscCheckbox && mscCheckbox.checked) {
            const mscNumberInput = document.getElementById('class-num-msc');
            const classNumber = parseInt(mscNumberInput.value) || 1;

            for (let i = 1; i <= classNumber; i++) {
                // Read Mass Storage configuration values
                const mscConfigValues = readMassStorageConfigValues(i);
                const bulkInValuesFS = readMassStorageBulkInValuesFS(i);
                const bulkOutValuesFS = readMassStorageBulkOutValuesFS(i);

                // Build Interface Descriptor for Mass Storage
                const interfaceDescriptor = [
                    0x09,  // bLength
                    0x04,  // bDescriptorType (Interface)
                    getClassInterfaceOffset('msc') + (i - 1), // bInterfaceNumber
                    0x00,  // bAlternateSetting
                    0x02,  // bNumEndpoints (Bulk IN + Bulk OUT)
                    0x08,  // bInterfaceClass (Mass Storage)
                    parseHexValue(mscConfigValues.mscBInterfaceSubClass || '0x06', 1, 'mscBInterfaceSubClass'),  // bInterfaceSubClass
                    parseHexValue(mscConfigValues.mscBInterfaceProtocol || '0x50', 1, 'mscBInterfaceProtocol'),  // bInterfaceProtocol
                    0x00   // iInterface
                ];
                descriptor = descriptor.concat(interfaceDescriptor);

                // Add comments for Interface descriptor
                const interfaceComments = [
                    'bLength',
                    'bDescriptorType',
                    'bInterfaceNumber',
                    'bAlternateSetting',
                    'bNumEndpoints',
                    'bInterfaceClass',
                    'bInterfaceSubClass',
                    'bInterfaceProtocol',
                    'iInterface'
                ];
                interfaceComments.forEach((comment) => {
                    commentsList.push(`[MSC-IF-${i}] ${comment}`);
                });

                // Build Bulk IN Endpoint Descriptor
                const bulkInEndpoint = parseHexValue(bulkInValuesFS.mscBulkInEndpointFS || '0x81', 1, 'bulkInEndpoint');
                const bulkInMaxPacket = parseHexValue(bulkInValuesFS.mscBulkInMaxPacketSizeFS || '0x0040', 2, 'bulkInMaxPacket');
                const bulkInInterval = parseHexValue(bulkInValuesFS.mscBulkInIntervalFS || '0', 1, 'bulkInInterval');
                const bulkInDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkInEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkInMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkInMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkInInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkInDescriptor);

                const bulkInComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkInComments.forEach((comment) => {
                    commentsList.push(`[MSC-EP-IN-${i}] ${comment}`);
                });

                // Build Bulk OUT Endpoint Descriptor
                const bulkOutEndpoint = parseHexValue(bulkOutValuesFS.mscBulkOutEndpointFS || '0x01', 1, 'bulkOutEndpoint');
                const bulkOutMaxPacket = parseHexValue(bulkOutValuesFS.mscBulkOutMaxPacketSizeFS || '0x0040', 2, 'bulkOutMaxPacket');
                const bulkOutInterval = parseHexValue(bulkOutValuesFS.mscBulkOutIntervalFS || '0', 1, 'bulkOutInterval');
                const bulkOutDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkOutEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkOutMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkOutMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkOutInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkOutDescriptor);

                const bulkOutComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkOutComments.forEach((comment) => {
                    commentsList.push(`[MSC-EP-OUT-${i}] ${comment}`);
                });
            }
        }
        finalizeClassSegment('msc', mscSegmentStart);

        const dfuSegmentStart = beginClassSegment();
        // Generate DFU descriptors if DFU is enabled and append to device descriptor
        const dfuCheckbox = document.getElementById('class-dfu');
        if (dfuCheckbox && dfuCheckbox.checked) {
            const dfuNumberInput = document.getElementById('class-num-dfu');
            const classNumber = parseInt(dfuNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('dfu');

            for (let i = 1; i <= classNumber; i++) {
                const dfuConfigValues = readDFUConfigValues(i);

                const interfaceDescriptor = [
                    0x09,
                    0x04,
                    interfaceOffset + (i - 1),
                    0x00,
                    0x00,
                    0xFE,
                    parseHexValue(dfuConfigValues.dfuBInterfaceSubClass || '0x01', 1, 'dfuBInterfaceSubClass'),
                    parseHexValue(dfuConfigValues.dfuBInterfaceProtocol || '0x02', 1, 'dfuBInterfaceProtocol'),
                    parseNumberValue(dfuConfigValues.dfuIInterface || 0, { id: 'dfuIInterface', min: 0, max: 255, default: 0 })
                ];
                descriptor = descriptor.concat(interfaceDescriptor);

                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[DFU-IF-${i}] ${comment}`));

                const bcdDFUVersion = parseHexValue(dfuConfigValues.dfuBcdVersion || '0x0110', 2, 'dfuBcdVersion');
                const detachTimeout = parseNumberValue(dfuConfigValues.dfuDetachTimeout || 1000, { id: 'dfuDetachTimeout', min: 0, max: 65535, default: 1000 });
                const transferSize = parseNumberValue(dfuConfigValues.dfuTransferSize || 1024, { id: 'dfuTransferSize', min: 1, max: 65535, default: 1024 });
                const functionalDescriptor = [
                    0x09,
                    0x21,
                    parseHexValue(dfuConfigValues.dfuBmAttributes || '0x0B', 1, 'dfuBmAttributes'),
                    detachTimeout & 0xFF,
                    (detachTimeout >> 8) & 0xFF,
                    transferSize & 0xFF,
                    (transferSize >> 8) & 0xFF,
                    bcdDFUVersion & 0xFF,
                    (bcdDFUVersion >> 8) & 0xFF
                ];
                descriptor = descriptor.concat(functionalDescriptor);

                ['bLength', 'bDescriptorType', 'bmAttributes', 'wDetachTimeOut LSB', 'wDetachTimeOut MSB', 'wTransferSize LSB', 'wTransferSize MSB', 'bcdDFUVersion LSB', 'bcdDFUVersion MSB']
                    .forEach((comment) => commentsList.push(`[DFU-FUNC-${i}] ${comment}`));
            }
        }
            finalizeClassSegment('dfu', dfuSegmentStart);

        const printerSegmentStart = beginClassSegment();
        // Generate Printer descriptors if Printer is enabled and append to device descriptor
        const printerCheckbox = document.getElementById('class-printer');
        if (printerCheckbox && printerCheckbox.checked) {
            const printerNumberInput = document.getElementById('class-num-printer');
            const classNumber = parseInt(printerNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('printer');

            for (let i = 1; i <= classNumber; i++) {
                const printerConfigValues = readPrinterConfigValues(i);
                const bulkInValuesFS = readPrinterBulkInValuesFS(i);
                const bulkOutValuesFS = readPrinterBulkOutValuesFS(i);

                const interfaceDescriptor = [
                    0x09,
                    0x04,
                    interfaceOffset + (i - 1),
                    0x00,
                    0x02,
                    0x07,
                    parseHexValue(printerConfigValues.printerBInterfaceSubClass || '0x01', 1, 'printerBInterfaceSubClass'),
                    parseHexValue(printerConfigValues.printerBInterfaceProtocol || '0x02', 1, 'printerBInterfaceProtocol'),
                    parseNumberValue(printerConfigValues.printerIInterface || 0, { id: 'printerIInterface', min: 0, max: 255, default: 0 })
                ];
                descriptor = descriptor.concat(interfaceDescriptor);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[PRN-IF-${i}] ${comment}`));

                const bulkInEndpoint = parseHexValue(bulkInValuesFS.printerBulkInEndpointFS || '0x81', 1, 'printerBulkInEndpointFS');
                const bulkInMaxPacket = parseHexValue(bulkInValuesFS.printerBulkInMaxPacketSizeFS || '0x0040', 2, 'printerBulkInMaxPacketSizeFS');
                const bulkInInterval = parseHexValue(bulkInValuesFS.printerBulkInIntervalFS || '0', 1, 'printerBulkInIntervalFS');
                const bulkInDescriptor = [0x07, 0x05, bulkInEndpoint, 0x02, bulkInMaxPacket & 0xFF, (bulkInMaxPacket >> 8) & 0xFF, bulkInInterval];
                descriptor = descriptor.concat(bulkInDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
                    .forEach((comment) => commentsList.push(`[PRN-EP-IN-${i}] ${comment}`));

                const bulkOutEndpoint = parseHexValue(bulkOutValuesFS.printerBulkOutEndpointFS || '0x01', 1, 'printerBulkOutEndpointFS');
                const bulkOutMaxPacket = parseHexValue(bulkOutValuesFS.printerBulkOutMaxPacketSizeFS || '0x0040', 2, 'printerBulkOutMaxPacketSizeFS');
                const bulkOutInterval = parseHexValue(bulkOutValuesFS.printerBulkOutIntervalFS || '0', 1, 'printerBulkOutIntervalFS');
                const bulkOutDescriptor = [0x07, 0x05, bulkOutEndpoint, 0x02, bulkOutMaxPacket & 0xFF, (bulkOutMaxPacket >> 8) & 0xFF, bulkOutInterval];
                descriptor = descriptor.concat(bulkOutDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
                    .forEach((comment) => commentsList.push(`[PRN-EP-OUT-${i}] ${comment}`));
            }
        }
            finalizeClassSegment('printer', printerSegmentStart);

        const videoSegmentStart = beginClassSegment();
        // Generate Video descriptors if Video is enabled and append to device descriptor
        const videoCheckbox = document.getElementById('class-video');
        if (videoCheckbox && videoCheckbox.checked) {
            const videoNumberInput = document.getElementById('class-num-video');
            const classNumber = parseInt(videoNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('video');

            for (let i = 1; i <= classNumber; i++) {
                const videoConfigValues = readVideoConfigValues(i);
                const bulkInValuesFS = readVideoBulkInValuesFS(i);
                const bulkOutValuesFS = readVideoBulkOutValuesFS(i);

                appendVideoDescriptorsFromValues(descriptor, commentsList, {
                    instanceIndex: i,
                    interfaceNumber: interfaceOffset + (i - 1),
                    configValues: videoConfigValues,
                    bulkInValues: bulkInValuesFS,
                    bulkOutValues: bulkOutValuesFS,
                    inEndpointKey: 'videoBulkInEndpointFS',
                    inPacketKey: 'videoBulkInMaxPacketSizeFS',
                    inIntervalKey: 'videoBulkInIntervalFS',
                    outEndpointKey: 'videoBulkOutEndpointFS',
                    outPacketKey: 'videoBulkOutMaxPacketSizeFS',
                    outIntervalKey: 'videoBulkOutIntervalFS',
                    defaultInEndpoint: '0x83',
                    defaultOutEndpoint: '0x03',
                    defaultPacket: '0x0040',
                    defaultInterval: '0'
                });
            }
        }
        finalizeClassSegment('video', videoSegmentStart);

        const mtpSegmentStart = beginClassSegment();
        // Generate MTP descriptors if MTP is enabled and append to device descriptor
        const mtpCheckbox = document.getElementById('class-mtp');
        if (mtpCheckbox && mtpCheckbox.checked) {
            const mtpNumberInput = document.getElementById('class-num-mtp');
            const classNumber = parseInt(mtpNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('mtp');

            for (let i = 1; i <= classNumber; i++) {
                const mtpConfigValues = readMTPConfigValues(i);
                const bulkInValuesFS = readMTPBulkInValuesFS(i);
                const bulkOutValuesFS = readMTPBulkOutValuesFS(i);

                appendMTPDescriptorsFromValues(descriptor, commentsList, {
                    instanceIndex: i,
                    interfaceNumber: interfaceOffset + (i - 1),
                    configValues: mtpConfigValues,
                    bulkInValues: bulkInValuesFS,
                    bulkOutValues: bulkOutValuesFS,
                    inEndpointKey: 'mtpBulkInEndpointFS',
                    inPacketKey: 'mtpBulkInMaxPacketSizeFS',
                    inIntervalKey: 'mtpBulkInIntervalFS',
                    outEndpointKey: 'mtpBulkOutEndpointFS',
                    outPacketKey: 'mtpBulkOutMaxPacketSizeFS',
                    outIntervalKey: 'mtpBulkOutIntervalFS',
                    defaultInEndpoint: '0x84',
                    defaultOutEndpoint: '0x04',
                    defaultPacket: '0x0040',
                    defaultInterval: '0'
                });
            }
        }
        finalizeClassSegment('mtp', mtpSegmentStart);

        const ptpSegmentStart = beginClassSegment();
        // Generate PTP descriptors if PTP is enabled and append to device descriptor
        const ptpCheckbox = document.getElementById('class-ptp');
        if (ptpCheckbox && ptpCheckbox.checked) {
            const ptpNumberInput = document.getElementById('class-num-ptp');
            const classNumber = parseInt(ptpNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('ptp');

            for (let i = 1; i <= classNumber; i++) {
                const ptpConfigValues = readPTPConfigValues(i);
                const bulkInValuesFS = readPTPBulkInValuesFS(i);
                const bulkOutValuesFS = readPTPBulkOutValuesFS(i);

                appendPTPDescriptorsFromValues(descriptor, commentsList, {
                    instanceIndex: i,
                    interfaceNumber: interfaceOffset + (i - 1),
                    configValues: ptpConfigValues,
                    bulkInValues: bulkInValuesFS,
                    bulkOutValues: bulkOutValuesFS,
                    inEndpointKey: 'ptpBulkInEndpointFS',
                    inPacketKey: 'ptpBulkInMaxPacketSizeFS',
                    inIntervalKey: 'ptpBulkInIntervalFS',
                    outEndpointKey: 'ptpBulkOutEndpointFS',
                    outPacketKey: 'ptpBulkOutMaxPacketSizeFS',
                    outIntervalKey: 'ptpBulkOutIntervalFS',
                    defaultInEndpoint: '0x85',
                    defaultOutEndpoint: '0x05',
                    defaultPacket: '0x0040',
                    defaultInterval: '0'
                });
            }
        }
        finalizeClassSegment('ptp', ptpSegmentStart);

        const cdcSegmentStart = beginClassSegment();
        // Generate CDC ACM descriptors if CDC is enabled and append to device descriptor
        const cdcCheckbox = document.getElementById('class-cdc');
        if (cdcCheckbox && cdcCheckbox.checked) {
            const cdcNumberInput = document.getElementById('class-num-cdc');
            const classNumber = parseInt(cdcNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('cdc');

            for (let i = 1; i <= classNumber; i++) {
                // Read CDC ACM configuration values
                const cdcConfigValues = readCDCConfigValues(i);
                const notifyValuesFS = readCDCNotifyValuesFS(i);
                const bulkInValuesFS = readCDCBulkInValuesFS(i);
                const bulkOutValuesFS = readCDCBulkOutValuesFS(i);

                const commInterfaceNum = interfaceOffset + (i - 1) * 2;
                const dataInterfaceNum = commInterfaceNum + 1;

                appendIadDescriptor(descriptor, commentsList, `CDC-IAD-${i}`, commInterfaceNum, 2,
                    0x02,
                    parseHexValue(cdcConfigValues.cdcBInterfaceSubClass || '0x02', 1, 'cdcBInterfaceSubClass'),
                    parseHexValue(cdcConfigValues.cdcBInterfaceProtocol || '0x01', 1, 'cdcBInterfaceProtocol'));

                // Build Communication Interface Descriptor
                const commInterfaceDescriptor = [
                    0x09,  // bLength
                    0x04,  // bDescriptorType (Interface)
                    commInterfaceNum,  // bInterfaceNumber
                    0x00,  // bAlternateSetting
                    0x01,  // bNumEndpoints (Notification endpoint only)
                    0x02,  // bInterfaceClass (Communications)
                    parseHexValue(cdcConfigValues.cdcBInterfaceSubClass || '0x02', 1, 'cdcBInterfaceSubClass'),  // bInterfaceSubClass
                    parseHexValue(cdcConfigValues.cdcBInterfaceProtocol || '0x01', 1, 'cdcBInterfaceProtocol'),  // bInterfaceProtocol
                    0x00   // iInterface
                ];
                descriptor = descriptor.concat(commInterfaceDescriptor);

                const commInterfaceComments = [
                    'bLength',
                    'bDescriptorType',
                    'bInterfaceNumber',
                    'bAlternateSetting',
                    'bNumEndpoints',
                    'bInterfaceClass',
                    'bInterfaceSubClass',
                    'bInterfaceProtocol',
                    'iInterface'
                ];
                commInterfaceComments.forEach((comment) => {
                    commentsList.push(`[CDC-COMM-IF-${i}] ${comment}`);
                });

                // CDC Header Functional Descriptor
                const cdcBcdCDC = parseHexValue(cdcConfigValues.cdcBcdCDC || '0x0110', 2, 'cdcBcdCDC');
                const headerDescriptor = [
                    0x05,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x00,  // bDescriptorSubtype (Header)
                    cdcBcdCDC & 0xFF,  // bcdCDC LSB
                    (cdcBcdCDC >> 8) & 0xFF   // bcdCDC MSB
                ];
                descriptor = descriptor.concat(headerDescriptor);

                const headerComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bcdCDC LSB',
                    'bcdCDC MSB'
                ];
                headerComments.forEach((comment) => {
                    commentsList.push(`[CDC-HEADER-${i}] ${comment}`);
                });

                // CDC Call Management Functional Descriptor
                const callMgmtDescriptor = [
                    0x05,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x01,  // bDescriptorSubtype (Call Management)
                    0x00,  // bmCapabilities (no call management)
                    dataInterfaceNum   // bDataInterface
                ];
                descriptor = descriptor.concat(callMgmtDescriptor);

                const callMgmtComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bmCapabilities',
                    'bDataInterface'
                ];
                callMgmtComments.forEach((comment) => {
                    commentsList.push(`[CDC-CALL-MGMT-${i}] ${comment}`);
                });

                // CDC ACM Functional Descriptor
                const acmDescriptor = [
                    0x04,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x02,  // bDescriptorSubtype (ACM)
                    parseHexValue(cdcConfigValues.cdcBmCapabilities || '0x02', 1, 'cdcBmCapabilities')  // bmCapabilities
                ];
                descriptor = descriptor.concat(acmDescriptor);

                const acmComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bmCapabilities'
                ];
                acmComments.forEach((comment) => {
                    commentsList.push(`[CDC-ACM-${i}] ${comment}`);
                });

                // CDC Union Functional Descriptor
                const unionDescriptor = [
                    0x05,  // bFunctionLength
                    0x24,  // bDescriptorType (CS_INTERFACE)
                    0x06,  // bDescriptorSubtype (Union)
                    commInterfaceNum,  // bMasterInterface
                    dataInterfaceNum   // bSlaveInterface0
                ];
                descriptor = descriptor.concat(unionDescriptor);

                const unionComments = [
                    'bFunctionLength',
                    'bDescriptorType',
                    'bDescriptorSubtype',
                    'bMasterInterface',
                    'bSlaveInterface0'
                ];
                unionComments.forEach((comment) => {
                    commentsList.push(`[CDC-UNION-${i}] ${comment}`);
                });

                // Notification Endpoint Descriptor (Interrupt IN)
                const notifyEndpoint = parseHexValue(notifyValuesFS.cdcNotifyEndpointFS || '0x81', 1, 'notifyEndpoint');
                const notifyMaxPacket = parseHexValue(notifyValuesFS.cdcNotifyMaxPacketSizeFS || '0x0008', 2, 'notifyMaxPacket');
                const notifyInterval = parseHexValue(notifyValuesFS.cdcNotifyIntervalFS || '16', 1, 'notifyInterval');
                const notifyDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    notifyEndpoint,  // bEndpointAddress
                    0x03,  // bmAttributes (Interrupt)
                    notifyMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (notifyMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    notifyInterval   // bInterval
                ];
                descriptor = descriptor.concat(notifyDescriptor);

                const notifyComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                notifyComments.forEach((comment) => {
                    commentsList.push(`[CDC-EP-NOTIFY-${i}] ${comment}`);
                });

                // Build Data Interface Descriptor
                const dataInterfaceDescriptor = [
                    0x09,  // bLength
                    0x04,  // bDescriptorType (Interface)
                    dataInterfaceNum,  // bInterfaceNumber
                    0x00,  // bAlternateSetting
                    0x02,  // bNumEndpoints (Bulk IN + Bulk OUT)
                    0x0A,  // bInterfaceClass (CDC Data)
                    0x00,  // bInterfaceSubClass
                    0x00,  // bInterfaceProtocol
                    0x00   // iInterface
                ];
                descriptor = descriptor.concat(dataInterfaceDescriptor);

                const dataInterfaceComments = [
                    'bLength',
                    'bDescriptorType',
                    'bInterfaceNumber',
                    'bAlternateSetting',
                    'bNumEndpoints',
                    'bInterfaceClass',
                    'bInterfaceSubClass',
                    'bInterfaceProtocol',
                    'iInterface'
                ];
                dataInterfaceComments.forEach((comment) => {
                    commentsList.push(`[CDC-DATA-IF-${i}] ${comment}`);
                });

                // Data Bulk IN Endpoint Descriptor
                const bulkInEndpoint = parseHexValue(bulkInValuesFS.cdcBulkInEndpointFS || '0x82', 1, 'bulkInEndpoint');
                const bulkInMaxPacket = parseHexValue(bulkInValuesFS.cdcBulkInMaxPacketSizeFS || '0x0040', 2, 'bulkInMaxPacket');
                const bulkInInterval = parseHexValue(bulkInValuesFS.cdcBulkInIntervalFS || '0', 1, 'bulkInInterval');
                const bulkInDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkInEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkInMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkInMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkInInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkInDescriptor);

                const bulkInComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkInComments.forEach((comment) => {
                    commentsList.push(`[CDC-EP-IN-${i}] ${comment}`);
                });

                // Data Bulk OUT Endpoint Descriptor
                const bulkOutEndpoint = parseHexValue(bulkOutValuesFS.cdcBulkOutEndpointFS || '0x02', 1, 'bulkOutEndpoint');
                const bulkOutMaxPacket = parseHexValue(bulkOutValuesFS.cdcBulkOutMaxPacketSizeFS || '0x0040', 2, 'bulkOutMaxPacket');
                const bulkOutInterval = parseHexValue(bulkOutValuesFS.cdcBulkOutIntervalFS || '0', 1, 'bulkOutInterval');
                const bulkOutDescriptor = [
                    0x07,  // bLength
                    0x05,  // bDescriptorType (Endpoint)
                    bulkOutEndpoint,  // bEndpointAddress
                    0x02,  // bmAttributes (Bulk)
                    bulkOutMaxPacket & 0xFF,  // wMaxPacketSize LSB
                    (bulkOutMaxPacket >> 8) & 0xFF,  // wMaxPacketSize MSB
                    bulkOutInterval   // bInterval
                ];
                descriptor = descriptor.concat(bulkOutDescriptor);

                const bulkOutComments = [
                    'bLength',
                    'bDescriptorType',
                    'bEndpointAddress',
                    'bmAttributes',
                    'wMaxPacketSize LSB',
                    'wMaxPacketSize MSB',
                    'bInterval'
                ];
                bulkOutComments.forEach((comment) => {
                    commentsList.push(`[CDC-EP-OUT-${i}] ${comment}`);
                });
            }
        }
        finalizeClassSegment('cdc', cdcSegmentStart);

        const rndisSegmentStart = beginClassSegment();
        // Generate CDC RNDIS descriptors if enabled and append to device descriptor
        const rndisCheckbox = document.getElementById('class-rndis');
        if (rndisCheckbox && rndisCheckbox.checked) {
            const rndisNumberInput = document.getElementById('class-num-rndis');
            const classNumber = parseInt(rndisNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('rndis');

            for (let i = 1; i <= classNumber; i++) {
                const rndisConfig = readRNDISConfigValues(i);
                const notifyValuesFS = readRNDISNotifyValuesFS(i);
                const bulkInValuesFS = readRNDISBulkInValuesFS(i);
                const bulkOutValuesFS = readRNDISBulkOutValuesFS(i);

                const commInterfaceNum = interfaceOffset + (i - 1) * 2;
                const dataInterfaceNum = commInterfaceNum + 1;

                appendIadDescriptor(descriptor, commentsList, `RNDIS-IAD-${i}`, commInterfaceNum, 2, 0x02, 0x02, 0xFF);

                appendCDCNetworkingDescriptors(descriptor, commentsList, {
                    instanceIndex: i,
                    commInterfaceNum,
                    dataInterfaceNum,
                    subclass: 0x02,
                    protocol: 0xFF,
                    bcdCdc: parseHexValue(rndisConfig.rndisBcdCDC || '0x0110', 2, 'rndisBcdCDC'),
                    callMgmtCaps: parseHexValue(rndisConfig.rndisCallMgmtCapabilities || '0x00', 1, 'rndisCallMgmtCapabilities'),
                    acmCaps: parseHexValue(rndisConfig.rndisAcmCapabilities || '0x00', 1, 'rndisAcmCapabilities'),
                    notifyEndpoint: parseHexValue(notifyValuesFS.notifyEndpointFS || '0x81', 1, 'rndisNotifyEndpointFS'),
                    notifyMaxPacket: parseHexValue(notifyValuesFS.notifyMaxPacketSizeFS || '0x0008', 2, 'rndisNotifyMaxPacketSizeFS'),
                    notifyInterval: parseHexValue(notifyValuesFS.notifyIntervalFS || '16', 1, 'rndisNotifyIntervalFS'),
                    bulkInEndpoint: parseHexValue(bulkInValuesFS.bulkInEndpointFS || '0x82', 1, 'rndisBulkInEndpointFS'),
                    bulkInMaxPacket: parseHexValue(bulkInValuesFS.bulkInMaxPacketSizeFS || '0x0040', 2, 'rndisBulkInMaxPacketSizeFS'),
                    bulkInInterval: parseHexValue(bulkInValuesFS.bulkInIntervalFS || '0', 1, 'rndisBulkInIntervalFS'),
                    bulkOutEndpoint: parseHexValue(bulkOutValuesFS.bulkOutEndpointFS || '0x02', 1, 'rndisBulkOutEndpointFS'),
                    bulkOutMaxPacket: parseHexValue(bulkOutValuesFS.bulkOutMaxPacketSizeFS || '0x0040', 2, 'rndisBulkOutMaxPacketSizeFS'),
                    bulkOutInterval: parseHexValue(bulkOutValuesFS.bulkOutIntervalFS || '0', 1, 'rndisBulkOutIntervalFS'),
                    tagPrefix: 'RNDIS',
                    includeCallMgmt: true,
                    includeAcm: true,
                    includeEthernet: false
                });
            }
        }
        finalizeClassSegment('rndis', rndisSegmentStart);

        const ecmSegmentStart = beginClassSegment();
        // Generate CDC ECM descriptors if enabled and append to device descriptor
        const ecmCheckbox = document.getElementById('class-ecm');
        if (ecmCheckbox && ecmCheckbox.checked) {
            const ecmNumberInput = document.getElementById('class-num-ecm');
            const classNumber = parseInt(ecmNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('ecm');

            for (let i = 1; i <= classNumber; i++) {
                const ecmConfig = readECMConfigValues(i);
                const notifyValuesFS = readECMNotifyValuesFS(i);
                const bulkInValuesFS = readECMBulkInValuesFS(i);
                const bulkOutValuesFS = readECMBulkOutValuesFS(i);

                const commInterfaceNum = interfaceOffset + (i - 1) * 2;
                const dataInterfaceNum = commInterfaceNum + 1;

                appendIadDescriptor(descriptor, commentsList, `ECM-IAD-${i}`, commInterfaceNum, 2, 0x02, 0x06, 0x00);

                appendCDCNetworkingDescriptors(descriptor, commentsList, {
                    instanceIndex: i,
                    commInterfaceNum,
                    dataInterfaceNum,
                    subclass: 0x06,
                    protocol: 0x00,
                    bcdCdc: parseHexValue(ecmConfig.ecmBcdCDC || '0x0120', 2, 'ecmBcdCDC'),
                    callMgmtCaps: 0x00,
                    acmCaps: 0x00,
                    notifyEndpoint: parseHexValue(notifyValuesFS.notifyEndpointFS || '0x81', 1, 'ecmNotifyEndpointFS'),
                    notifyMaxPacket: parseHexValue(notifyValuesFS.notifyMaxPacketSizeFS || '0x0008', 2, 'ecmNotifyMaxPacketSizeFS'),
                    notifyInterval: parseHexValue(notifyValuesFS.notifyIntervalFS || '16', 1, 'ecmNotifyIntervalFS'),
                    bulkInEndpoint: parseHexValue(bulkInValuesFS.bulkInEndpointFS || '0x82', 1, 'ecmBulkInEndpointFS'),
                    bulkInMaxPacket: parseHexValue(bulkInValuesFS.bulkInMaxPacketSizeFS || '0x0040', 2, 'ecmBulkInMaxPacketSizeFS'),
                    bulkInInterval: parseHexValue(bulkInValuesFS.bulkInIntervalFS || '0', 1, 'ecmBulkInIntervalFS'),
                    bulkOutEndpoint: parseHexValue(bulkOutValuesFS.bulkOutEndpointFS || '0x02', 1, 'ecmBulkOutEndpointFS'),
                    bulkOutMaxPacket: parseHexValue(bulkOutValuesFS.bulkOutMaxPacketSizeFS || '0x0040', 2, 'ecmBulkOutMaxPacketSizeFS'),
                    bulkOutInterval: parseHexValue(bulkOutValuesFS.bulkOutIntervalFS || '0', 1, 'ecmBulkOutIntervalFS'),
                    tagPrefix: 'ECM',
                    includeCallMgmt: false,
                    includeAcm: false,
                    includeEthernet: true,
                    ethernet: {
                        macStringIndex: ecmConfig.ecmMacStringIndex,
                        maxSegmentSize: ecmConfig.ecmMaxSegmentSize,
                        numMcFilters: ecmConfig.ecmNumMcFilters,
                        numPowerFilters: ecmConfig.ecmNumPowerFilters
                    }
                });
            }
        }
        finalizeClassSegment('ecm', ecmSegmentStart);

        const audioSegmentStart = beginClassSegment();
        // Generate Audio 1.0 descriptors if Audio is enabled and append to device descriptor
        const audioCheckbox = document.getElementById('class-audio');
        if (audioCheckbox && audioCheckbox.checked) {
            const audioNumberInput = document.getElementById('class-num-audio');
            const classNumber = parseInt(audioNumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('audio');

            for (let i = 1; i <= classNumber; i++) {
                const audioConfig = readAudioConfigValues(i);
                const audioEpFS = readAudioEndpointValuesFS(i);

                const acInterfaceNum = interfaceOffset + (i - 1) * 2;
                const asInterfaceNum = acInterfaceNum + 1;
                const acSubClass = parseHexValue(audioConfig.audioControlSubClass || '0x01', 1, 'audioControlSubClass');
                const asSubClass = parseHexValue(audioConfig.audioStreamingSubClass || '0x02', 1, 'audioStreamingSubClass');

                appendIadDescriptor(descriptor, commentsList, `AUDIO-IAD-${i}`, acInterfaceNum, 2, 0x01, acSubClass, 0x00);
                const terminalLink = parseNumberValue(audioConfig.audioTerminalLink || 1, { id: 'audioTerminalLink', min: 1, max: 253, default: 1 });
                const inputTerminalType  = parseHexValue(audioConfig.audioInputTerminalType  || '0x0101', 2, 'audioInputTerminalType');
                const outputTerminalType = parseHexValue(audioConfig.audioOutputTerminalType || '0x0301', 2, 'audioOutputTerminalType');
                const formatTag = parseHexValue(audioConfig.audioFormatTag || '0x0001', 2, 'audioFormatTag');
                const formatType = parseHexValue(audioConfig.audioFormatType, 1, 'audioFormatType');
                const numChannels = parseNumberValue(audioConfig.audioNumChannels || 2, { id: 'audioNumChannels', min: 1, max: 8, default: 2 });
                const bitResolution = parseNumberValue(audioConfig.audioBitResolution || 16, { id: 'audioBitResolution', min: 1, max: 32, default: 16 });
                const subframeSize = Math.ceil(bitResolution / 8);
                const maxBitRate = parseNumberValue(audioConfig.audioMaxBitRate || 128, { id: 'audioMaxBitRate', min: 1, max: 65535, default: 128 });
                const samplesPerFrame = parseNumberValue(audioConfig.audioSamplesPerFrame || 1, { id: 'audioSamplesPerFrame', min: 1, max: 65535, default: 1 });
                const sampleRates = Array.isArray(audioConfig.audioSampleRate) && audioConfig.audioSampleRate.length > 0
                    ? audioConfig.audioSampleRate : [48000];

                // AC topology IDs: IT=terminalLink, FU=terminalLink+2, OT=terminalLink+1
                const fuId = terminalLink + 2;
                const otId = terminalLink + 1;
                const channelConfig = getChannelConfigFromCount(numChannels);

                // Pre-compute bControlSize: 2 if Bass Boost (bit8) or Loudness (bit9) selected
                const _masterCtrlRaw = Array.isArray(audioConfig.audioFuMasterControls) ? audioConfig.audioFuMasterControls : [0, 1];
                const _channelCtrlRaw = Array.isArray(audioConfig.audioFuChannelControls) ? audioConfig.audioFuChannelControls : [];
                const bControlSize = (_masterCtrlRaw.some((b) => b >= 8) || _channelCtrlRaw.some((b) => b >= 8)) ? 2 : 1;
                // FU bLength = 7 + (bNrChannels + 1) * bControlSize
                const fuLength = 7 + (numChannels + 1) * bControlSize;
                // wTotalLength = CS header(9) + IT(12) + FU(fuLength) + OT(9)
                const acTotalLength = 9 + 12 + fuLength + 9;

                const audioAcIInterface = parseNumberValue(audioConfig.audioAcIInterface || 0, { id: 'audioAcIInterface', min: 0, max: 255, default: 0 });
                const acEpEnabled   = !!audioConfig.audioAcEpEnabled;
                const acEpAddress   = acEpEnabled ? parseHexValue(audioConfig.audioAcEpAddressFS   || '0x83', 1, 'audioAcEpAddressFS')   : 0;
                const acEpMaxPacket = acEpEnabled ? parseNumberValue(audioConfig.audioAcEpMaxPacketFS || 2,  { id: 'audioAcEpMaxPacketFS', min: 1, max: 64,  default: 2 })  : 0;
                const acEpInterval  = acEpEnabled ? parseNumberValue(audioConfig.audioAcEpIntervalFS  || 16, { id: 'audioAcEpIntervalFS',  min: 1, max: 255, default: 16 }) : 0;
                const acInterfaceDescriptor = [
                    0x09,
                    0x04,
                    acInterfaceNum,
                    0x00,
                    acEpEnabled ? 0x01 : 0x00,
                    0x01,
                    acSubClass,
                    0x00,
                    audioAcIInterface
                ];
                descriptor = descriptor.concat(acInterfaceDescriptor);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-IF-${i}] ${comment}`));

                const acClassSpecificDescriptor = [
                    0x09,
                    0x24,
                    0x01,
                    0x00,
                    0x01,
                    acTotalLength & 0xFF,
                    (acTotalLength >> 8) & 0xFF,
                    0x01,
                    asInterfaceNum
                ];
                descriptor = descriptor.concat(acClassSpecificDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bcdADC LSB', 'bcdADC MSB', 'wTotalLength LSB', 'wTotalLength MSB', 'bInCollection', 'baInterfaceNr(1)']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-CS-${i}] ${comment}`));

                // Input Terminal Descriptor (UAC1 Table 4-3, 12 bytes)
                const acInputTerminalDesc = [
                    0x0C, 0x24, 0x02,
                    terminalLink,
                    inputTerminalType & 0xFF, (inputTerminalType >> 8) & 0xFF,
                    0x00,
                    numChannels,
                    channelConfig & 0xFF, (channelConfig >> 8) & 0xFF,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(acInputTerminalDesc);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bNrChannels', 'wChannelConfig LSB', 'wChannelConfig MSB', 'iChannelNames', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-IT-${i}] ${comment}`));

                // Feature Unit Descriptor (UAC1 Table 4-6)
                const masterControlBits = _masterCtrlRaw.filter((b) => b >= 0);
                const channelControlBits = _channelCtrlRaw.filter((b) => b >= 0);
                const masterLsb = masterControlBits.filter((b) => b < 8).reduce((acc, b) => acc | (1 << b), 0);
                const masterMsb = masterControlBits.filter((b) => b >= 8).reduce((acc, b) => acc | (1 << (b - 8)), 0);
                const channelLsb = channelControlBits.filter((b) => b < 8).reduce((acc, b) => acc | (1 << b), 0);
                const channelMsb = channelControlBits.filter((b) => b >= 8).reduce((acc, b) => acc | (1 << (b - 8)), 0);
                const fuBmaControls = [];
                if (bControlSize === 1) {
                    fuBmaControls.push(masterLsb);
                    for (let ch = 0; ch < numChannels; ch++) fuBmaControls.push(channelLsb);
                } else {
                    fuBmaControls.push(masterLsb, masterMsb);
                    for (let ch = 0; ch < numChannels; ch++) fuBmaControls.push(channelLsb, channelMsb);
                }
                const acFeatureUnitDesc = [
                    fuLength, 0x24, 0x06,
                    fuId,
                    terminalLink,
                    bControlSize,
                    ...fuBmaControls,
                    0x00
                ];
                descriptor = descriptor.concat(acFeatureUnitDesc);
                const fuComments = ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bUnitID', 'bSourceID', 'bControlSize'];
                for (let ch = 0; ch <= numChannels; ch++) {
                    if (bControlSize === 1) { fuComments.push(`bmaControls(${ch})`); }
                    else { fuComments.push(`bmaControls(${ch}) LSB`, `bmaControls(${ch}) MSB`); }
                }
                fuComments.push('iFeature');
                fuComments.forEach((comment) => commentsList.push(`[AUDIO-AC-FU-${i}] ${comment}`));

                // Output Terminal Descriptor (UAC1 Table 4-4, 9 bytes)
                const acOutputTerminalDesc = [
                    0x09, 0x24, 0x03,
                    otId,
                    outputTerminalType & 0xFF, (outputTerminalType >> 8) & 0xFF,
                    0x00,
                    fuId,
                    0x00
                ];
                descriptor = descriptor.concat(acOutputTerminalDesc);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bSourceID', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO-AC-OT-${i}] ${comment}`));

                if (acEpEnabled) {
                    const acEpDescriptor = [
                        0x09, 0x05, acEpAddress, 0x03,
                        acEpMaxPacket & 0xFF, (acEpMaxPacket >> 8) & 0xFF,
                        acEpInterval, 0x00, 0x00
                    ];
                    descriptor = descriptor.concat(acEpDescriptor);
                    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval', 'bRefresh', 'bSynchAddress']
                        .forEach((comment) => commentsList.push(`[AUDIO-AC-EP-${i}] ${comment}`));
                }

                const audioAsIInterface = parseNumberValue(audioConfig.audioAsIInterface || 0, { id: 'audioAsIInterface', min: 0, max: 255, default: 0 });

                const asInterfaceAlt0 = [
                    0x09,
                    0x04,
                    asInterfaceNum,
                    0x00,
                    0x00,
                    0x01,
                    asSubClass,
                    0x00,
                    audioAsIInterface
                ];
                descriptor = descriptor.concat(asInterfaceAlt0);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO-AS-IF0-${i}] ${comment}`));

                const audioSyncTypeFS = parseInt(audioEpFS.audioSyncType, 16) || 0x01;
                const hasFeedbackEpFS = audioSyncTypeFS === 0x05;
                const asInterfaceAlt1 = [
                    0x09,
                    0x04,
                    asInterfaceNum,
                    0x01,
                    hasFeedbackEpFS ? 0x02 : 0x01,
                    0x01,
                    asSubClass,
                    0x00,
                    audioAsIInterface
                ];
                descriptor = descriptor.concat(asInterfaceAlt1);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO-AS-IF1-${i}] ${comment}`));

                const asGeneralDescriptor = [
                    0x07,
                    0x24,
                    0x01,
                    terminalLink,
                    0x01,
                    formatTag & 0xFF,
                    (formatTag >> 8) & 0xFF
                ];
                descriptor = descriptor.concat(asGeneralDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalLink', 'bDelay', 'wFormatTag LSB', 'wFormatTag MSB']
                    .forEach((comment) => commentsList.push(`[AUDIO-AS-CS-GEN-${i}] ${comment}`));

                const samFreqBytesFS = [];
                const samFreqCommentsFS = [];
                sampleRates.forEach((rate, idx) => {
                    samFreqBytesFS.push(rate & 0xFF, (rate >> 8) & 0xFF, (rate >> 16) & 0xFF);
                    samFreqCommentsFS.push(`tSamFreq[${idx}] byte0`, `tSamFreq[${idx}] byte1`, `tSamFreq[${idx}] byte2`);
                });

                let formatTypeDescriptor;
                let formatTypeComments;
                if (formatType === 0x02) {
                    // Type II: wMaxBitRate + wSamplesPerFrame instead of channel/resolution fields
                    formatTypeDescriptor = [
                        0x09 + sampleRates.length * 3,
                        0x24, 0x02, 0x02,
                        maxBitRate & 0xFF, (maxBitRate >> 8) & 0xFF,
                        samplesPerFrame & 0xFF, (samplesPerFrame >> 8) & 0xFF,
                        sampleRates.length,
                        ...samFreqBytesFS
                    ];
                    formatTypeComments = ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bFormatType', 'wMaxBitRate LSB', 'wMaxBitRate MSB', 'wSamplesPerFrame LSB', 'wSamplesPerFrame MSB', 'bSamFreqType', ...samFreqCommentsFS];
                } else {
                    // Type I or Type III: bNrChannels + bSubframeSize + bBitResolution
                    formatTypeDescriptor = [
                        0x08 + sampleRates.length * 3,
                        0x24, 0x02, formatType,
                        numChannels, subframeSize, bitResolution,
                        sampleRates.length,
                        ...samFreqBytesFS
                    ];
                    formatTypeComments = ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bFormatType', 'bNrChannels', 'bSubframeSize', 'bBitResolution', 'bSamFreqType', ...samFreqCommentsFS];
                }
                descriptor = descriptor.concat(formatTypeDescriptor);
                formatTypeComments.forEach((comment) => commentsList.push(`[AUDIO-AS-CS-FMT-${i}] ${comment}`));

                const epAddress = parseHexValue(audioEpFS.audioEndpointAddressFS || '0x81', 1, 'audioEndpointAddressFS');
                const epMaxPacket = parseInt(audioEpFS.audioMaxPacketSizeFS, 10) || 192;
                const epInterval = parseHexValue(audioEpFS.audioIntervalFS || '1', 1, 'audioIntervalFS');
                const fbEpAddressFS   = hasFeedbackEpFS ? parseHexValue(audioEpFS.audioFbEpAddressFS || '0x01', 1, 'audioFbEpAddressFS') : 0;
                const fbEpMaxPacketFS = hasFeedbackEpFS ? (parseInt(audioEpFS.audioFbEpMaxPacketFS, 10) || 3) : 0;
                const fbEpIntervalFS  = hasFeedbackEpFS ? (parseInt(audioEpFS.audioFbEpIntervalFS,  10) || 1) : 0;
                const isocEndpointDescriptor = [
                    0x09,
                    0x05,
                    epAddress,
                    audioSyncTypeFS & 0xFF,
                    epMaxPacket & 0xFF,
                    (epMaxPacket >> 8) & 0xFF,
                    epInterval,
                    0x00,
                    hasFeedbackEpFS ? fbEpAddressFS : 0x00
                ];
                descriptor = descriptor.concat(isocEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval', 'bRefresh', 'bSynchAddress']
                    .forEach((comment) => commentsList.push(`[AUDIO-EP-${i}] ${comment}`));

                const csIsoEndpointDescriptor = [
                    0x07,
                    0x25,
                    0x01,
                    sampleRates.length > 1 ? 0x01 : 0x00,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(csIsoEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bmAttributes', 'bLockDelayUnits', 'wLockDelay LSB', 'wLockDelay MSB']
                    .forEach((comment) => commentsList.push(`[AUDIO-CS-EP-${i}] ${comment}`));

                if (hasFeedbackEpFS) {
                    const fbEpDescriptorFS = [
                        0x09, 0x05, fbEpAddressFS, 0x11,
                        fbEpMaxPacketFS & 0xFF, (fbEpMaxPacketFS >> 8) & 0xFF,
                        fbEpIntervalFS, 0x00, 0x00
                    ];
                    descriptor = descriptor.concat(fbEpDescriptorFS);
                    ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval', 'bRefresh', 'bSynchAddress']
                        .forEach((comment) => commentsList.push(`[AUDIO-FB-EP-${i}] ${comment}`));
                }
            }
        }
                finalizeClassSegment('audio', audioSegmentStart);

        const audio2SegmentStart = beginClassSegment();
        // Generate Audio 2.0 descriptors if Audio 2.0 is enabled and append to device descriptor
        const audio2Checkbox = document.getElementById('class-audio2');
        if (audio2Checkbox && audio2Checkbox.checked) {
            const audio2NumberInput = document.getElementById('class-num-audio2');
            const classNumber = parseInt(audio2NumberInput.value) || 1;
            const interfaceOffset = getClassInterfaceOffset('audio2');

            for (let i = 1; i <= classNumber; i++) {
                const audio2Config = readAudio2ConfigValues(i);
                const audio2EpFS = readAudio2EndpointValuesFS(i);

                const acInterfaceNum = interfaceOffset + (i - 1) * 2;
                const asInterfaceNum = acInterfaceNum + 1;
                const acSubClass = parseHexValue(audio2Config.audio20ControlSubClass || '0x01', 1, 'audio20ControlSubClass');
                const asSubClass = parseHexValue(audio2Config.audio20StreamingSubClass || '0x02', 1, 'audio20StreamingSubClass');

                appendIadDescriptor(descriptor, commentsList, `AUDIO2-IAD-${i}`, acInterfaceNum, 2, 0x01, acSubClass, 0x20);
                const clockSourceId = parseNumberValue(audio2Config.audio20ClockSourceId || 16, { id: 'audio20ClockSourceId', min: 1, max: 255, default: 16 });
                const terminalLink = parseNumberValue(audio2Config.audio20TerminalLink || 1, { id: 'audio20TerminalLink', min: 1, max: 255, default: 1 });
                const numChannels = parseNumberValue(audio2Config.audio20NumChannels || 2, { id: 'audio20NumChannels', min: 1, max: 8, default: 2 });
                const subslotSize = parseHexValue(audio2Config.audio20SubslotSize || '0x02', 1, 'audio20SubslotSize');
                const bitResolution = parseNumberValue(audio2Config.audio20BitResolution || 16, { id: 'audio20BitResolution', min: 1, max: 32, default: 16 });

                const acInterfaceDescriptor = [
                    0x09,
                    0x04,
                    acInterfaceNum,
                    0x00,
                    0x00,
                    0x01,
                    acSubClass,
                    0x20,
                    0x00
                ];
                descriptor = descriptor.concat(acInterfaceDescriptor);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AC-IF-${i}] ${comment}`));

                const acHeaderDescriptor = [
                    0x09,
                    0x24,
                    0x01,
                    0x00,
                    0x02,
                    0x25,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(acHeaderDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bcdADC LSB', 'bcdADC MSB', 'wTotalLength LSB', 'wTotalLength MSB', 'bmControls LSB', 'bmControls MSB']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AC-HDR-${i}] ${comment}`));

                const clockSourceDescriptor = [
                    0x08,
                    0x24,
                    0x0A,
                    clockSourceId,
                    0x03,
                    0x07,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(clockSourceDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bClockID', 'bmAttributes', 'bmControls', 'bAssocTerminal', 'iClockSource']
                    .forEach((comment) => commentsList.push(`[AUDIO2-CLOCK-${i}] ${comment}`));

                const inputTerminalDescriptor = [
                    0x11,
                    0x24,
                    0x02,
                    terminalLink,
                    0x01,
                    0x01,
                    0x00,
                    clockSourceId,
                    numChannels,
                    0x03,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(inputTerminalDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bCSourceID', 'bNrChannels', 'bmChannelConfig B0', 'bmChannelConfig B1', 'bmChannelConfig B2', 'bmChannelConfig B3', 'iChannelNames', 'bmControls LSB', 'bmControls MSB', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO2-INTERM-${i}] ${comment}`));

                const outputTerminalDescriptor = [
                    0x0C,
                    0x24,
                    0x03,
                    terminalLink + 1,
                    0x01,
                    0x03,
                    0x00,
                    terminalLink,
                    clockSourceId,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(outputTerminalDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalID', 'wTerminalType LSB', 'wTerminalType MSB', 'bAssocTerminal', 'bSourceID', 'bCSourceID', 'bmControls LSB', 'bmControls MSB', 'iTerminal']
                    .forEach((comment) => commentsList.push(`[AUDIO2-OUTTERM-${i}] ${comment}`));

                const asInterfaceAlt0 = [
                    0x09,
                    0x04,
                    asInterfaceNum,
                    0x00,
                    0x00,
                    0x01,
                    asSubClass,
                    0x20,
                    0x00
                ];
                descriptor = descriptor.concat(asInterfaceAlt0);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-IF0-${i}] ${comment}`));

                const asInterfaceAlt1 = [
                    0x09,
                    0x04,
                    asInterfaceNum,
                    0x01,
                    0x01,
                    0x01,
                    asSubClass,
                    0x20,
                    0x00
                ];
                descriptor = descriptor.concat(asInterfaceAlt1);
                ['bLength', 'bDescriptorType', 'bInterfaceNumber', 'bAlternateSetting', 'bNumEndpoints', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol', 'iInterface']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-IF1-${i}] ${comment}`));

                const asGeneralDescriptor = [
                    0x10,
                    0x24,
                    0x01,
                    terminalLink,
                    0x00,
                    0x00,
                    0x01,
                    0x00,
                    0x00,
                    0x00,
                    numChannels,
                    0x03,
                    0x00,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(asGeneralDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bTerminalLink', 'bmControls LSB', 'bmControls MSB', 'bFormatType', 'bmFormats B0', 'bmFormats B1', 'bmFormats B2', 'bNrChannels', 'bmChannelConfig B0', 'bmChannelConfig B1', 'bmChannelConfig B2', 'bmChannelConfig B3', 'iChannelNames']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-GEN-${i}] ${comment}`));

                const formatTypeDescriptor = [
                    0x06,
                    0x24,
                    0x02,
                    subslotSize,
                    bitResolution,
                    0x00
                ];
                descriptor = descriptor.concat(formatTypeDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bSubslotSize', 'bBitResolution', 'bReserved']
                    .forEach((comment) => commentsList.push(`[AUDIO2-AS-FMT-${i}] ${comment}`));

                const epAddress = parseHexValue(audio2EpFS.audio2EndpointAddressFS || '0x81', 1, 'audio2EndpointAddressFS');
                const epMaxPacket = parseHexValue(audio2EpFS.audio2MaxPacketSizeFS || '0x00C0', 2, 'audio2MaxPacketSizeFS');
                const epInterval = parseHexValue(audio2EpFS.audio2IntervalFS || '1', 1, 'audio2IntervalFS');
                const isocEndpointDescriptor = [
                    0x07,
                    0x05,
                    epAddress,
                    0x05,
                    epMaxPacket & 0xFF,
                    (epMaxPacket >> 8) & 0xFF,
                    epInterval
                ];
                descriptor = descriptor.concat(isocEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bEndpointAddress', 'bmAttributes', 'wMaxPacketSize LSB', 'wMaxPacketSize MSB', 'bInterval']
                    .forEach((comment) => commentsList.push(`[AUDIO2-EP-${i}] ${comment}`));

                const csIsoEndpointDescriptor = [
                    0x08,
                    0x25,
                    0x01,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00
                ];
                descriptor = descriptor.concat(csIsoEndpointDescriptor);
                ['bLength', 'bDescriptorType', 'bDescriptorSubtype', 'bmAttributes', 'bmControls LSB', 'bmControls MSB', 'bLockDelayUnits', 'wLockDelay']
                    .forEach((comment) => commentsList.push(`[AUDIO2-CS-EP-${i}] ${comment}`));
            }
        }
        finalizeClassSegment('audio2', audio2SegmentStart);

        getCurrentClassOrder().forEach((className) => {
            const segment = classSegments.get(className);
            if (!segment) {
                return;
            }

            descriptor = descriptor.concat(segment.descriptorPart);
            commentsList = commentsList.concat(segment.commentsPart);
        });

        // Configuration descriptor starts right after the 18-byte device descriptor.
        configValues.wTotalLength = updateConfigurationTotalLength(descriptor, 18);

        renderCArray(descriptor, commentsList);
        renderJSONArray(values, configValues);
        renderHexDump(descriptor);
        renderReadable(values, configValues, descriptor);
    } catch (error) {
        alert('Error generating descriptor: ' + error.message);
    }
}

function readHIDFormValues() {
    const hidValues = [];
    const hidNumberInput = document.getElementById('class-num-hid');
    if (!hidNumberInput) return hidValues;

    const classNumber = parseInt(hidNumberInput.value) || 1;

    for (let i = 1; i <= classNumber; i++) {
        const bcdHIDInput = document.getElementById(`hid-bcdHID-${i}`);
        const countryCodeSelect = document.getElementById(`hid-bCountryCode-${i}`);
        const bNumDescriptorsInput = document.getElementById(`hid-bNumDescriptors-${i}`);

        if (!bcdHIDInput) continue;

        const descriptors = [];
        const numDescriptors = parseInt(bNumDescriptorsInput.value) || 1;

        for (let j = 1; j <= numDescriptors; j++) {
            const typeSelect = document.getElementById(`hid-desc-type-${i}-${j}`);
            const lengthInput = document.getElementById(`hid-desc-length-${i}-${j}`);

            if (typeSelect && lengthInput) {
                descriptors.push({
                    type: typeSelect.value,
                    length: parseInt(lengthInput.value) || 0
                });
            }
        }

        hidValues.push({
            instance: i,
            bcdHID: bcdHIDInput.value,
            bCountryCode: countryCodeSelect.value,
            bNumDescriptors: numDescriptors,
            descriptors: descriptors
        });
    }

    return hidValues;
}

function buildInterfaceDescriptor(instanceNumber) {
    return [
        0x09,                       // bLength
        0x04,                       // bDescriptorType (Interface)
        instanceNumber - 1,         // bInterfaceNumber (0-indexed)
        0x00,                       // bAlternateSetting
        0x01,                       // bNumEndpoints
        0x03,                       // bInterfaceClass (HID)
        0x01,                       // bInterfaceSubClass (Boot Interface Subclass)
        0x00,                       // bInterfaceProtocol
        0x00                        // iInterface
    ];
}

function buildEndpointDescriptor(instanceNumber) {
    return [
        0x07,                               // bLength
        0x05,                               // bDescriptorType (Endpoint)
        0x81 + (instanceNumber - 1),        // bEndpointAddress (IN endpoint per HID instance)
        0x03,                               // bmAttributes (Interrupt)
        0x40,                               // wMaxPacketSize LSB (64 bytes)
        0x00,                               // wMaxPacketSize MSB
        0x08                                // bInterval
    ];
}

function buildHIDDescriptor(hidData) {
    const descriptor = [
        9,  // bLength (HID descriptor is 9 bytes)
        0x21,  // bDescriptorType (HID)
    ];

    // Parse bcdHID (2 bytes, little-endian)
    const bcdHIDMatch = hidData.bcdHID.match(/0x([0-9A-Fa-f]{4})/);
    if (bcdHIDMatch) {
        const bcdValue = parseInt(bcdHIDMatch[1], 16);
        descriptor.push(bcdValue & 0xFF);
        descriptor.push((bcdValue >> 8) & 0xFF);
    } else {
        descriptor.push(0x10);
        descriptor.push(0x01);
    }

    // bCountryCode
    const countryValue = parseInt(hidData.bCountryCode, 16);
    descriptor.push(countryValue);

    // bNumDescriptors
    descriptor.push(hidData.bNumDescriptors);

    // Descriptor entries
    for (const desc of hidData.descriptors) {
        const typeValue = parseInt(desc.type, 16);
        descriptor.push(typeValue);

        const length = desc.length;
        descriptor.push(length & 0xFF);
        descriptor.push((length >> 8) & 0xFF);
    }

    return descriptor;
}

function getHIDDescriptorComment(index, hidData) {
    const comments = [
        'bLength',
        'bDescriptorType',
        'bcdHID LSB',
        'bcdHID MSB',
        'bCountryCode',
        'bNumDescriptors'
    ];

    if (index < comments.length) {
        return comments[index];
    }

    const remainingIndex = index - comments.length;
    const perDescriptor = 3;
    const descriptorIndex = Math.floor(remainingIndex / perDescriptor);
    const fieldIndex = remainingIndex % perDescriptor;

    const descriptorComments = ['Type', 'Length LSB', 'Length MSB'];
    return `Descriptor ${descriptorIndex + 1} ${descriptorComments[fieldIndex]}`;
}

function renderInterfaceDescriptorFields(instanceNum) {
    const fieldsForTemplate = INTERFACE_SCHEMA.fields.map((field) => ({
        ...field,
        id: `interface-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    document.getElementById(`interface-form-fields-${instanceNum}`).innerHTML = html;

    const container = document.getElementById(`interface-form-fields-${instanceNum}`);
    const indexElement = document.getElementById(`interface-${instanceNum}-iInterface`);
    const stringElement = document.getElementById(`interface-${instanceNum}-interfaceString`);
    const indexGroup = indexElement ? indexElement.closest('.form-group') : null;
    const stringGroup = stringElement ? stringElement.closest('.form-group') : null;

    if (container && indexGroup && stringGroup) {
        const wrapper = document.createElement('div');
        wrapper.className = 'string-field-group';
        wrapper.id = `interface-${instanceNum}-string-group`;
        wrapper.innerHTML = `
            <div class="string-field-group-title">Interface String (iInterface)</div>
            <div class="string-field-grid">
                <div class="form-group string-language-group">
                    <label for="interface-${instanceNum}-languageIdDisplay">
                        Language ID ${createInfoIcon('Select language when multiple language IDs are enabled')}
                    </label>
                    <select id="interface-${instanceNum}-languageIdDisplay" class="interface-language-selector" data-instance="${instanceNum}"></select>
                </div>
            </div>
        `;

        const grid = wrapper.querySelector('.string-field-grid');
        container.insertBefore(wrapper, indexGroup);
        grid.insertBefore(indexGroup, grid.firstChild);
        grid.insertBefore(stringGroup, grid.querySelector('.string-language-group'));

        setFormGroupLabel(indexGroup, 'Index', '0 = no string');
        setFormGroupLabel(stringGroup, 'String', 'Interface string');
        updateInterfaceLanguageSelectors();
    }

    const protocolElement = document.getElementById(`interface-${instanceNum}-bInterfaceProtocol`);
    const getDefaultInterfaceStringByProtocol = () => {
        if (!protocolElement) {
            return 'custom';
        }

        const protocolValue = parseInt(protocolElement.value, 16);
        if (protocolValue === 0x02) {
            return 'Mouse';
        }
        if (protocolValue === 0x01) {
            return 'Keybord';
        }

        return 'custom';
    };

    const syncInterfaceStringFromProtocol = () => {
        const currentStringElement = document.getElementById(`interface-${instanceNum}-interfaceString`);
        if (!currentStringElement) {
            return;
        }

        currentStringElement.value = getDefaultInterfaceStringByProtocol();
    };

    const updateInterfaceStringState = () => {
        const currentIndexElement = document.getElementById(`interface-${instanceNum}-iInterface`);
        const currentStringElement = document.getElementById(`interface-${instanceNum}-interfaceString`);
        const languageSelector = document.getElementById(`interface-${instanceNum}-languageIdDisplay`);

        if (!currentIndexElement || !currentStringElement) {
            return;
        }

        const currentIndexGroup = currentIndexElement.closest('.form-group');
        const currentStringGroup = currentStringElement.closest('.form-group');

        if (currentIndexGroup) {
            currentIndexGroup.style.display = 'block';
        }
        if (currentStringGroup) {
            currentStringGroup.style.display = 'block';
        }

        const indexValue = parseFloat(currentIndexElement.value) || 0;
        const isEditable = indexValue !== 0;
        currentStringElement.disabled = !isEditable;

        if (languageSelector) {
            languageSelector.style.display = 'block';
            languageSelector.dataset.lockedByIndex = isEditable ? 'false' : 'true';
            languageSelector.disabled = !isEditable;
        }
    };

    // Set up conditional visibility for this instance
    updateClassConditionalFieldsVisibility(instanceNum, INTERFACE_SCHEMA, 'interface');
    syncInterfaceStringFromProtocol();
    updateInterfaceStringState();

    // Add change listener for iInterface field
    const iInterfaceElement = document.getElementById(`interface-${instanceNum}-iInterface`);
    if (iInterfaceElement) {
        iInterfaceElement.addEventListener('change', () => {
            updateClassConditionalFieldsVisibility(instanceNum, INTERFACE_SCHEMA, 'interface');
            updateInterfaceStringState();
        });
        iInterfaceElement.addEventListener('input', () => {
            updateClassConditionalFieldsVisibility(instanceNum, INTERFACE_SCHEMA, 'interface');
            updateInterfaceStringState();
        });
    }

    if (protocolElement) {
        protocolElement.addEventListener('change', () => {
            syncInterfaceStringFromProtocol();
            updateInterfaceStringState();
        });
        protocolElement.addEventListener('input', () => {
            syncInterfaceStringFromProtocol();
            updateInterfaceStringState();
        });
    }
}

function renderHIDDetailsFields(instanceNum) {
    const fieldsForTemplate = HID_DESC_SCHEMA.fields.map((field) => ({
        ...field,
        id: `hid-details-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    document.getElementById(`hid-details-form-fields-${instanceNum}`).innerHTML = html;
}

function renderEndpointInDescriptorFields(instanceNum) {
    const fieldsForTemplate = ENDPOINT_SCHEMA.fields.map((field) => ({
        ...field,
        id: `endpoint-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    document.getElementById(`endpoint-form-fields-${instanceNum}`).innerHTML = html;
}

function renderEndpointInDescriptorFieldsHighSpeed(instanceNum) {
    const fieldsForTemplate = ENDPOINT_SCHEMA.fields.map((field) => ({
        ...field,
        id: `endpoint-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`endpoint-form-fields-hs-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderEndpointOutDescriptorFields(instanceNum) {
    const fieldsForTemplate = ENDPOINT_SCHEMA.fields.map((field) => ({
        ...field,
        id: `endpoint-out-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`endpoint-out-form-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderEndpointOutDescriptorFieldsHighSpeed(instanceNum) {
    const fieldsForTemplate = ENDPOINT_SCHEMA.fields.map((field) => ({
        ...field,
        id: `endpoint-out-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`endpoint-out-form-fields-hs-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function parseHexValue(rawValue, bytes, fieldId) {
    // Remove spaces and convert to uppercase
    const cleanValue = String(rawValue || '').trim().toUpperCase();

    // Check if it's hex format
    const match = cleanValue.match(/^(0X)?([0-9A-F]+)$/);
    if (!match || match[2].length > bytes * 2) {
        console.error(`Invalid hex value: ${rawValue} for field ${fieldId}`);
        return 0;
    }

    const hexStr = match[2].padStart(bytes * 2, '0').slice(-bytes * 2);
    return parseInt(hexStr, 16);
}

function parseNumberValue(rawValue, field) {
    const num = parseInt(rawValue);

    if (isNaN(num)) {
        return field.default || 0;
    }

    if (field.min !== undefined && num < field.min) {
        console.warn(`Value ${num} below minimum ${field.min} for field`);
        return field.min;
    }

    if (field.max !== undefined && num > field.max) {
        console.warn(`Value ${num} above maximum ${field.max} for field`);
        return field.max;
    }

    return num;
}

function readInterfaceDescriptorValues(instanceNum) {
    const values = {};

    INTERFACE_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`interface-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    // HID interface fields fixed by spec for this generator.
    values.bAlternateSetting = 0x00;
    values.bInterfaceClass = 0x03;
    values.bNumEndpoints = 1;  // Default to 1 endpoint (IN only), will be set to 2 if OUT endpoint is present

    return values;
}

function readHIDDetailsValues(instanceNum) {
    const values = {};

    HID_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`hid-details-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readEndpointInDescriptorValues(instanceNum) {
    const values = {};

    ENDPOINT_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`endpoint-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    // HID endpoint attributes fixed by spec for this generator.
    values.endpointBmAttributes = 0x03;  // Interrupt transfer type

    return values;
}

function readEndpointOutDescriptorValues(instanceNum) {
    const values = {};

    ENDPOINT_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`endpoint-out-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    // HID endpoint attributes fixed by spec for this generator.
    values.endpointBmAttributes = 0x03;  // Interrupt transfer type

    return values;
}

function readEndpointInDescriptorValuesHighSpeed(instanceNum) {
    const values = {};

    ENDPOINT_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`endpoint-hs-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    // HID endpoint attributes fixed by spec for this generator.
    values.endpointBmAttributes = 0x03;  // Interrupt transfer type

    return values;
}

function readEndpointOutDescriptorValuesHighSpeed(instanceNum) {
    const values = {};

    ENDPOINT_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`endpoint-out-hs-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = parseHexValue(rawValue, field.bytes, field.id);
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    // HID endpoint attributes fixed by spec for this generator.
    values.endpointBmAttributes = 0x03;  // Interrupt transfer type

    return values;
}

function readMassStorageConfigValues(instanceNum) {
    const values = {};

    MASS_STORAGE_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`msc-config-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = element.value;  // Return raw value for later parsing
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readDFUConfigValues(instanceNum) {
    const values = {};

    DFU_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`dfu-config-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = element.value;
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readPrinterConfigValues(instanceNum) {
    const values = {};

    PRINTER_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`printer-config-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = element.value;
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readMassStorageBulkInValuesFS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`msc-bulk-in-fs-${instanceNum}-mscBulkInEndpointFS`);
    const maxPacketElement = document.getElementById(`msc-bulk-in-fs-${instanceNum}-mscBulkInMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`msc-bulk-in-fs-${instanceNum}-mscBulkInIntervalFS`);

    if (endpointElement) {
        values.mscBulkInEndpointFS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.mscBulkInMaxPacketSizeFS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.mscBulkInIntervalFS = intervalElement.value;
    }

    return values;
}

function readMassStorageBulkInValuesHS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`msc-bulk-in-hs-${instanceNum}-mscBulkInEndpointHS`);
    const maxPacketElement = document.getElementById(`msc-bulk-in-hs-${instanceNum}-mscBulkInMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`msc-bulk-in-hs-${instanceNum}-mscBulkInIntervalHS`);

    if (endpointElement) {
        values.mscBulkInEndpointHS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.mscBulkInMaxPacketSizeHS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.mscBulkInIntervalHS = intervalElement.value;
    }

    return values;
}

function readMassStorageBulkOutValuesFS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`msc-bulk-out-fs-${instanceNum}-mscBulkOutEndpointFS`);
    const maxPacketElement = document.getElementById(`msc-bulk-out-fs-${instanceNum}-mscBulkOutMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`msc-bulk-out-fs-${instanceNum}-mscBulkOutIntervalFS`);

    if (endpointElement) {
        values.mscBulkOutEndpointFS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.mscBulkOutMaxPacketSizeFS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.mscBulkOutIntervalFS = intervalElement.value;
    }

    return values;
}

function readMassStorageBulkOutValuesHS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`msc-bulk-out-hs-${instanceNum}-mscBulkOutEndpointHS`);
    const maxPacketElement = document.getElementById(`msc-bulk-out-hs-${instanceNum}-mscBulkOutMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`msc-bulk-out-hs-${instanceNum}-mscBulkOutIntervalHS`);

    if (endpointElement) {
        values.mscBulkOutEndpointHS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.mscBulkOutMaxPacketSizeHS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.mscBulkOutIntervalHS = intervalElement.value;
    }

    return values;
}

function readPrinterBulkInValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`printer-bulk-in-fs-${instanceNum}-printerBulkInEndpointFS`);
    const maxPacketElement = document.getElementById(`printer-bulk-in-fs-${instanceNum}-printerBulkInMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`printer-bulk-in-fs-${instanceNum}-printerBulkInIntervalFS`);
    if (endpointElement) values.printerBulkInEndpointFS = endpointElement.value;
    if (maxPacketElement) values.printerBulkInMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.printerBulkInIntervalFS = intervalElement.value;
    return values;
}

function readPrinterBulkInValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`printer-bulk-in-hs-${instanceNum}-printerBulkInEndpointHS`);
    const maxPacketElement = document.getElementById(`printer-bulk-in-hs-${instanceNum}-printerBulkInMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`printer-bulk-in-hs-${instanceNum}-printerBulkInIntervalHS`);
    if (endpointElement) values.printerBulkInEndpointHS = endpointElement.value;
    if (maxPacketElement) values.printerBulkInMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.printerBulkInIntervalHS = intervalElement.value;
    return values;
}

function readPrinterBulkOutValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`printer-bulk-out-fs-${instanceNum}-printerBulkOutEndpointFS`);
    const maxPacketElement = document.getElementById(`printer-bulk-out-fs-${instanceNum}-printerBulkOutMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`printer-bulk-out-fs-${instanceNum}-printerBulkOutIntervalFS`);
    if (endpointElement) values.printerBulkOutEndpointFS = endpointElement.value;
    if (maxPacketElement) values.printerBulkOutMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.printerBulkOutIntervalFS = intervalElement.value;
    return values;
}

function readPrinterBulkOutValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`printer-bulk-out-hs-${instanceNum}-printerBulkOutEndpointHS`);
    const maxPacketElement = document.getElementById(`printer-bulk-out-hs-${instanceNum}-printerBulkOutMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`printer-bulk-out-hs-${instanceNum}-printerBulkOutIntervalHS`);
    if (endpointElement) values.printerBulkOutEndpointHS = endpointElement.value;
    if (maxPacketElement) values.printerBulkOutMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.printerBulkOutIntervalHS = intervalElement.value;
    return values;
}

function readVideoConfigValues(instanceNum) {
    const values = {};

    VIDEO_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`video-config-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;
        if (field.format === 'hex') {
            values[field.id] = element.value;
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readVideoBulkInValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`video-bulk-in-fs-${instanceNum}-videoBulkInEndpointFS`);
    const maxPacketElement = document.getElementById(`video-bulk-in-fs-${instanceNum}-videoBulkInMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`video-bulk-in-fs-${instanceNum}-videoBulkInIntervalFS`);
    if (endpointElement) values.videoBulkInEndpointFS = endpointElement.value;
    if (maxPacketElement) values.videoBulkInMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.videoBulkInIntervalFS = intervalElement.value;
    return values;
}

function readVideoBulkInValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`video-bulk-in-hs-${instanceNum}-videoBulkInEndpointHS`);
    const maxPacketElement = document.getElementById(`video-bulk-in-hs-${instanceNum}-videoBulkInMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`video-bulk-in-hs-${instanceNum}-videoBulkInIntervalHS`);
    if (endpointElement) values.videoBulkInEndpointHS = endpointElement.value;
    if (maxPacketElement) values.videoBulkInMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.videoBulkInIntervalHS = intervalElement.value;
    return values;
}

function readVideoBulkOutValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`video-bulk-out-fs-${instanceNum}-videoBulkOutEndpointFS`);
    const maxPacketElement = document.getElementById(`video-bulk-out-fs-${instanceNum}-videoBulkOutMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`video-bulk-out-fs-${instanceNum}-videoBulkOutIntervalFS`);
    if (endpointElement) values.videoBulkOutEndpointFS = endpointElement.value;
    if (maxPacketElement) values.videoBulkOutMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.videoBulkOutIntervalFS = intervalElement.value;
    return values;
}

function readVideoBulkOutValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`video-bulk-out-hs-${instanceNum}-videoBulkOutEndpointHS`);
    const maxPacketElement = document.getElementById(`video-bulk-out-hs-${instanceNum}-videoBulkOutMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`video-bulk-out-hs-${instanceNum}-videoBulkOutIntervalHS`);
    if (endpointElement) values.videoBulkOutEndpointHS = endpointElement.value;
    if (maxPacketElement) values.videoBulkOutMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.videoBulkOutIntervalHS = intervalElement.value;
    return values;
}

function readMTPConfigValues(instanceNum) {
    const values = {};

    MTP_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`mtp-config-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;
        if (field.format === 'hex') {
            values[field.id] = element.value;
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readMTPBulkInValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`mtp-bulk-in-fs-${instanceNum}-mtpBulkInEndpointFS`);
    const maxPacketElement = document.getElementById(`mtp-bulk-in-fs-${instanceNum}-mtpBulkInMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`mtp-bulk-in-fs-${instanceNum}-mtpBulkInIntervalFS`);
    if (endpointElement) values.mtpBulkInEndpointFS = endpointElement.value;
    if (maxPacketElement) values.mtpBulkInMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.mtpBulkInIntervalFS = intervalElement.value;
    return values;
}

function readMTPBulkInValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`mtp-bulk-in-hs-${instanceNum}-mtpBulkInEndpointHS`);
    const maxPacketElement = document.getElementById(`mtp-bulk-in-hs-${instanceNum}-mtpBulkInMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`mtp-bulk-in-hs-${instanceNum}-mtpBulkInIntervalHS`);
    if (endpointElement) values.mtpBulkInEndpointHS = endpointElement.value;
    if (maxPacketElement) values.mtpBulkInMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.mtpBulkInIntervalHS = intervalElement.value;
    return values;
}

function readMTPBulkOutValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`mtp-bulk-out-fs-${instanceNum}-mtpBulkOutEndpointFS`);
    const maxPacketElement = document.getElementById(`mtp-bulk-out-fs-${instanceNum}-mtpBulkOutMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`mtp-bulk-out-fs-${instanceNum}-mtpBulkOutIntervalFS`);
    if (endpointElement) values.mtpBulkOutEndpointFS = endpointElement.value;
    if (maxPacketElement) values.mtpBulkOutMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.mtpBulkOutIntervalFS = intervalElement.value;
    return values;
}

function readMTPBulkOutValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`mtp-bulk-out-hs-${instanceNum}-mtpBulkOutEndpointHS`);
    const maxPacketElement = document.getElementById(`mtp-bulk-out-hs-${instanceNum}-mtpBulkOutMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`mtp-bulk-out-hs-${instanceNum}-mtpBulkOutIntervalHS`);
    if (endpointElement) values.mtpBulkOutEndpointHS = endpointElement.value;
    if (maxPacketElement) values.mtpBulkOutMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.mtpBulkOutIntervalHS = intervalElement.value;
    return values;
}

function readPTPConfigValues(instanceNum) {
    const values = {};

    PTP_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`ptp-config-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;
        if (field.format === 'hex') {
            values[field.id] = element.value;
            return;
        }

        if (field.format === 'string') {
            values[field.id] = String(rawValue || field.default || '');
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readPTPBulkInValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`ptp-bulk-in-fs-${instanceNum}-ptpBulkInEndpointFS`);
    const maxPacketElement = document.getElementById(`ptp-bulk-in-fs-${instanceNum}-ptpBulkInMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`ptp-bulk-in-fs-${instanceNum}-ptpBulkInIntervalFS`);
    if (endpointElement) values.ptpBulkInEndpointFS = endpointElement.value;
    if (maxPacketElement) values.ptpBulkInMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.ptpBulkInIntervalFS = intervalElement.value;
    return values;
}

function readPTPBulkInValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`ptp-bulk-in-hs-${instanceNum}-ptpBulkInEndpointHS`);
    const maxPacketElement = document.getElementById(`ptp-bulk-in-hs-${instanceNum}-ptpBulkInMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`ptp-bulk-in-hs-${instanceNum}-ptpBulkInIntervalHS`);
    if (endpointElement) values.ptpBulkInEndpointHS = endpointElement.value;
    if (maxPacketElement) values.ptpBulkInMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.ptpBulkInIntervalHS = intervalElement.value;
    return values;
}

function readPTPBulkOutValuesFS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`ptp-bulk-out-fs-${instanceNum}-ptpBulkOutEndpointFS`);
    const maxPacketElement = document.getElementById(`ptp-bulk-out-fs-${instanceNum}-ptpBulkOutMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`ptp-bulk-out-fs-${instanceNum}-ptpBulkOutIntervalFS`);
    if (endpointElement) values.ptpBulkOutEndpointFS = endpointElement.value;
    if (maxPacketElement) values.ptpBulkOutMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.ptpBulkOutIntervalFS = intervalElement.value;
    return values;
}

function readPTPBulkOutValuesHS(instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`ptp-bulk-out-hs-${instanceNum}-ptpBulkOutEndpointHS`);
    const maxPacketElement = document.getElementById(`ptp-bulk-out-hs-${instanceNum}-ptpBulkOutMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`ptp-bulk-out-hs-${instanceNum}-ptpBulkOutIntervalHS`);
    if (endpointElement) values.ptpBulkOutEndpointHS = endpointElement.value;
    if (maxPacketElement) values.ptpBulkOutMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.ptpBulkOutIntervalHS = intervalElement.value;
    return values;
}

function readCDCConfigValues(instanceNum) {
    const values = {};

    CDC_ACM_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`cdc-config-${instanceNum}-${field.id}`);
        if (element) {
            values[field.id] = element.value;
        }
    });

    return values;
}

function readCDCNotifyValuesFS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`cdc-notify-fs-${instanceNum}-cdcNotifyEndpointFS`);
    const maxPacketElement = document.getElementById(`cdc-notify-fs-${instanceNum}-cdcNotifyMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`cdc-notify-fs-${instanceNum}-cdcNotifyIntervalFS`);

    if (endpointElement) {
        values.cdcNotifyEndpointFS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.cdcNotifyMaxPacketSizeFS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.cdcNotifyIntervalFS = intervalElement.value;
    }

    return values;
}

function readCDCNotifyValuesHS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`cdc-notify-hs-${instanceNum}-cdcNotifyEndpointHS`);
    const maxPacketElement = document.getElementById(`cdc-notify-hs-${instanceNum}-cdcNotifyMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`cdc-notify-hs-${instanceNum}-cdcNotifyIntervalHS`);

    if (endpointElement) {
        values.cdcNotifyEndpointHS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.cdcNotifyMaxPacketSizeHS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.cdcNotifyIntervalHS = intervalElement.value;
    }

    return values;
}

function readCDCBulkInValuesFS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`cdc-bulk-in-fs-${instanceNum}-cdcBulkInEndpointFS`);
    const maxPacketElement = document.getElementById(`cdc-bulk-in-fs-${instanceNum}-cdcBulkInMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`cdc-bulk-in-fs-${instanceNum}-cdcBulkInIntervalFS`);

    if (endpointElement) {
        values.cdcBulkInEndpointFS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.cdcBulkInMaxPacketSizeFS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.cdcBulkInIntervalFS = intervalElement.value;
    }

    return values;
}

function readCDCBulkInValuesHS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`cdc-bulk-in-hs-${instanceNum}-cdcBulkInEndpointHS`);
    const maxPacketElement = document.getElementById(`cdc-bulk-in-hs-${instanceNum}-cdcBulkInMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`cdc-bulk-in-hs-${instanceNum}-cdcBulkInIntervalHS`);

    if (endpointElement) {
        values.cdcBulkInEndpointHS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.cdcBulkInMaxPacketSizeHS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.cdcBulkInIntervalHS = intervalElement.value;
    }

    return values;
}

function readCDCBulkOutValuesFS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`cdc-bulk-out-fs-${instanceNum}-cdcBulkOutEndpointFS`);
    const maxPacketElement = document.getElementById(`cdc-bulk-out-fs-${instanceNum}-cdcBulkOutMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`cdc-bulk-out-fs-${instanceNum}-cdcBulkOutIntervalFS`);

    if (endpointElement) {
        values.cdcBulkOutEndpointFS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.cdcBulkOutMaxPacketSizeFS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.cdcBulkOutIntervalFS = intervalElement.value;
    }

    return values;
}

function readCDCBulkOutValuesHS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`cdc-bulk-out-hs-${instanceNum}-cdcBulkOutEndpointHS`);
    const maxPacketElement = document.getElementById(`cdc-bulk-out-hs-${instanceNum}-cdcBulkOutMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`cdc-bulk-out-hs-${instanceNum}-cdcBulkOutIntervalHS`);

    if (endpointElement) {
        values.cdcBulkOutEndpointHS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.cdcBulkOutMaxPacketSizeHS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.cdcBulkOutIntervalHS = intervalElement.value;
    }

    return values;
}

function renderCDCNetworkConfigFields(prefix, schema, instanceNum) {
    const fieldsForTemplate = schema.fields.map((field) => ({
        ...field,
        id: `${prefix}-config-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`${prefix}-config-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNetworkNotifyFieldsFS(prefix, instanceNum) {
    const endpointFields = [
        {
            id: 'notifyEndpointFS',
            label: 'Notification Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for notifications (e.g., 0x81).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'notifyMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed interrupt endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0008',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'notifyIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for Full Speed interrupt endpoint.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '16',
            min: 1,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `${prefix}-notify-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`${prefix}-notify-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNetworkNotifyFieldsHS(prefix, instanceNum) {
    const endpointFields = [
        {
            id: 'notifyEndpointHS',
            label: 'Notification Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for notifications (e.g., 0x81).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'notifyMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed interrupt endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' },
                { value: '0x0200', text: '512 bytes (0x0200)' },
                { value: '0x0400', text: '1024 bytes (0x0400)' }
            ]
        },
        {
            id: 'notifyIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for High Speed interrupt endpoint.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '16',
            min: 1,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `${prefix}-notify-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`${prefix}-notify-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNetworkBulkInFieldsFS(prefix, instanceNum) {
    const endpointFields = [
        {
            id: 'bulkInEndpointFS',
            label: 'Bulk IN Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for data (e.g., 0x82).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x82',
            placeholder: '0x82'
        },
        {
            id: 'bulkInMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed bulk endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'bulkInIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `${prefix}-bulk-in-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`${prefix}-bulk-in-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNetworkBulkInFieldsHS(prefix, instanceNum) {
    const endpointFields = [
        {
            id: 'bulkInEndpointHS',
            label: 'Bulk IN Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for data (e.g., 0x82).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x82',
            placeholder: '0x82'
        },
        {
            id: 'bulkInMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed bulk endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0200', text: '512 bytes (0x0200)' }
            ]
        },
        {
            id: 'bulkInIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `${prefix}-bulk-in-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`${prefix}-bulk-in-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNetworkBulkOutFieldsFS(prefix, instanceNum) {
    const endpointFields = [
        {
            id: 'bulkOutEndpointFS',
            label: 'Bulk OUT Endpoint Address (bEndpointAddress)',
            help: 'OUT endpoint address for data (e.g., 0x02).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            placeholder: '0x02'
        },
        {
            id: 'bulkOutMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed bulk endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'bulkOutIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `${prefix}-bulk-out-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`${prefix}-bulk-out-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNetworkBulkOutFieldsHS(prefix, instanceNum) {
    const endpointFields = [
        {
            id: 'bulkOutEndpointHS',
            label: 'Bulk OUT Endpoint Address (bEndpointAddress)',
            help: 'OUT endpoint address for data (e.g., 0x02).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            placeholder: '0x02'
        },
        {
            id: 'bulkOutMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed bulk endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0200', text: '512 bytes (0x0200)' }
            ]
        },
        {
            id: 'bulkOutIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `${prefix}-bulk-out-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`${prefix}-bulk-out-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function readCDCNetworkConfigValues(prefix, schema, instanceNum) {
    const values = {};

    schema.fields.forEach((field) => {
        const element = document.getElementById(`${prefix}-config-${instanceNum}-${field.id}`);
        if (element) {
            values[field.id] = element.value;
        }
    });

    return values;
}

function readCDCNetworkNotifyValuesFS(prefix, instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`${prefix}-notify-fs-${instanceNum}-notifyEndpointFS`);
    const maxPacketElement = document.getElementById(`${prefix}-notify-fs-${instanceNum}-notifyMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`${prefix}-notify-fs-${instanceNum}-notifyIntervalFS`);

    if (endpointElement) values.notifyEndpointFS = endpointElement.value;
    if (maxPacketElement) values.notifyMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.notifyIntervalFS = intervalElement.value;

    return values;
}

function readCDCNetworkNotifyValuesHS(prefix, instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`${prefix}-notify-hs-${instanceNum}-notifyEndpointHS`);
    const maxPacketElement = document.getElementById(`${prefix}-notify-hs-${instanceNum}-notifyMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`${prefix}-notify-hs-${instanceNum}-notifyIntervalHS`);

    if (endpointElement) values.notifyEndpointHS = endpointElement.value;
    if (maxPacketElement) values.notifyMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.notifyIntervalHS = intervalElement.value;

    return values;
}

function readCDCNetworkBulkInValuesFS(prefix, instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`${prefix}-bulk-in-fs-${instanceNum}-bulkInEndpointFS`);
    const maxPacketElement = document.getElementById(`${prefix}-bulk-in-fs-${instanceNum}-bulkInMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`${prefix}-bulk-in-fs-${instanceNum}-bulkInIntervalFS`);

    if (endpointElement) values.bulkInEndpointFS = endpointElement.value;
    if (maxPacketElement) values.bulkInMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.bulkInIntervalFS = intervalElement.value;

    return values;
}

function readCDCNetworkBulkInValuesHS(prefix, instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`${prefix}-bulk-in-hs-${instanceNum}-bulkInEndpointHS`);
    const maxPacketElement = document.getElementById(`${prefix}-bulk-in-hs-${instanceNum}-bulkInMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`${prefix}-bulk-in-hs-${instanceNum}-bulkInIntervalHS`);

    if (endpointElement) values.bulkInEndpointHS = endpointElement.value;
    if (maxPacketElement) values.bulkInMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.bulkInIntervalHS = intervalElement.value;

    return values;
}

function readCDCNetworkBulkOutValuesFS(prefix, instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`${prefix}-bulk-out-fs-${instanceNum}-bulkOutEndpointFS`);
    const maxPacketElement = document.getElementById(`${prefix}-bulk-out-fs-${instanceNum}-bulkOutMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`${prefix}-bulk-out-fs-${instanceNum}-bulkOutIntervalFS`);

    if (endpointElement) values.bulkOutEndpointFS = endpointElement.value;
    if (maxPacketElement) values.bulkOutMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.bulkOutIntervalFS = intervalElement.value;

    return values;
}

function readCDCNetworkBulkOutValuesHS(prefix, instanceNum) {
    const values = {};
    const endpointElement = document.getElementById(`${prefix}-bulk-out-hs-${instanceNum}-bulkOutEndpointHS`);
    const maxPacketElement = document.getElementById(`${prefix}-bulk-out-hs-${instanceNum}-bulkOutMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`${prefix}-bulk-out-hs-${instanceNum}-bulkOutIntervalHS`);

    if (endpointElement) values.bulkOutEndpointHS = endpointElement.value;
    if (maxPacketElement) values.bulkOutMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.bulkOutIntervalHS = intervalElement.value;

    return values;
}

function readRNDISConfigValues(instanceNum) {
    return readCDCNetworkConfigValues('rndis', CDC_RNDIS_DESC_SCHEMA, instanceNum);
}

function readRNDISNotifyValuesFS(instanceNum) {
    return readCDCNetworkNotifyValuesFS('rndis', instanceNum);
}

function readRNDISNotifyValuesHS(instanceNum) {
    return readCDCNetworkNotifyValuesHS('rndis', instanceNum);
}

function readRNDISBulkInValuesFS(instanceNum) {
    return readCDCNetworkBulkInValuesFS('rndis', instanceNum);
}

function readRNDISBulkInValuesHS(instanceNum) {
    return readCDCNetworkBulkInValuesHS('rndis', instanceNum);
}

function readRNDISBulkOutValuesFS(instanceNum) {
    return readCDCNetworkBulkOutValuesFS('rndis', instanceNum);
}

function readRNDISBulkOutValuesHS(instanceNum) {
    return readCDCNetworkBulkOutValuesHS('rndis', instanceNum);
}

function readECMConfigValues(instanceNum) {
    const rawValues = readCDCNetworkConfigValues('ecm', CDC_ECM_DESC_SCHEMA, instanceNum);
    const values = {};
    values.ecmBcdCDC = rawValues.ecmBcdCDC;
    values.ecmMacStringIndex = parseNumberValue(rawValues.ecmMacStringIndex || 4, { id: 'ecmMacStringIndex', min: 0, max: 255, default: 4 });
    values.ecmMaxSegmentSize = parseNumberValue(rawValues.ecmMaxSegmentSize || 1514, { id: 'ecmMaxSegmentSize', min: 64, max: 65535, default: 1514 });
    values.ecmNumMcFilters = parseNumberValue(rawValues.ecmNumMcFilters || 0, { id: 'ecmNumMcFilters', min: 0, max: 65535, default: 0 });
    values.ecmNumPowerFilters = parseNumberValue(rawValues.ecmNumPowerFilters || 0, { id: 'ecmNumPowerFilters', min: 0, max: 255, default: 0 });
    return values;
}

function readECMNotifyValuesFS(instanceNum) {
    return readCDCNetworkNotifyValuesFS('ecm', instanceNum);
}

function readECMNotifyValuesHS(instanceNum) {
    return readCDCNetworkNotifyValuesHS('ecm', instanceNum);
}

function readECMBulkInValuesFS(instanceNum) {
    return readCDCNetworkBulkInValuesFS('ecm', instanceNum);
}

function readECMBulkInValuesHS(instanceNum) {
    return readCDCNetworkBulkInValuesHS('ecm', instanceNum);
}

function readECMBulkOutValuesFS(instanceNum) {
    return readCDCNetworkBulkOutValuesFS('ecm', instanceNum);
}

function readECMBulkOutValuesHS(instanceNum) {
    return readCDCNetworkBulkOutValuesHS('ecm', instanceNum);
}

function computeAudioIsoMaxPacketSize(instanceNum) {
    const config = readAudioConfigValues(instanceNum);
    const sampleRates = Array.isArray(config.audioSampleRate) && config.audioSampleRate.length > 0
        ? config.audioSampleRate : [48000];
    const maxRate = Math.max(...sampleRates);
    const channels = typeof config.audioNumChannels === 'number' ? config.audioNumChannels : 2;
    const bitRes = typeof config.audioBitResolution === 'number' ? config.audioBitResolution : 16;
    const bytesPerSample = Math.ceil(bitRes / 8);
    return Math.ceil(maxRate / 1000) * channels * bytesPerSample;
}

function updateAudioMaxPacketSizeFields(instanceNum) {
    const pkt = computeAudioIsoMaxPacketSize(instanceNum);
    const syncEl = document.getElementById(`audio-ep-sync-${instanceNum}`);
    const isAsync = syncEl && syncEl.value === '0x05';
    let targetPkt = pkt;
    if (isAsync) {
        const numChEl  = document.getElementById(`audio-config-${instanceNum}-audioNumChannels`);
        const bitResEl = document.getElementById(`audio-config-${instanceNum}-audioBitResolution`);
        const ch   = parseInt((numChEl  || {}).value, 10) || 2;
        const bits = parseInt((bitResEl || {}).value, 10) || 16;
        targetPkt = pkt + ch * Math.ceil(bits / 8);
    }
    const fsEl = document.getElementById(`audio-ep-fs-${instanceNum}-audioMaxPacketSizeFS`);
    if (fsEl) fsEl.value = String(targetPkt);
    const hsEl = document.getElementById(`audio-ep-hs-${instanceNum}-audioMaxPacketSizeHS`);
    if (hsEl) hsEl.value = String(targetPkt);
}

function readAudioConfigValues(instanceNum) {
    const values = {};

    AUDIO10_DESC_SCHEMA.fields.forEach((field) => {
        const fullId = `audio-config-${instanceNum}-${field.id}`;

        if (field.type === 'multiselect') {
            const checked = document.querySelectorAll(`input[name="${fullId}"]:checked`);
            const selected = Array.from(checked).map((cb) => parseInt(cb.value, 10));
            values[field.id] = selected.length > 0 ? selected : (Array.isArray(field.default) ? field.default : [field.default]);
            return;
        }

        const element = document.getElementById(fullId);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = element.value;
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    // bFormatType is always derived from wFormatTag – never read from the UI
    values.audioFormatType = getFormatTypeFromTag(values.audioFormatTag || '0x0001');

    // wChannelConfig is always derived from bNrChannels – never read from the UI
    values.audioChannelConfig = getChannelConfigFromCount(values.audioNumChannels || 2);

    // AC-only fields (not in AUDIO10_DESC_SCHEMA)
    const acIInterfaceEl = document.getElementById(`audio-ac-${instanceNum}-iInterface`);
    values.audioAcIInterface = acIInterfaceEl ? (parseInt(acIInterfaceEl.value, 10) || 0) : 0;
    const acStrEl = document.getElementById(`audio-ac-${instanceNum}-interfaceString`);
    values.audioAcInterfaceString = acStrEl ? acStrEl.value : '';

    // AC interrupt endpoint (optional)
    values.audioAcEpEnabled = !!audioAcEpState[instanceNum];
    const acEpAddrFsEl   = document.getElementById(`audio-ac-ep-fs-${instanceNum}-epAddressFS`);
    const acEpPktFsEl    = document.getElementById(`audio-ac-ep-fs-${instanceNum}-epMaxPacketFS`);
    const acEpIntvlFsEl  = document.getElementById(`audio-ac-ep-fs-${instanceNum}-epIntervalFS`);
    values.audioAcEpAddressFS   = acEpAddrFsEl  ? acEpAddrFsEl.value  : '0x83';
    values.audioAcEpMaxPacketFS = acEpPktFsEl   ? (parseInt(acEpPktFsEl.value,  10) || 2)  : 2;
    values.audioAcEpIntervalFS  = acEpIntvlFsEl ? (parseInt(acEpIntvlFsEl.value, 10) || 16) : 16;
    const acEpAddrHsEl   = document.getElementById(`audio-ac-ep-hs-${instanceNum}-epAddressHS`);
    const acEpPktHsEl    = document.getElementById(`audio-ac-ep-hs-${instanceNum}-epMaxPacketHS`);
    const acEpIntvlHsEl  = document.getElementById(`audio-ac-ep-hs-${instanceNum}-epIntervalHS`);
    values.audioAcEpAddressHS   = acEpAddrHsEl  ? acEpAddrHsEl.value  : '0x83';
    values.audioAcEpMaxPacketHS = acEpPktHsEl   ? (parseInt(acEpPktHsEl.value,  10) || 2)  : 2;
    values.audioAcEpIntervalHS  = acEpIntvlHsEl ? (parseInt(acEpIntvlHsEl.value, 10) || 6) : 6;

    // AS-only fields (not in AUDIO10_DESC_SCHEMA)
    const asIInterfaceEl = document.getElementById(`audio-as-${instanceNum}-iInterface`);
    values.audioAsIInterface = asIInterfaceEl ? (parseInt(asIInterfaceEl.value, 10) || 0) : 0;
    const asStrEl = document.getElementById(`audio-as-${instanceNum}-interfaceString`);
    values.audioAsInterfaceString = asStrEl ? asStrEl.value : '';

    return values;
}

function readAudioEndpointValuesFS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`audio-ep-fs-${instanceNum}-audioEndpointAddressFS`);
    const maxPacketElement = document.getElementById(`audio-ep-fs-${instanceNum}-audioMaxPacketSizeFS`);
    const intervalElement = document.getElementById(`audio-ep-fs-${instanceNum}-audioIntervalFS`);

    if (endpointElement) values.audioEndpointAddressFS = endpointElement.value;
    if (maxPacketElement) values.audioMaxPacketSizeFS = maxPacketElement.value;
    if (intervalElement) values.audioIntervalFS = intervalElement.value;

    const syncEl = document.getElementById(`audio-ep-sync-${instanceNum}`);
    values.audioSyncType = syncEl ? syncEl.value : '0x01';

    const fbAddrEl  = document.getElementById(`audio-fb-ep-fs-${instanceNum}-fbAddressFS`);
    const fbPktEl   = document.getElementById(`audio-fb-ep-fs-${instanceNum}-fbMaxPacketFS`);
    const fbIntvlEl = document.getElementById(`audio-fb-ep-fs-${instanceNum}-fbIntervalFS`);
    values.audioFbEpAddressFS   = fbAddrEl  ? fbAddrEl.value  : '0x01';
    values.audioFbEpMaxPacketFS = fbPktEl   ? (parseInt(fbPktEl.value,   10) || 3) : 3;
    values.audioFbEpIntervalFS  = fbIntvlEl ? (parseInt(fbIntvlEl.value, 10) || 1) : 1;

    return values;
}

function readAudioEndpointValuesHS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`audio-ep-hs-${instanceNum}-audioEndpointAddressHS`);
    const maxPacketElement = document.getElementById(`audio-ep-hs-${instanceNum}-audioMaxPacketSizeHS`);
    const intervalElement = document.getElementById(`audio-ep-hs-${instanceNum}-audioIntervalHS`);

    if (endpointElement) values.audioEndpointAddressHS = endpointElement.value;
    if (maxPacketElement) values.audioMaxPacketSizeHS = maxPacketElement.value;
    if (intervalElement) values.audioIntervalHS = intervalElement.value;

    const syncEl = document.getElementById(`audio-ep-sync-${instanceNum}`);
    values.audioSyncType = syncEl ? syncEl.value : '0x01';

    const fbAddrEl  = document.getElementById(`audio-fb-ep-hs-${instanceNum}-fbAddressHS`);
    const fbPktEl   = document.getElementById(`audio-fb-ep-hs-${instanceNum}-fbMaxPacketHS`);
    const fbIntvlEl = document.getElementById(`audio-fb-ep-hs-${instanceNum}-fbIntervalHS`);
    values.audioFbEpAddressHS   = fbAddrEl  ? fbAddrEl.value  : '0x01';
    values.audioFbEpMaxPacketHS = fbPktEl   ? (parseInt(fbPktEl.value,   10) || 4) : 4;
    values.audioFbEpIntervalHS  = fbIntvlEl ? (parseInt(fbIntvlEl.value, 10) || 4) : 4;

    return values;
}

function readAudio2ConfigValues(instanceNum) {
    const values = {};

    AUDIO20_DESC_SCHEMA.fields.forEach((field) => {
        const element = document.getElementById(`audio2-config-${instanceNum}-${field.id}`);
        if (!element) return;

        const rawValue = element.value;

        if (field.format === 'hex') {
            values[field.id] = element.value;
            return;
        }

        values[field.id] = parseNumberValue(rawValue, field);
    });

    return values;
}

function readAudio2EndpointValuesFS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`audio2-ep-fs-${instanceNum}-audio2EndpointAddressFS`);
    const maxPacketElement = document.getElementById(`audio2-ep-fs-${instanceNum}-audio2MaxPacketSizeFS`);
    const intervalElement = document.getElementById(`audio2-ep-fs-${instanceNum}-audio2IntervalFS`);

    if (endpointElement) {
        values.audio2EndpointAddressFS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.audio2MaxPacketSizeFS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.audio2IntervalFS = intervalElement.value;
    }

    return values;
}

function readAudio2EndpointValuesHS(instanceNum) {
    const values = {};

    const endpointElement = document.getElementById(`audio2-ep-hs-${instanceNum}-audio2EndpointAddressHS`);
    const maxPacketElement = document.getElementById(`audio2-ep-hs-${instanceNum}-audio2MaxPacketSizeHS`);
    const intervalElement = document.getElementById(`audio2-ep-hs-${instanceNum}-audio2IntervalHS`);

    if (endpointElement) {
        values.audio2EndpointAddressHS = endpointElement.value;
    }

    if (maxPacketElement) {
        values.audio2MaxPacketSizeHS = maxPacketElement.value;
    }

    if (intervalElement) {
        values.audio2IntervalHS = intervalElement.value;
    }

    return values;
}

function buildInterfaceDescriptorFromValues(ifValues) {
    return [
        9,  // bLength
        0x04,  // bDescriptorType (Interface)
        ifValues.bInterfaceNumber,  // bInterfaceNumber
        ifValues.bAlternateSetting,  // bAlternateSetting
        ifValues.bNumEndpoints,  // bNumEndpoints
        ifValues.bInterfaceClass,  // bInterfaceClass
        ifValues.bInterfaceSubClass,  // bInterfaceSubClass
        ifValues.bInterfaceProtocol,  // bInterfaceProtocol
        ifValues.iInterface  // iInterface
    ];
}

function buildHIDDetailsDescriptorFromValues(hidValues) {
    return [
        9,  // bLength
        0x21,  // bDescriptorType (HID)
        hidValues.hidBcdHID & 0xFF,  // bcdHID LSB
        (hidValues.hidBcdHID >> 8) & 0xFF,  // bcdHID MSB
        hidValues.hidBCountryCode,  // bCountryCode
        hidValues.hidBNumDescriptors,  // bNumDescriptors
        0x22,  // bReportDescriptorType (Report)
        0x00,  // wReportDescriptorLength LSB (placeholder)
        0x01   // wReportDescriptorLength MSB (placeholder)
    ];
}

function buildEndpointInDescriptorFromValues(epValues) {
    return [
        7,  // bLength
        0x05,  // bDescriptorType (Endpoint)
        epValues.endpointBEndpointAddress,  // bEndpointAddress
        epValues.endpointBmAttributes,  // bmAttributes
        epValues.endpointWMaxPacketSize & 0xFF,  // wMaxPacketSize LSB
        (epValues.endpointWMaxPacketSize >> 8) & 0xFF,  // wMaxPacketSize MSB
        epValues.endpointBInterval  // bInterval
    ];
}

function switchTab(tabId, clickedButton) {
    document.querySelectorAll('.tab-content').forEach((tab) => tab.classList.remove('active'));
    document.querySelectorAll('.tab-button').forEach((button) => button.classList.remove('active'));

    document.getElementById(tabId).classList.add('active');
    clickedButton.classList.add('active');
}

function saveJSON() {
    try {
        const values = readFormValues();
        const configValues = readConfigurationValues();
        validateUniqueStringIndexes(values, configValues);

        const jsonOutput = collectConfiguredParameters(values, configValues);
        const jsonText = JSON.stringify(jsonOutput, null, 2);
        const blob = new Blob([jsonText], { type: 'application/json' });
        const url = URL.createObjectURL(blob);

        const link = document.createElement('a');
        link.href = url;
        link.download = 'usb_descriptor_config.json';
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);

        URL.revokeObjectURL(url);
    } catch (error) {
        alert('Error saving JSON: ' + error.message);
    }
}

function copyToClipboard(elementId, buttonElement) {
    const text = document.getElementById(elementId).innerText;

    navigator.clipboard.writeText(text).then(() => {
        const originalText = buttonElement.textContent;
        buttonElement.textContent = 'Copied!';
        buttonElement.style.background = '#50fa7b';

        setTimeout(() => {
            buttonElement.textContent = originalText;
            buttonElement.style.background = '#667eea';
        }, 2000);
    }).catch((error) => {
        alert('Failed to copy: ' + error);
    });
}

function newDescriptor() {
    // Reset the form to default values
    document.getElementById('descriptorForm').reset();

    Object.keys(optionalEndpointState).forEach((key) => {
        delete optionalEndpointState[key];
    });

    // Clear custom string descriptors
    customStringDescriptors = [];
    customStringNextId = 1;
    renderCustomStringDescriptorCard();

    // Hide HID descriptor section
    const hidCheckbox = document.getElementById('class-hid');
    if (hidCheckbox) {
        hidCheckbox.checked = false;
        document.getElementById('hidDescriptorCard').style.display = 'none';
    }

    // Hide Mass Storage descriptor section
    const mscCheckbox = document.getElementById('class-msc');
    if (mscCheckbox) {
        mscCheckbox.checked = false;
        document.getElementById('massStorageDescriptorCard').style.display = 'none';
    }

    // Hide DFU descriptor section
    const dfuCheckbox = document.getElementById('class-dfu');
    if (dfuCheckbox) {
        dfuCheckbox.checked = false;
        document.getElementById('dfuDescriptorCard').style.display = 'none';
    }

    // Hide Printer descriptor section
    const printerCheckbox = document.getElementById('class-printer');
    if (printerCheckbox) {
        printerCheckbox.checked = false;
        document.getElementById('printerDescriptorCard').style.display = 'none';
    }

    // Hide Video descriptor section
    const videoCheckbox = document.getElementById('class-video');
    if (videoCheckbox) {
        videoCheckbox.checked = false;
        document.getElementById('videoDescriptorCard').style.display = 'none';
    }

    // Hide MTP descriptor section
    const mtpCheckbox = document.getElementById('class-mtp');
    if (mtpCheckbox) {
        mtpCheckbox.checked = false;
        document.getElementById('mtpDescriptorCard').style.display = 'none';
    }

    // Hide PTP descriptor section
    const ptpCheckbox = document.getElementById('class-ptp');
    if (ptpCheckbox) {
        ptpCheckbox.checked = false;
        document.getElementById('ptpDescriptorCard').style.display = 'none';
    }

    // Hide CDC ACM descriptor section
    const cdcCheckbox = document.getElementById('class-cdc');
    if (cdcCheckbox) {
        cdcCheckbox.checked = false;
        document.getElementById('cdcDescriptorCard').style.display = 'none';
    }

    // Hide CDC RNDIS descriptor section
    const rndisCheckbox = document.getElementById('class-rndis');
    if (rndisCheckbox) {
        rndisCheckbox.checked = false;
        document.getElementById('cdcRndisDescriptorCard').style.display = 'none';
    }

    // Hide CDC ECM descriptor section
    const ecmCheckbox = document.getElementById('class-ecm');
    if (ecmCheckbox) {
        ecmCheckbox.checked = false;
        document.getElementById('cdcEcmDescriptorCard').style.display = 'none';
    }

    // Hide Audio 1.0 descriptor section
    const audioCheckbox = document.getElementById('class-audio');
    if (audioCheckbox) {
        audioCheckbox.checked = false;
        document.getElementById('audioDescriptorCard').style.display = 'none';
    }

    // Hide Audio 2.0 descriptor section
    const audio2Checkbox = document.getElementById('class-audio2');
    if (audio2Checkbox) {
        audio2Checkbox.checked = false;
        document.getElementById('audio2DescriptorCard').style.display = 'none';
    }

    // Clear outputs
    document.getElementById('c-array-output').innerHTML = '/* Click "Generate Descriptor" to see output */';
    document.getElementById('json-array-output').innerHTML = '/* Click "Generate Descriptor" to see output */';
    document.getElementById('hex-output').innerHTML = '/* Click "Generate Descriptor" to see output */';
    document.getElementById('readable-output').innerHTML = '/* Click "Generate Descriptor" to see output */';

    alert('New descriptor created. Form has been reset to default values.');
}

function resetForm() {
    document.getElementById('descriptorForm').reset();
    Object.keys(optionalEndpointState).forEach((key) => {
        delete optionalEndpointState[key];
    });

    if (document.getElementById('class-hid').checked) {
        renderHIDDescriptorFields();
    }

    if (document.getElementById('class-cdc').checked) {
        renderCDCACMDescriptorFields();
    }

    if (document.getElementById('class-rndis').checked) {
        renderCDCRNDISDescriptorFields();
    }

    if (document.getElementById('class-ecm').checked) {
        renderCDCECMDescriptorFields();
    }

    if (document.getElementById('class-msc').checked) {
        renderMassStorageDescriptorFields();
    }

    if (document.getElementById('class-dfu').checked) {
        renderDFUDescriptorFields();
    }

    if (document.getElementById('class-printer').checked) {
        renderPrinterDescriptorFields();
    }

    if (document.getElementById('class-video').checked) {
        renderVideoDescriptorFields();
    }

    if (document.getElementById('class-mtp').checked) {
        renderMTPDescriptorFields();
    }

    if (document.getElementById('class-ptp').checked) {
        renderPTPDescriptorFields();
    }

    if (document.getElementById('class-audio').checked) {
        renderAudioDescriptorFields();
    }

    if (document.getElementById('class-audio2').checked) {
        renderAudio2DescriptorFields();
    }

    generateDescriptor();
}

function setElementValueFromJson(elementId, value) {
    if (value === undefined || value === null) {
        return;
    }

    const element = document.getElementById(elementId);
    if (!element) {
        return;
    }

    if (element.type === 'checkbox') {
        element.checked = !!value;
        return;
    }

    let stringValue = String(value);
    if (element.dataset && element.dataset.format === 'hex' && typeof value === 'number') {
        const bytes = parseInt(element.dataset.bytes || '1', 10) || 1;
        stringValue = toHex(value, bytes);
    }

    element.value = stringValue;
}

function applyObjectToPrefixedFields(valuesObject, prefix) {
    if (!valuesObject || typeof valuesObject !== 'object') {
        return;
    }

    Object.entries(valuesObject).forEach(([key, value]) => {
        setElementValueFromJson(`${prefix}-${key}`, value);
    });
}

function applySavedClassInstances(classesJson, isHighSpeedEnabled) {
    if (!classesJson || typeof classesJson !== 'object') {
        return;
    }

    const applyInstanceSections = (instances, sections) => {
        (instances || []).forEach((instance, index) => {
            const i = index + 1;
            sections.forEach((section) => {
                if (section.highSpeedOnly && !isHighSpeedEnabled) {
                    return;
                }
                applyObjectToPrefixedFields(instance[section.prop], `${section.prefix}-${i}`);
            });
        });
    };

    const hidInstances = classesJson.hid && Array.isArray(classesJson.hid.instances) ? classesJson.hid.instances : [];
    hidInstances.forEach((instance, index) => {
        const i = index + 1;
        applyObjectToPrefixedFields(instance.interfaceDescriptor, `interface-${i}`);
        applyObjectToPrefixedFields(instance.hidDescriptor, `hid-details-${i}`);
        applyObjectToPrefixedFields(instance.endpointInFS, `endpoint-${i}`);
        if (isHighSpeedEnabled) {
            applyObjectToPrefixedFields(instance.endpointInHS, `endpoint-hs-${i}`);
        }

        const wantsOutEndpoint = !!instance.optionalOutEndpointEnabled;
        if (wantsOutEndpoint && isProtocolNone(i)) {
            optionalEndpointState[i] = true;
            renderEndpointOutDescriptorFields(i);
            renderEndpointOutDescriptorFieldsHighSpeed(i);
            updateOptionalEndpointUI(i);

            applyObjectToPrefixedFields(instance.endpointOutFS, `endpoint-out-${i}`);
            if (isHighSpeedEnabled) {
                applyObjectToPrefixedFields(instance.endpointOutHS, `endpoint-out-hs-${i}`);
            }
        }
    });

    applyInstanceSections(classesJson.massStorage && classesJson.massStorage.instances, [
        { prop: 'interfaceConfig', prefix: 'msc-config' },
        { prop: 'bulkInFS', prefix: 'msc-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'msc-bulk-out-fs' },
        { prop: 'bulkInHS', prefix: 'msc-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'msc-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.dfu && classesJson.dfu.instances, [
        { prop: 'interfaceConfig', prefix: 'dfu-config' }
    ]);

    applyInstanceSections(classesJson.printer && classesJson.printer.instances, [
        { prop: 'interfaceConfig', prefix: 'printer-config' },
        { prop: 'bulkInFS', prefix: 'printer-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'printer-bulk-out-fs' },
        { prop: 'bulkInHS', prefix: 'printer-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'printer-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.video && classesJson.video.instances, [
        { prop: 'interfaceConfig', prefix: 'video-config' },
        { prop: 'bulkInFS', prefix: 'video-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'video-bulk-out-fs' },
        { prop: 'bulkInHS', prefix: 'video-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'video-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.mtp && classesJson.mtp.instances, [
        { prop: 'interfaceConfig', prefix: 'mtp-config' },
        { prop: 'bulkInFS', prefix: 'mtp-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'mtp-bulk-out-fs' },
        { prop: 'bulkInHS', prefix: 'mtp-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'mtp-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.ptp && classesJson.ptp.instances, [
        { prop: 'interfaceConfig', prefix: 'ptp-config' },
        { prop: 'bulkInFS', prefix: 'ptp-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'ptp-bulk-out-fs' },
        { prop: 'bulkInHS', prefix: 'ptp-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'ptp-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.cdcAcm && classesJson.cdcAcm.instances, [
        { prop: 'interfaceConfig', prefix: 'cdc-config' },
        { prop: 'notifyFS', prefix: 'cdc-notify-fs' },
        { prop: 'bulkInFS', prefix: 'cdc-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'cdc-bulk-out-fs' },
        { prop: 'notifyHS', prefix: 'cdc-notify-hs', highSpeedOnly: true },
        { prop: 'bulkInHS', prefix: 'cdc-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'cdc-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.rndis && classesJson.rndis.instances, [
        { prop: 'interfaceConfig', prefix: 'rndis-config' },
        { prop: 'notifyFS', prefix: 'rndis-notify-fs' },
        { prop: 'bulkInFS', prefix: 'rndis-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'rndis-bulk-out-fs' },
        { prop: 'notifyHS', prefix: 'rndis-notify-hs', highSpeedOnly: true },
        { prop: 'bulkInHS', prefix: 'rndis-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'rndis-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.ecm && classesJson.ecm.instances, [
        { prop: 'interfaceConfig', prefix: 'ecm-config' },
        { prop: 'notifyFS', prefix: 'ecm-notify-fs' },
        { prop: 'bulkInFS', prefix: 'ecm-bulk-in-fs' },
        { prop: 'bulkOutFS', prefix: 'ecm-bulk-out-fs' },
        { prop: 'notifyHS', prefix: 'ecm-notify-hs', highSpeedOnly: true },
        { prop: 'bulkInHS', prefix: 'ecm-bulk-in-hs', highSpeedOnly: true },
        { prop: 'bulkOutHS', prefix: 'ecm-bulk-out-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.audio10 && classesJson.audio10.instances, [
        { prop: 'interfaceConfig', prefix: 'audio-config' },
        { prop: 'endpointFS', prefix: 'audio-ep-fs' },
        { prop: 'endpointHS', prefix: 'audio-ep-hs', highSpeedOnly: true }
    ]);

    applyInstanceSections(classesJson.audio20 && classesJson.audio20.instances, [
        { prop: 'interfaceConfig', prefix: 'audio2-config' },
        { prop: 'endpointFS', prefix: 'audio2-ep-fs' },
        { prop: 'endpointHS', prefix: 'audio2-ep-hs', highSpeedOnly: true }
    ]);
}

function loadJSON() {
    // Create a file input element dynamically
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';

    input.onchange = (event) => {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                const json = JSON.parse(e.target.result);
                const classMappings = [
                    { jsonKey: 'hid', className: 'hid' },
                    { jsonKey: 'massStorage', className: 'msc' },
                    { jsonKey: 'dfu', className: 'dfu' },
                    { jsonKey: 'printer', className: 'printer' },
                    { jsonKey: 'video', className: 'video' },
                    { jsonKey: 'mtp', className: 'mtp' },
                    { jsonKey: 'ptp', className: 'ptp' },
                    { jsonKey: 'cdcAcm', className: 'cdc' },
                    { jsonKey: 'rndis', className: 'rndis' },
                    { jsonKey: 'ecm', className: 'ecm' },
                    { jsonKey: 'audio10', className: 'audio' },
                    { jsonKey: 'audio20', className: 'audio2' }
                ];

                document.getElementById('descriptorForm').reset();
                Object.keys(optionalEndpointState).forEach((key) => {
                    delete optionalEndpointState[key];
                });

                if (Array.isArray(json.classOrder) && json.classOrder.length > 0) {
                    const valid = json.classOrder.filter((className) => CLASS_CARD_CONFIG[className]);
                    const missing = CLASS_ORDER_DEFAULT.filter((className) => !valid.includes(className));
                    classCardOrder = valid.concat(missing);
                    applyClassCardOrder();
                }

                // Start from a clean class selection state.
                classMappings.forEach((mapping) => {
                    const checkbox = document.getElementById(`class-${mapping.className}`);
                    if (checkbox) {
                        checkbox.checked = false;
                        toggleUSBClass(mapping.className);
                    }
                });

                const deviceJson = json.device || json;
                const configurationJson = json.configuration || {};
                const classesJson = json.classes || {};
                const hsEnabled = !!(
                    (json.speeds && json.speeds.highSpeed) ||
                    (configurationJson && configurationJson.highSpeed)
                );

                const speedHighCheckbox = document.getElementById('speed-high');
                if (speedHighCheckbox) {
                    speedHighCheckbox.checked = hsEnabled;
                    toggleUSBSpeed('high');
                }

                Object.entries(deviceJson).forEach(([key, value]) => {
                    setElementValueFromJson(key, value);
                });

                const configFieldKeys = ['iConfiguration', 'configurationString', 'bmAttributes', 'bMaxPower'];
                configFieldKeys.forEach((key) => {
                    setElementValueFromJson(key, configurationJson[key]);
                });

                if (configurationJson.highSpeed && typeof configurationJson.highSpeed === 'object') {
                    setElementValueFromJson('iConfigurationHS', configurationJson.highSpeed.iConfigurationHS);
                    setElementValueFromJson('configurationStringHS', configurationJson.highSpeed.configurationStringHS);
                }

                if (Array.isArray(json.languageIds)) {
                    const languageIdMap = {
                        '0x0409': 'lang-en-us',
                        '0x0809': 'lang-en-uk',
                        '0x040C': 'lang-fr',
                        '0x0407': 'lang-de',
                        '0x0C0A': 'lang-es',
                        '0x0410': 'lang-it',
                        '0x0411': 'lang-ja',
                        '0x0804': 'lang-zh-cn'
                    };

                    Object.values(languageIdMap).forEach((checkboxId) => setElementValueFromJson(checkboxId, false));
                    json.languageIds.forEach((lang) => {
                        if (!lang || !lang.code) {
                            return;
                        }

                        const checkboxId = languageIdMap[String(lang.code).toUpperCase()];
                        if (checkboxId) {
                            setElementValueFromJson(checkboxId, true);
                        }
                    });
                }

                classMappings.forEach((mapping) => {
                    const classEntry = classesJson[mapping.jsonKey];
                    const instances = classEntry && Array.isArray(classEntry.instances) ? classEntry.instances : null;
                    if (!instances || instances.length === 0) {
                        return;
                    }

                    const countInput = document.getElementById(`class-num-${mapping.className}`);
                    if (countInput) {
                        countInput.value = String(instances.length);
                    }

                    const checkbox = document.getElementById(`class-${mapping.className}`);
                    if (checkbox) {
                        checkbox.checked = true;
                        toggleUSBClass(mapping.className);
                    }
                });

                applySavedClassInstances(classesJson, hsEnabled);

                // Restore custom string descriptors
                customStringDescriptors = [];
                customStringNextId = 1;
                if (Array.isArray(json.customStrings)) {
                    json.customStrings.forEach((csd) => {
                        if (csd && typeof csd.index === 'number' && typeof csd.text === 'string') {
                            customStringDescriptors.push({
                                id: customStringNextId++,
                                index: csd.index,
                                text: csd.text,
                                languageId: csd.languageId || ''
                            });
                        }
                    });
                }
                renderCustomStringDescriptorCard();

                updateStringLanguageFields();
                updateConditionalFieldsVisibility();
                updateClassReorderButtonsState();
                enforceFixedStringIndexes();
                generateDescriptor();
                alert('JSON loaded successfully.');
            } catch (error) {
                alert('Error parsing JSON: ' + error.message);
            }
        };
        reader.readAsText(file);
    };

    input.click();
}

function loadTemplate() {
    // Create a modal-like selection dialog
    const templates = {
        'HID Mouse': {
            idVendor: '0x046D',
            idProduct: '0xC077',
            bcdUSB: '0x0200',
            bDeviceClass: '0x00',
            bDeviceSubClass: '0x00',
            bDeviceProtocol: '0x00',
            bMaxPacketSize0: '8',
            bcdDevice: '0x0110',
            iManufacturer: '1',
            iProduct: '2',
            iSerialNumber: '3',
            bNumConfigurations: '1'
        },
        'HID Keyboard': {
            idVendor: '0x046D',
            idProduct: '0xC31C',
            bcdUSB: '0x0200',
            bDeviceClass: '0x00',
            bDeviceSubClass: '0x00',
            bDeviceProtocol: '0x00',
            bMaxPacketSize0: '8',
            bcdDevice: '0x0110',
            iManufacturer: '1',
            iProduct: '2',
            iSerialNumber: '3',
            bNumConfigurations: '1'
        },
        'CDC Serial': {
            idVendor: '0x2341',
            idProduct: '0x0043',
            bcdUSB: '0x0200',
            bDeviceClass: '0x02',
            bDeviceSubClass: '0x00',
            bDeviceProtocol: '0x00',
            bMaxPacketSize0: '64',
            bcdDevice: '0x0100',
            iManufacturer: '1',
            iProduct: '2',
            iSerialNumber: '3',
            bNumConfigurations: '1'
        }
    };

    const templateNames = Object.keys(templates);
    const choice = prompt('Select a template:\n' + templateNames.map((name, idx) => `${idx + 1}. ${name}`).join('\n'));

    if (!choice) return;

    const index = parseInt(choice) - 1;
    if (index >= 0 && index < templateNames.length) {
        const templateName = templateNames[index];
        const template = templates[templateName];

        // Populate form with template values
        Object.keys(template).forEach(key => {
            const element = document.getElementById(key);
            if (element) {
                element.value = template[key];
            }
        });

        alert(`Template "${templateName}" loaded successfully!`);
        generateDescriptor();
    } else {
        alert('Invalid selection');
    }
}

function toggleUSBClass(className) {
    const checkbox = document.getElementById('class-' + className);
    const numberInput = document.getElementById('class-num-' + className);

    if (checkbox.checked) {
        if (className === 'dfu') {
            numberInput.value = '1';
            numberInput.disabled = true;
        } else {
            numberInput.disabled = false;
        }
        console.log('USB Class ' + className + ' is enabled with number: ' + numberInput.value);

        // Show HID descriptor section if HID is enabled
        if (className === 'hid') {
            document.getElementById('hidDescriptorCard').style.display = 'block';
            renderHIDDescriptorFields();
        }
        // Show CDC ACM configuration section if CDC is enabled
        else if (className === 'cdc') {
            document.getElementById('cdcDescriptorCard').style.display = 'block';
            renderCDCACMDescriptorFields();
        }
        // Show CDC RNDIS configuration section if RNDIS is enabled
        else if (className === 'rndis') {
            document.getElementById('cdcRndisDescriptorCard').style.display = 'block';
            renderCDCRNDISDescriptorFields();
        }
        // Show CDC ECM configuration section if ECM is enabled
        else if (className === 'ecm') {
            document.getElementById('cdcEcmDescriptorCard').style.display = 'block';
            renderCDCECMDescriptorFields();
        }
        // Show Audio 1.0 configuration section if Audio is enabled
        else if (className === 'audio') {
            document.getElementById('audioDescriptorCard').style.display = 'block';
            renderAudioDescriptorFields();
        }
        // Show Audio 2.0 configuration section if Audio 2.0 is enabled
        else if (className === 'audio2') {
            document.getElementById('audio2DescriptorCard').style.display = 'block';
            renderAudio2DescriptorFields();
        }
        // Show Mass Storage configuration section if MSC is enabled
        else if (className === 'msc') {
            document.getElementById('massStorageDescriptorCard').style.display = 'block';
            renderMassStorageDescriptorFields();
        }
        // Show DFU configuration section if DFU is enabled
        else if (className === 'dfu') {
            document.getElementById('dfuDescriptorCard').style.display = 'block';
            renderDFUDescriptorFields();
        }
        // Show Printer configuration section if Printer is enabled
        else if (className === 'printer') {
            document.getElementById('printerDescriptorCard').style.display = 'block';
            renderPrinterDescriptorFields();
        }
        // Show Video configuration section if Video is enabled
        else if (className === 'video') {
            document.getElementById('videoDescriptorCard').style.display = 'block';
            renderVideoDescriptorFields();
        }
        // Show MTP configuration section if MTP is enabled
        else if (className === 'mtp') {
            document.getElementById('mtpDescriptorCard').style.display = 'block';
            renderMTPDescriptorFields();
        }
        // Show PTP configuration section if PTP is enabled
        else if (className === 'ptp') {
            document.getElementById('ptpDescriptorCard').style.display = 'block';
            renderPTPDescriptorFields();
        }
    } else {
        if (className === 'dfu') {
            numberInput.value = '1';
        }
        numberInput.disabled = true;
        console.log('USB Class ' + className + ' is disabled');

        // Hide HID descriptor section if HID is disabled
        if (className === 'hid') {
            document.getElementById('hidDescriptorCard').style.display = 'none';
        }
        // Hide CDC ACM configuration section if CDC is disabled
        else if (className === 'cdc') {
            document.getElementById('cdcDescriptorCard').style.display = 'none';
        }
        // Hide CDC RNDIS configuration section if RNDIS is disabled
        else if (className === 'rndis') {
            document.getElementById('cdcRndisDescriptorCard').style.display = 'none';
        }
        // Hide CDC ECM configuration section if ECM is disabled
        else if (className === 'ecm') {
            document.getElementById('cdcEcmDescriptorCard').style.display = 'none';
        }
        // Hide Audio 1.0 configuration section if Audio is disabled
        else if (className === 'audio') {
            document.getElementById('audioDescriptorCard').style.display = 'none';
        }
        // Hide Audio 2.0 configuration section if Audio 2.0 is disabled
        else if (className === 'audio2') {
            document.getElementById('audio2DescriptorCard').style.display = 'none';
        }
        // Hide Mass Storage configuration section if MSC is disabled
        else if (className === 'msc') {
            document.getElementById('massStorageDescriptorCard').style.display = 'none';
        }
        // Hide DFU configuration section if DFU is disabled
        else if (className === 'dfu') {
            document.getElementById('dfuDescriptorCard').style.display = 'none';
        }
        // Hide Printer configuration section if Printer is disabled
        else if (className === 'printer') {
            document.getElementById('printerDescriptorCard').style.display = 'none';
        }
        // Hide Video configuration section if Video is disabled
        else if (className === 'video') {
            document.getElementById('videoDescriptorCard').style.display = 'none';
        }
        // Hide MTP configuration section if MTP is disabled
        else if (className === 'mtp') {
            document.getElementById('mtpDescriptorCard').style.display = 'none';
        }
        // Hide PTP configuration section if PTP is disabled
        else if (className === 'ptp') {
            document.getElementById('ptpDescriptorCard').style.display = 'none';
        }
    }

    setClassConfigCardExpanded(className, checkbox.checked);

    updateClassReorderButtonsState();
}

function toggleUSBSpeed(speedType) {
    const checkbox = document.getElementById('speed-' + speedType);
    console.log('USB Speed ' + speedType + ' is now: ' + (checkbox.checked ? 'enabled' : 'disabled'));

    // Update speed-specific endpoint card visibility in place.
    updateHighSpeedEndpointCardsVisibility();

    // Update conditional field visibility (e.g., High Speed bMaxPacketSize0)
    updateConditionalFieldsVisibility();
}

function isProtocolNone(instanceNum) {
    const protocolSelect = document.getElementById(`interface-${instanceNum}-bInterfaceProtocol`);
    return !!(protocolSelect && protocolSelect.value === '0x00');
}

function updateOptionalEndpointUI(instanceNum) {
    const addButton = document.getElementById(`add-endpoint-btn-${instanceNum}`);
    const outCard = document.getElementById(`endpoint-out-card-${instanceNum}`);
    const removeButton = document.getElementById(`remove-endpoint-btn-${instanceNum}`);
    const canUseOutEndpoint = isProtocolNone(instanceNum);

    if (addButton) {
        addButton.style.display = canUseOutEndpoint && !optionalEndpointState[instanceNum] ? 'inline-block' : 'none';
    }

    if (outCard) {
        outCard.style.display = canUseOutEndpoint && !!optionalEndpointState[instanceNum] ? 'block' : 'none';
    }

    if (removeButton) {
        removeButton.style.display = canUseOutEndpoint && !!optionalEndpointState[instanceNum] ? 'inline-block' : 'none';
    }
}

function addAudioAcEndpoint(instanceNum) {
    audioAcEpState[instanceNum] = true;
    renderAudioAcEndpointFieldsFS(instanceNum);
    renderAudioAcEndpointFieldsHS(instanceNum);
    updateAudioAcEpUI(instanceNum);
    updateHighSpeedEndpointCardsVisibility();
    generateDescriptor();
}

function removeAudioAcEndpoint(instanceNum) {
    audioAcEpState[instanceNum] = false;
    const elFS = document.getElementById(`audio-ac-ep-fs-fields-${instanceNum}`);
    if (elFS) elFS.innerHTML = '';
    const elHS = document.getElementById(`audio-ac-ep-hs-fields-${instanceNum}`);
    if (elHS) elHS.innerHTML = '';
    updateAudioAcEpUI(instanceNum);
    generateDescriptor();
}

function updateAudioAcEpUI(instanceNum) {
    const addBtn    = document.getElementById(`audio-ac-add-ep-btn-${instanceNum}`);
    const epCard    = document.getElementById(`audio-ac-ep-card-${instanceNum}`);
    const removeBtn = document.getElementById(`audio-ac-remove-ep-btn-${instanceNum}`);
    const enabled   = !!audioAcEpState[instanceNum];
    if (addBtn)    addBtn.style.display    = enabled ? 'none'         : 'inline-block';
    if (epCard)    epCard.style.display    = enabled ? 'block'        : 'none';
    if (removeBtn) removeBtn.style.display = enabled ? 'inline-block' : 'none';
}

function renderAudioAcEndpointFieldsFS(instanceNum) {
    const target = document.getElementById(`audio-ac-ep-fs-fields-${instanceNum}`);
    if (!target) return;
    const fields = [
        {
            id: `audio-ac-ep-fs-${instanceNum}-epAddressFS`,
            label: `Endpoint Address (bEndpointAddress) ${createInfoIcon('Interrupt IN endpoint address, e.g. 0x83.')}`,
            type: 'text', value: '0x83', placeholder: '0x83'
        },
        {
            id: `audio-ac-ep-fs-${instanceNum}-epMaxPacketFS`,
            label: `Max Packet Size (wMaxPacketSize) ${createInfoIcon('Maximum payload bytes per interrupt transfer. Typically 2–8 bytes for UAC1 status interrupts.')}`,
            type: 'number', value: '2', min: 1, max: 64, placeholder: '2'
        },
        {
            id: `audio-ac-ep-fs-${instanceNum}-epIntervalFS`,
            label: `Polling Interval (bInterval) ${createInfoIcon('Polling interval in milliseconds (1–255 for FS).')}`,
            type: 'number', value: '16', min: 1, max: 255, placeholder: '16'
        }
    ];
    target.innerHTML = fields.map((f) => `
        <div class="form-group">
            <label for="${f.id}">${f.label}</label>
            <input type="${f.type}" id="${f.id}" value="${f.value}" placeholder="${f.placeholder}"
                ${f.min !== undefined ? `min="${f.min}"` : ''}
                ${f.max !== undefined ? `max="${f.max}"` : ''}>
        </div>`).join('');
    target.querySelectorAll('input').forEach((el) => el.addEventListener('input', generateDescriptor));
}

function renderAudioAcEndpointFieldsHS(instanceNum) {
    const target = document.getElementById(`audio-ac-ep-hs-fields-${instanceNum}`);
    if (!target) return;
    const fields = [
        {
            id: `audio-ac-ep-hs-${instanceNum}-epAddressHS`,
            label: `Endpoint Address (bEndpointAddress) ${createInfoIcon('Interrupt IN endpoint address, e.g. 0x83.')}`,
            type: 'text', value: '0x83', placeholder: '0x83'
        },
        {
            id: `audio-ac-ep-hs-${instanceNum}-epMaxPacketHS`,
            label: `Max Packet Size (wMaxPacketSize) ${createInfoIcon('Maximum payload bytes per interrupt transfer. Typically 2–8 bytes for UAC1 status interrupts.')}`,
            type: 'number', value: '2', min: 1, max: 1024, placeholder: '2'
        },
        {
            id: `audio-ac-ep-hs-${instanceNum}-epIntervalHS`,
            label: `Polling Interval (bInterval) ${createInfoIcon('Polling interval in 125 µs microframes for HS (1–16).')}`,
            type: 'number', value: '6', min: 1, max: 16, placeholder: '6'
        }
    ];
    target.innerHTML = fields.map((f) => `
        <div class="form-group">
            <label for="${f.id}">${f.label}</label>
            <input type="${f.type}" id="${f.id}" value="${f.value}" placeholder="${f.placeholder}"
                ${f.min !== undefined ? `min="${f.min}"` : ''}
                ${f.max !== undefined ? `max="${f.max}"` : ''}>
        </div>`).join('');
    target.querySelectorAll('input').forEach((el) => el.addEventListener('input', generateDescriptor));
}

function addOptionalEndpoint(instanceNum) {
    renderEndpointOutDescriptorFields(instanceNum);
    renderEndpointOutDescriptorFieldsHighSpeed(instanceNum);
    updateOptionalEndpointUI(instanceNum);
    generateDescriptor();
}

function removeOptionalEndpoint(instanceNum) {
    optionalEndpointState[instanceNum] = false;
    const outFields = document.getElementById(`endpoint-out-form-fields-${instanceNum}`);
    if (outFields) {
        outFields.innerHTML = '';
    }
    const outFieldsHS = document.getElementById(`endpoint-out-form-fields-hs-${instanceNum}`);
    if (outFieldsHS) {
        outFieldsHS.innerHTML = '';
    }
    updateOptionalEndpointUI(instanceNum);
    generateDescriptor();
}

function updateHighSpeedEndpointCardsVisibility() {
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    document.querySelectorAll('.endpoint-high-speed-card').forEach((card) => {
        card.style.display = isHighSpeedEnabled ? 'block' : 'none';
    });
}

function formatClassInstanceCardTitle(classLabel, instanceIndex, instanceCount) {
    if (instanceCount > 1) {
        return `${classLabel} - ${instanceIndex} Configuration`;
    }

    return `${classLabel} Configuration`;
}

function renderHIDDescriptorFields() {
    const hidNumberInput = document.getElementById('class-num-hid');
    const classNumber = parseInt(hidNumberInput.value);
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseHIDInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('HID', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseHIDInstance${i}" class="collapse">
                    <div class="card-body">
                        <!-- Interface Descriptor Card -->
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseInterface${i}', this)" aria-expanded="false">
                                        Interface Descriptor
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseInterface${i}" class="collapse">
                                <div class="card-body">
                                    <div id="interface-form-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <!-- HID Configuration Card -->
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseHIDDetails${i}', this)" aria-expanded="false">
                                        HID Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseHIDDetails${i}" class="collapse">
                                <div class="card-body">
                                    <div id="hid-details-form-fields-${i}"></div>

                                    <div id="hid-descriptors-${i}" style="margin-top: 15px;"></div>
                                </div>
                            </div>
                        </div>

                        <!-- Endpoint IN Descriptor Card -->
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseEndpoint${i}', this)" aria-expanded="false">
                                        Endpoint IN Descriptor
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseEndpoint${i}" class="collapse">
                                <div class="card-body">
                                    <div class="card" style="margin-bottom: 0;">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseEndpointFullSpeed${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - Full Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseEndpointFullSpeed${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="endpoint-form-fields-${i}"></div>
                                            </div>
                                        </div>
                                    </div>

                                    <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseEndpointHighSpeed${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - High Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseEndpointHighSpeed${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="endpoint-form-fields-hs-${i}"></div>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>

                        <button id="add-endpoint-btn-${i}" class="btn btn-sm" type="button" onclick="addOptionalEndpoint(${i})" style="display: none; margin-top: -4px; margin-bottom: 12px; padding: 4px 10px; font-size: 0.85em;">
                            + add endpoint
                        </button>

                        <div id="endpoint-out-card-${i}" class="card" style="margin-bottom: 15px; display: none;">
                            <div class="card-header">
                                <h3 class="mb-0" style="display: flex; align-items: center; justify-content: space-between; gap: 8px;">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseEndpointOut${i}', this)" aria-expanded="false" style="flex: 1; text-align: left;">
                                        Endpoint OUT Descriptor
                                    </button>
                                    <button id="remove-endpoint-btn-${i}" class="btn btn-sm" type="button" onclick="removeOptionalEndpoint(${i})" style="display: none; flex: 0 0 auto; margin: 6px 8px 6px 0; padding: 4px 8px; font-size: 0.75em;">
                                        remove
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseEndpointOut${i}" class="collapse">
                                <div class="card-body">
                                    <div class="card" style="margin-bottom: 0;">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseEndpointOutFullSpeed${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - Full Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseEndpointOutFullSpeed${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="endpoint-out-form-fields-${i}"></div>
                                            </div>
                                        </div>
                                    </div>

                                    <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseEndpointOutHighSpeed${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - High Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseEndpointOutHighSpeed${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="endpoint-out-form-fields-hs-${i}"></div>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('hid-descriptor-fields');
    container.innerHTML = html;

    // Render the 3 descriptor form fields for each instance
    for (let i = 1; i <= classNumber; i++) {
        renderInterfaceDescriptorFields(i);
        renderHIDDetailsFields(i);
        renderEndpointInDescriptorFields(i);
        renderEndpointInDescriptorFieldsHighSpeed(i);

        if (optionalEndpointState[i]) {
            renderEndpointOutDescriptorFields(i);
            renderEndpointOutDescriptorFieldsHighSpeed(i);
        }

        // Add event listener for bNumDescriptors to render descriptor entries
        const numDescsInput = document.getElementById(`hid-details-${i}-hidBNumDescriptors`);
        if (numDescsInput) {
            numDescsInput.addEventListener('change', () => {
                renderHIDDescriptorEntries(i);
            });
            renderHIDDescriptorEntries(i);
        }

        const protocolSelect = document.getElementById(`interface-${i}-bInterfaceProtocol`);
        if (protocolSelect) {
            protocolSelect.addEventListener('change', () => {
                if (!isProtocolNone(i) && optionalEndpointState[i]) {
                    optionalEndpointState[i] = false;
                }
                updateOptionalEndpointUI(i);
                generateDescriptor();
            });
            updateOptionalEndpointUI(i);
        }
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderHIDDescriptorEntries(classInstance) {
    const numDescsInput = document.getElementById(`hid-details-${classInstance}-hidBNumDescriptors`);
    if (!numDescsInput) return;
    const numDescriptors = parseInt(numDescsInput.value) || 1;
    let html = '';

    for (let j = 1; j <= numDescriptors; j++) {
        html += `
            <div style="padding: 10px; background: #f8f9fa; margin-bottom: 10px; border-radius: 4px;">
                <label>Descriptor ${j}:</label>
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 8px;">
                    <div>
                        <label for="hid-desc-type-${classInstance}-${j}" style="font-size: 0.9em;">Type</label>
                        <select id="hid-desc-type-${classInstance}-${j}">
                            <option value="0x21">0x21 - HID Descriptor</option>
                            <option value="0x22">0x22 - Report Descriptor</option>
                            <option value="0x23">0x23 - Physical Descriptor</option>
                        </select>
                    </div>
                    <div>
                        <label for="hid-desc-length-${classInstance}-${j}" style="font-size: 0.9em;">Length (wDescriptorLength)</label>
                        <input type="number" id="hid-desc-length-${classInstance}-${j}" min="0" max="65535" value="100" style="width: 100%;">
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById(`hid-descriptors-${classInstance}`);
    container.innerHTML = html;
}

function renderMassStorageDescriptorFields() {
    const mscNumberInput = document.getElementById('class-num-msc');
    const classNumber = parseInt(mscNumberInput.value);
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('Mass Storage', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseMSCInstance${i}" class="collapse">
                    <div class="card-body">
                        <!-- Mass Storage Configuration Card -->
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCConfig${i}', this)" aria-expanded="false">
                                        Mass Storage Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseMSCConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="msc-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <!-- Bulk IN Endpoint Descriptor Card -->
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCBulkIn${i}', this)" aria-expanded="false">
                                        Bulk IN Endpoint Descriptor
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseMSCBulkIn${i}" class="collapse">
                                <div class="card-body">
                                    <div class="card" style="margin-bottom: 0;">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCBulkInFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - Full Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseMSCBulkInFS${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="msc-bulk-in-fs-fields-${i}"></div>
                                            </div>
                                        </div>
                                    </div>

                                    <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCBulkInHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - High Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseMSCBulkInHS${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="msc-bulk-in-hs-fields-${i}"></div>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>

                        <!-- Bulk OUT Endpoint Descriptor Card -->
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCBulkOut${i}', this)" aria-expanded="false">
                                        Bulk OUT Endpoint Descriptor
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseMSCBulkOut${i}" class="collapse">
                                <div class="card-body">
                                    <div class="card" style="margin-bottom: 0;">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCBulkOutFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - Full Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseMSCBulkOutFS${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="msc-bulk-out-fs-fields-${i}"></div>
                                            </div>
                                        </div>
                                    </div>

                                    <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                        <div class="card-header">
                                            <h3 class="mb-0">
                                                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMSCBulkOutHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">
                                                    Endpoint - High Speed
                                                </button>
                                            </h3>
                                        </div>
                                        <div id="collapseMSCBulkOutHS${i}" class="collapse">
                                            <div class="card-body">
                                                <div id="msc-bulk-out-hs-fields-${i}"></div>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('mass-storage-descriptor-fields');
    container.innerHTML = html;

    // Render the configuration and endpoint fields for each instance
    for (let i = 1; i <= classNumber; i++) {
        renderMassStorageConfigFields(i);
        renderMassStorageBulkInFieldsFS(i);
        renderMassStorageBulkInFieldsHS(i);
        renderMassStorageBulkOutFieldsFS(i);
        renderMassStorageBulkOutFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderMassStorageConfigFields(instanceNum) {
    const fieldsForTemplate = MASS_STORAGE_DESC_SCHEMA.fields.map((field) => ({
        ...field,
        id: `msc-config-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`msc-config-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderMassStorageBulkInFieldsFS(instanceNum) {
    const endpointFields = [
        {
            id: 'mscBulkInEndpointFS',
            label: 'Bulk IN Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address (e.g., 0x81 for endpoint 1)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'mscBulkInMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'mscBulkInIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for bulk endpoint. Typically 0 for Full Speed bulk endpoints.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `msc-bulk-in-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`msc-bulk-in-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderMassStorageBulkInFieldsHS(instanceNum) {
    const endpointFields = [
        {
            id: 'mscBulkInEndpointHS',
            label: 'Bulk IN Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address (e.g., 0x81 for endpoint 1)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'mscBulkInMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0200', text: '512 bytes (0x0200)' }
            ]
        },
        {
            id: 'mscBulkInIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for bulk endpoint. For High Speed bulk, this is the NAK rate (0 = as fast as possible).',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `msc-bulk-in-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`msc-bulk-in-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderMassStorageBulkOutFieldsFS(instanceNum) {
    const endpointFields = [
        {
            id: 'mscBulkOutEndpointFS',
            label: 'Bulk OUT Endpoint Address (bEndpointAddress)',
            help: 'OUT endpoint address (e.g., 0x01 for endpoint 1)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            placeholder: '0x01'
        },
        {
            id: 'mscBulkOutMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'mscBulkOutIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for bulk endpoint. Typically 0 for Full Speed bulk endpoints.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `msc-bulk-out-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`msc-bulk-out-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderMassStorageBulkOutFieldsHS(instanceNum) {
    const endpointFields = [
        {
            id: 'mscBulkOutEndpointHS',
            label: 'Bulk OUT Endpoint Address (bEndpointAddress)',
            help: 'OUT endpoint address (e.g., 0x01 for endpoint 1)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x01',
            placeholder: '0x01'
        },
        {
            id: 'mscBulkOutMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0200', text: '512 bytes (0x0200)' }
            ]
        },
        {
            id: 'mscBulkOutIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for bulk endpoint. For High Speed bulk, this is the NAK rate (0 = as fast as possible).',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `msc-bulk-out-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`msc-bulk-out-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderDFUDescriptorFields() {
    const dfuNumberInput = document.getElementById('class-num-dfu');
    const classNumber = parseInt(dfuNumberInput.value) || 1;
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseDFUInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('DFU', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseDFUInstance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseDFUConfig${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        DFU Interface and Functional Descriptor Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseDFUConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="dfu-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('dfu-descriptor-fields');
    if (container) {
        container.innerHTML = html;
    }

    for (let i = 1; i <= classNumber; i++) {
        const fieldsForTemplate = DFU_DESC_SCHEMA.fields.map((field) => ({
            ...field,
            id: `dfu-config-${i}-${field.id}`,
            isSelect: field.type === 'select',
            inputType: field.type === 'number' ? 'number' : 'text',
            hasMin: typeof field.min === 'number',
            hasMax: typeof field.max === 'number'
        }));

        const configHtml = templates.field({ fields: fieldsForTemplate });
        const target = document.getElementById(`dfu-config-fields-${i}`);
        if (target) {
            target.innerHTML = configHtml;
        }

        // Set up conditional visibility
        updateClassConditionalFieldsVisibility(i, DFU_DESC_SCHEMA, 'dfu-config');

        // Add change listener for dfuIInterface field
        const dfuIInterfaceElement = document.getElementById(`dfu-config-${i}-dfuIInterface`);
        if (dfuIInterfaceElement) {
            dfuIInterfaceElement.addEventListener('change', () => {
                updateClassConditionalFieldsVisibility(i, DFU_DESC_SCHEMA, 'dfu-config');
            });
            dfuIInterfaceElement.addEventListener('input', () => {
                updateClassConditionalFieldsVisibility(i, DFU_DESC_SCHEMA, 'dfu-config');
            });
        }
    }
}

function renderPrinterDescriptorFields() {
    const printerNumberInput = document.getElementById('class-num-printer');
    const classNumber = parseInt(printerNumberInput.value) || 1;
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('Printer', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapsePrinterInstance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterConfig${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Printer Interface Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapsePrinterConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="printer-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterBulkIn${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk IN Endpoint Descriptor</button></h3></div>
                            <div id="collapsePrinterBulkIn${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterBulkInFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapsePrinterBulkInFS${i}" class="collapse"><div class="card-body"><div id="printer-bulk-in-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterBulkInHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapsePrinterBulkInHS${i}" class="collapse"><div class="card-body"><div id="printer-bulk-in-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterBulkOut${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk OUT Endpoint Descriptor</button></h3></div>
                            <div id="collapsePrinterBulkOut${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterBulkOutFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapsePrinterBulkOutFS${i}" class="collapse"><div class="card-body"><div id="printer-bulk-out-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePrinterBulkOutHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapsePrinterBulkOutHS${i}" class="collapse"><div class="card-body"><div id="printer-bulk-out-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('printer-descriptor-fields');
    if (container) {
        container.innerHTML = html;
    }

    for (let i = 1; i <= classNumber; i++) {
        const fieldsForTemplate = PRINTER_DESC_SCHEMA.fields.map((field) => ({
            ...field,
            id: `printer-config-${i}-${field.id}`,
            isSelect: field.type === 'select',
            inputType: field.type === 'number' ? 'number' : 'text',
            hasMin: typeof field.min === 'number',
            hasMax: typeof field.max === 'number'
        }));

        const configHtml = templates.field({ fields: fieldsForTemplate });
        const configTarget = document.getElementById(`printer-config-fields-${i}`);
        if (configTarget) {
            configTarget.innerHTML = configHtml;

                // Set up conditional visibility
                updateClassConditionalFieldsVisibility(i, PRINTER_DESC_SCHEMA, 'printer-config');

                // Add change listener for printerIInterface field
                const printerIInterfaceElement = document.getElementById(`printer-config-${i}-printerIInterface`);
                if (printerIInterfaceElement) {
                    printerIInterfaceElement.addEventListener('change', () => {
                        updateClassConditionalFieldsVisibility(i, PRINTER_DESC_SCHEMA, 'printer-config');
                    });
                    printerIInterfaceElement.addEventListener('input', () => {
                        updateClassConditionalFieldsVisibility(i, PRINTER_DESC_SCHEMA, 'printer-config');
                    });
                }
        }

        renderPrinterBulkInFieldsFS(i);
        renderPrinterBulkInFieldsHS(i);
        renderPrinterBulkOutFieldsFS(i);
        renderPrinterBulkOutFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderPrinterBulkInFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'printerBulkInEndpointFS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x81).', type: 'text', format: 'hex', bytes: 1, default: '0x81', placeholder: '0x81' },
        { id: 'printerBulkInMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'printerBulkInIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `printer-bulk-in-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`printer-bulk-in-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderPrinterBulkInFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'printerBulkInEndpointHS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x81).', type: 'text', format: 'hex', bytes: 1, default: '0x81', placeholder: '0x81' },
        { id: 'printerBulkInMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'printerBulkInIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `printer-bulk-in-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`printer-bulk-in-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderPrinterBulkOutFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'printerBulkOutEndpointFS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x01).', type: 'text', format: 'hex', bytes: 1, default: '0x01', placeholder: '0x01' },
        { id: 'printerBulkOutMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'printerBulkOutIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `printer-bulk-out-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`printer-bulk-out-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderPrinterBulkOutFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'printerBulkOutEndpointHS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x01).', type: 'text', format: 'hex', bytes: 1, default: '0x01', placeholder: '0x01' },
        { id: 'printerBulkOutMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'printerBulkOutIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `printer-bulk-out-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`printer-bulk-out-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderVideoDescriptorFields() {
    const videoNumberInput = document.getElementById('class-num-video');
    const classNumber = parseInt(videoNumberInput.value) || 1;
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('Video', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseVideoInstance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoConfig${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Video Interface Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseVideoConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="video-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoBulkIn${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk IN Endpoint Descriptor</button></h3></div>
                            <div id="collapseVideoBulkIn${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoBulkInFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapseVideoBulkInFS${i}" class="collapse"><div class="card-body"><div id="video-bulk-in-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoBulkInHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapseVideoBulkInHS${i}" class="collapse"><div class="card-body"><div id="video-bulk-in-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoBulkOut${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk OUT Endpoint Descriptor</button></h3></div>
                            <div id="collapseVideoBulkOut${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoBulkOutFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapseVideoBulkOutFS${i}" class="collapse"><div class="card-body"><div id="video-bulk-out-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseVideoBulkOutHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapseVideoBulkOutHS${i}" class="collapse"><div class="card-body"><div id="video-bulk-out-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('video-descriptor-fields');
    if (container) {
        container.innerHTML = html;
    }

    for (let i = 1; i <= classNumber; i++) {
        const fieldsForTemplate = VIDEO_DESC_SCHEMA.fields.map((field) => ({
            ...field,
            id: `video-config-${i}-${field.id}`,
            isSelect: field.type === 'select',
            inputType: field.type === 'number' ? 'number' : 'text',
            hasMin: typeof field.min === 'number',
            hasMax: typeof field.max === 'number'
        }));

        const configHtml = templates.field({ fields: fieldsForTemplate });
        const configTarget = document.getElementById(`video-config-fields-${i}`);

                // Set up conditional visibility
                updateClassConditionalFieldsVisibility(i, VIDEO_DESC_SCHEMA, 'video-config');

                // Add change listener for videoIInterface field
                const videoIInterfaceElement = document.getElementById(`video-config-${i}-videoIInterface`);
                if (videoIInterfaceElement) {
                    videoIInterfaceElement.addEventListener('change', () => {
                        updateClassConditionalFieldsVisibility(i, VIDEO_DESC_SCHEMA, 'video-config');
                    });
                    videoIInterfaceElement.addEventListener('input', () => {
                        updateClassConditionalFieldsVisibility(i, VIDEO_DESC_SCHEMA, 'video-config');
                    });
                }
        if (configTarget) {
            configTarget.innerHTML = configHtml;
        }

        renderVideoBulkInFieldsFS(i);
        renderVideoBulkInFieldsHS(i);
        renderVideoBulkOutFieldsFS(i);
        renderVideoBulkOutFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderVideoBulkInFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'videoBulkInEndpointFS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x83).', type: 'text', format: 'hex', bytes: 1, default: '0x83', placeholder: '0x83' },
        { id: 'videoBulkInMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'videoBulkInIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `video-bulk-in-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`video-bulk-in-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderVideoBulkInFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'videoBulkInEndpointHS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x83).', type: 'text', format: 'hex', bytes: 1, default: '0x83', placeholder: '0x83' },
        { id: 'videoBulkInMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'videoBulkInIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `video-bulk-in-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`video-bulk-in-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderVideoBulkOutFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'videoBulkOutEndpointFS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x03).', type: 'text', format: 'hex', bytes: 1, default: '0x03', placeholder: '0x03' },
        { id: 'videoBulkOutMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'videoBulkOutIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `video-bulk-out-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`video-bulk-out-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderVideoBulkOutFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'videoBulkOutEndpointHS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x03).', type: 'text', format: 'hex', bytes: 1, default: '0x03', placeholder: '0x03' },
        { id: 'videoBulkOutMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'videoBulkOutIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `video-bulk-out-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`video-bulk-out-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderMTPDescriptorFields() {
    const mtpNumberInput = document.getElementById('class-num-mtp');
    const classNumber = parseInt(mtpNumberInput.value) || 1;
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('MTP', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseMTPInstance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPConfig${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        MTP Interface Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseMTPConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="mtp-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPBulkIn${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk IN Endpoint Descriptor</button></h3></div>
                            <div id="collapseMTPBulkIn${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPBulkInFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapseMTPBulkInFS${i}" class="collapse"><div class="card-body"><div id="mtp-bulk-in-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPBulkInHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapseMTPBulkInHS${i}" class="collapse"><div class="card-body"><div id="mtp-bulk-in-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPBulkOut${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk OUT Endpoint Descriptor</button></h3></div>
                            <div id="collapseMTPBulkOut${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPBulkOutFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapseMTPBulkOutFS${i}" class="collapse"><div class="card-body"><div id="mtp-bulk-out-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseMTPBulkOutHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapseMTPBulkOutHS${i}" class="collapse"><div class="card-body"><div id="mtp-bulk-out-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('mtp-descriptor-fields');
    if (container) {
        container.innerHTML = html;
    }

    for (let i = 1; i <= classNumber; i++) {
        const fieldsForTemplate = MTP_DESC_SCHEMA.fields.map((field) => ({
            ...field,
            id: `mtp-config-${i}-${field.id}`,
            isSelect: field.type === 'select',
            inputType: field.type === 'number' ? 'number' : 'text',
            hasMin: typeof field.min === 'number',
            hasMax: typeof field.max === 'number'
        }));

        const configHtml = templates.field({ fields: fieldsForTemplate });
        const configTarget = document.getElementById(`mtp-config-fields-${i}`);

                // Set up conditional visibility
                updateClassConditionalFieldsVisibility(i, MTP_DESC_SCHEMA, 'mtp-config');

                // Add change listener for mtpIInterface field
                const mtpIInterfaceElement = document.getElementById(`mtp-config-${i}-mtpIInterface`);
                if (mtpIInterfaceElement) {
                    mtpIInterfaceElement.addEventListener('change', () => {
                        updateClassConditionalFieldsVisibility(i, MTP_DESC_SCHEMA, 'mtp-config');
                    });
                    mtpIInterfaceElement.addEventListener('input', () => {
                        updateClassConditionalFieldsVisibility(i, MTP_DESC_SCHEMA, 'mtp-config');
                    });
                }
        if (configTarget) {
            configTarget.innerHTML = configHtml;
        }

        renderMTPBulkInFieldsFS(i);
        renderMTPBulkInFieldsHS(i);
        renderMTPBulkOutFieldsFS(i);
        renderMTPBulkOutFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderMTPBulkInFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'mtpBulkInEndpointFS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x84).', type: 'text', format: 'hex', bytes: 1, default: '0x84', placeholder: '0x84' },
        { id: 'mtpBulkInMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'mtpBulkInIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `mtp-bulk-in-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`mtp-bulk-in-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderMTPBulkInFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'mtpBulkInEndpointHS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x84).', type: 'text', format: 'hex', bytes: 1, default: '0x84', placeholder: '0x84' },
        { id: 'mtpBulkInMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'mtpBulkInIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `mtp-bulk-in-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`mtp-bulk-in-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderMTPBulkOutFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'mtpBulkOutEndpointFS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x04).', type: 'text', format: 'hex', bytes: 1, default: '0x04', placeholder: '0x04' },
        { id: 'mtpBulkOutMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'mtpBulkOutIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `mtp-bulk-out-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`mtp-bulk-out-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderMTPBulkOutFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'mtpBulkOutEndpointHS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x04).', type: 'text', format: 'hex', bytes: 1, default: '0x04', placeholder: '0x04' },
        { id: 'mtpBulkOutMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'mtpBulkOutIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `mtp-bulk-out-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`mtp-bulk-out-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderPTPDescriptorFields() {
    const ptpNumberInput = document.getElementById('class-num-ptp');
    const classNumber = parseInt(ptpNumberInput.value) || 1;
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('PTP', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapsePTPInstance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPConfig${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        PTP Interface Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapsePTPConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="ptp-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPBulkIn${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk IN Endpoint Descriptor</button></h3></div>
                            <div id="collapsePTPBulkIn${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPBulkInFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapsePTPBulkInFS${i}" class="collapse"><div class="card-body"><div id="ptp-bulk-in-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPBulkInHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapsePTPBulkInHS${i}" class="collapse"><div class="card-body"><div id="ptp-bulk-in-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPBulkOut${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Bulk OUT Endpoint Descriptor</button></h3></div>
                            <div id="collapsePTPBulkOut${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPBulkOutFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                    <div id="collapsePTPBulkOutFS${i}" class="collapse"><div class="card-body"><div id="ptp-bulk-out-fs-fields-${i}"></div></div></div>
                                </div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapsePTPBulkOutHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                    <div id="collapsePTPBulkOutHS${i}" class="collapse"><div class="card-body"><div id="ptp-bulk-out-hs-fields-${i}"></div></div></div>
                                </div>
                            </div></div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('ptp-descriptor-fields');
    if (container) {
        container.innerHTML = html;
    }

    for (let i = 1; i <= classNumber; i++) {
        const fieldsForTemplate = PTP_DESC_SCHEMA.fields.map((field) => ({
            ...field,
            id: `ptp-config-${i}-${field.id}`,
            isSelect: field.type === 'select',
            inputType: field.type === 'number' ? 'number' : 'text',
            hasMin: typeof field.min === 'number',
            hasMax: typeof field.max === 'number'
        }));

        const configHtml = templates.field({ fields: fieldsForTemplate });
        const configTarget = document.getElementById(`ptp-config-fields-${i}`);

                // Set up conditional visibility
                updateClassConditionalFieldsVisibility(i, PTP_DESC_SCHEMA, 'ptp-config');

                // Add change listener for ptpIInterface field
                const ptpIInterfaceElement = document.getElementById(`ptp-config-${i}-ptpIInterface`);
                if (ptpIInterfaceElement) {
                    ptpIInterfaceElement.addEventListener('change', () => {
                        updateClassConditionalFieldsVisibility(i, PTP_DESC_SCHEMA, 'ptp-config');
                    });
                    ptpIInterfaceElement.addEventListener('input', () => {
                        updateClassConditionalFieldsVisibility(i, PTP_DESC_SCHEMA, 'ptp-config');
                    });
                }
        if (configTarget) {
            configTarget.innerHTML = configHtml;
        }

        renderPTPBulkInFieldsFS(i);
        renderPTPBulkInFieldsHS(i);
        renderPTPBulkOutFieldsFS(i);
        renderPTPBulkOutFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderPTPBulkInFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'ptpBulkInEndpointFS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x85).', type: 'text', format: 'hex', bytes: 1, default: '0x85', placeholder: '0x85' },
        { id: 'ptpBulkInMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'ptpBulkInIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `ptp-bulk-in-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`ptp-bulk-in-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderPTPBulkInFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'ptpBulkInEndpointHS', label: 'Bulk IN Endpoint Address (bEndpointAddress)', help: 'IN endpoint address (e.g., 0x85).', type: 'text', format: 'hex', bytes: 1, default: '0x85', placeholder: '0x85' },
        { id: 'ptpBulkInMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'ptpBulkInIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `ptp-bulk-in-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`ptp-bulk-in-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderPTPBulkOutFieldsFS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'ptpBulkOutEndpointFS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x05).', type: 'text', format: 'hex', bytes: 1, default: '0x05', placeholder: '0x05' },
        { id: 'ptpBulkOutMaxPacketSizeFS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for Full Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0040', options: [ { value: '0x0008', text: '8 bytes (0x0008)' }, { value: '0x0010', text: '16 bytes (0x0010)' }, { value: '0x0020', text: '32 bytes (0x0020)' }, { value: '0x0040', text: '64 bytes (0x0040)' } ] },
        { id: 'ptpBulkOutIntervalFS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `ptp-bulk-out-fs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`ptp-bulk-out-fs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderPTPBulkOutFieldsHS(instanceNum) {
    const fieldsForTemplate = [
        { id: 'ptpBulkOutEndpointHS', label: 'Bulk OUT Endpoint Address (bEndpointAddress)', help: 'OUT endpoint address (e.g., 0x05).', type: 'text', format: 'hex', bytes: 1, default: '0x05', placeholder: '0x05' },
        { id: 'ptpBulkOutMaxPacketSizeHS', label: 'Max Packet Size (wMaxPacketSize)', help: 'Maximum packet size for High Speed bulk endpoint.', type: 'select', format: 'hex', bytes: 2, default: '0x0200', options: [ { value: '0x0200', text: '512 bytes (0x0200)' } ] },
        { id: 'ptpBulkOutIntervalHS', label: 'Polling Interval (bInterval)', help: 'Typically 0 for bulk endpoints.', type: 'number', format: 'hex', bytes: 1, default: '0', min: 0, max: 255 }
    ].map((field) => ({ ...field, id: `ptp-bulk-out-hs-${instanceNum}-${field.id}`, isSelect: field.type === 'select', inputType: field.type === 'number' ? 'number' : 'text', hasMin: typeof field.min === 'number', hasMax: typeof field.max === 'number' }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`ptp-bulk-out-hs-fields-${instanceNum}`);
    if (target) target.innerHTML = html;
}

function renderCDCACMDescriptorFields() {
    const cdcNumberInput = document.getElementById('class-num-cdc');
    const classNumber = parseInt(cdcNumberInput.value);
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('CDC ACM', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseCDCInstance${i}" class="collapse">
                    <div class="card-body">
                        <!-- CDC ACM Configuration Card -->
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCConfig${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Communication Interface Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseCDCConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="cdc-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCNotify${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Notification Endpoint Descriptor
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseCDCNotify${i}" class="collapse">
                                <div class="card-body">
                                    <div class="card" style="margin-bottom: 0;">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCNotifyFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                        <div id="collapseCDCNotifyFS${i}" class="collapse"><div class="card-body"><div id="cdc-notify-fs-fields-${i}"></div></div></div>
                                    </div>
                                    <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCNotifyHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                        <div id="collapseCDCNotifyHS${i}" class="collapse"><div class="card-body"><div id="cdc-notify-hs-fields-${i}"></div></div></div>
                                    </div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCBulkIn${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Data Bulk IN Endpoint Descriptor
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseCDCBulkIn${i}" class="collapse">
                                <div class="card-body">
                                    <div class="card" style="margin-bottom: 0;">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCBulkInFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                        <div id="collapseCDCBulkInFS${i}" class="collapse"><div class="card-body"><div id="cdc-bulk-in-fs-fields-${i}"></div></div></div>
                                    </div>
                                    <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCBulkInHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                        <div id="collapseCDCBulkInHS${i}" class="collapse"><div class="card-body"><div id="cdc-bulk-in-hs-fields-${i}"></div></div></div>
                                    </div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCBulkOut${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Data Bulk OUT Endpoint Descriptor
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseCDCBulkOut${i}" class="collapse">
                                <div class="card-body">
                                    <div class="card" style="margin-bottom: 0;">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCBulkOutFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                                        <div id="collapseCDCBulkOutFS${i}" class="collapse"><div class="card-body"><div id="cdc-bulk-out-fs-fields-${i}"></div></div></div>
                                    </div>
                                    <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseCDCBulkOutHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                                        <div id="collapseCDCBulkOutHS${i}" class="collapse"><div class="card-body"><div id="cdc-bulk-out-hs-fields-${i}"></div></div></div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('cdc-descriptor-fields');
    container.innerHTML = html;

    // Render fields for each instance
    for (let i = 1; i <= classNumber; i++) {
        renderCDCConfigFields(i);
        renderCDCNotifyFieldsFS(i);
        renderCDCNotifyFieldsHS(i);
        renderCDCBulkInFieldsFS(i);
        renderCDCBulkInFieldsHS(i);
        renderCDCBulkOutFieldsFS(i);
        renderCDCBulkOutFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderCDCConfigFields(instanceNum) {
    const fieldsForTemplate = CDC_ACM_DESC_SCHEMA.fields.map((field) => ({
        ...field,
        id: `cdc-config-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`cdc-config-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNotifyFieldsFS(instanceNum) {
    const endpointFields = [
        {
            id: 'cdcNotifyEndpointFS',
            label: 'Notification Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for notifications (e.g., 0x81)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'cdcNotifyMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed interrupt endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0008',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'cdcNotifyIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for interrupt endpoint (in milliseconds for FS)',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '16',
            min: 1,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `cdc-notify-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`cdc-notify-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNotifyFieldsHS(instanceNum) {
    const endpointFields = [
        {
            id: 'cdcNotifyEndpointHS',
            label: 'Notification Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for notifications (e.g., 0x81)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'cdcNotifyMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed interrupt endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' },
                { value: '0x0200', text: '512 bytes (0x0200)' },
                { value: '0x0400', text: '1024 bytes (0x0400)' }
            ]
        },
        {
            id: 'cdcNotifyIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Polling interval for interrupt endpoint (in 125μs units for HS)',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '16',
            min: 1,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `cdc-notify-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`cdc-notify-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCBulkInFieldsFS(instanceNum) {
    const endpointFields = [
        {
            id: 'cdcBulkInEndpointFS',
            label: 'Bulk IN Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for data (e.g., 0x82)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x82',
            placeholder: '0x82'
        },
        {
            id: 'cdcBulkInMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'cdcBulkInIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `cdc-bulk-in-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`cdc-bulk-in-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCBulkInFieldsHS(instanceNum) {
    const endpointFields = [
        {
            id: 'cdcBulkInEndpointHS',
            label: 'Bulk IN Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for data (e.g., 0x82)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x82',
            placeholder: '0x82'
        },
        {
            id: 'cdcBulkInMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0200', text: '512 bytes (0x0200)' }
            ]
        },
        {
            id: 'cdcBulkInIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `cdc-bulk-in-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`cdc-bulk-in-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCBulkOutFieldsFS(instanceNum) {
    const endpointFields = [
        {
            id: 'cdcBulkOutEndpointFS',
            label: 'Bulk OUT Endpoint Address (bEndpointAddress)',
            help: 'OUT endpoint address for data (e.g., 0x02)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            placeholder: '0x02'
        },
        {
            id: 'cdcBulkOutMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0040',
            options: [
                { value: '0x0008', text: '8 bytes (0x0008)' },
                { value: '0x0010', text: '16 bytes (0x0010)' },
                { value: '0x0020', text: '32 bytes (0x0020)' },
                { value: '0x0040', text: '64 bytes (0x0040)' }
            ]
        },
        {
            id: 'cdcBulkOutIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `cdc-bulk-out-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`cdc-bulk-out-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCBulkOutFieldsHS(instanceNum) {
    const endpointFields = [
        {
            id: 'cdcBulkOutEndpointHS',
            label: 'Bulk OUT Endpoint Address (bEndpointAddress)',
            help: 'OUT endpoint address for data (e.g., 0x02)',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x02',
            placeholder: '0x02'
        },
        {
            id: 'cdcBulkOutMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed bulk endpoint',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0200', text: '512 bytes (0x0200)' }
            ]
        },
        {
            id: 'cdcBulkOutIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Typically 0 for bulk endpoints',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '0',
            min: 0,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `cdc-bulk-out-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`cdc-bulk-out-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderCDCNetworkDescriptorFields(classId, classLabel, containerId, configSchema, prefix) {
    const classNumberInput = document.getElementById(`class-num-${classId}`);
    const classNumber = parseInt(classNumberInput.value) || 1;
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        const upper = prefix.toUpperCase();
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}Instance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle(classLabel, i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapse${upper}Instance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}Config${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Communication Interface Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapse${upper}Config${i}" class="collapse">
                                <div class="card-body">
                                    <div id="${prefix}-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}Notify${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Notification Endpoint Descriptor</button></h3></div>
                            <div id="collapse${upper}Notify${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}NotifyFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div><div id="collapse${upper}NotifyFS${i}" class="collapse"><div class="card-body"><div id="${prefix}-notify-fs-fields-${i}"></div></div></div></div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}NotifyHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div><div id="collapse${upper}NotifyHS${i}" class="collapse"><div class="card-body"><div id="${prefix}-notify-hs-fields-${i}"></div></div></div></div>
                            </div></div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}BulkIn${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Data Bulk IN Endpoint Descriptor</button></h3></div>
                            <div id="collapse${upper}BulkIn${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}BulkInFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div><div id="collapse${upper}BulkInFS${i}" class="collapse"><div class="card-body"><div id="${prefix}-bulk-in-fs-fields-${i}"></div></div></div></div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}BulkInHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div><div id="collapse${upper}BulkInHS${i}" class="collapse"><div class="card-body"><div id="${prefix}-bulk-in-hs-fields-${i}"></div></div></div></div>
                            </div></div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}BulkOut${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Data Bulk OUT Endpoint Descriptor</button></h3></div>
                            <div id="collapse${upper}BulkOut${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}BulkOutFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div><div id="collapse${upper}BulkOutFS${i}" class="collapse"><div class="card-body"><div id="${prefix}-bulk-out-fs-fields-${i}"></div></div></div></div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapse${upper}BulkOutHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div><div id="collapse${upper}BulkOutHS${i}" class="collapse"><div class="card-body"><div id="${prefix}-bulk-out-hs-fields-${i}"></div></div></div></div>
                            </div></div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById(containerId);
    if (container) {
        container.innerHTML = html;
    }

    for (let i = 1; i <= classNumber; i++) {
        renderCDCNetworkConfigFields(prefix, configSchema, i);
        renderCDCNetworkNotifyFieldsFS(prefix, i);
        renderCDCNetworkNotifyFieldsHS(prefix, i);
        renderCDCNetworkBulkInFieldsFS(prefix, i);
        renderCDCNetworkBulkInFieldsHS(prefix, i);
        renderCDCNetworkBulkOutFieldsFS(prefix, i);
        renderCDCNetworkBulkOutFieldsHS(prefix, i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderCDCRNDISDescriptorFields() {
    renderCDCNetworkDescriptorFields('rndis', 'CDC RNDIS', 'rndis-descriptor-fields', CDC_RNDIS_DESC_SCHEMA, 'rndis');
}

function renderCDCECMDescriptorFields() {
    renderCDCNetworkDescriptorFields('ecm', 'CDC ECM', 'ecm-descriptor-fields', CDC_ECM_DESC_SCHEMA, 'ecm');
}

function renderAudioDescriptorFields() {
    const audioNumberInput = document.getElementById('class-num-audio');
    const classNumber = parseInt(audioNumberInput.value);
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioInstance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('Audio 1.0', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseAudioInstance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioControl${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Audio Control Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseAudioControl${i}" class="collapse">
                                <div class="card-body">
                                    <div id="audio-ac-fields-${i}"></div>
                                </div>
                            </div>
                        </div>
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioConfig${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Audio Streaming Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseAudioConfig${i}" class="collapse">
                                <div class="card-body">
                                    <div id="audio-config-fields-${i}"></div>

                                    <div class="card" id="audio-fb-ep-card-${i}" style="margin-top: 15px; margin-bottom: 0; display: none;">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioFbEp${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Feedback Endpoint Descriptor</button></h3></div>
                                        <div id="collapseAudioFbEp${i}" class="collapse"><div class="card-body">
                                            <div class="card" style="margin-bottom: 0;"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioFbEpFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Feedback Endpoint - Full Speed</button></h3></div><div id="collapseAudioFbEpFS${i}" class="collapse"><div class="card-body"><div id="audio-fb-ep-fs-fields-${i}"></div></div></div></div>
                                            <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: none;"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioFbEpHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Feedback Endpoint - High Speed</button></h3></div><div id="collapseAudioFbEpHS${i}" class="collapse"><div class="card-body"><div id="audio-fb-ep-hs-fields-${i}"></div></div></div></div>
                                        </div></div>
                                    </div>

                                    <div class="card" style="margin-top: 15px; margin-bottom: 0;">
                                        <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioEndpoint${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Isochronous Endpoint Descriptor</button></h3></div>
                                        <div id="collapseAudioEndpoint${i}" class="collapse"><div class="card-body">
                                            <div class="card" style="margin-bottom: 0;"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioEndpointFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div><div id="collapseAudioEndpointFS${i}" class="collapse"><div class="card-body"><div id="audio-ep-fs-fields-${i}"></div></div></div></div>
                                            <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioEndpointHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div><div id="collapseAudioEndpointHS${i}" class="collapse"><div class="card-body"><div id="audio-ep-hs-fields-${i}"></div></div></div></div>
                                        </div></div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('audio-descriptor-fields');
    container.innerHTML = html;

    for (let i = 1; i <= classNumber; i++) {
        renderAudioControlFields(i);
        renderAudioConfigFields(i);
        renderAudioFeedbackEndpointFieldsFS(i);
        renderAudioFeedbackEndpointFieldsHS(i);
        renderAudioEndpointFieldsFS(i);
        renderAudioEndpointFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderAudioControlFields(instanceNum) {
    const target = document.getElementById(`audio-ac-fields-${instanceNum}`);
    if (!target) return;

    // Read-only bcdADC display
    const bcdAdcGroup = document.createElement('div');
    bcdAdcGroup.className = 'form-group';
    bcdAdcGroup.innerHTML =
        `<label>ADC Version (bcdADC) <span class="info-icon" title="Audio Device Class specification release number in BCD. Fixed to 0x0100 for UAC 1.0." data-tooltip="Fixed to 0x0100 for UAC 1.0.">&#x2139;</span></label>` +
        `<input type="text" readonly style="background:#f0f0f0;color:#555;cursor:default;" value="0x0100  (UAC 1.0)">`;
    target.appendChild(bcdAdcGroup);

    // iInterface string group — Index | String | Language ID
    const wrapper = document.createElement('div');
    wrapper.className = 'string-field-group';
    wrapper.id = `audio-ac-${instanceNum}-string-group`;
    wrapper.innerHTML = `
        <div class="string-field-group-title">Audio Control Interface String (iInterface)</div>
        <div class="string-field-grid">
            <div class="form-group">
                <label for="audio-ac-${instanceNum}-iInterface">Index ${createInfoIcon('String descriptor index. 0 = no string.')}</label>
                <input type="number" id="audio-ac-${instanceNum}-iInterface" min="0" max="255" value="0">
            </div>
            <div class="form-group">
                <label for="audio-ac-${instanceNum}-interfaceString">String ${createInfoIcon('String descriptor text for the AudioControl interface.')}</label>
                <input type="text" id="audio-ac-${instanceNum}-interfaceString" placeholder="e.g., Audio Control" value="" disabled>
            </div>
            <div class="form-group string-language-group">
                <label for="audio-ac-${instanceNum}-languageIdDisplay">Language ID ${createInfoIcon('Select language when multiple language IDs are enabled')}</label>
                <select id="audio-ac-${instanceNum}-languageIdDisplay" class="interface-language-selector" data-instance="audio-ac-${instanceNum}" disabled></select>
            </div>
        </div>
    `;
    target.appendChild(wrapper);
    updateInterfaceLanguageSelectors();

    const indexEl = document.getElementById(`audio-ac-${instanceNum}-iInterface`);
    const stringEl = document.getElementById(`audio-ac-${instanceNum}-interfaceString`);
    const langEl   = document.getElementById(`audio-ac-${instanceNum}-languageIdDisplay`);

    const syncState = () => {
        const val = parseInt(indexEl.value, 10) || 0;
        const active = val > 0;
        stringEl.disabled = !active;
        langEl.disabled   = !active;
        langEl.dataset.lockedByIndex = active ? 'false' : 'true';
    };

    if (indexEl) indexEl.addEventListener('input', syncState);
    syncState();

    // Interrupt Endpoint — "+ add interrupt endpoint" button and collapsible card
    const addEpBtn = document.createElement('button');
    addEpBtn.id = `audio-ac-add-ep-btn-${instanceNum}`;
    addEpBtn.className = 'btn btn-sm';
    addEpBtn.type = 'button';
    addEpBtn.textContent = '+ add interrupt endpoint';
    addEpBtn.style.cssText = 'margin-top: 12px; margin-bottom: 12px; padding: 4px 10px; font-size: 0.85em;';
    addEpBtn.setAttribute('onclick', `addAudioAcEndpoint(${instanceNum})`);
    target.appendChild(addEpBtn);

    const epCard = document.createElement('div');
    epCard.id = `audio-ac-ep-card-${instanceNum}`;
    epCard.className = 'card';
    epCard.style.cssText = 'margin-top: 10px; margin-bottom: 0; display: none;';
    epCard.innerHTML = `
        <div class="card-header">
            <h3 class="mb-0" style="display: flex; align-items: center; justify-content: space-between; gap: 8px;">
                <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioAcEp${instanceNum}', this)" aria-expanded="false" style="flex: 1; text-align: left; font-size: 0.85em;">
                    Interrupt Endpoint Descriptor
                </button>
                <button id="audio-ac-remove-ep-btn-${instanceNum}" class="btn btn-sm" type="button" onclick="removeAudioAcEndpoint(${instanceNum})" style="display: none; flex: 0 0 auto; margin: 6px 8px 6px 0; padding: 4px 8px; font-size: 0.75em;">
                    remove
                </button>
            </h3>
        </div>
        <div id="collapseAudioAcEp${instanceNum}" class="collapse">
            <div class="card-body">
                <div class="card" style="margin-bottom: 0;">
                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioAcEpFS${instanceNum}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div>
                    <div id="collapseAudioAcEpFS${instanceNum}" class="collapse"><div class="card-body"><div id="audio-ac-ep-fs-fields-${instanceNum}"></div></div></div>
                </div>
                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: none;">
                    <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudioAcEpHS${instanceNum}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div>
                    <div id="collapseAudioAcEpHS${instanceNum}" class="collapse"><div class="card-body"><div id="audio-ac-ep-hs-fields-${instanceNum}"></div></div></div>
                </div>
            </div>
        </div>
    `;
    target.appendChild(epCard);

    updateAudioAcEpUI(instanceNum);
}

function renderAudioConfigFields(instanceNum) {
    const fieldsForTemplate = AUDIO10_DESC_SCHEMA.fields
        .filter((field) => !field.section)

        .filter((field) => !field.section)
        .map((field) => ({
        ...field,
        id: `audio-config-${instanceNum}-${field.id}`,
        isMultiSelect: field.type === 'multiselect',
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`audio-config-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }

    // Inject a read-only bFormatType display row after the wFormatTag select
    const formatTagEl = document.getElementById(`audio-config-${instanceNum}-audioFormatTag`);
    if (formatTagEl) {
        const formatTagGroup = formatTagEl.closest('.form-group');
        if (formatTagGroup) {
            const displayId = `audio-config-${instanceNum}-audioFormatType-display`;
            const roGroup = document.createElement('div');
            roGroup.className = 'form-group';
            roGroup.innerHTML =
                `<label>Format Type (bFormatType) <span class="info-icon" title="Automatically derived from wFormatTag. Type I: PCM-family. Type II: encoded (MPEG/AC-3). Type III: IEC 60958 pass-through." data-tooltip="Automatically derived from wFormatTag.">&#x2139;</span></label>` +
                `<input type="text" id="${displayId}" readonly style="background:#f0f0f0;color:#555;cursor:default;" value="">`;
            formatTagGroup.insertAdjacentElement('afterend', roGroup);
        }

        const updateFromFormatTag = () => {
            const derivedType = getFormatTypeFromTag(formatTagEl.value);
            const displayEl = document.getElementById(`audio-config-${instanceNum}-audioFormatType-display`);
            if (displayEl) {
                displayEl.value = derivedType + '  (' + (FORMAT_TYPE_NAMES[derivedType] || derivedType) + ')';
            }
            AUDIO10_DESC_SCHEMA.fields.forEach((field) => {
                if (!field.showForFormatTypes) return;
                const el = document.getElementById(`audio-config-${instanceNum}-${field.id}`);
                if (!el) return;
                const group = el.closest('.form-group');
                if (group) {
                    group.style.display = field.showForFormatTypes.includes(derivedType) ? '' : 'none';
                }
            });
            // wChannelConfig display follows bNrChannels visibility (Type I / III only)
            const chanCfgGroup = document.getElementById(`audio-config-${instanceNum}-audioChannelConfig-display-group`);
            if (chanCfgGroup) {
                chanCfgGroup.style.display = (derivedType === '0x01' || derivedType === '0x03') ? '' : 'none';
            }
        };

        formatTagEl.addEventListener('change', updateFromFormatTag);
        updateFromFormatTag();
    }

    // Inject a read-only wChannelConfig display row after the bNrChannels input
    const numChEl = document.getElementById(`audio-config-${instanceNum}-audioNumChannels`);
    if (numChEl) {
        const numChGroup = numChEl.closest('.form-group');
        if (numChGroup) {
            const chanCfgGroupId = `audio-config-${instanceNum}-audioChannelConfig-display-group`;
            const chanCfgDisplayId = `audio-config-${instanceNum}-audioChannelConfig-display`;
            const roGroup = document.createElement('div');
            roGroup.className = 'form-group';
            roGroup.id = chanCfgGroupId;
            roGroup.innerHTML =
                `<label>Channel Config (wChannelConfig) <span class="info-icon" title="Automatically derived from bNrChannels. Standard spatial channel bitmap per USB Audio 1.0 spec Table 3-6 (Input Terminal descriptor)." data-tooltip="Automatically derived from bNrChannels.">&#x2139;</span></label>` +
                `<input type="text" id="${chanCfgDisplayId}" readonly style="background:#f0f0f0;color:#555;cursor:default;" value="">`;
            numChGroup.insertAdjacentElement('afterend', roGroup);
        }

        const updateChannelConfig = () => {
            const count = parseInt(numChEl.value, 10) || 2;
            const raw = getChannelConfigFromCount(count);
            const hex = '0x' + raw.toString(16).toUpperCase().padStart(4, '0');
            const label = CHANNEL_CONFIG_LABELS[Math.max(1, Math.min(8, count))] || '';
            const displayEl = document.getElementById(`audio-config-${instanceNum}-audioChannelConfig-display`);
            if (displayEl) {
                displayEl.value = hex + '  (' + label + ')';
            }
        };

        numChEl.addEventListener('change', updateChannelConfig);
        numChEl.addEventListener('input', updateChannelConfig);
        updateChannelConfig();
    }

    // iInterface string group for Audio Streaming interface
    const asTarget = document.getElementById(`audio-config-fields-${instanceNum}`);
    if (asTarget) {
        const asWrapper = document.createElement('div');
        asWrapper.className = 'string-field-group';
        asWrapper.id = `audio-as-${instanceNum}-string-group`;
        asWrapper.innerHTML = `
            <div class="string-field-group-title">Audio Streaming Interface String (iInterface)</div>
            <div class="string-field-grid">
                <div class="form-group">
                    <label for="audio-as-${instanceNum}-iInterface">Index ${createInfoIcon('String descriptor index. 0 = no string.')}</label>
                    <input type="number" id="audio-as-${instanceNum}-iInterface" min="0" max="255" value="0">
                </div>
                <div class="form-group">
                    <label for="audio-as-${instanceNum}-interfaceString">String ${createInfoIcon('String descriptor text for the AudioStreaming interface.')}</label>
                    <input type="text" id="audio-as-${instanceNum}-interfaceString" placeholder="e.g., Audio Streaming" value="" disabled>
                </div>
                <div class="form-group string-language-group">
                    <label for="audio-as-${instanceNum}-languageIdDisplay">Language ID ${createInfoIcon('Select language when multiple language IDs are enabled')}</label>
                    <select id="audio-as-${instanceNum}-languageIdDisplay" class="interface-language-selector" data-instance="audio-as-${instanceNum}" disabled></select>
                </div>
            </div>
        `;
        asTarget.appendChild(asWrapper);
        updateInterfaceLanguageSelectors();

        const asIndexEl  = document.getElementById(`audio-as-${instanceNum}-iInterface`);
        const asStringEl = document.getElementById(`audio-as-${instanceNum}-interfaceString`);
        const asLangEl   = document.getElementById(`audio-as-${instanceNum}-languageIdDisplay`);

        const syncAsState = () => {
            const active = (parseInt(asIndexEl.value, 10) || 0) > 0;
            asStringEl.disabled = !active;
            asLangEl.disabled   = !active;
            asLangEl.dataset.lockedByIndex = active ? 'false' : 'true';
        };
        asIndexEl.addEventListener('input', syncAsState);
        syncAsState();
    }

    // Mutual exclusion between "None (0x00)" chip and the control bits chips
    ['audioFuMasterControls', 'audioFuChannelControls'].forEach((fieldId) => {
        const groupName = `audio-config-${instanceNum}-${fieldId}`;
        document.querySelectorAll(`input[name="${groupName}"]`).forEach((cb) => {
            cb.addEventListener('change', () => {
                const allInGroup = document.querySelectorAll(`input[name="${groupName}"]`);
                if (cb.value === '-1' && cb.checked) {
                    // None selected — uncheck all other chips
                    allInGroup.forEach((other) => {
                        if (other.value !== '-1') {
                            other.checked = false;
                            other.closest('.chip')?.classList.remove('chip-checked');
                        }
                    });
                } else if (cb.value !== '-1' && cb.checked) {
                    // A real control selected — uncheck None
                    allInGroup.forEach((other) => {
                        if (other.value === '-1') {
                            other.checked = false;
                            other.closest('.chip')?.classList.remove('chip-checked');
                        }
                    });
                }
            });
        });
    });

    // Synchronization type selector (bmAttributes[3:2] of the isochronous endpoint)
    const syncContainer = document.getElementById(`audio-config-fields-${instanceNum}`);
    if (syncContainer) {
        const syncGroup = document.createElement('div');
        syncGroup.className = 'form-group';
        syncGroup.innerHTML = `
            <label for="audio-ep-sync-${instanceNum}">Synchronization Type (bmAttributes[3:2]) ${createInfoIcon('Isochronous endpoint synchronization type. Asynchronous (0x05) requires a separate feedback endpoint.')}</label>
            <select id="audio-ep-sync-${instanceNum}">
                <option value="0x01">No Synchronization (0x01)</option>
                <option value="0x05">Asynchronous (0x05)</option>
                <option value="0x09">Adaptive (0x09)</option>
                <option value="0x0D">Synchronous (0x0D)</option>
            </select>`;
        syncContainer.appendChild(syncGroup);
        const syncEl = document.getElementById(`audio-ep-sync-${instanceNum}`);
        if (syncEl) {
            syncEl.addEventListener('change', () => {
                const fbCard = document.getElementById(`audio-fb-ep-card-${instanceNum}`);
                if (fbCard) fbCard.style.display = syncEl.value === '0x05' ? '' : 'none';
                updateAudioMaxPacketSizeFields(instanceNum);
                updateHighSpeedEndpointCardsVisibility();
                generateDescriptor();
            });
        }
    }
}

function renderAudioFeedbackEndpointFieldsFS(instanceNum) {
    const target = document.getElementById(`audio-fb-ep-fs-fields-${instanceNum}`);
    if (!target) return;
    const fields = [
        {
            id: `audio-fb-ep-fs-${instanceNum}-fbAddressFS`,
            label: `Feedback Endpoint Address (bEndpointAddress) ${createInfoIcon('OUT direction feedback endpoint address, e.g. 0x01.')}`,
            type: 'text', value: '0x01', placeholder: '0x01'
        },
        {
            id: `audio-fb-ep-fs-${instanceNum}-fbMaxPacketFS`,
            label: `Max Packet Size (wMaxPacketSize) ${createInfoIcon('FS feedback endpoint: 3 bytes (10.10 fixed-point format, USB Audio 1.0 spec).')}`,
            type: 'number', value: '3', min: 1, max: 3, placeholder: '3'
        },
        {
            id: `audio-fb-ep-fs-${instanceNum}-fbIntervalFS`,
            label: `Polling Interval (bInterval) ${createInfoIcon('Feedback polling interval for full speed. Value 1 = every SOF (1 ms).')}`,
            type: 'number', value: '1', min: 1, max: 16, placeholder: '1'
        }
    ];
    target.innerHTML = fields.map((f) => `
        <div class="form-group">
            <label for="${f.id}">${f.label}</label>
            <input type="${f.type}" id="${f.id}" value="${f.value}" placeholder="${f.placeholder}"
                ${f.min !== undefined ? `min="${f.min}"` : ''}
                ${f.max !== undefined ? `max="${f.max}"` : ''}>
        </div>`).join('');
    target.querySelectorAll('input').forEach((el) => el.addEventListener('input', generateDescriptor));
}

function renderAudioFeedbackEndpointFieldsHS(instanceNum) {
    const target = document.getElementById(`audio-fb-ep-hs-fields-${instanceNum}`);
    if (!target) return;
    const fields = [
        {
            id: `audio-fb-ep-hs-${instanceNum}-fbAddressHS`,
            label: `Feedback Endpoint Address (bEndpointAddress) ${createInfoIcon('OUT direction feedback endpoint address, e.g. 0x01.')}`,
            type: 'text', value: '0x01', placeholder: '0x01'
        },
        {
            id: `audio-fb-ep-hs-${instanceNum}-fbMaxPacketHS`,
            label: `Max Packet Size (wMaxPacketSize) ${createInfoIcon('HS feedback endpoint: 4 bytes (16.16 fixed-point format, USB Audio 1.0 spec).')}`,
            type: 'number', value: '4', min: 1, max: 4, placeholder: '4'
        },
        {
            id: `audio-fb-ep-hs-${instanceNum}-fbIntervalHS`,
            label: `Polling Interval (bInterval) ${createInfoIcon('Feedback polling interval for high speed. Value 4 = 2^(4-1) = 8 microframes (1 ms).')}`,
            type: 'number', value: '4', min: 1, max: 16, placeholder: '4'
        }
    ];
    target.innerHTML = fields.map((f) => `
        <div class="form-group">
            <label for="${f.id}">${f.label}</label>
            <input type="${f.type}" id="${f.id}" value="${f.value}" placeholder="${f.placeholder}"
                ${f.min !== undefined ? `min="${f.min}"` : ''}
                ${f.max !== undefined ? `max="${f.max}"` : ''}>
        </div>`).join('');
    target.querySelectorAll('input').forEach((el) => el.addEventListener('input', generateDescriptor));
}

function renderAudioEndpointFieldsFS(instanceNum) {
    const computedPkt = computeAudioIsoMaxPacketSize(instanceNum);
    const computedFSStr = String(computedPkt);
    const endpointFields = [
        {
            id: 'audioEndpointAddressFS',
            label: 'Isochronous Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for audio stream (e.g., 0x81).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'audioMaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Auto-computed: ceil(maxSampleRate / 1000) × channels × (bitResolution / 8). Updates automatically when audio settings change.',
            type: 'number',
            format: 'number',
            bytes: 2,
            default: computedFSStr,
            placeholder: computedFSStr,
            min: 1,
            max: 1023
        },
        {
            id: 'audioIntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Service interval in frames for FS isochronous endpoint.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '1',
            min: 1,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `audio-ep-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`audio-ep-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderAudioEndpointFieldsHS(instanceNum) {
    const computedPkt = computeAudioIsoMaxPacketSize(instanceNum);
    const computedHSStr = String(computedPkt);
    const endpointFields = [
        {
            id: 'audioEndpointAddressHS',
            label: 'Isochronous Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for audio stream (e.g., 0x81).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'audioMaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Auto-computed: ceil(maxSampleRate / 1000) × channels × (bitResolution / 8). Updates automatically when audio settings change.',
            type: 'number',
            format: 'number',
            bytes: 2,
            default: computedHSStr,
            placeholder: computedHSStr,
            min: 1,
            max: 3072
        },
        {
            id: 'audioIntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Service interval in microframes for HS isochronous endpoint.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '1',
            min: 1,
            max: 16
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `audio-ep-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`audio-ep-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderAudio2DescriptorFields() {
    const audio2NumberInput = document.getElementById('class-num-audio2');
    const classNumber = parseInt(audio2NumberInput.value);
    const isHighSpeedEnabled = !!(document.getElementById('speed-high') && document.getElementById('speed-high').checked);
    let html = '';

    for (let i = 1; i <= classNumber; i++) {
        html += `
            <div class="card">
                <div class="card-header">
                    <h2 class="mb-0">
                        <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudio2Instance${i}', this)" aria-expanded="false">
                            ${formatClassInstanceCardTitle('Audio 2.0', i, classNumber)}
                        </button>
                    </h2>
                </div>
                <div id="collapseAudio2Instance${i}" class="collapse">
                    <div class="card-body">
                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header">
                                <h3 class="mb-0">
                                    <button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudio2Config${i}', this)" aria-expanded="false" style="font-size: 0.85em;">
                                        Audio 2.0 Streaming Configuration
                                    </button>
                                </h3>
                            </div>
                            <div id="collapseAudio2Config${i}" class="collapse">
                                <div class="card-body">
                                    <div id="audio2-config-fields-${i}"></div>
                                </div>
                            </div>
                        </div>

                        <div class="card" style="margin-bottom: 15px;">
                            <div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudio2Endpoint${i}', this)" aria-expanded="false" style="font-size: 0.85em;">Isochronous Endpoint Descriptor</button></h3></div>
                            <div id="collapseAudio2Endpoint${i}" class="collapse"><div class="card-body">
                                <div class="card" style="margin-bottom: 0;"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudio2EndpointFS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - Full Speed</button></h3></div><div id="collapseAudio2EndpointFS${i}" class="collapse"><div class="card-body"><div id="audio2-ep-fs-fields-${i}"></div></div></div></div>
                                <div class="card endpoint-high-speed-card" style="margin-top: 10px; margin-bottom: 0; display: ${isHighSpeedEnabled ? 'block' : 'none'};"><div class="card-header"><h3 class="mb-0"><button class="btn-collapse collapsed" type="button" onclick="toggleCollapse('collapseAudio2EndpointHS${i}', this)" aria-expanded="false" style="font-size: 0.7em;">Endpoint - High Speed</button></h3></div><div id="collapseAudio2EndpointHS${i}" class="collapse"><div class="card-body"><div id="audio2-ep-hs-fields-${i}"></div></div></div></div>
                            </div></div>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    const container = document.getElementById('audio2-descriptor-fields');
    container.innerHTML = html;

    for (let i = 1; i <= classNumber; i++) {
        renderAudio2ConfigFields(i);
        renderAudio2EndpointFieldsFS(i);
        renderAudio2EndpointFieldsHS(i);
    }

    updateHighSpeedEndpointCardsVisibility();
}

function renderAudio2ConfigFields(instanceNum) {
    const fieldsForTemplate = AUDIO20_DESC_SCHEMA.fields.map((field) => ({
        ...field,
        id: `audio2-config-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`audio2-config-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderAudio2EndpointFieldsFS(instanceNum) {
    const endpointFields = [
        {
            id: 'audio2EndpointAddressFS',
            label: 'Isochronous Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for UAC2 stream (e.g., 0x81).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'audio2MaxPacketSizeFS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for Full Speed UAC2 isochronous endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x00C0',
            options: [
                { value: '0x0040', text: '64 bytes (0x0040)' },
                { value: '0x0080', text: '128 bytes (0x0080)' },
                { value: '0x00C0', text: '192 bytes (0x00C0)' },
                { value: '0x0100', text: '256 bytes (0x0100)' }
            ]
        },
        {
            id: 'audio2IntervalFS',
            label: 'Polling Interval (bInterval)',
            help: 'Service interval in frames for FS isochronous endpoint.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '1',
            min: 1,
            max: 255
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `audio2-ep-fs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`audio2-ep-fs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function renderAudio2EndpointFieldsHS(instanceNum) {
    const endpointFields = [
        {
            id: 'audio2EndpointAddressHS',
            label: 'Isochronous Endpoint Address (bEndpointAddress)',
            help: 'IN endpoint address for UAC2 stream (e.g., 0x81).',
            type: 'text',
            format: 'hex',
            bytes: 1,
            default: '0x81',
            placeholder: '0x81'
        },
        {
            id: 'audio2MaxPacketSizeHS',
            label: 'Max Packet Size (wMaxPacketSize)',
            help: 'Maximum packet size for High Speed UAC2 isochronous endpoint.',
            type: 'select',
            format: 'hex',
            bytes: 2,
            default: '0x0200',
            options: [
                { value: '0x0100', text: '256 bytes (0x0100)' },
                { value: '0x0200', text: '512 bytes (0x0200)' },
                { value: '0x0300', text: '768 bytes (0x0300)' },
                { value: '0x0400', text: '1024 bytes (0x0400)' }
            ]
        },
        {
            id: 'audio2IntervalHS',
            label: 'Polling Interval (bInterval)',
            help: 'Service interval in microframes for HS isochronous endpoint.',
            type: 'number',
            format: 'hex',
            bytes: 1,
            default: '1',
            min: 1,
            max: 16
        }
    ];

    const fieldsForTemplate = endpointFields.map((field) => ({
        ...field,
        id: `audio2-ep-hs-${instanceNum}-${field.id}`,
        isSelect: field.type === 'select',
        inputType: field.type === 'number' ? 'number' : 'text',
        hasMin: typeof field.min === 'number',
        hasMax: typeof field.max === 'number'
    }));

    const html = templates.field({ fields: fieldsForTemplate });
    const target = document.getElementById(`audio2-ep-hs-fields-${instanceNum}`);
    if (target) {
        target.innerHTML = html;
    }
}

function toggleCollapse(targetId, button) {
    const target = document.getElementById(targetId);
    const isExpanded = target.classList.contains('show');

    if (isExpanded) {
        target.classList.remove('show');
        button.classList.add('collapsed');
        button.setAttribute('aria-expanded', 'false');
    } else {
        target.classList.add('show');
        button.classList.remove('collapsed');
        button.setAttribute('aria-expanded', 'true');
    }
}

window.addEventListener('load', () => {
    registerHandlebarsHelpers();
    compileTemplates();
    renderForm();
    injectClassCardReorderControls();

    // Chip toggle: keep chip-checked class in sync when checkbox state changes
    document.body.addEventListener('change', (e) => {
        const cb = e.target;
        if (cb.type === 'checkbox' && cb.closest('.chip-group')) {
            const chip = cb.closest('.chip');
            if (chip) {
                chip.classList.toggle('chip-checked', cb.checked);
            }
        }

        // Audio config fields -> auto-recompute isochronous endpoint max packet size
        const idOrName = e.target.id || e.target.name || '';
        const audioConfigMatch = idOrName.match(/^audio-config-(\d+)-audio(?:NumChannels|BitResolution|SampleRate)$/);
        if (audioConfigMatch) {
            updateAudioMaxPacketSizeFields(parseInt(audioConfigMatch[1], 10));
        }
    });

    // Add listener for HID class number changes
    const hidNumberInput = document.getElementById('class-num-hid');
    hidNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-hid').checked) {
            renderHIDDescriptorFields();
        }
    });

    // Add listener for Mass Storage class number changes
    const mscNumberInput = document.getElementById('class-num-msc');
    mscNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-msc').checked) {
            renderMassStorageDescriptorFields();
        }
    });

    // Add listener for DFU class number changes
    const dfuNumberInput = document.getElementById('class-num-dfu');
    dfuNumberInput.addEventListener('change', () => {
        dfuNumberInput.value = '1';
        if (document.getElementById('class-dfu').checked) {
            renderDFUDescriptorFields();
        }
    });

    // Add listener for Printer class number changes
    const printerNumberInput = document.getElementById('class-num-printer');
    printerNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-printer').checked) {
            renderPrinterDescriptorFields();
        }
    });

    // Add listener for Video class number changes
    const videoNumberInput = document.getElementById('class-num-video');
    videoNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-video').checked) {
            renderVideoDescriptorFields();
        }
    });

    // Add listener for MTP class number changes
    const mtpNumberInput = document.getElementById('class-num-mtp');
    mtpNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-mtp').checked) {
            renderMTPDescriptorFields();
        }
    });

    // Add listener for PTP class number changes
    const ptpNumberInput = document.getElementById('class-num-ptp');
    ptpNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-ptp').checked) {
            renderPTPDescriptorFields();
        }
    });

    // Add listener for CDC ACM class number changes
    const cdcNumberInput = document.getElementById('class-num-cdc');
    cdcNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-cdc').checked) {
            renderCDCACMDescriptorFields();
        }
    });

    // Add listener for CDC RNDIS class number changes
    const rndisNumberInput = document.getElementById('class-num-rndis');
    rndisNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-rndis').checked) {
            renderCDCRNDISDescriptorFields();
        }
    });

    // Add listener for CDC ECM class number changes
    const ecmNumberInput = document.getElementById('class-num-ecm');
    ecmNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-ecm').checked) {
            renderCDCECMDescriptorFields();
        }
    });

    // Add listener for Audio class number changes
    const audioNumberInput = document.getElementById('class-num-audio');
    audioNumberInput.addEventListener('change', () => {
        if (document.getElementById('class-audio').checked) {
            renderAudioDescriptorFields();
        }
    });

    // Add listener for Audio 2.0 class number changes
    const audio2NumberInput = document.getElementById('class-num-audio2');
    audio2NumberInput.addEventListener('change', () => {
        if (document.getElementById('class-audio2').checked) {
            renderAudio2DescriptorFields();
        }
    });

    generateDescriptor();
});
