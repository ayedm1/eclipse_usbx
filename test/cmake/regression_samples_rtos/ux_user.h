#ifndef UX_USER
#define UX_USER

#ifdef DEVICE_HID_MOUSE_DEMO

#define UX_DEVICE_ALTERNATE_SETTING_SUPPORT_DISABLE

#define UX_DEVICE_INITIALIZE_FRAMEWORK_SCAN_DISABLE
#define UX_MAX_DEVICE_ENDPOINTS                         1 /* Endpoint Interrupt.  */
#define UX_MAX_DEVICE_INTERFACES                        1 /* Interface HID.  */

#define UX_MAX_SLAVE_INTERFACES                         1
#define UX_MAX_SLAVE_CLASS_DRIVER                       1

#define UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH             64 /* > 62 for descriptors.  */
#define UX_SLAVE_REQUEST_DATA_MAX_LENGTH                8 /* HID frame.  */

#define UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH         8
#define UX_DEVICE_CLASS_HID_MAX_EVENTS_QUEUE            2

#define UX_NAME_REFERENCED_BY_POINTER

#define UX_DEVICE_SIDE_ONLY

#define UX_HOST_SIDE_ONLY  /* FOR TEST */

#endif


#endif /* UX_USER */