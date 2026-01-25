/* Direct SIL regression test for samples/demo_device_hid_mouse_rtos.c */
#include <stdio.h>
#include "ux_api.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"

void  test_control_return(UINT status);
extern UINT ux_demo_device_hid_init(VOID);

UX_HOST_CLASS_HID_MOUSE     *mouse;
UX_HOST_CLASS_HID          *hid_instance;
static UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance);

/* Ensure device-only demo compiles within the test. */
#define UX_DEMO_STACK_SIZE         1024
static UX_THREAD ux_demo_thread_host_simulation;
static ULONG ux_simulation_thread_stack[UX_DEMO_STACK_SIZE / sizeof(ULONG)];
static void  tx_demo_thread_host_simulation_entry(ULONG arg);

void usbx_hid_mouse_demo_device_rtos_test_application_define(void *first_unused_memory)
{
    UX_PARAMETER_NOT_USED(first_unused_memory);

    UINT status;

    printf("Running HID Mouse Demo Test............................ ");


    /* Initialize device side using the demo's API (creates its HID thread). */
    status = ux_demo_device_hid_init();
    
    if (status != UX_SUCCESS)
    { 
        printf("Demo device init fail\n"); 
        test_control_return(1); 
    }

    /* Initialize host side. */
    status = ux_host_stack_initialize(ux_host_event_callback);

    if (status != UX_SUCCESS)
    { 
        printf("Host init fail\n");
        test_control_return(1); 
    }

    status = ux_host_stack_class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);

    if (status != UX_SUCCESS)
    {
        printf("Host HID reg fail\n"); 
        test_control_return(1); 
    }

    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name, ux_host_class_hid_mouse_entry);

    if (status != UX_SUCCESS) 
    { 
        printf("Host mouse client reg fail\n"); 
        test_control_return(1); 
    }

    /* Register all the USB host controllers available in this system */
    status =  ux_host_stack_hcd_register(_ux_system_host_hcd_simulator_name, ux_hcd_sim_host_initialize,0,0);

    /* Check for error.  */
    if (status != UX_SUCCESS)
    {

        printf("Error on line %d, error code: %d\n", __LINE__, status);
        test_control_return(1);
    }

    /* Create the main host simulation thread.  */
    status =  ux_utility_thread_create(&ux_demo_thread_host_simulation, "tx demo host simulation",
                                       tx_demo_thread_host_simulation_entry, 0, ux_simulation_thread_stack, 
                                       UX_DEMO_STACK_SIZE, 20, 20, 1, UX_AUTO_START);

    /* Check for error.  */
    if (status != UX_SUCCESS)
    {

        printf("Error on line %d, error code: %d\n", __LINE__, status);
        test_control_return(1);
    }

}

static void  tx_demo_thread_host_simulation_entry(ULONG arg)
{

UINT    status;

ULONG   cur_mouse_buttons           = 0;
SLONG   cur_mouse_x_position        = 0;
SLONG   cur_mouse_y_position        = 0;
SLONG   cur_mouse_wheel_movement    = 0;


  while (1)
  {
    /* Start if the hid client is a mouse and connected */
    if ((mouse != NULL) &&
        (mouse -> ux_host_class_hid_mouse_state == (ULONG) UX_HOST_CLASS_INSTANCE_LIVE))
    {

        status = ux_host_class_hid_mouse_buttons_get(mouse, &cur_mouse_buttons);
        if (status != UX_SUCCESS)
        {

            printf("Error on line %d\n", __LINE__);
            test_control_return(1);
        }

        status = ux_host_class_hid_mouse_position_get(mouse, &cur_mouse_x_position, &cur_mouse_y_position);
        if (status != UX_SUCCESS)
        {

            printf("Error on line %d\n", __LINE__);
            test_control_return(1);
        }

        status = ux_host_class_hid_mouse_wheel_get(mouse, &cur_mouse_wheel_movement);
        if (status != UX_SUCCESS)
        {

            printf("Error on line %d\n", __LINE__);
            test_control_return(1);
        }

        status = ux_host_class_hid_mouse_position_get(mouse, &cur_mouse_x_position, &cur_mouse_y_position);

//        if (cur_mouse_x_position != (cur_mouse_x_position+20))
//

    }
    else
    {
        ux_utility_delay_ms(MS_TO_TICK(10));
    }
  }


}

UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *current_class, VOID *current_instance)
{
  UINT status = UX_SUCCESS;

  /* Get current Hid Client */
  UX_HOST_CLASS_HID_CLIENT *client  = (UX_HOST_CLASS_HID_CLIENT *)current_instance;

  switch (event)
  {
    case UX_DEVICE_INSERTION:

       /* Get current Hid Class */
      if (current_class -> ux_host_class_entry_function == ux_host_class_hid_entry)
      {
        if (hid_instance == UX_NULL)
        {
          /* Get current Hid Instance */
          hid_instance = (UX_HOST_CLASS_HID *)current_instance;
        }
      }

      break;

    case UX_DEVICE_REMOVAL:

      /* Free HID Instance */
      if ((VOID*)hid_instance == current_instance)
      {
        hid_instance = UX_NULL;
      }

      break;

    case UX_HID_CLIENT_INSERTION:

      /* Check the HID_client if this is a HID mouse device */
      if (client -> ux_host_class_hid_client_handler == ux_host_class_hid_mouse_entry)
      {
        /* Get current Hid Client */
        if (mouse == UX_NULL)
        {
          mouse = client -> ux_host_class_hid_client_local_instance;
        }
      }


      break;

    case UX_HID_CLIENT_REMOVAL:

  
      if ((VOID*)mouse == client -> ux_host_class_hid_client_local_instance)
      {
        /* Clear hid mouse instance */
        mouse = UX_NULL;
      }

      break;

    case UX_DEVICE_CONNECTION:
      break;

    case UX_DEVICE_DISCONNECTION:
      break;

    default:
      break;
  }

  return status;
}

