/***************************************************************************
 * Copyright (c) 2024 Microsoft Corporation
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 **************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Synopsys DWC2 Controller Driver                                     */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_dcd_dwc2.h"
#include "ux_device_stack.h"
#include "ux_system.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                 RELEASE      */
/*                                                                        */
/*    _ux_dcd_dwc2_interrupt_handler                       PORTABLE C    */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function is the interrupt handler for the Synopsys DWC2.       */
/*    The controller will trigger an interrupt when something happens on  */
/*    an endpoint whose mask has been set in the interrupt enable         */
/*    register, or when a bus reset is detected.                          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_dcd_dwc2_initialize_complete      Complete initialization       */
/*    _ux_dcd_dwc2_register_read            Read register                 */
/*    _ux_dcd_dwc2_register_write           Write register                */
/*    _ux_dcd_dwc2_transfer_callback        Process callback              */
/*    _ux_device_stack_disconnect           Disconnect device             */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    USBX Device Stack                                                   */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
VOID  _ux_dcd_dwc2_interrupt_handler(VOID)
{

ULONG                   dwc2_pending_interrupt;
ULONG                   dwc2_masked_interrupt;
ULONG                   endpoint_index;
ULONG                   endpoint_mask;
ULONG                   transfer_length;
ULONG                   transfer_status;
ULONG                   dwc2_daint_interrupt;
ULONG                   dwc2_dsts_register;
ULONG                   dwc2_grxstp_register;
ULONG                   dwc2_doepint_register;
ULONG                   dwc2_diepint_register;
UX_SLAVE_TRANSFER       *transfer_request;
UX_DCD_DWC2_ENDPOINT    *ed;
UX_SLAVE_ENDPOINT       *endpoint;
UX_SLAVE_DCD            *dcd;
UX_DCD_DWC2             *dcd_dwc2;
UX_SLAVE_DEVICE         *device;

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* Get the pointer to the DWC2 DCD.  */
    dcd_dwc2 = (UX_DCD_DWC2 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Get the pointer to the device.  */
    device =  &_ux_system_slave -> ux_system_slave_device;

    /* Read the interrupt status register from the controller.  */
    dwc2_pending_interrupt =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_GINTSTS);

    /* Mask only with the interrupts we have programmed.  */
    dwc2_masked_interrupt =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_GINTMSK);

    /* Source of interrupt must be masked.  */
    dwc2_pending_interrupt &= dwc2_masked_interrupt;

    /* Check if we have a SUSPEND.  */
    if (dwc2_pending_interrupt & UX_DCD_DWC2_OTG_GINTSTS_USBSUSP)
    {

        /* Clear the SUSPEND interrupt.  */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GINTSTS, UX_DCD_DWC2_OTG_GINTSTS_USBSUSP);

        /* Check the status change callback.  */
        if(_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
        {

            /* Inform the application if a callback function was programmed.  */
            _ux_system_slave -> ux_system_slave_change_function(UX_DEVICE_SUSPENDED);
        }

        /* If the device is attached or configured, we have a disconnection signal.  */
        if (device -> ux_slave_device_state !=  UX_DEVICE_RESET)

            /* Device is reset, the behavior is the same as disconnection.  */
            _ux_device_stack_disconnect();


    }

    /* Check if we have an RESUME.  */
    if (dwc2_pending_interrupt & UX_DCD_DWC2_OTG_GINTSTS_WKUINT)
    {

        /* Clear the RESUME interrupt.  */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GINTSTS, UX_DCD_DWC2_OTG_GINTSTS_WKUINT);

        /* Check the status change callback.  */
        if(_ux_system_slave -> ux_system_slave_change_function != UX_NULL)
        {

            /* Inform the application if a callback function was programmed.  */
            _ux_system_slave -> ux_system_slave_change_function(UX_DEVICE_RESUMED);
        }
    }

    /* Check the source of the interrupt. Is it Enumeration Done Interrupt (End of Bus Reset) ?  */
    if (dwc2_pending_interrupt & UX_DCD_DWC2_OTG_GINTSTS_ENUMDNE)
    {

        /* Clear the Enumeration done interrupt.  */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GINTSTS, UX_DCD_DWC2_OTG_GINTSTS_ENUMDNE);

        /* Check for bouncing RESET.  */
        if (device -> ux_slave_device_state !=  UX_DEVICE_ATTACHED && device -> ux_slave_device_state !=  UX_DEVICE_CONFIGURED)
        {

            /* Read the DSTS register to isolate speed.  */
            dwc2_dsts_register =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_DSTS);

            /* We have a device connection, read at what speed we are connected.  */
            if ((dwc2_dsts_register & UX_DCD_DWC2_OTG_DSTS_ENUMSPD_MASK) == UX_DCD_DWC2_OTG_DSTS_ENUMSPD_HS)
                _ux_system_slave -> ux_system_slave_speed =  UX_HIGH_SPEED_DEVICE;

            else
                _ux_system_slave -> ux_system_slave_speed =  UX_FULL_SPEED_DEVICE;

            /* Complete the device initialization.  */
            _ux_dcd_dwc2_initialize_complete();

            /* Mark the device as attached now.  */
            device -> ux_slave_device_state =  UX_DEVICE_ATTACHED;
        }
    }

    /* Check the source of the interrupt. Is it RESET ?  */
    if (dwc2_pending_interrupt & UX_DCD_DWC2_OTG_GINTSTS_USBRST)
    {

        /* Clear the Reset interrupt.  */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GINTSTS, UX_DCD_DWC2_OTG_GINTSTS_USBRST);

        /* If the device is attached or configured, we have a disconnection signal.  */
        if (device -> ux_slave_device_state !=  UX_DEVICE_RESET)

            /* Device is reset, the behavior is the same as disconnection.  */
            _ux_device_stack_disconnect();

        /* Mark the device as RESET now.  */
        device -> ux_slave_device_state =  UX_DEVICE_RESET;
    }

    /* Check the source of the interrupt. Is RX FIFO non Empty interrupt ? Meaning we have received an OUT or SETUP token.
       Reading the FIFO will trigger a Transfer complete interrupt.  */
    if (dwc2_pending_interrupt & UX_DCD_DWC2_OTG_GINTSTS_RFXLVL)
    {

        /* Clear the RX FIFO non Empty interrupt.  */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GINTSTS, UX_DCD_DWC2_OTG_GINTSTS_RFXLVL);

        /* Mask this interrupt for now.  */
        _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_GINTMSK, UX_DCD_DWC2_OTG_GINTMSK_RFXLVLM);

        /* Get the GRXSTSP register. This tells us on what endpoint the INT happened.  */
        dwc2_grxstp_register =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_GRXSTSP);

        /* Calculate the endpoint index.  */
        endpoint_index = dwc2_grxstp_register & UX_DCD_DWC2_OTG_GRXSTSP_EPNUM_MASK;

        /* Get the physical endpoint associated with this endpoint.  */
        ed =  &dcd_dwc2 -> ux_dcd_dwc2_endpoint[endpoint_index];

        /* Get the logical endpoint from the physical endpoint.  */
        endpoint =  ed -> ux_dcd_dwc2_endpoint;

        /* Isolate the transfer status.  */
        transfer_status = (dwc2_grxstp_register & UX_DCD_DWC2_OTG_GRXSTSP_PKTSTS_MASK) >> UX_DCD_DWC2_OTG_GRXSTSP_PKTSTS_SHIFT;

        /* Isolate the transfer length.  */
        transfer_length = (dwc2_grxstp_register & UX_DCD_DWC2_OTG_GRXSTSP_BCNT_MASK) >> UX_DCD_DWC2_OTG_GRXSTSP_BCNT_SHIFT;

        /* Get the pointer to the transfer request.  */
        transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

        /* Is this for a OUT endpoint or a SETUP packet ?  */
        switch (transfer_status)
        {

            case UX_DCD_DWC2_OTG_GRXSTSP_PKTSTS_SETUP_RCVD:

                /* Read the Fifo and store the data into the setup buffer.  */
                _ux_dcd_dwc2_fifo_read(dcd_dwc2, endpoint_index, transfer_request -> ux_slave_transfer_request_setup, UX_SETUP_SIZE);

                /* Save the length in the ED payload.  */
                ed -> ux_dcd_dwc2_endpoint_payload_length = transfer_length;

                break;

            case UX_DCD_DWC2_OTG_GRXSTSP_PKTSTS_OUT_RCVD:

                /* This is for a regular OUT endpoint.  */
                _ux_dcd_dwc2_fifo_read(dcd_dwc2, endpoint_index, transfer_request -> ux_slave_transfer_request_current_data_pointer, transfer_length);

                /* Adjust the buffer address.  */
                transfer_request -> ux_slave_transfer_request_current_data_pointer += transfer_length;

                /* Update the length of the data received.  */
                transfer_request -> ux_slave_transfer_request_actual_length += transfer_length;

                /* Save the length in the ED payload.  */
                ed -> ux_dcd_dwc2_endpoint_payload_length = transfer_length;

                break;
        }

        /* Reenable this interrupt.  */
        _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GINTMSK, UX_DCD_DWC2_OTG_GINTMSK_RFXLVLM);

    }

    /* Check the source of the interrupt. Is it a IN or OUT endpoint interrupt ? */
    if (dwc2_pending_interrupt & (UX_DCD_DWC2_OTG_GINTSTS_IEPINT | UX_DCD_DWC2_OTG_GINTSTS_OEPINT))
    {

        /* Read the DAINT register. Stored the IN/OUT individual endpoint interrupts.  */
        dwc2_daint_interrupt =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_DAINT);

        /* Parse the 32 bit indexed register. It is split in IEPINT/OEPINT.  */
        for (endpoint_mask = 0; endpoint_mask < 32; endpoint_mask++)
        {

            /* Check for an interrupt to have occurred on a specific endpoint.  */
            if (dwc2_daint_interrupt & (1 << endpoint_mask))
            {

                /* Build the endpoint index adjusting the endpoint mask.  */
                endpoint_index = endpoint_mask;

                /* If the endpoint mask is >= 16 we are in the OUT endpoints.  */
                if (endpoint_mask >= 16)
                {

                    /* Adjust the index.  */
                    endpoint_index -= 16;

                    /* Get the physical endpoint associated with this endpoint.  */
                    ed =  &dcd_dwc2 -> ux_dcd_dwc2_endpoint[endpoint_index];

                    /* Reset the endpoint transfer status. */
                    ed -> ux_dcd_dwc2_endpoint_transfer_status =  UX_DCD_DWC2_ENDPOINT_TRANSFER_STATUS_IDLE;

                    /* Get the register for the DOEPTINT.  */
                    dwc2_doepint_register =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_DOEPINT +
                                                                        (endpoint_index * UX_DCD_DWC2_ENDPOINT_CHANNEL_SIZE));

                    /* Find out what triggered the interrupt.  */
                    if (dwc2_doepint_register & UX_DCD_DWC2_OTG_DOEPINT_STUP)
                    {
                        /* Flag the setup. */
                        ed -> ux_dcd_dwc2_endpoint_transfer_status =  UX_DCD_DWC2_ENDPOINT_TRANSFER_STATUS_SETUP;

                    }

                    /* Find out what triggered the interrupt.  */
                    if (dwc2_doepint_register & UX_DCD_DWC2_OTG_DOEPINT_XFRC)
                    {

                        /* Flag the transfer completion. */
                        ed -> ux_dcd_dwc2_endpoint_transfer_status =  UX_DCD_DWC2_ENDPOINT_TRANSFER_STATUS_OUT_COMPLETION;
                    }
                }
                else
                {

                    /* Get the physical endpoint associated with this endpoint.  */
                    ed =  &dcd_dwc2 -> ux_dcd_dwc2_endpoint[endpoint_index];

                    /* Reset the endpoint transfer status. */
                    ed -> ux_dcd_dwc2_endpoint_transfer_status =  UX_DCD_DWC2_ENDPOINT_TRANSFER_STATUS_IDLE;

                    /* Get the register for the DIEPINT.  */
                    dwc2_diepint_register =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPINT +
                                                                        (endpoint_index * UX_DCD_DWC2_ENDPOINT_CHANNEL_SIZE));

                    /* Find out what triggered the interrupt.  */
                    if (dwc2_diepint_register & UX_DCD_DWC2_OTG_DIEPINT_XFRC)
                        /* Flag the transfer completion. */
                        ed -> ux_dcd_dwc2_endpoint_transfer_status =  UX_DCD_DWC2_ENDPOINT_TRANSFER_STATUS_IN_COMPLETION;

                }

                /* Get the logical endpoint from the physical endpoint.  */
                endpoint =  ed -> ux_dcd_dwc2_endpoint;

                /* Get the pointer to the transfer request.  */
                transfer_request =  &endpoint -> ux_slave_endpoint_transfer_request;

                /* Process the call back.  */
                _ux_dcd_dwc2_transfer_callback(dcd_dwc2, transfer_request);

                /* Now clean the interrupt that started this.  */
                /* If the endpoint mask is >= 16 we are in the OUT endpoints.  */
                if (endpoint_mask >= 16)

                    /* Clean the interrupt in the register for the DOEPTINT.  */
                    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DOEPINT +
                                                (endpoint_index * UX_DCD_DWC2_ENDPOINT_CHANNEL_SIZE), dwc2_doepint_register);

                else

                    /* Clean the interrupt in the register for the DIEPTINT.  */
                    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPINT +
                                                (endpoint_index * UX_DCD_DWC2_ENDPOINT_CHANNEL_SIZE), dwc2_diepint_register);

            }
        }
    }
}
