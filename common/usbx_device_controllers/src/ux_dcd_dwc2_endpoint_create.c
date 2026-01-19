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


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_dcd_dwc2_endpoint_create                        PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will create a physical endpoint.                      */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_dwc2                              Pointer to device controller  */
/*    endpoint                              Pointer to endpoint container */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_dcd_dwc2_endpoint_reset           Reset endpoint                */
/*    _ux_dcd_dwc2_register_read            Read register                 */
/*    _ux_dcd_dwc2_register_write           Write register                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    DWC2 Controller Driver                                              */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_dcd_dwc2_endpoint_create(UX_DCD_DWC2 *dcd_dwc2, UX_SLAVE_ENDPOINT *endpoint)
{

ULONG                   endpoint_index;
ULONG                   endpoint_register;
ULONG                   endpoint_address;
UX_DCD_DWC2_ENDPOINT    *ed;

    /* The endpoint index in the array of the DWC2 must match the endpoint number.
       The DWC2 has 4 endpoints maximum. Each can be IN/OUT.  */
    endpoint_index =  endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & ~UX_ENDPOINT_DIRECTION;

    /* Fetch the address of the physical endpoint.  */
    ed =  &dcd_dwc2 -> ux_dcd_dwc2_endpoint[endpoint_index];

    if (ed == UX_NULL)
        return(UX_NO_ED_AVAILABLE);

    /* Check the endpoint status, if it is free, reserve it. If not reject this endpoint.  */
    if ((ed -> ux_dcd_dwc2_endpoint_status & UX_DCD_DWC2_ENDPOINT_STATUS_USED) == 0)
    {

        /* We can use this endpoint.  */
        ed -> ux_dcd_dwc2_endpoint_status |=  UX_DCD_DWC2_ENDPOINT_STATUS_USED;

        /* Keep the physical endpoint address in the endpoint container.  */
        endpoint -> ux_slave_endpoint_ed =  (VOID *) ed;

        /* Save the endpoint pointer.  */
        ed -> ux_dcd_dwc2_endpoint =  endpoint;

        /* And its index.  */
        ed -> ux_dcd_dwc2_endpoint_index =  endpoint_index;

        /* And its direction.  */
        ed -> ux_dcd_dwc2_endpoint_direction =  endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION;

        /* And its type.  */
        ed -> ux_dcd_dwc2_endpoint_type =  endpoint -> ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE;

        /* Reset the endpoint.  */
        _ux_dcd_dwc2_endpoint_reset(dcd_dwc2, endpoint);

        /* Reset the endpoint register.  */
        endpoint_register = 0;

        /* For IN endpoint, the FIFO number is stored in the DIEPCTL register.  */
        if (ed -> ux_dcd_dwc2_endpoint_direction == UX_ENDPOINT_IN)

            /* Set the FIFO number based on the endpoint index.  */
            endpoint_register |= endpoint_index << UX_DCD_DWC2_OTG_DIEPCTL_TXFNUM_SHIFT;

        /* Set USBAEP Active endpoint bit.  */
        endpoint_register |= UX_DCD_DWC2_OTG_DIEPCTL_USBAEP;

        /* Build the endpoint DIEP or DOEP register. */
        switch (ed -> ux_dcd_dwc2_endpoint_type)
        {

        case UX_CONTROL_ENDPOINT:

            /* Set the MaxPacketSize.  This is different for Control endpoints and other endpoints.  */
            switch (endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize)
            {

                /* Is it 64 bytes ? */
                case      64    :
                    endpoint_register = 0;
                    break;

                /* Is it 32 bytes ? */
                case      32    :
                    endpoint_register = 1;
                    break;

                /* Is it 16 bytes ? */
                case      16    :
                    endpoint_register = 2;
                    break;

                /* Is it 8 bytes ? */
                case      8    :
                    endpoint_register = 3;
                    break;
            }

            /* Set the DIEPCTL register.  */
            _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPCTL, endpoint_register);

            /* Set the DAINTMSK field for control endpoint IN.  */
            _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_DAINTMSK, 1 << endpoint_index);

            /* Set the DOEPCTL register.  */
            _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DOEPCTL, endpoint_register);

            /* Set the DAINTMSK field for control endpoint OUT.  */
            _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_DAINTMSK, 0x10000 << endpoint_index);

            break;

        case UX_BULK_ENDPOINT:

            /* Set the MPS field.  */
            endpoint_register |= endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;

            /* Set the SD0PID flag.  */
            endpoint_register |= UX_DCD_DWC2_OTG_DIEPCTL_SD0PID;

            /* Bulk endpoint. Set the USBAEP and SNAK fields.  */
            endpoint_register |=  UX_DCD_DWC2_OTG_DIEPCTL_EPTYP_BULK |
                                  UX_DCD_DWC2_OTG_DIEPCTL0_USBAEP    |
                                  UX_DCD_DWC2_OTG_DIEPCTL0_SNAK;
            break;

        case UX_INTERRUPT_ENDPOINT:

            /* Set the MPS field.  */
            endpoint_register |= endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;

            /* Set the SD0PID flag.  */
            endpoint_register |= UX_DCD_DWC2_OTG_DIEPCTL_SD0PID;

            /* Interrupt endpoint. Set the USBAEP and SNAK fields. */
            endpoint_register |=  UX_DCD_DWC2_OTG_DIEPCTL_EPTYP_INTERRUPT |
                                  UX_DCD_DWC2_OTG_DIEPCTL0_USBAEP         |
                                  UX_DCD_DWC2_OTG_DIEPCTL0_SNAK;

            break;

        case UX_ISOCHRONOUS_ENDPOINT:

            /* Set the MPS field.  */
            endpoint_register |= endpoint -> ux_slave_endpoint_descriptor.wMaxPacketSize;

            /* ISO endpoint. */
            endpoint_register |=  UX_DCD_DWC2_OTG_DIEPCTL_EPTYP_ISO | UX_DCD_DWC2_OTG_DIEPCTL0_USBAEP;

            break;

        default:

            return(UX_ERROR);
        }

        /* Continue initialization for non control endpoints.  */

        if (ed -> ux_dcd_dwc2_endpoint_type != UX_CONTROL_ENDPOINT)
        {

            /* Set the endpoint direction.  */
            if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
            {

                /* Reset FIFO for endpoint IN.  */
                _ux_dcd_dwc2_fifo_flush(dcd_dwc2, UX_DCD_DWC2_FLUSH_TX_FIFO, endpoint_index);

                /* Set the DAINTMSK field for endpoint IN.  */
                _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_DAINTMSK, 1 << endpoint_index);
            }
            else

                /* Set the DAINTMSK field for endpoint OUT.  */
                _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_DAINTMSK, 0x10000 << endpoint_index);

            /* Get the endpoint address.  */
            endpoint_address = _ux_dcd_dwc2_endpoint_register_address_get(ed);

            /* Set the endpoint register at the EP address.  */
            _ux_dcd_dwc2_register_write(dcd_dwc2, endpoint_address, endpoint_register);
        }

        /* Return successful completion.  */
        return(UX_SUCCESS);
    }

    /* Return an error.  */
    return(UX_NO_ED_AVAILABLE);
}
