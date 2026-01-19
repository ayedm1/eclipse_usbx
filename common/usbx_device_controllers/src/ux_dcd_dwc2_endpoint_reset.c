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
/*  FUNCTION                                                 RELEASE      */
/*                                                                        */
/*    _ux_dcd_dwc2_endpoint_reset                           PORTABLE C    */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will reset a physical endpoint.                       */
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
/*    _ux_dcd_dwc2_register_set             Set register                  */
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
UINT  _ux_dcd_dwc2_endpoint_reset(UX_DCD_DWC2 *dcd_dwc2, UX_SLAVE_ENDPOINT *endpoint)
{

UX_DCD_DWC2_ENDPOINT    *ed;
ULONG                   endpoint_register;
ULONG                   endpoint_address;

    /* Get the physical endpoint address in the endpoint container.  */
    ed =  (UX_DCD_DWC2_ENDPOINT *) endpoint -> ux_slave_endpoint_ed;

    /* Set the status of the endpoint to not stalled.  */
    ed -> ux_dcd_dwc2_endpoint_status &= ~UX_DCD_DWC2_ENDPOINT_STATUS_STALLED;

    /* Set the state of the endpoint to IDLE.  */
    ed -> ux_dcd_dwc2_endpoint_state =  UX_DCD_DWC2_ENDPOINT_STATE_IDLE;

    /* Get the endpoint address.  */
    endpoint_address = _ux_dcd_endpoint_register_address_get(ed);

    /* Read the current EP register.  */
    endpoint_register =  _ux_dcd_dwc2_register_read(dcd_dwc2, endpoint_address);

    /* Reset the stall bit.  */
    endpoint_register &= ~UX_DCD_DWC2_OTG_DIEPCTL_STALL;

    /* If the endpoint is Bulk or INT we need to reset the PID Data to DATA0.  */
    if (ed -> ux_dcd_dwc2_endpoint_type == UX_BULK_ENDPOINT || ed -> ux_dcd_dwc2_endpoint_type == UX_INTERRUPT_ENDPOINT)

        /* Set the SD0PID flag.  */
        endpoint_register |= UX_DCD_DWC2_OTG_DIEPCTL_SD0PID;

    /* Write the endpoint register.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, endpoint_address, endpoint_register);

    /* This function never fails.  */
    return(UX_SUCCESS);
}
