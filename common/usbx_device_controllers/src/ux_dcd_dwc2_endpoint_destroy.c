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
/*    _ux_dcd_dwc2_endpoint_destroy                         PORTABLE C    */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will destroy a physical endpoint.                     */
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
UINT  _ux_dcd_dwc2_endpoint_destroy(UX_DCD_DWC2 *dcd_dwc2, UX_SLAVE_ENDPOINT *endpoint)
{

UX_DCD_DWC2_ENDPOINT    *ed;
ULONG                   endpoint_address;

    /* Keep the physical endpoint address in the endpoint container.  */
    ed =  (UX_DCD_DWC2_ENDPOINT *) endpoint -> ux_slave_endpoint_ed;
    /* Set the endpoint direction.  */
    if ((endpoint -> ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN)
    {

        /* Clear the DAINTMSK field for endpoint IN.  */
        _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_DAINTMSK, 1 << ed -> ux_dcd_dwc2_endpoint_index);

    }
    else
    {

        /* Clear the DAINTMSK field for endpoint OUT.  */
        _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_DAINTMSK, 0x1000 << ed -> ux_dcd_dwc2_endpoint_index);
    }

    /* Get the endpoint address.  */
    endpoint_address = _ux_dcd_dwc2_endpoint_register_address_get(ed);

    /* Clear the endpoint register at the EP address.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, endpoint_address, 0);

    /* We can free this endpoint.  */
    ed -> ux_dcd_dwc2_endpoint_status =  UX_DCD_DWC2_ENDPOINT_STATUS_UNUSED;

    /* This function never fails.  */
    return(UX_SUCCESS);
}
