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
/*    _ux_dcd_dwc2_endpoint_status                          PORTABLE C    */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will retrieve the status of the endpoint.             */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_dwc2                              Pointer to device controller  */
/*    endpoint_index                        Endpoint index                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
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
UINT  _ux_dcd_dwc2_endpoint_status(UX_DCD_DWC2 *dcd_dwc2, ULONG endpoint_index)
{

UX_DCD_DWC2_ENDPOINT    *ed;

    /* Fetch the address of the physical endpoint.  */
    ed =  &dcd_dwc2 -> ux_dcd_dwc2_endpoint[endpoint_index];

    /* Check the endpoint status, if it is free, we have a illegal endpoint.  */
    if ((ed -> ux_dcd_dwc2_endpoint_status & UX_DCD_DWC2_ENDPOINT_STATUS_USED) == 0)
        return(UX_ERROR);

    /* Check if the endpoint is stalled.  */
    if ((ed -> ux_dcd_dwc2_endpoint_status & UX_DCD_DWC2_ENDPOINT_STATUS_STALLED) == 0)
        return(UX_FALSE);
    else
        return(UX_TRUE);
}
