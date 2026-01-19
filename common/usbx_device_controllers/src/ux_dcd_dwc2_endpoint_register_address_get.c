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
/*    _ux_dcd_dwc2_endpoint_register_address_get          PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function returns the address of the endpoint register          */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ed                                    Endpoint descriptor           */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Register address of endpoint.                                       */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    DWC2 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
ULONG  _ux_dcd_dwc2_endpoint_register_address_get(UX_DCD_DWC2_ENDPOINT *ed)
{

    /* Check for endpoint 0.  */
    if (ed -> ux_dcd_dwc2_endpoint_index == 0)

        /* Return the address of endpoint 0. */
        return(UX_DCD_DWC2_OTG_DOEPCTL);

    else
    {

        /* Check the endpoint direction for the base.  */
        if (ed -> ux_dcd_dwc2_endpoint_direction  == UX_ENDPOINT_IN)
            return(UX_DCD_DWC2_OTG_DIEPCTL + (ed -> ux_dcd_dwc2_endpoint_index * UX_DCD_DWC2_ENDPOINT_SPACE_SIZE));
        else
            return(UX_DCD_DWC2_OTG_DOEPCTL + (ed -> ux_dcd_dwc2_endpoint_index * UX_DCD_DWC2_ENDPOINT_SPACE_SIZE));

    }
}
