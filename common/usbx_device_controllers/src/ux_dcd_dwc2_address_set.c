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
/*  FUNCTION                                              RELEASE         */
/*                                                                        */
/*    _ux_dcd_dwc2_address_set                           PORTABLE C       */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will set the address of the device after we have      */
/*    received a SET_ADDRESS command from the host.                       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_dwc2                              Pointer to device controller  */
/*    address                               Address to set                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
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
UINT  _ux_dcd_dwc2_address_set(UX_DCD_DWC2 *dcd_dwc2, ULONG address)
{

    /* Clear the previous address.  */
    _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_DCFG, UX_DCD_DWC2_OTG_DCFG_DAD_MASK);

    /* Store the new address of the device.  */
    _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_DCFG, address << UX_DCD_DWC2_OTG_DCFG_DAD_SHIFT);

    /* This function always succeeds.  */
    return(UX_SUCCESS);
}
