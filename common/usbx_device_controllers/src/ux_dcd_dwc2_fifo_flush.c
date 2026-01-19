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
#include "ux_utility.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                                RELEASE       */
/*                                                                        */
/*    _ux_dcd_dwc2_fifo_flush                              PORTABLE C     */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will flush a selective or all TX Fifo or all RX fifos.*/
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_dwc2                              Pointer to device controller  */
/*    fifo type                             FIFO type (TX or RX)          */
/*    fifo_index                            Fifo index for TX             */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
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
UINT  _ux_dcd_dwc2_fifo_flush(UX_DCD_DWC2 *dcd_dwc2, ULONG fifo_type, ULONG fifo_index)
{
ULONG    dwc2_register;

    /* Check if this is for RX or TX Fifos.  */
    if (fifo_type == UX_DCD_DWC2_FLUSH_RX_FIFO)

        /* Set the RXFFLSH bit.  */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GRSTCTL,
                                    UX_DCD_DWC2_OTG_GRSTCTL_RXFFLSH);
    else

        /* Set the Fifo number and the TXFFLSH bit.  */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GRSTCTL,
                                    (UX_DCD_DWC2_OTG_GRSTCTL_TXFFLSH |
                                    (fifo_index << UX_DCD_DWC2_OTG_GRSTCTL_TXFNUM_SHIFT)));

    /* Wait for the FIFO to be flushed.  */
    do
    {
        /* Read the GRSTCTL register.  */
        dwc2_register = _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_GRSTCTL);

    } while (dwc2_register & fifo_type);

    /* Spec says to wait for 3 PHY Clocks.  */
    _ux_dcd_dwc2_delay(3);

    return(UX_SUCCESS);
}
