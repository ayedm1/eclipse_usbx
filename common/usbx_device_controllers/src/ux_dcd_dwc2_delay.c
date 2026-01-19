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
/*    _ux_dcd_dwc2_delay                                 PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function performs a wait of usec on the DWC2 platform.         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_dwc2                              Pointer to device controller  */
/*    usec                                  Wait period.                  */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Synopsys DWC2 Controller Driver                                     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
VOID  _ux_dcd_dwc2_delay(ULONG usec)
{
volatile ULONG     utime;
volatile ULONG     ucount = 0;
ULONG              ucount_local;
ULONG              utime_local;

    /* Calculate the time to wait in cycles.  */
    utime = UX_DCD_DWC2_CONTROLLER_DELAY *usec;

    /* Now loop to wait.  */
    do
    {
        /* Check the count.  Place volatile variables in non-volatile to
           avoid compiler confusion regarding the order of volatile
           comparisons.  */
        ucount_local =  ++ucount;
        utime_local =  utime;
        if (ucount_local > utime_local)

            /* Done.  */
            return;

    } while(1);
}
