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
/*    _ux_dcd_dwc2_fifo_read                               PORTABLE C     */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function will read from the FIFO of a particular endpoint.     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd_dwc2                              Pointer to device controller  */
/*    endpoint_index                        Endpoint index                */
/*    fifo_buffer                           Pointer to transfer buffer    */
/*    fifo_length                           Length to read                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_short_put                 Put a short data word         */
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
UINT  _ux_dcd_dwc2_fifo_read(UX_DCD_DWC2 *dcd_dwc2, ULONG endpoint_index,
                             UCHAR *data_pointer, ULONG fifo_length)
{

ULONG    fifo_address;
ULONG    fifo_value;

    UX_INTERRUPT_SAVE_AREA

    /* Calculate the address of the FIFO.  */
    fifo_address = UX_DCD_DWC2_DATA_FIFO_OFFSET + (endpoint_index * UX_DCD_DWC2_DATA_FIFO_SIZE);

    /* Number of bytes to read is based on DWORDS.  */
    fifo_length = (fifo_length + 3) / sizeof(ULONG);

    /* Lockout interrupts.  */
    UX_DISABLE

    /* Read one DWORD at a time.  */
    while (fifo_length--)
    {

        /* Read from FIFO.  */
        fifo_value = _ux_dcd_dwc2_register_read(dcd_dwc2, fifo_address);

        /* Store this value in a endian agnostic way.  */
        _ux_utility_long_put(data_pointer, fifo_value);

        /* Increment the data pointer buffer address.  */
        data_pointer += sizeof(ULONG);

    }

    /* Restore interrupts.  */
    UX_RESTORE

    /* Return successful completion.  */
    return(UX_SUCCESS);
}
