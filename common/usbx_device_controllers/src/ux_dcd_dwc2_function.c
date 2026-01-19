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
/*    _ux_dcd_dwc2_function                               PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function dispatches the DCD function internally to the DWC2    */
/*    controller.                                                         */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd                                   Pointer to device controller  */
/*    function                              Function requested            */
/*    parameter                             Pointer to function parameters*/
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_dcd_dwc2_endpoint_create          Create endpoint               */
/*    _ux_dcd_dwc2_endpoint_destroy         Destroy endpoint              */
/*    _ux_dcd_dwc2_endpoint_reset           Reset endpoint                */
/*    _ux_dcd_dwc2_endpoint_stall           Stall endpoint                */
/*    _ux_dcd_dwc2_endpoint_status          Get endpoint status           */
/*    _ux_dcd_dwc2_frame_number_get         Get frame number              */
/*    _ux_dcd_dwc2_state_change             Change state                  */
/*    _ux_dcd_dwc2_transfer_request         Request data transfer         */
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
UINT  _ux_dcd_dwc2_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter)
{

UINT             status;
UX_DCD_DWC2     *dcd_dwc2;


    /* Check the status of the controller.  */
    if (dcd -> ux_slave_dcd_status == UX_UNUSED)
    {

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        return(UX_CONTROLLER_UNKNOWN);
    }

    /* Get the pointer to the DWC2 DCD.  */
    dcd_dwc2 =  (UX_DCD_DWC2 *) dcd -> ux_slave_dcd_controller_hardware;

    /* Look at the function and route it.  */
    switch(function)
    {

    case UX_DCD_GET_FRAME_NUMBER:

        status =  _ux_dcd_dwc2_frame_number_get(dcd_dwc2, (ULONG *) parameter);
        break;

    case UX_DCD_TRANSFER_REQUEST:

#if defined(UX_DEVICE_STANDALONE)
        status =  _ux_dcd_dwc2_transfer_run(dcd_dwc2, (UX_SLAVE_TRANSFER *) parameter);
#else
        status =  _ux_dcd_dwc2_transfer_request(dcd_dwc2, (UX_SLAVE_TRANSFER *) parameter);
#endif /* defined(UX_DEVICE_STANDALONE) */
        break;

    case UX_DCD_TRANSFER_ABORT:
        status = _ux_dcd_dwc2_transfer_abort(dcd_dwc2, parameter);
        break;

    case UX_DCD_CREATE_ENDPOINT:

        status =  _ux_dcd_dwc2_endpoint_create(dcd_dwc2, parameter);
        break;

    case UX_DCD_DESTROY_ENDPOINT:

        status =  _ux_dcd_dwc2_endpoint_destroy(dcd_dwc2, parameter);
        break;

    case UX_DCD_RESET_ENDPOINT:

        status =  _ux_dcd_dwc2_endpoint_reset(dcd_dwc2, parameter);
        break;

    case UX_DCD_STALL_ENDPOINT:

        status =  _ux_dcd_dwc2_endpoint_stall(dcd_dwc2, parameter);
        break;

    case UX_DCD_SET_DEVICE_ADDRESS:

        status =  _ux_dcd_dwc2_address_set(dcd_dwc2, (ULONG) parameter);
        break;

    case UX_DCD_CHANGE_STATE:

        status =  _ux_dcd_dwc2_state_change(dcd_dwc2, (ULONG) parameter);
        break;

    case UX_DCD_ENDPOINT_STATUS:

        status =  _ux_dcd_dwc2_endpoint_status(dcd_dwc2, (ULONG) parameter);
        break;

#if defined(UX_DEVICE_STANDALONE)
    case UX_DCD_ISR_PENDING:

        _ux_dcd_dwc2_setup_isr_pending(dcd_dwc2);
        status = UX_SUCCESS;
        break;
#endif /* defined(UX_DEVICE_STANDALONE) */

    default:

        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_DCD, UX_FUNCTION_NOT_SUPPORTED);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

        status =  UX_FUNCTION_NOT_SUPPORTED;
        break;
    }

    /* Return completion status.  */
    return(status);
}
