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
/*    _ux_dcd_dwc2_initialize                             PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function initializes the USB device controller of the DWC2.    */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    dcd                                   Address of DCD                */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_dcd_dwc2_register_write           Write register                */
/*    _ux_utility_memory_allocate           Allocate memory               */
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
UINT  _ux_dcd_dwc2_initialize(ULONG dcd_io, ULONG parameter)
{

UX_SLAVE_DCD        *dcd;
UX_DCD_DWC2         *dcd_dwc2;
ULONG               dwc2_register;
ULONG               fifo_address;

    /* Get the pointer to the DCD.  */
    dcd =  &_ux_system_slave -> ux_system_slave_dcd;

    /* The controller initialized here is of DWC2 type.  */
    dcd -> ux_slave_dcd_controller_type =  UX_DCD_DWC2_SLAVE_CONTROLLER;

    /* Allocate memory for this DWC2 DCD instance.  */
    dcd_dwc2 =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DCD_DWC2));

    /* Check if memory was properly allocated.  */
    if(dcd_dwc2 == UX_NULL)
        return(UX_MEMORY_INSUFFICIENT);

    /* Set the pointer to the DWC2 DCD.  */
    dcd -> ux_slave_dcd_controller_hardware =  (VOID *) dcd_dwc2;

    /* Save the base address of the controller.  */
    dcd -> ux_slave_dcd_io =        dcd_io;
    dcd_dwc2 -> ux_dcd_dwc2_base =  dcd_io;

    /* Set the generic DCD owner for the DWC2 DCD.  */
    dcd_dwc2 -> ux_dcd_dwc2_dcd_owner =  dcd;

    /* Initialize the function collector for this DCD.  */
    dcd -> ux_slave_dcd_function =  _ux_dcd_dwc2_function;

    /* Reset the GINT Global Interrupt Mask register.  */
    _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_GAHBCFG, UX_DCD_DWC2_OTG_GAHBCFG_GINT);

    /* Check the controller. Full Speed or HighSpeed initialization.  */
    if (dcd_dwc2 -> ux_dcd_dwc2_base == UX_DCD_DWC2_OTG_FULL_SPEED)
    {

        /* PHY Selection in FS mode.  */
        _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GUSBCFG, UX_DCD_DWC2_OTG_GUSBCFG_PHYSEL);

        #ifdef STM32F767ZI_NUCLEO_USBX
        /* Initialize the PHY Power options. In device mode, set B-VBUS */
        _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GCCFG ,
                                            UX_DCD_DWC2_OTG_GCCFG_PWRDWN |
                                            UX_DCD_DWC2_OTG_GCCFG_VBDEN);

        #else
         /* Initialize the PHY Power options. In device mode, set B-VBUS */
        _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GCCFG ,
                                            UX_DCD_DWC2_OTG_GCCFG_PWRDWN  |
                                            UX_DCD_DWC2_OTG_GCCFG_VBUSSENSINGB);
        #endif
    }

    else
    {

        /* PHY Selection in HS mode.  */
        _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GUSBCFG, UX_DCD_DWC2_OTG_GUSBCFG_ULPI_UTMI_SEL);
    }

    /* Wait for 50 cycles.  */
    _ux_dcd_dwc2_delay(50);

    /* Wait for AHB master Idle State.  */
    do
    {

        /* Wait for 5 cycles.  */
        _ux_dcd_dwc2_delay(5);

        /* Read the RST Control register.  */
        dwc2_register =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_GRSTCTL);

    } while ((dwc2_register & UX_DCD_DWC2_OTG_GRSTCTL_AHBIDL) == 0);

    /* Perform the core soft reset.  */
    _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GRSTCTL, UX_DCD_DWC2_OTG_GRSTCTL_CSRST);

    /* Wait for Soft Reset to be completed.  */
    do
    {

        /* Read the RST Control register.  */
        dwc2_register =  _ux_dcd_dwc2_register_read(dcd_dwc2, UX_DCD_DWC2_OTG_GRSTCTL);

    } while (dwc2_register & UX_DCD_DWC2_OTG_GRSTCTL_CSRST);

    /* Wait for 10 cycles.  */
    _ux_dcd_dwc2_delay(10);

    /* Set the controller to device mode.  */
    _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GUSBCFG, UX_DCD_DWC2_OTG_GUSBCFG_FDMOD);

    /* Wait for 50 cycles.  */
    _ux_dcd_dwc2_delay(50);

    /* Check the controller. Full Speed or HighSpeed initialization.  */
    if (dcd_dwc2 -> ux_dcd_dwc2_base == UX_DCD_DWC2_OTG_FS_FULL_SPEED)

        /* Set the PHY speed to full speed. */
        _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_DCFG, UX_DCD_DWC2_OTG_DCFG_DSPD_FULL_SPEED);

    /* Set the turnaround time.  */
    _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_GUSBCFG, UX_DCD_DWC2_OTG_GUSBCFG_TRDT_MASK);
    _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GUSBCFG, UX_DCD_DWC2_OTG_GUSBCFG_TRDT_8);
    /* Restart the PHY clock. */
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_PCGCCTL, 0);

    /* Set the Device default address to 0 and the Periodic Frame interval to 80%. */
    _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_DCFG,
                                (UX_DCD_DWC2_OTG_DCFG_PFVIL_MASK | UX_DCD_DWC2_OTG_DCFG_DAD_MASK));

    /* Set the RX FIFO. Size.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GRXFSIZ, UX_DCD_DWC2_RX_FIFO_SIZE);
    /* Set the fifo address.  */
    fifo_address = UX_DCD_DWC2_RX_FIFO_SIZE;

    /* Set the NP TX FIFO. Size.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GNPTXFSIZ, (fifo_address |
                                (UX_DCD_DWC2_NP_TX_FIFO_SIZE << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));

    /* Adjust the fifo address.  */
    fifo_address += UX_DCD_DWC2_NP_TX_FIFO_SIZE;

    /* Setup the IN FIFOs. Check the controller. Full-Speed or High-Speed initialization.  */
    if (dcd_dwc2 -> ux_dcd_dwc2_base == UX_DCD_DWC2_OTG_HIGH_SPEED)
    {

        /* For OTG_HS, the maximum amount of RAM we have for FIFOs is 4kb.  */

        /* Set the values for the IN Fifos (5 generic IN Fifos). */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));

        /* Adjust the fifo address.  */
        fifo_address += UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS;
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF + 4, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));

        /* Adjust the fifo address.  */
        fifo_address += UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS;
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF + 8, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));

        /* Adjust the fifo address.  */
        fifo_address += UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS;
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF + 12, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));

        /* Adjust the fifo address.  */
        fifo_address += UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS;
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF + 16, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_HS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));
    }
    else
    {

        /* For OTG_FS, the maximum amount of RAM we have for FIFOs is 1.25kb.  */

        /* Set the values for the IN Fifos (3 generic IN Fifos). */
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_FS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));

        /* Adjust the fifo address.  */
        fifo_address += UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_FS;
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF + 4, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_FS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));

        /* Adjust the fifo address.  */
        fifo_address +=     UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_FS;
        _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPTXF + 8, (fifo_address |
                                    (UX_DCD_DWC2_ENDPOINT_TX_FIFO_SIZE_FS << UX_DCD_DWC2_OTG_GNPTXFSIZ_NPTXFSD_SHIFT)));
    }

    /* Clear DIEPMSK and DOEPMSK.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DIEPMSK, 0);
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DOEPMSK, 0);

    /* Clear all pending interrupts on endpoint.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DAINT, 0xFFFFFFFF);
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_DAINTMSK, 0);

    /* Disable all interrupts.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GINTMSK, 0);

    /* Clear all pending interrupts.  */
    _ux_dcd_dwc2_register_write(dcd_dwc2, UX_DCD_DWC2_OTG_GINTSTS, 0xFFFFFFFF);

    /* Enable interrupts for the device mode only.  */
    _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GINTMSK, UX_DCD_DWC2_OTG_GINTMSK_WKUINTM  |
                                                                 UX_DCD_DWC2_OTG_GINTMSK_RFXLVLM  |
                                                                 UX_DCD_DWC2_OTG_GINTMSK_DISCINTM |
                                                                 UX_DCD_DWC2_OTG_GINTMSK_USBSUSPM |
                                                                 UX_DCD_DWC2_OTG_GINTMSK_USBRSTM  |
                                                                 UX_DCD_DWC2_OTG_GINTMSK_ENUMDNEM |
                                                                 UX_DCD_DWC2_OTG_GINTMSK_IEPINTM  |
                                                                 UX_DCD_DWC2_OTG_GINTMSK_OEPINTM);


    /* Set the state of the controller to OPERATIONAL now.  */
    dcd -> ux_slave_dcd_status =  UX_DCD_STATUS_OPERATIONAL;

    /* Set the GINT Global Interrupt Mask register.  */
    _ux_dcd_dwc2_register_set(dcd_dwc2, UX_DCD_DWC2_OTG_GAHBCFG, UX_DCD_DWC2_OTG_GAHBCFG_GINT);

    /* Clear soft disconnect bit.  */
    _ux_dcd_dwc2_register_clear(dcd_dwc2, UX_DCD_DWC2_OTG_DCTL, UX_DCD_DWC2_OTG_DCTL_SDIS);

    /* Return successful completion.  */
    return(UX_SUCCESS);
}
