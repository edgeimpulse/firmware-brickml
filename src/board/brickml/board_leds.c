/***********************************************************************************************************************
 * Copyright [2020-2022] Renesas Electronics Corporation and/or its affiliates.  All Rights Reserved.
 *
 * This software and documentation are supplied by Renesas Electronics America Inc. and may only be used with products
 * of Renesas Electronics Corp. and its affiliates ("Renesas").  No other uses are authorized.  Renesas products are
 * sold pursuant to Renesas terms and conditions of sale.  Purchasers are solely responsible for the selection and use
 * of Renesas products and Renesas assumes no liability.  No license, express or implied, to any intellectual property
 * right is granted by Renesas. This software is protected under all applicable laws, including copyright laws. Renesas
 * reserves the right to change or discontinue this software and/or this documentation. THE SOFTWARE AND DOCUMENTATION
 * IS DELIVERED TO YOU "AS IS," AND RENESAS MAKES NO REPRESENTATIONS OR WARRANTIES, AND TO THE FULLEST EXTENT
 * PERMISSIBLE UNDER APPLICABLE LAW, DISCLAIMS ALL WARRANTIES, WHETHER EXPLICITLY OR IMPLICITLY, INCLUDING WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT, WITH RESPECT TO THE SOFTWARE OR
 * DOCUMENTATION.  RENESAS SHALL HAVE NO LIABILITY ARISING OUT OF ANY SECURITY VULNERABILITY OR BREACH.  TO THE MAXIMUM
 * EXTENT PERMITTED BY LAW, IN NO EVENT WILL RENESAS BE LIABLE TO YOU IN CONNECTION WITH THE SOFTWARE OR DOCUMENTATION
 * (OR ANY PERSON OR ENTITY CLAIMING RIGHTS DERIVED FROM YOU) FOR ANY LOSS, DAMAGES, OR CLAIMS WHATSOEVER, INCLUDING,
 * WITHOUT LIMITATION, ANY DIRECT, CONSEQUENTIAL, SPECIAL, INDIRECT, PUNITIVE, OR INCIDENTAL DAMAGES; ANY LOST PROFITS,
 * OTHER ECONOMIC DAMAGE, PROPERTY DAMAGE, OR PERSONAL INJURY; AND EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY
 * OF SUCH LOSS, DAMAGES, CLAIMS OR COSTS.
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @addtogroup BOARD_RA6M5_EK_LEDS
 *
 * @{
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "bsp_api.h"
#include "board_leds.h"
#include "common_data.h"

#if 1 // defined(BOARD_BRICKML)

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private global variables and functions
 **********************************************************************************************************************/

/** Array of LED IOPORT pins. */
static const uint16_t g_bsp_prv_leds[] =
{
    (uint16_t) BSP_IO_PORT_00_PIN_07,  ///< LED1
    (uint16_t) BSP_IO_PORT_06_PIN_02,  ///< LED2
    (uint16_t) BSP_IO_PORT_00_PIN_01,  ///< LED3
};

/***********************************************************************************************************************
 * Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/

/** Structure with LED information for this board. */

const bsp_leds_t g_bsp_leds =
{
    .led_count = (uint16_t) ((sizeof(g_bsp_prv_leds) / sizeof(g_bsp_prv_leds[0]))),
    .p_leds    = &g_bsp_prv_leds[0]
};

/***********************************************************************************************************************
 * Exported global variables (to be accessed by other files)
 **********************************************************************************************************************/

#endif

/*******************************************************************************************************************//**
 * @brief This function updates led state as per operation status
 * @param[in]  led_state      Selects which led has to be made high
 * @retval     None
 **********************************************************************************************************************/
void led_update(led_state_t led_state)
{
    switch(led_state)
    {
        case red:
        {
            /* Red LED state is made high to show error state */
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[0], BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[1], BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[2], BSP_IO_LEVEL_LOW);
            /* Delay */
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);
            break;
        }
        case green:
        {
            /* Green LED state is made high to show succesful state */
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[0], BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[1], BSP_IO_LEVEL_LOW);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[2], BSP_IO_LEVEL_HIGH);
            /* Delay */
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);
            break;
        }
        case blue:
        {
            /* Blue LED state is made high to show operation is in progress */
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[0], BSP_IO_LEVEL_LOW);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[1], BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[2], BSP_IO_LEVEL_HIGH);
            /* Delay */
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);
            break;
        }
        default:
        {
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[0], BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[1], BSP_IO_LEVEL_HIGH);
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[2], BSP_IO_LEVEL_HIGH);
            /* Delay */
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MICROSECONDS);
            break;
        }
    }
}

void led_on(void)
{
    led_green_on();
}

void led_off(void)
{
    led_green_off();
}

void led_toggle(void)
{
    led_green_toggle();
}

void led_red_on(void)
{
    g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_00_PIN_01, 1 );
}

void led_red_off(void)
{
    g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_00_PIN_01, 0 );
}

void led_red_toggle(void)
{
    static uint8_t led_red_status = 1;
#if 1
    if (led_red_status == 0)
    {
        led_red_status = 1;
        led_red_on();
    }
    else
    {
        led_red_status = 0;
        led_red_off();
    }
#endif
}

void led_green_on(void)
{
    g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_06_PIN_02, 1 );
}

void led_green_off(void)
{
    g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_06_PIN_02, 0 );
}

void led_green_toggle(void)
{
    static uint8_t led_green_status = 1;
#if 1
    if (led_green_status == 0)
    {
        led_green_status = 1;
        led_green_on();
    }
    else
    {
        led_green_status = 0;
        led_green_off();
    }
#endif
}

void led_blue_on(void)
{
    g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_00_PIN_07, 1 );
}

void led_blue_off(void)
{
    g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_00_PIN_07, 0 );
}

void led_blue_toggle(void)
{
    static uint8_t led_green_status = 1;
#if 1
    if (led_green_status == 0)
    {
        led_green_status = 1;
        led_blue_on();
    }
    else
    {
        led_green_status = 0;
        led_blue_off();
    }
#endif
}

void pin_toggle(void)
{
#if 0
    static uint8_t pin_status = 0;
    if (pin_status == 0)
    {
        ASSERT( g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_02_PIN_05, 1  ) == FSP_SUCCESS);
        pin_status = 1;
    }
    else
    {
        pin_status = 0;
        ASSERT( g_ioport.p_api->pinWrite( g_ioport.p_ctrl, BSP_IO_PORT_02_PIN_05, 0  ) == FSP_SUCCESS);
    }
#endif
}

/** @} (end addtogroup BOARD_RA6M5_EK_LEDS) */
