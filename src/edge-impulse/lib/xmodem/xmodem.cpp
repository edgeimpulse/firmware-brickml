/***********************************************************************************************************************
 * File Name    : xmodem.c
 * Description  : Contains XModem protocol implementations
 ***********************************************************************************************************************/
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

#include <string.h>
#include "xmodem.h"
#include "comms.h"
//#include "hal_data.h"
#include "peripheral/board_ctrl.h"
#include "usb_thread_interface.h"

static bool StartCondition;

static fsp_err_t xmodem_read_buffer(uint8_t* pbuf, uint32_t *max_len, uint32_t timeout);
static uint8_t erase_partition(uint8_t partition);

/*******************************************************************************************************************//**
 * @brief This function implements the XModem protocol and handles the flash operation
 * @param[IN]   FlashAddress            Start address of the flash where the new image will be downloaded
 * @retval      FSP_SUCCESS             Upon successful FLash_HP data flash operations.
 * @retval      Any Other Error code    Upon unsuccessful Flash_HP data flash operations.
 **********************************************************************************************************************/
unsigned char XmodemDownloadAndProgramFlash (unsigned long FlashAddress)
{
    const char tx_str[] = "\r\nResetting the system\r\n";

/*
XmodemDownloadAndProgramFlash() takes a memory address as the base address to
which data downloaded is programmed.  The data is downloaded using the XModem
protocol developed in 1977 by Ward Christensen.
The routine detects XM_ERRORs due to XM_TIMEOUTs, comms XM_ERRORs or invalid checksums.
The routine reports the following to the caller:
-Success
-Invalid address	(not implemented here as the address is checked prior to this
					function being called)
-Comms XM_ERROR
-XM_TIMEOUT XM_ERROR
-Failure to program flash


Expects:    
--------
FlashAddress:
32-bit address located in Flash memory space starting on a 128-byte boundary

Returns:
--------
XM_OK				-	Download and Flash programming performed ok
XM_ADDRESS_XM_ERROR 	-	Address was either not on a 128-bit boundary or not in valid Flash
XM_COMMS_XM_ERROR	 	-	Comms parity, framing or overrun XM_ERROR
XM_XM_TIMEOUT			-	Transmitter did not respond to this receiver
XM_PROG_FAIL		-	Failed to program one or more bytes of the Flash memory
*/
	unsigned char   xm_ret;
	unsigned char   ExpectedBlkNum;
	unsigned char   RetryCounter;
	unsigned char   RxByteBufferIndex;
	unsigned char   checksum;
	fsp_err_t err;
	unsigned long   Address;
	unsigned char   RxByteBuffer[132] BSP_ALIGN_VARIABLE(4);
	uint8_t         tx_byte BSP_ALIGN_VARIABLE(4);
	uint32_t        rx_len;
	uint8_t         local_try = 10;
	uint8_t         erase_retval = 0;

	usb_clean_buffer();
	// first xmodem block number is 1
	ExpectedBlkNum = 1;
	
	StartCondition = true;
	
	Address = FlashAddress;

	erase_retval = erase_partition(1);

	while(1)
	{
		//	{1}
		//	initialise Rx attempts
		RetryCounter = 10;
	
		//	decrement Rx attempts counter & get Rx byte with a 10 sec TIMEOUT repeat until Rx attempts is 0
		xm_ret = XM_TOUT;
		while ( (RetryCounter > 0) && (xm_ret == XM_TOUT) )
		{

		    if(true == StartCondition)
			{
				//	if this is the start of the xmodem frame
				//	send a NAK to the transmitter
			    tx_byte = NAK;
			    do{
	                comms_send(&tx_byte, 1, 100);    // Kick off the XModem transfer

	                /* Request 132 bytes from host with a delay of 10 seconds */
	                rx_len = 132;
	                memset((void *)&RxByteBuffer[0], 0, 132);
	                //err = comms_read((uint8_t *)&RxByteBuffer[0], &rx_len, 10000);
	                err = xmodem_read_buffer((uint8_t *)&RxByteBuffer[0], &rx_len, 1000);
	                local_try--;
			    }while(local_try && (err != FSP_SUCCESS));

			}                             
			else
			{
			    /* Request 132 bytes from host with a delay of 1 second */

                rx_len = 132;
                memset((void *)&RxByteBuffer[0], 0, 132);
                //err = comms_read((uint8_t *)&RxByteBuffer[0], &rx_len, 1000);
                err = xmodem_read_buffer((uint8_t *)&RxByteBuffer[0], &rx_len, 1000);

			}
		    if(FSP_SUCCESS == err)
            {
                xm_ret = OK;
            }
            else if(FSP_ERR_TIMEOUT == err)
            {
                xm_ret = XM_TOUT;
            }
            else
            {
                xm_ret = XM_ERROR;
            }
			RetryCounter--;
		}
			

		StartCondition = false;

		if ( xm_ret == XM_ERROR )
        {
            return ( XM_ERROR );
        }
		else if ( xm_ret == XM_TOUT )
		{
		    //  if timed out after 10 attempts
		    return ( XM_TIMEOUT );
		}
		else			
		{
			// if first received byte is 'end of frame'
			// return ACK to sender
		    if ( RxByteBuffer[0] == EOT )
			{
		        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
                tx_byte = ACK;
                comms_send(&tx_byte, 1, 100);

                comms_send((uint8_t *)tx_str, strlen((char *)tx_str), 100);
                NVIC_SystemReset();

                return( XM_OK );

			}
			else
			{				
                // data Rx ok
                // calculate the checksum of the data bytes only
                checksum = 0;
                int cs = 0;

                /* check if the data make sense */
                for (RxByteBufferIndex=0; RxByteBufferIndex<128; RxByteBufferIndex++)
                {
                    cs += RxByteBuffer[RxByteBufferIndex + 3];
                }

                /* This int to unsigned char conversion needed to eliminate conversion warning with checksum calculation. */
                checksum = (unsigned char)cs;

                //	if SOH, BLK#, 255-BLK# or checksum not valid
                //	(BLK# is valid if the same as expected blk counter or is 1 less
                if ( !( (RxByteBuffer[0] == SOH) && ((RxByteBuffer[1] == ExpectedBlkNum) || (RxByteBuffer[1] == ExpectedBlkNum - 1) ) && (RxByteBuffer[2] + RxByteBuffer[1] == 255 ) && (RxByteBuffer[131] == checksum) ) )
                {
                    //	send NAK and loop back to (1)
                    tx_byte = NAK;
                    comms_send(&tx_byte, 1, 100);
                }
                else
                {
                    //	if blk# is expected blk num
                    if ( RxByteBuffer[1] == ExpectedBlkNum )
                    {
                        //	Program the received data into flash

                        /* Assume flash area is erased */

                        int32_t status = 0;
                        ThreadsAndInterrupts(DISABLE);
                        status = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t)&RxByteBuffer[3], (uint32_t)Address, 128);
                        ThreadsAndInterrupts(RE_ENABLE);
                        if(FSP_SUCCESS == status)
                        {
                            Address += 128;
                            ExpectedBlkNum++;
                            tx_byte = ACK;
                            comms_send(&tx_byte, 1, 100);
                        }
                        else
                        {
                            // prog fail
                            tx_byte = NAK;
                            comms_send(&tx_byte, 1, 100);
                            // cancel xmodem download
                            tx_byte = CAN;
                            comms_send(&tx_byte, 1, 100);

                            return( XM_PROG_FAIL );
                        }
                    }
                    else
                    {
                        // 	block number is valid but this data block has already been received
                        //	send ACK and loop to (1)
                        tx_byte = ACK;
                        comms_send(&tx_byte, 1, 100);
                    }
                }
            }
		}
	}		
}

/**
 *
 * @param pbuf
 * @param max_len
 * @param timeout
 * @return
 */
static fsp_err_t xmodem_read_buffer(uint8_t* pbuf, uint32_t *max_len, uint32_t timeout)
{
    fsp_err_t retval = FSP_SUCCESS;
    uint32_t read = 0;

    do{
        vTaskDelay(1);
        timeout--;
        read = ei_get_serial_buffer(pbuf, *max_len);
    } while((read == 0) && timeout);

    if (read == 0) {
        retval = FSP_ERR_ABORTED;
    }
    *max_len = read;

    return retval;
}

/**
 *
 * @param partition
 * @return
 */
static uint8_t erase_partition(uint8_t partition)
{
    uint8_t retval = 0;
    volatile fsp_err_t err = FSP_SUCCESS;
    flash_result_t blank_check_result = FLASH_RESULT_BLANK;

    (void)partition;

    ThreadsAndInterrupts(DISABLE);

    err = R_FLASH_HP_Erase(&g_flash0_ctrl, (uint32_t) SECONDARY_IMAGE_START_ADDRESS, SECONDARY_IMAGE_NUM_BLOCKS);
    if(FSP_SUCCESS == err)
    {
        err = R_FLASH_HP_BlankCheck(&g_flash0_ctrl, SECONDARY_IMAGE_START_ADDRESS, SECONDARY_IMAGE_END_ADDRESS - SECONDARY_IMAGE_START_ADDRESS + 1, &blank_check_result);
        if(FSP_SUCCESS == err)
        {
            ThreadsAndInterrupts(RE_ENABLE);
            if (FLASH_RESULT_BLANK != blank_check_result)
            {
                retval = 1;
            }
        }
        else
        {
            retval = 2;
        }
    }
    else
    {
        retval = 3;
    }


    ThreadsAndInterrupts(RE_ENABLE);

    return retval;
}
