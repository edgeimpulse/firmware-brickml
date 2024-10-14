/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "qspi.h"
#include "hal_data.h"

/* QSPI flash address through page*/
#define QSPI_FLASH_WRITE_ADDRESS(page_no)       (uint8_t *) (QSPI_DEVICE_START_ADDRESS + (page_no * PAGE_WRITE_SIZE))

// After R_QSPI_Open() completes successfully, the QSPI flash device contents are mapped to address 0x60000000 and can be read like on-chip flash
#define QSPI_FLASH_READ_ADDRESS(address)        (uint8_t *) (QSPI_DEVICE_START_ADDRESS + address)


#define W25QXX_COMMAND_VOLATILE_SR_WRITE_ENABLE (0x50)

#define W25QXX_COMMAND_READ_STATUS_REG2         (0x35)
#define W25QXX_CMD_WRITE_ST_REG2                (0x31)
/* QPI mode entry command */
#define W25QXX_CMD_ENTER_QSPI_MODE              (0x38)

//
static bool qspi_is_init = false;
static uint32_t size;

static fsp_err_t get_qspi_flash_status(void);
static uint32_t qspi_get_size(void);
static uint32_t qspi_get_clear_size(uint32_t desired_size, uint32_t sector_size);
static uint32_t qspi_get_write_size(uint32_t desired_size);
static fsp_err_t qpi_mode_set(void);

static uint8_t *p_mem_addr                            = (uint8_t *)QSPI_DEVICE_START_ADDRESS;

/* one byte data transfer */
#define ONE_BYTE                        (0x01)

/**
 *
 * @return
 */
uint32_t qspi_init(void)
{
    fsp_err_t error = FSP_SUCCESS;

    if (SPI_FLASH_PROTOCOL_QPI == g_qspi0_cfg.spi_protocol) {
        /*
           * this needs to be done since QPI is set by user in configuration
           * and it sets QPI only in MCU but not in flash device
           * so as a system (MCU + QSPI flash device) QPI mode does not get set by
           * simply calling only R_QSPI_Open in QPI mode.
           * Rather QPI mode enabling has to be done in Flash device as well
           * So opening the driver in extended SPI mode only
           * and QPI mode is enabled when qpi_mode_set sub-function is called
           */
          spi_flash_cfg_t l_qspi_cfg;

          memcpy((spi_flash_cfg_t *)&l_qspi_cfg, (spi_flash_cfg_t *)&g_qspi0_cfg, sizeof (spi_flash_cfg_t));

          l_qspi_cfg.spi_protocol = SPI_FLASH_PROTOCOL_EXTENDED_SPI;
          error = R_QSPI_Open(&g_qspi0_ctrl, &l_qspi_cfg);

    }
    else {
        error = R_QSPI_Open(&g_qspi0_ctrl, &g_qspi0_cfg);
    }


    if (FSP_SUCCESS == error) {
        /* write enable for further operations */
        error = R_QSPI_DirectWrite(&g_qspi0_ctrl, &(g_qspi0_cfg.write_enable_command), ONE_BYTE, false);    // enable write

        error = get_qspi_flash_status();

    }

    if (SPI_FLASH_PROTOCOL_QPI == g_qspi0_cfg.spi_protocol)
    {
        /* set QPI mode in flash and MCU device */
        error = qpi_mode_set();
        if (FSP_SUCCESS != error) {

        }
    }

    if (FSP_SUCCESS == error) {
        qspi_is_init = true;
        size = qspi_get_size();
    }



    return (uint32_t)size;
}

/**
 *
 * @return
 */
uint32_t qspi_deinit(void)
{
    fsp_err_t error = FSP_SUCCESS;

    error = R_QSPI_Close(&g_qspi0_ctrl);

    if (FSP_SUCCESS == error) {
        qspi_is_init = false;
    }

    return (uint32_t)error;
}

/**
 *
 * @param data
 * @param bytes
 * @param address
 * @return
 */
uint32_t qspi_read(uint8_t* data, uint32_t bytes, uint32_t address)
{
    uint32_t bytes_read = 0;

    if (data != NULL) {
        memcpy(data, (uint8_t *)QSPI_FLASH_READ_ADDRESS(address), bytes);
        bytes_read = bytes;
    }

    return (uint32_t)bytes_read;
}

/**
 *
 * @param data
 * @param bytes
 * @param address
 * @return
 */
uint32_t qspi_write(const uint8_t* data, uint32_t bytes, uint32_t address)
{
    fsp_err_t error = FSP_SUCCESS;
    uint32_t bytes_written = bytes;
    uint32_t blocks_to_write = qspi_get_write_size(bytes);
    uint32_t i;

    for (i = 0; i < blocks_to_write; i++) {
        /* Write data to QSPI Flash */
        error = R_QSPI_Write(&g_qspi0_ctrl, &data[i * PAGE_WRITE_SIZE], (uint8_t *)(QSPI_FLASH_WRITE_ADDRESS(i) + address), PAGE_WRITE_SIZE);

        if (FSP_SUCCESS == error) {
            error = get_qspi_flash_status();
        }

        if (FSP_SUCCESS != error) {
            break;
        }
    }

    if (FSP_SUCCESS != error) {
        bytes_written = 0;
    }

    return (uint32_t)bytes_written;
}

/**
 *
 * @param address
 * @param bytes
 * @return
 */
uint32_t qspi_erase(uint32_t address, uint32_t bytes)
{
    fsp_err_t error = FSP_SUCCESS;
    uint32_t blocks_number = 0;
    uint32_t bytes_to_clear = bytes;
    uint32_t sector_size = 32768;

    //if ()
    blocks_number = qspi_get_clear_size(bytes, 32768);

    // address need to be 4k aligned !

    for (uint16_t i = 0; i < blocks_number; i++) {
        error = R_QSPI_Erase(&g_qspi0_ctrl, address + p_mem_addr + (i * sector_size), (sector_size));
        if (error == FSP_SUCCESS) {
            error = get_qspi_flash_status();
        }

        if (error != FSP_SUCCESS) {
            break;
        }
    }

    if (error != FSP_SUCCESS) {
        bytes_to_clear = 0;
    }

    return bytes_to_clear;
}

/**
 * @brief test the QSPI operation
 * @return
 */
uint32_t qspi_test(void)
{
    const uint16_t test_size = 512;
    uint8_t test_write[test_size];
    uint8_t test_read[test_size];
    uint16_t i;
    uint32_t start;
    uint32_t end;

    xTaskGetTickCount();

    if (qspi_erase(0, test_size) != test_size) {
        return 500;
    }

    if (qspi_read(test_read, test_size, 0) != test_size) {
        return 502;
    }

    for (i = 0; i< test_size; i++) {
        if (test_read[i] != 0xFF) {
            return (i+1);
        }
    }

    for (i = 0; i< test_size; i++) {
        test_write[i] = (uint8_t)i;
    }

    start = xTaskGetTickCount();
    if (qspi_write(test_write, test_size, 0) != test_size) {
        return 501;
    }
    end = xTaskGetTickCount();

    if (qspi_read(test_read, test_size, 0) != test_size) {
        return 502;
    }

    for (i = 0; i< test_size; i++) {
        if (test_write[i] != test_read[i]) {
            return 503;
        }
    }

    return (end - start);
}


/**
 *
 * @return
 */
static fsp_err_t get_qspi_flash_status(void)
{
    spi_flash_status_t status = {.write_in_progress = true};
    int32_t time_out          = (INT32_MAX);
    fsp_err_t err             = FSP_SUCCESS;

    do {
        /* Get status from QSPI flash device */
        err = R_QSPI_StatusGet(&g_qspi0_ctrl, &status);
        if (FSP_SUCCESS!= err) {
            return err;
        }

        /* Decrement time out to avoid infinite loop in case of consistent failure */
        --time_out;

        if ( 0 >= time_out) {
            return FSP_ERR_TIMEOUT;
        }

    }while (false != status.write_in_progress);

    return err;
}

#define QSPI_EXAMPLE_COMMAND_READ_ID      (0x9F)
#define QSPI_EXAMPLE_COMMAND_READ_SFDP    (0x5A)

/**
 *
 * @return
 */
static uint32_t qspi_get_size(void)
{
    uint32_t  device_size_bytes;
    fsp_err_t err;

    uint8_t data[4];
    data[0] = QSPI_EXAMPLE_COMMAND_READ_ID;
    err     = R_QSPI_DirectWrite(&g_qspi0_ctrl, &data[0], 1, true);
    assert(FSP_SUCCESS == err);
    /* Read 3 bytes. The third byte often represents the size of the QSPI, where the size of the QSPI = 2 ^ N. */
    err = R_QSPI_DirectRead(&g_qspi0_ctrl, &data[0], 3);
    assert(FSP_SUCCESS == err);
    device_size_bytes = 1U << data[2];

    return device_size_bytes;
}

/**
 *
 * @param desired_size
 * @return
 */
static uint32_t qspi_get_clear_size(uint32_t desired_size, uint32_t sector_size)
{
    if (desired_size < sector_size)
    {
        return 1u;
    }
    else
    {
        return (desired_size % sector_size == 0) ? desired_size/sector_size : (desired_size/sector_size + 1);
    }
}



/**
 *
 * @param desired_size
 * @return
 */
static uint32_t qspi_get_write_size(uint32_t desired_size)
{
    if (desired_size < PAGE_WRITE_SIZE)
    {
        return 1u;
    }
    else
    {
        return (desired_size % PAGE_WRITE_SIZE == 0) ? desired_size/PAGE_WRITE_SIZE : (desired_size/PAGE_WRITE_SIZE + 1);
    }
}

/* QPI mode exit command */
#define QSPI_MX25L_CMD_EXIT_QPI_MODE    (0xF5)

/*******************************************************************************************************************//**
 *  @brief       set QPI Mode in flash device and MCU
 *  @param[IN]   none
 *  @retval      FSP_SUCCESS or any other possible error codes
 **********************************************************************************************************************/
static fsp_err_t qpi_mode_set(void)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t read_reg = 0;
    uint8_t command = W25QXX_COMMAND_READ_STATUS_REG2;
    uint8_t comm_buf[3] = {0};

    /*
     Read status
     check bit
     volatile WR enable
     write status reg 2
     write bit
    */
    err = R_QSPI_DirectWrite(&g_qspi0_ctrl, &(command), ONE_BYTE, true);
    if (err != FSP_SUCCESS) {
        return err;
    }
    else {
        err = get_qspi_flash_status();
        if (FSP_SUCCESS != err)
        {

            return err;
        }

    }
    err = R_QSPI_DirectRead(&g_qspi0_ctrl, &read_reg, ONE_BYTE);

    if ((read_reg & 0x02) == 0x02) {
        return FSP_SUCCESS; // no need
    }


    /* write enable once again section 9-1 states that
     * we should do it before sending 0x35 to flash device
     */
    err = R_QSPI_DirectWrite(&g_qspi0_ctrl, &(g_qspi0_cfg.write_enable_command), ONE_BYTE, false);
    if (FSP_SUCCESS != err)
    {

        return err;
    }
    else
    {
        err = get_qspi_flash_status();
        if (FSP_SUCCESS != err)
        {

            return err;
        }
    }
    command = W25QXX_COMMAND_VOLATILE_SR_WRITE_ENABLE;
    err = R_QSPI_DirectWrite(&g_qspi0_ctrl, &(command), ONE_BYTE, true);
    if (err != FSP_SUCCESS) {
        return err;
    }
    else {
        err = get_qspi_flash_status();
        if (FSP_SUCCESS != err)
        {

            return err;
        }

    }

    /* send QPI mode enable command in flash device
     * Note - no status register read after this operation
     * because flash device has gone in QPI mode
     * and MCU at this point is in extended SPI mode only.
     * vice versa same is applicable while exiting QPI mode too.
     */
    command = W25QXX_CMD_WRITE_ST_REG2;
    comm_buf[0] = W25QXX_CMD_WRITE_ST_REG2;
    comm_buf[1] = 0x02;
    err = R_QSPI_DirectWrite(&g_qspi0_ctrl, &comm_buf[0], 2, false);
    err = get_qspi_flash_status();
    if (FSP_SUCCESS != err) {

        return err;
    }
    /*
    command = W25QXX_CMD_ENTER_QSPI_MODE;
    err = R_QSPI_DirectWrite(&g_qspi0_ctrl, &command, ONE_BYTE, false);
    err = get_qspi_flash_status();
    if (FSP_SUCCESS != err)
    {

        return err;
    }*/
    
    // check if set
    command = W25QXX_COMMAND_READ_STATUS_REG2;
    err = R_QSPI_DirectWrite(&g_qspi0_ctrl, &(command), ONE_BYTE, true);
    if (err != FSP_SUCCESS) {
        return err;
    }
    else {
        err = get_qspi_flash_status();
        if (FSP_SUCCESS != err)
        {

            return err;
        }

    }
    err = R_QSPI_DirectRead(&g_qspi0_ctrl, &read_reg, ONE_BYTE);
    if ((read_reg & 0x02) == 0) {
        return FSP_SUCCESS; // no need
    }

    /* Command byte transferred to flash-> NOW  set the QPI protocol in MCU run time */
    err = R_QSPI_SpiProtocolSet(&g_qspi0_ctrl, SPI_FLASH_PROTOCOL_QPI);
    if (FSP_SUCCESS != err) {

    }

    return err;
}

