/*
Author      - Calvin Gross
Date        - 8/11/26
Modified    - 8/12/26
Title       - SD Card SPI
Project     - Full Attitude and Heading Reference System
Description - In this file I abstract away the SPI hardware by declaring 7 functions
            at the bottom of this file. The first 2 are setting the clock speed, which 
            allows for the necessary slow Initialization speed for SPI. The next 2 are 
            for controlling the chip select. Finally, the last 3 are for transmitting 
            and receiving data.
*/


/** Function to pass the preconfigured SDI_HandleTypeDef and Chip Select pin into a handle
    for the state passed to the functions at this abstraction layer (this file).
 */

#include "sd_spi.h"
#include <string.h>


HAL_StatusTypeDef SD_SPI_Init(SD_SPI_Handle *new_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin, SPI_HandleTypeDef *hspi) {
    HAL_StatusTypeDef status;

    if (new_handle == NULL || cs_port == NULL || cs_pin == 0 || hspi == NULL) {
        return HAL_ERROR;
    }
    new_handle->hspi = hspi; 
    new_handle->cs_port = cs_port;
    new_handle->cs_pin = cs_pin;

    status = SD_SPI_Deselect_Card(new_handle);
    if (status != HAL_OK) return status;

    return SD_SPI_SetInitializationSpeed(new_handle);
}

// Functions to set clock speed.
HAL_StatusTypeDef SD_SPI_SetOperatingSpeed(SD_SPI_Handle *cur_handle) {
    if (cur_handle == NULL || cur_handle->hspi == NULL) {
        return HAL_ERROR;
    }
    cur_handle->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    return HAL_SPI_Init(cur_handle->hspi);
}
HAL_StatusTypeDef SD_SPI_SetInitializationSpeed(SD_SPI_Handle *cur_handle) {
    if (cur_handle == NULL || cur_handle->hspi == NULL) {
        return HAL_ERROR;
    }
    cur_handle->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    return HAL_SPI_Init(cur_handle->hspi);
}

// Set and reset Chip Select functions.
HAL_StatusTypeDef SD_SPI_Select_Card(SD_SPI_Handle *cur_handle) {
    if (cur_handle == NULL || cur_handle->cs_port == NULL || cur_handle->cs_pin == 0) {
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(cur_handle->cs_port, cur_handle->cs_pin, GPIO_PIN_RESET);

    return HAL_OK;
}
HAL_StatusTypeDef SD_SPI_Deselect_Card(SD_SPI_Handle *cur_handle) {
        if (cur_handle == NULL || cur_handle->cs_port == NULL || cur_handle->cs_pin == 0) {
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(cur_handle->cs_port, cur_handle->cs_pin, GPIO_PIN_SET);

    return HAL_OK;
}

/** Function to transmit and receive a byte because I am always using full duplex spi.
    Most use-cases will not need both the transmitted_byte or received_byte_adr, in which 
    case 0xFF and NULL should be passed in respectively.
 */
HAL_StatusTypeDef SD_SPI_Interchange_Byte(SD_SPI_Handle *cur_handle, uint8_t transmitted_byte, uint8_t *received_byte_adr) {
    if (cur_handle == NULL || cur_handle->hspi == NULL) {
        return HAL_ERROR;
    }  

    uint8_t discarded_received_byte;
    if (received_byte_adr == NULL) received_byte_adr = &discarded_received_byte;

    return HAL_SPI_TransmitReceive(cur_handle->hspi, &transmitted_byte, received_byte_adr, 1, 100);
}

/** Function to transmit multiple bytes at a time over spi to the configured micro sd card.
 */
HAL_StatusTypeDef SD_SPI_Transmit_Data(SD_SPI_Handle *cur_handle, uint8_t *transmit_adr, uint16_t byte_count) {
    if (cur_handle == NULL || cur_handle->hspi == NULL || transmit_adr == NULL || byte_count == 0) {
        return HAL_ERROR;
    }  

    return HAL_SPI_Transmit(cur_handle->hspi, transmit_adr, byte_count, 100);
}

/** Function to receive multiple bytes at a time over spi from the configured micro sd card.
 */
HAL_StatusTypeDef SD_SPI_Receive_Data(SD_SPI_Handle *cur_handle, uint8_t *receive_adr, uint16_t byte_count) {
    if (cur_handle == NULL || cur_handle->hspi == NULL || receive_adr == NULL || byte_count == 0) {
        return HAL_ERROR;
    }  

    // content of receive_adr is sent over SPI first during HAL_SPI_Receive() so I set it to all 1s.
    memset(receive_adr, 0xFF, byte_count);

    return HAL_SPI_Receive(cur_handle->hspi, receive_adr, byte_count, 100);
}

HAL_StatusTypeDef SD_SPI_Send_Idle_Clocks(SD_SPI_Handle *cur_handle) {
    if (cur_handle == NULL || cur_handle->hspi == NULL) {
        return HAL_ERROR;
    }  

    HAL_StatusTypeDef status;

    status = SD_SPI_Deselect_Card(cur_handle);
    if (status != HAL_OK) return status;

    uint8_t idle_data[10];
    memset(idle_data, 0xff, sizeof(idle_data));

    return SD_SPI_Transmit_Data(cur_handle, idle_data, sizeof(idle_data));
}