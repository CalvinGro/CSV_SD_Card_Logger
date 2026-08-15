/*
Author      - Calvin Gross
Date        - 8/12/26
Title       - SD Card SPI
Project     - Full Attitude and Heading Reference System
Description - In this file I abstract away the SD Card protocols. This handles
            all of the command packet communication with the Card. 
*/

#include "sd_spi.h"
#include "sd_card.h"
#include <string.h>
#include <stdbool.h>


/** Calculate the Cyclic Redundancy Check (CRC) for any 16-bit pattern. 
    Adds the extra 15 bits to the end for building a crc to send.

    Returns the final two byte frame check sequence (FCS) in the response_adr.
 */
HAL_StatusTypeDef Calc_CRC16(
    uint16_t pattern, 
    const uint8_t *data, 
    uint32_t byte_count, 
    uint16_t *response_adr, 
    bool building_crc
    ) {
        if (byte_count < 2 || response_adr == NULL || data == NULL) {
            return HAL_ERROR;
        }
        uint16_t cur_data = (data[0] << 8) | data[1];
        for (uint32_t i = 2; i < byte_count; i++) {
            for (uint8_t j = 0; j < 8; j++) {
                bool bit16_set = (cur_data & 0x8000) == 0x8000;
                // Send each byte MSB first, so (7-j) flips the order.
                cur_data = (uint16_t)((cur_data << 1) | ((data[i] & (uint16_t)1 << (7 - j)) >> (7 - j)));
                if (bit16_set) {
                    cur_data ^= pattern;
                }
            }
        }

        if (building_crc) {
            for (uint8_t i = 0; i < 16; i++) {
                bool bit16_set = (cur_data & 0x8000) == 0x8000;
                cur_data = (uint16_t)(cur_data << 1);

                if (bit16_set) {
                    cur_data ^= pattern;
                }
            }
        }
        *response_adr = cur_data;

        return HAL_OK;
}


/** This function waits until the card is ready to communicate over SPI.
    It assumes that the card is already selected.
 */
HAL_StatusTypeDef Wait_Till_Ready(Card_Handle *card) {
    if (card == NULL || card->spi_handle == NULL) return HAL_ERROR;

    HAL_StatusTypeDef status;
    uint8_t received_byte;
    uint32_t start_time = HAL_GetTick();
    do {
        status = SD_SPI_Interchange_Byte(card->spi_handle, 0xff, &received_byte);
        if (status != HAL_OK) return status;
    } while ((HAL_GetTick() - start_time < 300) && received_byte != 0xff);

    if (received_byte == 0xff) return HAL_OK;
    return HAL_TIMEOUT;
}


HAL_StatusTypeDef Wait_To_Receive(Card_Handle *card) {
    if (card == NULL || card->spi_handle == NULL) return HAL_ERROR;

    HAL_StatusTypeDef status;
    uint8_t received_byte;
    uint32_t start_time = HAL_GetTick();
    do {
        status = SD_SPI_Interchange_Byte(card->spi_handle, 0xff, &received_byte);
        if (status != HAL_OK) return status;
        if ((received_byte != 0xff) && (received_byte != 0xfe)) {
            return HAL_ERROR;
        }
    } while ((HAL_GetTick() - start_time < 300) && received_byte != 0xfe);

    if (received_byte == 0xff) return HAL_OK;
    return HAL_TIMEOUT;
}


/** This function sends the specified command to the card using the sd card protocol.
    The command is six bytes, the first is the cmd type, the next four are the command
    argument, then the final byte is the crc, which represents the end of the command
    and is calculated based off the command and its argument.

    The card should already be selected before sending commands.

    Additionally, this function handles the R1 response, but all other responses should
    be handled by the caller.
 */
HAL_StatusTypeDef Send_Command(Card_Handle *card, uint8_t cmd, uint32_t cmd_arg, uint8_t *R1_adr) {
    if (card == NULL || card->spi_handle == NULL || R1_adr == NULL) {
        return HAL_ERROR;
    }

    // set start bit to 0 and transmission bit to host-to-card (1).
    cmd &= 0x7f;
    cmd |= 0x40;

    HAL_StatusTypeDef status;
    uint8_t R1_response;

    // calculate crc7 using generator polynomial (pattern).
    uint8_t crc7_byte;
    uint64_t data = (((uint64_t)cmd << 32) | (uint64_t)cmd_arg);
    uint64_t pattern = (uint64_t)0x89 << 39; // divisor 
    uint64_t data_dividend = data << 7;
    for (uint8_t i = 46; i >= 7; i--) {
        if ((data_dividend & ((uint64_t)1 << i)) != 0) {
            data_dividend ^= pattern;
        }
        pattern >>= 1;
    } 
    crc7_byte = ((uint8_t)(data_dividend & (uint64_t)0x7f) << 1) | 1;

    //
    uint8_t command_packet[6];
    command_packet[0] = cmd;
    command_packet[1] = (uint8_t)(data >> 24);
    command_packet[2] = (uint8_t)(data >> 16);
    command_packet[3] = (uint8_t)(data >> 8);
    command_packet[4] = (uint8_t)data;
    command_packet[5] = crc7_byte;

    // transmission loop, this can be repeated up to 3 times in the case of crc errors.
    uint8_t attempt_count = 0;
    while (attempt_count < 3) {
        status = Wait_Till_Ready(card);
        if (status != HAL_OK) return status;

        // send the cmd packet to the card
        status = SD_SPI_Transmit_Data(card->spi_handle, command_packet, sizeof(command_packet));
        if (status != HAL_OK) return status;
        
        // listen for R1 response
        uint32_t start_time = HAL_GetTick();
        do {
            status = SD_SPI_Interchange_Byte(card->spi_handle, 0xff, &R1_response);
            if (status != HAL_OK) return status;
        } while ((HAL_GetTick() - start_time < 50) && (R1_response & 0x80) == 0x80);
        if ((R1_response & 0x80) == 0x80) return HAL_TIMEOUT;

        // if no crc error don't repeat the transmission
        if ((R1_response & 0x08) != 0x08) {
            break;
        }

        attempt_count++;
    }

    *R1_adr = R1_response;

    if (attempt_count == 3) return HAL_ERROR;
    return HAL_OK;
}

/** Calculate the number of sectors in the current card.
 */
HAL_StatusTypeDef Get_Sector_Count(Card_Handle *card, uint64_t *sector_count_adr) {
    if (card == NULL || card->spi_handle == NULL || sector_count_adr == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = SD_SPI_Select_Card(card->spi_handle);
    if (status != HAL_OK) goto cleanup;

    // retrieve CSD register
    uint8_t attempt_count = 0;
    uint16_t response;
    uint8_t CSD_reg[18];
    do {
        uint8_t cmd9 = 0x49;
        uint32_t cmd9_arg = 0;
        uint8_t R1_response;

        status = Send_Command(card, cmd9, cmd9_arg, &R1_response);
        if (status != HAL_OK) goto cleanup;
        if (R1_response != 0) {
            status = HAL_ERROR;
            goto cleanup;
        }

        // wait till 0xFE byte to signal the start of data transfers.
        status = Wait_To_Receive(card);
        if (status != HAL_OK) goto cleanup;


        status = SD_SPI_Receive_Data(card->spi_handle, CSD_reg, sizeof(CSD_reg));
        if (status != HAL_OK) goto cleanup;


        // check the crc16
        uint16_t pattern = 0x1021; 
        
        status = Calc_CRC16(pattern, CSD_reg, sizeof(CSD_reg), &response, false);
        if (status != HAL_OK) goto cleanup;

        attempt_count++;
    } while (response != 0 && attempt_count < 3);
    if (response != 0) {
        status = HAL_ERROR;
        goto cleanup;
    }

    // verify that it is a High Capacity or Extended Capacity Card. 
    if ((CSD_reg[0] & 0xc0) != 0x40) {
        status = HAL_ERROR;
        goto cleanup;
    }

    // Find the c_size from the CSD register.
    uint64_t c_size = (((uint64_t)CSD_reg[7] & 0x3f) << 16) | ((uint64_t)CSD_reg[8] << 8) | (uint64_t)CSD_reg[9];

    *sector_count_adr = (c_size + 1) * 1024;

    status = HAL_OK;

cleanup:
    (void)SD_SPI_Deselect_Card(card->spi_handle);

    // transmit 0xff byte to provide trailing clocks after CS deselects the chip to reset SPI.
    uint8_t filler_response;
    (void)SD_SPI_Interchange_Byte(card->spi_handle, 0xff, &filler_response);

    return status;
}


/** This init function first resets the card into SPI mode, than validates the
    card, waits for the card to be ready, 
 */
HAL_StatusTypeDef Card_Init(Card_Handle *card,) {

}