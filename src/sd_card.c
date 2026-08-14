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



HAL_StatusTypeDef Wait_Ready(Card_Handle *card) {

    HAL_StatusTypeDef status;
    uint8_t received_byte;
    uint32_t start_time = HAL_GetTick();
    do {
        status = SD_SPI_Interchange_Byte(card->spi_handle, 0xff, &received_byte);
        if (status != HAL_OK) return status;
    } while ((HAL_GetTick() - start_time < 100) && received_byte == 0x00);

    if (received_byte == 0xff) {
        return HAL_OK;
    }
    return HAL_TIMEOUT;
}


/** This function sends the specified command to the card using the sd card protocol.
    The command is six bytes, the first is the cmd type, the next four are the command
    argument, then the final byte is the crc, which represents the end of the command
    and is calculated based off the command and its argument.

    The card should already be selected before sending commands
 */
HAL_StatusTypeDef Send_Command(Card_Handle *card, uint8_t cmd, uint32_t cmd_arg, uint8_t *R1_adr) {
    
}

/** This init function first resets the card into SPI mode, than validates the
    card, waits for the card to be ready, 
 */
HAL_StatusTypeDef Card_Init(Card_Handle *card,) {

}