#ifndef SD_SPI_H
#define SD_SPI_H

#include "stm32f4xx_hal.h"


typedef struct {
    SPI_HandleTypeDef *hspi;
    GIPO_TypeDef cs_port;
    uint16_t cs_pin;
} SD_SPI_Handle;


HAL_StatusTypeDef SD_SPI_Init(SD_SPI_Handle *new_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin, SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef SD_SPI_SetOperatingSpeed(SD_SPI_Handle *cur_handle);
HAL_StatusTypeDef SD_SPI_SetInitializationSpeed(SD_SPI_Handle *cur_handle);

HAL_StatusTypeDef SD_SPI_Select_Card(SD_SPI_Handle *cur_handle);
HAL_StatusTypeDef SD_SPI_Deselect_Card(SD_SPI_Handle *cur_handle);

HAL_StatusTypeDef SD_SPI_Interchange_Byte(SD_SPI_Handle *cur_handle, uint8_t transmitted_byte, uint8_t *received_byte_adr);
HAL_StatusTypeDef SD_SPI_Transmit_Data(SD_SPI_Handle *cur_handle, uint8_t *transmit_adr, uint16_t byte_count);
HAL_StatusTypeDef SD_SPI_Receive_Data(SD_SPI_Handle *cur_handle, uint8_t *receive_adr, uint16_t byte_count);

HAL_StatusTypeDef SD_SPI_Send_Idle_Clocks(SD_SPI_Handle *cur_handle);

#endif