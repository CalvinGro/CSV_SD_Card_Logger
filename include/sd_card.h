#ifndef SD_CARD_H
#define SD_CARD_H

#include "stm32f4xx_hal.h"


typedef struct {
    SD_SPI_Handle *spi_handle;
    uint32_t card_sector_count; 
} Card_Handle;



#endif