#ifndef SD_SPI_H
#define SD_SPI_H

#include "stm32f4xx_hal.h"


typedef struct {
    SPI_HandleTypeDef *hspi;
    GIPO_TypeDef cs_port;
    uint16_t cs_pin;
} SD_SPI_Handle;



#endif