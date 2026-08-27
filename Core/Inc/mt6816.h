#ifndef MT6816_H
#define MT6816_H

#include "main.h"
#include <stdbool.h>

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} MT6816_t;

typedef enum {
    MT6816_OK = 0,
    MT6816_ERR_NO_MAGNET,
    MT6816_ERR_PARITY
} MT6816_Status;

/* Call once at startup */
void          MT6816_Init(MT6816_t *dev, SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Main API */
MT6816_Status MT6816_ReadRaw    (MT6816_t *dev, uint16_t *raw);   // 0..16383
MT6816_Status MT6816_ReadDegrees(MT6816_t *dev, float *degrees);  // 0..360
MT6816_Status MT6816_ReadRadians(MT6816_t *dev, float *radians);  // 0..2pi

/* Convenience: returns angle or -1.0f on any error */
float         MT6816_GetAngle(MT6816_t *dev);

#endif
