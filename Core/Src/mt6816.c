#include "mt6816.h"

#define MT6816_CMD_REG03    0x8300
#define MT6816_CMD_REG04    0x8400
#define MT6816_NO_MAG_BIT   (1u << 1)
#define MT6816_RESOLUTION   16384.0f    /* 2^14 */

/* ---------- private helpers ---------- */

static inline void cs_low (MT6816_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(MT6816_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);   }

static uint16_t xfer16(MT6816_t *d, uint16_t cmd)
{
    uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    uint8_t rx[2] = { 0, 0 };

    cs_low(d);
    HAL_SPI_TransmitReceive(d->hspi, tx, rx, 2, 10);
    cs_high(d);

    return (uint16_t)((rx[0] << 8) | rx[1]);
}

static bool parity_ok(uint16_t data)
{
    uint8_t ones = 0;
    for (uint8_t i = 1; i < 16; i++)
        if (data & (1u << i)) ones++;
    return ((ones & 1u) == (data & 1u));
}

/* ---------- public API ---------- */

void MT6816_Init(MT6816_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    cs_high(dev);                 /* CSN idles HIGH */
}

MT6816_Status MT6816_ReadRaw(MT6816_t *dev, uint16_t *raw)
{
    uint16_t r03 = xfer16(dev, MT6816_CMD_REG03) & 0xFF;
    uint16_t r04 = xfer16(dev, MT6816_CMD_REG04) & 0xFF;
    uint16_t combined = (uint16_t)((r03 << 8) | r04);

    if (combined & MT6816_NO_MAG_BIT) return MT6816_ERR_NO_MAGNET;
    if (!parity_ok(combined))         return MT6816_ERR_PARITY;

    *raw = (uint16_t)((r03 << 6) | ((r04 >> 2) & 0x3F));
    return MT6816_OK;
}

MT6816_Status MT6816_ReadDegrees(MT6816_t *dev, float *degrees)
{
    uint16_t raw;
    MT6816_Status st = MT6816_ReadRaw(dev, &raw);
    if (st == MT6816_OK) *degrees = (raw / MT6816_RESOLUTION) * 360.0f;
    return st;
}

MT6816_Status MT6816_ReadRadians(MT6816_t *dev, float *radians)
{
    uint16_t raw;
    MT6816_Status st = MT6816_ReadRaw(dev, &raw);
    if (st == MT6816_OK) *radians = (raw / MT6816_RESOLUTION) * 6.28318530718f;
    return st;
}

float MT6816_GetAngle(MT6816_t *dev)
{
    float deg;
    return (MT6816_ReadDegrees(dev, &deg) == MT6816_OK) ? deg : -1.0f;
}
