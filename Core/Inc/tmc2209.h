/*
 * tmc2209.h
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */

#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* TMC2209 Register Map */
#define TMC2209_GCONF        0x00
#define TMC2209_IHOLD_IRUN   0x10
#define TMC2209_TPOWERDOWN   0x11
#define TMC2209_VACTUAL      0x22
#define TMC2209_CHOPCONF     0x6C

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t node_address;
} TMC2209_t;

void TMC2209_Init(TMC2209_t *driver, UART_HandleTypeDef *huart, uint8_t node_addr);
void TMC2209_WriteRegister(TMC2209_t *driver, uint8_t reg, uint32_t value);
void TMC2209_SetCurrent(TMC2209_t *driver, uint8_t run_current_scale, uint8_t hold_current_scale);
void TMC2209_SetVelocity(TMC2209_t *driver, int32_t velocity);
void TMC2209_MaxTorque(TMC2209_t *driver);

#endif
