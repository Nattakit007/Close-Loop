/*
 * tmc2209.h
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */

#ifndef INC_TMC2209_H_
#define INC_TMC2209_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Core TMC2209 Registers */
#define TMC2209_GCONF        0x00
#define TMC2209_IHOLD_IRUN   0x10
#define TMC2209_VACTUAL      0x22
#define TMC2209_CHOPCONF     0x6C

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t node_address;
} TMC2209_t;

void TMC2209_Init(TMC2209_t *driver, UART_HandleTypeDef *huart, uint8_t node_addr);
void TMC2209_WriteRegister(TMC2209_t *driver, uint8_t reg, uint32_t value);
void TMC2209_SetCurrent(TMC2209_t *driver, uint8_t run_current, uint8_t hold_current);
void TMC2209_SetVelocity(TMC2209_t *driver, int32_t velocity);



#endif /* INC_TMC2209_H_ */
