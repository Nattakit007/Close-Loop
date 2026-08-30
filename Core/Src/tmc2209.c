/*
 * tmc2209.c
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */
#include "tmc2209.h"

/* Standard CRC-8 Calculation for TMC2209 Data frames */
static uint8_t TMC2209_CalcCRC(uint8_t *datagram, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t current_byte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) crc = (crc << 1) ^ 0x07;
            else crc = (crc << 1);
            current_byte >>= 1;
        }
    }
    return crc;
}

void TMC2209_Init(TMC2209_t *driver, UART_HandleTypeDef *huart, uint8_t node_addr) {
    driver->huart = huart;
    driver->node_address = node_addr;

    /* 1. Enable UART software control */
    TMC2209_WriteRegister(driver, TMC2209_GCONF, 0x00000040);

    /* 2. Configure CHOPCONF for robust 24V supply silent operation[cite: 3] */
    /* Baseline 0x10000043 applies TOFF=3, TBL=2, HSTRT=4, HEND=0[cite: 5] */
    TMC2209_WriteRegister(driver, TMC2209_CHOPCONF, 0x10000043);

    /* 3. Set standard NEMA 17 current scaling (IRUN=16, IHOLD=8) */
    TMC2209_SetCurrent(driver, 16, 8);
}

void TMC2209_WriteRegister(TMC2209_t *driver, uint8_t reg, uint32_t value) {
    uint8_t frame[8];
    frame[0] = 0x05;                        // Sync byte
    frame[1] = driver->node_address;        // Node ID (0-3)
    frame[2] = reg | 0x80;                  // Register Address | Write Flag
    frame[3] = (uint8_t)(value >> 24);
    frame[4] = (uint8_t)(value >> 16);
    frame[5] = (uint8_t)(value >> 8);
    frame[6] = (uint8_t)(value);
    frame[7] = TMC2209_CalcCRC(frame, 7);

    /* Single-Wire Half-Duplex hardware automatically handles TX/RX pin toggling */
    HAL_UART_Transmit(driver->huart, frame, 8, 10);
}

void TMC2209_SetCurrent(TMC2209_t *driver, uint8_t run_current, uint8_t hold_current) {
    /* Maps IHOLD to bits 0-4, IRUN to bits 8-12, IHOLDDELAY to bits 16-19 */
    uint32_t val = ((hold_current & 0x1F) << 0) | ((run_current & 0x1F) << 8) | (4 << 16);
    TMC2209_WriteRegister(driver, TMC2209_IHOLD_IRUN, val);
}

void TMC2209_SetVelocity(TMC2209_t *driver, int32_t velocity) {
    /* Actuate motor directly via UART bypassing STEP/DIR */
    TMC2209_WriteRegister(driver, TMC2209_VACTUAL, (uint32_t)velocity);
}

