/*
 * tmc2209.c
 *
 *  Created on: Aug 30, 2026
 *      Author: kitanat
 */
#include "tmc2209.h"

static uint8_t TMC2209_CalcCRC(uint8_t *datagram, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t current_byte = datagram[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            current_byte >>= 1;
        }
    }
    return crc;
}

void TMC2209_Init(TMC2209_t *driver, UART_HandleTypeDef *huart, uint8_t node_addr) {
    driver->huart = huart;
    driver->node_address = node_addr;

    /* 1. GCONF: 0x000000C0
     * Bit 0 (I_scale_analog)  = 0 -> Ignore VREF pin, use internal reference for current
     * Bit 6 (pdn_disable)     = 1 -> Disable PDN pin pull-down, enable UART control[cite: 2, 3]
     * Bit 7 (mstep_reg_select)= 1 -> Set microstepping via CHOPCONF.MRES instead of MS1/MS2 pins
     */
    TMC2209_WriteRegister(driver, TMC2209_GCONF, 0x000000C0);

    /* 2. CHOPCONF: 0x14000043
     * TOFF=3, TBL=2, HSTRT=4, HEND=0, MRES=4 (1/16 step with MicroPlyer interpolation)[cite: 2, 4, 5]
     */
    TMC2209_WriteRegister(driver, TMC2209_CHOPCONF, 0x14000043);

    /* 3. Set current scaling via UART: IRUN=16 (approx. 53%), IHOLD=8 (approx. 28%) */
    TMC2209_SetCurrent(driver, 31, 8);

    /* 4. Standstill power-down delay (~20 * 2^18 clocks) */
    TMC2209_WriteRegister(driver, TMC2209_TPOWERDOWN, 0x00000014);
}

void TMC2209_WriteRegister(TMC2209_t *driver, uint8_t reg, uint32_t value) {
    uint8_t frame[8];

    frame[0] = 0x05;                        // Sync byte
    frame[1] = driver->node_address;        // Node ID (0-3)
    frame[2] = reg | 0x80;                  // Register address + Write Flag
    frame[3] = (uint8_t)(value >> 24);
    frame[4] = (uint8_t)(value >> 16);
    frame[5] = (uint8_t)(value >> 8);
    frame[6] = (uint8_t)(value);
    frame[7] = TMC2209_CalcCRC(frame, 7);

    HAL_UART_Transmit(driver->huart, frame, 8, 10);
}

void TMC2209_SetCurrent(TMC2209_t *driver, uint8_t run_current_scale, uint8_t hold_current_scale) {
    /* IHOLD: bits 0..4 (0-31), IRUN: bits 8..12 (0-31), IHOLDDELAY: bits 16..19[cite: 2, 3] */
    uint32_t val = ((hold_current_scale & 0x1F) << 0)  |
                   ((run_current_scale  & 0x1F) << 8)  |
                   (4 << 16); // IHOLDDELAY = 4
    TMC2209_WriteRegister(driver, TMC2209_IHOLD_IRUN, val);
}

void TMC2209_SetVelocity(TMC2209_t *driver, int32_t velocity) {
    TMC2209_WriteRegister(driver, TMC2209_VACTUAL, (uint32_t)velocity);
}

void TMC2209_MaxTorque(TMC2209_t *driver)
{
    /* UART control + register microsteps + forced SpreadCycle */
    TMC2209_WriteRegister(driver, TMC2209_GCONF, 0x000000C4);

    /*
     * Existing CHOPCONF:
     * 1/16 microstepping, interpolation enabled, VSENSE = 0.
     */
    TMC2209_WriteRegister(driver, TMC2209_CHOPCONF, 0x14000043);

    /* Maximum current scale */
    TMC2209_SetCurrent(driver, 31, 31);
}

