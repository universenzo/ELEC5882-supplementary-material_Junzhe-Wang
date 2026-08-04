#ifndef STM32_CAN_IV_PORT_H
#define STM32_CAN_IV_PORT_H

#include "can_iv_protocol.h"
#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

bool stm32_can_iv_port_start(void);
bool stm32_can_iv_port_transmit(void *context, const can_iv_frame_t *frame);
bool stm32_can_iv_port_receive(void *context, can_iv_frame_t *frame);

uint32_t stm32_can_iv_port_rx_count(void);
uint32_t stm32_can_iv_port_tx_count(void);
uint32_t stm32_can_iv_port_rx_drop_count(void);
uint32_t stm32_can_iv_port_tx_drop_count(void);

/*
 * CubeMX/HAL I2C1 commissioning pinout:
 * PA15 = I2C1_SCL, PB9 = I2C1_SDA. Use direct jumper wires to these pins.
 * MX_I2C1_Init() must run before stm32_ina226_i2c_init().
 */
bool stm32_ina226_i2c_init(void);
bool stm32_ina226_i2c_lines_high(void);
bool stm32_ina226_read_register(void *context,
                                uint8_t register_address,
                                uint16_t *value);
bool stm32_ina226_write_register(void *context,
                                 uint8_t register_address,
                                 uint16_t value);

#endif
