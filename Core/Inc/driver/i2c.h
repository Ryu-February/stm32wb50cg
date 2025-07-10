/*
 * i2c.h
 *
 *  Created on: Jul 9, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_I2C_H_
#define INC_DRIVER_I2C_H_

#include "stm32wbxx_hal.h"


void i2c_init(void);
void i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);
uint8_t i2c_read(uint8_t slave_addr, uint8_t reg_addr);

#endif /* INC_DRIVER_I2C_H_ */
