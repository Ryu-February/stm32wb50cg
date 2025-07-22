/*
 * ir.h
 *
 *  Created on: Jul 22, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_IR_H_
#define INC_DRIVER_IR_H_

#include "stm32wbxx_hal.h"

#define IR_THRESHOLD 	30

uint16_t ir_read_adc(void);
uint8_t ir_is_black(void);

#endif /* INC_DRIVER_IR_H_ */
