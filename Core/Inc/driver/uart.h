/*
 * uart.h
 *
 *  Created on: Jul 8, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_UART_H_
#define INC_DRIVER_UART_H_

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32wbxx_hal.h"


void uart_printf(const char *fmt, ...);



#endif /* INC_DRIVER_UART_H_ */
