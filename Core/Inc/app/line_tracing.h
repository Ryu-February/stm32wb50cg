/*
 * line_tracing.h
 *
 *  Created on: Jul 22, 2025
 *      Author: RCY
 */

#ifndef INC_APP_LINE_TRACING_H_
#define INC_APP_LINE_TRACING_H_


#include "stm32wbxx_hal.h"
#include "color.h"


typedef enum
{
	ON_LINE,
	LEFT_OFF,
	RIGHT_OFF,
	LOST
}LineState;

void line_tracing_fsm(void);
void line_tracing_pid(void);
bh1745_color_data_t line_tracing_read_rgb(uint8_t color_addr);

#endif /* INC_APP_LINE_TRACING_H_ */
