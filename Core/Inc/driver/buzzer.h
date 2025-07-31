/*
 * buzzer.h
 *
 *  Created on: Jul 29, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_BUZZER_H_
#define INC_DRIVER_BUZZER_H_


#include "stm32wbxx_hal.h"


void buzzer_init(void);
void buzzer_beep(uint16_t freq, uint16_t duration_ms);


#endif /* INC_DRIVER_BUZZER_H_ */
