/*
 * buzzer.c
 *
 *  Created on: Jul 29, 2025
 *      Author: RCY
 */

#include "buzzer.h"
#include <stdbool.h>
#include "uart.h"

extern TIM_HandleTypeDef htim1;
volatile bool buzzer_enabled = false;
volatile bool buzzer_start = false;
#define TIM1_IRQ_PERIOD		1000000



void buzzer_init(void)
{

}

void buzzer_beep(uint16_t freq, uint16_t duration_ms)
{
    if (freq == 0) return;

    uint32_t toggle_us = 1000000 / (freq * 2);
    if (toggle_us == 0) toggle_us = 1;

    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim1, toggle_us - 1);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim1);

    buzzer_enabled = true;
    HAL_TIM_Base_Start_IT(&htim1);

    HAL_Delay(duration_ms);

    buzzer_enabled = false;
    HAL_TIM_Base_Stop_IT(&htim1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void buzzer_op(buzzer_t op)
{
	if(op == BUZZER_ON)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	}
	else if(op == BUZZER_OFF)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(op == BUZZER_TOGGLE)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	}
}

void pitches_to_period(uint16_t tone)
{
	float temp		= 1.0f / tone;		//tone == frequency
	uint16_t period = (float)(temp * TIM1_IRQ_PERIOD) - 1;
	TIM1->ARR = period;
	TIM1->CNT = 0;
	TIM1->EGR |= TIM_EGR_UG;  // ARR 갱신

	buzzer_start = true;
}
