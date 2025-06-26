/*
 * rgb.c
 *
 *  Created on: Jun 26, 2025
 *      Author: RCY
 */


#include "rgb.h"


extern TIM_HandleTypeDef htim1;


#define PWM_CH_RED		TIM_CHANNEL_1
#define PWM_CH_GREEN	TIM_CHANNEL_2
#define PWM_CH_BLUE		TIM_CHANNEL_3




const rgb_led_t led_map[COLOR_COUNT] =
{
		[COLOR_BLACK]       = {   0,   0,   0 },
		[COLOR_RED]         = { 255,   0,   0 },
		[COLOR_ORANGE]      = { 255, 165,   0 },
		[COLOR_YELLOW]      = { 255, 255,   0 },
		[COLOR_GREEN]       = {   0, 255,   0 },
		[COLOR_LIGHT_GREEN] = {  26, 255,  26 },
		[COLOR_CYAN]        = {   0, 255, 255 },
		[COLOR_LIGHT_BLUE]  = {   0, 180, 180 },
		[COLOR_BLUE]        = {   0,   0, 255 },
		[COLOR_INDIGO]      = {  75,   0, 130 },
		[COLOR_VIOLET]      = { 238, 130, 238 },
		[COLOR_PINK]        = { 255, 105, 180 },
		[COLOR_WHITE]       = { 255, 255, 255 }
};


void rgb_init(void)
{
//	HAL_TIM_PWM_Start(&htim1, PWM_CH_RED);
//	HAL_TIM_PWM_Start(&htim1, PWM_CH_GREEN);
//	HAL_TIM_PWM_Start(&htim1, PWM_CH_BLUE);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

//	rgb_set_color(COLOR_WHITE);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);//white led init
}


void rgb_set_pwm(uint8_t r, uint8_t g, uint8_t b)
{
//	__HAL_TIM_SET_COMPARE(&htim1, PWM_CH_RED, 	255 - r);
//	__HAL_TIM_SET_COMPARE(&htim1, PWM_CH_GREEN, 255 - g);
//	__HAL_TIM_SET_COMPARE(&htim1, PWM_CH_BLUE, 	255 - b);
	static uint8_t pwm_period = 0;

	if(++pwm_period >= 255)
	{
		pwm_period = 0;
	}

	if(pwm_period > 255 - r)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}

	if(pwm_period > 255 - g)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	}

	if(pwm_period > 255 - b)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}

}

void rgb_set_color(color_t color)
{
	if(color >= COLOR_COUNT)
		return;

	rgb_set_pwm(led_map[color].r, led_map[color].g, led_map[color].b);
}
