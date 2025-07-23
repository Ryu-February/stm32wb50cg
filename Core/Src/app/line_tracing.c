/*
 * line_tracing.c
 *
 *  Created on: Jul 22, 2025
 *      Author: RCY
 */


#include "line_tracing.h"
#include "ir.h"
#include "step.h"
#include "color.h"
#include "uart.h"

extern uint8_t  offset_side;
extern uint16_t offset_average;
extern volatile uint32_t timer17_ms;

// PID 계수
float Kp = 30;
float Ki = 0.0;
float Kd = 10;

float prev_error = 0;
float integral = 0;

void line_tracing_fsm(void)
{
	bh1745_color_data_t left_color, right_color;

	left_color  = bh1745_read_rgbc(BH1745_ADDR_LEFT);
	right_color = bh1745_read_rgbc(BH1745_ADDR_RIGHT);


	color_t detected_L, detected_R;

	detected_L =
			classify_color(BH1745_ADDR_LEFT, left_color.red, left_color.green, left_color.blue, left_color.clear);
	detected_R =
			classify_color(BH1745_ADDR_RIGHT, right_color.red, right_color.green, right_color.blue, right_color.clear);

	LineState state;

	if (detected_L != COLOR_BLACK && detected_R != COLOR_BLACK)
	    state = ON_LINE;
	else if (detected_L == COLOR_BLACK && detected_R != COLOR_BLACK)
	    state = LEFT_OFF;
	else if (detected_L != COLOR_BLACK && detected_R == COLOR_BLACK)
	    state = RIGHT_OFF;
	else
	    state = LOST;

	switch (state)
	{
	    case ON_LINE:
	        step_drive(FORWARD);
	        break;
	    case LEFT_OFF:
	    	step_drive(TURN_LEFT);
	        break;
	    case RIGHT_OFF:
	    	step_drive(TURN_RIGHT);
	        break;
	    case LOST:
	    	step_drive(STOP);
	        break;
	}
}


void line_tracing_pid(void)
{
    bh1745_color_data_t left_color = bh1745_read_rgbc(BH1745_ADDR_LEFT);
    bh1745_color_data_t right_color = bh1745_read_rgbc(BH1745_ADDR_RIGHT);

    uint32_t left_brightness  = calculate_brightness(left_color.red, left_color.green, left_color.blue);
    uint32_t right_brightness = calculate_brightness(right_color.red, right_color.green, right_color.blue);

	if(offset_side == LEFT)
	{
		left_brightness -= offset_average;
	}
	else
	{
		right_brightness -= offset_average;
	}

    float error = (float)right_brightness - left_brightness;
//    integral += error * dt;
    float derivative = error - prev_error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    float base_speed = 3000;
    float left_speed = base_speed + output;  // 보정 강도 조정
    float right_speed = base_speed - output;

//    if(left_speed > 1800)	left_speed = 1800;
//    if(right_speed > 1800)	right_speed = 1800;
//    if(left_speed < 600)	left_speed = 600;
//    if(right_speed < 600)	right_speed = 600;

	step_drive_ratio(left_speed, right_speed);  // 비율 기반 회전 제어

//    uart_printf("left_bightness: %d, right_brightness: %d\r\n", left_brightness, right_brightness);
//    uart_printf("error: %.2f | prev_error: %.2f\r\n", error, prev_error);
//    uart_printf("output: %.2f\r\n", output);
//    uart_printf("left_speed: %.2f, right_speed: %.2f\r\n", left_speed, right_speed);

//    step_drive(FORWARD);
}

