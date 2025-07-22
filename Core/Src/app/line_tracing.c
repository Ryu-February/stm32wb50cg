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

// PID 계수
float Kp = 0.8;
float Ki = 0.0;
float Kd = 0.4;

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

    float left_brightness = calculate_brightness(left_color.red, left_color.green, left_color.blue);
    float right_brightness = calculate_brightness(right_color.red, right_color.green, right_color.blue);

    float error = right_brightness - left_brightness;
    integral += error;
    float derivative = error - prev_error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    float base_speed = 1000;
    float left_speed = base_speed - output * 500;  // 보정 강도 조정
    float right_speed = base_speed + output * 500;

    step_drive_ratio(left_speed, right_speed);  // 비율 기반 회전 제어
}

