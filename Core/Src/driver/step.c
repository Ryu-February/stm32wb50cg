/*
 * step.c
 *
 *  Created on: Jul 11, 2025
 *      Author: RCY
 */

#include "uart.h"
#include "step.h"

extern volatile uint32_t timer17_ms;
extern TIM_HandleTypeDef htim2;  // 10kHz interrupt for microstep PWM

/*                            Motor(15BY25-119)                         */
/*                           Motor driver(A3916)                        */

/************************************************************************/
/*                       MCU  |     NET    | DRIVER                     */
/*                       PA0 -> MOT_L_IN1 ->  IN1                       */
/*                       PA1 -> MOT_L_IN2 ->  IN2                       */
/*                       PA2 -> MOT_L_IN3 ->  IN3                       */
/*                       PA3 -> MOT_L_IN4 ->  IN4                       */
/*                                                                      */
/*                       PB4 -> MOT_R_IN1 ->  IN1                       */
/*                       PB5 -> MOT_R_IN2 ->  IN2                       */
/*                       PB6 -> MOT_R_IN3 ->  IN3                       */
/*                       PB7 -> MOT_R_IN4 ->  IN4                       */
/************************************************************************/

/*
 * A3916 Stepper Motor Operation Table (Half Step + Full Step)
 *
 * IN1 IN2 IN3 IN4 | OUT1A OUT1B OUT2A OUT2B | Function
 * --------------------------------------------------------
 *  0   0   0   0  |  Off   Off   Off   Off   | Disabled
 *  1   0   1   0  |  High  Low   High  Low   | Full Step 1 / 	½ Step 1
 *  0   0   1   0  |  Off   Off   High  Low   |             	½ Step 2
 *  0   1   1   0  |  Low   High  High  Low   | Full Step 2 /	½ Step 3
 *  0   1   0   0  |  Low   High  Off   Off   |             	½ Step 4
 *  0   1   0   1  |  Low   High  Low   High  | Full Step 3 / 	½ Step 5
 *  0   0   0   1  |  Off   Off   Low   High  |             	½ Step 6
 *  1   0   0   1  |  High  Low   Low   High  | Full Step 4 / 	½ Step 7
 *  1   0   0   0  |  High  Low   Off   Off   |             	½ Step 8
 */

//
//StepMotor motor_l = {
//  .in1_port = GPIOA, .in1_pin = GPIO_PIN_0,
//  .in2_port = GPIOA, .in2_pin = GPIO_PIN_1,
//  .in3_port = GPIOA, .in3_pin = GPIO_PIN_2,
//  .in4_port = GPIOA, .in4_pin = GPIO_PIN_3,
//};
//
//StepMotor motor_r = {
//  .in1_port = GPIOB, .in1_pin = GPIO_PIN_4,
//  .in2_port = GPIOB, .in2_pin = GPIO_PIN_5,
//  .in3_port = GPIOB, .in3_pin = GPIO_PIN_6,
//  .in4_port = GPIOB, .in4_pin = GPIO_PIN_7,
//};

DEFINE_STEP_MOTOR(step_motor_left,
	GPIOA, GPIO_PIN_0,   // IN1: PA0
	GPIOA, GPIO_PIN_1,   // IN2: PA1
	GPIOA, GPIO_PIN_2,   // IN3: PA2
	GPIOA, GPIO_PIN_3    // IN4: PA3
);

DEFINE_STEP_MOTOR(step_motor_right,
	GPIOB, GPIO_PIN_4,   // IN1: PB4
	GPIOB, GPIO_PIN_5,   // IN2: PB5
	GPIOB, GPIO_PIN_6,   // IN3: PB6
	GPIOB, GPIO_PIN_7    // IN4: PB7
);

#if (_USE_STEP_MODE == _STEP_MODE_HALF)
static const uint8_t step_table[8][4] = {
	{1,0,1,0},
	{0,0,1,0},
	{0,1,1,0},
	{0,1,0,0},
	{0,1,0,1},
	{0,0,0,1},
	{1,0,0,1},
	{1,0,0,0}
};
#elif(_USE_STEP_MODE == _STEP_MODE_FULL)

static const uint8_t step_table[4][4] = {
	{1,0,1,0},  // A+ & B+
	{0,1,1,0},  // A- & B+
	{0,1,0,1},  // A- & B-
	{1,0,0,1},  // A+ & B-
};

#elif(_USE_STEP_MODE == _STEP_MODE_MICRO)

//sin table
//360도를 32스텝으로 쪼갰을 때 11.25가 나오는데 11.25도의 간격을 pwm으로 표현하면 이렇게 나옴
const uint8_t step_table[32] = {			//sin(degree) -> pwm
  128, 152, 176, 198, 218, 234, 245, 253,
  255, 253, 245, 234, 218, 198, 176, 152,
  128, 103,  79,  57,  37,  21,  10,   2,
    0,   2,  10,  21,  37,  57,  79, 103
};
#endif




static void apply_step(StepMotor *m)
{
#if (_USE_STEP_MODE == _STEP_MODE_MICRO)
  m->vA = step_table[m->step_idx];
  m->vB = step_table[(m->step_idx + (STEP_MASK >> 2)) & STEP_MASK]; //(STEP_MASK >> 2) == 8 == 90°(difference sin with cos)
  //sin파와 cos파의 위상 차가 90도가 나니까 +8을 한 거임 +8은 32를 360도로 치환했을 때 90도를 의미함

  HAL_GPIO_WritePin(m->in1_port, m->in1_pin, (m->vA > 127));
  HAL_GPIO_WritePin(m->in2_port, m->in2_pin, !(m->vA > 127));
  HAL_GPIO_WritePin(m->in3_port, m->in3_pin, (m->vB > 127));
  HAL_GPIO_WritePin(m->in4_port, m->in4_pin, !(m->vB > 127));
#else
  HAL_GPIO_WritePin(m->in1_port, m->in1_pin, step_table[m->step_idx][0]);
  HAL_GPIO_WritePin(m->in2_port, m->in2_pin, step_table[m->step_idx][1]);
  HAL_GPIO_WritePin(m->in3_port, m->in3_pin, step_table[m->step_idx][2]);
  HAL_GPIO_WritePin(m->in4_port, m->in4_pin, step_table[m->step_idx][3]);
#endif
}

void step_init(StepMotor *m)
{
  m->step_idx = 0;
  m->prev_time_us = 0;
  m->dir = (m == &step_motor_left) ? LEFT : RIGHT;

  m->forward = (m == &step_motor_left) ? step_forward : step_reverse;
  m->reverse = (m == &step_motor_left) ? step_reverse : step_forward;
  m->brake   = step_brake;
  m->slide   = step_slide;
}

void step_forward(StepMotor *m)
{
  apply_step(m);
  m->step_idx = (m->step_idx + 1) & STEP_MASK;
}

void step_reverse(StepMotor *m)
{
  apply_step(m);
  m->step_idx = (m->step_idx - 1) & STEP_MASK;
}

void step_brake(StepMotor *m)
{
  HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(m->in3_port, m->in3_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(m->in4_port, m->in4_pin, GPIO_PIN_SET);
}

void step_slide(StepMotor *m)
{
  HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m->in3_port, m->in3_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m->in4_port, m->in4_pin, GPIO_PIN_RESET);
}

uint32_t rpm_to_period(uint16_t rpm)
{
  if (rpm == 0) rpm = 1;
  if (rpm > SAFE_MAX_RPM) rpm = SAFE_MAX_RPM;
  return 60000000UL / (rpm * STEP_PER_REV);
}

uint32_t pwm_to_rpm(uint8_t pwm)
{
  if (pwm > MAX_SPEED) pwm = MAX_SPEED;
  return (uint32_t)pwm * SAFE_MAX_RPM / MAX_SPEED;
}

void step_operate(StepMotor *m, uint8_t speed, uint8_t dir)
{
  if (speed == 0)
  {
    m->brake(m);
    return;
  }

  m->period_us = rpm_to_period(pwm_to_rpm(speed));
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);  // must be 1us timer

  if ((now - m->prev_time_us) >= m->period_us)
  {
    m->prev_time_us = now;
    (dir == FORWARD ? m->forward : m->reverse)(m);
  }
}

void step_init_all(void)
{
	step_motor_left.init(&step_motor_left);
	step_motor_right.init(&step_motor_right);
}

void step_idx_init(void)
{
	step_motor_left.step_idx = 0;
	step_motor_right.step_idx = 0;
}

void step_test(StepOperation op)
{
	if(op == FORWARD)
	{
		step_motor_left.forward(&step_motor_left);
		step_motor_right.forward(&step_motor_right);
	}
	else if(op == REVERSE)
	{
		step_motor_left.reverse(&step_motor_left);
		step_motor_right.reverse(&step_motor_right);
	}
	else if(op == TURN_LEFT)
	{
		step_motor_left.reverse(&step_motor_left);
		step_motor_right.forward(&step_motor_right);
	}
	else if(op == TURN_RIGHT)
	{
		step_motor_left.forward(&step_motor_left);
		step_motor_right.reverse(&step_motor_right);
	}


}

void step_stop(void)
{
	step_motor_left.brake(&step_motor_left);
	step_motor_right.brake(&step_motor_right);
}


void step_run(StepOperation op)
{
	/*
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t prev_time = 0;
    uint32_t period_us = 3000;

	prev_time = (step_motor_left.dir == LEFT) ? step_motor_left.prev_time_us : step_motor_right.prev_time_us;

//	period_us = (step_motor_left.dir == LEFT) ? step_motor_left.period_us : step_motor_right.period_us;
//	period_us = 3000;

    if ((now - prev_time) >= period_us)
    {
    	step_motor_left.prev_time_us = now;

        if(op == FORWARD)
		{
			step_motor_left.forward(&step_motor_left);
			step_motor_right.forward(&step_motor_right);
		}
		else if(op == REVERSE)
		{
			step_motor_left.reverse(&step_motor_left);
			step_motor_right.reverse(&step_motor_right);
		}
		else if(op == TURN_LEFT)
		{
			step_motor_left.reverse(&step_motor_left);
			step_motor_right.forward(&step_motor_right);
		}
		else if(op == TURN_RIGHT)
		{
			step_motor_left.forward(&step_motor_left);
			step_motor_right.reverse(&step_motor_right);
		}
    }*/

    uint64_t now = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t period_us = 3000;

	if ((now - step_motor_left.prev_time_us) >= period_us)
	{
	   step_motor_left.prev_time_us = now;

	   if(op == FORWARD || op == TURN_RIGHT)
		   step_motor_left.forward(&step_motor_left);
	   else
		   step_motor_left.reverse(&step_motor_left);
	}
	else
	{
		step_motor_left.brake(&step_motor_left);
	}

	if ((now - step_motor_right.prev_time_us) >= period_us)
	{
	   step_motor_right.prev_time_us = now;

	   if(op == FORWARD || op == TURN_LEFT)
		   step_motor_right.forward(&step_motor_right);
	   else
		   step_motor_right.reverse(&step_motor_right);
	}
	else
	{
		step_motor_right.brake(&step_motor_right);
	}
}

