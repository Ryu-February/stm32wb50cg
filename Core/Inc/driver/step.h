/*
 * step.h
 *
 *  Created on: Jul 11, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_STEP_H_
#define INC_DRIVER_STEP_H_


#include "stm32wbxx_hal.h"

#define LEFT        0
#define RIGHT       1
//#define FORWARD     0
//#define REVERSE     1
//#define TURN_LEFT   2
//#define TURN_RIGHT  3

#define SAFE_MAX_RPM    1200
#define MAX_SPEED       100
#define MIN_SPEED       0

#define MICRO_MAX_PWM   255

#define _STEP_MODE_FULL  0
#define _STEP_MODE_HALF  1
#define _STEP_MODE_MICRO 2

#define _USE_STEP_MODE   _STEP_MODE_MICRO   // 변경 가능: FULL, HALF, MICRO

#if (_USE_STEP_MODE == _STEP_MODE_HALF)
  #define STEP_MASK     0x07
  #define STEP_PER_REV  40
#elif (_USE_STEP_MODE == _STEP_MODE_FULL)
  #define STEP_MASK     0x03
  #define STEP_PER_REV  20
#elif (_USE_STEP_MODE == _STEP_MODE_MICRO)
  #define STEP_MASK     0x1F
  #define STEP_PER_REV  80
#endif

typedef struct StepMotor
{
  GPIO_TypeDef* in1_port; uint16_t in1_pin;
  GPIO_TypeDef* in2_port; uint16_t in2_pin;
  GPIO_TypeDef* in3_port; uint16_t in3_pin;
  GPIO_TypeDef* in4_port; uint16_t in4_pin;

  uint8_t dir;
  uint8_t step_idx;
  uint64_t period_us;
  uint64_t prev_time_us;

  uint32_t total_step;
  uint8_t vA, vB;      // for micro step

  void (*init)(struct StepMotor*);
  void (*forward)(struct StepMotor*);
  void (*reverse)(struct StepMotor*);
  void (*brake)(struct StepMotor*);
  void (*slide)(struct StepMotor*);
} StepMotor;

typedef struct
{
  StepMotor *base;
  uint8_t micro_idx;
  uint8_t vA;
  uint8_t vB;
} MicroStepMotor;

typedef enum
{
	NONE,
    FORWARD,
    REVERSE,
    TURN_LEFT,
    TURN_RIGHT,
	STOP
} StepOperation;

typedef enum
{
    MOTOR_LEFT,
    MOTOR_RIGHT
} MotorSide;

void step_init(StepMotor *m);
void step_forward(StepMotor *m);
void step_reverse(StepMotor *m);
void step_brake(StepMotor *m);
void step_slide(StepMotor *m);

#define DEFINE_STEP_MOTOR(name,	\
in1p, in1, 						\
in2p, in2, 						\
in3p, in3, 						\
in4p, in4) 						\
								\
StepMotor name = 				\
{ 								\
	.in1_port = in1p, 			\
	.in1_pin  = in1, 			\
								\
	.in2_port = in2p,			\
	.in2_pin  = in2, 			\
								\
	.in3_port = in3p, 			\
	.in3_pin  = in3, 			\
								\
	.in4_port = in4p,			\
	.in4_pin  = in4, 			\
								\
	.step_idx  = 0, 			\
								\
	.init      = step_init,		\
	.slide     = step_slide, 	\
	.forward   = step_forward,	\
	.reverse   = step_reverse,	\
	.brake     = step_brake		\
};

void step_init_all(void);
void roe_operate(uint8_t m_pin, uint8_t speed, uint8_t m_dir);
void ms_operate(uint8_t m_pin, uint8_t speed, uint8_t m_dir);

uint32_t get_current_steps(void);
void total_step_init(void);
void step_drive(StepOperation op);

void step_idx_init(void);
void step_stop(void);
void step_drive_ratio(uint8_t left_speed, uint8_t right_speed);
void step_set_period(uint16_t period_us);


#endif /* INC_DRIVER_STEP_H_ */
