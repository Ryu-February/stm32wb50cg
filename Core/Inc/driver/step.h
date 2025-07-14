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
#define FORWARD     0
#define REVERSE     1

#define SAFE_MAX_RPM    1200
#define MAX_SPEED       100
#define MIN_SPEED       0

#define MICRO_MAX_PWM   255

#define STEP_MODE_FULL  0
#define STEP_MODE_HALF  1
#define STEP_MODE_MICRO 2

#define USE_STEP_MODE   STEP_MODE_HALF   // 변경 가능: FULL, HALF, MICRO

#if (USE_STEP_MODE == STEP_MODE_HALF)
  #define STEP_MASK     0x07
  #define STEP_PER_REV  40
#elif (USE_STEP_MODE == STEP_MODE_FULL)
  #define STEP_MASK     0x03
  #define STEP_PER_REV  20
#elif (USE_STEP_MODE == STEP_MODE_MICRO)
  #define STEP_MASK     0x0F
  #define STEP_PER_REV  80
#endif

typedef struct StepMotor
{
  GPIO_TypeDef* AIN1_Port;
  uint16_t AIN1_Pin;
  GPIO_TypeDef* AIN2_Port;
  uint16_t AIN2_Pin;
  GPIO_TypeDef* BIN1_Port;
  uint16_t BIN1_Pin;
  GPIO_TypeDef* BIN2_Port;
  uint16_t BIN2_Pin;

  uint8_t step_idx;
  uint32_t period_us;
  uint32_t prev_time_us;

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

void sm_init(StepMotor *m);
void sm_forward(StepMotor *m);
void sm_reverse(StepMotor *m);
void sm_brake(StepMotor *m);
void sm_slide(StepMotor *m);

void roe_sm_init(void);
void roe_operate(uint8_t m_pin, uint8_t speed, uint8_t m_dir);
void ms_operate(uint8_t m_pin, uint8_t speed, uint8_t m_dir);


#endif /* INC_DRIVER_STEP_H_ */
