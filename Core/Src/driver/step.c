/*
 * step.c
 *
 *  Created on: Jul 11, 2025
 *      Author: RCY
 */


#include "step.h"

extern TIM_HandleTypeDef htim16;  // 10kHz interrupt for microstep PWM

static const uint8_t STEP_TABLE[8][4] = {
    {1,0,1,0}, {0,0,1,0}, {0,1,1,0}, {0,1,0,0},
    {0,1,0,1}, {0,0,0,1}, {1,0,0,1}, {1,0,0,0}
};

static const uint8_t MICRO_TABLE[16][2] = {
    {128, 255}, {180, 240}, {224, 200}, {253, 144},
    {255, 80}, {238, 28}, {200, 2}, {144, 0},
    {80, 4}, {28, 28}, {2, 80}, {0, 144},
    {4, 200}, {28, 240}, {80, 255}, {144, 253}
};

StepMotor motor_l = {
  .AIN1_Port = GPIOA, .AIN1_Pin = GPIO_PIN_0,
  .AIN2_Port = GPIOA, .AIN2_Pin = GPIO_PIN_1,
  .BIN1_Port = GPIOA, .BIN1_Pin = GPIO_PIN_2,
  .BIN2_Port = GPIOA, .BIN2_Pin = GPIO_PIN_3,
};

StepMotor motor_r = {
  .AIN1_Port = GPIOB, .AIN1_Pin = GPIO_PIN_4,
  .AIN2_Port = GPIOB, .AIN2_Pin = GPIO_PIN_5,
  .BIN1_Port = GPIOB, .BIN1_Pin = GPIO_PIN_6,
  .BIN2_Port = GPIOB, .BIN2_Pin = GPIO_PIN_7,
};

MicroStepMotor ms_l = { .base = &motor_l, .micro_idx = 0 };
MicroStepMotor ms_r = { .base = &motor_r, .micro_idx = 8 };

volatile uint8_t ms_pwm_cnt = 0;

static void apply_coils(StepMotor *m)
{
  const uint8_t* seq = STEP_TABLE[m->step_idx];
  HAL_GPIO_WritePin(m->AIN1_Port, m->AIN1_Pin, seq[0]);
  HAL_GPIO_WritePin(m->AIN2_Port, m->AIN2_Pin, seq[1]);
  HAL_GPIO_WritePin(m->BIN1_Port, m->BIN1_Pin, seq[2]);
  HAL_GPIO_WritePin(m->BIN2_Port, m->BIN2_Pin, seq[3]);
}

static void apply_ms_coils(StepMotor *m, uint8_t vA, uint8_t vB)
{
  HAL_GPIO_WritePin(m->AIN1_Port, m->AIN1_Pin, ms_pwm_cnt < vA);
  HAL_GPIO_WritePin(m->AIN2_Port, m->AIN2_Pin, ms_pwm_cnt < (MICRO_MAX_PWM - vA));
  HAL_GPIO_WritePin(m->BIN1_Port, m->BIN1_Pin, ms_pwm_cnt < vB);
  HAL_GPIO_WritePin(m->BIN2_Port, m->BIN2_Pin, ms_pwm_cnt < (MICRO_MAX_PWM - vB));
}

void sm_init(StepMotor *m)
{
  m->step_idx = 0;
  m->forward = sm_forward;
  m->reverse = sm_reverse;
  m->brake = sm_brake;
  m->slide = sm_slide;
}

void sm_forward(StepMotor *m)
{
  apply_coils(m);
  m->step_idx = (m->step_idx + 1) & STEP_MASK;
}

void sm_reverse(StepMotor *m)
{
  apply_coils(m);
  m->step_idx = (m->step_idx - 1 + (STEP_MASK+1)) & STEP_MASK;
}

void sm_brake(StepMotor *m)
{
  HAL_GPIO_WritePin(m->AIN1_Port, m->AIN1_Pin, 1);
  HAL_GPIO_WritePin(m->AIN2_Port, m->AIN2_Pin, 1);
  HAL_GPIO_WritePin(m->BIN1_Port, m->BIN1_Pin, 1);
  HAL_GPIO_WritePin(m->BIN2_Port, m->BIN2_Pin, 1);
}

void sm_slide(StepMotor *m)
{
  HAL_GPIO_WritePin(m->AIN1_Port, m->AIN1_Pin, 0);
  HAL_GPIO_WritePin(m->AIN2_Port, m->AIN2_Pin, 0);
  HAL_GPIO_WritePin(m->BIN1_Port, m->BIN1_Pin, 0);
  HAL_GPIO_WritePin(m->BIN2_Port, m->BIN2_Pin, 0);
}

static uint32_t rpm_to_period(uint16_t rpm)
{
  if (rpm == 0) return 0xFFFFFFFF;
  return 60000000UL / (rpm * STEP_PER_REV);
}

void roe_operate(uint8_t m_pin, uint8_t speed, uint8_t m_dir)
{
  StepMotor *m = (m_pin == LEFT) ? &motor_l : &motor_r;
  if (speed == 0)
  {
    m->brake(m);
    return;
  }
  uint32_t rpm = speed * SAFE_MAX_RPM / MAX_SPEED;
  m->period_us = rpm_to_period(rpm);

  uint32_t now = __HAL_TIM_GET_COUNTER(&htim16);
  if (now - m->prev_time_us >= m->period_us)
  {
    m->prev_time_us = now;
    (m_dir == FORWARD ? m->forward : m->reverse)(m);
  }
}

void ms_operate(uint8_t m_pin, uint8_t speed, uint8_t m_dir)
{
  MicroStepMotor *m = (m_pin == LEFT) ? &ms_l : &ms_r;
  if (speed == 0)
  {
    m->base->brake(m->base);
    return;
  }

  m->base->period_us = rpm_to_period(speed * SAFE_MAX_RPM / MAX_SPEED);
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim16);
  if (now - m->base->prev_time_us < m->base->period_us) return;
  m->base->prev_time_us = now;

  m->micro_idx = (m->micro_idx + (m_dir == FORWARD ? 1 : STEP_MASK)) & STEP_MASK;
  m->vA = MICRO_TABLE[m->micro_idx][0];
  m->vB = MICRO_TABLE[m->micro_idx][1];
}

// 💥 Timer interrupt (10kHz or faster!)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM16)
  {
    ms_pwm_cnt = (ms_pwm_cnt + 1) % MICRO_MAX_PWM;
    apply_ms_coils(ms_l.base, ms_l.vA, ms_l.vB);
    apply_ms_coils(ms_r.base, ms_r.vA, ms_r.vB);
  }
}
