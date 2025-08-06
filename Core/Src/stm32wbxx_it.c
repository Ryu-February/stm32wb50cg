/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wbxx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32wbxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <color.h>
#include <rgb.h>
#include <stdbool.h>
#include <step.h>
#include <buzzer.h>
#include <stm32wbxx_hal.h>
#include <stm32wbxx_hal_gpio.h>
#include <stm32wbxx_hal_i2c.h>
#include <stm32wbxx_hal_rcc.h>
#include <stm32wbxx_hal_rtc.h>
#include <stm32wbxx_hal_rtc_ex.h>
#include <stm32wbxx_hal_tim.h>
#include <stm32wbxx_hal_uart.h>
#include <sys/_stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int i = 0;
volatile uint32_t timer2_1us = 0;
volatile uint32_t timer16_1us = 0;
volatile uint32_t timer16_10us = 0;
volatile uint32_t timer17_ms = 0;
volatile uint32_t timer17_uart_ms = 0;
volatile uint32_t pb0_pressed_time = 0;

volatile bool delay_flag = false;

volatile unsigned char cur_mode = false;

volatile bool switch_valid = false;
volatile bool uart_enable = false;
volatile bool pb0_pressed = false;
volatile bool tim16_irq = false;

volatile bool flag_step_drive = false;
volatile StepOperation step_op = NONE;

volatile uint8_t repeat_target = 1;      // 총 몇 번 반복할지 (예: 2번 반복하고 종료)
volatile uint16_t buz_cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern color_t detected_left;
extern color_t detected_right;
extern color_mode_t color_mode;
extern volatile bool check_color;
extern volatile bool line_tracing_mod;
extern color_mode_t insert_queue[MAX_INSERTED_COMMANDS];
extern uint8_t insert_index;
extern volatile bool buzzer_enabled;
extern volatile bool buzzer_start;
extern volatile bool card_once;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WBxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wbxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC wake-up interrupt through EXTI line 19.
  */
void RTC_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_WKUP_IRQn 0 */

  /* USER CODE END RTC_WKUP_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_WKUP_IRQn 1 */

  /* USER CODE END RTC_WKUP_IRQn 1 */
}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
//  if (htim1.Instance != NULL)
//  {
//    HAL_TIM_IRQHandler(&htim1);
//  }
  if (htim16.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim16);
  }
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) &&
	 __HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE))
  {
	  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

//	  if (buzzer_enabled)
//		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);  // 피에조 부저 사각파 출력

	  if(buzzer_start)
	  {
		  if(buz_cnt < 100)
		  	  buzzer_op(BUZZER_TOGGLE);
		  else
		  {
			  buzzer_op(BUZZER_OFF);
			  buzzer_start = false;
		  }
	  }

  }
  if (htim1.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim1);
  }

  timer16_10us++;
  tim16_irq = true;
  rgb_set_color(detected_left);

  static uint16_t fix_step = 0;
  static uint16_t prev_ms = 0;

  if(delay_flag == true)
  {
//	  flag_step_drive = true;
	  switch (color_mode)
	    {
			case MODE_FORWARD :
				#if(_USE_STEP_NUM == _STEP_NUM_119)
				fix_step = 1000;
				#elif(_USE_STEP_NUM == _STEP_NUM_728)
				fix_step = 1500;
				#else
				fix_step = 2800;
				#endif
				step_drive(FORWARD);
				step_op = FORWARD;
				break;
			case MODE_BACKWARD :
				#if(_USE_STEP_NUM == _STEP_NUM_119)
				fix_step = 1000;
				#elif(_USE_STEP_NUM == _STEP_NUM_728)
				fix_step = 1500;
				#else
				fix_step = 2800;
				#endif
				step_drive(REVERSE);
				step_op = REVERSE;
			 	break;
			case MODE_LEFT :
				#if(_USE_STEP_NUM == _STEP_NUM_119)
				fix_step = 390;
				#elif(_USE_STEP_NUM == _STEP_NUM_728)
				fix_step = 535;
				#else
				fix_step = 1130;
				#endif
				step_drive(TURN_LEFT);
				step_op = TURN_LEFT;
				break;
			case MODE_RIGHT :
				#if(_USE_STEP_NUM == _STEP_NUM_119)
				fix_step = 390;
				#elif(_USE_STEP_NUM == _STEP_NUM_728)
				fix_step = 535;
				#else
				fix_step = 1130;
				#endif
				step_drive(TURN_RIGHT);
				step_op = TURN_RIGHT;
				break;
			case MODE_LINE_TRACE :
				fix_step = 30000;
				line_tracing_mod = true;
				step_drive(step_op);
				break;
			case MODE_FAST_FORWARD :
				fix_step = 2500;
				step_drive(FORWARD);
				step_set_period(2000, 700);
				break;
			case MODE_SLOW_FORWARD :
				fix_step = 2500;
				step_drive(FORWARD);
				step_set_period(700, 2000);
				break;
			case MODE_FAST_BACKWARD :
				fix_step = 1000;
				step_drive(REVERSE);
				step_set_period(1500, 1000);
				break;
			case MODE_SLOW_BACKWARD :
				fix_step = 1000;
				step_drive(REVERSE);
				step_set_period(1000, 1500);
				break;
			case MODE_LONG_FORWARD :
				fix_step = 1900;
				step_drive(FORWARD);
				break;
			case MODE_INSERT :
//				color_mode = MODE_NONE;
				break;
			case MODE_RUN :
				static uint8_t run_index = 0;
				static bool is_running = false;
				static uint16_t run_period_left, run_period_right = 0;
				static uint16_t run_step = 0;
				static uint8_t repeat_count = 0;       // 현재까지 몇 번 반복했는지


				if (!is_running && run_index < insert_index)
				{
					color_mode_t next_cmd = insert_queue[run_index];
					step_op = mode_to_step(next_cmd);  // 이거도 따로 함수화 가능
					run_step = mode_to_step_count(next_cmd);  // 예: 1000 등
					run_period_left  = mode_to_left_period(insert_queue[run_index]);
					run_period_right = mode_to_right_period(insert_queue[run_index]);
					step_set_period(run_period_left, run_period_right);
					is_running = true;

				}
				else if (is_running)
				{
					// fix_step 만큼 스텝 수행 완료된 뒤
					step_drive(step_op);

					if (get_current_steps() > run_step)
					{
						static uint8_t once_flag = 1;
						if(once_flag)
						{
							prev_ms = timer17_ms;
							once_flag = 0;
						}
						step_op = STOP;

						uint16_t cur_ms = timer17_ms;
						if(cur_ms - prev_ms > 1000)
						{
//							prev_ms = cur_ms;
							is_running = false;
							run_index++;
							total_step_init();
							once_flag = 1;
						}

						if (run_index >= insert_index)
						{
							run_index = 0;
							repeat_count++;
							if(repeat_count >= repeat_target)
							{
								step_stop();
								color_mode = MODE_NONE;
//								insert_index = 0;
								run_index = 0;
								step_set_period(500, 500);
								line_tracing_mod = false;
								step_op = NONE;
								delay_flag = false;
								total_step_init();
//								repeat_target = 1;
								repeat_count = 0;
								card_once = false;
							}

						}

					}

				}
				break;
			default :
				step_op = NONE;
				break;
	    }
  }

  if(get_current_steps() > fix_step && color_mode != MODE_RUN)
  {
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  step_stop();
	  detected_left = COLOR_BLACK;
	  total_step_init();
	  step_idx_init();
	  delay_flag = false;
	  line_tracing_mod = false;
	  step_op = NONE;
	  step_set_period(500, 500);
  }

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  if (htim1.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim1);
  }
  if (htim17.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim17);
  }
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */
  timer17_ms++;

//  uint16_t buz_cnt = 0;
//  static bool buz_init = false;

//  if(!buz_init)
//  {
//	  if(++buz_cnt > 100)
//	  {
//		  buz_init = true;
//	  }
//	  else
//		  buzzer_op(BUZZER_TOGGLE);
//  }

  if(buzzer_start)
  {
	  buz_cnt++;
  }


  if(pb0_pressed == true)
  {
	  if(++pb0_pressed_time >= 5000 && cur_mode != MODE_CALIBRATION)
	  {
		  cur_mode = MODE_CALIBRATION;
		  pb0_pressed_time = 0;
	  }
	  else if(++pb0_pressed_time >= 5000 && cur_mode == MODE_CALIBRATION)
	  {
		  cur_mode = 0;
		  pb0_pressed_time = 0;
	  }
  }
  else
  {
	  pb0_pressed_time = 0;
  }


//  if(line_tracing_mod == true)
//  {
//	  uint32_t now = timer17_ms;
//	  static uint32_t prev = 0;
//
//	  if(now - prev > 100)
//	  {
//		  prev = now;
//		  line_tracing_pid();
//	  }
//  }
  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
//  timer2_1us = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
