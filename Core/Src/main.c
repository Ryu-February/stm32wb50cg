/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "rgb.h"
#include "color.h"
#include "uart.h"
#include "i2c.h"
#include "flash.h"
#include "step.h"
#include "ir.h"
#include "buzzer.h"
#include "pitches.h"
#include "queue.h"
#include "line_tracing.h"

//#include "rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile bool is_sleep_requested = false;
bool restored_standby = false;
extern volatile bool uart_enable;
extern volatile bool pb0_pressed;
//static unsigned char once_flag2 = false;
volatile bool check_color = false;

volatile bool mode_entry = false;
extern volatile uint64_t timer2_1us;
extern volatile uint32_t timer16_10us;
extern volatile uint32_t timer17_ms;
extern volatile bool delay_flag;
uint8_t rx_buf[32];       // 수신 버퍼
uint8_t rx_index = 0;     // 수신 인덱스
bool command_ready = false; // 파싱 준비 완료 flag
int steps = 0;
extern volatile uint64_t tim2_us;
extern volatile bool idx_change;
extern volatile unsigned char cur_mode;
extern volatile bool flag_step_drive;
extern volatile StepOperation step_op;
color_t detected_left = COLOR_BLACK;
color_t detected_right = COLOR_BLACK;
StepOperation op = NONE;
color_mode_t color_mode = MODE_NONE;

extern volatile bool tim16_irq;
volatile bool line_tracing_mod = false;

uint8_t  offset_side = 0;
uint16_t offset_black = 0;
uint16_t offset_white = 0;
uint16_t offset_average = 0;

bh1745_color_data_t line_left, line_right;

extern color_mode_t insert_queue[MAX_INSERTED_COMMANDS];
extern uint8_t insert_index;
extern volatile uint8_t repeat_target;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t byte = rx_buf[rx_index];

        if (byte == '>')  // 끝 문자
        {
            command_ready = true;
            rx_buf[rx_index + 1] = '\0';  // null-terminate
            rx_index = 0;
        }
        else
        {
            if (rx_index < sizeof(rx_buf) - 2)
                rx_index++;
        }

        HAL_UART_Receive_IT(&huart1, &rx_buf[rx_index], 1);  // 다음 바이트 준비
    }
}

void parse_uart_command(void)
{
    if (!command_ready)
        return;

    command_ready = false;

    if (rx_buf[0] == '<' && rx_buf[1] != '\0')
    {
        char dir = rx_buf[1];
        steps = atoi((char*)&rx_buf[2]);  // 예: "100"

        if (dir == 'F')
		{
			op = FORWARD;
			detected_left = COLOR_RED;
		}
        else if (dir == 'B')
        {
        	op = REVERSE;
        	detected_left = COLOR_BLUE;
        }
        else if (dir == 'L')
		{
			op = TURN_LEFT;
			detected_left = COLOR_GREEN;
		}
        else if (dir == 'R')
		{
			op = TURN_RIGHT;
			detected_left = COLOR_WHITE;
		}
    }
    uart_printf("rx_buffer: %s\r\n", rx_buf);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) // PB0 눌림
    {
//        is_sleep_requested ^= 1; // 0->1 또는 1->0 토글!
    	pb0_pressed ^= true;

    	if(pb0_pressed == true)
		{
    		check_color = true;
		}

    }
}

void standby_with_rtc(uint32_t seconds)
{
//	CLEAR_BIT(PWR->CR4, PWR_CR4_C2BOOT);
    uint32_t ticks = seconds * 2048;

    // GPIO High-Z 설정 (예: PA4, PA5, PA6)

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);  // 먼저 클리어!
//    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

    if(   (LL_PWR_IsActiveFlag_C1SB() == 0)
         || (LL_PWR_IsActiveFlag_C2SB() == 0)
        )
      {
        /* Set the lowest low-power mode for CPU2: shutdown mode */
        LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);//cpu2 shutdown
      }
    HAL_PWR_EnterSTANDBYMode();  // 진입
//    NVIC_SystemReset();  // 진짜 리셋!
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

//  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);
  HAL_UART_Init(&huart1);
  HAL_UART_Receive_IT(&huart1, &rx_buf[rx_index], 1);

  rgb_init();
  buzzer_init();

  bh1745_init(BH1745_ADDR_LEFT);
  bh1745_init(BH1745_ADDR_RIGHT);

  step_init_all();
  step_stop();
  step_set_period(1500, 1500);

  load_color_reference_table();
  calculate_color_brightness_offset();
  debug_print_color_reference_table();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  bh1745_color_data_t left_color, right_color;

  Command current_cmd;
  bool executing = false;
  int executed_steps = 0;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  parse_uart_command(); // 수신 명령 파싱 및 실행

	  if (!executing)
	  {
		  if (dequeue_command(&current_cmd))
		  {
			  executing = true;
			  executed_steps = 0;
 //			  check_color = true;
			  uart_printf("START %d step (%d)\r\n", current_cmd.op, current_cmd.steps);
		  }
	 }

	  if(check_color == true && executing)
	  {
		  HAL_Delay(1000);

		  switch (current_cmd.op)
		  {
			  case NONE :
				  check_color = false;
				  command_ready = false;
				  uart_printf("ERROR\r\n");
				  break;
			  case FORWARD :
				  uart_printf("FORWARD\r\n");
				  break;
			  case REVERSE :
				  uart_printf("REVERSE\r\n");
				  break;
			  case TURN_LEFT :
				  uart_printf("TURN_LEFT\r\n");
				  break;
			  case TURN_RIGHT :
				  uart_printf("TURN_RIGHT\r\n");
				  break;
			  default :
				  uart_printf("UNKNOWN CMD\r\n");
				  break;
		  }

		  uart_printf("step: %d\r\n", current_cmd.steps);

		  if (current_cmd.op != NONE && current_cmd.steps > 0 && current_cmd.steps <= 10000)
		  {
			  for (int i = 0; i < current_cmd.steps;)
			  {
				  uint64_t now = __HAL_TIM_GET_COUNTER(&htim2);
				  static uint64_t prev_us = 0;

				  if(now - prev_us > 10)
				  {
					  prev_us = now;
					  step_drive(current_cmd.op);
				  }

				  if(idx_change == true)
				  {
					  idx_change = false;
					  i++;
					  if(total_steps < i)
					  {
						  check_color = false;
					  }
				  }
			  }

			  step_stop();
 //	          check_color = false;
			  memset(rx_buf, 0, sizeof(rx_buf));
			  executing = false;
		  }
	  }

	  /*
	  static bool buz_once = true;
	  if(buz_once == true)
	  {
		  buzzer_beep(B_8, 300);
//		  buzzer_beep(B_5, 100);
//		  buzzer_beep(B_8, 100);
//		  buzzer_beep(15000, 200);
//		  buzzer_beep(DS_5, 100);
//		  buzzer_beep(G_5, 200);
//		  buzzer_beep(C_6, 200);
		  buz_once = false;
	  }


	  if(check_color == true && cur_mode == 0)
	  {
		  left_color  = bh1745_read_rgbc(BH1745_ADDR_LEFT);
		  right_color = bh1745_read_rgbc(BH1745_ADDR_RIGHT);

		  detected_left =
				  classify_color(BH1745_ADDR_LEFT, left_color.red, left_color.green, left_color.blue, left_color.clear);
		  detected_right =
				  classify_color(BH1745_ADDR_RIGHT, right_color.red, right_color.green, right_color.blue, right_color.clear);

		  uint16_t left_brightness, right_brightness = 0;

		  left_brightness = calculate_brightness(left_color.red, left_color.green, left_color.blue);
		  right_brightness = calculate_brightness(right_color.red, right_color.green, right_color.blue);

		  uart_printf("-------------------------------------------------------\r\n");
		  uart_printf("[LEFT ] R:%u G:%u B:%u C:%u\r\n",
		              left_color.red, left_color.green, left_color.blue, left_color.clear);

		  uart_printf("[RIGHT] R:%u G:%u B:%u C:%u\r\n",
		              right_color.red, right_color.green, right_color.blue, right_color.clear);
		  uart_printf("[LEFT ] Detected Color: %s\r\n", color_to_string(detected_left));
		  uart_printf("[RIGHT] Detected Color: %s\r\n", color_to_string(detected_right));
		  uart_printf("[LEFT ] brightness: %d\r\n", left_brightness);
		  uart_printf("[RIGHT] brightness: %d\r\n", right_brightness);
		  uart_printf("default offset: %d\r\n", left_brightness - right_brightness);
		  uart_printf("-------------------------------------------------------\r\n");

		  if (detected_left == detected_right && color_mode != MODE_INSERT)
		  {
			  switch (detected_left)
			  {
			  	  case COLOR_RED:          color_mode = MODE_FORWARD; 		break;
			  	  case COLOR_ORANGE:       color_mode = MODE_BACKWARD; 		break;
			  	  case COLOR_YELLOW:       color_mode = MODE_LEFT; 			break;
			  	  case COLOR_GREEN:        color_mode = MODE_RIGHT; 		break;
			  	  case COLOR_BLUE:         color_mode = MODE_LINE_TRACE;  	break;
			  	  case COLOR_PURPLE:       color_mode = MODE_FAST_FORWARD;  break;
			  	  case COLOR_LIGHT_GREEN:  color_mode = MODE_SLOW_FORWARD;  break;
			  	  case COLOR_SKY_BLUE:     color_mode = MODE_FAST_BACKWARD; break;
			  	  case COLOR_PINK:         color_mode = MODE_SLOW_BACKWARD; break;
			  	  case COLOR_GRAY:         color_mode = MODE_LONG_FORWARD; 	break;
			  	  default:                 break;
			  }
		  }
		  else
		  {
			  if (detected_left == COLOR_LIGHT_GREEN && detected_right == COLOR_ORANGE && color_mode != MODE_INSERT)
			  {
				  color_mode = MODE_INSERT;
				  uart_printf(">> Mode [INSERT]\r\n");
//		  			insert_index = 0;
			  }
			  else if (detected_left == COLOR_LIGHT_GREEN && detected_right == COLOR_ORANGE && color_mode == MODE_INSERT)
			  {
				  color_mode = MODE_RUN;
				  uart_printf(">> Mode [RUN]\r\n");
			  }
			  else
			  {
				  if (detected_left == detected_right && color_mode == MODE_INSERT)
				  {
					  if (detected_left == COLOR_BLACK)
					  {
						  repeat_target = 2;
					  }
					  else if(detected_left == COLOR_WHITE)
					  {
						  repeat_target = 3;
					  }
					  else
					  {
						  if (insert_index < MAX_INSERTED_COMMANDS)
						  {
							  insert_queue[insert_index++] = color_to_mode(detected_left);
							  uart_printf(">> Inserted Command [%d]: %d\r\n", insert_index, color_to_mode(detected_left));
						  }
						  else
						  {
							  uart_printf("!! Queue Full\r\n");
						  }
					  }
				  }
				  delay_flag = false;
			  }
		  }

		  HAL_Delay(500);
		  delay_flag = true;
		  check_color = false;
	  }

	  static unsigned char once_flag = 0;

	  if(cur_mode == MODE_CALIBRATION && color_mode != MODE_INSERT)
	  {
		  if(!once_flag)
		  {
			  uart_printf("bh1745 initialize\r\n");
			  flash_erase_color_table(BH1745_ADDR_LEFT);
			  flash_erase_color_table(BH1745_ADDR_RIGHT);
			  once_flag = 1;
		  }

		  if(pb0_pressed == false)
		  {
			  mode_entry = true;
		  }

		  if(pb0_pressed == true && mode_entry == true)
		  {
			  static uint8_t init_cnt = 0;

			  uart_printf("color set: [%s]\r\n", color_to_string(init_cnt));


			  left_color  = bh1745_read_rgbc(BH1745_ADDR_LEFT);
			  right_color = bh1745_read_rgbc(BH1745_ADDR_RIGHT);

			  uart_printf("[LEFT]  R:%u G:%u B:%u C:%u\r\n",
						  left_color.red, left_color.green, left_color.blue, left_color.clear);

			  uart_printf("[RIGHT] R:%u G:%u B:%u C:%u\r\n",
						  right_color.red, right_color.green, right_color.blue, right_color.clear);

			  save_color_reference(BH1745_ADDR_LEFT, init_cnt, left_color.red, left_color.green, left_color.blue);
			  save_color_reference(BH1745_ADDR_RIGHT, init_cnt, right_color.red, right_color.green, right_color.blue);

			  uart_printf("--------------------------------\r\n");
			  init_cnt++;

			  mode_entry = false;

			  if(init_cnt > COLOR_GRAY)
			  {
				  once_flag = 0;
				  cur_mode = 0;
				  init_cnt = 0;
				  debug_print_color_reference_table();
			  }

		  }
	  }
//	  uart_printf("hi\r\n");

	  if(line_tracing_mod == true)
	  {
		  line_tracing_pid();
	  }

//	  uart_printf("hi\r\n");
//	  HAL_Delay(1000);
//	  uart_printf("timer2_1us: %d\r\n", (uint32_t) timer2_1us);
//	    if (flag_step_drive)
//	    {
//	    	uint64_t now = __HAL_TIM_GET_COUNTER(&htim2);
//			static uint64_t prev_us = 0;
//
//			if(now - prev_us > 10)
//			{
//				flag_step_drive = false;  // 사용 후 바로 클리어
//				if (step_op != NONE)
//				{
//					step_drive(step_op);  // 안전하게 main loop에서 실행
//				}
//				prev_us = now;
//			}
//
//	    }*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x106133FF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
//  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
//
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
//
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 63;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 29;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 63;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
