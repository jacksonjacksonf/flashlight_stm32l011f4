/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	idle,
	start,
	run,
	stop,
	chn_brtnss,
	low_battery
}state_machine_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED 10
#define PAUSE 10000
#define MAX_BRNESS 150
#define DOUBLE_CLICK_TIME_OUT 200
#define SLEEP_MS 5000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

uint16_t g_level = 0;
uint8_t g_bt_flag = 0;

uint16_t g_brightness = 200;

uint8_t g_direction = 0;
uint16_t g_speed = 0;
uint16_t g_long_press = 0;
uint16_t g_double_click_time_out = 0;

uint16_t sleep_time_out = SLEEP_MS;

extern uint8_t tic;

state_machine_t st = idle;

uint16_t CRT10_square(uint16_t val) {
    return ((long)val * val + 1023) >> 10;
}

uint16_t CRT10_cubic(uint16_t val) {
    return ((long)val * val * val + 2094081) >> 20;
}

void set_brightness(uint16_t val)
{
	if(val > 1023)
	{
		val = 1023;
	}
	  TIM2->CCR1 = CRT10_square(val);
}
void button_on(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
}

void button_off(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}

void time_out(void (*p_sleep_foo)(void))
{
	if(sleep_time_out > 1)
	{
		sleep_time_out--;
	}
	else
	{
		//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		p_sleep_foo();
	}
}
void start_lp_timer_10s(void)
{
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1,2890,0);
}

void sleep_mode(void)
{
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void stop_mode(void)
{
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
void reset_led_satus(void)
{
	g_level = 0;
	g_direction = 0;
}

void check_button(void)
{
	if(g_bt_flag == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0)
	{
		if(g_long_press > 2002)
		{
			g_bt_flag = 0;
			g_long_press = 0;
			g_level = g_brightness;
			set_brightness(g_level);
			st = run;
		}
		else
		{
			g_bt_flag = 0;
			if(g_double_click_time_out > 0)
			{
				g_brightness = 1023;
			}
			else
			{
				g_double_click_time_out = DOUBLE_CLICK_TIME_OUT;
			}

			if(st == idle)
			{
				st = start;
			}
			else
			{
				st = stop;
			}
		}

	}
	else if(g_bt_flag == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1)
	{
		g_long_press++;

		if(g_long_press > 2000 && g_long_press < 2002 && st == run)
		{
			st = chn_brtnss;
			g_brightness = g_level;
		}

		if(g_long_press > 20000)
		{
			button_off();
			HAL_Delay(100);
			g_bt_flag = 0;
			st = idle;
			set_brightness(0);
			button_on();
		}

	}
	else
	{
		g_long_press = 0;

		if(g_double_click_time_out > 0)
		{
			g_double_click_time_out--;
		}
	}

}

static void indicate_low_batt_status(void)
{
	//HAL_LPTIM_TimeOut_Start_IT(&hlptim1,2890,0);
	if(g_speed++ > SPEED)
	{
		g_speed = 0;

		if(g_direction)
		{
			if(g_level-- > 1)
			{
				set_brightness(g_level);
			}
			else
			{
				g_direction = 0;
				start_lp_timer_10s();
				HAL_Delay(50);
				stop_mode();

			}
		}
		else
		{
			if(g_level++ < MAX_BRNESS)
			{
				 set_brightness(g_level);
			}
			else
			{
				g_direction = 1;
			}
		}
	}

}

void state_of_device(state_machine_t state)
{
	switch(state)
	{
		case idle:
		{
			//put your code here
			time_out(stop_mode);
			break;
		}
		case start:
		{
			//put your code here
			if(g_level++ < g_brightness - 1)
			{
				set_brightness(g_level);
			}
			else
			{
				st = run;
			}
			break;
		}
		case run:
		{
			//put your code here
			time_out(sleep_mode);
			break;
		}
		case stop:
		{
			//put your code here
			if(g_level-- > 1)
			{
				set_brightness(g_level);
			}
			else
			{
				st = idle;
			}
			break;
		}
		case low_battery:
		{
			//put your code here
			indicate_low_batt_status();
			break;
		}
		case chn_brtnss:
		{
			//put your code here
			if(g_direction)
			{
				if(g_brightness-- > 80)
				{
					set_brightness(g_brightness);
				}
				else
				{
					g_direction = 0;
				}
			}
			else
			{
				if(g_brightness++ < 1023)
				{
					set_brightness(g_brightness);
				}
				else
				{
					g_direction = 1;
				}
			}
			break;
		}
		default:
		{
			//put your code here

			break;
		}

	}
}

void start_pvd(void)
{
	PWR_PVDTypeDef sConfigPVD;

	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PVD_IRQn);

	sConfigPVD.PVDLevel = PWR_CR_PLS_LEV6;
	sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING_FALLING;


	HAL_PWR_ConfigPVD(&sConfigPVD);
	HAL_PWR_EnablePVD();

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_LPTIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
  //start_pvd();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  set_brightness(0);
  button_on();
  start_pvd();

  //HAL_LPTIM_TimeOut_Start_IT(&hlptim1,0xffff,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(tic > 0)
	  {
		  state_of_device(st);
		  check_button();

		  tic = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_ENDOFPERIOD;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1024;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if(GPIO_Pin == GPIO_PIN_5)
   {
	   g_bt_flag = 1;
	   sleep_time_out = SLEEP_MS;
	  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   }
}

void HAL_PWR_PVDCallback(void)
{

	if(PWR->CSR & PWR_CSR_PVDO)
	{
		//dead battery
		st = low_battery;
		button_off();
		reset_led_satus();
		set_brightness(0);
	}
	else
	{
		HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
		st = idle;
		set_brightness(0);
		button_on();
	}
}
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

#ifdef  USE_FULL_ASSERT
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
