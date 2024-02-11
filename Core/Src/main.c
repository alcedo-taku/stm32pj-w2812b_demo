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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wrapper.hpp"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_LED 8
#define USE_BRIGHTNESS 1

uint8_t led_data[MAX_LED][4];
uint8_t led_mod[MAX_LED][4];  // for brightness


int datasentflag = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
    datasentflag = 1;
}

void set_led(int led_num, int red, int green, int blue){
    led_data[led_num][0] = led_num;
    led_data[led_num][1] = green;
    led_data[led_num][2] = red;
    led_data[led_num][3] = blue;
}

void set_brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS
    if (brightness > 45) brightness = 45;
    for (int i=0; i<MAX_LED; i++)
    {
        for (int j=1; j<4; j++)
        {
            float angle = 90-brightness;  // in degrees
            angle = angle * M_PI / 180;  // in rad
            led_mod[i][j] = (led_data[i][j])/(tan(angle));
        }
    }
#endif
}

uint16_t pwmData[(24*MAX_LED)+50];

void ws2812_send(void){
    uint32_t indx = 0;
    uint32_t color;
    for (uint8_t i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
        color = ((led_mod[i][1]<<16) | (led_mod[i][2]<<8) | (led_mod[i][3]));
#else
        color = ((led_data[i][1]<<16) | (led_data[i][2]<<8) | (led_data[i][3]));
#endif
        for (int j = 23; j >= 0; j--) {
            if (color&(1<<j)) pwmData[indx] = 60; // 2/3 of 90
            else pwmData[indx] = 30; // 1/3 of 90
            indx++;
        }
    }

    for (uint8_t i = 0; i < 50; i++) {
        pwmData[indx] = 0;
        indx++;
    }

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwmData, indx);
    while(!datasentflag){};
    datasentflag = 0;
}

uint16_t pwmData_singl[24];

void send(int green, int red, int blue){
    uint32_t color = (green<<16) | (red<<8) | blue;
    for (int i = 23; i >= 0; i--) {
        if (color&(1<<i)) pwmData_singl[i] = 60; // 2/3 of 90
        else pwmData_singl[i] = 30; // 1/3 of 90
    }
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)pwmData_singl, 24);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//  send(5, 255, 101);
//  set_led(0, 255, 0, 0);
//  set_led(1, 0, 255, 0);
//  set_led(2, 0, 0, 255);
//  ws2812_send();
  init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    send(5, 255, 101);
    set_led(0, 255, 0, 0);
    set_led(1, 0, 255, 0);
    set_led(2, 0, 0, 255);
    set_led(3, 46, 89, 128);
    set_led(4, 156, 233, 100);
    set_led(5, 102, 0, 235);
    set_led(6, 47, 38, 77);
    set_led(7, 255, 200, 0);

    for (int i=0; i<46; i++){
        set_brightness(i);
        ws2812_send();
        HAL_Delay (50);
    }

    for (int i=45; i>=0; i--){
        set_brightness(i);
        ws2812_send();
        HAL_Delay (50);
    }

//        ws2812_send();
//        HAL_Delay(100);
        loop();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
