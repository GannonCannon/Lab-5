/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// Hal initializaiton / system clock configh
  HAL_Init();
  SystemClock_Config();

	// I2C2 Initialization
		// Enable I2C2 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		// Configure I2C2 bus timing 
			// Set PRESC to 1
			I2C2->TIMINGR |= (1 << 28);
			// Set SCLL to 0x13
			I2C2->TIMINGR |= 0x13;
			// Set SCLH to 0xF
			I2C2->TIMINGR |= (0xF << 8);
			// Set SDADEL to 0x2
			I2C2->TIMINGR |= (0x2 << 16);
			// Set SCLDEL to 0x4
			I2C2->TIMINGR |= (0x4 << 20);	
		// Enable peripheral / set PE bit
		I2C2->CR1 |= (1 << 0);
	
	// GPIO Initialization
		// Enable GPIOB & GPIOC
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		//Set PB11 to alternate function mode - I2C2_SDA (AF1, 0001)
		GPIOB->MODER |= (1 << 23);
		GPIOB->OTYPER |= (1 << 11);
		GPIOB->AFR[1] &= ~(1 << 13) | (1 << 14) | (1 << 15);
		GPIOB->AFR[1] |= (1 << 12);
		// Set PB13 to alternate function mode - I2C2_SCL (AF5, 0101)
		GPIOB->MODER |= (1 << 27);
		GPIOB->OTYPER |= (1 << 13);
		GPIOB->AFR[1] &= ~(1 << 23) | (1 << 21);
		GPIOB->AFR[1] |= (1 << 20) | (1 << 22);
		// PB14 initialization
		GPIOB->MODER |= (1 << 28);
		GPIOB->OTYPER &= ~(1 << 14);
			// Initialize pin high
			GPIOB->ODR = (1 << 14);
		// PC0 initialization
		GPIOC->MODER |= (1 << 0);
		GPIOC->OTYPER &= ~(1 << 0);
			//Initialize pin high
			GPIOC->ODR = (1 << 0);
		
	
		
	
	
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
