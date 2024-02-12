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

int main(void) {
	
	
	// _________________ START FIRST CHECKOFF
	
	HAL_Init(); // Reset of all peripherals, init the Flash interface and Systick
	SystemClock_Config(); //Configure the system clock
	
	
	
	//RED is 6, BLUE is 7, ORANGE is 8, GREEN is 9

__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, 
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Start PC8 high
	
	/* Set PA0 to input mode (bits 0 and 1 of GPIOA) On*/
	GPIOA->MODER = 0x28000000; // This is the reset state for GPIOA datasheet page 158
	// the above code is a cleaner way to set it to input mode. The datasheet calls out bits 0 and 1 both be set to 0 for input mode, but above code accomplishes the same thing.
	
	//Set the pins to low speed
	GPIOA->OSPEEDR &= ~(1<<0); // Sets the 0th bit = 0 -- NOTE: Next bit is a don't care
	
	//Enable the pull-down resistor --
	GPIOA->PUPDR |= 2; // Since pull down is 10 for bits 1 and 0 respectively (2 in decimal) we can just set this register to = 2.
	
	//set up interrupt on EXTI line 0
	EXTI->IMR |= EXTI_IMR_MR0_Msk;
	
	//Set rising edge trigger
	EXTI->RTSR |= (1<< 0);
	
	//SYSCFG Clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN_Msk;
	
	//Ensure PA0 routes to SYSCFG
	SYSCFG->EXTICR[0] &= ~(7 << 0);
	
	//Enable in NVIC
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn, 3); // Change the priority to 1 to starve Systick, to 3 to allow Systick
	
	NVIC_SetPriority(SysTick_IRQn, 2);
	
	while (1) {
		HAL_Delay(600); // Delay 600ms
		// Toggle the output state of both PC8 and PC9
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	}

	//___________ END OF FIRST CHECKOFF
	
	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	
	
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		GPIOC->BSRR = (1<<6);
		HAL_Delay(500); //500 ms delay
		GPIOC->BSRR = (1<<22);
		HAL_Delay(500); //500 ms delay
		
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

void EXTI0_1_IRQHandler(void)
{
//	unsigned int i = 0;
//	GPIOC->ODR ^= (1<<8); // Flip it
//	GPIOC->ODR ^= (1<<9); // Flip it
//	
////	while(1){
////		//i++;
////	}
////	i=0;
//	
////	while(i < UINT32_MAX-1){
////		i++;
////	}
////	i=0;

//	GPIOC->ODR ^= (1<<8); // Flip it
//	GPIOC->ODR ^= (1<<9); // Flip it
	
	unsigned int count = 0;
	
	GPIOC->ODR ^= (1<<8);
	GPIOC->ODR ^= (1<<9);
	
	while(count < 1500000)
		count++;
	
	GPIOC->ODR ^= (1<<8);
	GPIOC->ODR ^= (1<<9);
	EXTI->PR |= (1<<0);
	
	EXTI->PR = EXTI_PR_PR0; /* clear exti line 0 flag */
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
