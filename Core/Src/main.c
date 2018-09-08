
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <apds9960/APDS9960.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static volatile char isr_flag = 0;
#define LIGHT_ON GPIO_PIN_RESET
#define LIGHT_OFF GPIO_PIN_SET
#define LIGHT_GPIO GPIOA
#define LIGHT1_PIN GPIO_PIN_11
#define LIGHT2_PIN GPIO_PIN_10
#define LIGHT1_DEFAULT_VALUE LIGHT_OFF
#define LIGHT2_DEFAULT_VALUE LIGHT_OFF
#define GESTURE_FROM_LEFT_DIR_VALUE LIGHT_ON
#define GESTURE_FROM_RIGHT_DIR_VALUE LIGHT_OFF
#define GESTURE_FROM_UP_DIR_VALUE LIGHT_ON
#define GESTURE_FROM_DOWN_DIR_VALUE LIGHT_OFF


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Process Incoming Gesture
 */
static void process_gesture( void );
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static int Init_APDS9960(){
	// Initialize APDS-9960 (configure I2C and initial values)

	    if (APDS9960_init(&hi2c1) == -1){
//	    	trace_puts("Error: Gesture not init!");
	    	return -1;
	    } else {
	    	if (!APDS9960_enableGestureSensor()){
//	    		trace_puts("Error: APDS-9960 - not enabled!");
	    		return -1;
	    	}
//	    	trace_puts("Initialize APDS-9960 - OK");
	    }
	    return 0;
}
static void light_setup(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LIGHT_OFF);
	HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT1_PIN, LIGHT_OFF);
	HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT2_PIN, LIGHT_OFF);

	for(int i = 0; i < 10; i++){
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LIGHT_ON);
//		HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT1_PIN, LIGHT_ON);
//		HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT2_PIN, LIGHT_ON);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, LIGHT_OFF);
//		HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT1_PIN, LIGHT_OFF);
//		HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT2_PIN, LIGHT_OFF);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  Init_APDS9960();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  light_setup();
  uint32_t debugTimer = HAL_GetTick();
  bool debugF = true;
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//
	  if( isr_flag )
	  {
		  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		  process_gesture();
		  isr_flag = false;
		  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	  }
	   HAL_IWDG_Refresh(&hiwdg);
	   uint32_t t1 = HAL_GetTick();
	   if (t1 - debugTimer > 1000){
		   debugTimer = t1;
		   if (debugF){
			   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		   } else {
			   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		   }
		   debugF = !debugF;
	   }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 1024;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin== GPIO_PIN_15) {
//	if( ir_gesture_is_interrupted( INT_GESTURE ) )
	{
		isr_flag = true;
	}
    /* Clear interrupt flag */
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
  } else{
    __NOP();
  }
}

static void process_gesture()
{
    if ( APDS9960_isGestureAvailable() )
    {
    	int gestureValue = APDS9960_readGesture(400);
        switch ( gestureValue )
        {
          case DIR_UP:
//        	trace_printf("Up\n");
        	  HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT2_PIN, GESTURE_FROM_UP_DIR_VALUE);
            break;
          case DIR_DOWN:
//        	  trace_printf("Down\n");
        	  HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT2_PIN, GESTURE_FROM_DOWN_DIR_VALUE);
            break;
          case DIR_LEFT:
//        	  trace_printf("Left\n");
        	  HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT1_PIN, GESTURE_FROM_LEFT_DIR_VALUE);
        	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GESTURE_FROM_LEFT_DIR_VALUE);
            break;
          case DIR_RIGHT:
//        	  trace_printf("Right\n");
        	  HAL_GPIO_WritePin(LIGHT_GPIO, LIGHT1_PIN, GESTURE_FROM_RIGHT_DIR_VALUE);
        	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GESTURE_FROM_RIGHT_DIR_VALUE);
            break;
          case DIR_NEAR:
//        	  trace_printf("Near\n");
            break;
          case DIR_FAR:
//        	  trace_printf("Far\n");
            break;
          default:
        	  __NOP();
//        	  trace_printf("None\n");
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
