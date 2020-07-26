/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "usart.h"
#include "gpio.h"

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
#define FLASH_APPL_START_ADDR 		0x08004000U
#define FLASH_APPL_PAGES			128   // each page is 128bytes

#define BL_GET_VER 			0xB1
#define BL_GET_CID 			0xB2
#define BL_ERASE_FLASH 		0xB3

#define BL_GET_VER_LEN 		5
#define BL_GET_CID_LEN 		5
#define BL_ERASE_FLASH_LEN  5

#define BL_ERASE_FLASH_SUCCESS	0xC1
#define BL_ERASE_FLASH_FAILURE	0xC2

#define BL_ACK				0xA5
#define BL_NACK 			0xA6

#define TRUE 				1
#define FALSE 				0

GPIO_PinState Jump_App = GPIO_PIN_RESET;
void Bootloader_jump_to_app(void);
void bootloader_uart_read_data(void);
uint8_t bootloader_verify_crc(uint8_t * uart_databuf, uint8_t length);
void bootloader_getcid(void);
void bootloader_getver(uint8_t * uart_databuf);
void bootloader_erase_appl_flash(uint8_t * uart_databuf);
HAL_StatusTypeDef status;
uint8_t uart_databuf[30];

uint32_t received_crc 	= 0;
uint32_t calculated_crc = 0;
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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
 {
   /* USER CODE END WHILE */
	  Jump_App = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	  if(GPIO_PIN_SET == Jump_App)
	  {
		  Bootloader_jump_to_app();
		  break;
	  }
	  else
	  {
		  while(1){
		  	HAL_UART_Transmit(&huart2, (uint8_t *)"In BootLoader\r\n", 15 , 300);
		  	bootloader_uart_read_data();
		  }
	  }
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Bootloader_jump_to_app(void)
{
	uint32_t msp_value = *(uint32_t *) FLASH_APPL_START_ADDR ;
	//uint32_t reset_handler_addr = *(volatile uint32_t *) 0x08004004U ;
	void (*app_reset_handler)(void) = (void *) (*(volatile uint32_t *) (FLASH_APPL_START_ADDR + 4 ));

	HAL_UART_Transmit(&huart2, (uint8_t *)"Exiting BootLoader\r\n", 20 , 300);
	__set_MSP(msp_value);
	app_reset_handler();
}

void bootloader_uart_read_data(void)
{
	uint8_t command  = 0;
	while(1)
	{
		status = HAL_UART_Receive(&huart2,(uint8_t*) uart_databuf, 5, HAL_MAX_DELAY);
		command = uart_databuf[0];
		switch(command)
		{
		case BL_GET_VER:
			bootloader_getver(uart_databuf);
			break;
		case BL_GET_CID:
			bootloader_getcid();
			break;
		case BL_ERASE_FLASH:
			bootloader_erase_appl_flash(uart_databuf);
			break;
		default:
			HAL_UART_Transmit(&huart2,(uint8_t *) "Invalid Command\r\n", 17, HAL_MAX_DELAY);
		}
	}
}

void bootloader_erase_appl_flash(uint8_t * uart_databuf)
{
	uint8_t ack = BL_NACK;
	uint8_t sts = BL_ERASE_FLASH_FAILURE;
	if(  bootloader_verify_crc(uart_databuf, BL_GET_VER_LEN)  )
	{
		ack = BL_ACK;
		HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
		// code to erase application
		FLASH_EraseInitTypeDef pEraseInit;
		uint32_t PageError;
		pEraseInit.TypeErase 	= FLASH_TYPEERASE_PAGES;
		pEraseInit.PageAddress 	= FLASH_APPL_START_ADDR;
		pEraseInit.NbPages		= FLASH_APPL_PAGES;
		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&pEraseInit, &PageError);
		HAL_FLASH_Lock();
		if (PageError == 0xFFFFFFFF)
		{
			sts = BL_ERASE_FLASH_SUCCESS;
		}
		HAL_UART_Transmit(&huart2,(uint8_t *) &sts, 1, 300);
	}
	else
	{
		HAL_UART_Transmit(&huart2, &ack, 1, HAL_MAX_DELAY);
	}
}

void bootloader_getver(uint8_t * uart_databuf)
{
	uint8_t bl_version = 0x10;
	uint8_t ack = BL_NACK;

	if(  bootloader_verify_crc(uart_databuf, BL_GET_VER_LEN)  )
	{
		ack = BL_ACK;
		HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
		HAL_UART_Transmit(&huart2, &bl_version, 1, 300);
	}
	else
	{
		HAL_UART_Transmit(&huart2, &ack, 1, HAL_MAX_DELAY);
	}

}

void bootloader_getcid(void)
{
	uint32_t cid = 0;
	uint8_t ack = BL_NACK;
	if(  bootloader_verify_crc(uart_databuf, BL_GET_CID_LEN)  )
	{
		ack = BL_ACK;
		cid = DBGMCU->IDCODE;
		HAL_UART_Transmit(&huart2,(uint8_t *) &ack,  1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2,(uint8_t *) &cid , 4, HAL_MAX_DELAY);
	}
	else
	{
		HAL_UART_Transmit(&huart2, &ack, 1, HAL_MAX_DELAY);
	}
}

uint8_t bootloader_verify_crc(uint8_t * uart_databuf, uint8_t length)
{
	received_crc 	= 0;
	// MASK the received CRC first
	for (int i = 3 ; i >= 0 ; i--)
	{
		received_crc |= uart_databuf[length-i-1] << 8*i;
	}
	// Calculate CRC from existing Data
	calculated_crc  = HAL_CRC_Calculate(&hcrc,(uint32_t *) uart_databuf, (uint32_t)length-4);
	// Validate the received and calculated
	if(received_crc == calculated_crc)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
