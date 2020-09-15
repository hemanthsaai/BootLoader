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

/* This Macro will mask last 4 bytes of the buffer(which is CRC)  */
#define MASK_CRC_BYTES(buf,len_of_buf) ((buf[len_of_buf-1]<<24)| \
						   (buf[len_of_buf-2]<<16)| \
						   (buf[len_of_buf-3]<<8)|  \
						   (buf[len_of_buf-4]))

/* This Macro will byte1 and byte2 in the buffer
 * which is length. byte0 is the command */
#define MASK_LEN_BYTES(buf) ((buf[2]<<8)| \
		   	   	   	   	   	 (buf[1]))

#define FLASH_APPL_START_ADDR 		0x08004000U
#define FLASH_APPL_PAGES			128   // each page is 128bytes

/* Commands Received from the SerialPort */
#define BL_GET_VER 			0xB1
#define BL_GET_CID 			0xB2
#define BL_ERASE_FLASH 		0xB3
#define BL_FLASH_APPL		0xB4
#define BL_INTEG_CHECK		0xB5
#define BL_JUMP_APPL		0xB6
#define	BL_TEST				0xB7

/* Each Command is followed by some other messages after acknowledgment
 * So with the command itself we receive the length. We will wait for this
 * length number of bytes from UART now */
#define BL_GET_VER_LEN 				5
#define BL_GET_CID_LEN 				5
#define BL_ERASE_FLASH_LEN 		 	5
#define BL_TEST_LEN					5
#define BL_RX_PACK_INFO_LEN			7

/* Acknowledgment sent back after executing commands */
#define BL_ACK				0xA5
#define BL_NACK 			0xA6
#define BL_ERASE_FLASH_SUCCESS		0xC1
#define BL_ERASE_FLASH_FAILURE		0xC2


/* Size of Each Binary packet shall be defined here and same in the sender
 * python script. Used to flash application */
#define BL_RX_BIN_PKT_SIZE			260

/* Standard definitions */
#define TRUE 				1
#define FALSE 				0

/* Global Variable Decleration*/
GPIO_PinState Jump_App = GPIO_PIN_RESET;	/* Jump to application when pin is set. read the code to understand */
HAL_StatusTypeDef status;					/* Status to return */
uint8_t uart_databuf[30];					/* Buffer for UART to receive data */
uint32_t received_crc 	= 0;				/* Holds Masked CRC */
uint32_t calculated_crc = 0;				/* Holds CRC calculated from internal CRC engine*/

/* Function Declerations*/
void Bootloader_jump_to_app(void);
void bootloader_uart_read_data(void);
uint8_t bootloader_verify_crc(uint8_t * uart_databuf, uint32_t length);
void bootloader_getcid(void);
void bootloader_getver(uint8_t * uart_databuf);
void bootloader_erase_appl_flash(uint8_t * uart_databuf);
void bootloader_flash_appl(uint8_t * uart_databuf);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); /* First we set the pin to high and check if still pin is high*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
 {
   /* USER CODE END WHILE */
	  Jump_App = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	  if(GPIO_PIN_SET == Jump_App)
	  {
		  /* to exit boot loader and jump to application GPIO_PIN_11 shall be connected to ground*/
		  Bootloader_jump_to_app();
		  break; /* application will never return here */
	  }
	  else
	  {
		  /* User Explicitly requested to stay in boot loader
		   * Just print and wait for command*/
		  HAL_UART_Transmit(&huart2, (uint8_t *)"In BootLoader\r\n", 15 , 300);
		  while(1){
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

/*  BRIEF\
 *  This Function reads data from UART and checks the command.
 *  Calls the respective interface to process accordingly
 *
 *	CALL FREQUENCY\
 *	Called in a while Loop
 *
 *	TODO
 *	BL_INTEG_CHECK has to be implemented
 *
 */
void bootloader_uart_read_data(void)
{
	uint8_t command  = 0;
	status = HAL_UART_Receive(&huart2,(uint8_t*) uart_databuf, 5, 5000);
	if (status == HAL_OK)
	{
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
		case BL_FLASH_APPL:
			bootloader_flash_appl(uart_databuf);
			break;
		case BL_JUMP_APPL:
			Bootloader_jump_to_app();
			break;
		default:
			HAL_UART_Transmit(&huart2,(uint8_t *) "Invalid Command\r\n", 17, HAL_MAX_DELAY);
		}
	}
	else
	{
		/* Do Nothing */
	}
}

/*  BRIEF\
 *  This Function is called when there is a request to flash the application
 *  process ->  send ACK if CRC is valid
 *          ->  Mask the length -  Number of upcoming packets in this connection
 *          ->  Unlock the flash memory first -  else we cannot write to pflash
 *          ->  Loop for masked length times and do the below
 *              ->  Receive Packet size
 *              ->  wait for the received size number of bytes from uart.
 *              ->  After the packet is received and CRC found valid
 *              	-> write the bytes to memory and return ACK
 *              ->  If data not received or CRC error return NACK
 *
 *
 *	CALL FREQUENCY\
 *	Called once BL_FLASH_APPL command is received
 *
 *	TODO
 *	Once the wrong CRC is received in b/w the communication then WHAT???
 *
 */
void bootloader_flash_appl(uint8_t * uart_databuf)
{
	uint8_t ack = BL_NACK;
	uint8_t bl_bin_pkt[260] = {0};
	uint32_t *flash_addr_appl_end=(uint32_t *)FLASH_APPL_START_ADDR;
	uint32_t opcode = 0;
	uint16_t app_max_num_pkts = 0;
	uint16_t app_current_pkt_len = 0;
	if(  bootloader_verify_crc(uart_databuf, BL_GET_VER_LEN)  )
	{
		ack = BL_ACK;
		HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
		status = HAL_UART_Receive(&huart2,(uint8_t*) uart_databuf, BL_RX_PACK_INFO_LEN, HAL_MAX_DELAY);
		if(  bootloader_verify_crc(uart_databuf, BL_RX_PACK_INFO_LEN)  )
		{
			ack = BL_ACK;
			HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
			app_max_num_pkts = MASK_LEN_BYTES(uart_databuf);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			HAL_FLASH_Unlock();
			for (uint16_t pkt_num = 0; pkt_num < app_max_num_pkts; pkt_num++)
			{
				status = HAL_UART_Receive(&huart2,(uint8_t*) uart_databuf, BL_RX_PACK_INFO_LEN, HAL_MAX_DELAY);
				if(  bootloader_verify_crc(uart_databuf, BL_RX_PACK_INFO_LEN)  )
				{
					ack = BL_ACK;
					HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);

					app_current_pkt_len = MASK_LEN_BYTES(uart_databuf);
					status = HAL_UART_Receive(&huart2,(uint8_t*) bl_bin_pkt, app_current_pkt_len, HAL_MAX_DELAY);
					if(  bootloader_verify_crc(bl_bin_pkt, app_current_pkt_len)  )
					{

						for (int i = 0; i < BL_RX_BIN_PKT_SIZE - 4; i+=4)
						{
							opcode = (bl_bin_pkt[i+3] << 24) | (bl_bin_pkt[i+2] << 16) | (bl_bin_pkt[i+1] << 8) | (bl_bin_pkt[i]);
							*flash_addr_appl_end = opcode;
							flash_addr_appl_end++;
						}

						ack = BL_ACK;
						HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
					}
					else
					{
						ack = BL_NACK;
						HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
					}
				}
				else
				{
					ack = BL_NACK;
					HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
				}
			}
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			HAL_FLASH_Lock();
		}
		else
		{
			/* Invalid CRC received for LEngth */
			ack = BL_NACK;
			HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);
		}
	}
	else
	{
		/* Invalid CRC received for the command */
		ack = BL_NACK;
		HAL_UART_Transmit(&huart2, &ack, 1, HAL_MAX_DELAY);
	}
}

/*  BRIEF\
 *  This Function erases the flash
 *  from FLASH_APPL_START_ADDR
 *  up to FLASH_APPL_PAGES
 *
 *
 *	CALL FREQUENCY\
 *	Called once BL_ERASE_FLASH command is received
 *
 *	TODO
 *	1. Configuration of page erase and size shall be taken through parameters
 *	2. HAL_MAX_DELAY is used while transmission
 *
 */
void bootloader_erase_appl_flash(uint8_t * uart_databuf)
{
	uint8_t ack = BL_NACK;
	uint8_t sts = BL_ERASE_FLASH_FAILURE;
	/* Proceed to erase only of CRC is Valid */
	if(  bootloader_verify_crc(uart_databuf, BL_GET_VER_LEN)  )
	{
		ack = BL_ACK;
		HAL_UART_Transmit(&huart2,(uint8_t *) &ack, 1, 300);		/*Send ACK first indicating erase started*/
		// code to erase application
		FLASH_EraseInitTypeDef pEraseInit;
		uint32_t PageError;
		/* Fill the structure pEraseInit with required data and then start*/
		pEraseInit.TypeErase 	= FLASH_TYPEERASE_PAGES;
		pEraseInit.PageAddress 	= FLASH_APPL_START_ADDR;
		pEraseInit.NbPages		= FLASH_APPL_PAGES;

		HAL_FLASH_Unlock();											/* Unlock the flash before erase */
		HAL_FLASHEx_Erase(&pEraseInit, &PageError);					/* Call HAL erase interface */
		HAL_FLASH_Lock();											/* Lock it again */
		if (PageError == 0xFFFFFFFF)
		{	/*0xFFFFFFFF indicates erase is successful*/
			sts = BL_ERASE_FLASH_SUCCESS;
		}
		HAL_UART_Transmit(&huart2,(uint8_t *) &sts, 1, 300);
	}
	else
	{
		HAL_UART_Transmit(&huart2, &ack, 1, HAL_MAX_DELAY);
	}
}

/*  BRIEF\
 *  This Function returns the version of current Bootloader
 *
 *
 *	CALL FREQUENCY\
 *	Called once BL_GET_VER command is received
 *
 *	TODO
 *	Version is currently in a Global variable
 *	Instead ensure version is always at a particular address
 *
 */
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

/*  BRIEF\
 *  This Function returns the chip ID
 *
 *
 *	CALL FREQUENCY\
 *	Called once BL_GET_CID command is received
 *
 *	TODO
 *	Ensure the Chip ID is unique
 *
 */
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

/*  BRIEF\
 *  This Function verifies the received CRC against Calculated CRC
 *
 *
 *	CALL FREQUENCY\
 *	Called for every UART receive
 *
 *	TODO
 *
 *
 */
uint8_t bootloader_verify_crc(uint8_t * uart_databuf, uint32_t length)
{
	received_crc 	= 0;
	// MASK the received CRC. RECEIVED DATA IS ALWAYS MSB FIRST
	received_crc = MASK_CRC_BYTES(uart_databuf,length);
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
