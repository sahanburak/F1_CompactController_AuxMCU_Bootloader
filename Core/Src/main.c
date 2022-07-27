/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "io.h"
#include "rt_bus_proto.h"
#include "rt_init.h"
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
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern uint8_t gSPI_Tx_DMA_Buf[MAX_SPI_PACKET_SIZE];
extern uint8_t gSPI_Rx_DMA_Buf[MAX_SPI_PACKET_SIZE];
extern tPDO g_PDO;
extern tPDI g_PDI;
extern eLEDFlashMode g_led_flash_mode;

extern volatile uint32_t iapMailbox[2];
extern const uint8_t appsha[20];
extern tRT_FW_Update_Command_Packet sRT_FW_Update_Command_Packet;

uint16_t currentDMA = MAX_SPI_PACKET_SIZE;
uint8_t gRunMode = SWLEVEL_NONE;
uint8_t gStayInBootloader = 0;
uint8_t g_comms_mode = COMMS_MODE_PDIO;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void comms_set_mode();
void comms_init();
void bootloader(void);
void normalboot(void);
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
	uint8_t appStatus = 0;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	rt_get_app_info();
	rt_init();
	/* Infinite loop */

	// Check application validity (reset vector location, sha etc)
	// if valid
	// 		if iapMailbox is matches with magic number
	//			stay in bootloader mode
	// 		else
	//			jump to application after clearing iapMailbox
	// else
	//			stay in bootloader mode
	dbprintf("Checking....");
	appStatus = iap_CheckApplication();  /*  App requested to enter bootloader */
	if (appStatus == IAP_APP_SUCCESS)
	{
		if ((iapMailbox[0] == MAGIC_1) && (iapMailbox[1] == MAGIC_2))
		{
			dbprintf("Magic !!!");
			iapMailbox[0] = 0;
			iapMailbox[1] = 0;
			bootloader();
		}
		else
		{
			normalboot();
		}
	}
	bootloader();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Set_Comms_Mode()
{
	if (STM_MODE_SEL_GPIO_Port->IDR & STM_MODE_SEL_Pin){
		g_comms_mode = COMMS_MODE_CONF;
		dbprintf("Conf Mode");
		ERR_LED1_On();
		g_led_flash_mode = LED_MODE_HS_FLASH;
	}else{
		g_comms_mode = COMMS_MODE_PDIO;
		dbprintf("PDIO Mode");
		ERR_LED1_Off();
		g_led_flash_mode = LED_MODE_MS_FLASH;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Set_Comms_Mode();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	rt_bus_proto_bl_dt();
}

void rt_status_led_blink(void){
	static uint32_t led_blink_counter= 0;

	switch(g_led_flash_mode){
		case LED_MODE_NONE:
			ERR_LED2_Off();
			break;
		case LED_MODE_LS_FLASH:
		case LED_MODE_MS_FLASH:
		case LED_MODE_HS_FLASH:
			if((++led_blink_counter) == g_led_flash_mode){
							ERR_LED2_Toggle();
							led_blink_counter = 0;
			}
			break;
		default:
			break;
	}
}


void bootloader(void)
{
	gRunMode = SWLEVEL_BOOTLOADER;
	g_led_flash_mode = LED_MODE_MS_FLASH;
	while (1)
	{
		rt_status_led_blink();
	}
}


void normalboot(void)
{
	dbprintf("Application starting...");
	gRunMode = SWLEVEL_JUMPING;
	/* For normal boot clear mailbox and jumping to application */
	iapMailbox[0] = 0;
	iapMailbox[1] = 0;
	iap_JumpToApplication();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
