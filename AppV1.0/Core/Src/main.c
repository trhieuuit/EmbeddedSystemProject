/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Application (Refactored & CubeMX Friendly)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "etx_ota_update.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAJOR 1	 //App major version number
#define MINOR 9  //App minor version number
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const uint8_t App_Version[2] = {MAJOR,MINOR};
uint8_t rx_buf[4];

// Cờ báo hiệu (Flag) để chuyển xử lý từ Ngắt ra Main loop (Safe Mode)
volatile bool flag_ota_update_req = false;
volatile bool flag_system_reset_req = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef write_cfg_to_flash( ETX_GNRL_CFG_ *cfg );

// --- MODULES TÁCH RIÊNG ---
void OTA_Task_Handler(void);   // Module xử lý OTA/Reset
void App_Main_Task(void);      // Module ứng dụng chính (LED, Sensor...)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // 1. Nhận lệnh OTA
    if( !strncmp("ota", (char*)rx_buf, 3) )
    {
       //Bật cờ
       flag_ota_update_req = true;
    }
    // 2. Nhận lệnh Check Version
    else if ( !strncmp("ver", (char*)rx_buf, 3) )
    {
        char response[32];
        sprintf(response, "Ver:%d.%d\n", MAJOR, MINOR);
        HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
    }
    // 3. Nhận lệnh Reset
    else if ( !strncmp("rst", (char*)rx_buf, 3) )
	{
         flag_system_reset_req = true;
	}

    // Tiếp tục nhận dữ liệu mới
    HAL_UART_Receive_IT(&huart2, rx_buf, 3);

    // Xóa buffer an toàn
    memset(rx_buf, 0, sizeof(rx_buf));
  }
}

/**
  * @brief  Module 1: Xử lý các yêu cầu hệ thống (OTA, Reset)
  * Hàm này được gọi liên tục trong Main Loop.
  */
void OTA_Task_Handler(void)
{
    // --- XỬ LÝ LỆNH RESET ---
    if (flag_system_reset_req)
    {
        printf("[SYS] Processing Reset Request...\r\n");


        HAL_IWDG_Refresh(&hiwdg);

        char *ack = "rst:ok\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)ack, strlen(ack), 100);
        HAL_Delay(100);

        HAL_NVIC_SystemReset();
    }

    // --- XỬ LÝ LỆNH OTA UPDATE ---
    if (flag_ota_update_req)
    {
        printf("[SYS] Processing OTA Request...\r\n");
        HAL_IWDG_Refresh(&hiwdg);

        // Khóa ngắt để ghi Flash an toàn
        __disable_irq();

        ETX_GNRL_CFG_ cfg;
        memcpy( &cfg, (ETX_GNRL_CFG_*) (ETX_CONFIG_FLASH_ADDR), sizeof(ETX_GNRL_CFG_) );
        cfg.reboot_cause = ETX_OTA_REQUEST;

        if (write_cfg_to_flash( &cfg ) == HAL_OK)
        {
            HAL_NVIC_SystemReset();
        }
        else
        {
            __enable_irq(); // Mở lại ngắt nếu lỗi
            printf("[SYS] Config Write Error!\r\n");
            flag_ota_update_req = false; // Xóa cờ để chạy tiếp
        }
    }
}

/**
  * @brief  Module 2: Ứng dụng chính của người dùng
  */
void App_Main_Task(void)
{
    static uint32_t last_toggle = 0;

    if (HAL_GetTick() - last_toggle >= 500)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
        last_toggle = HAL_GetTick();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    // [BẮT BUỘC] Dời Vector Table về địa chỉ App
	SCB->VTOR = 0x08020000;
	__enable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting Application (%d.%d)\n", App_Version[0], App_Version[1]);
  HAL_UART_Receive_IT(&huart2, rx_buf, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      // 1. Chạy module quản lý hệ thống (OTA, Reset)
      OTA_Task_Handler();

      // 2. Chạy module ứng dụng
      App_Main_Task();

      // 3. Giám sát hệ thống (Watchdog Refresh)
      HAL_IWDG_Refresh(&hiwdg);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 3000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

static HAL_StatusTypeDef write_cfg_to_flash( ETX_GNRL_CFG_ *cfg )
{
  HAL_StatusTypeDef ret;
  do
  {
    if( cfg == NULL ) { ret = HAL_ERROR; break; }

    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK ) break;

    FLASH_WaitForLastOperation( HAL_MAX_DELAY );

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    // Đảm bảo SECTOR_4 đúng với địa chỉ Config của bạn
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector        = FLASH_SECTOR_4;
    EraseInitStruct.NbSectors     = 1;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

    ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
    if( ret != HAL_OK ) break;

    uint8_t *data = (uint8_t *) cfg;
    for( uint32_t i = 0u; i < sizeof(ETX_GNRL_CFG_); i++ )
    {
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE, ETX_CONFIG_FLASH_ADDR + i, data[i] );
      if( ret != HAL_OK ) break;
    }

    FLASH_WaitForLastOperation( HAL_MAX_DELAY );
    if( ret != HAL_OK ) break;

    ret = HAL_FLASH_Lock();
  }while( false );

  return ret;
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
