/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32 Bootloader (Refactored)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "etx_ota_update.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAM_START  0x20000000
#define RAM_END    0x20030000 // F407 (192KB RAM) -> 0x20030000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAJOR 0
#define MINOR 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const uint8_t BL_Version[2] = {MAJOR, MINOR};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
static void goto_application(void);
static void Print_Banner(void);
static void Check_Boot_Reason(bool *goto_ota, bool *is_factory_reset);
static bool Check_Hardware_Trigger(void);
static void Enter_OTA_Mode(void);
static void Enter_App_Mode(bool is_factory_reset);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
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
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init(); // Debug
  MX_USART2_UART_Init(); // ESP32
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */

  Print_Banner();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // LED ON indicating Bootloader
  bool goto_ota_mode = false;
  bool is_factory_reset = false;

  Check_Boot_Reason(&goto_ota_mode, &is_factory_reset);

  if (!goto_ota_mode)
  {
      if (Check_Hardware_Trigger())
      {
          goto_ota_mode = true;
      }
  }

  if (goto_ota_mode)
  {
      Enter_OTA_Mode();
  }
  else
  {
      Enter_App_Mode(is_factory_reset);
  }

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

/* ============================================================================
   HELPER FUNCTIONS IMPLEMENTATION
   ============================================================================ */

static void Print_Banner(void)
{
    printf("\r\n=========================================\r\n");
    printf("Starting Bootloader (v%d.%d)\r\n", BL_Version[0], BL_Version[1]);
    printf("=========================================\r\n");
}

static void Check_Boot_Reason(bool *goto_ota, bool *is_factory_reset)
{
    ETX_GNRL_CFG_ *cfg = (ETX_GNRL_CFG_*) (ETX_CONFIG_FLASH_ADDR);

    printf("Reading Reboot Cause at 0x%X: 0x%lX\r\n", ETX_CONFIG_FLASH_ADDR, cfg->reboot_cause);

    if (cfg->reboot_cause == 0xFFFFFFFF)
    {
        // --- Trường hợp 1: Config bị trống (Chip mới hoặc vừa nạp dây) ---
        printf("Config is Empty (0xFFFFFFFF). Checking for Factory App...\r\n");

        // Kiểm tra tính hợp lệ của App qua MSP
        uint32_t app_msp = *(__IO uint32_t*)ETX_APP_FLASH_ADDR;

        if (app_msp >= RAM_START && app_msp <= RAM_END)
        {
            printf(">> Valid App found at 0x%X (MSP: 0x%lX).\r\n", ETX_APP_FLASH_ADDR, app_msp);
            printf(">> Assuming Factory Flashed Device. Skipping OTA.\r\n");
            *is_factory_reset = true;
            *goto_ota = false;
        }
        else
        {
            printf(">> No Valid App found (MSP: 0x%lX). Flash is empty.\r\n", app_msp);
            printf(">> Forcing OTA Mode (Rescue).\r\n");
            *goto_ota = true;
        }
    }
    else
    {
        // --- Trường hợp 2: Đã có Config ---
        switch( cfg->reboot_cause )
        {
            case ETX_NORMAL_BOOT:
                printf("Cause: Normal Boot\r\n");
                *goto_ota = false;
                break;
            case ETX_OTA_REQUEST:
                printf("Cause: OTA Request from App\r\n");
                *goto_ota = true;
                break;
            case ETX_FIRST_TIME_BOOT:
                printf("Cause: First Time Boot Flag\r\n");
                *goto_ota = true;
                break;
            default:
                printf("Cause: Unknown (0x%lX), assuming Normal Boot\r\n", cfg->reboot_cause);
                *goto_ota = false;
                break;
        };
    }
}

static bool Check_Hardware_Trigger(void)
{
    printf("Press KEY0 (PE4) within 1s to Force OTA...\r\n");
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < 1000)
    {
        // Nút bấm Active LOW (Nhấn = 0)
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET)
        {
            printf(">> Button Pressed! Forcing OTA Mode.\r\n");
            return true;
        }
    }
    return false;
}

static void Enter_OTA_Mode(void)
{
    printf(">>> ENTERING OTA MODE <<<\r\n");


    while(1)
    {
        // Gọi hàm nhận file từ thư viện OTA
        ETX_OTA_EX_ status = etx_ota_download_and_flash();

        if( status == ETX_OTA_EX_OK )
        {
            printf("OTA Update SUCCESS! System Resetting...\r\n");
            HAL_Delay(500);
            HAL_NVIC_SystemReset();
        }
        else
        {
            printf("OTA Failed/Timeout. Retrying...\r\n");
            HAL_Delay(1000);
        }
    }
}

static void Enter_App_Mode(bool is_factory_reset)
{
    if (is_factory_reset)
    {
        printf("Factory Reset detected. Skipping Load/Verify Logic.\r\n");
    }
    else
    {
        printf("Checking for pending updates...\r\n");
        // Hàm này kiểm tra Slot mới, Copy, Verify CRC, và Rollback nếu lỗi
        load_new_app();
    }

    // 2. Nhảy vào App
    goto_application();
}

/* SYSTEM CONFIGURATION */

static void goto_application(void)
{
    printf("Jumping to Application at 0x%X...\r\n", ETX_APP_FLASH_ADDR);

    uint32_t main_stack_pointer = *(__IO uint32_t*)ETX_APP_FLASH_ADDR;
    uint32_t reset_handler_addr = *(__IO uint32_t*)(ETX_APP_FLASH_ADDR + 4);

    pFunction jump_to_app = (pFunction)reset_handler_addr;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // LED OFF

    // De-init Peripherals
    HAL_CRC_DeInit(&hcrc);
    HAL_UART_DeInit(&huart2);
    HAL_UART_DeInit(&huart3);
    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    __set_MSP(main_stack_pointer);

    jump_to_app();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_USART3_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
