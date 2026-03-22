/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float position;
    float speed;
} MotorCmd;

typedef enum {
    DEV1 = 1,   // PE2
    DEV2 = 2,   // PE4
    DEV3 = 3,   // PE5
    DEV4 = 4    // PE6
} SpiDevId;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SLAVES            4
#define BYTES_PER_MOTOR       4
#define MAX_TOTAL_MOTORS      16
#define MAX_TXRX_BYTES        (MAX_TOTAL_MOTORS * BYTES_PER_MOTOR)

// --- Constants (DO NOT CHANGE) ---
#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -20.0f
#define V_MAX    20.0f
#define KP_MIN    0.0f
#define KP_MAX 5000.0f
#define KD_MIN    0.0f
#define KD_MAX  100.0f
#define T_MIN   -60.0f
#define T_MAX    60.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* Configure how many motors belong to each slave.
   Example:
   {1,1,1,1} => DEV1 gets motor0, DEV2 gets motor1, DEV3 gets motor2, DEV4 gets motor3
   {2,1,0,1} => DEV1 gets motor0-1, DEV2 gets motor2, DEV3 gets none, DEV4 gets motor3
*/
static const uint8_t motors_per_slave[NUM_SLAVES] = {1, 1, 1, 1};

static MotorCmd g_motor_cmd[MAX_TOTAL_MOTORS] = {
    {0.0f, 1.0f},
    {0.0f, 0.0f},
    {0.0f, 3.0f},
    {0.0f, 4.0f}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void uart_printf(const char *fmt, ...);
static void uart_dump_bytes(const char *tag, const uint8_t *buf, int n);
static inline void CS_ALL_HIGH(void);
static inline void CS_SELECT(SpiDevId dev);
static uint16_t get_total_configured_motors(void);
static void pack_one_motor(uint8_t *out, const MotorCmd *cmd);
static uint16_t build_slave_buf(uint8_t slave_index, uint8_t *tx_buf, const MotorCmd *all_motor_cmd);
static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev, uint8_t slave_index, const MotorCmd *all_motor_cmd);
static HAL_StatusTypeDef spi_update_all_slaves_param(const MotorCmd *all_motor_cmd);
static void uart_print_one_motor_rx(uint16_t motor_index, const uint8_t *rx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)((1U << bits) - 1U) / span));
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1U << bits) - 1U)) + offset;
}

static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

    HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
}

static void uart_dump_bytes(const char *tag, const uint8_t *buf, int n)
{
    uart_printf("%s", tag);
    for (int i = 0; i < n; i++) {
        uart_printf(" %02X", buf[i]);
    }
    uart_printf("\r\n");
}

static inline void CS_ALL_HIGH(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);
}

static inline void CS_SELECT(SpiDevId dev)
{
    CS_ALL_HIGH();
    switch (dev) {
        case DEV1: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); break;
        case DEV2: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); break;
        case DEV3: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); break;
        case DEV4: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); break;
        default: break;
    }
}

static uint16_t get_total_configured_motors(void)
{
    uint16_t total = 0;
    for (int i = 0; i < NUM_SLAVES; i++) {
        total += motors_per_slave[i];
    }
    return total;
}

static void pack_one_motor(uint8_t *out, const MotorCmd *cmd)
{
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed, V_MIN, V_MAX, 16);

    out[0] = (uint8_t)(pos_u16 & 0xFF);        // POS LSB
    out[1] = (uint8_t)((pos_u16 >> 8) & 0xFF); // POS MSB
    out[2] = (uint8_t)(spd_u16 & 0xFF);        // SPD LSB
    out[3] = (uint8_t)((spd_u16 >> 8) & 0xFF); // SPD MSB
}

static void uart_print_one_motor_encoding(uint16_t motor_index, const MotorCmd *cmd)
{
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed,    V_MIN, V_MAX, 16);

    uart_printf("[MOTOR %d] float: pos=%.3f spd=%.3f\r\n",
                (int)motor_index,
                cmd->position,
                cmd->speed);

    uart_printf("[MOTOR %d] uint16: pos=%u (0x%04X) spd=%u (0x%04X)\r\n",
                (int)motor_index,
                pos_u16, pos_u16,
                spd_u16, spd_u16);

    uart_printf("[MOTOR %d] bytes: %02X %02X %02X %02X\r\n",
                (int)motor_index,
                (uint8_t)(pos_u16 & 0xFF),
                (uint8_t)((pos_u16 >> 8) & 0xFF),
                (uint8_t)(spd_u16 & 0xFF),
                (uint8_t)((spd_u16 >> 8) & 0xFF));
}

static void uart_print_one_motor_rx(uint16_t motor_index, const uint8_t *rx)
{
    uint16_t pos_u16 = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    uint16_t spd_u16 = (uint16_t)rx[2] | ((uint16_t)rx[3] << 8);

    float pos_f = uint_to_float((int)pos_u16, P_MIN, P_MAX, 16);
    float spd_f = uint_to_float((int)spd_u16, V_MIN, V_MAX, 16);

    uart_printf("[RX MOTOR %d] bytes : %02X %02X %02X %02X\r\n",
                (int)motor_index,
                rx[0], rx[1], rx[2], rx[3]);

    uart_printf("[RX MOTOR %d] uint16: pos=%u (0x%04X) spd=%u (0x%04X)\r\n",
                (int)motor_index,
                pos_u16, pos_u16,
                spd_u16, spd_u16);

    uart_printf("[RX MOTOR %d] float : pos=%.3f spd=%.3f\r\n",
                (int)motor_index,
                pos_f,
                spd_f);
}

static uint16_t build_slave_buf(uint8_t slave_index,
                                uint8_t *tx_buf,
                                const MotorCmd *all_motor_cmd)
{
    uint16_t motor_start = 0;
    uint16_t motor_count = motors_per_slave[slave_index];

    for (int i = 0; i < slave_index; i++) {
        motor_start += motors_per_slave[i];
    }

    for (uint16_t m = 0; m < motor_count; m++) {
        pack_one_motor(&tx_buf[m * BYTES_PER_MOTOR], &all_motor_cmd[motor_start + m]);
    }

    return motor_count * BYTES_PER_MOTOR;
}

static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev,
                                               uint8_t slave_index,
                                               const MotorCmd *all_motor_cmd)
{
    uint8_t tx_buf[MAX_TXRX_BYTES] = {0};
    uint8_t rx_buf[MAX_TXRX_BYTES] = {0};

    if (dev < DEV1 || dev > DEV4) return HAL_ERROR;
    if (slave_index >= NUM_SLAVES) return HAL_ERROR;

    uint16_t tx_len = build_slave_buf(slave_index, tx_buf, all_motor_cmd);
    if (tx_len == 0) {
        uart_printf("[SPI] dev=%d skip (0 motor)\r\n", (int)dev);
        return HAL_OK;
    }

    uart_printf("[SPI] start dev=%d, motors=%d, bytes=%d\r\n",
                (int)dev,
                (int)motors_per_slave[slave_index],
                (int)tx_len);

    uint16_t motor_start = 0;
    for (int i = 0; i < slave_index; i++) {
        motor_start += motors_per_slave[i];
    }

    for (uint16_t m = 0; m < motors_per_slave[slave_index]; m++) {
        uart_print_one_motor_encoding(motor_start + m, &all_motor_cmd[motor_start + m]);
    }

    uart_dump_bytes("[SPI] TX:", tx_buf, tx_len);

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1,
                                                   tx_buf,
                                                   rx_buf,
                                                   tx_len,
                                                   HAL_MAX_DELAY);
    CS_ALL_HIGH();

    if (st == HAL_OK) {
        uart_dump_bytes("[SPI] RX:", rx_buf, tx_len);

        uint16_t motor_start = 0;
        for (int i = 0; i < slave_index; i++) {
            motor_start += motors_per_slave[i];
        }

        for (uint16_t m = 0; m < motors_per_slave[slave_index]; m++) {
            uart_print_one_motor_rx(motor_start + m, &rx_buf[m * BYTES_PER_MOTOR]);
        }

        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    } else {
        uart_printf("[SPI] dev=%d transmit error\r\n", (int)dev);
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }

    return st;
}

static HAL_StatusTypeDef spi_update_all_slaves_param(const MotorCmd *all_motor_cmd)
{
    HAL_StatusTypeDef st;

    st = spi_send_to_one_slave(DEV1, 0, all_motor_cmd);
    if (st != HAL_OK) return st;

    st = spi_send_to_one_slave(DEV2, 1, all_motor_cmd);
    if (st != HAL_OK) return st;

    st = spi_send_to_one_slave(DEV3, 2, all_motor_cmd);
    if (st != HAL_OK) return st;

    st = spi_send_to_one_slave(DEV4, 3, all_motor_cmd);
    if (st != HAL_OK) return st;

    return HAL_OK;
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uart_printf("\r\n=== SPI parameterized motor demo ===\r\n");
  uart_printf("Configured motors total = %d\r\n", (int)get_total_configured_motors());
  float dir = 1.0;
  float step = 0.1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    g_motor_cmd[0].position += step * dir;
    g_motor_cmd[0].speed    = 5.0f;

    if((g_motor_cmd[0].position >= 12.0f)||(g_motor_cmd[0].position <= -12.0f)){
    	dir *= -1.0f;
    }


//    g_motor_cmd[1].position += 0.00f;
//    g_motor_cmd[1].speed     = 0.0f;
//
//    g_motor_cmd[2].position += 0.30f;
//    g_motor_cmd[2].speed     = 3.0f;
//
//    g_motor_cmd[3].position += 0.40f;
//    g_motor_cmd[3].speed     = 4.0f;

    uart_printf("[CMD] M0 pos=%.2f spd=%.2f\r\n", g_motor_cmd[0].position, g_motor_cmd[0].speed);
//    uart_printf("[CMD] M1 pos=%.2f spd=%.2f\r\n", g_motor_cmd[1].position, g_motor_cmd[1].speed);
//    uart_printf("[CMD] M2 pos=%.2f spd=%.2f\r\n", g_motor_cmd[2].position, g_motor_cmd[2].speed);
//    uart_printf("[CMD] M3 pos=%.2f spd=%.2f\r\n", g_motor_cmd[3].position, g_motor_cmd[3].speed);

    spi_send_to_one_slave(DEV1, 0, g_motor_cmd);

    HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
