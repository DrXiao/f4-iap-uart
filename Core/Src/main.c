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
#include "fatfs.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iap-uart.h"
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

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef
    DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef
    DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

HCD_HandleTypeDef hhcd_USB_OTG_FS;
HCD_HandleTypeDef hhcd_USB_OTG_HS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_USB_OTG_HS_HCD_Init(void);
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
        /* USER CODE BEGIN 1 */

        /* USER CODE END 1 */

        /* MCU
         * Configuration--------------------------------------------------------*/

        /* Reset of all peripherals, Initializes the Flash interface and the
         * Systick. */
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
        MX_CAN1_Init();
        MX_CAN2_Init();
        MX_ETH_Init();
        MX_I2C1_Init();
        MX_SDIO_SD_Init();
        MX_SPI2_Init();
        MX_USART1_UART_Init();
        MX_USART2_UART_Init();
        MX_USB_OTG_FS_HCD_Init();
        MX_USB_OTG_HS_HCD_Init();
        /* USER CODE BEGIN 2 */
        extern volatile bool iap_uart_recv_flag;
        extern volatile bool iap_uart_send_flag;
        extern volatile uint32_t iap_uart_recv_remaining_cnt;

        iap_uart_config_t iap_uart_config = {
            .huart = &huart1,
            .recv_flag = &iap_uart_recv_flag,
            .send_flag = &iap_uart_send_flag,
            .recv_remaining_cnt = &iap_uart_recv_remaining_cnt};

        chip_config_t chip_config = {.chip_family = CHIP_FAMILY_F40x,
                                     .chip_flash_addr = 0x8000000,
                                     .chip_flash_size = 0x80000,
                                     .chip_ram_addr = 0x20000000,
                                     .chip_ram_size = 0x20000};

        iap_t iap;
        iap_init(&iap, &iap_uart_config, &chip_config);

        iap.iap_proc(&iap);
        /* USER CODE END 2 */

        /* Infinite loop */
        /* USER CODE BEGIN WHILE */
        while (1) {
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

        /** Configure the main internal regulator output voltage
         */
        __HAL_RCC_PWR_CLK_ENABLE();
        __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

        /** Initializes the RCC Oscillators according to the specified
         * parameters in the RCC_OscInitTypeDef structure.
         */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 8;
        RCC_OscInitStruct.PLL.PLLN = 336;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = 7;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
                Error_Handler();
        }

        /** Initializes the CPU, AHB and APB buses clocks
         */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                      RCC_CLOCKTYPE_SYSCLK |
                                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) !=
            HAL_OK) {
                Error_Handler();
        }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{
        /* USER CODE BEGIN CAN1_Init 0 */

        /* USER CODE END CAN1_Init 0 */

        /* USER CODE BEGIN CAN1_Init 1 */

        /* USER CODE END CAN1_Init 1 */
        hcan1.Instance = CAN1;
        hcan1.Init.Prescaler = 3;
        hcan1.Init.Mode = CAN_MODE_NORMAL;
        hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
        hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
        hcan1.Init.TimeTriggeredMode = DISABLE;
        hcan1.Init.AutoBusOff = DISABLE;
        hcan1.Init.AutoWakeUp = DISABLE;
        hcan1.Init.AutoRetransmission = DISABLE;
        hcan1.Init.ReceiveFifoLocked = DISABLE;
        hcan1.Init.TransmitFifoPriority = DISABLE;
        if (HAL_CAN_Init(&hcan1) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN CAN1_Init 2 */

        /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{
        /* USER CODE BEGIN CAN2_Init 0 */

        /* USER CODE END CAN2_Init 0 */

        /* USER CODE BEGIN CAN2_Init 1 */

        /* USER CODE END CAN2_Init 1 */
        hcan2.Instance = CAN2;
        hcan2.Init.Prescaler = 16;
        hcan2.Init.Mode = CAN_MODE_NORMAL;
        hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
        hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
        hcan2.Init.TimeTriggeredMode = DISABLE;
        hcan2.Init.AutoBusOff = DISABLE;
        hcan2.Init.AutoWakeUp = DISABLE;
        hcan2.Init.AutoRetransmission = DISABLE;
        hcan2.Init.ReceiveFifoLocked = DISABLE;
        hcan2.Init.TransmitFifoPriority = DISABLE;
        if (HAL_CAN_Init(&hcan2) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN CAN2_Init 2 */

        /* USER CODE END CAN2_Init 2 */
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void)
{
        /* USER CODE BEGIN ETH_Init 0 */

        /* USER CODE END ETH_Init 0 */

        static uint8_t MACAddr[6];

        /* USER CODE BEGIN ETH_Init 1 */

        /* USER CODE END ETH_Init 1 */
        heth.Instance = ETH;
        MACAddr[0] = 0x00;
        MACAddr[1] = 0x80;
        MACAddr[2] = 0xE1;
        MACAddr[3] = 0x00;
        MACAddr[4] = 0x00;
        MACAddr[5] = 0x00;
        heth.Init.MACAddr = &MACAddr[0];
        heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
        heth.Init.TxDesc = DMATxDscrTab;
        heth.Init.RxDesc = DMARxDscrTab;
        heth.Init.RxBuffLen = 1524;

        /* USER CODE BEGIN MACADDRESS */

        /* USER CODE END MACADDRESS */

        if (HAL_ETH_Init(&heth) != HAL_OK) {
                Error_Handler();
        }

        memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
        TxConfig.Attributes =
            ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
        TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
        TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
        /* USER CODE BEGIN ETH_Init 2 */

        /* USER CODE END ETH_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
        /* USER CODE BEGIN I2C1_Init 0 */

        /* USER CODE END I2C1_Init 0 */

        /* USER CODE BEGIN I2C1_Init 1 */

        /* USER CODE END I2C1_Init 1 */
        hi2c1.Instance = I2C1;
        hi2c1.Init.ClockSpeed = 100000;
        hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
        hi2c1.Init.OwnAddress1 = 0;
        hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hi2c1.Init.OwnAddress2 = 0;
        hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
        if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN I2C1_Init 2 */

        /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{
        /* USER CODE BEGIN SDIO_Init 0 */

        /* USER CODE END SDIO_Init 0 */

        /* USER CODE BEGIN SDIO_Init 1 */

        /* USER CODE END SDIO_Init 1 */
        hsd.Instance = SDIO;
        hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
        hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
        hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
        hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
        hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
        hsd.Init.ClockDiv = 118;
        /* USER CODE BEGIN SDIO_Init 2 */

        /* USER CODE END SDIO_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{
        /* USER CODE BEGIN SPI2_Init 0 */

        /* USER CODE END SPI2_Init 0 */

        /* USER CODE BEGIN SPI2_Init 1 */

        /* USER CODE END SPI2_Init 1 */
        /* SPI2 parameter configuration*/
        hspi2.Instance = SPI2;
        hspi2.Init.Mode = SPI_MODE_MASTER;
        hspi2.Init.Direction = SPI_DIRECTION_2LINES;
        hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi2.Init.NSS = SPI_NSS_SOFT;
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
        hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hspi2.Init.CRCPolynomial = 10;
        if (HAL_SPI_Init(&hspi2) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN SPI2_Init 2 */

        /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
        /* USER CODE BEGIN USART1_Init 0 */

        /* USER CODE END USART1_Init 0 */

        /* USER CODE BEGIN USART1_Init 1 */

        /* USER CODE END USART1_Init 1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        if (HAL_UART_Init(&huart1) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN USART1_Init 2 */

        /* USER CODE END USART1_Init 2 */
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
        if (HAL_UART_Init(&huart2) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN USART2_Init 2 */

        /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_HCD_Init(void)
{
        /* USER CODE BEGIN USB_OTG_FS_Init 0 */

        /* USER CODE END USB_OTG_FS_Init 0 */

        /* USER CODE BEGIN USB_OTG_FS_Init 1 */

        /* USER CODE END USB_OTG_FS_Init 1 */
        hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
        hhcd_USB_OTG_FS.Init.Host_channels = 8;
        hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
        hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
        hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
        hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
        if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN USB_OTG_FS_Init 2 */

        /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
 * @brief USB_OTG_HS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_HS_HCD_Init(void)
{
        /* USER CODE BEGIN USB_OTG_HS_Init 0 */

        /* USER CODE END USB_OTG_HS_Init 0 */

        /* USER CODE BEGIN USB_OTG_HS_Init 1 */

        /* USER CODE END USB_OTG_HS_Init 1 */
        hhcd_USB_OTG_HS.Instance = USB_OTG_HS;
        hhcd_USB_OTG_HS.Init.Host_channels = 12;
        hhcd_USB_OTG_HS.Init.speed = HCD_SPEED_FULL;
        hhcd_USB_OTG_HS.Init.dma_enable = DISABLE;
        hhcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
        hhcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
        hhcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
        hhcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
        hhcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
        if (HAL_HCD_Init(&hhcd_USB_OTG_HS) != HAL_OK) {
                Error_Handler();
        }
        /* USER CODE BEGIN USB_OTG_HS_Init 2 */

        /* USER CODE END USB_OTG_HS_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
        /* DMA controller clock enable */
        __HAL_RCC_DMA2_CLK_ENABLE();

        /* DMA interrupt init */
        /* DMA2_Stream2_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
        /* DMA2_Stream3_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
        /* DMA2_Stream7_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
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
        __HAL_RCC_GPIOH_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOE,
                          GPIO_PIN_3 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_13 |
                              GPIO_PIN_14 | GPIO_PIN_15,
                          GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

        /*Configure GPIO pins : PE3 PE7 PE8 PE13
                                 PE14 PE15 */
        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_7 | GPIO_PIN_8 |
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /*Configure GPIO pin : PB1 */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pin : PE9 */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /*Configure GPIO pins : PE10 PE11 PE12 */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /*Configure GPIO pin : PA8 */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*Configure GPIO pin : PD3 */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /*Configure GPIO pin : PD7 */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
        /* User can add his own implementation to report the HAL error return
         * state */
        __disable_irq();
        while (1) {
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
        /* User can add his own implementation to report the file name and line
           number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
           file, line) */
        /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
