/**
  ******************************************************************************
  * @file    Wifi/WiFi_Client_Server/src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private defines -----------------------------------------------------------*/

#define TERMINAL_USE

/* Update SSID and PASSWORD with own Access point settings */
#define SSID     "Zhao"
#define PASSWORD "Zhao49800312"

uint8_t RemoteIP[] = {192,168,0,18};
#define RemotePORT	25550

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

/* Sensor data send interval in ms */
#define SENSOR_SEND_INTERVAL          100

#if defined (TERMINAL_USE)
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif

/*
 * LSM6DSL Significant Motion Detection - Register definitions
 * Reference: LSM6DSL datasheet (DocID029467) + AN5040
 *
 * Registers already defined in lsm6dsl.h (BSP driver):
 *   LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS  0x01
 *   LSM6DSL_ACC_GYRO_CTRL3_C          0x12
 *   LSM6DSL_ACC_GYRO_CTRL10_C         0x19
 *   LSM6DSL_ACC_GYRO_FUNC_SRC         0x53
 *   LSM6DSL_ACC_GYRO_TAP_CFG1         0x58
 *   LSM6DSL_ACC_GYRO_MD1_CFG          0x5E
 *   LSM6DSL_ACC_GYRO_SM_STEP_THS      0x13  (embedded bank)
 *
 * Bit definitions (from datasheet):
 */

/* FUNC_CFG_ACCESS (0x01) */
#define FUNC_CFG_EN                   0x80  /* Bit 7: enable embedded function bank */

/* CTRL10_C (0x19) */
#define FUNC_EN                       0x04  /* Bit 2: enable embedded functions */
#define SIGN_MOTION_EN                0x01  /* Bit 0: enable significant motion */
#define PEDO_EN                       0x10  /* Bit 4: enable pedometer (required for sig motion) */

/* TAP_CFG (0x58) */
#define INTERRUPTS_ENABLE             0x80  /* Bit 7: enable basic interrupts routing */
#define LIR                           0x01  /* Bit 0: latched interrupt */

/* MD1_CFG (0x5E) */
#define INT1_SIGN_MOT                 0x40  /* Bit 6: route significant motion to INT1 */

/* FUNC_SRC1 (0x53) */
#define SIGN_MOT_IA                   0x40  /* Bit 6: significant motion event detected */

/* LSM6DSL INT1 pin on B-L475E-IOT01A: PD11 (from board schematic UM2153) */
#define LSM6DSL_INT1_GPIO_PORT        GPIOD
#define LSM6DSL_INT1_GPIO_PIN         GPIO_PIN_11
#define LSM6DSL_INT1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define LSM6DSL_INT1_EXTI_IRQn        EXTI15_10_IRQn

/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
static uint8_t RxData [500];

/* Significant motion detection flag (set in EXTI ISR) */
static volatile uint8_t significant_motion_detected = 0;

/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small TERMOUT (option LD Linker->Libraries->Small TERMOUT
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
static void LSM6DSL_SignificantMotion_Init(void);
static void LSM6DSL_INT1_GPIO_Init(void);
extern  SPI_HandleTypeDef hspi;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure LSM6DSL INT1 pin (PD11) as EXTI interrupt
  */
static void LSM6DSL_INT1_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  LSM6DSL_INT1_GPIO_CLK_ENABLE();

  GPIO_InitStruct.Pin = LSM6DSL_INT1_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LSM6DSL_INT1_GPIO_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(LSM6DSL_INT1_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LSM6DSL_INT1_EXTI_IRQn);
}

/**
  * @brief  Enable LSM6DSL significant motion detection (AN5040 procedure)
  *
  *   Initialization sequence per AN5040 Section 5.5:
  *   1. Set accelerometer ODR >= 26 Hz           (done by BSP_ACCELERO_Init at 52 Hz)
  *   2. Set significant motion threshold          (embedded bank register SM_THS)
  *   3. Enable FUNC_EN + SIGN_MOTION_EN + PEDO_EN in CTRL10_C
  *   4. Enable INTERRUPTS_ENABLE + LIR in TAP_CFG
  *   5. Route interrupt to INT1 via MD1_CFG
  *   6. Clear any pending event by reading FUNC_SRC
  */
static void LSM6DSL_SignificantMotion_Init(void)
{
  uint8_t tmp;

  /* Step 1: Configure threshold via embedded function register bank */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
    LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS, FUNC_CFG_EN);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
    LSM6DSL_ACC_GYRO_SM_STEP_THS, 0x01);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
    LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS, 0x00);

  /* Step 2: Enable embedded functions + significant motion + pedometer */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C);
  tmp |= (FUNC_EN | SIGN_MOTION_EN | PEDO_EN);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C, tmp);

  /* Step 3: Enable interrupt routing + latched interrupt mode */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1);
  tmp |= (INTERRUPTS_ENABLE | LIR);
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, tmp);

  /* Step 4: Route significant motion event to INT1 */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG);
  tmp |= INT1_SIGN_MOT;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, tmp);

  /* Step 5: Clear any residual event */
  (void)SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FUNC_SRC);

  /* Debug readback */
  TERMOUT("> [SigMot] CTRL1_XL  = 0x%02X  (ODR+FS)\n",
    SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, 0x10));
  TERMOUT("> [SigMot] CTRL3_C   = 0x%02X  (BDU/IF_INC/INT polarity)\n",
    SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C));
  TERMOUT("> [SigMot] CTRL10_C  = 0x%02X  (expect 0x15)\n",
    SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL10_C));
  TERMOUT("> [SigMot] TAP_CFG1  = 0x%02X  (expect 0x81)\n",
    SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1));
  TERMOUT("> [SigMot] MD1_CFG   = 0x%02X  (expect 0x40)\n",
    SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG));
  TERMOUT("> [SigMot] FUNC_SRC  = 0x%02X\n",
    SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FUNC_SRC));
  TERMOUT("> [SigMot] INT1 pin  = %d\n",
    HAL_GPIO_ReadPin(LSM6DSL_INT1_GPIO_PORT, LSM6DSL_INT1_GPIO_PIN));

  /* Verify SM_THS in embedded bank */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
    LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS, FUNC_CFG_EN);
  TERMOUT("> [SigMot] SM_THS    = 0x%02X  (threshold)\n",
    SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_SM_STEP_THS));
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
    LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS, 0x00);

  TERMOUT("> Significant Motion detection enabled on INT1 (PD11).\n");
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t  MAC_Addr[6] = {0};
  uint8_t  IP_Addr[4] = {0};
  uint8_t  TxData[512];
  int32_t  Socket = -1;
  uint16_t Datalen;
  int32_t  ret;
  int16_t  Trials = CONNECTION_TRIAL_MAX;
  int16_t  pDataXYZ[3] = {0};
  float    pGyroXYZ[3] = {0.0f};
  uint8_t  sig_motion_flag = 0;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Configure LED2 */
  BSP_LED_Init(LED2);

#if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  BSP_COM_Init(COM1, &hDiscoUart);
#endif /* TERMINAL_USE */

  /* Initialize accelerometer */
  if (BSP_ACCELERO_Init() != ACCELERO_OK)
  {
    TERMOUT("> ERROR : Accelerometer init failed\n");
  }
  else
  {
    TERMOUT("> Accelerometer initialized.\n");
  }

  /* Initialize gyroscope */
  if (BSP_GYRO_Init() != GYRO_OK)
  {
    TERMOUT("> ERROR : Gyroscope init failed\n");
  }
  else
  {
    TERMOUT("> Gyroscope initialized.\n");
  }

  /* Initialize significant motion detection (GPIO EXTI + LSM6DSL registers) */
  LSM6DSL_INT1_GPIO_Init();
  LSM6DSL_SignificantMotion_Init();

  TERMOUT("****** WIFI Module in TCP Client mode demonstration ****** \n\n");
  TERMOUT("Sending Accelerometer + Gyroscope data with Significant Motion detection\n\n");

  /* Initialize WIFI module */
  TERMOUT("[LOG] Initializing WiFi module...\n");
  if(WIFI_Init() !=  WIFI_STATUS_OK)
  {
    TERMOUT("[ERR] WiFi module initialization FAILED.\n");
    BSP_LED_On(LED2);
    goto wifi_error;
  }
  TERMOUT("[LOG] WiFi module initialized OK.\n");

  if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
  {
    TERMOUT("[LOG] MAC Address: %X:%X:%X:%X:%X:%X\n",
             MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
             MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
  }
  else
  {
    TERMOUT("[WARN] Cannot get MAC address, continuing...\n");
  }

  /* Connect to WiFi AP with retry */
  TERMOUT("[LOG] Connecting to AP \"%s\"...\n", SSID);
  for (int wifi_try = 1; wifi_try <= CONNECTION_TRIAL_MAX; wifi_try++)
  {
    TERMOUT("[LOG] WiFi connect attempt %d/%d\n", wifi_try, CONNECTION_TRIAL_MAX);
    if(WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
    {
      TERMOUT("[LOG] WiFi connected to \"%s\".\n", SSID);
      break;
    }
    TERMOUT("[WARN] WiFi connect attempt %d failed.\n", wifi_try);
    if (wifi_try == CONNECTION_TRIAL_MAX)
    {
      TERMOUT("[ERR] WiFi connect FAILED after %d attempts.\n", CONNECTION_TRIAL_MAX);
      BSP_LED_On(LED2);
      goto wifi_error;
    }
    HAL_Delay(1000);
  }

  if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
  {
    TERMOUT("[LOG] IP Address: %d.%d.%d.%d\n",
           IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]);
  }
  else
  {
    TERMOUT("[ERR] Cannot get IP address.\n");
    BSP_LED_On(LED2);
    goto wifi_error;
  }

  /* Open TCP connection with retry */
  TERMOUT("[LOG] Connecting to server %d.%d.%d.%d:%d ...\n",
         RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3], RemotePORT);

  Trials = CONNECTION_TRIAL_MAX;
  while (Trials--)
  {
    TERMOUT("[LOG] TCP connect attempt %d/%d\n",
           CONNECTION_TRIAL_MAX - Trials, CONNECTION_TRIAL_MAX);
    if(WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
    {
      TERMOUT("[LOG] TCP connection opened OK.\n");
      Socket = 0;
      break;
    }
    TERMOUT("[WARN] TCP connect attempt %d failed.\n", CONNECTION_TRIAL_MAX - Trials);
    HAL_Delay(500);
  }
  if(Socket == -1)
  {
    TERMOUT("[ERR] TCP connect FAILED after %d attempts.\n", CONNECTION_TRIAL_MAX);
    BSP_LED_On(LED2);
  }

wifi_error:

  TERMOUT("[LOG] Entering main loop.\n");
  uint32_t send_count = 0;

  while(1)
  {
    if(Socket == -1)
    {
      /* No connection - blink LED and wait */
      BSP_LED_Toggle(LED2);
      HAL_Delay(500);
      continue;
    }

    /* Read accelerometer data */
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);

    /* Read gyroscope data */
    BSP_GYRO_GetXYZ(pGyroXYZ);

    /*
     * Check significant motion: poll FUNC_SRC as backup for EXTI.
     * With LIR=1, reading FUNC_SRC also clears the latched INT1 pin,
     * allowing the next event to trigger a new rising edge on EXTI.
     */
    {
      uint8_t func_src = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
                                         LSM6DSL_ACC_GYRO_FUNC_SRC);
      if (func_src & SIGN_MOT_IA)
      {
        significant_motion_detected = 1;
      }
    }

    sig_motion_flag = 0;
    if (significant_motion_detected)
    {
      sig_motion_flag = 1;
      significant_motion_detected = 0;
      BSP_LED_Toggle(LED2);
      TERMOUT("[EVENT] Significant Motion Detected!\n");
    }

    /* Format data as JSON */
    sprintf((char *)TxData,
      "{\"acc\":{\"x\":%d,\"y\":%d,\"z\":%d},"
      "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
      "\"sig_motion\":%d}\n",
      pDataXYZ[0], pDataXYZ[1], pDataXYZ[2],
      pGyroXYZ[0], pGyroXYZ[1], pGyroXYZ[2],
      sig_motion_flag);

    ret = WIFI_SendData(Socket, (uint8_t *)TxData, strlen((char *)TxData), &Datalen, WIFI_WRITE_TIMEOUT);
    if (ret != WIFI_STATUS_OK)
    {
      TERMOUT("[ERR] Send failed (count=%lu), closing socket.\n", send_count);
      WIFI_CloseClientConnection(0);
      Socket = -1;

      /* Try to reconnect */
      TERMOUT("[LOG] Attempting TCP reconnect...\n");
      HAL_Delay(1000);
      for (int retry = 1; retry <= CONNECTION_TRIAL_MAX; retry++)
      {
        TERMOUT("[LOG] Reconnect attempt %d/%d\n", retry, CONNECTION_TRIAL_MAX);
        if(WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
        {
          TERMOUT("[LOG] Reconnected OK.\n");
          Socket = 0;
          send_count = 0;
          break;
        }
        HAL_Delay(500);
      }
      if (Socket == -1)
      {
        TERMOUT("[ERR] Reconnect failed. Idling.\n");
      }
      continue;
    }

    send_count++;
    if (send_count % 100 == 0)
    {
      TERMOUT("[LOG] Sent %lu packets.\n", send_count);
    }

    /* Wait for next send interval, but break early if sig motion ISR fires */
    {
      uint32_t wait_start = HAL_GetTick();
      while ((HAL_GetTick() - wait_start) < SENSOR_SEND_INTERVAL)
      {
        if (significant_motion_detected)
          break;
      }
    }
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library TERMOUT function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Auto-insert \r before \n for proper serial terminal line endings */
  if (ch == '\n')
  {
    uint8_t cr = '\r';
    HAL_UART_Transmit(&hDiscoUart, &cr, 1, 0xFFFF);
  }
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: TERMOUT("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    case (LSM6DSL_INT1_GPIO_PIN):
    {
      /* LSM6DSL INT1: significant motion detected via hardware interrupt */
      significant_motion_detected = 1;
      break;
    }
    default:
    {
      break;
    }
  }
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}
