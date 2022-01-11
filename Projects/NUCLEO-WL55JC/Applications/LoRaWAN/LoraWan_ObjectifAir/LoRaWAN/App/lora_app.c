/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "sys_conf.h"
#include "sys_sensors.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/* Neopixel like micropython implementation's */
#define NS_CYCLES_OVERHEAD (6)
#define PIXEL_COUNT 12 /* 12 for the ring */
#define PIXEL_MAX_VALUE 254
#define PIXEL_PORT GPIOB /* On D2 connector */
#define PIXEL_PIN GPIO_PIN_12

uint8_t neopixel_buf[PIXEL_COUNT/* nb of LED*/][3/*Green/Red/Blue*/] = {0};

static void mp_hal_ticks_cpu_enable(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        #if defined(__CORTEX_M) && __CORTEX_M == 7
        // on Cortex-M7 we must unlock the DWT before writing to its registers
        DWT->LAR = 0xc5acce55;
        #endif
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static void machine_bitstream_high_low(GPIO_TypeDef* GPIOx, uint32_t Pin, const uint8_t *buf, size_t len) {
    const uint32_t high_mask = Pin;
    const uint32_t low_mask = Pin << 16;
    volatile uint32_t *bsrr = &GPIOx->BSRR;
    uint32_t timing_ns[4] = {400, 850, 800, 450}; /* 800KHz */
    //uint32_t timing_ns[4] = {800, 1700, 1600, 900}; /* 400KHz */

    // Convert ns to cycles [high_time_0, low_time_0, high_time_1, low_time_1].
    for (size_t i = 0; i < 4; ++i) {
        timing_ns[i] = SystemCoreClock / 1000000 * timing_ns[i] / 1000;
        if (timing_ns[i] > NS_CYCLES_OVERHEAD) {
            timing_ns[i] -= NS_CYCLES_OVERHEAD;
        }
        if (i % 2 == 1) {
            timing_ns[i] += timing_ns[i - 1];
        }
    }

    HAL_Delay(1);
    mp_hal_ticks_cpu_enable();

    /* Enter critical section */
    UTILS_ENTER_CRITICAL_SECTION();

    for (size_t i = 0; i < len; ++i) {
        uint8_t b = buf[i];
        for (size_t j = 0; j < 8; ++j) {
            DWT->CYCCNT = 0;
            *bsrr = high_mask;
            uint32_t *t = &timing_ns[b >> 6 & 2];
            while (DWT->CYCCNT < t[0]) {
                ;
            }
            *bsrr = low_mask;
            b <<= 1;
            while (DWT->CYCCNT < t[1]) {
                ;
            }
        }
    }

    /* Exit critical section: restore previous priority mask */
    UTILS_EXIT_CRITICAL_SECTION();
}

/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRa application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/*!
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */
static void OnMacProcessNotify(void);

/* USER CODE BEGIN PFP */

/**
  * @brief  LED Tx timer callback function
  * @param  context ptr of LED context
  */
static void OnTxTimerLedEvent(void *context);

/**
  * @brief  LED Rx timer callback function
  * @param  context ptr of LED context
  */
static void OnRxTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  context ptr of LED context
  */
static void OnSensorTimerEvent(void *context);

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =           GetBatteryLevel,
  .GetTemperature =            GetTemperatureLevel,
  .GetUniqueId =               GetUniqueId,
  .GetDevAddr =                GetDevAddr,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/* USER CODE BEGIN PV */
/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/**
  * @brief Specifies the state of the application LED
  */
static uint8_t AppLedStateOn = RESET;

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static UTIL_TIMER_Object_t TxLedTimer;

/**
  * @brief Timer to handle the application Rx Led to toggle
  */
static UTIL_TIMER_Object_t RxLedTimer;

/**
  * @brief Timer to handle the application sensor datas & LCD update
  */
static UTIL_TIMER_Object_t SensorsTimer;


/**
  * @brief User sensor datas
  */
static sensor_t SensorData;

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */
void neopixel_fill(uint8_t buff[][3], uint8_t cnt, uint8_t r,uint8_t g,uint8_t b){
	  uint8_t j;
	  for(j=0; j<cnt; j++){
		  buff[j][0] = g;
		  buff[j][1] = r;
		  buff[j][2] = b;
	  }
}
/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_1 */
  GPIO_InitTypeDef  gpio_init_structure = {0};

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  gpio_init_structure.Pin = PIXEL_PIN;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PIXEL_PORT, &gpio_init_structure);

  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);

  /* Clear ring */
  machine_bitstream_high_low(PIXEL_PORT,PIXEL_PIN,neopixel_buf,sizeof(neopixel_buf));
  /* Quick test of neopixel LED */
  uint8_t j;
  for(j=0; j < 3; j++){
	  neopixel_fill(neopixel_buf, PIXEL_COUNT, (j==0)?PIXEL_MAX_VALUE:0,(j==2)?PIXEL_MAX_VALUE:0,(j==1)?PIXEL_MAX_VALUE:0);
	  HAL_Delay(500);
	  machine_bitstream_high_low(PIXEL_PORT,PIXEL_PIN,neopixel_buf,sizeof(neopixel_buf));
  }

  /* Get LoRa APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION:        V%X.%X.%X\r\n",
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW LoraWAN info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:   V%X.%X.%X\r\n",
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB2_SHIFT));

  UTIL_TIMER_Create(&TxLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&RxLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnRxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&SensorsTimer, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, OnSensorTimerEvent, NULL);
  UTIL_TIMER_SetPeriod(&TxLedTimer, 500);
  UTIL_TIMER_SetPeriod(&RxLedTimer, 500);
  UTIL_TIMER_SetPeriod(&SensorsTimer, APP_SENSOR_CYCLE);

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  UTIL_TIMER_Start(&SensorsTimer);

  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType);

  /* send every time timer elapses */
  UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
  UTIL_TIMER_SetPeriod(&TxTimer,  APP_TX_DUTYCYCLE);
  UTIL_TIMER_Start(&TxTimer);

  /* USER CODE BEGIN LoRaWAN_Init_3 */
  /* When button is pushed, start sensor calibration */
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  /* USER CODE END LoRaWAN_Init_3 */

  /* USER CODE BEGIN LoRaWAN_Init_Last */
#define HEX4(X,Y)   X[Y+0], X[Y+1], X[Y+2], X[Y+3]

  uint8_t devEUI[8];
  LmHandlerGetDevEUI(devEUI);
  /* Initialize for LCD */
  BSP_I2C2_Init();
  rgb_lcd_setCursor(0,0);
  rgb_lcd_print("EUI %02X:%02X:%02X:%02X", HEX4(devEUI,0));
  rgb_lcd_setCursor(0,1);
  rgb_lcd_print("    %02X:%02X:%02X:%02X", HEX4(devEUI,4));
  /* Initialize for LCD */
  BSP_I2C2_DeInit();

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */
/* Note: Current the stm32wlxx_it.c generated by STM32CubeMX does not support BSP for PB in EXTI mode. */
/* In order to get a push button IRS by code automatically generated */
/* HAL_GPIO_EXTI_Callback is today the only available possibility. */
/* Using HAL_GPIO_EXTI_Callback() shortcuts the BSP. */
/* If users wants to go through the BSP, stm32wlxx_it.c should be updated  */
/* in the USER CODE SESSION of the correspondent EXTIn_IRQHandler() */
/* to call the BSP_PB_IRQHandler() or the HAL_EXTI_IRQHandler(&H_EXTI_n);. */
/* Then the below HAL_GPIO_EXTI_Callback() can be replaced by BSP callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case  BUTTON_SW1_PIN:
      /* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      break;
    case  BUTTON_SW2_PIN:
      break;
    case  BUTTON_SW3_PIN:
      break;
    default:
      break;
  }
}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  if ((appData != NULL) || (params != NULL))
  {
    BSP_LED_On(LED_BLUE) ;

    UTIL_TIMER_Start(&RxLedTimer);

    static const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };

    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Indication ==========\r\n");
    APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\r\n",
            params->DownlinkCounter, slotStrings[params->RxSlot], appData->Port, params->Datarate, params->Rssi, params->Snr);
    switch (appData->Port)
    {
      case LORAWAN_SWITCH_CLASS_PORT:
        /*this port switches the class*/
        if (appData->BufferSize == 1)
        {
          switch (appData->Buffer[0])
          {
            case 0:
            {
              LmHandlerRequestClass(CLASS_A);
              break;
            }
            case 1:
            {
              LmHandlerRequestClass(CLASS_B);
              break;
            }
            case 2:
            {
              LmHandlerRequestClass(CLASS_C);
              break;
            }
            default:
              break;
          }
        }
        break;
      case LORAWAN_USER_APP_PORT:
        if (appData->BufferSize == 1)
        {
          AppLedStateOn = appData->Buffer[0] & 0x01;
          if (AppLedStateOn == RESET)
          {
            APP_LOG(TS_OFF, VLEVEL_M, "LED RED OFF\r\n");
            BSP_LED_Off(LED_RED) ;
          }
          else
          {
            APP_LOG(TS_OFF, VLEVEL_M, "LED RED ON\r\n");
            BSP_LED_On(LED_RED) ;
          }
        }
        break;

      default:

        break;
    }
  }
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  int8_t temperature = 0;
  uint8_t humidity = 0;
  uint16_t co2 = 0;
  UTIL_TIMER_Time_t nextTxIn = 0;
  uint32_t i = 0;

  /* Needed for LCD */
  BSP_I2C2_Init();
  rgb_lcd_setCursor(15,1);
  rgb_lcd_write('\0');
  /* Needed for LCD */
  BSP_I2C2_DeInit();

  temperature = (SYS_GetTemperatureLevel() >> 8);
  APP_LOG(TS_ON, VLEVEL_M, "mcu temp= %d\n", temperature);

  temperature = (int8_t)SensorData.temperature;
  humidity    = (uint8_t)SensorData.humidity;
  co2         = (int16_t)SensorData.co2;

  AppData.Port = LORAWAN_USER_APP_PORT;
  humidity    = (uint8_t)(SensorData.humidity);
  AppData.Buffer[i++] = (uint8_t)((co2 >> 8) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(co2 & 0xFF);
  AppData.Buffer[i++] = (uint8_t)temperature;
  AppData.Buffer[i++] = (uint8_t)humidity;
  AppData.Buffer[i++] = AppLedStateOn;
  AppData.BufferSize = i;

  if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
  {
    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
  }
  else if (nextTxIn > 0)
  {
    APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
  }

  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */
static void OnTxTimerLedEvent(void *context)
{
	/* Needed for LCD */
	BSP_I2C2_Init();
	rgb_lcd_setCursor(15,1);
	rgb_lcd_write(' ');
	/* Needed for LCD */
	BSP_I2C2_DeInit();
	BSP_LED_Off(LED_GREEN);
}

static void OnRxTimerLedEvent(void *context)
{
  BSP_LED_Off(LED_BLUE) ;
}

static void OnSensorTimerEvent(void *context)
{

  int8_t temperature = 0;
  uint8_t humidity = 0;
  uint16_t co2 = 0;

  /* Needed for LCD */
  BSP_I2C2_Init();

  EnvSensors_Read(&SensorData);
  temperature = (int8_t)SensorData.temperature;
  humidity    = (uint8_t)SensorData.humidity;
  co2         = (int16_t)SensorData.co2;

  rgb_lcd_setCursor(0,0);
  rgb_lcd_print("CO2 %3d ppm", co2);
  rgb_lcd_setCursor(0,1);
  rgb_lcd_print("T\2%2d\2C - H %2d%c ", temperature, humidity,'%');

  if((co2>800)&&(co2<1000))
	  neopixel_fill(neopixel_buf,PIXEL_COUNT,PIXEL_MAX_VALUE,PIXEL_MAX_VALUE/2,0);
  else if(co2>1000)
	  neopixel_fill(neopixel_buf,PIXEL_COUNT,PIXEL_MAX_VALUE,0,0);
  else
	  neopixel_fill(neopixel_buf,PIXEL_COUNT,0,PIXEL_MAX_VALUE,0);
  machine_bitstream_high_low(PIXEL_PORT,PIXEL_PIN,neopixel_buf,sizeof(neopixel_buf));

  APP_LOG(TS_ON, VLEVEL_M, "temp= %d\n", temperature);
  APP_LOG(TS_ON, VLEVEL_M, "humidity= %d\n", humidity);
  APP_LOG(TS_ON, VLEVEL_M, "co2= %d\n", co2);
  APP_LOG(TS_ON, VLEVEL_M, "@Graph:CO2:%d|T:%d|Hum:%d|\r", co2, temperature, humidity);

  /* Needed for LCD */
  BSP_I2C2_DeInit();

}

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  if ((params != NULL))
  {
    /* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
    if (params->IsMcpsConfirm != 0)
    {
      BSP_LED_On(LED_GREEN) ;
      UTIL_TIMER_Start(&TxLedTimer);

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
      APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
              params->AppData.Port, params->Datarate, params->TxPower);

      APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
      if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
      {
        APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
      }
    }
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  static uint8_t onlyonce=1;

  /* Needed for LCD */
  BSP_I2C2_Init();

  if(onlyonce){
     rgb_lcd_clear();
     onlyonce = 0;
  }

  /* USER CODE BEGIN OnJoinRequest_1 */
  if (joinParams != NULL)
  {
	rgb_lcd_setCursor(13,0);
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
      BSP_LED_Off(LED_RED) ;
      rgb_lcd_print("\1Ok");

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
      }
    }
    else
    {
	  rgb_lcd_print("\1Ko");
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }
  }
  BSP_I2C2_DeInit();
  /* USER CODE END OnJoinRequest_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
