/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "spi.h"
#include "printf.h"
#include "LIS302DL.h"
#include "Led.h"
#include "semphr.h"
#include "timers.h"

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
/* USER CODE BEGIN Variables */
uint8_t ReciveBuffer;
/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HardBitTask */
osThreadId_t HardBitTaskHandle;
const osThreadAttr_t HardBitTask_attributes = {
  .name = "HardBitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AccTask */
osThreadId_t AccTaskHandle;
const osThreadAttr_t AccTask_attributes = {
  .name = "AccTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for QueueAccData */
osMessageQueueId_t QueueAccDataHandle;
const osMessageQueueAttr_t QueueAccData_attributes = {
  .name = "QueueAccData"
};
/* Definitions for QueueUartTransmitData */
osMessageQueueId_t QueueUartTransmitDataHandle;
const osMessageQueueAttr_t QueueUartTransmitData_attributes = {
  .name = "QueueUartTransmitData"
};
/* Definitions for TimerPcRequestSimulator */
osTimerId_t TimerPcRequestSimulatorHandle;
const osTimerAttr_t TimerPcRequestSimulator_attributes = {
  .name = "TimerPcRequestSimulator"
};
/* Definitions for TimerUartTransmitData */
osTimerId_t TimerUartTransmitDataHandle;
const osTimerAttr_t TimerUartTransmitData_attributes = {
  .name = "TimerUartTransmitData"
};
/* Definitions for SemaphoreUartTransmitData */
osSemaphoreId_t SemaphoreUartTransmitDataHandle;
const osSemaphoreAttr_t SemaphoreUartTransmitData_attributes = {
  .name = "SemaphoreUartTransmitData"
};
/* Definitions for SemaphoreGetData */
osSemaphoreId_t SemaphoreGetDataHandle;
const osSemaphoreAttr_t SemaphoreGetData_attributes = {
  .name = "SemaphoreGetData"
};
/* Definitions for SemaphorePcRequestDataToLed */
osSemaphoreId_t SemaphorePcRequestDataToLedHandle;
const osSemaphoreAttr_t SemaphorePcRequestDataToLed_attributes = {
  .name = "SemaphorePcRequestDataToLed"
};
/* Definitions for SemaphorePcRequestDataToText */
osSemaphoreId_t SemaphorePcRequestDataToTextHandle;
const osSemaphoreAttr_t SemaphorePcRequestDataToText_attributes = {
  .name = "SemaphorePcRequestDataToText"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartHardBitTask(void *argument);
void StartAccTask(void *argument);
void StartUartTask(void *argument);
void CallbackTimerPcRequestSimulator(void *argument);
void CallbackTimerUartTransmitData(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreUartTransmitData */
  SemaphoreUartTransmitDataHandle = osSemaphoreNew(1, 1, &SemaphoreUartTransmitData_attributes);

  /* creation of SemaphoreGetData */
  SemaphoreGetDataHandle = osSemaphoreNew(1, 1, &SemaphoreGetData_attributes);

  /* creation of SemaphorePcRequestDataToLed */
  SemaphorePcRequestDataToLedHandle = osSemaphoreNew(1, 1, &SemaphorePcRequestDataToLed_attributes);

  /* creation of SemaphorePcRequestDataToText */
  SemaphorePcRequestDataToTextHandle = osSemaphoreNew(1, 1, &SemaphorePcRequestDataToText_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerPcRequestSimulator */
  TimerPcRequestSimulatorHandle = osTimerNew(CallbackTimerPcRequestSimulator, osTimerPeriodic, NULL, &TimerPcRequestSimulator_attributes);

  /* creation of TimerUartTransmitData */
  TimerUartTransmitDataHandle = osTimerNew(CallbackTimerUartTransmitData, osTimerPeriodic, NULL, &TimerUartTransmitData_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueAccData */
  QueueAccDataHandle = osMessageQueueNew (8, sizeof(LIS302DL_DataScaled_t), &QueueAccData_attributes);

  /* creation of QueueUartTransmitData */
  QueueUartTransmitDataHandle = osMessageQueueNew (8, sizeof(LIS302DL_DataScaled_t), &QueueUartTransmitData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of HardBitTask */
  HardBitTaskHandle = osThreadNew(StartHardBitTask, NULL, &HardBitTask_attributes);

  /* creation of AccTask */
  AccTaskHandle = osThreadNew(StartAccTask, NULL, &AccTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
	LIS302DL_DataScaled_t _myData;
	Led_Tilt_t actualTilt;

	xTimerChangePeriod((TimerHandle_t)TimerPcRequestSimulatorHandle, 3000 / portTICK_PERIOD_MS, portMAX_DELAY);

  /* Infinite loop */
  for(;;)
  {

	  //get data()
	  if (pdTRUE == xSemaphoreTake(SemaphoreGetDataHandle,0))
	  {
		  xQueueReceive(QueueAccDataHandle, &_myData, portMAX_DELAY);
	  }

	  //Parse_ACC_data()
	  actualTilt.x = _myData.x;
	  actualTilt.y = _myData.y;

	  //Set leds
	  if (pdTRUE == xSemaphoreGive((SemaphoreHandle_t)SemaphorePcRequestDataToLedHandle))
	  {
			Led_AllON();
	  }
	  else
	  {
		  	Led_ShowTilt(actualTilt);
	  }

	  if (pdTRUE == xSemaphoreTake(SemaphoreUartTransmitDataHandle,0))
	  {
		  xQueueSendToBack(QueueUartTransmitDataHandle, &_myData, 0);
	  }


	  //wait 500ms
	  vTaskDelay(100 / portTICK_PERIOD_MS);

  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartHardBitTask */
/**
* @brief Function implementing the HardBitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHardBitTask */
void StartHardBitTask(void *argument)
{
  /* USER CODE BEGIN StartHardBitTask */
  /* Infinite loop */
  for(;;)
  {
	  	//printf(".");
		vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartHardBitTask */
}

/* USER CODE BEGIN Header_StartAccTask */
/**
* @brief Function implementing the AccTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccTask */
void StartAccTask(void *argument)
{
  /* USER CODE BEGIN StartAccTask */
	LIS302DL_DataScaled_t _myData;
	LIS302DL_Init_t myAccel;

	taskENTER_CRITICAL();

	myAccel.dataRate=LIS302DL_DATARATE_400;
	myAccel.powerDown=LIS302DL_ACTIVE;
	myAccel.fullScale=LIS302DL_FULLSCALE_2;
	myAccel.enableAxes=LIS302DL_XYZ_ENABLE;
	myAccel.serialInterfaceMode=LIS302DL_SERIAL_INTERFACE_4WIRE;
	myAccel.rebootMemory=LIS302DL_BOOT_NORMAL_MODE;
	myAccel.filterConfig=LIS302DL_FILTERING_NONE;
	myAccel.interruptConfig=LIS302DL_INTERRUPT_NONE;

	LIS302DL_Init(&hspi1, &myAccel);

	//calibration acc to meas range
	LIS302DL_X_calibrate(4.0, -4.0);
	LIS302DL_Y_calibrate(4.0, -4.0);
	LIS302DL_Z_calibrate(4.0, -4.0);

	Led_Init();

	taskEXIT_CRITICAL();

  /* Infinite loop */
  for(;;)
  {
	  if (pdTRUE == xSemaphoreGive((SemaphoreHandle_t)SemaphoreGetDataHandle))
	  {
		  //if (LIS302DL_DRDY_Test())
		  {
			  _myData = LIS302DL_GetDataScaled();
			  xQueueSendToBack(QueueAccDataHandle, &_myData, portMAX_DELAY);
		  }
	  }
	  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartAccTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  /* Infinite loop */
	LIS302DL_DataScaled_t _myData;

	xTimerChangePeriod((TimerHandle_t)TimerUartTransmitDataHandle, 100 / portTICK_PERIOD_MS, portMAX_DELAY);
	HAL_UART_Receive_IT(&huart2, &ReciveBuffer, 1);

  for(;;)
  {

	  xQueueReceive(QueueUartTransmitDataHandle, &_myData, portMAX_DELAY);

		  if (pdTRUE == xSemaphoreGive((SemaphoreHandle_t)SemaphorePcRequestDataToTextHandle))
		  {
			  printf("\n\r****************************************************");
			  switch (ReciveBuffer)
			  {

			  case '?':
						  printf("\n\r*** What do you want PC? <- Answer to PC request ***");
						  break;
			  case 'x':
			  case 'X':
						  printf("\n\r*** x = %4.1f <- Answer to PC request             ***",_myData.x);
						  break;
			  case 'y':
			  case 'Y':
						  printf("\n\r*** y = %4.1f <- Answer to PC request             ***",_myData.y);
						  break;
			  case 'z':
			  case 'Z':
						  printf("\n\r*** z = %4.1f <- Answer to PC request             ***",_myData.z);
						  break;
			  default:
						  printf("\n\r*** Unknown command <- Answer to PC request");
			  }
			  printf("\n\r****************************************************");
		  }
		  else
		  {
			  printf("\n\rx:%4.1f y:%4.1f z:%4.1f", _myData.x, _myData.y, _myData.z);
		  }

	  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartUartTask */
}

/* CallbackTimerPcRequestSimulator function */
void CallbackTimerPcRequestSimulator(void *argument)
{
  /* USER CODE BEGIN CallbackTimerPcRequestSimulator */
//	xSemaphoreTake((SemaphoreHandle_t)SemaphorePcRequestDataToLedHandle, 0);
//	xSemaphoreTake((SemaphoreHandle_t)SemaphorePcRequestDataToTextHandle, 0);
	//printf("\n\rPC");

  /* USER CODE END CallbackTimerPcRequestSimulator */
}

/* CallbackTimerUartTransmitData function */
void CallbackTimerUartTransmitData(void *argument)
{
  /* USER CODE BEGIN CallbackTimerUartTransmitData */
	xSemaphoreGive(SemaphoreUartTransmitDataHandle);
  /* USER CODE END CallbackTimerUartTransmitData */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) &character, 1, 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (USART2 == huart->Instance)
	{
//		printf("\n\rSomething is comming on UART");
		xSemaphoreTakeFromISR((SemaphoreHandle_t)SemaphorePcRequestDataToLedHandle, NULL);
		xSemaphoreTakeFromISR((SemaphoreHandle_t)SemaphorePcRequestDataToTextHandle, NULL);
		HAL_UART_Receive_IT(&huart2, &ReciveBuffer, 1);
//		printf("\n\rSomething is comming on UART");
	}

}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
