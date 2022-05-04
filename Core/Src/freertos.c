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
uint8_t TransmitBuffer;

/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 256 * 4,
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
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for QueueAccData */
osMessageQueueId_t QueueAccDataHandle;
const osMessageQueueAttr_t QueueAccData_attributes = {
  .name = "QueueAccData"
};
/* Definitions for QueueUartRequestData */
osMessageQueueId_t QueueUartRequestDataHandle;
const osMessageQueueAttr_t QueueUartRequestData_attributes = {
  .name = "QueueUartRequestData"
};
/* Definitions for QueueTransmitData */
osMessageQueueId_t QueueTransmitDataHandle;
const osMessageQueueAttr_t QueueTransmitData_attributes = {
  .name = "QueueTransmitData"
};
/* Definitions for QueueLedTilt */
osMessageQueueId_t QueueLedTiltHandle;
const osMessageQueueAttr_t QueueLedTilt_attributes = {
  .name = "QueueLedTilt"
};
/* Definitions for Semaphore4LedsOn */
osSemaphoreId_t Semaphore4LedsOnHandle;
const osSemaphoreAttr_t Semaphore4LedsOn_attributes = {
  .name = "Semaphore4LedsOn"
};
/* Definitions for SemaphoreDataRequest */
osSemaphoreId_t SemaphoreDataRequestHandle;
const osSemaphoreAttr_t SemaphoreDataRequest_attributes = {
  .name = "SemaphoreDataRequest"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartAccTask(void *argument);
void StartUartTask(void *argument);
void StartLedTask(void *argument);

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
  /* creation of Semaphore4LedsOn */
  Semaphore4LedsOnHandle = osSemaphoreNew(1, 1, &Semaphore4LedsOn_attributes);

  /* creation of SemaphoreDataRequest */
  SemaphoreDataRequestHandle = osSemaphoreNew(1, 1, &SemaphoreDataRequest_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueAccData */
  QueueAccDataHandle = osMessageQueueNew (8, sizeof(LIS302DL_DataScaled_t), &QueueAccData_attributes);

  /* creation of QueueUartRequestData */
  QueueUartRequestDataHandle = osMessageQueueNew (8, sizeof(uint8_t), &QueueUartRequestData_attributes);

  /* creation of QueueTransmitData */
  QueueTransmitDataHandle = osMessageQueueNew (8, sizeof(LIS302DL_DataScaled_t), &QueueTransmitData_attributes);

  /* creation of QueueLedTilt */
  QueueLedTiltHandle = osMessageQueueNew (8, sizeof(Led_Tilt_t), &QueueLedTilt_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of AccTask */
  AccTaskHandle = osThreadNew(StartAccTask, NULL, &AccTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

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
	Led_Tilt_t _myTilt;

	xSemaphoreTake(Semaphore4LedsOnHandle,0);

  /* Infinite loop */
  for(;;)
  {

	xSemaphoreGive(SemaphoreDataRequestHandle);
	xQueueReceive(QueueAccDataHandle, &_myData, 0);
	xQueueSendToBack(QueueTransmitDataHandle, &_myData, 0);
	_myTilt.x = _myData.x;
	_myTilt.y = _myData.y;
	xQueueSendToBack(QueueLedTiltHandle, &_myTilt, 0);

	vTaskDelay(100 / portTICK_PERIOD_MS);

  }
  /* USER CODE END StartMainTask */
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

	  if (pdTRUE == xSemaphoreTake(SemaphoreDataRequestHandle, portMAX_DELAY))
	  {
		  _myData = LIS302DL_GetDataScaled();
		  xQueueSendToBack(QueueAccDataHandle, &_myData, 0);
	  }
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

	LIS302DL_DataScaled_t _myData;
	uint8_t buffor;
	HAL_UART_Receive_IT(&huart2, &ReciveBuffer, 1);

	/* Infinite loop */
  for(;;)
  {

	  	if (pdPASS == xQueueReceive(QueueUartRequestDataHandle, &buffor, 0))
	  	{
	  		switch (buffor)
			{
	  			case '?':	printf("PC is asking for tilt data.\n\r");
	  						break;
	  			default	:	printf("Unrecognized request from PC.\n\r");
			}
	  		xSemaphoreGive(Semaphore4LedsOnHandle);
	  	}

		if (pdPASS == xQueueReceive(QueueTransmitDataHandle, &_myData, portMAX_DELAY))
		{
			printf("x:%4.1f y:%4.1f z:%4.1f\n\r", _myData.x, _myData.y, _myData.z);
		}
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */

	Led_Tilt_t _myTilt;
	/* Infinite loop */
	for(;;)
	{

	  if (pdTRUE == xSemaphoreTake(Semaphore4LedsOnHandle, 0))
	  {
		  Led_AllON();
		  vTaskDelay(500 / portTICK_PERIOD_MS);
	  }
	  else
	  {
		  xQueueReceive(QueueLedTiltHandle, &_myTilt, 0);
		  Led_ShowTilt(_myTilt);
		  vTaskDelay(100 / portTICK_PERIOD_MS);
	  }
	}
  /* USER CODE END StartLedTask */
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
		xQueueSendToBackFromISR(QueueUartRequestDataHandle, &ReciveBuffer, NULL);
		HAL_UART_Receive_IT(&huart2, &ReciveBuffer, 1);
	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
