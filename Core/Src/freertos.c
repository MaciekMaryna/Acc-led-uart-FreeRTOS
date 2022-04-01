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
#include "semphr.h"

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

/* USER CODE END Variables */
/* Definitions for Led4Task */
osThreadId_t Led4TaskHandle;
const osThreadAttr_t Led4Task_attributes = {
  .name = "Led4Task",
  .stack_size = 128 * 4,
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PrintUartMutex */
osMutexId_t PrintUartMutexHandle;
const osMutexAttr_t PrintUartMutex_attributes = {
  .name = "PrintUartMutex"
};
/* Definitions for PrintSemaphore */
osSemaphoreId_t PrintSemaphoreHandle;
const osSemaphoreAttr_t PrintSemaphore_attributes = {
  .name = "PrintSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLed4Task(void *argument);
void StartHardBitTask(void *argument);
void StartAccTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of PrintUartMutex */
  PrintUartMutexHandle = osMutexNew(&PrintUartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of PrintSemaphore */
  PrintSemaphoreHandle = osSemaphoreNew(1, 1, &PrintSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Led4Task */
  Led4TaskHandle = osThreadNew(StartLed4Task, NULL, &Led4Task_attributes);

  /* creation of HardBitTask */
  HardBitTaskHandle = osThreadNew(StartHardBitTask, NULL, &HardBitTask_attributes);

  /* creation of AccTask */
  AccTaskHandle = osThreadNew(StartAccTask, NULL, &AccTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLed4Task */
/**
  * @brief  Function implementing the Led4Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLed4Task */
void StartLed4Task(void *argument)
{
  /* USER CODE BEGIN StartLed4Task */
  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartLed4Task */
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
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);

		if (pdTRUE == xSemaphoreTake(PrintSemaphoreHandle, 0))
		{
			printf(".");
			xSemaphoreGive(PrintSemaphoreHandle);
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);

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
	LIS302DL_DataScaled myData;
	LIS302DL_InitTypeDef myAccel;

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

	//calibration to meas range
	LIS302DL_X_calibrate(4.0, -4.0);
	LIS302DL_Y_calibrate(4.0, -4.0);
	LIS302DL_Z_calibrate(4.0, -4.0);
	taskEXIT_CRITICAL();
  /* Infinite loop */
  for(;;)
  {


	  myData = LIS302DL_GetDataScaled();

	  xSemaphoreTake(PrintSemaphoreHandle, portMAX_DELAY);

	  printf("\n\rx: %4.1f y: %4.1f z: %4.1f ", myData.x, myData.y, myData.z);

	  xSemaphoreGive(PrintSemaphoreHandle);

	  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  /* USER CODE END StartAccTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{

	xSemaphoreTake(PrintUartMutexHandle, portMAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) &character, 1, 1000);
	xSemaphoreGive(PrintUartMutexHandle);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
