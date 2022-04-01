/*
 * LIS302DL.c
 *
 *  Created on: 14 mar 2022
 *      Author: Maryna
 */
#include "LIS302DL.h"
#include "main.h"
#include "stdio.h"
//#include "tools.h"
//SPI Chip Select
#define _LIS302DL_CS_ENBALE			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
#define _LIS302DL_CS_DISABLE		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

//Library variables
//SPI handle
static SPI_HandleTypeDef accSPI_Handle;

//Sensitivity value
static float LIS302DL_Sensitivity = LIS302DL_SENSITIVITY_0_018G;
//Ofset variables
static float xOffset = 0.0f;
static float yOffset = 0.0f;
static float zOffset = 0.0f;
//Scaling variables
static float xScale = 1.0f;
static float yScale = 1.0f;
static float zScale = 1.0f;

//Private functions
//Write IO
void LIS302DL_WriteIO(uint8_t reg, uint8_t *dataW, uint8_t size)
{
	uint8_t spiReg = reg;
	_LIS302DL_CS_ENBALE;
	HAL_SPI_Transmit(&accSPI_Handle, &spiReg, 1, 10);
	HAL_SPI_Transmit(&accSPI_Handle, dataW, size, 10);
	_LIS302DL_CS_DISABLE;
}

//Read IO
void LIS302DL_ReadIO(uint8_t reg, uint8_t *dataR, uint8_t size)
{
	uint8_t spiBuf[4];
	spiBuf[0] = reg | 0x80;
	_LIS302DL_CS_ENBALE;
	HAL_SPI_Transmit(&accSPI_Handle, spiBuf, 1, 10);
	HAL_SPI_Receive(&accSPI_Handle, spiBuf, size, 10);
	_LIS302DL_CS_DISABLE;

	for(uint8_t i=0; i<(size&0x3); i++)
	{
		dataR[i] = spiBuf[i];
	}
}

//Accelerometer initialise function
void LIS302DL_Init(SPI_HandleTypeDef *accSPI, LIS302DL_InitTypeDef *accInitDef)
{

	uint8_t spiData;

	memcpy(&accSPI_Handle, accSPI, sizeof(*accSPI));
	//CTRL_REG1 forming
	spiData = 0;
	if (accInitDef->dataRate) spiData |= 0x80;
	if (accInitDef->powerDown) spiData |= 0x40;
	if (accInitDef->fullScale) spiData |= 0x20;
	spiData |= (accInitDef->enableAxes & 0x07);


	//Write CTRL_REG1 and UART echo
	LIS302DL_WriteIO(LIS302DL_CTRL_REG1_ADDR, &spiData, 1);
//	sprintf(uartBuffer ,"REG1: 0x%0X\n\r", spiData);
//	uartLog(uartBuffer);

	//CTRL_REG2 forming
	spiData = 0;
	if (accInitDef->serialInterfaceMode) spiData |= 0x80;
	if (accInitDef->rebootMemory) spiData |= 0x40;
	if (accInitDef->filterConfig) spiData |= 0x1F;
	LIS302DL_WriteIO(LIS302DL_CTRL_REG2_ADDR, &spiData, 1);
	//Write CTRL_REG2 and UART echo
//	sprintf(uartBuffer ,"REG2: 0x%0X\n\r", spiData);
//	uartLog(uartBuffer);

	if (accInitDef->interruptConfig)
		{
			//CTRL_REG3 forming
			spiData = accInitDef->interruptConfig;
			//Write CTRL_REG3 and UART echo
			LIS302DL_WriteIO(LIS302DL_CTRL_REG3_ADDR, &spiData, 1);
//			sprintf(uartBuffer ,"REG3: 0x%0X\n\r", spiData);
//			uartLog(uartBuffer);
		}
	//Assign sensor sensitivity
	if (accInitDef->fullScale)
	{
		LIS302DL_Sensitivity = LIS302DL_SENSITIVITY_0_018G;
	}
	else
	{
		LIS302DL_Sensitivity = LIS302DL_SENSITIVITY_0_072G;
	}
}

//Get Accelerometer raw data
LIS302DL_DataRaw LIS302DL_GetDataRaw(void)
{
	LIS302DL_DataRaw tempDataRaw;
	LIS302DL_ReadIO(LIS302DL_OUT_X_ADDR, &tempDataRaw.x, 1);
	LIS302DL_ReadIO(LIS302DL_OUT_Y_ADDR, &tempDataRaw.y, 1);
	LIS302DL_ReadIO(LIS302DL_OUT_Z_ADDR, &tempDataRaw.z, 1);
	return tempDataRaw;
}
//Get Accelerometer scaled data [g]
LIS302DL_DataScaled LIS302DL_GetDataScaled(void)
{

	LIS302DL_DataRaw tempRawData = LIS302DL_GetDataRaw();
	LIS302DL_DataScaled tempScaledData;
	tempScaledData.x = (tempRawData.x * LIS302DL_Sensitivity * xScale) + 0.0f - xOffset;
	tempScaledData.y = (tempRawData.y * LIS302DL_Sensitivity * yScale) + 0.0f - yOffset;
	tempScaledData.z = (tempRawData.z * LIS302DL_Sensitivity * zScale) + 0.0f - zOffset;

	return tempScaledData;
}
//Poll for Data Ready
bool LIS302DL_PollDRDY(uint32_t msTimeout)
{
	uint8_t Acc_status;
	uint32_t startTick = HAL_GetTick();
	do
	{
		//Read status register with a timeout
		LIS302DL_ReadIO(LIS302DL_STATUS_ADDR, &Acc_status, 1);
		if (Acc_status & 0x07) break;
	}
	while ((Acc_status & 0x07) == 0 && (HAL_GetTick() - startTick) < msTimeout);
	if (Acc_status & 0x07)
	{
		return true;
	}
	return false;
}

//** Calibration functions **//
//X-Axis calibrate
void LIS302DL_X_calibrate(float x_min, float x_max)
{
	xOffset = (x_max + x_min) / 2.0f;
	xScale = 2.0f / (x_max - x_min);
}
//Y-Axis calibrate
void LIS302DL_Y_calibrate(float y_min, float y_max)
{
	yOffset = (y_max + y_min) / 2.0f;
	yScale = 2.0f / (y_max - y_min);
}
//Z-Axis calibrate
void LIS302DL_Z_calibrate(float z_min, float z_max)
{
	zOffset = (z_max + z_min) / 2.0f;
	zScale = 2.0f / (z_max - z_min);
}


