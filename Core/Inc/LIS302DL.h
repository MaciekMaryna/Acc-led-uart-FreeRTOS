/*
 * LIS302DL.h
 *
 *  Created on: 14 mar 2022
 *      Author: Maryna
 */
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

#ifndef INC_LIS302DL_H_
#define INC_LIS302DL_H_
#endif /* INC_LIS302DL_H_ */
//LIS302DL registers addresses
#define LIS302DL_WHO_AM_I_ADDR                0x0F

#define LIS302DL_CTRL_REG1_ADDR               0x20
#define LIS302DL_CTRL_REG2_ADDR               0x21
#define LIS302DL_CTRL_REG3_ADDR               0x22

#define LIS302DL_STATUS_ADDR        	      0x27

#define LIS302DL_OUT_X_ADDR                   0x29
#define LIS302DL_OUT_Y_ADDR                   0x2B
#define LIS302DL_OUT_Z_ADDR                   0x2D


#define LIS302DL_POWERDOWN     			      false	// Power Down Mode
#define LIS302DL_ACTIVE		       			  true	// Power Active Mode

//Data rate
#define LIS302DL_DATARATE_100                 false	// 100Hz Normal Mode
#define LIS302DL_DATARATE_400                 true	// 400Hz Normal Mode

//Serial interface mode
#define LIS302DL_SERIAL_INTERFACE_4WIRE 	  false
#define LIS302DL_SERIAL_INTERFACE_3WIRE 	  true

//Reboot memory content
#define LIS302DL_BOOT_NORMAL_MODE 			  false
#define LIS302DL_BOOT_REBOOT_MEMORY		 	  true

//Filtering
#define LIS302DL_FILTERING_NONE 			  ((uint8_t)0x00)

//Interrupt
#define LIS302DL_INTERRUPT_NONE				  ((uint8_t)0x00)

//Full scale
#define LIS302DL_FULLSCALE_2                  false	// 2.3g
#define LIS302DL_FULLSCALE_8                  true	// 9.2g

//Enabled axis
#define LIS302DL_X_ENABLE                     ((uint8_t)0x01)
#define LIS302DL_Y_ENABLE                     ((uint8_t)0x02)
#define LIS302DL_Z_ENABLE                     ((uint8_t)0x04)
#define LIS302DL_XYZ_ENABLE                   ((uint8_t)0x07)


//Typedefs
//Accelerometer config struct
typedef struct
{
	bool dataRate;				// 0: for 100 Hz, 1: for 400 Hz output data rate
	bool powerDown;				// 0: for power down, 1: for active mode
	bool fullScale;				// 0: for 2.3g, 1: for 9.2 measure ramnge (respectively 18 or 72 mg/digit)
	uint8_t enableAxes;			// 7: enables XYZ axes, see reference CTRL_REG1(20h)
	bool serialInterfaceMode;	// 0: 4-wire, 1: 3-wire SPI interface
	bool rebootMemory;			// 0: normal mode, 1: reboot memory content
	uint8_t filterConfig;		// 0: no filtering, see reference CTRL_REG2(21h)
	uint8_t interruptConfig;	// 0: no interrupt, see reference CTRL_REG2(22h)

}LIS302DL_Init_t;

//Accelerometer raw data
typedef struct
{
	int8_t x;
	int8_t y;
	int8_t z;
}LIS302DL_DataRaw_t;

//Accelerometer scaled data [g]
typedef struct
{
	float x;
	float y;
	float z;
}LIS302DL_DataScaled_t;


//Accelerometer initialise function
void LIS302DL_Init(SPI_HandleTypeDef *accSPI, LIS302DL_Init_t *accInitDef);
//Get Accelerometer raw data
LIS302DL_DataRaw_t LIS302DL_GetDataRaw(void);
//Get Accelerometer mg data
LIS302DL_DataScaled_t LIS302DL_GetDataScaled(void);
//Poll for Data Ready
bool LIS302DL_PollDRDY(uint32_t msTimeout);
//Check Data Ready withot polling
bool LIS302DL_DRDY_Test();

//** Calibration functions **//
//Set X-Axis calibrate
void LIS302DL_X_calibrate(float x_min, float x_max);
//Set Y-Axis calibrate
void LIS302DL_Y_calibrate(float y_min, float y_max);
//Set Z-Axis calibrate
void LIS302DL_Z_calibrate(float z_min, float z_max);




