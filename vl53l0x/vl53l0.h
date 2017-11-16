#ifndef __VL53L0_H
#define __VL53L0_H

//#include "stm32f10x.h"
#include <string.h>
//#include "usart.h"
//#include "timer.h"
#include "i2c.h"
#include "global.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "vl53l0x_api.h"
#include <limits.h>

#define IIC0_LIGHT_NUM  (6)
#define IIC1_LIGHT_NUM  (7)
#define LIGHT_NUM		    (13)

	typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
} RangingConfig_e;
//char *RangingConfigTxt[3] = {"LR", "HS", "HA"};

typedef enum {
	RANGE_VALUE 	= 0, /*!< Range displayed in cm */
	BAR_GRAPH 		= 1, /*!< Range displayed as a bar graph : one bar per sensor */
} DemoMode_e;

static union CurIOVal_u {
    uint8_t bytes[4];   /*!<  4 bytes array i/o view */
    uint32_t u32;       /*!<  single dword i/o view */
}
/** cache the extended IO values */
CurIOVal;


#define ERR_DETECT             -1
#define ERR_DEMO_RANGE_ONE     1
#define ERR_DEMO_RANGE_MULTI   2

#define I2cExpAddr0 ((int)(0x43*2))
/**
 * Expander 1 i2c address[7..0] format
 */
#define I2cExpAddr1 ((int)(0x42*2))
/** @} XNUCLEO53L0A1_I2CExpanders*/


/**
 * GPIO monitor pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPMR    0x10
/**
 * STMPE1600 GPIO set pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPSR    0x12
/**
 * STMPE1600 GPIO set pin direction register
 * 16 bit register LSB at lowest offset
 */
#define GPDR    0x14
extern uint8_t light;
extern uint8_t real_data;
extern void VL53L0X_Init(void);
extern int VL53L0X_Detect(void);
extern void VL53L0X_Setup(RangingConfig_e rangingConfig);
extern void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange);
extern uint8_t VL53L0X_StartRange(void);
extern void VL53L0X_GetCalibration(void);
extern uint8_t VL53L0X_GetRange(void);
extern uint8_t light_data_filter(uint8_t d);


#endif
