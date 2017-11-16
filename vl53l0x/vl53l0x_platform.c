//?????#include "hal.h"
#include "vl53l0x_platform.h"
#include "i2c.h"
#include "vl53l0.h"
//#include "vl53l0x_api.h"

//#include "stm32xxx_hal.h"
#include <string.h>

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
//#define VL53L0X_OsDelay(...) delay_ms(2)



#ifndef VL53L0X_OsDelay
#   define  VL53L0X_OsDelay(...) (void)0
#endif


//uint8_t _I2CBuffer[64];



VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

		Status = single_read_I2C0(Dev->I2cDevAddr, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;

		Status = single_write_I2C0(Dev->I2cDevAddr, index, data);		
		
done:
    return Status;
}


VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // do nothing
    VL53L0X_OsDelay();
    return status;
}

//end of file
