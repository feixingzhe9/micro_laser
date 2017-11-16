#include "stm32f103xb.h"
#include <string.h>


/* From Board */
#define EXT_ID_B1    (0x2C0E0280)
#define EXT_ID_B2    (0x2C0E2280)
#define EXT_ID_B3    (0x2C0E4280)
#define EXT_ID_B4    (0x2C0E6280)
#define EXT_ID_B5    (0x2C0E8280)
#define EXT_ID_B6    (0x2C0EA280)
#define EXT_ID_B7    (0x2C0EC280)
#define EXT_ID_B8    (0x2C0EE280)
#define EXT_ID_B9    (0x2C0F0280)
#define EXT_ID_B10    (0x2C0F2280)
#define EXT_ID_B11   (0x2C0F4280)
#define EXT_ID_B12   (0x2C0F6280)
#define EXT_ID_B13   (0x2C0F8280)
//#define EXT_ID_B11   (0x2DEF4280)
//#define EXT_ID_B12   (0x2DEF6280)
//#define EXT_ID_B13   (0x2DEF8280)


/* From Board */
#define EXT_ID_X1    (0x2C0E0280)
#define EXT_ID_X2    (0x2C0E2280)
#define EXT_ID_X3    (0x2C0E4280)
#define EXT_ID_X4    (0x2C0E6280)
#define EXT_ID_X5    (0x2C0E8280)
#define EXT_ID_X6    (0x2C0EA280)
#define EXT_ID_X7    (0x2C0EC280)
#define EXT_ID_X8    (0x2C0EE280)
#define EXT_ID_X9    (0x2C0F0280)
#define EXT_ID_X10    (0x2C0F2280)
#define EXT_ID_X11   (0x2C0F4280)
#define EXT_ID_X12   (0x2C0F6280)
#define EXT_ID_X13   (0x2C0F8280)
//#define EXT_ID_X11   (0x2DEF4280)
//#define EXT_ID_X12   (0x2DEF6280)
//#define EXT_ID_X13   (0x2DEF8280)



#define IDLE_TIME    (46)
#define CAN_RECV_TEST        (0)
#define DEBUG_PRINTF         (0)
#define DEBUG_TIME           (0)
#define DEBUG_CAN            (0)

#define L_NUM				 (13)
#define U_NUM				 (9)
#define H_NUM				 (2)
#define R_NUM				 (4)
#define E_NUM				 (1)

#define SENSOR_NUM   (L_NUM + U_NUM)


//#define RECV_FLASH_LEN (96)
#define RECV_FLASH_LEN (6)
#define WRITE_FLASH_SIZE (2)

#define FLASH_START_ADDRESS     (uint32_t)0x08000000
#define FLASH_END_ADDRESS       (uint32_t)0x0800FFFF
#define FLASH_SIZE              (FLASH_END_ADDRESS - FLASH_START_ADDRESS)
        
#define BOOT_START_ADDRESS      (uint32_t)0x08000400
#define BOOT_END_ADDRESS        (uint32_t)0x08004EFF
#define BOOT_FLASH_SIZE         (BOOT_END_ADDRESS - BOOT_START_ADDRESS)

#define PARA_START_ADDRESS      (uint32_t)0x08004F00
#define PARA_END_ADDRESS        (uint32_t)0x08004FEF
#define PARA_FLASH_SIZE         (PARA_END_ADDRESS - PARA_START_ADDRESS)

#define APP_START_ADDRESS       (uint32_t)0x08005000
#define APP_END_ADDRESS         (uint32_t)0x08009FFF
#define APP_FLASH_SIZE          (APP_END_ADDRESS - APP_START_ADDRESS)
  
#define UPDATE_START_ADDRESS    (uint32_t)0x0800A000
#define UPDATE_END_ADDRESS      (uint32_t)0x0800FFFF
#define UPDATE_FLASH_SIZE       (UPDATE_END_ADDRESS - UPDATE_START_ADDRESS)

#define DATA_FUNC  			0x01
#define VISION_FUNC			0x02
/*******************/
#define SEVEN		//如果define SEVEN，适用于整机7个超声波，且后壳超声波屏蔽

//#define EIGHT		//如果define EIGHT，适用于整机8个超声波，且后壳超声波屏蔽
/******************/  	//若都没define，则适用于8个超声波，且后壳超声波正常使用不屏蔽

extern struct allsensor_t allsensor;

extern uint8_t size_sensor(void);

extern uint8_t time_count;

extern uint8_t count_copy;

extern uint8_t status;

extern uint8_t sensor_status[SENSOR_NUM];

extern uint8_t ultra_test;

extern uint8_t func_set;
