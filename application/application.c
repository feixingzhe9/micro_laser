#include "mico.h"
#include "platform.h"
#include "platform_internal.h"
#include "platform_config.h"
#include "board_init.h"
#include "protocol.h"
#include "app_platform.h"
#include "upgrade_flash.h"
#include "can_protocol.h"
#include "platform_tim.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "VL53L0.H"
#include "vl53l0x_def.h"

#define os_PowerBoard_log(format, ...)  custom_log("Light", format, ##__VA_ARGS__)


const  char menu[] =
"\r\n"
"Application for %s,\r\nSOFTWARE_VERSION: %s,\r\nHARDWARE_REVISION: %s\r\n";

void SysLed(void);
void SysLight(void);
void SysSensor(void);
uint8_t light_index = 0;
int main( void )
{
  init_clocks();
  init_architecture();
  init_platform();
 
  os_PowerBoard_log( "System clock = %d Hz",HAL_RCC_GetHCLKFreq() );
  printf ( menu, MODEL, SW_VERSION, HARDWARE_REVISION );
  bsp_Init();
  Platform_Init();

  MicoCanInitialize( MICO_CAN1 );

  I2C0_Init();
  VL53L0X_Detect();
  VL53L0X_Setup(HIGH_SPEED); 
  CanLongBufInit();
  
  for(uint8_t i = 0; i < 10; i++)
  {
      //delay_ms(500);
  }
  
  for(;;)
  {
    SysLight();
    can_protocol_period();
    SysLed();  
  }
}

 
#define LED_PERIOD      500/SYSTICK_PERIOD
uint32_t sys_led_start_time = 0;
void SysLed(void)
{ 
    static uint32_t led_cnt = 0;  
    if(os_get_time() - sys_led_start_time >= LED_PERIOD)
    {
        led_cnt++;
        MicoGpioOutputTrigger(MICO_GPIO_SYS_LED);
        if(led_cnt % 2)
        {
            MicoGpioOutputLow(MICO_GPIO_SYS_LED);
        }
        else
        {
            MicoGpioOutputHigh(MICO_GPIO_SYS_LED);
        }
        sys_led_start_time = os_get_time();
    }
}




#define LIGHT_PERIOD      35/SYSTICK_PERIOD
uint32_t sys_light_time = 0;
static uint8_t light_delay = 0;
uint8_t real_data = 0;
void SysLight(void)
{
    uint8_t light = 0;
    if(os_get_time() - sys_light_time >= LIGHT_PERIOD)
    {
        light_delay++;
        if(light_delay%2)
        {
            VL53L0X_StartRange();
            light = VL53L0X_GetRange();
            real_data = light_data_filter(light);
        }
        else
        {
            light_delay = 0;
            //can_light_send();
        }
        sys_light_time = os_get_time();
    }
}

