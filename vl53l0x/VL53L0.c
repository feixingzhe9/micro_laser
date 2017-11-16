#include "VL53L0.h"
#include "stm32f103xb.h"
#include "global.h"
#include "vl53l0x_types.h"

int LeakyFactorFix8 = (int)( 0.6 *256);

static VL53L0X_Dev_t VL53L0XDevs;

VL53L0X_RangingMeasurementData_t RangingMeasurementData;

int VL53L0X_Detect(void) 
{
    uint16_t Id = 0;
    uint8_t status = VL53L0X_ERROR_NONE;
    uint8_t VhvSettings;
    uint8_t PhaseCal;	
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    VL53L0XDevs.I2cDevAddr = 0x52;
    VL53L0XDevs.Present = 0;
    delay_ms(2);
    Id = 0;

    /* Set I2C standard mode (400 KHz) before doing the first register access */
    status = single_write_I2C0(VL53L0XDevs.I2cDevAddr, 0x88, 0x00);			

    /* Try to read one register using default 0x52 address */
    if (status == VL53L0X_ERROR_NONE)
    {
          status = multi_read_I2C0(VL53L0XDevs.I2cDevAddr, VL53L0X_REG_IDENTIFICATION_MODEL_ID, (uint8_t *)&Id,2);	
    }

    if (status) 
    {
         printf("Read id fail\n");
    }
                                                    
    if (Id == 0xAAEE) 
    {
          status = VL53L0X_DataInit(&VL53L0XDevs);
          if(status)
          {
               printf("VL53L0X_DataInit fail\n");					  
          }
          
          status=VL53L0X_StaticInit(&VL53L0XDevs);
          if( status )
          {
               printf("VL53L0X_StaticInit failed\n");
          }
 
          status = VL53L0X_PerformRefCalibration(&VL53L0XDevs, &VhvSettings, &PhaseCal);
          if( status )
          {
               printf("VL53L0X_PerformRefCalibration failed\n");
          }
          
          status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs, &refSpadCount, &isApertureSpads);
          if( status )
          {
               printf("VL53L0X_PerformRefSpadManagement failed\n");
          }

          status = VL53L0X_SetDeviceMode(&VL53L0XDevs, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode

          if( status )
          {
                printf("VL53L0X_SetDeviceMode failed\n");
          }

          status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
          if( status )
          {
                 printf("VL53L0X_SetLimitCheckEnable failed\n");
          }

          status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
          if( status )
          {
                printf("VL53L0X_SetLimitCheckEnable failed\n");
          }
                                                  
          VL53L0XDevs.Present = 1;

          //printf("VL53L0X %d Present and initiated to final 0x%x\n", VL53L0XDevs.Id, VL53L0XDevs.I2cDevAddr);
    }
    else 
    {
          printf("unknown ID %x\n",Id);
          status = 1;
    }
		
    return status;
}



/******************************************************************************/
#define VL53L0X_REG_CALIBRATION_OFFSET            (0x10)
#define VL53L0X_REG_CALIBRATION_XTALK             (0x20)
FixPoint1616_t CalDistanceMilliMeter;
int32_t  pOffsetMicroMeter=0;
int32_t _pOffsetMicroMeter_1=0,_pOffsetMicroMeter_2=0,CAL_pOffsetMicroMeter=0;
FixPoint1616_t XTalkCalDistance;
FixPoint1616_t pXTalkCompensationRateMegaCps;
int32_t OffsetCalibrationDataMicroMeter;
uint8_t XTalkCompensationEnable=1;
uint32_t OffsetMicroMeter = 0;
FixPoint1616_t XTalkCompensationRateMegaCps = 0;
void VL53L0X_GetCalibration(void)
{
    uint8_t result[4] = {0};
	uint8_t result1[4] = {0};

	status = multi_read_I2C0(VL53L0XDevs.I2cDevAddr, VL53L0X_REG_CALIBRATION_OFFSET, result,4);	
	OffsetMicroMeter = (result[0]<<24)+(result[1]<<16)+(result[2]<<8)+(result[3]);
#if DEBUG_PRAMA
	printf("%d\n", OffsetMicroMeter);
#endif
	VL53L0X_SetOffsetCalibrationDataMicroMeter(&VL53L0XDevs, OffsetMicroMeter);
	//VL53L0X_PerformOffsetCalibration(&VL53L0XDevs, 100*65536, &OffsetMicroMeter);
			
	status = multi_read_I2C0(VL53L0XDevs.I2cDevAddr, VL53L0X_REG_CALIBRATION_XTALK, result1,4);			
	XTalkCompensationRateMegaCps = (FixPoint1616_t)(result1[0]<<24)+(result1[1]<<16)+(result1[2]<<8)+(result1[3]);
	VL53L0X_SetXTalkCompensationRateMegaCps(&VL53L0XDevs, XTalkCompensationRateMegaCps);	  
#if DEBUG_PRAMA
	printf("%d\n", XTalkCompensationRateMegaCps);
#endif
	//VL53L0X_PerformXTalkCalibration(&VL53L0XDevs, 100*65536, &XTalkCompensationRateMegaCps);

}



/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
void VL53L0X_Setup(RangingConfig_e rangingConfig)
{
      uint8_t status;
      uint8_t VhvSettings;
      uint8_t PhaseCal;
      FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
      FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
      uint32_t timingBudget = 33000;
      uint8_t preRangeVcselPeriod = 14;
      uint8_t finalRangeVcselPeriod = 10;

      if( 1 == VL53L0XDevs.Present)
      {   
          /* Ranging configuration */
          switch(rangingConfig) 
          {
          case LONG_RANGE:
                  signalLimit = (FixPoint1616_t)(0.10*65536);
                  sigmaLimit = (FixPoint1616_t)(60*65536);
                  timingBudget = 33000;
                  preRangeVcselPeriod = 18;
                  finalRangeVcselPeriod = 14;
                  break;
          
          case HIGH_ACCURACY:
                  signalLimit = (FixPoint1616_t)(0.25*65536);
                  sigmaLimit = (FixPoint1616_t)(18*65536);
                  timingBudget = 200000;
                  preRangeVcselPeriod = 14;
                  finalRangeVcselPeriod = 10;
                  break;
          
          case HIGH_SPEED:
                  signalLimit = (FixPoint1616_t)(0.25*65536);
                  sigmaLimit = (FixPoint1616_t)(18*65536);
                  timingBudget = 20000;
                  preRangeVcselPeriod = 24;
                  finalRangeVcselPeriod = 10;
                  break;
          default:
                  printf("Not Supported");
          }

          status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
          if( status )
          {
                   printf("VL53L0X_SetLimitCheckValue failed\n");
          }

          status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
          if( status )
          {
                   printf("VL53L0X_SetLimitCheckValue failed\n");
          }

          status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs,  timingBudget);
          if( status )
          {
                   printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
          }

          status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
          if( status )
          {
                   printf("VL53L0X_SetVcselPulsePeriod failed\n");
          }

          status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
          if( status )
          {
                   printf("VL53L0X_SetVcselPulsePeriod failed\n");
          }

          status = VL53L0X_PerformRefCalibration(&VL53L0XDevs, &VhvSettings, &PhaseCal);
          if( status )
          {
                   printf("VL53L0X_PerformRefCalibration failed\n");
          }

          //int32_t OffsetMicroMeter = 0;
          //uint32_t XTalkCompensationRateMegaCps = 0;
          //VL53L0X_PerformOffsetCalibration(&VL53L0XDevs, 100*65536, &OffsetMicroMeter);
          //VL53L0X_PerformXTalkCalibration(&VL53L0XDevs, 100*65536, &XTalkCompensationRateMegaCps);

          //VL53L0X_GetCalibration();
          VL53L0XDevs.LeakyFirst = 1;
     }
}



/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange)
{
    if( pRange->RangeStatus == 0 )
	{
        if( pDev->LeakyFirst )			
		{
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }
        else
		{
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }
    else
	{
        pDev->LeakyFirst = 1;
    }
}


/**
 * Implement the ranging demo with all modes managed through the blue button (short and long press)
 * This function implements a while loop until the blue button is pressed
 * @param UseSensorsMask Mask of any sensors to use if not only one present
 * @param rangingConfig Ranging configuration to be used (same for all sensors)
 */
uint8_t VL53L0X_StartRange(void)
{
    uint8_t status;
	
    /* Start ranging until blue button is pressed */
    if(1 == VL53L0XDevs.Present)
    {
        /* Call All-In-One blocking API function */
        status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs,&RangingMeasurementData);
    }
	
    return status;
}



uint8_t VL53L0X_GetRange(void)
{
        uint8_t rtn;
        uint16_t val = 0;
        uint8_t light_val = 0;
        uint8_t status = VL53L0X_ERROR_NONE;
        uint8_t RangeStatus = 0;
        VL53L0X_RangingMeasurementData_t rangle_data;
	
        if(1 == VL53L0XDevs.Present)
        {				
            status = VL53L0X_GetRangingMeasurementData(&VL53L0XDevs, &rangle_data);
                                      
            if(VL53L0X_ERROR_NONE == status)
           {
            status = VL53L0X_ClearInterruptMask(&VL53L0XDevs, 0);
           }

        if(VL53L0X_ERROR_NONE == status)
        {
#if DEBUG_PRINTF
     printf(" %4d ", rangle_data.RangeMilliMeter);
#endif
              RangeStatus = rangle_data.RangeStatus;
              switch(RangeStatus)
              {
              case 0:
                  val = rangle_data.RangeMilliMeter;
                  if(val >2550)
                  {
                           val = 1200;
                  }
                   
                  val = val/10;
                  light_val = (uint8_t)val;
                  if(light_val < 1)
                  {
                      light_val = 120;
                  }
                  break;
                           
               case 1:
               case 2:
               case 3:					 
               case 4:
                    light_val = 120; 
                    break;
           
                default:
                     light_val = 120;
                     break;							 
                }										
            }
            else
            {
                    light_val = 120;
            }	
	}
	else
	{
		 light_val = 120;
	}
					
	Sensor_SetNewRange(&VL53L0XDevs,&rangle_data);
        
        rtn = light_val;
        
        return (rtn);
}



#define FILTER_BUFF_LEN   (4)
#define FILTER_MOVE_LEN   (FILTER_BUFF_LEN-1)
#define FILTER_LAST_0     (FILTER_BUFF_LEN-1)
#define FILTER_LAST_1     (FILTER_BUFF_LEN-2)
#define FILTER_LAST_2     (FILTER_BUFF_LEN-3)
#define CHANGE_BUFF_LEN   (3)
#define CHANGE_MOVE_LEN   (CHANGE_BUFF_LEN-1)
#define CHANGE_LAST_0     (CHANGE_BUFF_LEN-1)
#define CHANGE_LAST_1     (CHANGE_BUFF_LEN-2)
static uint8_t meas_st = 0; /* 1: 有障碍物； 0：无障碍物 */
static uint8_t change_st = 0;  /* 1: 有跳变； 0：无跳变 */
static uint8_t filter_buff[FILTER_BUFF_LEN] = {120,120,120,120};
static uint8_t change_buff[CHANGE_BUFF_LEN] = {120,120,120};
uint8_t light_data_filter(uint8_t d)
{
	   uint8_t rtn = 0;
       uint8_t i = 0;
     
       /* 更新buff */	
	   for(i = 0; i < FILTER_MOVE_LEN; i++)
	   {
		   filter_buff[i] = filter_buff[i+1];
	   }
	   filter_buff[i] = d;
		 
		 /* 状态判断 */
	   switch(meas_st)
	   {
	   case 0: /* 无障碍物 */
		   if((filter_buff[FILTER_LAST_0] != 120) && (filter_buff[FILTER_LAST_1] != 120) && (filter_buff[FILTER_LAST_2] != 120))
		   {
				meas_st = 1;
				rtn = filter_buff[FILTER_LAST_0];
		   }
		   else
		   {
				rtn = 120;
		   }
		   break;
			 
	  case 1: /* 有障碍物 */
		   if((filter_buff[FILTER_LAST_0] == 120) && (filter_buff[FILTER_LAST_1] == 120) && (filter_buff[FILTER_LAST_2] == 120))
		   {
				meas_st = 0;		
				rtn = 120;					 
		   }
		   else
		   {
				for(i = FILTER_LAST_0; i >= 0; i--)
				{
					if(filter_buff[i] != 120)
					{
						rtn = filter_buff[i];
						break;
					}
				}
		   }
		   break;
		   
	  default:
		  meas_st = 0;		
		  break;
	 }
		 
#if CHANGE_TEST
	 if(1 == meas_st)
	 {
		 for(i = 0; i < CHANGE_MOVE_LEN; i++)
		 {
			  change_buff[i] = change_buff[i+1];
		 }
		 change_buff[i] = rtn;
		 
		 switch(change_st)
		 {
		 case 0:  /* 无跳变 */
			if(abs(change_buff[CHANGE_LAST_0] - change_buff[CHANGE_LAST_1]) > 5)
			{
				change_st = 1;
				rtn = change_buff[CHANGE_LAST_1];
			}				
			break;
			 
		 case 1:  /* 有跳变 */
			if(abs(change_buff[CHANGE_LAST_0] - change_buff[CHANGE_LAST_1]) <= 5)
			{
				change_st = 0;
				rtn = change_buff[CHANGE_LAST_0];
			}				
			break;
			
		 default:
			change_st = 0; 
			break;
		 }
	 }
#endif
	 
	 return (rtn);
}



