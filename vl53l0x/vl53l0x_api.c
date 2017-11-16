/*******************************************************************************
 Copyright © 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "vl53l0x_api.h"
#include "vl53l0x_tuning.h"
#include "vl53l0x_interrupt_threshold_settings.h"
#include "vl53l0x_api_core.h"
#include "vl53l0x_api_calibration.h"
#include "vl53l0.h"
#include "vl53l0x_api_strings.h"
#include "i2c.h"



VL53L0X_Error VL53L0X_SetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev,
	int32_t OffsetCalibrationDataMicroMeter)
{
			VL53L0X_Error Status = VL53L0X_ERROR_NONE;

			Status = VL53L0X_set_offset_calibration_data_micro_meter(Dev,
				OffsetCalibrationDataMicroMeter);

			return Status;
}


VL53L0X_Error VL53L0X_GetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev,
	int32_t *pOffsetCalibrationDataMicroMeter)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_get_offset_calibration_data_micro_meter(Dev,
		pOffsetCalibrationDataMicroMeter);

	return Status;
}



VL53L0X_Error VL53L0X_DataInit(VL53L0X_DEV Dev)
{
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
		VL53L0X_DeviceParameters_t CurrentParameters;
		uint8_t i;
		uint8_t StopVariable;

		/* Set I2C standard mode */
		if (Status == VL53L0X_ERROR_NONE)
		{
				Status = single_write_I2C0(Dev->I2cDevAddr, 0x88, 0x00);	
		}
			
		VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone, 0);


		/* Default value is 1000 for Linearity Corrective Gain */
		PALDevDataSet(Dev, LinearityCorrectiveGain, 1000);

		/* Dmax default Parameter */
		PALDevDataSet(Dev, DmaxCalRangeMilliMeter, 400);
		PALDevDataSet(Dev, DmaxCalSignalRateRtnMegaCps,(FixPoint1616_t)((0x00016B85))); /* 1.42 No Cover Glass*/

		/* Set Default static parameters
		 *set first temporary values 9.44MHz * 65536 = 618660 */
		VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz, 618660);

		/* Set Default XTalkCompensationRateMegaCps to 0  */
		VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, 0);

		/* Get default parameters */
		Status = VL53L0X_GetDeviceParameters(Dev, &CurrentParameters);
		if (Status == VL53L0X_ERROR_NONE) 
		{
			/* initialize PAL values */
			CurrentParameters.DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
			CurrentParameters.HistogramMode = VL53L0X_HISTOGRAMMODE_DISABLED;
			PALDevDataSet(Dev, CurrentParameters, CurrentParameters);
		}

		/* Sigma estimator variable */
		PALDevDataSet(Dev, SigmaEstRefArray, 200);
		PALDevDataSet(Dev, SigmaEstEffPulseWidth, 900);
		PALDevDataSet(Dev, SigmaEstEffAmbWidth, 500);
		PALDevDataSet(Dev, targetRefRate, 0x0A00); /* 20 MCPS in 9:7 format */

		/* Use internal default settings */
		PALDevDataSet(Dev, UseInternalTuningSettings, 1);
		Status |= single_write_I2C0(Dev->I2cDevAddr, 0x80, 0x01);
		Status |= single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x01);
		Status |= single_write_I2C0(Dev->I2cDevAddr, 0x00, 0x00);
		Status |= single_read_I2C0(Dev->I2cDevAddr, 0x91, &StopVariable);
		PALDevDataSet(Dev, StopVariable, StopVariable);
		Status |= single_write_I2C0(Dev->I2cDevAddr, 0x00, 0x01);
		Status |= single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x00);
		Status |= single_write_I2C0(Dev->I2cDevAddr, 0x80, 0x00);


		/* Enable all check */
		for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) 
		{
			if (Status == VL53L0X_ERROR_NONE)
			{
					Status |= VL53L0X_SetLimitCheckEnable(Dev, i, 1);
			}
			else
			{
					break;
			}
		}

		/* Disable the following checks */
		if (Status == VL53L0X_ERROR_NONE)
			Status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, 0);

		if (Status == VL53L0X_ERROR_NONE)
			Status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);

		if (Status == VL53L0X_ERROR_NONE)
			Status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, 0);

		if (Status == VL53L0X_ERROR_NONE)
			Status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, 0);

		/* Limit default values */
		if (Status == VL53L0X_ERROR_NONE) 
		{
			Status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536));
		}
		if (Status == VL53L0X_ERROR_NONE) 
		{
			Status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(25 * 65536 / 100));
					/* 0.25 * 65536 */
		}

		if (Status == VL53L0X_ERROR_NONE) 
			{
			Status = VL53L0X_SetLimitCheckValue(Dev,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
					(FixPoint1616_t)(35 * 65536));
		}

		if (Status == VL53L0X_ERROR_NONE) 
			{
			Status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(0 * 65536));
		}

		if (Status == VL53L0X_ERROR_NONE) 
			{

			PALDevDataSet(Dev, SequenceConfig, 0xFF);
			Status = single_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);		
			
			/* Set PAL state to tell that we are waiting for call to
			 * VL53L0X_StaticInit */
			PALDevDataSet(Dev, PalState, VL53L0X_STATE_WAIT_STATICINIT);
		}

		if (Status == VL53L0X_ERROR_NONE)
			VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 0);
				
			return Status;
}


VL53L0X_Error VL53L0X_StaticInit(VL53L0X_DEV Dev)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_DeviceParameters_t CurrentParameters = {0};
	uint8_t *pTuningSettingBuffer;
	uint16_t tempword = 0;
	uint8_t tempbyte = 0;
	uint8_t UseInternalTuningSettings = 0;
	volatile uint32_t count = 0;
	uint8_t isApertureSpads = 0;
	uint32_t refSpadCount = 0;
	volatile uint8_t ApertureSpads = 0;
	uint8_t vcselPulsePeriodPCLK;
	uint32_t seqTimeoutMicroSecs;


	Status = VL53L0X_get_info_from_device(Dev, 1);
	 
	/* set the ref spad from NVM */
	count	= (uint32_t)VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
		ReferenceSpadCount);
	ApertureSpads = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
		ReferenceSpadType);
	//printf("count :%d\n", count);
	//printf("ApertureSpads :%d\n", ApertureSpads);

	/* NVM value invalid */
//	if ((ApertureSpads > 1) ||
//		((ApertureSpads == 1) && (count > 12)) ||
//		((ApertureSpads == 0) && (count > 12)))
		Status = VL53L0X_perform_ref_spad_management(Dev, &refSpadCount,
			&isApertureSpads);
//	else
//		Status = VL53L0X_set_reference_spads(Dev, count, ApertureSpads);

	/* Initialize tuning settings buffer to prevent compiler warning. */
	pTuningSettingBuffer = DefaultTuningSettings;

	if (Status == VL53L0X_ERROR_NONE) {
		UseInternalTuningSettings = PALDevDataGet(Dev,
			UseInternalTuningSettings);

		if (UseInternalTuningSettings == 0)
			pTuningSettingBuffer = PALDevDataGet(Dev,
				pTuningSettingsPointer);
		else
			pTuningSettingBuffer = DefaultTuningSettings;

	}

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_load_tuning_settings(Dev, pTuningSettingBuffer);


	/* Set interrupt config to new sample ready */
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetGpioConfig(Dev, 0, 0,
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
		VL53L0X_INTERRUPTPOLARITY_LOW);
	}

	if (Status == VL53L0X_ERROR_NONE) 
	{
			Status = single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x01);
			Status |= multi_read_I2C0(Dev->I2cDevAddr, 0x84, (uint8_t *)&tempword,2);
			Status |= single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x00);			
	}


	if (Status == VL53L0X_ERROR_NONE) {
		VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz,
			VL53L0X_FIXPOINT412TOFIXPOINT1616(tempword));
	}

	/* After static init, some device parameters may be changed,
	 * so update them */
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_GetDeviceParameters(Dev, &CurrentParameters);


	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_GetFractionEnable(Dev, &tempbyte);
		if (Status == VL53L0X_ERROR_NONE)
			PALDevDataSet(Dev, RangeFractionalEnable, tempbyte);

	}

	if (Status == VL53L0X_ERROR_NONE)
		PALDevDataSet(Dev, CurrentParameters, CurrentParameters);


	/* read the sequence config and save it */
	if (Status == VL53L0X_ERROR_NONE) 
	{
			Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &tempbyte);				

		if (Status == VL53L0X_ERROR_NONE)
			PALDevDataSet(Dev, SequenceConfig, tempbyte);

	}


	/* Disable MSRC and TCC by default */
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetSequenceStepEnable(Dev,
					VL53L0X_SEQUENCESTEP_TCC, 0);


	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetSequenceStepEnable(Dev,
		VL53L0X_SEQUENCESTEP_MSRC, 0);


	/* Set PAL State to standby */
	if (Status == VL53L0X_ERROR_NONE)
		PALDevDataSet(Dev, PalState, VL53L0X_STATE_IDLE);



	/* Store pre-range vcsel period */
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_GetVcselPulsePeriod(
			Dev,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE,
			&vcselPulsePeriodPCLK);
	}

	if (Status == VL53L0X_ERROR_NONE) {
			VL53L0X_SETDEVICESPECIFICPARAMETER(
				Dev,
				PreRangeVcselPulsePeriod,
				vcselPulsePeriodPCLK);
	}

	/* Store final-range vcsel period */
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_GetVcselPulsePeriod(
			Dev,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
			&vcselPulsePeriodPCLK);
	}

	if (Status == VL53L0X_ERROR_NONE) {
			VL53L0X_SETDEVICESPECIFICPARAMETER(
				Dev,
				FinalRangeVcselPulsePeriod,
				vcselPulsePeriodPCLK);
	}

	/* Store pre-range timeout */
	if (Status == VL53L0X_ERROR_NONE) {
		Status = get_sequence_step_timeout(
			Dev,
			VL53L0X_SEQUENCESTEP_PRE_RANGE,
			&seqTimeoutMicroSecs);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		VL53L0X_SETDEVICESPECIFICPARAMETER(
			Dev,
			PreRangeTimeoutMicroSecs,
			seqTimeoutMicroSecs);
	}

	/* Store final-range timeout */
	if (Status == VL53L0X_ERROR_NONE) {
		Status = get_sequence_step_timeout(
			Dev,
			VL53L0X_SEQUENCESTEP_FINAL_RANGE,
			&seqTimeoutMicroSecs);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		VL53L0X_SETDEVICESPECIFICPARAMETER(
			Dev,
			FinalRangeTimeoutMicroSecs,
			seqTimeoutMicroSecs);
	}

	return Status;
}



VL53L0X_Error VL53L0X_GetDeviceParameters(VL53L0X_DEV Dev,
	VL53L0X_DeviceParameters_t *pDeviceParameters)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	int i;

	Status = VL53L0X_GetDeviceMode(Dev, &(pDeviceParameters->DeviceMode));

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_GetInterMeasurementPeriodMilliSeconds(Dev,
		&(pDeviceParameters->InterMeasurementPeriodMilliSeconds));


	if (Status == VL53L0X_ERROR_NONE)
		pDeviceParameters->XTalkCompensationEnable = 0;

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_GetXTalkCompensationRateMegaCps(Dev,
			&(pDeviceParameters->XTalkCompensationRateMegaCps));


	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_GetOffsetCalibrationDataMicroMeter(Dev,
			&(pDeviceParameters->RangeOffsetMicroMeters));


	if (Status == VL53L0X_ERROR_NONE) {
		for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
			/* get first the values, then the enables.
			 * VL53L0X_GetLimitCheckValue will modify the enable
			 * flags
			 */
			if (Status == VL53L0X_ERROR_NONE) {
				Status |= VL53L0X_GetLimitCheckValue(Dev, i,
				&(pDeviceParameters->LimitChecksValue[i]));
			} else {
				break;
			}
			if (Status == VL53L0X_ERROR_NONE) {
				Status |= VL53L0X_GetLimitCheckEnable(Dev, i,
				&(pDeviceParameters->LimitChecksEnable[i]));
			} else {
				break;
			}
		}
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_GetWrapAroundCheckEnable(Dev,
			&(pDeviceParameters->WrapAroundCheckEnable));
	}

	/* Need to be done at the end as it uses VCSELPulsePeriod */
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_GetMeasurementTimingBudgetMicroSeconds(Dev,
		&(pDeviceParameters->MeasurementTimingBudgetMicroSeconds));
	}

	return Status;
}

VL53L0X_Error VL53L0X_SetDeviceMode(VL53L0X_DEV Dev, VL53L0X_DeviceModes DeviceMode)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	switch (DeviceMode) {
	case VL53L0X_DEVICEMODE_SINGLE_RANGING:
	case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
	case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
	case VL53L0X_DEVICEMODE_GPIO_DRIVE:
	case VL53L0X_DEVICEMODE_GPIO_OSC:
		/* Supported modes */
		VL53L0X_SETPARAMETERFIELD(Dev, DeviceMode, DeviceMode);
		break;
	default:
		/* Unsupported mode */
		Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
	}

	return Status;
}

VL53L0X_Error VL53L0X_GetDeviceMode(VL53L0X_DEV Dev,
	VL53L0X_DeviceModes *pDeviceMode)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	VL53L0X_GETPARAMETERFIELD(Dev, DeviceMode, *pDeviceMode);

	return Status;
}


VL53L0X_Error VL53L0X_GetFractionEnable(VL53L0X_DEV Dev, uint8_t *pEnabled)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_RANGE_CONFIG, pEnabled);					

	if (Status == VL53L0X_ERROR_NONE)
	{
			*pEnabled = (*pEnabled & 1);
	}

	return Status;
}


VL53L0X_Error VL53L0X_SetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
	uint32_t MeasurementTimingBudgetMicroSeconds)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_set_measurement_timing_budget_micro_seconds(Dev,
		MeasurementTimingBudgetMicroSeconds);


	return Status;
}

VL53L0X_Error VL53L0X_GetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
	uint32_t *pMeasurementTimingBudgetMicroSeconds)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_get_measurement_timing_budget_micro_seconds(Dev,
		pMeasurementTimingBudgetMicroSeconds);

	return Status;
}

VL53L0X_Error VL53L0X_SetVcselPulsePeriod(VL53L0X_DEV Dev,
	VL53L0X_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_set_vcsel_pulse_period(Dev, VcselPeriodType,
		VCSELPulsePeriodPCLK);

	return Status;
}

VL53L0X_Error VL53L0X_GetVcselPulsePeriod(VL53L0X_DEV Dev,
	VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_get_vcsel_pulse_period(Dev, VcselPeriodType,
		pVCSELPulsePeriodPCLK);

	return Status;
}

VL53L0X_Error VL53L0X_SetSequenceStepEnable(VL53L0X_DEV Dev,
	VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t SequenceConfig = 0;
	uint8_t SequenceConfigNew = 0;
	uint32_t MeasurementTimingBudgetMicroSeconds;


	Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &SequenceConfig);			


	SequenceConfigNew = SequenceConfig;

	if (Status == VL53L0X_ERROR_NONE) {
		if (SequenceStepEnabled == 1) {

			/* Enable requested sequence step
			 */
			switch (SequenceStepId) {
			case VL53L0X_SEQUENCESTEP_TCC:
				SequenceConfigNew |= 0x10;
				break;
			case VL53L0X_SEQUENCESTEP_DSS:
				SequenceConfigNew |= 0x28;
				break;
			case VL53L0X_SEQUENCESTEP_MSRC:
				SequenceConfigNew |= 0x04;
				break;
			case VL53L0X_SEQUENCESTEP_PRE_RANGE:
				SequenceConfigNew |= 0x40;
				break;
			case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
				SequenceConfigNew |= 0x80;
				break;
			default:
				Status = VL53L0X_ERROR_INVALID_PARAMS;
			}
		} else {
			/* Disable requested sequence step
			 */
			switch (SequenceStepId) {
			case VL53L0X_SEQUENCESTEP_TCC:
				SequenceConfigNew &= 0xef;
				break;
			case VL53L0X_SEQUENCESTEP_DSS:
				SequenceConfigNew &= 0xd7;
				break;
			case VL53L0X_SEQUENCESTEP_MSRC:
				SequenceConfigNew &= 0xfb;
				break;
			case VL53L0X_SEQUENCESTEP_PRE_RANGE:
				SequenceConfigNew &= 0xbf;
				break;
			case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
				SequenceConfigNew &= 0x7f;
				break;
			default:
				Status = VL53L0X_ERROR_INVALID_PARAMS;
			}
		}
	}

	if (SequenceConfigNew != SequenceConfig) {
		/* Apply New Setting */
		if (Status == VL53L0X_ERROR_NONE) 
		{
				Status = single_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfigNew);	
		}
		if (Status == VL53L0X_ERROR_NONE)
			PALDevDataSet(Dev, SequenceConfig, SequenceConfigNew);


		/* Recalculate timing budget */
		if (Status == VL53L0X_ERROR_NONE) {
			VL53L0X_GETPARAMETERFIELD(Dev,
				MeasurementTimingBudgetMicroSeconds,
				MeasurementTimingBudgetMicroSeconds);

			VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev,
				MeasurementTimingBudgetMicroSeconds);
		}
	}


	return Status;
}

VL53L0X_Error sequence_step_enabled(VL53L0X_DEV Dev,
	VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceConfig,
	uint8_t *pSequenceStepEnabled)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	*pSequenceStepEnabled = 0;

	switch (SequenceStepId) {
	case VL53L0X_SEQUENCESTEP_TCC:
		*pSequenceStepEnabled = (SequenceConfig & 0x10) >> 4;
		break;
	case VL53L0X_SEQUENCESTEP_DSS:
		*pSequenceStepEnabled = (SequenceConfig & 0x08) >> 3;
		break;
	case VL53L0X_SEQUENCESTEP_MSRC:
		*pSequenceStepEnabled = (SequenceConfig & 0x04) >> 2;
		break;
	case VL53L0X_SEQUENCESTEP_PRE_RANGE:
		*pSequenceStepEnabled = (SequenceConfig & 0x40) >> 6;
		break;
	case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
		*pSequenceStepEnabled = (SequenceConfig & 0x80) >> 7;
		break;
	default:
		Status = VL53L0X_ERROR_INVALID_PARAMS;
	}

	return Status;
}



VL53L0X_Error VL53L0X_GetSequenceStepEnables(VL53L0X_DEV Dev,
	VL53L0X_SchedulerSequenceSteps_t *pSchedulerSequenceSteps)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t SequenceConfig = 0;
	

	Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &SequenceConfig);			

	if (Status == VL53L0X_ERROR_NONE) {
		Status = sequence_step_enabled(Dev,
		VL53L0X_SEQUENCESTEP_TCC, SequenceConfig,
			&pSchedulerSequenceSteps->TccOn);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = sequence_step_enabled(Dev,
		VL53L0X_SEQUENCESTEP_DSS, SequenceConfig,
			&pSchedulerSequenceSteps->DssOn);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = sequence_step_enabled(Dev,
		VL53L0X_SEQUENCESTEP_MSRC, SequenceConfig,
			&pSchedulerSequenceSteps->MsrcOn);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = sequence_step_enabled(Dev,
		VL53L0X_SEQUENCESTEP_PRE_RANGE, SequenceConfig,
			&pSchedulerSequenceSteps->PreRangeOn);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = sequence_step_enabled(Dev,
		VL53L0X_SEQUENCESTEP_FINAL_RANGE, SequenceConfig,
			&pSchedulerSequenceSteps->FinalRangeOn);
	}

	return Status;
}



VL53L0X_Error VL53L0X_GetSequenceStepTimeout(VL53L0X_DEV Dev,
	VL53L0X_SequenceStepId SequenceStepId, FixPoint1616_t *pTimeOutMilliSecs)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint32_t TimeoutMicroSeconds;

	Status = get_sequence_step_timeout(Dev, SequenceStepId,
		&TimeoutMicroSeconds);
	if (Status == VL53L0X_ERROR_NONE) {
		TimeoutMicroSeconds <<= 8;
		*pTimeOutMilliSecs = (TimeoutMicroSeconds + 500)/1000;
		*pTimeOutMilliSecs <<= 8;
	}

	return Status;
}



VL53L0X_Error VL53L0X_GetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
	uint32_t *pInterMeasurementPeriodMilliSeconds)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint16_t osc_calibrate_val;
	uint32_t IMPeriodMilliSeconds;


	Status = multi_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_OSC_CALIBRATE_VAL, (uint8_t *)&osc_calibrate_val,2);

	if (Status == VL53L0X_ERROR_NONE) 
	{
			Status = multi_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, (uint8_t *)&IMPeriodMilliSeconds,4);			
	}

	if (Status == VL53L0X_ERROR_NONE) 
	{
		if (osc_calibrate_val != 0) 
		{
			*pInterMeasurementPeriodMilliSeconds =
				IMPeriodMilliSeconds / osc_calibrate_val;
		}
		VL53L0X_SETPARAMETERFIELD(Dev,
			InterMeasurementPeriodMilliSeconds,
			*pInterMeasurementPeriodMilliSeconds);
	}

	return Status;
}



VL53L0X_Error VL53L0X_GetXTalkCompensationEnable(VL53L0X_DEV Dev,
	uint8_t *pXTalkCompensationEnable)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Temp8;

	VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
	*pXTalkCompensationEnable = Temp8;

	return Status;
}

VL53L0X_Error VL53L0X_SetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
	FixPoint1616_t XTalkCompensationRateMegaCps)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Temp8;
	uint16_t LinearityCorrectiveGain;
	uint16_t data;

	VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
	LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

	if (Temp8 == 0) { /* disabled write only internal value */
		VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
			XTalkCompensationRateMegaCps);
	} else {
		/* the following register has a format 3.13 */
		if (LinearityCorrectiveGain == 1000) 
		{
			data = VL53L0X_FIXPOINT1616TOFIXPOINT313(XTalkCompensationRateMegaCps);
		} 
		else 
		{
			data = 0;
		}

	
		Status = double_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, data);

		if (Status == VL53L0X_ERROR_NONE) 
		{
			VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
		}
	}

	return Status;
}




VL53L0X_Error VL53L0X_GetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
	FixPoint1616_t *pXTalkCompensationRateMegaCps)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint16_t Value;
	FixPoint1616_t TempFix1616;

	Status = multi_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, (uint8_t *)&Value,2);			

	if (Status == VL53L0X_ERROR_NONE) {
		if (Value == 0) {
			/* the Xtalk is disabled return value from memory */
			VL53L0X_GETPARAMETERFIELD(Dev,
				XTalkCompensationRateMegaCps, TempFix1616);
			*pXTalkCompensationRateMegaCps = TempFix1616;
			VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
				0);
		} else {
			TempFix1616 = VL53L0X_FIXPOINT313TOFIXPOINT1616(Value);
			*pXTalkCompensationRateMegaCps = TempFix1616;
			VL53L0X_SETPARAMETERFIELD(Dev,
				XTalkCompensationRateMegaCps, TempFix1616);
			VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
				1);
		}
	}

	return Status;
}

//VL53L0X_Error VL53L0X_SetRefCalibration(VL53L0X_DEV Dev, uint8_t VhvSettings,
//	uint8_t PhaseCal)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
//	LOG_FUNCTION_START("");

//	Status = VL53L0X_set_ref_calibration(Dev, VhvSettings, PhaseCal);

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

//VL53L0X_Error VL53L0X_GetRefCalibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings,
//	uint8_t *pPhaseCal)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
//	LOG_FUNCTION_START("");

//	Status = VL53L0X_get_ref_calibration(Dev, pVhvSettings, pPhaseCal);

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

///*
// * CHECK LIMIT FUNCTIONS
// */

//VL53L0X_Error VL53L0X_GetNumberOfLimitCheck(uint16_t *pNumberOfLimitCheck)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
//	LOG_FUNCTION_START("");

//	*pNumberOfLimitCheck = VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS;

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

//VL53L0X_Error VL53L0X_GetLimitCheckInfo(VL53L0X_DEV Dev, uint16_t LimitCheckId,
//	char *pLimitCheckString)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

//	LOG_FUNCTION_START("");

//	Status = VL53L0X_get_limit_check_info(Dev, LimitCheckId,
//		pLimitCheckString);

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

//VL53L0X_Error VL53L0X_GetLimitCheckStatus(VL53L0X_DEV Dev, uint16_t LimitCheckId,
//	uint8_t *pLimitCheckStatus)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
//	uint8_t Temp8;

//	LOG_FUNCTION_START("");

//	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
//		Status = VL53L0X_ERROR_INVALID_PARAMS;
//	} else {

//		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
//			LimitCheckId, Temp8);

//		*pLimitCheckStatus = Temp8;

//	}

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

VL53L0X_Error VL53L0X_SetLimitCheckEnable(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	uint8_t LimitCheckEnable)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	FixPoint1616_t TempFix1616 = 0;
	uint8_t LimitCheckEnableInt = 0;
	uint8_t LimitCheckDisable = 0;
	uint8_t Temp8;


	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
		Status = VL53L0X_ERROR_INVALID_PARAMS;
	} else {
		if (LimitCheckEnable == 0) {
			TempFix1616 = 0;
			LimitCheckEnableInt = 0;
			LimitCheckDisable = 1;

		} else {
			VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				LimitCheckId, TempFix1616);
			LimitCheckDisable = 0;
			/* this to be sure to have either 0 or 1 */
			LimitCheckEnableInt = 1;
		}

		switch (LimitCheckId) {

		case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
				LimitCheckEnableInt);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
				Status = double_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, VL53L0X_FIXPOINT1616TOFIXPOINT97(TempFix1616));			
			    break;

		case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
				LimitCheckEnableInt);

			break;

		case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
				LimitCheckEnableInt);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:

			Temp8 = (uint8_t)(LimitCheckDisable << 1);
			Status = VL53L0X_UpdateByte(Dev,
				VL53L0X_REG_MSRC_CONFIG_CONTROL,
				0xFE, Temp8);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:

			Temp8 = (uint8_t)(LimitCheckDisable << 4);
			Status = VL53L0X_UpdateByte(Dev,
				VL53L0X_REG_MSRC_CONFIG_CONTROL,
				0xEF, Temp8);

			break;


		default:
			Status = VL53L0X_ERROR_INVALID_PARAMS;

		}

	}

	if (Status == VL53L0X_ERROR_NONE) {
		if (LimitCheckEnable == 0) {
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
				LimitCheckId, 0);
		} else {
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
				LimitCheckId, 1);
		}
	}

	return Status;
}

VL53L0X_Error VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	uint8_t *pLimitCheckEnable)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Temp8;
	

	if (LimitCheckId >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
		Status = VL53L0X_ERROR_INVALID_PARAMS;
		*pLimitCheckEnable = 0;
	} else {
		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
			LimitCheckId, Temp8);
		*pLimitCheckEnable = Temp8;
	}

	return Status;
}

VL53L0X_Error VL53L0X_SetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	FixPoint1616_t LimitCheckValue)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Temp8;


	VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId,
		Temp8);

	if (Temp8 == 0) { /* disabled write only internal value */
		VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
			LimitCheckId, LimitCheckValue);
	} else {

		switch (LimitCheckId) {

		case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
				LimitCheckValue);
			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
				Status = double_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
				VL53L0X_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));					
			    break;

		case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
				LimitCheckValue);

			break;

		case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

			/* internal computation: */
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
				LimitCheckValue);

			break;

		case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
		case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
				Status = double_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT,
				VL53L0X_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));	
			break;

		default:
			Status = VL53L0X_ERROR_INVALID_PARAMS;

		}

		if (Status == VL53L0X_ERROR_NONE) {
			VL53L0X_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				LimitCheckId, LimitCheckValue);
		}
	}

	return Status;
}

VL53L0X_Error VL53L0X_GetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	FixPoint1616_t *pLimitCheckValue)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t EnableZeroValue = 0;
	uint16_t Temp16;
	FixPoint1616_t TempFix1616;


	switch (LimitCheckId) {

	case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
		/* internal computation: */
		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
			VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
			  Status = multi_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint8_t *)&Temp16,2);				
				if (Status == VL53L0X_ERROR_NONE)
						TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16); EnableZeroValue = 1;
				break;

	case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
		/* internal computation: */
		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
			VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
		/* internal computation: */
		VL53L0X_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
			VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
	case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
			Status = multi_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, (uint8_t *)&Temp16,2);	
		 if (Status == VL53L0X_ERROR_NONE)
			TempFix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(Temp16);
		  EnableZeroValue = 0;
		break;

	default:
		Status = VL53L0X_ERROR_INVALID_PARAMS;

	}

	if (Status == VL53L0X_ERROR_NONE) {

		if (EnableZeroValue == 1) {

			if (TempFix1616 == 0) {
				/* disabled: return value from memory */
				VL53L0X_GETARRAYPARAMETERFIELD(Dev,
					LimitChecksValue, LimitCheckId,
					TempFix1616);
				*pLimitCheckValue = TempFix1616;
				VL53L0X_SETARRAYPARAMETERFIELD(Dev,
					LimitChecksEnable, LimitCheckId, 0);
			} else {
				*pLimitCheckValue = TempFix1616;
				VL53L0X_SETARRAYPARAMETERFIELD(Dev,
					LimitChecksValue, LimitCheckId,
					TempFix1616);
				VL53L0X_SETARRAYPARAMETERFIELD(Dev,
					LimitChecksEnable, LimitCheckId, 1);
			}
		} else {
			*pLimitCheckValue = TempFix1616;
		}
	}

	return Status;

}

 

VL53L0X_Error VL53L0X_GetWrapAroundCheckEnable(VL53L0X_DEV Dev,
	uint8_t *pWrapAroundCheckEnable)
{
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
		uint8_t data;
	
		Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &data);			

		if (Status == VL53L0X_ERROR_NONE) 
		{
			PALDevDataSet(Dev, SequenceConfig, data);
			if (data & (0x01 << 7))
				*pWrapAroundCheckEnable = 0x01;
			else
				*pWrapAroundCheckEnable = 0x00;
		}
		
		if (Status == VL53L0X_ERROR_NONE) 
		{
			VL53L0X_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable,
				*pWrapAroundCheckEnable);
		}

		return Status;
}



/* Group PAL Measurement Functions */
VL53L0X_Error VL53L0X_PerformSingleMeasurement(VL53L0X_DEV Dev)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_DeviceModes DeviceMode;

	/* Get Current DeviceMode */
	Status = VL53L0X_GetDeviceMode(Dev, &DeviceMode);

	/* Start immediately to run a single ranging measurement in case of
	 * single ranging or single histogram */
	if (Status == VL53L0X_ERROR_NONE
		&& DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING)
		Status = VL53L0X_StartMeasurement(Dev);

	
	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_measurement_poll_for_completion(Dev);


	/* Change PAL State in case of single ranging or single histogram */
	if (Status == VL53L0X_ERROR_NONE
		&& DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING)
		PALDevDataSet(Dev, PalState, VL53L0X_STATE_IDLE);


	return Status;
}

//VL53L0X_Error VL53L0X_PerformSingleHistogramMeasurement(VL53L0X_DEV Dev,
//	VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;
//	LOG_FUNCTION_START("");

//	/* not implemented on VL53L0X */

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

VL53L0X_Error VL53L0X_PerformRefCalibration(VL53L0X_DEV Dev, uint8_t *pVhvSettings,
	uint8_t *pPhaseCal)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_perform_ref_calibration(Dev, pVhvSettings,
		pPhaseCal, 1);

	return Status;
}

//VL53L0X_Error VL53L0X_PerformXTalkMeasurement(VL53L0X_DEV Dev,
//	uint32_t TimeoutMs, FixPoint1616_t *pXtalkPerSpad,
//	uint8_t *pAmbientTooHigh)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;
//	LOG_FUNCTION_START("");

//	/* not implemented on VL53L0X */

//	LOG_FUNCTION_END(Status);
//	return Status;
//}


VL53L0X_Error VL53L0X_SetXTalkCompensationEnable(VL53L0X_DEV Dev,
	uint8_t XTalkCompensationEnable)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	FixPoint1616_t TempFix1616;
	uint16_t LinearityCorrectiveGain;


	LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

	if ((XTalkCompensationEnable == 0)
		|| (LinearityCorrectiveGain != 1000)) {
		TempFix1616 = 0;
	} else {
		VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
			TempFix1616);
	}

	/* the following register has a format 3.13 */
	Status = double_write_I2C0(Dev->I2cDevAddr,
	VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
		VL53L0X_FIXPOINT1616TOFIXPOINT313(TempFix1616));
	
	if (Status == VL53L0X_ERROR_NONE) {
		if (XTalkCompensationEnable == 0) {
			VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
				0);
		} else {
			VL53L0X_SETPARAMETERFIELD(Dev, XTalkCompensationEnable,
				1);
		}
	}

	return Status;
}


VL53L0X_Error VL53L0X_GetSequenceStepEnable(VL53L0X_DEV Dev,
	VL53L0X_SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t SequenceConfig = 0;

	Status = single_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
		SequenceConfig);

	if (Status == VL53L0X_ERROR_NONE) {
		Status = sequence_step_enabled(Dev, SequenceStepId,
			SequenceConfig, pSequenceStepEnabled);
	}

	return Status;
}

VL53L0X_Error VL53L0X_PerformXTalkCalibration(VL53L0X_DEV Dev,
	FixPoint1616_t XTalkCalDistance,
	FixPoint1616_t *pXTalkCompensationRateMegaCps)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_perform_xtalk_calibration(Dev, XTalkCalDistance,
		pXTalkCompensationRateMegaCps);

	return Status;
}

VL53L0X_Error VL53L0X_PerformOffsetCalibration(VL53L0X_DEV Dev,
	FixPoint1616_t CalDistanceMilliMeter, int32_t *pOffsetMicroMeter)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_perform_offset_calibration(Dev, CalDistanceMilliMeter,
		pOffsetMicroMeter);

	return Status;
}

VL53L0X_Error VL53L0X_CheckAndLoadInterruptSettings(VL53L0X_DEV Dev,
	uint8_t StartNotStopFlag)
{
	uint8_t InterruptConfig;
	FixPoint1616_t ThresholdLow;
	FixPoint1616_t ThresholdHigh;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
		Pin0GpioFunctionality);

	if ((InterruptConfig ==
		VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW) ||
		(InterruptConfig ==
		VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH) ||
		(InterruptConfig ==
		VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT)) {

		Status = VL53L0X_GetInterruptThresholds(Dev,
			VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
			&ThresholdLow, &ThresholdHigh);

		if (((ThresholdLow > 255*65536) ||
			(ThresholdHigh > 255*65536)) &&
			(Status == VL53L0X_ERROR_NONE)) {

			if (StartNotStopFlag != 0) 
			{
				Status = VL53L0X_load_tuning_settings(Dev,
					InterruptThresholdSettings);
			}
			else 
			{
						Status |= single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x04);
						Status |= single_write_I2C0(Dev->I2cDevAddr, 0x70, 0x00);
						Status |= single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x00);
						Status |= single_write_I2C0(Dev->I2cDevAddr, 0x80, 0x00);						
			}

		}


	}

	return Status;

}


VL53L0X_Error VL53L0X_StartMeasurement(VL53L0X_DEV Dev)
{
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
		VL53L0X_DeviceModes DeviceMode;
		uint8_t Byte;
		uint8_t StartStopByte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
		uint32_t LoopNb;

		/* Get Current DeviceMode */
		VL53L0X_GetDeviceMode(Dev, &DeviceMode);

		Status = single_write_I2C0(Dev->I2cDevAddr, 0x80, 0x01);
		Status = single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x01);
		Status = single_write_I2C0(Dev->I2cDevAddr, 0x00, 0x00);
		Status = single_write_I2C0(Dev->I2cDevAddr, 0x91, PALDevDataGet(Dev, StopVariable));
		Status = single_write_I2C0(Dev->I2cDevAddr, 0x00, 0x01);
		Status = single_write_I2C0(Dev->I2cDevAddr, 0xFF, 0x00);
		Status = single_write_I2C0(Dev->I2cDevAddr, 0x80, 0x00);		

		switch (DeviceMode) {
		case VL53L0X_DEVICEMODE_SINGLE_RANGING:
				 Status = single_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSRANGE_START, 0x01);
					Byte = StartStopByte;
					if (Status == VL53L0X_ERROR_NONE) 
					{
							/* Wait until start bit has been cleared */
							LoopNb = 0;
							do 
							{
								if (LoopNb > 0)
								{
										Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSRANGE_START, &Byte);
								}							
								LoopNb = LoopNb + 1;
						} while (((Byte & StartStopByte) == StartStopByte)&& (Status == VL53L0X_ERROR_NONE) && (LoopNb < VL53L0X_DEFAULT_MAX_LOOP));

						if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
								Status = VL53L0X_ERROR_TIME_OUT;
				}
			  break;
				
		case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
			/* Back-to-back mode */

			/* Check if need to apply interrupt settings */
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);
				Status = single_write_I2C0(Dev->I2cDevAddr,		VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);

			if (Status == VL53L0X_ERROR_NONE) 
			{
				/* Set PAL State to Running */
				PALDevDataSet(Dev, PalState, VL53L0X_STATE_RUNNING);
			}
			break;
			
		case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
			/* Continuous mode */
			/* Check if need to apply interrupt settings */
			if (Status == VL53L0X_ERROR_NONE)
				Status = VL53L0X_CheckAndLoadInterruptSettings(Dev, 1);
			Status = single_write_I2C0(Dev->I2cDevAddr,VL53L0X_REG_SYSRANGE_START,VL53L0X_REG_SYSRANGE_MODE_TIMED);
			if (Status == VL53L0X_ERROR_NONE) 
			{
					/* Set PAL State to Running */
					PALDevDataSet(Dev, PalState, VL53L0X_STATE_RUNNING);
			}
			break;
			
		default:
			/* Selected mode not supported */
			Status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
		  break;
		}

		return Status;
}



VL53L0X_Error VL53L0X_GetMeasurementDataReady(VL53L0X_DEV Dev,
	uint8_t *pMeasurementDataReady)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t SysRangeStatusRegister;
	uint8_t InterruptConfig;
	uint32_t InterruptMask;

	InterruptConfig = VL53L0X_GETDEVICESPECIFICPARAMETER(Dev,
		Pin0GpioFunctionality);

	if (InterruptConfig ==
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
		Status = VL53L0X_GetInterruptMaskStatus(Dev, &InterruptMask);
		if (InterruptMask ==
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY)
			*pMeasurementDataReady = 1;
		else
			*pMeasurementDataReady = 0;
	} 
	else 
	{
			Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_RESULT_RANGE_STATUS,&SysRangeStatusRegister);

		  if (Status == VL53L0X_ERROR_NONE) 
			{
				if (SysRangeStatusRegister & 0x01)
					*pMeasurementDataReady = 1;
				else
					*pMeasurementDataReady = 0;
			}
	}

	return Status;
}



VL53L0X_Error VL53L0X_GetRangingMeasurementData(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData)
{
		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
		uint8_t DeviceRangeStatus;
		uint8_t RangeFractionalEnable;
		uint8_t PalRangeStatus;
		uint8_t XTalkCompensationEnable;
		uint16_t AmbientRate;
		FixPoint1616_t SignalRate;
		uint16_t XTalkCompensationRateMegaCps;
		uint16_t EffectiveSpadRtnCount;
		uint16_t tmpuint16;
		uint16_t XtalkRangeMilliMeter;
		uint16_t LinearityCorrectiveGain;
		uint8_t localBuffer[12] = {0};
		VL53L0X_RangingMeasurementData_t LastRangeDataBuffer;
		
		/*
		 * use multi read even if some registers are not useful, result will
		 * be more efficient
		 * start reading at 0x14 dec20
		 * end reading at 0x21 dec33 total 14 bytes to read
		 */
		Status = multi_read_I2C0(Dev->I2cDevAddr, 0x14, localBuffer, 12);			

		
	  if (Status == VL53L0X_ERROR_NONE) 
		{
				pRangingMeasurementData->ZoneId = 0; /* Only one zone */
				pRangingMeasurementData->TimeStamp = 0; /* Not Implemented */

				tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11], localBuffer[10]);
				/* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
				 *(format 11.2) else no fractional */

				pRangingMeasurementData->MeasurementTimeUsec = 0;

				SignalRate = VL53L0X_FIXPOINT97TOFIXPOINT1616(
					VL53L0X_MAKEUINT16(localBuffer[7], localBuffer[6]));
				/* peak_signal_count_rate_rtn_mcps */
				pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate;

				AmbientRate = VL53L0X_MAKEUINT16(localBuffer[9], localBuffer[8]);
				pRangingMeasurementData->AmbientRateRtnMegaCps =
					VL53L0X_FIXPOINT97TOFIXPOINT1616(AmbientRate);

				EffectiveSpadRtnCount = VL53L0X_MAKEUINT16(localBuffer[3],
					localBuffer[2]);
				/* EffectiveSpadRtnCount is 8.8 format */
				pRangingMeasurementData->EffectiveSpadRtnCount =
					EffectiveSpadRtnCount;

				DeviceRangeStatus = localBuffer[0];

				/* Get Linearity Corrective Gain */
				LinearityCorrectiveGain = PALDevDataGet(Dev,
					LinearityCorrectiveGain);

				/* Get ranging configuration */
				RangeFractionalEnable = PALDevDataGet(Dev,
					RangeFractionalEnable);

				if (LinearityCorrectiveGain != 1000) 
				{

					tmpuint16 = (uint16_t)((LinearityCorrectiveGain
						* tmpuint16 + 500) / 1000);

					/* Implement Xtalk */
					VL53L0X_GETPARAMETERFIELD(Dev,
						XTalkCompensationRateMegaCps,
						XTalkCompensationRateMegaCps);
					VL53L0X_GETPARAMETERFIELD(Dev, XTalkCompensationEnable,
						XTalkCompensationEnable);

					if (XTalkCompensationEnable) 
					{
						if ((SignalRate - ((XTalkCompensationRateMegaCps* EffectiveSpadRtnCount) >> 8)) <= 0) 
						{
							if (RangeFractionalEnable)
								XtalkRangeMilliMeter = 8888;
							else
								XtalkRangeMilliMeter = 8888
									<< 2;
						} 
						else 
						{
							XtalkRangeMilliMeter =
							(tmpuint16 * SignalRate)
								/ (SignalRate
								- ((XTalkCompensationRateMegaCps
								* EffectiveSpadRtnCount)
								>> 8));
						}
						tmpuint16 = XtalkRangeMilliMeter;
				}
		}

		if (RangeFractionalEnable) 
		{
			pRangingMeasurementData->RangeMilliMeter =
				(uint16_t)((tmpuint16) >> 2);
			pRangingMeasurementData->RangeFractionalPart =
				(uint8_t)((tmpuint16 & 0x03) << 6);
		} 
		else 
		{
			pRangingMeasurementData->RangeMilliMeter = tmpuint16;
			pRangingMeasurementData->RangeFractionalPart = 0;
		}

		//printf("millim=%d\n", pRangingMeasurementData->RangeMilliMeter);
		/*
		 * For a standard definition of RangeStatus, this should
		 * return 0 in case of good result after a ranging
		 * The range status depends on the device so call a device
		 * specific function to obtain the right Status.
		 */
		Status |= VL53L0X_get_pal_range_status(Dev, DeviceRangeStatus,
			SignalRate, EffectiveSpadRtnCount,
			pRangingMeasurementData, &PalRangeStatus);

		if (Status == VL53L0X_ERROR_NONE)
			pRangingMeasurementData->RangeStatus = PalRangeStatus;

	}

	if (Status == VL53L0X_ERROR_NONE) {
		/* Copy last read data into Dev buffer */
		LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

		LastRangeDataBuffer.RangeMilliMeter =
			pRangingMeasurementData->RangeMilliMeter;
		LastRangeDataBuffer.RangeFractionalPart =
			pRangingMeasurementData->RangeFractionalPart;
		LastRangeDataBuffer.RangeDMaxMilliMeter =
			pRangingMeasurementData->RangeDMaxMilliMeter;
		LastRangeDataBuffer.MeasurementTimeUsec =
			pRangingMeasurementData->MeasurementTimeUsec;
		LastRangeDataBuffer.SignalRateRtnMegaCps =
			pRangingMeasurementData->SignalRateRtnMegaCps;
		LastRangeDataBuffer.AmbientRateRtnMegaCps =
			pRangingMeasurementData->AmbientRateRtnMegaCps;
		LastRangeDataBuffer.EffectiveSpadRtnCount =
			pRangingMeasurementData->EffectiveSpadRtnCount;
		LastRangeDataBuffer.RangeStatus =
			pRangingMeasurementData->RangeStatus;

		PALDevDataSet(Dev, LastRangeMeasure, LastRangeDataBuffer);
	}

	return Status;
}

//VL53L0X_Error VL53L0X_GetMeasurementRefSignal(VL53L0X_DEV Dev,
//	FixPoint1616_t *pMeasurementRefSignal)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
//	LOG_FUNCTION_START("");

//	*pMeasurementRefSignal = PALDevDataGet(Dev, LastSignalRefMcps);

//	LOG_FUNCTION_END(Status);
//	return Status;

//}

//VL53L0X_Error VL53L0X_GetHistogramMeasurementData(VL53L0X_DEV Dev,
//	VL53L0X_HistogramMeasurementData_t *pHistogramMeasurementData)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NOT_IMPLEMENTED;
//	LOG_FUNCTION_START("");

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

VL53L0X_Error VL53L0X_PerformSingleRangingMeasurement(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData)
{
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;

        /* This function will do a complete single ranging
         * Here we fix the mode! */
        Status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

        if (Status == VL53L0X_ERROR_NONE)
                Status = VL53L0X_PerformSingleMeasurement(Dev);


//			if (Status == VL53L0X_ERROR_NONE)
//				Status = VL53L0X_GetRangingMeasurementData(Dev,
//					pRangingMeasurementData);


//			if (Status == VL53L0X_ERROR_NONE)
//				Status = VL53L0X_ClearInterruptMask(Dev, 0);	
		
			return Status;
}

//VL53L0X_Error VL53L0X_SetNumberOfROIZones(VL53L0X_DEV Dev,
//	uint8_t NumberOfROIZones)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

//	LOG_FUNCTION_START("");

//	if (NumberOfROIZones != 1)
//		Status = VL53L0X_ERROR_INVALID_PARAMS;


//	LOG_FUNCTION_END(Status);
//	return Status;
//}

//VL53L0X_Error VL53L0X_GetNumberOfROIZones(VL53L0X_DEV Dev,
//	uint8_t *pNumberOfROIZones)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

//	LOG_FUNCTION_START("");

//	*pNumberOfROIZones = 1;

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

//VL53L0X_Error VL53L0X_GetMaxNumberOfROIZones(VL53L0X_DEV Dev,
//	uint8_t *pMaxNumberOfROIZones)
//{
//	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

//	LOG_FUNCTION_START("");

//	*pMaxNumberOfROIZones = 1;

//	LOG_FUNCTION_END(Status);
//	return Status;
//}

///* End Group PAL Measurement Functions */

VL53L0X_Error VL53L0X_SetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
	VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality,
	VL53L0X_InterruptPolarity Polarity)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t data;

	if (Pin != 0) 
	{
		Status = VL53L0X_ERROR_GPIO_NOT_EXISTING;
	} 
	else if (DeviceMode == VL53L0X_DEVICEMODE_GPIO_DRIVE) 
	{
		if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW)
			data = 0x10;
		else
			data = 1;

		Status = single_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, data);

	} 
	else if (DeviceMode == VL53L0X_DEVICEMODE_GPIO_OSC) 
	{
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xff, 0x01);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0x00, 0x00);

				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xff, 0x00);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0x80, 0x01);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0x85, 0x02);

				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xff, 0x04);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xcd, 0x00);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xcc, 0x11);

				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xff, 0x07);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xbe, 0x00);

				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xff, 0x06);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xcc, 0x09);

				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xff, 0x00);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0xff, 0x01);
				Status |= single_write_I2C0(Dev->I2cDevAddr, 0x00, 0x00);
	

	} 
	else 
	{

		if (Status == VL53L0X_ERROR_NONE) {
			switch (Functionality) {
			case VL53L0X_GPIOFUNCTIONALITY_OFF:
				data = 0x00;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
				data = 0x01;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
				data = 0x02;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
				data = 0x03;
				break;
			case VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY:
				data = 0x04;
				break;
			default:
				Status =
				VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
			}
		}

		if (Status == VL53L0X_ERROR_NONE)
				Status = single_write_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, data);	


		if (Status == VL53L0X_ERROR_NONE) {
			if (Polarity == VL53L0X_INTERRUPTPOLARITY_LOW)
				data = 0;
			else
				data = (uint8_t)(1 << 4);

			Status = VL53L0X_UpdateByte(Dev,
			VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, 0xEF, data);
		}

		if (Status == VL53L0X_ERROR_NONE)
			VL53L0X_SETDEVICESPECIFICPARAMETER(Dev,
				Pin0GpioFunctionality, Functionality);

		if (Status == VL53L0X_ERROR_NONE)
			Status = VL53L0X_ClearInterruptMask(Dev, 0);

	}

	return Status;
}



VL53L0X_Error VL53L0X_GetInterruptThresholds(VL53L0X_DEV Dev,
	VL53L0X_DeviceModes DeviceMode, FixPoint1616_t *pThresholdLow,
	FixPoint1616_t *pThresholdHigh)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint16_t Threshold16;

	/* no dependency on DeviceMode for Ewok */

	Status = multi_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_THRESH_LOW, (uint8_t *)&Threshold16,2);

	/* Need to multiply by 2 because the FW will apply a x2 */
	*pThresholdLow = (FixPoint1616_t)((0x00fff & Threshold16) << 17);

	if (Status == VL53L0X_ERROR_NONE) 
	{
			Status = multi_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_SYSTEM_THRESH_HIGH,			(uint8_t *)&Threshold16,2);
		/* Need to multiply by 2 because the FW will apply a x2 */
		*pThresholdHigh =
			(FixPoint1616_t)((0x00fff & Threshold16) << 17);
	}

	return Status;
}



/* Group PAL Interrupt Functions */
VL53L0X_Error VL53L0X_ClearInterruptMask(VL53L0X_DEV Dev, uint32_t InterruptMask)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t LoopCount;
	uint8_t Byte;

	/* clear bit 0 range interrupt, bit 1 error interrupt */
	LoopCount = 0;
	do {
				Status = single_write_I2C0(Dev->I2cDevAddr,
				VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
				Status |= single_write_I2C0(Dev->I2cDevAddr,
				VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x00);
				Status |= single_read_I2C0(Dev->I2cDevAddr,
				VL53L0X_REG_RESULT_INTERRUPT_STATUS, &Byte);					

		    LoopCount++;
	} while (((Byte & 0x07) != 0x00) && (LoopCount < 3) && (Status == VL53L0X_ERROR_NONE));


	if (LoopCount >= 3)
		Status = VL53L0X_ERROR_INTERRUPT_NOT_CLEARED;

	return Status;
}


VL53L0X_Error VL53L0X_GetInterruptMaskStatus(VL53L0X_DEV Dev,
	uint32_t *pInterruptMaskStatus)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t Byte;
	
	Status = single_read_I2C0(Dev->I2cDevAddr, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &Byte);		
	*pInterruptMaskStatus = Byte & 0x07;

	if (Byte & 0x18)
		Status = VL53L0X_ERROR_RANGE_ERROR;

	return Status;
}




VL53L0X_Error VL53L0X_PerformRefSpadManagement(VL53L0X_DEV Dev,
	uint32_t *refSpadCount, uint8_t *isApertureSpads)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	Status = VL53L0X_perform_ref_spad_management
          (Dev, refSpadCount,
		isApertureSpads);

	return Status;
}

