#include "global.h"
#include <stdio.h>

#define STM32F103XXX    stm32f103xb.h
uint8_t ultra_test = 0;

uint8_t time_count = 0;

uint8_t count_copy = 0;

uint8_t sensor_status[SENSOR_NUM] = {0};

uint8_t func_set = DATA_FUNC;