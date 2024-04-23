#ifndef __POWERIO_H
#define __POWERIO_H

#include "stm32f1xx.h"
#include "main.h"

typedef enum
{
    POWER_ENABLE = GPIO_PIN_SET,
    POWER_DISENABLE = GPIO_PIN_RESET
} POWER_State;

void app_power_Ctrl(POWER_State State);
void app_power_Check(void);

extern float v_Battery;

#endif