//
// Created by Erik Beerepoot 😊 on 2018-12-20.
//

#include <stm32f4xx_hal.h>
#include "../../Inc/sensing/MaxbotixRangefinder.h"
#include <cstdlib>

int32_t count;
int32_t lastCount;
uint32_t delta;

MaxbotixRangefinder::MaxbotixRangefinder(){
    count = 0;
    lastCount = 0;
    delta = 0;
}

float MaxbotixRangefinder::computeDistance(uint32_t delta){
    static double tickFrequency = 43945.2125;
    double pulseWidth = ((float)delta / tickFrequency);
    return (float)(pulseWidth / 0.00005787401575);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    count = htim->Instance->CNT;
    delta = (uint32_t)abs(((int)count - lastCount));
    lastCount = count;
}

float MaxbotixRangefinder::readDistance() {
    return computeDistance(delta);
}

