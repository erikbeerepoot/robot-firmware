//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#include "../../Inc/sensing/SharpRangefinder.h"

// minimum voltage is 0.3v
int minVoltageValue = 1229;

double inverseDistanceSlope = ((0.225)/37);

SharpRangefinder::SharpRangefinder(ADC_HandleTypeDef *adc, uint32_t channel){
    this->adc = adc;
    this->channel = channel;
}

float SharpRangefinder::readDistance(){
    HAL_ADC_Start(adc);
    HAL_ADC_PollForConversion(adc, 100);

    uint32_t value = HAL_ADC_GetValue(adc);

    HAL_ADC_Stop(adc);
    return 40 - (float)(((float)value/4096)*3.3 - 0.8)*(25);
}