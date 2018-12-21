//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_SHARPRANGEFINDER_H
#define ROBOT_SHARPRANGEFINDER_H

#include "Proximity.h"

#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>

class SharpRangefinder : Proxmity {
public:
    SharpRangefinder(ADC_HandleTypeDef *adc, uint32_t channel);
    float readDistance() override;
private:
    ADC_HandleTypeDef *adc;
    uint32_t channel;
};

#endif //ROBOT_SHARPRANGEFINDER_H
