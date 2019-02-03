//
// Created by Erik Beerepoot 😊 on 2019-02-02.
//

#ifndef ROBOT_STM32F401_CRC_H
#define ROBOT_STM32F401_CRC_H

#include "stm32f4xx_hal.h"
#include <hal2/IChecksumCalculator.h>

class ChecksumCalculator: IChecksumCalculator {
public:
    ChecksumCalculator(CRC_HandleTypeDef *crc);
    uint32_t computeChecksum(uint32_t *buffer, int length);
};



#endif //ROBOT_STM32F401_CRC_H
