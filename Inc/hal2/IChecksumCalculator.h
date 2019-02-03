//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//

#ifndef ROBOT_STM32F401_ICHECKSUMCALCULATOR_H
#define ROBOT_STM32F401_ICHECKSUMCALCULATOR_H

#include <cstdint>

class IChecksumCalculator {
public:
    virtual uint32_t computeChecksum(uint32_t *buffer, int length) = 0;
};

#endif //ROBOT_STM32F401_ICHECKSUMCALCULATOR_H
