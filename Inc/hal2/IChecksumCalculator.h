//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//

#ifndef ROBOT_STM32F401_ICHECKSUMCALCULATOR_H
#define ROBOT_STM32F401_ICHECKSUMCALCULATOR_H

#include <cstdint>
#include <string>

class IChecksumCalculator {
public:
    virtual uint32_t computeChecksum(const std::string &bytes) = 0;
};

#endif //ROBOT_STM32F401_ICHECKSUMCALCULATOR_H
