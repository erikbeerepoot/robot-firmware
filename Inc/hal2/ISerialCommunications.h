//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//

#ifndef ROBOT_STM32F401_SERIALCOMMUNICATIONSINTERFACE_H
#define ROBOT_STM32F401_SERIALCOMMUNICATIONSINTERFACE_H

#include <cstdint>

class ISerialCommunications {
public:
    virtual bool transmitBytes(const uint8_t *buffer, int length, int timeoutMs) = 0;

    virtual bool receiveBytes(uint8_t *buffer, uint16_t length) = 0;
};

#endif //ROBOT_STM32F401_SERIALCOMMUNICATIONSINTERFACE_H
