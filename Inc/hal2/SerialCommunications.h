//
// Created by Erik Beerepoot 😊 on 2019-02-02.
//

#ifndef ROBOT_STM32F401_SERIALCOMMUNICATIONS_H
#define ROBOT_STM32F401_SERIALCOMMUNICATIONS_H


#include <cstdint>

#include "stm32f4xx_hal.h"

class SerialCommunications {
public:
    SerialCommunications(UART_HandleTypeDef *uart);

    bool transmitBytes(const uint8_t *buffer, int length, int timeoutMs = 50);

    bool receiveBytes(uint8_t *buffer, uint16_t length);

private:
    UART_HandleTypeDef *uart;
};


#endif //ROBOT_STM32F401_SERIALCOMMUNICATIONS_H
