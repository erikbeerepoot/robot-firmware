//
// Created by Erik Beerepoot 😊 on 2019-02-02.
//

#ifndef ROBOT_STM32F401_SERIALCOMMUNICATIONS_H
#define ROBOT_STM32F401_SERIALCOMMUNICATIONS_H


#include <hal2/ISerialCommunications.h>
#include "../../../../../../../../usr/local/Caskroom/gcc-arm-embedded/7-2018-q2-update/gcc-arm-none-eabi-7-2018-q2-update/arm-none-eabi/include/c++/7.3.1/cstdint"

#include "../../../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

class SerialCommunications: ISerialCommunications {
public:
    SerialCommunications(UART_HandleTypeDef *uart);

    bool transmitBytes(const uint8_t *buffer, int length, int timeoutMs = 50);

    bool receiveBytes(uint8_t *buffer, uint16_t length);

private:
    UART_HandleTypeDef *uart;
};


#endif //ROBOT_STM32F401_SERIALCOMMUNICATIONS_H
