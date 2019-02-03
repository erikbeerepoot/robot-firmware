//
// Created by Erik Beerepoot 😊 on 2019-02-02.
//

#include "hal2/impl/SerialCommunications.h"

SerialCommunications::SerialCommunications(UART_HandleTypeDef *uart){
    this->uart = uart;
}

bool SerialCommunications::transmitBytes(const uint8_t *buffer, int length, int timeoutMs){
    HAL_UART_Transmit(uart, (uint8_t *) buffer, (uint16_t) length, timeoutMs);
}
bool SerialCommunications::receiveBytes(uint8_t *buffer, uint16_t length){
    HAL_UART_Receive_IT(uart, buffer, length);
}
