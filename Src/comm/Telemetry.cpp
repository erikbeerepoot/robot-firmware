//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#include "../Inc/comm/Telemetry.hpp"

UART_HandleTypeDef *uart;

uint8_t rxBuffer[16];
uint8_t txBuffer[16];

Telemetry::Telemetry(UART_HandleTypeDef *telemetryUART){
    uart = telemetryUART;
}

void Telemetry::service(RobotState state) {
    transmitTelemetry(state);
}

void Telemetry::init() {

}

void Telemetry::terminate() {

}

RobotState Telemetry::receiveCommand(RobotState lastState){

    if(HAL_UART_Receive(uart, &rxBuffer[0], 1, 2) != HAL_OK){
        return lastState;
    }

    switch(rxBuffer[0]){
        case 's':
            return STRAIGHT;
        case 'b':
            return REVERSE;
        case 'h':
            return STOPPED;
        case 'l':
            return LEFT;
        case 'r':
            return RIGHT;
        default:
            return lastState;
    }
}

void Telemetry::transmitTelemetry(RobotState state){

}
