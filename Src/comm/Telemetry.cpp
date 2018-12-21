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

RobotState Telemetry::service(RobotState state) {
    transmitTelemetry(state);
    return state;
}

void Telemetry::init() {

}

void Telemetry::terminate() {

}

bool Telemetry::receiveCommand(MotionState lastState, MotionState *newState){

    if(HAL_UART_Receive(uart, &rxBuffer[0], 1, 2) != HAL_OK){
        *newState = lastState;
        return false;
    }

    switch(rxBuffer[0]){
        case 's':
            *newState = STRAIGHT;
            break;
        case 'b':
            *newState = REVERSE;
            break;
        case 'h':
            *newState = STOPPED;
            break;
        case 'l':
            *newState = LEFT;
            break;
        case 'r':
            *newState = RIGHT;
            break;
        default:
            *newState = lastState;
            return false;
    }
    return true;
}

void Telemetry::transmitTelemetry(RobotState state){

}
