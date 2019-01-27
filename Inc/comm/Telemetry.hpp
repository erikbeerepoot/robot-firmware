//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <common/Task.hpp>
#include <common/RobotState.h>

#include "stm32f4xx_hal.h"

class Telemetry: Task {
public:
    Telemetry(UART_HandleTypeDef *uart);
    RobotState service(RobotState state);
    void init();
    void terminate();

    bool receiveCommand(MotionState lastState, MotionState *newState);

    //TODO: Should this be public? Or should some sort of scan queue exist
    void transmitScan(const char* buffer, int length);
private:
    void transmitTelemetry(RobotState state);

};

#endif //ROBOT_TELEMETRY_H
