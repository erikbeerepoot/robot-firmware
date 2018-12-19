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
    void service(RobotState state);
    void init();
    void terminate();

    RobotState receiveCommand(RobotState lastState);
private:
    void transmitTelemetry(RobotState state);
};

#endif //ROBOT_TELEMETRY_H
