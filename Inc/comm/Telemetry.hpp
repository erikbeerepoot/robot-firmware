//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <common/Task.hpp>
#include <common/RobotState.h>

#include "stm32f4xx_hal.h"

enum Command {
    Unknown = -1,
    Stop = 0,
    SetScanMode,
    SetTelemetryMode,
    SetVelocity
};

class Telemetry: Task {
public:
    Telemetry(UART_HandleTypeDef *uart, CRC_HandleTypeDef *telemetryCRC);
    RobotState service(RobotState state) override;
    void init() override;
    void terminate() override;
    bool receiveCommand(MotionState lastState, MotionState *newState);

    //TODO: Should this be public? Or should some sort of scan queue exist
    void transmitScan(const char* buffer, int length);
private:
    void transmitTelemetry(RobotState state);
    void parseCommandPacket(const char *buffer, int length);
    uint32_t parseChecksum(const char* buffer, int length);
    Command parseCommand(const char *buffer, int length);

};

#endif //ROBOT_TELEMETRY_H
