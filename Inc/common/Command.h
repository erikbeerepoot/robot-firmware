//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//

#ifndef ROBOT_STM32F401_COMMAND_H
#define ROBOT_STM32F401_COMMAND_H

/**
 * Defines the commands we can receive
 */
enum class Command: unsigned char {
    Unknown = 0,
    Stop = 'K',
    SetScanMode = 'S',
    SetTelemetryMode = 'T',
    SetVelocity = 'V'
};

#endif //ROBOT_STM32F401_COMMAND_H
