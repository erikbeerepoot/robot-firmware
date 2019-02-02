//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <common/Task.hpp>
#include <common/RobotState.h>
#include <utility>
#include <functional>

#include "stm32f4xx_hal.h"#inclu

enum Command {
    Unknown = -1,
    Stop = 0,
    SetScanMode,
    SetTelemetryMode,
    SetVelocity
};

class Telemetry : Task {
public:
    Telemetry(UART_HandleTypeDef *uart,
              CRC_HandleTypeDef *telemetryCRC,
              const std::function<void(Command, const unsigned char *payload, int payloadLength)> &commandCallback);

    RobotState service(RobotState state) override;

    void init() override;

    void terminate() override;

    // Reception of packets from remote
    void receiveCommand();


    void rxCallback(int length);

    //TODO: Move this to private
    void transmitScan(const char *buffer, int length);

private:
    void transmitTelemetry(RobotState state);

    /**
     * Parse a command packet
     * @param buffer The buffer with the data to parse
     * @param length The number of bytes in the buffer
     * @return true if a packet was parsed successfully, false otherwise
     */
    int parseCommandPacket(const unsigned char *buffer, int length);

    int parseIncomingChunk(const unsigned char *buffer, int length);

    long parseChecksum(const unsigned char *buffer, int length);

    uint32_t computeChecksum(const unsigned char *buffer, int length);

    Command parseCommand(const unsigned char *buffer, int length);

    bool areBoundariesValid(std::pair<int, int> boundaries, int packetLength);

    int processCommandPacket(const unsigned char *buffer, int length);

    int findPacketStart(const unsigned char *buffer, int length);

    int findPacketTerminator(const unsigned char *buffer, int length);

    std::function<void(Command, const unsigned char *payload, int payloadLength)> commandCallback;

};

#endif //ROBOT_TELEMETRY_H
