//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <common/Task.hpp>
#include <common/RobotState.h>
#include <utility>
#include <functional>
#include <hal2/ISerialCommunications.h>
#include <hal2/IChecksumCalculator.h>

enum Command {
    Unknown = -1,
    Stop = 0,
    SetScanMode,
    SetTelemetryMode,
    SetVelocity
};

class Telemetry : Task {
public:
    /**
     * Constructor
     * @param uart The UART module to use for transmission & reception
     * @param telemetryCRC The CRC module to use for checksum calculation
     * @param commandCallback The callback to invoke when a command is received
     */
    Telemetry(ISerialCommunications *serial,
              IChecksumCalculator *checksumCalculator,
              const std::function<void(Command, const unsigned char *payload, int payloadLength)> &commandCallback);

    /**
     * Perform repeated work ("servicing").
     */

    RobotState service(RobotState state) override;

    /**
     * Perform setup tasks
     */
    void init() override;

    /**
     * Method invoked when data has been received
     * @param length The number of bytes that were received
     */
    void rxCallback(int length);

    //TODO: Move this to private
    void transmitScan(const char *buffer, int length);

private:
    /**
     * Transmit telemetry over serial to listener
     * @param state
     */
    void transmitTelemetry(RobotState state);

    /**
     * Parse a command packet
     * @param buffer The buffer with the data to parse
     * @param length The number of bytes in the buffer
     * @return true if a packet was parsed successfully, false otherwise
     */
    int parseCommandPacket(const unsigned char *buffer, int length);

    Command parseCommand(const unsigned char *buffer, int length);

    int parseIncomingChunk(const unsigned char *buffer, int length);

    long parseChecksum(const unsigned char *buffer, int length);

    uint32_t computeChecksum(const unsigned char *buffer, int length);

    int processCommandPacket(const unsigned char *buffer, int length);

    int findPacketStart(const unsigned char *buffer, int length);

    int findPacketTerminator(const unsigned char *buffer, int length);

    bool areBoundariesValid(std::pair<int, int> boundaries, int packetLength);

    /// Callback invoked when a command is parsed
    std::function<void(Command, const unsigned char *payload, int payloadLength)> commandCallback;

    ISerialCommunications *serial;
    IChecksumCalculator *checksumCalculator;
};

#endif //ROBOT_TELEMETRY_H
