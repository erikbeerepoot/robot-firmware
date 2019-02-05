//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <utility>
#include <functional>
#include <common/Command.h>
#include <common/RobotState.h>
#include <common/Task.hpp>

#include <hal2/ISerialCommunications.h>
#include <hal2/IChecksumCalculator.h>

#include <communication/IPacketParser.h>

class Telemetry : Task {
public:
    /**
     * Constructor
     * @param uart The UART module to use for transmission & reception
     * @param telemetryCRC The CRC module to use for checksum calculation
     * @param commandCallback The callback to invoke when a command is received
     */
    Telemetry(ISerialCommunications *serial,
              IPacketParser *packetParser,
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

    int parseIncomingChunk(const unsigned char *buffer, int length);

private:
    /**
     * Transmit telemetry over serial to listener
     * @param state
     */
    void transmitTelemetry(RobotState state);

    /// Callback invoked when a command is parsed
    std::function<void(Command,
                       const unsigned char *payload,
                       int payloadLength
    )>
            commandCallback;

    ISerialCommunications *serial;
    IPacketParser *packetParser;
};

#endif //ROBOT_TELEMETRY_H
