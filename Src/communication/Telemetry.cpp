//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//


#include <communication/Telemetry.hpp>
#include <utility>
#include <cstring>
#include <cstdlib>
#include <iostream>

int rxBufferIndex = 0;
const int rxBufferSize = 128;
const uint16_t chunkSize = 28;

uint8_t rxBuffer[rxBufferSize];
uint8_t workingBuffer[chunkSize];

Telemetry::Telemetry(ISerialCommunications *serial,
                     IPacketParser *packetParser,
                     const std::function<void(Command, const unsigned char *payload, int payloadLength)> &callback) {
    this->serial = serial;
    this->packetParser = packetParser;
    commandCallback = callback;
}

RobotState Telemetry::service(RobotState state) {
    return state;
}

void Telemetry::init() {
    serial->receiveBytes(workingBuffer, chunkSize);
}

ParseResult Telemetry::parseIncomingData(const char *buffer, int length) {
    // Copy the bytes we've just received into the rx buffer (from working buffer)
    memcpy(rxBuffer + rxBufferIndex, buffer, (size_t) length);

    // Now attempt to process a packet out of the rx buffer
    auto result = packetParser->parse((const char*)rxBuffer, rxBufferSize);
    if(result.command == Command::Unknown){
        // If nothing got processed, just increment buffer index
        rxBufferIndex += length;

        // If we'll be exceeding the length of the buffer next
        // time we receive a chunk, discard the entire thing
        if (rxBufferIndex + length >= rxBufferSize) {
            memset(rxBuffer, 0, rxBufferSize);
            rxBufferIndex = 0;
        }
    } else {
        auto boundaries = packetParser->findPacketBoundaries((const char*)rxBuffer, rxBufferSize);
        // We've processed a packet. Move the rest of the buffer up by boundaries.second indices
        memcpy(rxBuffer, rxBuffer + boundaries.second, static_cast<size_t >(rxBufferSize - boundaries.second));
        rxBufferIndex = 0;
    }
    // Keep receiving bytes
    serial->receiveBytes(workingBuffer, chunkSize);
    return result;
}

/*************************
 **** Private methods ****
 *************************/

void Telemetry::transmitTelemetry(RobotState state) {

}

//FIXME: Actually make private
void Telemetry::transmitScan(const char *buffer, int length) {
    serial->transmitBytes((uint8_t *) buffer, length, 50);
}
