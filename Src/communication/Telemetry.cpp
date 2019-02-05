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
    commandCallback = callback;
}

RobotState Telemetry::service(RobotState state) {
    return state;
}

void Telemetry::init() {
    serial->receiveBytes(workingBuffer, chunkSize);
}

void Telemetry::transmitScan(const char *buffer, int length) {
    serial->transmitBytes((uint8_t *) buffer, length, 50);
}

void Telemetry::rxCallback(int length) {
    parseIncomingChunk(workingBuffer, length);
    serial->receiveBytes(workingBuffer, chunkSize);
}


/*************************
 **** Private methods ****
 *************************/

void Telemetry::transmitTelemetry(RobotState state) {

}

int Telemetry::parseIncomingChunk(const unsigned char *buffer, int length) {
    std::cout << "Got chunk: " << std::string((const char *) buffer) << " , length: " << length << std::endl;

    // Copy the bytes we've just received into the rx buffer (from working buffer)
    memcpy(rxBuffer + rxBufferIndex, buffer, (size_t) length);

    // Now attempt to process a packet out of the rx buffer
//    auto result = packetParser->parse(rxBuffer, rxBufferSize);
//    if (result.f > 0) {
//        // We've processed a packet.  Clear the buffer
//        // and copy remaining bytes into it.
//        size_t bytesLeft = (size_t) chunkSize - bytesProcessed;
//        memset(rxBuffer, 0, 64);
//        memcpy(rxBuffer, workingBuffer + bytesProcessed, bytesLeft);
//        rxBufferIndex = (int) bytesLeft;
//    } else {
//        // If nothing got processed, just increment buffer index
//        rxBufferIndex += length;
//
//        // If we'll be exceeding the length of the buffer next
//        // time we receive a chunk, discard the entire thing
//        if (rxBufferIndex + length >= rxBufferSize) {
//            memset(rxBuffer, 0, rxBufferSize);
//            rxBufferIndex = 0;
//        }
//    }
    return 0;
}