//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//


#include <comm/Telemetry.hpp>
#include <utility>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "../Inc/comm/Telemetry.hpp"

int rxBufferIndex = 0;
const int rxBufferSize = 128;
const uint16_t chunkSize = 28;
const char packetSectionSeparator = '\n';

uint8_t rxBuffer[rxBufferSize];
uint8_t workingBuffer[chunkSize];

Telemetry::Telemetry(ISerialCommunications *serial,
                     IChecksumCalculator *checksumCalculator,
                     const std::function<void(Command, const unsigned char *payload, int payloadLength)> &callback) {
    this->serial = serial;
    this->checksumCalculator = checksumCalculator;
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
