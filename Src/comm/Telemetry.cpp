//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//


#include <comm/Telemetry.hpp>
#include <utility>
#include <cstring>
#include <cstdlib>
#include "../Inc/comm/Telemetry.hpp"


UART_HandleTypeDef *uart;
CRC_HandleTypeDef *crc;

int rxBufferIndex = 0;
const int rxBufferSize = 128;
const uint16_t chunkSize = 28;
const char packetSectionSeparator = '\n';

uint8_t rxBuffer[rxBufferSize];
uint8_t workingBuffer[chunkSize];


Telemetry::Telemetry(UART_HandleTypeDef *telemetryUART, CRC_HandleTypeDef *telemetryCRC, const std::function<void(Command, const unsigned char *payload, int payloadLength)> &callback) {
    uart = telemetryUART;
    crc = telemetryCRC;
    commandCallback = callback;
}

RobotState Telemetry::service(RobotState state) {
    receiveCommand();
    return state;
}

void Telemetry::init() {}

void Telemetry::terminate() {}

void Telemetry::receiveCommand() {
    HAL_UART_Receive_IT(uart, workingBuffer, chunkSize);
}

void Telemetry::transmitScan(const char *buffer, int length) {
    HAL_UART_Transmit(uart, (uint8_t *) buffer, (uint16_t) length, HAL_MAX_DELAY);
}

void Telemetry::rxCallback(int length){
    parseIncomingChunk(workingBuffer, length);
    receiveCommand();
}


/*******************
 * Private methods *
 *******************/

void Telemetry::transmitTelemetry(RobotState state) {

}

std::pair<int, int> findPacketBoundaries(const unsigned char *buffer, int length, char separator) {
    int endIndex = 0;
    while (buffer[endIndex++] != separator && endIndex < length);
    if (endIndex >= length) {
        // invalid
        return {-1, -1};
    }
    return {0, endIndex - 1};
}

Command Telemetry::parseCommand(const unsigned char *buffer, int length) {
    if (length != 2 || buffer[0] != 'C') {
        return Unknown;
    }
    switch (buffer[1]) {
        case 'V':
            return SetVelocity;
        case 'S':
            return SetScanMode;
        case 'K': //KILL
            return Stop;
        case 'T':
            return SetTelemetryMode;
        default:
            return Unknown;
    }
}

long Telemetry::parseChecksum(const unsigned char *buffer, int length) {
    return strtol((const char*)buffer, nullptr, 16);
}

uint32_t Telemetry::computeChecksum(const unsigned char *buffer, int length) {
    return HAL_CRC_Accumulate(crc, (uint32_t *) buffer, (uint32_t) (length / 4));
}

bool Telemetry::areBoundariesValid(std::pair<int, int> boundaries, int packetLength) {
    return boundaries.first >= 0 && boundaries.second < packetLength && boundaries.second > boundaries.first;
}

int Telemetry::findPacketTerminator(const unsigned char *buffer, int length) {
    int index = 0;
    do {
        if (buffer[index] == '\n' && buffer[index + 1] == '\n')
            return index;
    } while (index++ < length);
    return -1;
}

int Telemetry::findPacketStart(const unsigned char *buffer, int length){
    int index = length;
    do {
        if(parseCommand(&buffer[index-1], 2) != Unknown) return index - 1;
    } while(index-- > 1);
    return -1;
}

int Telemetry::parseIncomingChunk(const unsigned char *buffer, int length){
    // Copy the bytes we've just received into the rx buffer (from working buffer)
    memcpy(rxBuffer + rxBufferIndex, buffer, (size_t)length);

    // Now attempt to process a packet out of the rx buffer
    int bytesProcessed = parseCommandPacket(rxBuffer, rxBufferSize);
    if (bytesProcessed > 0) {
        // We've processed a packet.  Clear the buffer
        // and copy remaining bytes into it.
        size_t bytesLeft = (size_t) chunkSize - bytesProcessed;
        memset(rxBuffer, 0, 64);
        memcpy(rxBuffer, workingBuffer + bytesProcessed, bytesLeft);
        rxBufferIndex = bytesLeft;
    } else {
        // If nothing got processed, just increment buffer index
        rxBufferIndex += length;

        // If we'll be exceeding the length of the buffer next
        // time we receive a chunk, discard the entire thing
        if (rxBufferIndex + length >= rxBufferSize) {
            memset(rxBuffer, 0, rxBufferSize);
            rxBufferIndex = 0;
        }
    }
    return 0;
}

int Telemetry::processCommandPacket(const unsigned char* buffer, int length){
    std::pair<int, int> boundaries = findPacketBoundaries(buffer, length, packetSectionSeparator);
    if (!areBoundariesValid(boundaries, length)) {
        return 0;
    }

    // Command is the first two bytes
    Command command = parseCommand(buffer + boundaries.first, boundaries.second - boundaries.first);
    if(command == Unknown){
        return 0;
    }
    //payload boundaries
    std::pair<int, int> payloadBoundaries = findPacketBoundaries(buffer + boundaries.second + 1,
                                                                 length - boundaries.second, packetSectionSeparator);
    // apply offset from first boundaries
    payloadBoundaries.first += boundaries.second + 1;
    payloadBoundaries.second += payloadBoundaries.first;

    //checksum boundaries
    boundaries = findPacketBoundaries(buffer + payloadBoundaries.second + 1, length - payloadBoundaries.second,
                                      packetSectionSeparator);
    // apply offset from payload boundaries
    boundaries.first += payloadBoundaries.second + 1;
    boundaries.second += boundaries.first;

    uint32_t computedChecksum = computeChecksum(buffer + payloadBoundaries.first, payloadBoundaries.second - payloadBoundaries.first - 1);
    long parsedChecksum = parseChecksum(buffer + boundaries.first, boundaries.second - boundaries.first);

    //TODO: Compare checksum
    commandCallback(command, buffer + payloadBoundaries.first, payloadBoundaries.second - payloadBoundaries.first - 1);

    return boundaries.second + 2; // add 2 for the double line feed
}

int Telemetry::parseCommandPacket(const unsigned char *buffer, int length) {
    int terminatorIndex = findPacketTerminator(buffer, length);
    if(terminatorIndex < 1){
        return 0;
    }

    int startIndex = findPacketStart(buffer, terminatorIndex);
    if(startIndex < 0){
        return 0;
    }
    return processCommandPacket(buffer + startIndex, terminatorIndex - startIndex);
}


