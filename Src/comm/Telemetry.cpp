//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//


#include <comm/Telemetry.hpp>
#include <utility>
#include "../Inc/comm/Telemetry.hpp"


UART_HandleTypeDef *uart;
CRC_HandleTypeDef *crc;
uint8_t rxBuffer[16];
uint8_t txBuffer[16];

const char packetSectionSeparator = '\n';

Telemetry::Telemetry(UART_HandleTypeDef *telemetryUART, CRC_HandleTypeDef *telemetryCRC){
    uart = telemetryUART;
    crc = telemetryCRC;
}


RobotState Telemetry::service(RobotState state) {
    transmitTelemetry(state);
    return state;
}

void Telemetry::init() {}
void Telemetry::terminate() {}

bool Telemetry::receiveCommand(MotionState lastState, MotionState *newState){

    if(HAL_UART_Receive(uart, &rxBuffer[0], 16, 1) != HAL_OK){
        // no command received
        *newState = lastState;
        return false;
    } else {
        //
    }
    return false;
}


void Telemetry::transmitTelemetry(RobotState state){

}

void Telemetry::transmitScan(const char* buffer, int length){
    HAL_UART_Transmit(uart, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
}

std::pair<int, int> findPacketBoundaries(const char *buffer, int length, char separator){
    int endIndex = 0;
    while(buffer[endIndex++] != separator && endIndex < length);
    if(endIndex >= length){
        // invalid
        return {-1,-1};
    }
    //FIXME: start index
    return {0, endIndex};
}

Command Telemetry::parseCommand(const char *buffer, int length){
    if(length != 2){
        return Unknown;
    }
    switch(buffer[1]){
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

uint32_t Telemetry::parseChecksum(const char* buffer, int length){
    return HAL_CRC_Calculate(crc, (uint32_t *)buffer, (uint32_t)(length / 4));
}

bool areBoundariesValid(std::pair<int,int> boundaries, int packetLength){
    return boundaries.first > 0 && boundaries.second < packetLength && boundaries.second > boundaries.first;
}

void Telemetry::parseCommandPacket(const char *buffer, int length){
    int startIndex = 0;

    std::pair<int,int> boundaries = findPacketBoundaries(buffer, length, packetSectionSeparator);
    if(!areBoundariesValid(boundaries, length)){
        return;
    }
    Command command = parseCommand(buffer + boundaries.first, boundaries.second - boundaries.first);

    //payload boundaries
    boundaries = findPacketBoundaries(buffer + boundaries.second + 1, length - boundaries.second, packetSectionSeparator);

    //checksum boundaries
    boundaries = findPacketBoundaries(buffer + boundaries.second + 1, length - boundaries.second, packetSectionSeparator);
    int32_t checksum = parseChecksum(buffer + boundaries.first, boundaries.second - boundaries.first);

}


