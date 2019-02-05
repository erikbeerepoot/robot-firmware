//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//
#include <communication/impl/PacketParser.h>
#include <utility>
#include <cstdlib>
#include <string>

const char packetSectionSeparator = '\n';

PacketParser::PacketParser(IChecksumCalculator *checksumCalculator) {
    this->checksumCalculator = checksumCalculator;
}

Command PacketParser::parseHeader(const char *buffer, int length) {
    if (length != 2 || buffer[0] != 'C') {
        return Command::Unknown;
    }
    return Command(buffer[1]);
}

long PacketParser::parseChecksum(const char *buffer) {
    return strtol(buffer, nullptr, 16);
}

std::pair<int, int> PacketParser::findPacketBoundaries(const char *buffer, int length) {
    int end = findPacketTerminator(buffer, length);
    int start = findPacketStart(buffer, end);
    return {start, end};
}

ParseResult PacketParser::parse(const char *buffer, int length) {
    // Since we can have multiple (possibly partial) packets in the buffer,
    // attempt to find the first complete packet in the buffer.
    std::pair<int, int> boundaries = findPacketBoundaries(buffer, length);
    if (boundaries.first == -1
        || boundaries.second == -1
        || boundaries.first > boundaries.second)
        return ParseResult();

    // First 2 characters is the command
    int commandSectionEnd = findNextSectionTerminator(buffer, length);
    if (commandSectionEnd == -1) return ParseResult();
    auto command = parseHeader(buffer, commandSectionEnd);

    // Next section is the payload
    int payloadSectionEnd = findNextSectionTerminator(buffer + commandSectionEnd + 1, length - commandSectionEnd);
    if (payloadSectionEnd == -1) return ParseResult();
    auto payload = std::string(buffer + commandSectionEnd + 1, buffer + commandSectionEnd + payloadSectionEnd + 1);

    // Finally, the tail section is the checksum
    auto checksum = parseChecksum(buffer + commandSectionEnd + payloadSectionEnd + 1);
    auto computedChecksum = checksumCalculator->computeChecksum(payload);
    if(checksum != computedChecksum) return ParseResult();

    return ParseResult(command, payload);
}


/***************************
 ****  Private methods  ****
 ***************************/

int PacketParser::findPacketTerminator(const char *buffer, int length) {
    int index = 0;
    do {
        if (buffer[index] == '\n' && buffer[index + 1] == '\n') return index;
    } while (index++ < length);
    return -1;
}

int PacketParser::findPacketStart(const char *buffer, int length) {
    int index = length;
    do {
        if (parseHeader(&buffer[index - 1], 2) != Command::Unknown) return index - 1;
    } while (index-- > 1);
    return -1;
}

int PacketParser::findNextSectionTerminator(const char *buffer, int length, char sectionDivider) {
    int index = 0;

    while (buffer[index] != sectionDivider && index < length) index++;
    if (buffer[index] != sectionDivider)
        return -1;

    return index;
}