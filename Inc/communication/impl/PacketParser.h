//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//

#ifndef ROBOT_STM32F401_PACKETPARSER_H
#define ROBOT_STM32F401_PACKETPARSER_H

#include <hal2/IChecksumCalculator.h>
#include <common/Command.h>
#include <utility>
#include <string>

typedef struct ParseResult {
    Command command;
    std::string payload;

    ParseResult(Command command = Command::Unknown, std::string payload = "") {
        this->command = command;
        this->payload = std::move(payload);
    }
} ParseResult;

class PacketParser {
public:
    PacketParser(IChecksumCalculator *checksumCalculator);

    /**
    * Parse a command packet
    * @param buffer The buffer with the data to parse
    * @param length The number of bytes in the buffer
    * @return true if a packet was parsed successfully, false otherwise
    */
    ParseResult parse(const char *buffer, int length);

    /**
     * Parses the packet header out of the given buffer,
     * returning the command contained therein.
     * @return The command contained in the packet header,
     * or Command::Unknown if not found.
     */
    Command parseHeader(const char *buffer, int length);

    /**
     * Parse the packet checksum out of a 'buffer'.
     * @return The parsed checksum.
     */
    long parseChecksum(const char *buffer);

    /**
     * Find the {start, end) of a packet in the buffer.
     * Note: this will attempt to find the first packet.
     * @return A {start, end} pair, {-1, -1} if not found.
     */
    std::pair<int, int> findPacketBoundaries(const char *buffer, int length);

    /**
     * Given a 'buffer' of size 'length' and (optionally) a section divider,
     * try to find the end index of the current section by looking for the divider.
     * @return The end index of the section, or -1 if no divider found.
     */
    int findNextSectionTerminator(const char *buffer, int length, char sectionDivider = '\n');

private:
    /**
     * Given a 'buffer' of size 'length', find the start index of a packet
     * @return The start index of a packet, or -1 if not found
     */
    int findPacketStart(const char *buffer, int length);

    /**
     * Given a 'buffer' of size 'length', find the end index of a packet
     * @return the end index of a packet, or -1 if not found
     */
    int findPacketTerminator(const char *buffer, int length);



    IChecksumCalculator *checksumCalculator;
};

#endif //ROBOT_STM32F401_PACKETPARSER_H
