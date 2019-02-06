//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//

#ifndef ROBOT_STM32F401_IPACKETPARSER_H
#define ROBOT_STM32F401_IPACKETPARSER_H

typedef struct ParseResult {
    Command command;
    std::string payload;

    explicit ParseResult(Command command = Command::Unknown, std::string payload = "") {
        this->command = command;
        this->payload = std::move(payload);
    }
} ParseResult;

class IPacketParser {
public:
    /**
    * Parse a command packet
    * @param buffer The buffer with the data to parse
    * @param length The number of bytes in the buffer
    * @return true if a packet was parsed successfully, false otherwise
    */
    virtual ParseResult parse(const char *buffer, int length) = 0;

    /**
     * Parses the packet header out of the given buffer,
     * returning the command contained therein.
     * @return The command contained in the packet header,
     * or Command::Unknown if not found.
     */
    virtual Command parseHeader(const char *buffer, int length) = 0;

    /**
     * Parse the packet checksum out of a 'buffer'.
     * @return The parsed checksum.
     */
    virtual long parseChecksum(const char *buffer) = 0;

    /**
     * Find the {start, end) of a packet in the buffer.
     * Note: this will attempt to find the first packet.
     * @return A {start, end} pair, {-1, -1} if not found.
     */
    virtual std::pair<int, int> findPacketBoundaries(const char *buffer, int length) = 0;

    /**
     * Given a 'buffer' of size 'length' and (optionally) a section divider,
     * try to find the end index of the current section by looking for the divider.
     * @return The end index of the section, or -1 if no divider found.
     */
    virtual int findNextSectionTerminator(const char *buffer, int length, char sectionDivider = '\n') = 0;
};
#endif //ROBOT_STM32F401_IPACKETPARSER_H
