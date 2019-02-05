//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//

#ifndef ROBOT_STM32F401_IPACKETPARSER_H
#define ROBOT_STM32F401_IPACKETPARSER_H

class IPacketParser {
    virtual std::pair <Command, std::string> parseCommandPacket(const unsigned char *buffer, int length) = 0;

    virtual Command parseCommand(const unsigned char *buffer, int length) = 0;

    virtual int parseIncomingChunk(const unsigned char *buffer, int length) = 0;

    virtual long parseChecksum(const unsigned char *buffer, int length) = 0;

    virtual uint32_t computeChecksum(const unsigned char *buffer, int length) = 0;

    virtual std::pair <Command, std::string> processCommandPacket(const unsigned char *buffer, int length) = 0;

    virtual int findPacketStart(const unsigned char *buffer, int length) = 0;

    virtual int findPacketTerminator(const unsigned char *buffer, int length) = 0;

    virtual bool areBoundariesValid(std::pair<int, int> boundaries, int packetLength) = 0;
};
#endif //ROBOT_STM32F401_IPACKETPARSER_H
