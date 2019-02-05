//
// Created by Erik Beerepoot 😊 on 2019-02-03.
//
#include <catch.hpp>
#include <fakeit.h>

#include <communication/impl/PacketParser.h>

TEST_CASE("Parsing packets should work", "[Packet Parsing]") {
    char validPacket[] = "CV\n0.1000.100\n12345678\n\n";
    long validChecksum = 305419896l;

    fakeit::Mock<IChecksumCalculator> mockChecksumCalculator;
    fakeit::When(Method(mockChecksumCalculator, computeChecksum)).Return((uint32_t)validChecksum);

    SECTION("A header should be parsed correctly.") {
        auto parser = PacketParser(&mockChecksumCalculator.get());

        auto command = parser.parseHeader(validPacket, 2);

        REQUIRE(command == Command::SetVelocity);
    }

    SECTION("A faulty header should return Command::Unknown.") {
        auto parser = PacketParser(&mockChecksumCalculator.get());
        char faultyHeader[] = "AB\n";

        auto command = parser.parseHeader(faultyHeader, 2);

        REQUIRE(command == Command::Unknown);
    }

    SECTION("The checksum should be extracted correctly.") {
        auto parser = PacketParser(&mockChecksumCalculator.get());

        auto checksum = parser.parseChecksum(validPacket + 14);

        REQUIRE(checksum == validChecksum);
    }

    SECTION("Packet boundaries should be found correctly (full valid packet).") {
        auto parser = PacketParser(&mockChecksumCalculator.get());

        auto boundaries = parser.findPacketBoundaries(validPacket, (int) (strlen(validPacket)));

        REQUIRE(boundaries.first == 0);
        REQUIRE(boundaries.second == strlen(validPacket) - 2);
    }

    SECTION("Packet boundaries should be found correctly (bookended packet).") {
        auto parser = PacketParser(&mockChecksumCalculator.get());
        char buffer[32]; //leave space for 4 bytes on either side
        sprintf(buffer, "XXXX%sXXXX", validPacket);

        auto boundaries = parser.findPacketBoundaries(buffer, (int) (strlen(buffer)));

        REQUIRE(boundaries.first == 4);
        REQUIRE(boundaries.second == (strlen(buffer) - 6));
    }

    SECTION("Should find section terminators correctly.") {
        auto parser = PacketParser(&mockChecksumCalculator.get());

        auto terminatorIndex = parser.findNextSectionTerminator(validPacket, (int) (strlen(validPacket)));
        auto terminatorIndex2 = parser.findNextSectionTerminator(validPacket + terminatorIndex + 1, (int) (strlen(validPacket)));
        auto terminatorIndex3 = parser.findNextSectionTerminator(validPacket + terminatorIndex + terminatorIndex2 + 2, (int) (strlen(validPacket)));

        REQUIRE(terminatorIndex == 2);
        REQUIRE(terminatorIndex2 == 10);
        REQUIRE(terminatorIndex3 == 8);
    }

    SECTION("Packet should be parsed correctly.") {
        auto parser = PacketParser(&mockChecksumCalculator.get());

        auto parseResult = parser.parse(validPacket, (int) (strlen(validPacket)));

        REQUIRE(parseResult.command == Command::SetVelocity);
        REQUIRE(parseResult.payload == "0.1000.100");
    }

    SECTION("Incomplete packet should not be parsed (missing start).") {
        auto parser = PacketParser(&mockChecksumCalculator.get());

        auto parseResult = parser.parse(validPacket + 2, (int) (strlen(validPacket)) - 2);

        REQUIRE(parseResult.command == Command::Unknown);
        REQUIRE(parseResult.payload == "");
    }

    SECTION("Incomplete packet should not be parsed (missing end).") {
        auto parser = PacketParser(&mockChecksumCalculator.get());

        auto parseResult = parser.parse(validPacket, (int) (strlen(validPacket)) - 3);

        REQUIRE(parseResult.command == Command::Unknown);
        REQUIRE(parseResult.payload == "");
    }

    SECTION("Complete packet bookended by other data should be parsed. ") {
        auto parser = PacketParser(&mockChecksumCalculator.get());
        char buffer[32]; //leave space for 4 bytes on either side
        sprintf(buffer, "XXXX%sXXXX", validPacket);

        auto parseResult = parser.parse(validPacket, (int)(strlen(buffer)));

        REQUIRE(parseResult.command == Command::SetVelocity);
        REQUIRE(parseResult.payload == "0.1000.100");
    }
}