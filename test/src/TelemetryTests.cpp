// Created by Erik Beerepoot 😊 on 2019-02-03.
#include <catch.hpp>
#include <fakeit.h>

#include <thread>

#include <communication/Telemetry.hpp>
#include <communication/impl/PacketParser.h>

bool waitForCondition(std::condition_variable &cv, int timeoutMs) {
    auto timeout = std::chrono::milliseconds(timeoutMs);
    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    return cv.wait_for(lck, timeout) == std::cv_status::no_timeout;
}

TEST_CASE("Telemetry should work", "[telemetry]") {
    fakeit::Mock<ISerialCommunications> mockSerial;
    fakeit::Mock<IChecksumCalculator> mockChecksumCalculator;
    fakeit::When(Method(mockChecksumCalculator, computeChecksum)).Return((uint32_t) 305419896l);
    fakeit::When(Method(mockSerial, receiveBytes)).AlwaysReturn(true);

    const char validPacket[] = "CV\n0.1000.100\n12345678\n\n";

    auto callback = [&](Command command,
                        const unsigned char *payload,
                        int length) {};

    /**
     * This test is meant to verify that a single packet arriving in
     * one piece is parsed correctly by the Telemetry class.
     */
    SECTION("Parses a valid velocity command packet succesfully") {
        auto packetParser = PacketParser(&mockChecksumCalculator.get());
        auto telemetry = Telemetry(&mockSerial.get(), &packetParser, callback);

        telemetry.init();
        auto result = telemetry.parseIncomingData(validPacket, (int) (strlen(validPacket)));

        REQUIRE(result.command == Command::SetVelocity);
        REQUIRE(result.payload == "0.1000.100");
    }

        /**
         * This test is meant to verify that a packet arriving in consecutive
         * pieces should be put together and parsed correctly.
         */
    SECTION("Parses a broken up but valid velocity command packet succesfully") {
        auto packetParser = PacketParser(&mockChecksumCalculator.get());
        auto telemetry = Telemetry(&mockSerial.get(), &packetParser, callback);

        telemetry.init();
        auto result1 = telemetry.parseIncomingData(validPacket, 10);
        auto result2 = telemetry.parseIncomingData(validPacket + 10, (int) (strlen(validPacket)) - 10);

        REQUIRE(result1.command == Command::Unknown);
        REQUIRE(result1.payload == "");
        REQUIRE(result2.command == Command::SetVelocity);
        REQUIRE(result2.payload == "0.1000.100");
    }

        /**
        * This test is meant to verify that a packet arriving in consecutive
        * pieces should be put together and parsed correctly.
        */
    SECTION("Fails to parse a corrupted packet.") {
        auto packetParser = PacketParser(&mockChecksumCalculator.get());
        auto telemetry = Telemetry(&mockSerial.get(), &packetParser, callback);
        // make the checksum validation fail
        fakeit::When(Method(mockChecksumCalculator, computeChecksum)).Return((uint32_t) 0);

        telemetry.init();
        auto result = telemetry.parseIncomingData(validPacket, (int) (strlen(validPacket)));

        REQUIRE(result.command == Command::Unknown);
        REQUIRE(result.payload == "");
    }
}