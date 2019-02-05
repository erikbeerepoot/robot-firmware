// Created by Erik Beerepoot 😊 on 2019-02-03.
#include <catch.hpp>
#include <fakeit.h>

#include <thread>

#include <communication/Telemetry.hpp>

bool waitForCondition(std::condition_variable &cv, int timeoutMs) {
    auto timeout = std::chrono::milliseconds(timeoutMs);
    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    return cv.wait_for(lck, timeout) == std::cv_status::no_timeout;
}

TEST_CASE("Telemetry should work", "[telemetry]") {
    fakeit::Mock<ISerialCommunications> mockSerial;
    fakeit::Mock<IChecksumCalculator> mockCheckCalculator;

    volatile int commandCount = 0;
    volatile int chunkCount = 0;
    std::vector<Command> receivedCommands = {};

    std::condition_variable cv;
    auto callback = [&](Command command,
                        const unsigned char *payload,
                        int length) {
        commandCount++;
        receivedCommands.push_back(command);
        cv.notify_one();
    };

//
//    /**
//     * This test is meant to verify that a single packet arriving in
//     * one piece is parsed correctly by the Telemetry class.
//     */
//    SECTION("Parses a valid velocity command packet succesfully") {
//        auto telemetry = Telemetry(&mockSerial.get(),, callback);
//        // Return valid packet in callback
//        fakeit::When(Method(mockSerial, receiveBytes))
//                .Do([&](uint8_t *buffer, int length) -> bool {
//                    std::cout << "asdf" << std::endl;
//                    if (chunkCount++ < 1) {
//                        strcpy((char *) buffer, "CV\n0.1000.100\n12345678\n\n");
//                        telemetry.rxCallback(24);
//                        return true;
//                    }
//                    return false;
//                });
//
//        telemetry.init();
//
//        waitForCondition(cv, 1000);
//        REQUIRE(commandCount > 0);
//        REQUIRE(receivedCommands.size() == 1);
//        REQUIRE(receivedCommands.front() == SetVelocity);
//    }
}