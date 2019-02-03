//
// Created by Erik Beerepoot 😊 on 2019-01-13.
//

#ifndef ROBOT_STM32F401_HOKUYO_H
#define ROBOT_STM32F401_HOKUYO_H

#include <stm32f4xx_hal.h>

#include <string>
#include <vector>
#include <functional>

static const int COMM_BUFFER_SIZE = 16384;
static const int USB_PACKET_SIZE = 64;
static const int MAX_NUMBER_OF_RETRIES = 100;

/**
 * Structure to store the version information for the URG sensor
 */
typedef struct SensorVersion {
    std::string vendor = "UNK";
    std::string productInfo = "UNK";
    std::string firmwareVersion = "UNK";
    std::string protocolVersion = "UNK";
    std::string serialNumber = "UNK";
} SensorVersion;

/**
 * Structure to store measurement parameters of the URG sensor
 */
typedef struct MeasurementParameters {
    std::string sensorModel = "UNK";
    int minimumMeasurement = -1;
    int maximumMeasurement = -1;
    int angularResolution = -1;
    int minimumStep = -1;
    int maximumStep = -1;
    int frontStep = -1;
    int defaultMotorSpeed = -1;
} MeasurementParameters;

/**
 * Represents the current communication status
 */
enum CommStatus {
    Idle = 0,
    Transmitting,
    TransmissionComplete,
    Receiving,
    RetryingReception,
    ReceptionComplete,
    RetryingTransmission
};

/**
 * Captures the full communication state (status + rx/tx data)
 */
struct CommState {
    CommStatus status = Idle;

    // Used to store data from UGX
    uint8_t buffer[COMM_BUFFER_SIZE] = {0};
    int bufferSize = COMM_BUFFER_SIZE;
    int bufferOffset = 0;

    // Captures how many times to retry a failed operation
    int maxNumberOfRetries = MAX_NUMBER_OF_RETRIES;
};

/**
 * The status of the sensor. Used to signal when the transition to
 * SCIP 2.0 is done.
 */
enum SensorStatus {
    Initializing = 0,
    Ready,
    Streaming,
    Fault,
};

/**
 * Sensor command definition
 */
enum HokuyoCommand {
    None = 0,
    ChangeToScip2 = 1,
    GetVersion,
    GetMeasurementParameters,
    DisableLaser,
    EnableLaser,
    ResetSettings,
    GetLongRangeScan,
    StreamLongRangeScans,
    GetShortRangeScan,
    StreamShortRangeScans,
};

/**
 * Status codes used by sensor when responding to commands
 */
enum HokuyoCommandResponseStatusCode {
    TxDataError = 'A' - '0',
    BufferOverrun = 'B' - '0',
    ParameterError1 = 'C' - '0',
    UndefinedCommand1 = 'D' - '0',
    UndefinedCommand2 = 'E' - '0',
    ParameterError2 = 'F' - '0',
    StringParameterExceedsMaxLength = 'G' - '0',
    StringParameterInvalidCharacters = 'F' - '0',
    SensorFirmwareUpdating = 'G' - '0',
};

/**
 * Hokuyo Sensor state model
 */
typedef struct HokuyoState {
    CommState commState;
    HokuyoCommand lastCommand = None;
    MeasurementParameters measurementParameters;
    SensorVersion version;
    SensorStatus status = Initializing;
} HokuyoState;


class Hokuyo {
public:
    explicit Hokuyo(const std::function<void(const char* buffer, int length)> &callback, int maxNumberOfRetries = MAX_NUMBER_OF_RETRIES);

    SensorVersion *getVersion();

    // Note: Default parameters below are for UGR-04LX
    void streamScans(int startStep = 44,
                     int endStep = 725,
                     int clusterCount = 1,
                     int scanInterval = 0,
                     int scanCount = 10);

    void getScan(int startStep = 44,
                 int endStep = 725,
                 int clusterCount = 1);

    void tick();

private:
    // Working buffer for USB packets
    unsigned char workingBuffer[512];

    // Represents the current state of the sensor
    HokuyoState state;

    // Function invoked when a scan is received
    std::function<void(const char* buffer, int length)> scanCallback;

    void sendCommand(HokuyoCommand command, const std::string &parameterString = "");

    void receiveResponse();

    void parseResponse();

    bool validateHeader(const char *buffer);

    void rxCallback();

    void txCallback();
};

#endif //ROBOT_STM32F401_HOKUYO_H
