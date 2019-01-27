//
// Created by Erik Beerepoot 😊 on 2019-01-13.
// Driver for Hokuyo URG-04LX
#include <sensing/Hokuyo.h>
#include <cstring>
#include <comm/usb/usb_host.h>
#include <algorithm>

//NOTE: This could throw an exception that cannot be caught
std::vector<std::string> commandStrings = { // NOLINT(cert-err58-cpp)
        "",
        "SCIP2.0", // switch to SCIP2.0 mode
        "VV", // get version
        "PP", // get measurement params
        "BM", // enable laser
        "QT", // disable laser
        "RS", // reset settings
        "GD", // single scan, 3 bytes
        "MD", // multiple scans, 3 bytes
        "GS", // single scan, 2 bytes
        "MS", // multiple scan, 2 bytes
};


static bool isFinalPacket(const uint8_t *buffer, int length);

static int findLength(const uint8_t *buffer, int length);

static SensorVersion parseVersionResponse(const char *buffer);

static MeasurementParameters parseMeasurementParametersResponse(const char *buffer);

static bool isScan(int length);

static bool isLastScan(const char *buffer, int length);

static int parseStatusHeader(const char *buffer, int length);

static void parseKeyValuePairs(std::function<void(std::string, std::string)> const &callback, const std::string &input);

static int computeSum(const char *buffer, int length);

/********************
 * Public interface *
 ********************/

/**
 * Initialize the sensor class
 * @param maxNumberOfRetries The maximum number of times to retry a failed operation
 */
Hokuyo::Hokuyo(const std::function<void(const char *buffer, int length)> &callback, int maxNumberOfRetries) {
    memset(state.commState.buffer, 0, (size_t) state.commState.bufferSize);

    scanCallback = callback;
    registerRxCallback(std::bind(&Hokuyo::rxCallback, this));
    registerTxCallback(std::bind(&Hokuyo::txCallback, this));

    state.commState.status = Idle;
    state.status = Initializing;
    state.commState.maxNumberOfRetries = maxNumberOfRetries;
    sendCommand(ChangeToScip2);
}

/**
 * Get the current version information for the sensor
 * @return the version structure
 */
SensorVersion *Hokuyo::getVersion() {
    return &state.version;
}

void Hokuyo::streamScans(int startStep,
                         int endStep,
                         int clusterCount,
                         int scanInterval,
                         int scanCount) {
    if (state.status != Ready) return;

    char parameters[13];
    sprintf(parameters, "%04d%04d%02d%d%02d", startStep, endStep, clusterCount, scanInterval, scanCount);
    std::string parameterString(parameters);
    sendCommand(StreamShortRangeScans, parameterString);
}

void Hokuyo::getScan(int startStep,
                     int endStep,
                     int clusterCount) {
    if (state.status != Ready) return;
    //TODO: Need to turn on laser, followed by getScan, followed by turning off the laser
    return;

    char parameters[10];
    sprintf(parameters, "%04d%04d%02d", startStep, endStep, clusterCount);
    std::string parameterString(parameters);
    sendCommand(GetShortRangeScan, parameterString);
}

/*******************
 * Private methods *
 *******************/

/**
 * Send a command to the Hokuyo sensor
 * @param command The command to send
 * @note Sending a command is an asynchronous process, and hence
 * this method does not have a return value.
 */
void Hokuyo::sendCommand(HokuyoCommand command, const std::string &parameterString) {
    if (state.commState.status != Idle && state.commState.status != RetryingTransmission) return;
    state.lastCommand = command;

    // Full command string is cmd + params + line feed
    std::string commandString = commandStrings[command] + parameterString + "\n";

    memset(workingBuffer, 0, 64);
    strcpy((char *) workingBuffer, commandString.c_str());

    // Send command over USB CDC, checking the result
    auto result = usb_transmit_bytes(workingBuffer, (uint16_t) commandString.length());
    state.commState.status = Transmitting;
    if (result != USBH_OK) {
        state.commState.status = RetryingTransmission;
    }
}

void Hokuyo::receiveResponse() {
    state.commState.bufferOffset = 0;
    memset(workingBuffer, 0, 64);

    auto result = usb_receive_bytes(workingBuffer, USB_PACKET_SIZE);
    state.commState.status = Receiving;
    if (result != USBH_OK) {
        state.commState.status = RetryingReception;
    }
}

void Hokuyo::parseResponse() {
    state.commState.status = Idle;

    int length = findLength(state.commState.buffer, COMM_BUFFER_SIZE);
    if (length < 1) {
        // we couldn't find the repeated linefeed signaling the end of packet
        // don't attempt to parse the response
        return;
    }
    if (!validateHeader((const char *) state.commState.buffer)) return;

    switch (state.lastCommand) {
        case ChangeToScip2:
            // Sanity check: obtain version info after switching modes
            sendCommand(GetVersion);
            state.status = Ready;
            break;
        case GetVersion:
            state.version = parseVersionResponse((const char *) state.commState.buffer);
            break;
        case GetMeasurementParameters:
            state.measurementParameters = parseMeasurementParametersResponse((const char *) state.commState.buffer);
            break;
        case DisableLaser:
            break;
        case EnableLaser:
            break;
        case ResetSettings:
            state.status = Initializing;
            break;
        case None:
            // On init, no last command exists
            break;
        case GetLongRangeScan:
            //TODO: Implement parsing
            break;
        case StreamLongRangeScans:
            //TODO: Implement parsing
            break;
        case GetShortRangeScan:
            if (isScan(length)) {
                scanCallback((const char *) state.commState.buffer, length);
            } else {
                receiveResponse();
            }
            break;
        case StreamShortRangeScans:
            if (isScan(length)) {
                scanCallback((const char *) state.commState.buffer, length);

                if(isLastScan((const char *) state.commState.buffer, length)){
                    state.status = Ready;
                } else {
                    receiveResponse();
                }
            } else {
                state.status = Streaming;
                receiveResponse();
            }
            break;
    }
}

bool Hokuyo::validateHeader(const char *buffer) {
    std::string header(buffer);

    std::string commandString = commandStrings[state.lastCommand];
    int statusCode = parseStatusHeader(buffer, header.length());

    std::vector<int> allowedStatusCodes = {0};
    // For scan data, 99 is the correct status code
    if (state.lastCommand == StreamShortRangeScans
        || state.lastCommand == StreamLongRangeScans
        || state.lastCommand == GetShortRangeScan
        || state.lastCommand == GetLongRangeScan) {
        allowedStatusCodes.push_back(99);
    }

    bool commandSuccessful =
            std::find(allowedStatusCodes.begin(), allowedStatusCodes.end(), statusCode) != allowedStatusCodes.end();
    if (commandSuccessful) {
        return header.find(commandString) != std::string::npos;
    } else {
        // UndefinedCommand2 is returned if the sensor was previously put in SCIP2.0 mode
        return state.lastCommand == ChangeToScip2 && statusCode == UndefinedCommand2;
    }
}

/**
 * State machine for processing data from sensor.
 */
void Hokuyo::tick() {
    static int retries = 0;

    switch (state.commState.status) {
        case Idle:
            break;
        case Transmitting:
            break;
        case RetryingTransmission:
            if (retries++ < state.commState.maxNumberOfRetries) {
                //TODO: Create a task manager to execute async tasks
                sendCommand(state.lastCommand);
            }
            break;
        case TransmissionComplete:
            // A transmission is always followed by a response from the sensor
            retries = 0;
            receiveResponse();
            break;
        case Receiving:
            break;
        case RetryingReception:
            if (retries++ < state.commState.maxNumberOfRetries) {
                receiveResponse();
            }
            break;
        case ReceptionComplete:
            retries = 0;
            parseResponse();
            break;
    }
}

/**
 * Called when we have transmitted data to the sensor
 */
void Hokuyo::txCallback() {
    state.commState.status = TransmissionComplete;
}

/**
 * Called when we have received a transmission from the sensor
 */
void Hokuyo::rxCallback() {
    if (isFinalPacket(workingBuffer, USB_PACKET_SIZE)) {
        // Copy data and mark reception as complete
        memcpy(state.commState.buffer + state.commState.bufferOffset, workingBuffer, USB_PACKET_SIZE);
        state.commState.status = ReceptionComplete;
    } else {
        // If the current packet does not contain a double line feed, it is not
        // the final packet. Copy the data into our data buffer and keep going.
        memcpy(state.commState.buffer + state.commState.bufferOffset, workingBuffer, USB_PACKET_SIZE);

        // Keep track of where to copy the next chunk
        state.commState.bufferOffset += USB_PACKET_SIZE;

        auto result = usb_receive_bytes(workingBuffer, USB_PACKET_SIZE);
        if (result != USBH_OK) {
            //TODO: mark reception as a failure
        }
    }
}

/***************************************
 * Utility functions for Hokuyo sensor *
 ***************************************/

/**
 * Checks if this packet is the final packet in a sequence
 * @param buffer The buffer to check for the termination sequence
 * @param length Total number of bytes in the buffer
 * @return true if the buffer contains the termination sequence, false otherwise
 */
static bool isFinalPacket(const uint8_t *buffer, int length) {
    return findLength(buffer, length) > 0;
}

/**
 * Looks for a double linefeed in the given buffer, which signals the end of a packet.
 *
 * @param buffer The buffer to search
 * @param length The total length of the buffer
 * @return The index of the double line feed (i.e. packet length) or 0 if not found
 */
static int findLength(const uint8_t *buffer, int length) {
    for (int index = 0; index < length - 1; index++) {
        //double line feed signals end of data
        if (buffer[index] == '\n' && buffer[index + 1] == '\n') {
            // +1 for the double newline and +1 because we're 0 indexed
            return index + 2;
        }
    }
    return 0;
}


/**
 * Parses the version response from the URG-LX04 sensor
 * @param buffer The buffer containing the data to parse
 * @return The version info structure (fields will be 'UNK' if parsing fails)
 */
static SensorVersion parseVersionResponse(const char *buffer) {
    std::string versionResponse(buffer);

    // Strip the header & focus on the payload
    unsigned int payloadStartIndex = versionResponse.find("VEND");
    std::string payload = versionResponse.substr(payloadStartIndex);

    SensorVersion version;
    parseKeyValuePairs([&](std::string key, std::string value) {
        if (key.find("VEND") < key.length()) {
            version.vendor = value;
        } else if (key.find("PROD") < key.length()) {
            version.productInfo = value;
        } else if (key.find("FIRM") < key.length()) {
            version.firmwareVersion = value;
        } else if (key.find("PROT") < key.length()) {
            version.protocolVersion = value;
        } else if (key.find("SERI") < key.length()) {
            version.serialNumber = value;
        }
    }, payload);
    return version;
}

/**
 * Parses the measurement parameters response into the measurement parameters structure.
 * @param buffer the response received from the sensor
 * @return the measurement parameters (fields will be 'UNK' if parsing fails)
 */
static MeasurementParameters parseMeasurementParametersResponse(const char *buffer) {
    std::string versionResponse(buffer);

    // Strip the header & focus on the payload
    unsigned int payloadStartIndex = versionResponse.find("MODL");
    std::string payload = versionResponse.substr(payloadStartIndex);

    MeasurementParameters parameters;
    parseKeyValuePairs([&](std::string key, std::string value) {
        if (key.find("MODL") < key.length()) {
            parameters.sensorModel = value;
        } else if (key.find("DMIN") < key.length()) {
            parameters.minimumMeasurement = std::stoi(value);
        } else if (key.find("DMAX") < key.length()) {
            parameters.maximumMeasurement = std::stoi(value);
        } else if (key.find("ARES") < key.length()) {
            parameters.angularResolution = std::stoi(value);
        } else if (key.find("AMIN") < key.length()) {
            parameters.minimumStep = std::stoi(value);
        } else if (key.find("AMAX") < key.length()) {
            parameters.maximumStep = std::stoi(value);
        } else if (key.find("AFRT") < key.length()) {
            parameters.frontStep = std::stoi(value);
        } else if (key.find("SCAN") < key.length()) {
            parameters.defaultMotorSpeed = std::stoi(value);
        }
    }, payload);
    return parameters;
}

/**
 * Helper to parse key-value pairs out of the response from the sensor
 * @param callback The callback to invoke for each parsed key-value pair
 * @param input The packet payload converted to a string
 */
static void
parseKeyValuePairs(std::function<void(std::string, std::string)> const &callback, const std::string &input) {
    std::string remaining = input;
    while (remaining.length() > 0) {
        unsigned int separatorIndex = remaining.find(':');
        unsigned int semiColonIndex = remaining.find(';');

        // If either index exceeds the string length, that means they are missing.
        // In this case, this version response is invalid and we exit early.
        if (separatorIndex > remaining.length() ||
            separatorIndex == std::string::npos ||
            semiColonIndex > remaining.length() ||
            semiColonIndex == std::string::npos)
            break;

        // Parse out KV pair and invoke processing callback to process it
        std::string key = remaining.substr(0, separatorIndex);
        std::string value = remaining.substr(separatorIndex + 1, semiColonIndex - separatorIndex - 1);
        callback(key, value);

        // only retain the non-processed part of the string
        remaining = remaining.substr(semiColonIndex + 1);
    }
}

/**
 * Computes the checksum for the given data buffer
 * @param buffer The buffer of data to compute the checksum for
 * @param length The number of bytes in the buffer
 * @return the resulting checksum
 */
static int computeSum(const char *buffer, int length) {
    int sum = 0;
    for (int index = 0; index < length; ++index) {
        sum += buffer[index];
    }
    return (sum & 0x3f) + '0';

}

/**
 * Parse the status header for a given sensor response, return the status code
 * @param buffer The buffer hoding the data to parse
 * @return The status code, or -1 for an invalid header
 */
static int parseStatusHeader(const char *buffer, int length) {
    int statusHeaderStartIndex = 0;
    int statusHeaderEndIndex = 0;

    // Find first line feed, that will be the start of the status header
    while (buffer[statusHeaderStartIndex] != '\n' && statusHeaderStartIndex <= length) statusHeaderStartIndex++;
    if (statusHeaderStartIndex > length) return -1;

    statusHeaderEndIndex = ++statusHeaderStartIndex;

    // The next line feed signals the end of the status header
    while (buffer[statusHeaderEndIndex] != '\n' && statusHeaderEndIndex <= length) statusHeaderEndIndex++;
    if (statusHeaderEndIndex > length) return -1;

    // Verify we have the correct length
    if (statusHeaderEndIndex - statusHeaderStartIndex != 3) return -1;

    int status = ((buffer[statusHeaderStartIndex] - '0') * 10) + (buffer[statusHeaderStartIndex + 1] - '0');
    int sum = buffer[statusHeaderStartIndex + 2];

    int computedSum = computeSum(buffer + statusHeaderStartIndex, 2);
    if (sum != computedSum) {
        return -1;
    }
    return status;
}

/**
 * Is this a scan or just the confirmation of the scan command?
 * @param length The number of bytes in this scan response
 * @return true if this is likely to be a scan rather than a scan command
 */
static bool isScan(int length) {
    return length > 21;
}


/**
 * Returns true if this scan is the last scan in a series
 * @param buffer The data buffer to parse the scan number out of
 * @param length The lenght of the buffer
 * @return true, if this is the last scan, false otherwise.
 */
static bool isLastScan(const char *buffer, int length) {
    std::string actualCommandString(buffer, 2);

    //This check only applies to streaming scans
    //TODO: Worth actually keeping the command around to compare and using it to verify more of the command?
    if (actualCommandString != commandStrings[StreamLongRangeScans] &&
        actualCommandString != commandStrings[StreamShortRangeScans])
        return false;



    // Find the end of the command header
    int commandHeaderEndIndex = 0;
    while (buffer[commandHeaderEndIndex] != '\n' && commandHeaderEndIndex <= length) commandHeaderEndIndex++;
    if (commandHeaderEndIndex >= length || commandHeaderEndIndex < 1) {
        // the second condition above prevents accessing elements before the buffer start
        // but this should not occur in practice. Still, better safe than sorry.
        return false;
    }

    // Take the two bytes before the end to be the current scan number
    int scanCount = (buffer[commandHeaderEndIndex - 2] - '0') * 10 + (buffer[commandHeaderEndIndex - 1] - '0');
    return scanCount == 0;
}