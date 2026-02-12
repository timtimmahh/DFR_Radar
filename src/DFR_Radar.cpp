/**
  * @file       DFR_Radar.h
  * @brief      An Arduino library that makes it easy to configure and use the DFRobot 24GHz millimeter-wave Human Presence Detection sensor (SEN0395)
  * @copyright  Copyright (c) 2010 DFRobot Co. Ltd. (http://www.dfrobot.com)
  *             Copyright (c) 2023 Matthew Clark (https://github.com/MaffooClock)
  * @license    The MIT License (MIT)
  * @authors    huyujie (yujie.hu@dfrobot.com)
  *             Matthew Clark
  * @version    v1.0
  * @date       2023-11-07
  * @url        https://github.com/MaffooClock/DFRobot_Radar
  */

#include <DFR_Radar.h>


DFR_Radar::DFR_Radar(Stream *s)
    : debugSerial(false) {
    sensorUART = s;
    // isConfigured = false;
    stopped = false;
    multiConfig = false;
}

bool DFR_Radar::begin() {
    /* Not sure if I want to impliment this, keeping it for future consideration...

    unsigned long startTime = millis() + startupDelay;

    // Give the sensor time to start up just in case this method is called too soon.
    //
    // There's probably a smarter way to do this.  Factory default configuration will
    // have the sensor dumping out $JYBSS messages once per second, so that could be
    // an easy way to tell that it's "ready".  But if the sensor is configured to send
    // these only when queried, or when an presence event occurs, or if the interval
    // is set too long, then this won't really work.
    //
    // Another way might be to send a `sensorStart` and see if 1) it complains about
    // not being ready, or 2) it responds with "sensor started already" and "Error",
    // or 3) actually starts?

    while( millis() < startTime )
      yield();

    if( !stop() )
      return false;

    // Disable command echoing (less response data that we have to parse through)
    sendCommand( comSetEcho );

    // Disable periodic $JYBSS messages (we will query for them)
    sendCommand( comSetUartOutput );

    if( !saveConfig() )
      return false;

    if( !start() )
      return false;

    isConfigured = true;

    */

    return true;
}

void DFR_Radar::setStream(Stream *s) {
    sensorUART = s;
}

bool DFR_Radar::isReady() const {
    return sensorUART != nullptr;
}

bool DFR_Radar::setDetectionRange(const float rangeStart, const float rangeEnd) {
    if (rangeStart < 0 || rangeStart > 9.45)
        return false;

    if (rangeEnd < 0 || rangeEnd > 9.45)
        return false;

    if (rangeEnd < rangeStart)
        return false;

    char _comSetRange[21] = {0};

#ifdef __AVR__
#ifdef _STDLIB_H_
    char _rangeStart[6] = {0};
    dtostrf(rangeStart, 1, 3, _rangeStart);

    char _rangeEnd[6] = {0};
    dtostrf(rangeEnd, 1, 3, _rangeEnd);

    sprintf(_comSetRange, comSetRange, _rangeStart, _rangeEnd);
#else
    sprintf(_comSetRange, comSetRange, (uint8_t) rangeStart, (uint16_t) rangeEnd);
#endif
#else
    sprintf(_comSetRange, comSetRange, rangeStart, rangeEnd);
#endif

    return setConfig(_comSetRange);
}

bool DFR_Radar::getDetectionRange(float &rangeStart, float &rangeEnd) {
    char _comGetRange[2][7] = {{0}, {0}};
    if (!getConfig<2, 7>(comGetRange, _comGetRange)) {
        if (debugSerial)
            Serial.println("Error getting range");
        return false;
    }

    rangeStart = strtof(_comGetRange[0], nullptr);
    rangeEnd = strtof(_comGetRange[1], nullptr);
    return true;
}

bool DFR_Radar::setSensitivity(const uint8_t level) {
    if (level > 9)
        return false;

    char _comSetSensitivity[17] = {0};
    sprintf(_comSetSensitivity, comSetSensitivity, level);

    return setConfig(_comSetSensitivity);
}

bool DFR_Radar::getSensitivity(uint8_t &level) {
    char _comGetSensitivity[1][2] = {{0}};
    if (!getConfig<1, 2>(comGetSensitivity, _comGetSensitivity)) {
        if (debugSerial)
            Serial.println("Error getting sensitivity");
        return false;
    }

    level = strtol(_comGetSensitivity[0], nullptr, 10);
    return true;
}

bool DFR_Radar::setTriggerLatency(const float confirmationDelay, const float disappearanceDelay) {
    if (confirmationDelay < 0 || confirmationDelay > 100)
        return false;

    if (disappearanceDelay < 0 || disappearanceDelay > 1500)
        return false;

    char _comSetLatency[28] = {0};

#ifdef __AVR__
#ifdef _STDLIB_H_
    char _confirmationDelay[8] = {0};
    dtostrf(confirmationDelay, 3, 3, _confirmationDelay);

    char _disappearanceDelay[9] = {0};
    dtostrf(disappearanceDelay, 4, 3, _disappearanceDelay);

    sprintf(_comSetLatency, comSetLatency, _confirmationDelay, _disappearanceDelay);
#else
    sprintf(_comSetLatency, comSetLatency, (uint8_t) confirmationDelay, (uint16_t) disappearanceDelay);
#endif
#else
    sprintf(_comSetLatency, comSetLatency, confirmationDelay, disappearanceDelay);
#endif

    return setConfig(_comSetLatency);
}

bool DFR_Radar::getTriggerLatency(float &confirmationDelay, float &disappearanceDelay) {
    char _comGetLatency[2][10] = {{0}, {0}};
    if (!getConfig<2, 10>(comGetLatency, _comGetLatency)) {
        if (debugSerial)
            Serial.println("Error getting latency");
        return false;
    }

    confirmationDelay = strtof(_comGetLatency[0], nullptr);
    disappearanceDelay = strtof(_comGetLatency[1], nullptr);
    return true;
}

bool DFR_Radar::setOutputLatency(const float triggerDelay, const float resetDelay) {
    if (triggerDelay < 0 || resetDelay < 0)
        return false;

    // Convert seconds into 25ms units
    const uint32_t _triggerDelay = triggerDelay * 1000 / 25;
    const uint32_t _resetDelay = resetDelay * 1000 / 25;

    if (_triggerDelay > 65535 || _resetDelay > 65535)
        return false;

    char _comOutputLatency[29] = {0};
    sprintf(
        _comOutputLatency,
        comOutputLatency,
        static_cast<uint16_t>(_triggerDelay),
        static_cast<uint16_t>(_resetDelay)
    );

    return setConfig(_comOutputLatency);
}

bool DFR_Radar::checkPresence() const {
    bool presence;
    readPresence(presence);
    return presence;
}

bool DFR_Radar::readPresence(bool &presence) const {
    char packet[packetLength] = {0};

    // Factory default settings have $JYBSS messages sent once per second,
    // but we won't want to wait; this will prompt for status immediately
    serialWrite(comGetOutput);

    /**
     * Get the response immediately after sending the command.
     *
     * If command echoing is enabled, there should be three lines:
     *   1. the "getOutput 1" echoed back
     *   2. a "Done" status
     *   3. the "leapMMW:/>" response followed by the $JYBSS data we want
     *
     * If command echoing is disabled, there should be two lines:
     *   1. a "Done" status
     *   2. the $JYBSS data we want
     *
     * Factory default is command echoing on (might change this in `begin()`)
     */
    size_t length = readLines(packet, 3);

    if (!length)
        return false;

    constexpr size_t expectedLength = 16;
    char data[expectedLength] = {0};
    uint8_t offset = 0;
    bool startCharacterFound = false, endCharacterFound = false;

    /**
     * Parse through the packet until we find a "$", and
     * then start capturing characters until we find a "*"
     *
     * We're expecting to get something like: $JYBSS,1, , , *
     */
    for (uint8_t i = 0; i < length; i++) {
        const char c = packet[i];

        if (c == '$')
            startCharacterFound = true;

        if (!startCharacterFound)
            continue;

        if (c == '*')
            endCharacterFound = true;

        data[offset++] = c;

        if (endCharacterFound || offset == expectedLength)
            break;
    }

    if (!startCharacterFound || !endCharacterFound) {
#ifdef DEBUG
        Serial.print("Error: Invalid data ");
        Serial.println(packet);
#endif
        return false;
    }

    presence = (data[7] == '1');
    return true;
}

bool DFR_Radar::setLockout(const float time) {
    if (time < 0.1 || time > 255)
        return false;

    char _comSetInhibit[19] = {0};

#ifdef __AVR__
#ifdef _STDLIB_H_
    char _time[8] = {0};
    dtostrf(time, 3, 3, _time);

    sprintf(_comSetInhibit, comSetInhibit, _time);
#else
    sprintf(_comSetInhibit, comSetInhibit, (uint8_t) time);
#endif
#else
    sprintf(_comSetInhibit, comSetInhibit, time);
#endif

    return setConfig(_comSetInhibit);
}

bool DFR_Radar::getLockout(float &time) {
    char _comGetInhibit[1][10] = {{0}};
    if (!getConfig<1, 10>(comGetInhibit, _comGetInhibit)) {
        if (debugSerial)
            Serial.println("Error getting inhibit");
        return false;
    }

    time = strtof(_comGetInhibit[0], nullptr);
    return true;
}

bool DFR_Radar::setTriggerLevel(const uint8_t ioPin, const uint8_t triggerLevel) {
    if (triggerLevel != HIGH || triggerLevel != LOW)
        return false;

    char _comSetGpioMode[16] = {0};
    sprintf(_comSetGpioMode, comSetGpioMode, ioPin, triggerLevel);

    return setConfig(_comSetGpioMode);
}

bool DFR_Radar::setTriggerLevel(const uint8_t triggerLevel) {
    return setTriggerLevel(2, triggerLevel);
}

bool DFR_Radar::getTriggerLevel(const uint8_t ioPin, uint8_t &triggerLevel) {
    char _comGetGpioModeCommand[14] = {0};
    sprintf(_comGetGpioModeCommand, comGetGpioMode, ioPin);

    char _comGetGpioMode[2][2] = {{0}};
    if (!getConfig<2, 2>(_comGetGpioModeCommand, _comGetGpioMode)) {
        if (debugSerial)
            Serial.println("Error getting gpio mode");
        return false;
    }

    triggerLevel = strtol(_comGetGpioMode[1], nullptr, 10);
    return true;
}

bool DFR_Radar::getTriggerLevel(uint8_t &triggerLevel) {
    return getTriggerLevel(2, triggerLevel);
}

bool DFR_Radar::setUartOutput(const uint8_t messageType, const bool enable, const bool push, const float period) {
    if (messageType < 1 || messageType > 3)
        return false;

    char _comSetUartOutput[30] = {0};
    sprintf(_comSetUartOutput, comSetUartOutputFull, messageType, enable, push, period);

    return setConfig(_comSetUartOutput);
}

bool DFR_Radar::configureUartDetectionOutput(const bool enable, const bool push, const float period) {
    return setUartOutput(1, enable, push, period);
}

bool DFR_Radar::configureUartPointCloudOutput(const bool enable, const bool push, const float period) {
    return setUartOutput(2, enable, push, period);
}

bool DFR_Radar::getUartOutput(const uint8_t messageType, bool &enable, bool &onChange, float &period) {
    if (messageType < 1 || messageType > 3)
        return false;

    char _comGetUartOutput[4][9] = {{0}, {0}, {0}, {0}};

    char _comGetUartOutputCommand[16] = {0};
    sprintf(_comGetUartOutputCommand, comGetUartOutput, messageType);

    if (!getConfig<4, 9>(_comGetUartOutputCommand, _comGetUartOutput)) {
        if (debugSerial)
            Serial.println("Error getting uart output");
        return false;
    }

    enable = strtol(_comGetUartOutput[1], nullptr, 10) == 1;
    onChange = strtol(_comGetUartOutput[2], nullptr, 10) == 1;
    period = strtof(_comGetUartOutput[3], nullptr);
    return true;
}

bool DFR_Radar::getUartDetectionOutput(bool &enable, bool &onChange, float &period) {
    return getUartOutput(1, enable, onChange, period);
}

bool DFR_Radar::getUartPointCloudOutput(bool &enable, bool &onChange, float &period) {
    return getUartOutput(2, enable, onChange, period);
}

bool DFR_Radar::setEcho(const bool enable) {
    char _comSetEcho[14] = {0};
    sprintf(_comSetEcho, comSetEcho, enable);
    return setConfig(_comSetEcho);
}

bool DFR_Radar::getEcho(bool &enable) {
    char _comGetEcho[1][2] = {{0}};
    if (!getConfig<1, 2>(comGetEcho, _comGetEcho)) {
        if (debugSerial)
            Serial.println("Error getting echo");
        return false;
    }

    enable = strtol(_comGetEcho[0], nullptr, 10) == 1;
    return true;
}

bool DFR_Radar::start() {
    if (!stopped)
        return true;

    if (sendCommand(comStart, comFailStarted)) {
        stopped = false;
        return true;
    }

    return false;
}

bool DFR_Radar::stop() {
    if (stopped)
        return true;

    if (sendCommand(comStop, comFailStopped)) {
        stopped = true;
        return true;
    }

    return false;
}

void DFR_Radar::reboot() const {
    sendCommand(comResetSystem);
}

bool DFR_Radar::disableLED() {
    return configureLED(true);
}

bool DFR_Radar::enableLED() {
    return configureLED(false);
}

bool DFR_Radar::configureLED(const bool disabled) {
    char _comSetLedMode[15] = {0};
    sprintf(_comSetLedMode, comSetLedMode, disabled);

    return setConfig(_comSetLedMode);
}

bool DFR_Radar::getLEDMode(bool &disabled) {
    char _comGetLedMode[2][2] = {{0}};
    if (!getConfig<2, 2>(comGetLedMode, _comGetLedMode)) {
        if (debugSerial)
            Serial.println("Error getting led mode");
        return false;
    }

    disabled = strtol(_comGetLedMode[1], nullptr, 10) == 1;
    return true;
}

bool DFR_Radar::configBegin() {
    if (multiConfig)
        return true;

    if (!stop())
        return false;

    multiConfig = true;

    return true;
}

bool DFR_Radar::configEnd() {
    if (!multiConfig)
        return false;

    multiConfig = false;

    if (!saveConfig())
        return false;

    if (!start())
        return false;

    return true;
}

bool DFR_Radar::factoryReset() {
    // if( !stop() )
    //   return false;
    stop();

    const bool success = sendCommand(comFactoryReset);
    delay(2000);

    return success;
}

bool DFR_Radar::getHWVersion(char *version) {
    char _comGetHWVersion[1][32] = {{0}};
    if (!getConfig<1, 32>(comGetHWV, _comGetHWVersion, "")) {
        if (debugSerial)
            Serial.println("Error getting HW version");
        return false;
    }

    strcpy(version, _comGetHWVersion[0]);
    return true;
}

bool DFR_Radar::getSWVersion(char *version) {
    char _comGetSWVersion[1][32] = {{0}};
    if (!getConfig<1, 32>(comGetSWV, _comGetSWVersion, "")) {
        if (debugSerial)
            Serial.println("Error getting SW version");
        return false;
    }

    strcpy(version, _comGetSWVersion[0]);
    return true;
}

size_t DFR_Radar::readLines(char *buffer, const size_t lineCount) const {
    const unsigned long timeLimit = millis() + readPacketTimeout;
    size_t offset = 0, linesLeft = lineCount;

    while (linesLeft && millis() < timeLimit) {
        if (sensorUART->available() <= 0)
            continue;

        char c = sensorUART->read();

        if (c == '\r')
            continue;

        buffer[offset++] = c;

        if (c == '\n')
            linesLeft--;
    }

    return strlen(buffer);
}

bool DFR_Radar::setConfig(const char *command) {
    if (multiConfig) {
        return sendCommand(command);
    }
    // if( !stop() )
    //   return false;
    stop();

    if (!sendCommand(command))
        return false;

    const bool saved = saveConfig();

    if (!start())
        return false;

    return saved;
}

bool DFR_Radar::saveConfig() const {
    return sendCommand(comSaveCfg);
}

size_t DFR_Radar::serialWrite(const char *command) const {
    static char _command[64] = {0};
    const size_t commandLength = strlen(command) + 3;

    // Ensure the re-usable buffer is empty
    memset(&_command[0], 0, sizeof(_command));

    // Make a properly-terminated copy of the command
    snprintf(_command, commandLength, "%s\r\n", command);

    // Make sure we have exactly enough time
    sensorUART->setTimeout(comTimeout);

    // Clear the receive buffer
    while (sensorUART->available())
        sensorUART->read();

    if (debugSerial)
        Serial.printf("Sending command: '%s'\n", command);

    // Send the command...
    sensorUART->write(_command);
    sensorUART->flush();

    return commandLength;
}

bool DFR_Radar::sendCommand(const char *command) const {
    return sendCommand(command, nullptr);
}

bool DFR_Radar::sendCommand(const char *command, const char *acceptableResponse) const {
    bool errorAcceptable = false;
    char lineBuffer[64] = {0};
    const unsigned long timeout = millis() + comTimeout;

    static const size_t successLength = strlen(comResponseSuccess);
    static const size_t failLength = strlen(comResponseFail);
    static const size_t minResponseLength = min(successLength, failLength);

    const size_t commandLength = strlen(command);
    size_t minLength = min(commandLength, minResponseLength);
    const size_t acceptableLength = acceptableResponse == nullptr ? minLength : strlen(acceptableResponse);

    if (acceptableResponse != nullptr)
        minLength = min(acceptableLength, minLength);

    // Send the command...
    serialWrite(command);

    // ...then wait for a response
    while (millis() < timeout) {
        if (sensorUART->available() <= 0)
            continue;

        // Ensure the buffer is empty
        memset(&lineBuffer[0], 0, sizeof(lineBuffer));

        // Read a whole line
        size_t responseLength = sensorUART->readBytesUntil('\n', lineBuffer, sizeof(lineBuffer));

        // The sensor is supposed to terminate lines with <CRLF>, and we stopped at <LF>,
        // so the last character in the line buffer should be a <CR>.  If so, swap it out
        // for a null terminator
        const uint8_t lastCharacter = strlen(lineBuffer) - 1;
        if (lineBuffer[lastCharacter] == '\r')
            lineBuffer[lastCharacter] = '\0';

        // Update the length and ensure the line we've received is properly terminated
        responseLength = strlen(lineBuffer);
        lineBuffer[responseLength] = '\0';

        if (debugSerial)
            Serial.printf("Read line: '%s'\n", lineBuffer);

        // We got something shorter than anything we're expecting, so try again
        if (responseLength < minLength)
            continue;

        // Check if that line is the command prompt
        if (strncmp(comPrompt, lineBuffer, strlen(comPrompt)) == 0)
            continue;

        // ...or if that line is an echo of the original command
        if (strncmp(command, lineBuffer, commandLength) == 0)
            continue;

        // ...or if that line contains an expected response
        if (acceptableResponse != nullptr && strncmp(acceptableResponse, lineBuffer, acceptableLength) == 0) {
            errorAcceptable = true;

            // Even though we got what we want, we can't return yet; we need to go one more round
            // so that we get the "Done" or "Error" that follows out of the serial buffer.
            continue;
        }

        // ...or if that line says "Done"
        if (strncmp(comResponseSuccess, lineBuffer, successLength) == 0)
            return true;

        // ...or if that line says "Error"
        if (strncmp(comResponseFail, lineBuffer, failLength) == 0)
            return errorAcceptable;

        // ...we got nothing we expected, so try again
    }

    // We've timed out
    return errorAcceptable;
}

template<size_t NParams, size_t MaxParamLength>
bool DFR_Radar::getConfig(
    const char *command,
    char outParams[NParams][MaxParamLength],
    const char *responsePrefix
) {
    char lineBuffer[64] = {0};
    const unsigned long timeout = millis() + comTimeout;

    static const size_t successLength = strlen(comResponseSuccess);
    static const size_t failLength = strlen(comResponseFail);
    static const size_t minResponseLength = min(successLength, failLength);

    const size_t commandLength = strlen(command);
    size_t minLength = min(commandLength, minResponseLength);

    const size_t responsePrefixLength = responsePrefix == nullptr ? 0 : strlen(responsePrefix);

    if (responsePrefix != nullptr)
        minLength = min(responsePrefixLength, minResponseLength);

    // Send the command...
    serialWrite(command);

    uint8_t paramIndex = 0;

    // ...then wait for a response
    while (millis() < timeout) {
        if (sensorUART->available() <= 0)
            continue;

        // Ensure the buffer is empty
        memset(&lineBuffer[0], 0, sizeof(lineBuffer));

        // Read a whole line
        size_t responseLength = sensorUART->readBytesUntil('\n', lineBuffer, sizeof(lineBuffer));

        // The sensor is supposed to terminate lines with <CRLF>, and we stopped at <LF>,
        // so the last character in the line buffer should be a <CR>.  If so, swap it out
        // for a null terminator
        const uint8_t lastCharacter = strlen(lineBuffer) - 1;
        if (lineBuffer[lastCharacter] == '\r')
            lineBuffer[lastCharacter] = '\0';

        // Update the length and ensure the line we've received is properly terminated
        responseLength = strlen(lineBuffer);
        lineBuffer[responseLength] = '\0';

        if (debugSerial)
            Serial.printf("Read line: '%s'\n", lineBuffer);

        // We got something shorter than anything we're expecting, so try again
        if (responseLength < minLength)
            continue;

        // Check if that line is the command prompt
        if (strncmp(comPrompt, lineBuffer, strlen(comPrompt)) == 0)
            continue;

        // ...or if that line is an echo of the original command
        if (strncmp(command, lineBuffer, commandLength) == 0)
            continue;

        // ...or if that line says "Done"
        if (strncmp(comResponseSuccess, lineBuffer, successLength) == 0)
            continue;

        // ...or if that line says "Error"
        if (strncmp(comResponseFail, lineBuffer, failLength) == 0)
            continue;

        if (responsePrefix != nullptr && strncmp(responsePrefix, lineBuffer, responsePrefixLength) == 0) {
            for (size_t i = responsePrefixLength; i < responseLength && paramIndex < NParams; i++) {
                if (isWhitespace(lineBuffer[i]))
                    continue;
                size_t paramCharIndex = 0;
                do {
                    outParams[paramIndex][paramCharIndex++] = lineBuffer[i++];
                } while (i < responseLength && paramCharIndex < MaxParamLength && !isWhitespace(lineBuffer[i]));
                outParams[paramIndex++][paramCharIndex] = '\0';
            }
            if (debugSerial) {
                Serial.printf("getConfig: Got %d/%d params\n", paramIndex, NParams);
                for (auto i = 0; i < NParams; i++) {
                    Serial.printf("getConfig:       Param %d: '%s'\n", i, outParams[i]);
                }
            }
            return paramIndex == NParams;
        }
    }
    return false;
}

template<size_t NParams, size_t MaxParamLength>
bool DFR_Radar::getConfig(const char *command, char outParams[NParams][MaxParamLength]) {
    return getConfig<NParams, MaxParamLength>(command, outParams, comResponse);
}
