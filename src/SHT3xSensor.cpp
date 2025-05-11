#include "SHT3xSensor.h"

SHT3xSensor::SHT3xSensor(uint8_t address) : address(address) {}

bool SHT3xSensor::begin() {
    Wire.begin();
    sensor.begin(Wire, address);

    resetSensor();
    
    uint16_t aStatusRegister = 0u;
    error = sensor.readStatusRegister(aStatusRegister);
    if (error != NO_ERROR) {
        return handleError("readStatusRegister");
    }

    error = sensor.startPeriodicMeasurement(REPEATABILITY_MEDIUM, MPS_ONE_PER_SECOND);
    return !handleError("startPeriodicMeasurement");
}

bool SHT3xSensor::readMeasurement(float &temperature, float &humidity) {
    error = sensor.blockingReadMeasurement(temperature, humidity);
    return !handleError("blockingReadMeasurement");
}

String SHT3xSensor::getLastError() const {
    return String(errorMessage);
}

void SHT3xSensor::resetSensor() {
    sensor.stopMeasurement();
    delay(1);
    sensor.softReset();
    delay(100);
}

bool SHT3xSensor::handleError(const String &context) {
    if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.print("Error trying to execute ");
        Serial.print(context);
        Serial.print(": ");
        Serial.println(errorMessage);
        return true;
    }
    return false;
}