#ifndef SHT3X_SENSOR_H
#define SHT3X_SENSOR_H

#include <Arduino.h>
#include <SensirionI2cSht3x.h>
#include <Wire.h>

class SHT3xSensor {
public:
    SHT3xSensor(uint8_t address = SHT30_I2C_ADDR_44);
    bool begin();
    bool readMeasurement(float &temperature, float &humidity);
    String getLastError() const;

private:
    SensirionI2cSht3x sensor;
    uint8_t address;
    char errorMessage[64];
    int16_t error;

    void resetSensor();
    bool handleError(const String &context);
};

#endif // SHT3X_SENSOR_H