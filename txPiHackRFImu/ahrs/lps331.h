#ifndef LPS331_H
#define LPS331_H

#include "I2CBus.h"
#include <stdint.h>

class LPS331
{
public:
    LPS331(const char * i2cDeviceName);

    float pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar = 1013.25);

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    long readPressureRaw(void);
    int readTemperatureRaw(void);
    float readTemperatureC(void);
    float readPressureMillibars(void);

    bool isReady() { return( ok ); }

private:
    uint8_t address;
    bool ok ;
    bool detectAddress();
    bool testWhoAmI();
    I2CBus i2c;
};

#endif // LPS331_H
