#include "lps331.h"
#include <math.h>
#define LPS331AP_ADDRESS_SA0_LOW  0b1011100
#define LPS331AP_ADDRESS_SA0_HIGH 0b1011101

#define LPS331_REF_P_XL       0x08
#define LPS331_REF_P_L        0x09
#define LPS331_REF_P_H        0x0A

#define LPS331_WHO_AM_I       0x0F

#define LPS331_RES_CONF       0x10

#define LPS331_CTRL_REG1      0x20
#define LPS331_CTRL_REG2      0x21
#define LPS331_CTRL_REG3      0x22
#define LPS331_INTERRUPT_CFG  0x23
#define LPS331_INT_SOURCE     0x24
#define LPS331_THS_P_L        0x25
#define LPS331_THS_P_H        0x26
#define LPS331_STATUS_REG     0x27

#define LPS331_PRESS_OUT_XL   0x28
#define LPS331_PRESS_OUT_L    0x29
#define LPS331_PRESS_OUT_H    0x2A

#define LPS331_TEMP_OUT_L     0x2B
#define LPS331_TEMP_OUT_H     0x2C

#define LPS331_AMP_CTRL       0x30

#define LPS331_DELTA_PRESS_XL 0x3C
#define LPS331_DELTA_PRESS_L  0x3D
#define LPS331_DELTA_PRESS_H  0x3E

LPS331::LPS331(const char *i2cDeviceName) : i2c(i2cDeviceName)
{

    ok = detectAddress();
    if( ok ) {
        writeReg( 0x20, 0b10000100 );
        writeReg( 0x21, 0b1);
    }
}

bool LPS331::detectAddress()
{
    // try each possible address and stop if reading WHO_AM_I returns the expected response
    address = LPS331AP_ADDRESS_SA0_LOW;
    if (testWhoAmI()) return true;
    address = LPS331AP_ADDRESS_SA0_HIGH;
    if (testWhoAmI()) return true;

    return false;
}

bool LPS331::testWhoAmI()
{
    i2c.addressSet(address);
    if (i2c.tryReadByte(LPS331_WHO_AM_I) == 0xBB)
    {
        // Detected
        return(true);
    }
    return(false);
}


void LPS331::writeReg(uint8_t reg, uint8_t value)
{
    i2c.writeByte(reg, value);
}

uint8_t LPS331::readReg(uint8_t reg)
{
    return i2c.readByte(reg);
}

// reads pressure in millibars (mbar)/hectopascals (hPa)
float LPS331::readPressureMillibars(void)
{
    return (float)readPressureRaw() / 4096.0;
}


// reads pressure and returns raw 24-bit sensor output
long LPS331::readPressureRaw(void)
{
    uint8_t data[4];
    data[0] = readReg(0x28) ;
    data[1] = readReg(0x29) ;
    data[2] = readReg(0x2A) ;
    data[4] = 0 ;

    long v = data[2]*65536 + data[1]*256 + data[0];

    return(v);
}

// reads temperature in degrees C
float LPS331::readTemperatureC(void)
{
    if( !ok ) {
        return(0);
    }
    return 42.5 + ((float)readTemperatureRaw()) / 480.0;
}


// reads temperature and returns raw 16-bit sensor output
int LPS331::readTemperatureRaw(void)
{
    uint8_t data[2];
    data[0] = readReg(LPS331_TEMP_OUT_L) ;
    data[1] = readReg(LPS331_TEMP_OUT_H) ;
    int16_t *v = (int16_t *)data ;
    return(*v);
}


// converts pressure in mbar to altitude in meters, using 1976 US
// Standard Atmosphere model (note that this formula only applies to a
// height of 11 km, or about 36000 ft)
//  If altimeter setting (QNH, barometric pressure adjusted to sea
//  level) is given, this function returns an indicated altitude
//  compensated for actual regional pressure; otherwise, it returns
//  the pressure altitude above the standard pressure level of 1013.25
//  mbar or 29.9213 inHg
float LPS331::pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar)
{
    return (1 - pow(pressure_mbar / altimeter_setting_mbar, 0.190263)) * 44330.8;
}
