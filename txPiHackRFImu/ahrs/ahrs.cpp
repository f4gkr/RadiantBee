#include "ahrs.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <system_error>
#include <QDebug>
AHRS::AHRS(QObject *parent) :
    QThread(parent)
{
    imu = new MinIMU9( (const char *)"/dev/i2c-1");
    fuse = &fuse_default ;
    roll = pitch = yaw = 0 ;
    altitude = 0 ;
}

void AHRS::run() {
    int cycles ;
    imu->loadCalibration();
    imu->enable();
    imu->measureOffsets();

    LPS331 baro((const char *)"/dev/i2c-1") ;
    if( baro.isReady() ) {
        qDebug() << "Temperature:" << baro.readTemperatureC();
        qDebug() << " Pression" << baro.readPressureMillibars();
    }
    float temperature = baro.readTemperatureC();
    float pressure = baro.readPressureMillibars() ;
    altitude = baro.pressureToAltitudeMeters(pressure) ;
    qDebug() << "temperature:" << temperature << "pressure:" << pressure << " alt=" << altitude ;

    // The quaternion that can convert a vector in body coordinates
    // to ground coordinates when it its changed to a matrix.
    quaternion rotation = quaternion::Identity();
    cycles = 0 ;
    int start = millis(); // truncate 64-bit return value
    while(1)
    {
        int last_start = start;
        start = millis();
        float dt = (start-last_start)/1000.0;
        if (dt < 0){ throw std::runtime_error("Time went backwards."); }

        vector angular_velocity = imu->readGyro();
        vector acceleration = imu->readAcc();
        vector magnetic_field = imu->readMag();

        fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);
        vector v = (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0) * (180 / M_PI));

        roll = roundf(v(0));
        pitch = roundf(v(1));
        yaw = roundf(v(2));

        cycles++ ;
        if( cycles > 50 ) {
            // measure baro every second
            baro.reset();
            temperature = baro.readTemperatureC();
            pressure = baro.readPressureMillibars() ;
            altitude = baro.pressureToAltitudeMeters(pressure) ;
            qDebug() << "temperature:" << temperature << "pressure:" << pressure << " alt=" << altitude ;
            cycles = 0 ;
        }

        // Ensure that each iteration of the loop takes at least 20 ms.
        while(millis() - start < 20)
        {
            usleep(1000);
        }

    }
}

int AHRS::millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}


//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
matrix AHRS::rotationFromCompass(const vector& acceleration, const vector& magnetic_field)
{
    vector down = -acceleration;     // usually true
    vector east = down.cross(magnetic_field); // actually it's magnetic east
    vector north = east.cross(down);

    east.normalize();
    north.normalize();
    down.normalize();

    matrix r;
    r.row(0) = north;
    r.row(1) = east;
    r.row(2) = down;
    return r;
}

void AHRS::fuse_compass_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    // Implicit conversion of rotation matrix to quaternion.
    rotation = rotationFromCompass(acceleration, magnetic_field);
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void AHRS::rotate(quaternion& rotation, const vector& w, float dt)
{
    // Multiply by first order approximation of the
    // quaternion representing this rotation.
    rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
    rotation.normalize();
}

void AHRS::fuse_gyro_only(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    rotate(rotation, angular_velocity, dt);
}

void AHRS::fuse_default(quaternion& rotation, float dt, const vector& angular_velocity,
  const vector& acceleration, const vector& magnetic_field)
{
    vector correction = vector(0, 0, 0);

    if (abs(acceleration.norm() - 1) <= 0.3)
    {
        // The magnetidude of acceleration is close to 1 g, so
        // it might be pointing up and we can do drift correction.

        const float correction_strength = 1;

        matrix rotationCompass = rotationFromCompass(acceleration, magnetic_field);
        matrix rotationMatrix = rotation.toRotationMatrix();

        correction = (
            rotationCompass.row(0).cross(rotationMatrix.row(0)) +
            rotationCompass.row(1).cross(rotationMatrix.row(1)) +
            rotationCompass.row(2).cross(rotationMatrix.row(2))
          ) * correction_strength;

    }

    rotate(rotation, angular_velocity + correction, dt);
}
