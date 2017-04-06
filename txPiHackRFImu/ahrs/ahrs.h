#ifndef AHRS_H
#define AHRS_H

#include <QObject>
#include <QThread>
#include "vector.h"
#include "MinIMU9.h"
#include "version.h"
#include "lps331.h"
typedef void fuse_function(quaternion& rotation, float dt, const vector& angular_velocity,
                  const vector& acceleration, const vector& magnetic_field);


class AHRS : public QThread
{
    Q_OBJECT
public:
    explicit AHRS(QObject *parent = 0);

    int getRoll() { return( roll) ; }
    int getPitch() { return( pitch) ; }
    int getYaw() { return( yaw) ; }

    float getAltitude() { return( altitude ); }
signals:

public slots:
private:
    IMU *imu ;
    float altitude;
    fuse_function * fuse;
    int roll,pitch,yaw;
    static int millis();

    static matrix rotationFromCompass(const vector& acceleration, const vector& magnetic_field);

    static void fuse_compass_only(quaternion& rotation, float dt, const vector& angular_velocity,
      const vector& acceleration, const vector& magnetic_field);
    static void fuse_gyro_only(quaternion& rotation, float dt, const vector& angular_velocity,
      const vector& acceleration, const vector& magnetic_field);
    static void fuse_default(quaternion& rotation, float dt, const vector& angular_velocity,
      const vector& acceleration, const vector& magnetic_field);

    static void rotate(quaternion& rotation, const vector& w, float dt);

    void run();
};

#endif // AHRS_H
